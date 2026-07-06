"""Minimal point-cloud registration for connector pose refinement.

STL loading, area-weighted surface sampling, and point-to-point ICP with an
SVD (Umeyama) solve. numpy + scipy.spatial only — deliberately no Open3D/PCL
dependency. Designed for the eye-in-hand case: a good initial pose comes from
the ArUco marker prior, so ICP only refines a few mm / a few degrees.

Convention: the template (model) is in the connector frame; ICP returns
T_camera<-connector (4x4) such that  x_cam ~= T @ x_model.
"""

import struct

import numpy as np
from scipy.spatial import cKDTree


# ---------------------------------------------------------------- STL loading

def load_stl(path):
    """Load an STL file (binary or ASCII). Returns (V, F): vertices (N,3)
    and triangle vertex indices (M,3)."""
    with open(path, 'rb') as f:
        data = f.read()
    if _is_binary_stl(data):
        return _parse_binary_stl(data)
    return _parse_ascii_stl(data.decode('ascii', errors='replace'))


def _is_binary_stl(data):
    if len(data) < 84:
        return False
    n_tri = struct.unpack_from('<I', data, 80)[0]
    return len(data) == 84 + n_tri * 50


def _parse_binary_stl(data):
    n_tri = struct.unpack_from('<I', data, 80)[0]
    raw = np.frombuffer(data, dtype=np.uint8, count=n_tri * 50, offset=84)
    tris = raw.reshape(n_tri, 50)[:, :48].copy().view('<f4').reshape(n_tri, 12)
    verts = tris[:, 3:12].reshape(n_tri * 3, 3).astype(np.float64)
    return _dedupe(verts)


def _parse_ascii_stl(text):
    verts = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == 'vertex':
            verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    verts = np.asarray(verts, dtype=np.float64)
    if len(verts) == 0 or len(verts) % 3 != 0:
        raise ValueError('Malformed ASCII STL')
    return _dedupe(verts)


def _dedupe(verts):
    """Merge duplicated vertices; return (V, F)."""
    rounded = np.round(verts, 9)
    uniq, inverse = np.unique(rounded, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    return uniq, faces


# ------------------------------------------------------------------- sampling

def sample_mesh(V, F, n_points, seed=0, return_normals=False):
    """Area-weighted uniform sampling of points on the mesh surface."""
    rng = np.random.default_rng(seed)
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cross = np.cross(b - a, c - a)
    areas = 0.5 * np.linalg.norm(cross, axis=1)
    if areas.sum() <= 0:
        raise ValueError('Degenerate mesh (zero surface area)')
    tri = rng.choice(len(F), size=n_points, p=areas / areas.sum())
    u = rng.random(n_points)
    v = rng.random(n_points)
    flip = u + v > 1.0
    u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
    points = (a[tri] * (1.0 - u - v)[:, None]
              + b[tri] * u[:, None]
              + c[tri] * v[:, None])
    if not return_normals:
        return points
    normals = cross[tri] / np.maximum(
        np.linalg.norm(cross[tri], axis=1, keepdims=True), 1e-12)
    return points, normals


def voxel_downsample(points, voxel, normals=None):
    """Mean point (and mean unit normal) per occupied voxel."""
    keys = np.floor(points / voxel).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inverse, points)
    mean_pts = sums / counts[:, None]
    if normals is None:
        return mean_pts
    n_sums = np.zeros((len(counts), 3))
    np.add.at(n_sums, inverse, normals)
    mean_n = n_sums / np.maximum(np.linalg.norm(n_sums, axis=1, keepdims=True), 1e-12)
    return mean_pts, mean_n


# ------------------------------------------------------------------------ ICP

def _umeyama(src, dst):
    """Rigid transform (R, t) minimizing ||R @ src + t - dst||."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    S = np.eye(3)
    if np.linalg.det(Vt.T @ U.T) < 0:
        S[2, 2] = -1.0
    R = Vt.T @ S @ U.T
    t = mu_d - R @ mu_s
    return R, t


def _rotvec_to_matrix(w):
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3)
    k = np.asarray(w) / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * K @ K


def _solve_point_to_plane(y, m, n):
    """Small rigid correction (R, t) minimizing sum ((R y + t - m) . n)^2,
    linearized about identity."""
    a = np.hstack([np.cross(y, n), n])              # (N,6)
    b = -np.einsum('ij,ij->i', y - m, n)            # (N,)
    x, *_ = np.linalg.lstsq(a, b, rcond=None)
    return _rotvec_to_matrix(x[:3]), x[3:]


def icp(scene, template, T_init, template_normals=None, max_corr_dist=0.008,
        max_iter=30, tol_trans=1e-5, tol_rot=1e-4, trim_fraction=1.0):
    """Refine T_camera<-connector so the template matches the scene.

    scene: (N,3) points in the camera frame.
    template: (M,3) points in the connector/model frame.
    T_init: 4x4 initial guess of T_camera<-connector (from the marker prior).
    template_normals: (M,3) unit normals; when given, point-to-plane
    minimization is used — much better tilt convergence on the flat,
    partially visible (top-only) surfaces a depth camera sees. Without
    normals, falls back to point-to-point (Umeyama).
    trim_fraction: only the best fraction of in-range correspondences drive
    the solve. Default 1.0 (no trimming): on partial top-only views trimming
    preferentially discards the minority side/ridge points that carry tilt
    information and measurably increases tilt bias. Lower it only for scenes
    with substantial foreign-object clutter.

    Returns (T, rms, inlier_fraction). rms is over inlier correspondences.
    """
    scene = np.asarray(scene, dtype=np.float64)
    tree = cKDTree(template)
    T = np.array(T_init, dtype=np.float64, copy=True)

    rms = np.inf
    inlier_frac = 0.0
    for _ in range(max_iter):
        # Scene expressed in the model frame under the current estimate
        R_inv = T[:3, :3].T
        y = (scene - T[:3, 3]) @ R_inv.T

        dists, idx = tree.query(y, k=1)
        inliers = dists < max_corr_dist
        inlier_frac = float(inliers.mean())
        if inliers.sum() < 10:
            return T, np.inf, inlier_frac
        rms = float(np.sqrt(np.mean(dists[inliers] ** 2)))

        # Trim: keep the best trim_fraction of the in-range correspondences
        in_idx = np.flatnonzero(inliers)
        keep = in_idx[np.argsort(dists[in_idx])[:max(10, int(len(in_idx) * trim_fraction))]]
        if template_normals is not None:
            R_a, t_a = _solve_point_to_plane(
                y[keep], template[idx[keep]], template_normals[idx[keep]])
        else:
            R_a, t_a = _umeyama(y[keep], template[idx[keep]])
        # y' = R_a y + t_a aligns scene-in-model with the template, so the
        # pose correction is T <- T @ inv(A)
        A_inv = np.eye(4)
        A_inv[:3, :3] = R_a.T
        A_inv[:3, 3] = -R_a.T @ t_a
        T = T @ A_inv

        d_trans = float(np.linalg.norm(t_a))
        d_rot = float(np.arccos(np.clip((np.trace(R_a) - 1.0) / 2.0, -1.0, 1.0)))
        if d_trans < tol_trans and d_rot < tol_rot:
            break

    return T, rms, inlier_frac


def depth_to_points(depth_m, fx, fy, cx, cy, stride=2):
    """Back-project a depth image (meters, invalid = 0/NaN) to (N,3) points
    in the camera optical frame."""
    h, w = depth_m.shape
    us, vs = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
    z = depth_m[vs, us]
    valid = np.isfinite(z) & (z > 0.0)
    z = z[valid]
    us = us[valid]
    vs = vs[valid]
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    return np.column_stack([x, y, z])


def crop_points(points, center, radius):
    """Points within a sphere around the expected connector location."""
    mask = np.linalg.norm(points - np.asarray(center), axis=1) < radius
    return points[mask]
