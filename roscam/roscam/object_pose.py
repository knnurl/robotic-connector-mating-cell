#!/usr/bin/env python3
"""Pose of a known part from depth, against its mesh (PERCEPTION_PLAN Phase 2).

ROS-free. A part file (tools/fr3/parts/*.yaml) gives the mesh (a box, or
an STL), the object frame (the mesh's own: origin at the top-face centre or
the mate point, Z out of that face), the symmetry and T_marker_object.
Nothing below is specific to one part.

Two kinds of depth evidence, both from the one mesh, solved together
(Gauss-Newton, robust weights):

  surface  point-to-plane distance of each segmented depth point to the
           mesh faces the camera can see. It pins height and tilt of any
           face, and everything else where the part shows 3-D shape.
  outline  the mesh's silhouette at the current pose against the outline of
           the segmented depth region, in the image. A flat face seen head
           on (the cube's top from TRACK or GRIP) can slide and spin in its
           own plane without changing the surface fit; its outline is the
           only depth evidence for those three directions.

weak_dof then names the directions the data still cannot pin, so a pose
that is not determined never passes as one.
"""

import pathlib
import time

import cv2
import numpy as np
import yaml
from scipy.ndimage import uniform_filter
from scipy.spatial import cKDTree

from roscam.icp import load_stl, sample_mesh
from roscam.plane_normal import fit_plane_robust

DOF_NAMES = ('rx', 'ry', 'rz', 'tx', 'ty', 'tz')


# ------------------------------------------------------------------- the part

def box_mesh(size):
    """(V, F) of a box of size (x, y, z) metres: origin at the top-face
    centre, Z out of the top face, outward-facing triangles."""
    hx, hy, h = float(size[0]) / 2.0, float(size[1]) / 2.0, float(size[2])
    V = np.array([[-hx, -hy, 0.0], [hx, -hy, 0.0], [-hx, hy, 0.0], [hx, hy, 0.0],
                  [-hx, -hy, -h], [hx, -hy, -h], [-hx, hy, -h], [hx, hy, -h]])
    F = np.array([[0, 1, 3], [0, 3, 2],          # top, +z
                  [4, 6, 7], [4, 7, 5],          # bottom
                  [1, 5, 7], [1, 7, 3],          # +x
                  [0, 2, 6], [0, 6, 4],          # -x
                  [2, 7, 6], [2, 3, 7],          # +y
                  [0, 4, 5], [0, 5, 1]])         # -y
    return V, F


def rpy_matrix(rpy_deg):
    r, p, y = np.radians(np.asarray(rpy_deg, dtype=float))
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_part(path):
    """Part file -> dict(name, V, F (metres, object frame), sym_order,
    T_marker_object 4x4)."""
    path = pathlib.Path(path)
    doc = yaml.safe_load(path.read_text())
    mesh = doc['mesh']
    if 'box_mm' in mesh:
        V, F = box_mesh(np.asarray(mesh['box_mm'], dtype=float) / 1000.0)
    else:
        V, F = load_stl(str(path.parent / mesh['stl']))
        V = V * float(mesh.get('scale', 0.001))           # STL in mm unless stated
    mo = doc.get('T_marker_object') or {}
    T_mo = np.eye(4)
    T_mo[:3, :3] = rpy_matrix(mo.get('rpy_deg', [0.0, 0.0, 0.0]))
    T_mo[:3, 3] = np.asarray(mo.get('xyz_mm', [0.0, 0.0, 0.0]), dtype=float) / 1000.0
    return {'name': doc.get('name', path.stem), 'V': V, 'F': F,
            'sym_order': int((doc.get('symmetry') or {}).get('order', 1)),
            'T_marker_object': T_mo}


# ------------------------------------------------------------------- helpers

def _skew(v):
    """(N,3) -> (N,3,3) with skew(v) @ w = v x w."""
    z = np.zeros(len(v))
    return np.stack([np.stack([z, -v[:, 2], v[:, 1]], -1),
                     np.stack([v[:, 2], z, -v[:, 0]], -1),
                     np.stack([-v[:, 1], v[:, 0], z], -1)], 1)


def _tukey(r, c):
    """Tukey biweight: residuals beyond c get no weight at all. Segmenting
    by depth sometimes lets an occluder in (a finger whose side face ramps
    down to the part); a loss that only softens outliers still lets them
    tilt the fit."""
    u = np.clip(np.abs(r) / c, 0.0, 1.0)
    return (1.0 - u * u) ** 2


def _quat(R):
    """(x, y, z, w) of a rotation matrix, stable at any angle."""
    tr = np.trace(R)
    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(max(1.0 + R[i, i] - R[j, j] - R[k, k], 1e-12))
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = s / 4
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    q = np.asarray(q)
    return q / np.linalg.norm(q)


def _to_pose7(T):
    return [float(v) for v in T[:3, 3]] + [float(v) for v in _quat(T[:3, :3])]


def pose_error(T_est, T_ref, sym_order=1):
    """(dxyz in the reference object frame (m), tilt (deg), in-plane (deg))
    of T_est against T_ref, the in-plane angle taken modulo the symmetry."""
    D = np.linalg.inv(T_ref) @ T_est
    tilt = np.degrees(np.arccos(np.clip(D[2, 2], -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(D[1, 0], D[0, 0]))
    per = 360.0 / max(1, sym_order)
    yaw = (yaw + per / 2.0) % per - per / 2.0
    return D[:3, 3], float(tilt), float(yaw)


# ----------------------------------------------------------------- estimator

class ObjectPoseEstimator:
    """process(depth, bgr_clean, K, dist, prior) -> (T, valid, quality).

    T is T_camera<-object (4x4, optical frame) or None; prior is the same
    kind of pose (marker o T_marker_object, or the last estimate) and only
    seeds the search. bgr_clean (the camera's own colour frame, never
    drawn on) gives the outline from colour edges (stage B); None: depth only."""

    def __init__(self, part, sample_m=0.0015, edge_step_m=0.0003, max_points=600,
                 search_m=0.010, prior_err_m=0.005, depth_band_m=0.015, support_band_m=0.003,
                 max_incidence_deg=86.0, rim_max_incidence_deg=75.0, outline_offset_px=0.5,
                 max_iter=30, min_points=200, max_rms_m=0.0015, min_inlier_frac=0.8,
                 min_outline_frac=0.6, max_shift_m=0.010, max_shift_deg=10.0,
                 size_ratio=(0.5, 1.6), weak_rel=1e-3, use_outline=True,
                 outline_weight=0.1, colour_edges=True, edge_min_step=6.0, edge_rel=0.3):
        self.part = part
        self.use_outline = use_outline            # False: the surface term alone
        # Stage B: the outline from colour edges instead of the depth region.
        # The D405 is passive stereo: plain untextured plastic returns no
        # depth, so the depth region's edge is missing wherever the part is
        # plain near its outline (measured on the cube: the fit slid ~2 mm
        # at 100 mm, ~4 at 300). The colour edge is sharp there. An edge
        # counts when its step (grey levels per pixel, over the colour
        # channels) is at least edge_min_step and edge_rel of the strongest
        # in its search window.
        self.colour_edges = colour_edges
        self.edge_min_step = edge_min_step
        self.edge_rel = edge_rel
        # A face near edge-on shows a patchy, trimmed extent in depth, so
        # silhouette edges bordering one are not used as outline evidence.
        self.rim_min_cos = float(np.cos(np.radians(rim_max_incidence_deg)))
        # The outline is pixel-quantised: let it pin only what the surface
        # cannot. Where the surface sees nothing (a flat face's slide and
        # spin) any weight gives the same answer; where it is strong, the
        # outline's quantisation bias pulls this much less.
        self.outline_weight = outline_weight
        self.sym_order = int(part.get('sym_order', 1))
        V, F = np.asarray(part['V'], dtype=float), np.asarray(part['F'])
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        cross = np.cross(b - a, c - a)
        area2 = np.linalg.norm(cross, axis=1)
        keep = area2 > 1e-14
        self.V, self.F = V, F[keep]
        self.face_n = cross[keep] / area2[keep, None]
        self.face_c = (a + b + c)[keep] / 3.0
        self.size_m = float(np.linalg.norm(V.max(0) - V.min(0)))

        # Surface samples with normals, about one per sample_m^2.
        n = int(np.clip(area2[keep].sum() / 2.0 / sample_m ** 2, 2000, 200000))
        self.samples, self.sample_n = sample_mesh(V, self.F, n, return_normals=True)
        self.tree = cKDTree(self.samples)

        # Unique edges with their one or two faces, and points along them.
        F3 = self.F
        E = np.sort(np.concatenate([F3[:, [0, 1]], F3[:, [1, 2]], F3[:, [2, 0]]]), axis=1)
        face_of = np.tile(np.arange(len(F3)), 3)
        order = np.lexsort((E[:, 1], E[:, 0]))
        E, face_of = E[order], face_of[order]
        uniq, start, count = np.unique(E, axis=0, return_index=True, return_counts=True)
        f0 = face_of[start]
        f1 = np.where(count >= 2, face_of[np.minimum(start + 1, len(E) - 1)], -1)
        f1 = np.where(count > 2, -2, f1)                 # non-manifold: always a candidate
        self.edge_faces = np.stack([f0, f1], 1)
        ed = V[uniq[:, 1]] - V[uniq[:, 0]]
        self.edge_dir = ed / np.maximum(np.linalg.norm(ed, axis=1, keepdims=True), 1e-12)
        pts, ids = [], []
        for k, (i, j) in enumerate(uniq):
            m = max(2, int(np.ceil(np.linalg.norm(V[j] - V[i]) / edge_step_m)) + 1)
            s = np.linspace(0.0, 1.0, m)[:, None]
            pts.append(V[i] + s * (V[j] - V[i]))
            ids.append(np.full(m, k))
        self.edge_pts = np.concatenate(pts)
        self.edge_id = np.concatenate(ids)

        self.max_points = max_points
        # How far off the prior may be (process() can say per call): the
        # search window is 4x it (at least search_m) around the prior, and the
        # gates start at 2x (surface) and 6x (outline). A marker prior is good
        # to a few mm; the last estimate while tracking, to about one.
        self.search_m = search_m
        self.prior_err_m = prior_err_m
        self.depth_band_m = depth_band_m
        self.support_band_m = support_band_m
        self.jump_ratio = float(np.tan(np.radians(max_incidence_deg)))
        self.outline_offset_px = outline_offset_px
        self.max_iter = max_iter
        self.gates = dict(min_points=min_points, max_rms_m=max_rms_m,
                          min_inlier_frac=min_inlier_frac, min_outline_frac=min_outline_frac,
                          max_shift_m=max_shift_m, max_shift_deg=max_shift_deg,
                          size_ratio=size_ratio, weak_rel=weak_rel)
        self._norm_key = None
        self._norm = None

    # ------------------------------------------------------------ geometry

    def _norm_grid(self, K, dist, shape):
        """(h, w, 2) undistorted normalised coordinates of every pixel."""
        key = (tuple(np.round(K.ravel(), 6)), tuple(np.round(np.ravel(dist), 9)), shape)
        if key != self._norm_key:
            h, w = shape
            us, vs = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
            px = np.stack([us.ravel(), vs.ravel()], -1).reshape(-1, 1, 2)
            self._norm = cv2.undistortPoints(px, K, dist).reshape(h, w, 2)
            self._norm_key = key
        return self._norm

    def _facing(self, R, t):
        """Faces whose front side the camera sees."""
        n = self.face_n @ R.T
        c = self.face_c @ R.T + t
        return np.einsum('ij,ij->i', n, c) < 0.0

    def _render(self, R, t, K, dist, roi, facing=None):
        """Mask (ROI-local) of the mesh at pose R, t; dist None = pinhole."""
        x0, y0, x1, y1 = roi
        facing = self._facing(R, t) if facing is None else facing
        tri = self.F[facing]
        mask = np.zeros((y1 - y0, x1 - x0), np.uint8)
        if not len(tri):
            return mask
        P = self.V @ R.T + t
        if np.any(P[:, 2] <= 1e-4):
            return mask
        if dist is None:
            uv = P[:, :2] / P[:, 2:] * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
        else:
            uv = cv2.projectPoints(P, np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
        polys = np.round((uv[tri] - [x0, y0]) * 16).astype(np.int32)
        cv2.fillPoly(mask, list(polys), 1, lineType=cv2.LINE_8, shift=4)
        return mask

    def _rim(self, R, t, K, dist, depth, roi):
        """Silhouette points of the mesh at R, t: points on edges between a
        front and a back face (or on open edges), kept where the rendered
        mask says they are on the outer outline (not hidden by the part),
        where the front face is not near edge-on (its extent in depth is
        unreliable), and where the observed depth is not nearer (hidden by
        something else: no outline of the part to see there)."""
        n = self.face_n @ R.T
        c = self.face_c @ R.T + t
        cos_inc = -np.einsum('ij,ij->i', n, c) / np.linalg.norm(c, axis=1)
        facing = cos_inc > 0.0
        f0, f1 = self.edge_faces[:, 0], self.edge_faces[:, 1]
        a = facing[f0]
        b = np.where(f1 >= 0, facing[np.maximum(f1, 0)], False)
        front = np.where(a, f0, np.maximum(f1, 0))
        sil = ((a != b) & (cos_inc[front] > self.rim_min_cos)) | (f1 == -2)
        on = sil[self.edge_id]
        X, eid = self.edge_pts[on], self.edge_id[on]
        if not len(X):
            return X, X, eid
        P = X @ R.T + t
        ok = P[:, 2] > 1e-4
        X, P, eid = X[ok], P[ok], eid[ok]
        # Render just around the part at this pose (the distance transform
        # costs by area; the search ROI is much larger up close).
        Pv = self.V @ R.T + t
        uvv = Pv[:, :2] / np.maximum(Pv[:, 2:], 1e-4) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
        box = (int(np.floor(uvv[:, 0].min())) - 4, int(np.floor(uvv[:, 1].min())) - 4,
               int(np.ceil(uvv[:, 0].max())) + 5, int(np.ceil(uvv[:, 1].max())) + 5)
        mask = self._render(R, t, K, None, box, facing)
        dt = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
        uv = P[:, :2] / P[:, 2:] * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
        u = np.clip(np.round(uv[:, 0] - box[0]).astype(int), 0, mask.shape[1] - 1)
        v = np.clip(np.round(uv[:, 1] - box[1]).astype(int), 0, mask.shape[0] - 1)
        keep = dt[v, u] <= 1.5
        X, P, eid = X[keep], P[keep], eid[keep]
        if len(P):
            px = np.round(cv2.projectPoints(P, np.zeros(3), np.zeros(3), K, dist)[0]
                          .reshape(-1, 2)).astype(int)
            h, w = depth.shape
            inside = (px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h)
            zs = np.zeros(len(P))
            zs[inside] = depth[px[inside, 1], px[inside, 0]]
            hidden = (zs > 0.0) & (zs < P[:, 2] - 0.003)
            X, P, eid = X[~hidden], P[~hidden], eid[~hidden]
        return X, P, eid

    # ---------------------------------------------------------------- scene

    def _segment(self, depth, K, dist, prior, q, err):
        """(points (N,3), outline uv pinhole (M,2), outline normals (M,2),
        roi, the region mask (ROI-local, holes filled)) of the part's depth
        region, or None with q['reason'] set; err: how far off the prior
        may be."""
        h, w = depth.shape
        R0, t0 = prior[:3, :3], prior[:3, 3]
        Pv = self.V @ R0.T + t0
        if np.any(Pv[:, 2] <= 0.01):
            q['reason'] = 'prior behind the camera'
            return None
        uv = cv2.projectPoints(Pv, np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
        search = max(self.search_m, 4.0 * err)
        margin = int(np.ceil(search * K[0, 0] / Pv[:, 2].min())) + 4
        x0 = int(max(0, np.floor(uv[:, 0].min()) - margin))
        y0 = int(max(0, np.floor(uv[:, 1].min()) - margin))
        x1 = int(min(w, np.ceil(uv[:, 0].max()) + margin + 1))
        y1 = int(min(h, np.ceil(uv[:, 1].max()) + margin + 1))
        if x1 - x0 < 8 or y1 - y0 < 8:
            q['reason'] = 'prior out of view'
            return None
        roi = (x0, y0, x1, y1)
        z = depth[y0:y1, x0:x1].astype(np.float64)
        valid = np.isfinite(z) & (z > 0.0)
        z = np.where(valid, z, 0.0)
        nrm = self._norm_grid(K, dist, depth.shape)[y0:y1, x0:x1]   # undistorted x/z, y/z

        def points(m):
            zz = z[m]
            return np.column_stack([nrm[..., 0][m] * zz, nrm[..., 1][m] * zz, zz])

        facing = self._facing(R0, t0)
        prior_mask = self._render(R0, t0, K, dist, roi, facing).astype(bool)
        if not prior_mask.any():
            q['reason'] = 'prior renders empty'
            return None

        # Support plane from sparse depth (every 4th pixel) around the prior
        # silhouette's box grown by half the search margin, out to at least
        # 0.6 of the part's size beyond it: wider than the search window, so
        # the fit sees real support even when the prior is tight.
        vy, vx = np.nonzero(prior_mask)
        bx0, bx1, by0, by1 = vx.min() + x0, vx.max() + x0, vy.min() + y0, vy.max() + y0
        g = max(3, margin // 2)
        G = max(g + 8, int(0.6 * max(bx1 - bx0, by1 - by0)))
        wx0, wy0 = max(0, bx0 - G), max(0, by0 - G)
        wx1, wy1 = min(w, bx1 + G + 1), min(h, by1 + G + 1)
        full_nrm = self._norm_grid(K, dist, depth.shape)
        zs = depth[wy0:wy1:4, wx0:wx1:4].astype(np.float64)
        ys, xs = np.mgrid[wy0:wy1:4, wx0:wx1:4]
        ring = (np.isfinite(zs) & (zs > 0.0)
                & ~((xs >= bx0 - g) & (xs <= bx1 + g) & (ys >= by0 - g) & (ys <= by1 + g)))
        cand = valid.copy()
        q['support'] = None
        if ring.sum() >= 100:
            zr = zs[ring]
            gr = full_nrm[wy0:wy1:4, wx0:wx1:4][ring]
            pts = np.column_stack([gr[:, 0] * zr, gr[:, 1] * zr, zr])
            if len(pts) > 800:                    # plenty for a plane
                pts = pts[np.linspace(0, len(pts) - 1, 800).astype(int)]
            fit = fit_plane_robust(pts)
            if fit is not None:
                n, c = fit[0], fit[1]
                n = -n if n[2] > 0 else n                  # toward the camera
                if (t0 - c) @ n > self.support_band_m + 0.002:   # the part is in front of it
                    # height above the plane, from depth and the ray grid
                    a = nrm[..., 0] * n[0] + nrm[..., 1] * n[1] + n[2]
                    cand &= z * a - n @ c > self.support_band_m
                    q['support'] = (n.tolist(), float(n @ c))

        # Depths the part can show at the prior, with a margin for its error.
        vis = np.unique(self.F[facing].ravel())
        zv = Pv[vis, 2]
        band = self.depth_band_m + search * 0.5
        cand &= (z > zv.min() - band) & (z < zv.max() + band)

        # Split at depth jumps (another surface, a finger over the part). A
        # step is a jump when no surface the camera can see could make it:
        # neighbours sit z/f apart sideways, so a face steeper than
        # max_incidence_deg (86: the D405 returns nothing steeper) would be
        # needed. A fixed threshold instead cut steep faces seen edge-on.
        # Only the FARTHER pixel of each jump goes: at an occluding edge the
        # nearer one is the surface's own last pixel, and trimming it only
        # where the neighbour happens to have depth would shift the outline.
        jump = np.zeros_like(valid)
        dx = np.diff(z, axis=1)
        dy = np.diff(z, axis=0)
        lim = self.jump_ratio / K[0, 0]
        jx = (np.abs(dx) > lim * np.minimum(z[:, 1:], z[:, :-1])) & valid[:, 1:] & valid[:, :-1]
        jy = (np.abs(dy) > lim * np.minimum(z[1:, :], z[:-1, :])) & valid[1:, :] & valid[:-1, :]
        jump[:, 1:] |= jx & (dx > 0)
        jump[:, :-1] |= jx & (dx < 0)
        jump[1:, :] |= jy & (dy > 0)
        jump[:-1, :] |= jy & (dy < 0)
        cand &= ~jump

        n_lab, lab = cv2.connectedComponents(cand.astype(np.uint8), connectivity=8)
        if n_lab < 2:
            q['reason'] = 'no depth at the part'
            return None
        overlap = np.bincount(lab[prior_mask], minlength=n_lab)
        overlap[0] = 0
        best = int(np.argmax(overlap))
        if overlap[best] == 0:
            q['reason'] = 'nothing where the prior is'
            return None
        region = (lab == best).astype(np.uint8)
        # Fill its holes (a marker's black cells give no depth) and take
        # back any candidate depth inside them.
        pad = np.pad(region, 1)                   # the corner is always outside
        cv2.floodFill(pad, np.zeros((pad.shape[0] + 2, pad.shape[1] + 2), np.uint8), (0, 0), 1)
        filled = region.copy()
        filled[pad[1:-1, 1:-1] == 0] = 1
        mask = (filled.astype(bool) & cand) | region.astype(bool)
        ratio = mask.sum() / max(1, prior_mask.sum())
        q['size_ratio'] = float(ratio)
        lo, hi = self.gates['size_ratio']
        if not lo <= ratio <= hi:
            q['reason'] = f'size {ratio:.2f} of the expected'
            return None

        # About max_points depth points, evenly over the region: every k-th
        # pixel both ways (a voxel grid costs a sort every frame).
        k = max(1, int(np.ceil(np.sqrt(mask.sum() / self.max_points))))
        sub = np.zeros_like(mask)
        sub[::k, ::k] = True
        pts = points(mask & sub)

        # Outline: the outer contour, with outward normals from its own
        # shape (the chord over +-3 neighbours, turned a quarter).
        cont, _ = cv2.findContours(filled, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cs, ns = [], []
        for cc in cont:
            cc = cc.reshape(-1, 2).astype(np.float64)
            if len(cc) < 8:
                continue
            tan = sum(np.roll(cc, -j, axis=0) - np.roll(cc, j, axis=0) for j in (1, 2, 3))
            cs.append(cc)
            ns.append(np.stack([tan[:, 1], -tan[:, 0]], 1))
        if not cs:
            q['reason'] = 'no outline'
            return None
        c = np.concatenate(cs)
        n_out = np.concatenate(ns)
        gn = np.linalg.norm(n_out, axis=1)
        ok = gn > 1e-9
        c, n_out = c[ok], n_out[ok] / gn[ok, None]
        H, W = filled.shape
        inward = np.round(c + 1.5 * n_out).astype(int)          # outward must leave the region
        inward[:, 0] = np.clip(inward[:, 0], 0, W - 1)
        inward[:, 1] = np.clip(inward[:, 1], 0, H - 1)
        n_out[filled[inward[:, 1], inward[:, 0]] > 0] *= -1.0
        ci = c.astype(int)
        # Not the ROI border (the part is cut there, not ending).
        ok = (ci[:, 0] > 1) & (ci[:, 0] < W - 2) & (ci[:, 1] > 1) & (ci[:, 1] < H - 2)
        c, ci, n_out = c[ok], ci[ok], n_out[ok]
        # Not where something NEARER borders it: that edge is the occluder's.
        probe = np.round(c + 3.0 * n_out).astype(int)
        probe[:, 0] = np.clip(probe[:, 0], 0, W - 1)
        probe[:, 1] = np.clip(probe[:, 1], 0, H - 1)
        z_in = z[ci[:, 1], ci[:, 0]]
        z_out = z[probe[:, 1], probe[:, 0]]
        occluded = valid[probe[:, 1], probe[:, 0]] & (z_out < z_in - 0.003)
        c, n_out = c[~occluded], n_out[~occluded]
        if len(c) < 20:
            q['reason'] = 'outline occluded'
            return None
        # Pixel centres of the outermost pixels sit half a pixel inside the edge.
        full = c + [x0, y0] + self.outline_offset_px * n_out
        pin = cv2.undistortPoints(full.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)
        pin2 = cv2.undistortPoints((full + 2.0 * n_out).reshape(-1, 1, 2), K, dist,
                                   P=K).reshape(-1, 2)
        nu = pin2 - pin
        nu /= np.maximum(np.linalg.norm(nu, axis=1, keepdims=True), 1e-12)
        return pts, pin, nu, roi, filled

    # ------------------------------------------------------------- solve

    def process(self, depth_m, bgr_clean, K, dist, prior, prior_err_m=None):
        t_start = time.perf_counter()
        err = self.prior_err_m if prior_err_m is None else float(prior_err_m)
        K = np.asarray(K, dtype=np.float64)
        dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64).ravel()
        prior = np.asarray(prior, dtype=np.float64)
        q = {'source': 'depth', 'valid': False, 'reason': '', 'n_pts': 0,
             'rms_mm': None, 'inlier_frac': None, 'outline_frac': None, 'weak_dof': [],
             'agree_mm': None, 'agree_deg': None, 'sym_index': 0, 'iterations': 0,
             'edge_source': None, 'support': None, 'cand_pose': None}
        depth = np.asarray(depth_m)
        seg = self._segment(depth, K, dist, prior, q, err)
        if seg is None:
            q['compute_ms'] = (time.perf_counter() - t_start) * 1e3
            return None, False, q
        S, out_uv, out_n, roi, region = seg
        q['n_pts'] = int(len(S))
        ctx = {'S': S, 'K': K, 'dist': dist, 'depth': depth, 'roi': roi,
               'f': 0.5 * (K[0, 0] + K[1, 1]), 'out_tree': cKDTree(out_uv),
               'out_uv': out_uv, 'out_n': out_n, 'region': region}

        # Stage A: the depth outline, from the prior (only close enough for
        # the colour edge search when stage B follows). When colour edges
        # follow and the prior is already inside their search window (the
        # last estimate while tracking), only height and tilt from the
        # surface: the depth outline would add time and nothing else.
        colour = self.colour_edges and self.use_outline and bgr_clean is not None
        near = colour and err * ctx['f'] / max(float(prior[2, 3]), 0.05) <= 6.0
        T, rim = self._solve(prior, ctx, q, coarse=colour, err=err, outline=not near)
        if T is None:
            q['compute_ms'] = (time.perf_counter() - t_start) * 1e3
            return None, False, q
        edges = obs = None
        q['edge_source'] = 'depth' if self.use_outline else None
        # Stage B: the outline from colour edges, from stage A's pose, with
        # the silhouette points it sees there. Kept only if enough of them
        # found an edge; otherwise stage A stands (edge_source 'depth').
        if colour:
            X, _, eid = self._rim(T[:3, :3], T[:3, 3], K, dist, depth, roi)
            if len(X):
                img = self._edge_image(bgr_clean, roi)
                T_b, frac, obs_b = self._stage_b(T, ctx, q, (X, eid), img)
                if T_b is not None and frac >= self.gates['min_outline_frac']:
                    T, rim, edges, obs = T_b, (X, eid), (img, (3, 4)), obs_b
                    q['edge_source'] = 'colour'
            if edges is None:                               # finish stage A properly
                T, rim = self._solve(T, ctx, q, err=0.0015 if not near else err)
                if T is None:
                    q['compute_ms'] = (time.perf_counter() - t_start) * 1e3
                    return None, False, q

        # Final statistics at the tight gates (the last colour round's edges).
        R, t = T[:3, :3], T[:3, 3]
        J, r, wts, _ = self._rows(ctx, R, t, 0.003, 0.003, q, rim=rim, edges=edges, obs=obs,
                                  final=True)
        if J is not None:
            L = max(self.size_m / 2.0, 0.005)
            D = np.diag([1.0 / L] * 3 + [1.0] * 3)
            Hs = D @ (J.T @ (wts[:, None] * J)) @ D
            ev, evec = np.linalg.eigh(Hs)
            weak = ev < self.gates['weak_rel'] * ev.max()
            q['weak_dof'] = sorted({DOF_NAMES[int(np.argmax(np.abs(evec[:, i])))]
                                    for i in np.flatnonzero(weak)})

        # Symmetry: the equivalent pose nearest the prior's X.
        if self.sym_order > 1:
            best, best_k = -2.0, 0
            for k in range(self.sym_order):
                a = 2.0 * np.pi * k / self.sym_order
                Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
                d = float((T[:3, :3] @ Rz)[:, 0] @ prior[:3, 0])
                if d > best:
                    best, best_k = d, k
            a = 2.0 * np.pi * best_k / self.sym_order
            Rz = np.eye(4)
            Rz[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
            T = T @ Rz
            q['sym_index'] = best_k

        D = np.linalg.inv(prior) @ T
        q['agree_mm'] = float(np.linalg.norm(D[:3, 3]) * 1e3)
        q['agree_deg'] = float(np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2,
                                                            -1, 1))))
        q['cand_pose'] = _to_pose7(T)
        g = self.gates
        reasons = []
        if q['n_pts'] < g['min_points']:
            reasons.append(f"{q['n_pts']} points")
        if q['rms_mm'] is None or q['rms_mm'] > g['max_rms_m'] * 1e3:
            reasons.append(f"rms {q['rms_mm']}")
        if (q['inlier_frac'] or 0.0) < g['min_inlier_frac']:
            reasons.append(f"inliers {q['inlier_frac']}")
        if (q['outline_frac'] or 0.0) < g['min_outline_frac']:
            reasons.append(f"outline {q['outline_frac']}")
        if q['agree_mm'] > g['max_shift_m'] * 1e3 or q['agree_deg'] > g['max_shift_deg']:
            reasons.append(f"shift {q['agree_mm']:.1f} mm / {q['agree_deg']:.1f} deg")
        if q['weak_dof']:
            reasons.append('weak ' + ','.join(q['weak_dof']))
        q['valid'] = not reasons
        q['reason'] = '; '.join(reasons)
        q['compute_ms'] = (time.perf_counter() - t_start) * 1e3
        return T, q['valid'], q

    def _solve(self, T0, ctx, q, coarse=False, err=0.005, outline=True):
        """Stage A: Gauss-Newton from T0 against the depth outline, gates
        shrinking from 2x err (surface) and 6x err (outline) to 3 mm, the
        silhouette points frozen once both are tight. coarse (colour edges
        follow and will set the outline anyway): stop at 0.05 mm / 0.05 deg
        a step instead of 0.01 / 0.01, and after 8 steps at most. outline
        False: the surface alone (height and tilt; the prior keeps the rest).
        Returns (T, rim) or (None, rim)."""
        T = np.array(T0, dtype=np.float64, copy=True)
        tol_t, tol_r = (5e-5, np.radians(0.05)) if coarse else (1e-5, np.radians(0.01))
        rim = None
        # coarse: stage B's search window (8 in, 10 out px) absorbs what is left
        for it in range(8 if coarse else self.max_iter):
            R, t = T[:3, :3], T[:3, 3]
            gate_s = max(0.003, 2.0 * err * 0.6 ** it)
            gate_o = max(0.003, 6.0 * err * 0.6 ** it)
            J, r, wts, X = self._rows(ctx, R, t, gate_s, gate_o, q, rim=rim, outline=outline)
            # Once the gates are tight, keep the silhouette points fixed:
            # re-deciding their visibility every step made them flicker in
            # and out by sub-pixel pose changes, and the answer with them.
            if outline and rim is None and gate_s <= 0.003 and gate_o <= 0.003:
                rim = X
            if J is None:
                return None, rim
            x = self._gn_step(J, r, wts)
            T = self._apply(T, x)
            q['iterations'] = q.get('iterations', 0) + 1
            if ((rim is not None or not outline) and np.linalg.norm(x[3:]) < tol_t
                    and np.linalg.norm(x[:3]) < tol_r):
                break
        return T, rim

    def _stage_b(self, T, ctx, q, rim, img):
        """Stage B: colour-edge rounds. Each round finds the edges from the
        current pose, then runs Gauss-Newton with them and the surface pairs
        held FIXED: re-searching every step let the targets move with the
        model and the solve crawl. A wide search (8 px in, 10 out), then a
        narrow one (3, 4) from where the first landed. Returns (T, the
        fraction of silhouette points that found an edge in the last round,
        that round's edges) or (None, that fraction, None)."""
        X, eid = rim
        frac = 0.0
        for win, steps in (((8, 10), 4), ((3, 4), 3)):
            R, t = T[:3, :3], T[:3, 3]
            obs = self._find_edges(X, eid, R, t, ctx, img, win)
            frac = len(obs[0]) / max(1, len(X))
            pairs = self._surface_pairs(ctx, R, t, 0.003)
            if len(obs[0]) + len(pairs[0]) < 12:
                return None, frac, None
            for _ in range(steps):
                J, r, w = self._fixed_rows(ctx, T, pairs, X, obs, max(win) + 1.0)
                x = self._gn_step(J, r, w)
                T = self._apply(T, x)
                q['iterations'] = q.get('iterations', 0) + 1
                if np.linalg.norm(x[3:]) < 1e-5 and np.linalg.norm(x[:3]) < np.radians(0.01):
                    break
        return T, frac, obs

    @staticmethod
    def _gn_step(J, r, w):
        """The Gauss-Newton update, at most 5 mm / 5 deg: one bad step must
        not carry the search into another basin."""
        H = J.T @ (w[:, None] * J)
        H += np.eye(6) * 1e-9 * max(np.trace(H), 1e-12)
        x = -np.linalg.solve(H, J.T @ (w * r))
        return x * min(1.0, 0.005 / max(np.linalg.norm(x[3:]), 1e-12),
                       np.radians(5.0) / max(np.linalg.norm(x[:3]), 1e-12))

    @staticmethod
    def _apply(T, x):
        """T <- T A^-1 for the update A = (rotvec x[:3], x[3:]) applied to
        the scene in the object frame (as icp.py does)."""
        R_a = cv2.Rodrigues(x[:3])[0]
        A_inv = np.eye(4)
        A_inv[:3, :3] = R_a.T
        A_inv[:3, 3] = -R_a.T @ x[3:]
        return T @ A_inv

    @staticmethod
    def _edge_image(bgr, roi, margin=16):
        """The colour image around the ROI, lightly smoothed, as float."""
        x0, y0, x1, y1 = roi
        h, w = bgr.shape[:2]
        x0, y0 = max(0, x0 - margin), max(0, y0 - margin)
        x1, y1 = min(w, x1 + margin), min(h, y1 + margin)
        img = cv2.GaussianBlur(np.ascontiguousarray(bgr[y0:y1, x0:x1], dtype=np.float32),
                               (0, 0), 0.7)
        return img, (x0, y0)

    @staticmethod
    def _outline_jac(Xo, Po, ns, R, K, scale):
        """Rows d(ns . uv)/d(w, v) * scale for model points Xo seen at Po."""
        zo = Po[:, 2]
        Jpi = np.zeros((len(Po), 2, 3))
        Jpi[:, 0, 0] = K[0, 0] / zo
        Jpi[:, 0, 2] = -K[0, 0] * Po[:, 0] / zo ** 2
        Jpi[:, 1, 1] = K[1, 1] / zo
        Jpi[:, 1, 2] = -K[1, 1] * Po[:, 1] / zo ** 2
        a = np.einsum('ij,ijk->ik', ns, Jpi) * scale[:, None]           # (M,3)
        dPw = R[None] @ _skew(Xo)                                       # d P / d w
        return np.hstack([np.einsum('ij,ijk->ik', a, dPw), -a @ R])    # d P / d v = -R

    def _surface_pairs(self, ctx, R, t, gate_s):
        """(scene point indices, model samples, their normals): each scene
        point with a camera-facing model sample within gate_s."""
        y = (ctx['S'] - t) @ R
        d, idx = self.tree.query(y, distance_upper_bound=gate_s)
        ok = np.flatnonzero(np.isfinite(d))
        m, nm = self.samples[idx[ok]], self.sample_n[idx[ok]]
        front = np.einsum('ij,ij->i', nm @ R.T, m @ R.T + t) < 0.0
        return ok[front], m[front], nm[front]

    @staticmethod
    def _surface_rows(ctx, R, t, pairs):
        si, mk, nk = pairs
        y = (ctx['S'][si] - t) @ R
        return np.hstack([np.cross(y, nk), nk]), np.einsum('ij,ij->i', y - mk, nk)

    def _depth_outline_rows(self, X, P, R, ctx, gate_o):
        """Silhouette points against the depth region's outline."""
        K, f = ctx['K'], ctx['f']
        z = P[:, 2]
        uv = P[:, :2] / z[:, None] * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
        gpx = gate_o * f / z
        dd, j = ctx['out_tree'].query(uv, distance_upper_bound=float(gpx.max()))
        ok = np.isfinite(dd)
        ok[ok] &= dd[ok] <= gpx[ok]
        if not ok.any():
            return np.zeros((0, 6)), np.zeros(0), np.zeros(0)
        Po, jo = P[ok], j[ok]
        ns = ctx['out_n'][jo]
        scale = Po[:, 2] / f                                            # metres per pixel
        ro = np.einsum('ij,ij->i', uv[ok] - ctx['out_uv'][jo], ns) * scale
        return self._outline_jac(X[ok], Po, ns, R, K, scale), ro, _tukey(ro, gate_o)

    def _find_edges(self, X, eid, R, t, ctx, img, win):
        """(indices into X, edge points (image px), normals) of the
        silhouette points that found a colour edge: from each point, along
        the silhouette's outward normal in the image, the OUTERMOST clear
        edge within win = (in, out) px. Outermost, because the silhouette is
        the part's outer boundary: a strong edge just inside it (a label, a
        patch of another colour) is the part's own marking."""
        K, dist = ctx['K'], ctx['dist']
        img, (ox, oy) = img
        win_in, win_out = win
        none = np.zeros(0, int), np.zeros((0, 2)), np.zeros((0, 2))
        if not len(X):
            return none
        z3 = np.zeros(3)
        P = X @ R.T + t
        uv = cv2.projectPoints(P, z3, z3, K, dist)[0].reshape(-1, 2)
        d = self.edge_dir[eid] @ R.T
        tan = cv2.projectPoints(P + 0.001 * d, z3, z3, K, dist)[0].reshape(-1, 2) - uv
        tan /= np.maximum(np.linalg.norm(tan, axis=1, keepdims=True), 1e-12)
        nrm = np.stack([tan[:, 1], -tan[:, 0]], 1)
        facing = self._facing(R, t)                    # outward = away from the front face
        f0, f1 = self.edge_faces[eid, 0], self.edge_faces[eid, 1]
        front = np.where(facing[f0], f0, np.maximum(f1, 0))
        uvc = cv2.projectPoints(self.face_c[front] @ R.T + t, z3, z3, K, dist)[0].reshape(-1, 2)
        nrm[np.einsum('ij,ij->i', uv - uvc, nrm) < 0.0] *= -1.0
        # Samples along each normal: the search window, and 3-7 px further
        # in for the part's own colours near its outline.
        s = np.arange(-win_in - 8, win_out + 2, dtype=np.float64)
        mx = (uv[:, :1] - ox + nrm[:, :1] * s).astype(np.float32)
        my = (uv[:, 1:] - oy + nrm[:, 1:] * s).astype(np.float32)
        smp = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        smp = smp.reshape(len(X), len(s), -1)
        win = smp[:, 7:]                                            # offsets -win_in-1..win_out+1
        M = np.sqrt((((win[:, 2:] - win[:, :-2]) * 0.5) ** 2).sum(-1))   # offsets -win_in..win_out
        thr = np.maximum(self.edge_min_step, self.edge_rel * M.max(axis=1))[:, None]
        pk = (M[:, 1:-1] >= M[:, :-2]) & (M[:, 1:-1] > M[:, 2:]) & (M[:, 1:-1] >= thr)
        # An edge is the silhouette only if its inner side looks like the part
        # near its outline: a dark band just outside (the stand and its
        # shadow coming into view from further up) has an edge of its own
        # that is further out, and 'outermost' alone took it. The part's
        # colours: a coarse histogram of the deep samples all round; only
        # its MAJOR colours count (3% of the samples), or small dark print
        # near the outline (the cube's tape) passes the band as the part.
        pk &= self._part_like(smp[:, 1:6].reshape(-1, smp.shape[2]),
                              0.5 * (smp[:, 6:6 + pk.shape[1]] + smp[:, 7:7 + pk.shape[1]]), pk)
        found = np.flatnonzero(pk.any(axis=1))
        if not len(found):
            return none
        k = pk.shape[1] - np.argmax(pk[found, ::-1], axis=1)       # outermost peak, index into M
        m0, m1, m2 = M[found, k - 1], M[found, k], M[found, k + 1]
        den = m0 - 2.0 * m1 + m2
        safe = np.abs(den) > 1e-9
        delta = np.clip(np.where(safe, 0.5 * (m0 - m2) / np.where(safe, den, 1.0), 0.0),
                        -0.5, 0.5)
        off = k - win_in + delta                                    # edge offset along nrm, px
        return found, uv[found] + off[:, None] * nrm[found], nrm[found]

    @staticmethod
    def _part_like(pool, inner, pk, share=0.03, tol=24.0, max_colours=12, blend_colours=6):
        """Which candidates (pk) have an inner colour that belongs to the
        part: near one of its MAJOR colours near the outline (bins of 16
        levels holding at least `share` of the pool), or near the blend of
        two of them - a narrow strip of one colour beside another (the
        cube's yellow beside its blue patch) reads as their blend."""
        q = np.clip(pool // 16, 0, 15).astype(int)
        ch = q.shape[1]
        flat = np.ravel_multi_index(tuple(q.T), (16,) * ch)
        hist = np.bincount(flat, minlength=16 ** ch).reshape((16,) * ch).astype(np.float64)
        hist = uniform_filter(hist, size=3, mode='constant') * 3 ** ch
        # The part's colours: every well-filled bin, not just the peaks - a
        # colour that varies (translucent tape over yellow) fills a spread of
        # bins that one peak per colour would not cover.
        major = np.argwhere(hist >= max(3.0, share * len(pool)))
        out = np.zeros_like(pk)
        if not len(major):
            return out
        major = major[np.argsort(-hist[tuple(major.T)])[:max_colours]]
        C = (major + 0.5) * 16.0                                        # (Kc, ch)
        rows, cols = np.nonzero(pk)
        c = inner[rows, cols]                                           # (P, ch)
        ok = (((c[:, None, :] - C[None]) ** 2).sum(axis=2) <= tol * tol).any(axis=1)
        # blends only between the main colours, and only for what is left
        B = C[:blend_colours]
        i, j = np.triu_indices(len(B), 1)
        rest = np.flatnonzero(~ok)
        if len(i) and len(rest):
            cr = c[rest]
            a, ab = B[i], B[j] - B[i]                                   # segments a -> a + ab
            tt = np.clip(np.einsum('pkc,kc->pk', cr[:, None, :] - a[None], ab)
                         / np.maximum((ab ** 2).sum(1), 1e-9), 0.0, 1.0)
            e = cr[:, None, :] - (a[None] + tt[..., None] * ab[None])
            ok[rest] = ((e ** 2).sum(axis=2) <= tol * tol).any(axis=1)
        out[rows, cols] = ok
        return out

    def _edge_obs_rows(self, X, obs, R, t, ctx, c_px):
        """Rows of silhouette points against FIXED edge points: the distance
        along each edge's normal, in the image."""
        idx, e_uv, nrm = obs
        if not len(idx):
            return np.zeros((0, 6)), np.zeros(0), np.zeros(0)
        K, f = ctx['K'], ctx['f']
        Xo = X[idx]
        Po = Xo @ R.T + t
        z3 = np.zeros(3)
        uv = cv2.projectPoints(Po, z3, z3, K, ctx['dist'])[0].reshape(-1, 2)
        e_px = np.einsum('ij,ij->i', uv - e_uv, nrm)                 # model minus edge
        scale = Po[:, 2] / f
        return self._outline_jac(Xo, Po, nrm, R, K, scale), e_px * scale, _tukey(e_px, c_px)

    def _fixed_rows(self, ctx, T, pairs, X, obs, c_px):
        """Both terms with their correspondences held fixed (stage B)."""
        R, t = T[:3, :3], T[:3, 3]
        Js, rs = self._surface_rows(ctx, R, t, pairs)
        Jo, ro, wo = self._edge_obs_rows(X, obs, R, t, ctx, c_px)
        return (np.vstack([Js, Jo]), np.concatenate([rs, ro]),
                np.concatenate([_tukey(rs, 0.003), self.outline_weight * wo]))

    def _rows(self, ctx, R, t, gate_s, gate_o, q, rim=None, edges=None, obs=None,
              final=False, outline=True):
        """Stacked Jacobian rows, residuals (m) and robust weights of both
        terms at pose R, t, in the parameters of an update A applied to the
        scene in the object frame (T <- T A^-1), as icp.py does; and the
        silhouette points used, (X, edge ids) (rim: use these instead of
        finding them; edges: (image, window) - the outline from colour
        edges found from this pose, or obs: those edges already found;
        outline False: the surface alone)."""
        S = ctx['S']
        pairs = self._surface_pairs(ctx, R, t, gate_s)
        Js, rs = self._surface_rows(ctx, R, t, pairs)

        # Outline: model silhouette points against the depth outline, or
        # against colour edges.
        if not (self.use_outline and outline):
            X, eid = np.zeros((0, 3)), np.zeros(0, int)
            Jo, ro, wo = np.zeros((0, 6)), np.zeros(0), np.zeros(0)
        else:
            if rim is not None:
                X, eid = rim
                P = X @ R.T + t
            else:
                X, P, eid = self._rim(R, t, ctx['K'], ctx['dist'], ctx['depth'], ctx['roi'])
            if not len(X):
                Jo, ro, wo = np.zeros((0, 6)), np.zeros(0), np.zeros(0)
            elif edges is not None:
                img, win = edges
                if obs is None:
                    obs = self._find_edges(X, eid, R, t, ctx, img, win)
                Jo, ro, wo = self._edge_obs_rows(X, obs, R, t, ctx, max(win) + 1.0)
            else:
                Jo, ro, wo = self._depth_outline_rows(X, P, R, ctx, gate_o)

        if final:
            q['rms_mm'] = float(np.sqrt(np.mean(rs ** 2)) * 1e3) if len(rs) else None
            q['inlier_frac'] = float(len(rs) / max(1, len(S)))
            q['outline_frac'] = float(len(ro) / max(1, len(X)))
        if len(rs) + len(ro) < 12:
            q['reason'] = 'too few correspondences'
            return None, None, None, (X, eid)
        J = np.vstack([Js, Jo])
        r = np.concatenate([rs, ro])
        wts = np.concatenate([_tukey(rs, gate_s), self.outline_weight * wo])
        return J, r, wts, (X, eid)
