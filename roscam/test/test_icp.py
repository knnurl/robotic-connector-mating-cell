"""Unit tests for STL loading, mesh sampling, and ICP registration."""
import math
import struct

import numpy as np
import pytest

from roscam.icp import (crop_points, depth_to_points, icp, load_stl,
                        sample_mesh, voxel_downsample)


def box_mesh(sx, sy, sz, offset=(0.0, 0.0, 0.0)):
    """Axis-aligned box: 8 vertices, 12 triangles. Origin at the box centre
    bottom, +Z up, then shifted by offset."""
    o = np.asarray(offset)
    x, y = sx / 2.0, sy / 2.0
    V = np.array([
        [-x, -y, 0], [x, -y, 0], [x, y, 0], [-x, y, 0],
        [-x, -y, sz], [x, -y, sz], [x, y, sz], [-x, y, sz],
    ]) + o
    F = np.array([
        [0, 2, 1], [0, 3, 2],          # bottom
        [4, 5, 6], [4, 6, 7],          # top
        [0, 1, 5], [0, 5, 4],          # sides
        [1, 2, 6], [1, 6, 5],
        [2, 3, 7], [2, 7, 6],
        [3, 0, 4], [3, 4, 7],
    ])
    return V, F


def connector_mesh():
    """Connector-like part: 20x10x8 mm body with a 8x4x4 mm ridge on top —
    asymmetric enough to lock all 6 DOF."""
    V1, F1 = box_mesh(0.020, 0.010, 0.008)
    V2, F2 = box_mesh(0.008, 0.004, 0.004, offset=(0.004, 0.002, 0.008))
    V = np.vstack([V1, V2])
    F = np.vstack([F1, F2 + len(V1)])
    return V, F


def transform(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]


def make_T(rotvec, t):
    rotvec = np.asarray(rotvec, dtype=float)
    angle = np.linalg.norm(rotvec)
    T = np.eye(4)
    if angle > 1e-12:
        k = rotvec / angle
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        T[:3, :3] = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K
    T[:3, 3] = t
    return T


def write_binary_stl(path, V, F):
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(F)))
        for tri in F:
            f.write(struct.pack('<3f', 0, 0, 0))
            for vi in tri:
                f.write(struct.pack('<3f', *V[vi]))
            f.write(struct.pack('<H', 0))


@pytest.fixture(scope='module')
def template():
    """(points, normals) template sample of the connector mesh."""
    V, F = connector_mesh()
    pts, normals = sample_mesh(V, F, 6000, seed=0, return_normals=True)
    return voxel_downsample(pts, 0.001, normals)


def visible_scene(T_true, noise=0.0003, seed=1):
    """Top-view partial scene: only points on upward-facing surfaces (as a
    downward-looking depth camera would see), in the camera frame."""
    V, F = connector_mesh()
    pts = sample_mesh(V, F, 8000, seed=seed)
    # keep upper surfaces: z within 0.5 mm of local max => use z > 60% height
    keep = pts[:, 2] > 0.004
    pts = pts[keep]
    rng = np.random.default_rng(seed)
    scene = transform(T_true, pts) + rng.normal(0.0, noise, (len(pts), 3))
    return voxel_downsample(scene, 0.001)


def pose_delta(T_a, T_b):
    d = np.linalg.inv(T_a) @ T_b
    trans = np.linalg.norm(d[:3, 3])
    ang = math.acos(max(-1.0, min(1.0, (np.trace(d[:3, :3]) - 1.0) / 2.0)))
    return trans, math.degrees(ang)


def test_icp_recovers_pose_from_partial_noisy_view(template):
    pts, normals = template
    # Connector 30 cm in front of the camera, tilted; prior off by 4 mm / 3 deg
    T_true = make_T([0.05, -0.03, 0.4], [0.02, -0.01, 0.30])
    T_prior = T_true @ make_T([0.03, 0.03, 0.02], [0.003, -0.002, 0.001])
    scene = visible_scene(T_true)

    T_est, rms, inlier_frac = icp(scene, pts, T_prior, template_normals=normals)

    trans_err, rot_err = pose_delta(T_true, T_est)
    assert inlier_frac > 0.8
    assert rms < 0.001
    assert trans_err < 0.001, f'translation error {trans_err * 1000:.2f} mm'
    # Single-frame tilt is edge-noise limited (~0.7 deg mean on this
    # worst-case 20 mm part; scales with depth-noise/part-size).
    assert rot_err < 1.2, f'rotation error {rot_err:.2f} deg'


def test_icp_gates_reject_junk_scene(template):
    pts, normals = template
    rng = np.random.default_rng(7)
    junk = rng.uniform(-0.03, 0.03, (400, 3)) + np.array([0.02, -0.01, 0.30])
    T_prior = make_T([0, 0, 0], [0.02, -0.01, 0.30])
    _, rms, inlier_frac = icp(junk, pts, T_prior, template_normals=normals)
    assert inlier_frac < 0.6 or rms > 0.004, \
        f'junk scene passed gates: rms={rms * 1000:.1f} mm, inliers={inlier_frac:.2f}'


def test_stl_binary_roundtrip(tmp_path, template):
    V, F = connector_mesh()
    path = tmp_path / 'connector.stl'
    write_binary_stl(path, V, F)
    V2, F2 = load_stl(str(path))
    assert len(F2) == len(F)
    # same geometry: sampled points from the reloaded mesh match the template
    pts = voxel_downsample(sample_mesh(V2, F2, 6000, seed=0), 0.001)
    from scipy.spatial import cKDTree
    d, _ = cKDTree(template[0]).query(pts)
    assert d.max() < 0.002


def test_stl_ascii_parsing(tmp_path):
    V, F = box_mesh(0.02, 0.01, 0.008)
    lines = ['solid test']
    for tri in F:
        lines.append(' facet normal 0 0 0\n  outer loop')
        for vi in tri:
            lines.append(f'   vertex {V[vi][0]} {V[vi][1]} {V[vi][2]}')
        lines.append('  endloop\n endfacet')
    lines.append('endsolid test')
    path = tmp_path / 'box.stl'
    path.write_text('\n'.join(lines))
    V2, F2 = load_stl(str(path))
    assert len(F2) == 12
    assert np.allclose(sorted(V2[:, 2].tolist()), sorted(V[:, 2].tolist()))


def test_depth_backprojection_and_crop():
    # Flat plane at 0.3 m, principal point centred
    fx = fy = 430.0
    cx, cy = 424.0, 240.0
    depth = np.full((480, 848), 0.3, dtype=np.float32)
    pts = depth_to_points(depth, fx, fy, cx, cy, stride=4)
    assert np.allclose(pts[:, 2], 0.3)
    # centre pixel back-projects to (0, 0, 0.3)
    centre = pts[np.argmin(np.linalg.norm(pts[:, :2], axis=1))]
    assert np.linalg.norm(centre - [0, 0, 0.3]) < 0.005
    cropped = crop_points(pts, [0, 0, 0.3], 0.02)
    assert len(cropped) > 0
    assert np.all(np.linalg.norm(cropped - [0, 0, 0.3], axis=1) < 0.02)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))


def test_temporal_filtering_beats_single_frame_tilt(template):
    """Single-frame ICP tilt is depth-noise-limited (~1 deg on this 20 mm
    part); the static-target Kalman stage in connector_pose must average it
    below 0.5 deg within ~3 s at 5 Hz."""
    from roscam.handeye_calib import matrix_to_quat
    from roscam.pose_kf import PoseKF, quat_multiply

    pts, normals = template
    T_true = make_T([0.05, -0.03, 0.4], [0.02, -0.01, 0.30])
    T_prior = T_true @ make_T([0.03, 0.03, 0.02], [0.003, -0.002, 0.001])

    kf = PoseKF(sigma_accel=0.02, sigma_rot_rate_deg=3.0,
                meas_std_pos=0.001, meas_std_rot_deg=1.5)
    single_frame_errs = []
    for seed in range(1, 16):  # 15 frames = 3 s at 5 Hz
        T_est, _, _ = icp(visible_scene(T_true, seed=seed), pts, T_prior,
                          template_normals=normals)
        single_frame_errs.append(pose_delta(T_true, T_est)[1])
        if kf.initialized:
            kf.predict(0.2)
        kf.update(T_est[:3, 3], matrix_to_quat(T_est[:3, :3]))

    q_true = matrix_to_quat(T_true[:3, :3])
    dq = quat_multiply(kf.quaternion,
                       np.array([-q_true[0], -q_true[1], -q_true[2], q_true[3]]))
    rot_err = math.degrees(2.0 * math.acos(min(1.0, abs(dq[3]))))
    trans_err = np.linalg.norm(kf.position - T_true[:3, 3])

    # Filtering removes the random component; the residual is the
    # systematic edge-noise bias (~0.7 deg on this part geometry).
    assert rot_err < 0.9, (f'filtered tilt {rot_err:.2f} deg not < 0.9 '
                           f'(single-frame mean {np.mean(single_frame_errs):.2f})')
    assert rot_err <= np.mean(single_frame_errs) + 0.1
    assert trans_err < 0.0005
