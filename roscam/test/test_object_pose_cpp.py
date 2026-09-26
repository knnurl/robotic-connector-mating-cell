#!/usr/bin/env python3
"""object_pose_cpp gives the Python estimator's answers: the same pose (to
rounding), validity, reason and quality numbers, on the scenes of
test_object_pose.py - depth only and with colour edges, the fallback, a
plain part, an angled view, an occluder, junk and a prior out of view.
Skipped where object_pose_cpp is not built (colcon build --packages-select
object_pose_cpp, then source the workspace)."""

import numpy as np
import pytest

pytest.importorskip('object_pose_cpp')

from roscam.object_pose import (CppObjectPoseEstimator, ObjectPoseEstimator,  # noqa: E402
                                box_mesh, pose_error, rpy_matrix)
from test_object_pose import (CUBE, K, PART, R_DOWN, _plain_plastic, colour_scene,  # noqa: E402
                              cube_scene, perturbed, pose, raycast)

DIST = np.array([-0.0545, 0.0569, 0.0006, 0.0006, -0.0187])   # the D405's, near enough


def same(a, b):
    (Ta, va, qa), (Tb, vb, qb) = a, b
    assert (va, qa['reason']) == (vb, qb['reason'])
    assert (Ta is None) == (Tb is None)
    for k in ('n_pts', 'weak_dof', 'edge_source', 'sym_index', 'iterations'):
        assert qa[k] == qb[k], k
    for k in ('rms_mm', 'inlier_frac', 'outline_frac', 'agree_mm', 'agree_deg',
              'agree_tilt_deg', 'agree_inplane_deg', 'size_ratio'):
        if qa.get(k) is None:
            assert qb.get(k) is None, k
        else:
            assert qb[k] == pytest.approx(qa[k], rel=1e-6, abs=1e-6), k
    if Ta is not None:
        d, tilt, yaw = pose_error(Tb, Ta, 4)
        assert np.linalg.norm(d) < 1e-7 and tilt < 1e-5 and abs(yaw) < 1e-5


def run_both(depth, bgr, prior, dist=None, err=None, **kw):
    py, cpp = ObjectPoseEstimator(PART, **kw), CppObjectPoseEstimator(PART, **kw)
    out = [e.process(depth, bgr, K, dist, prior, prior_err_m=err) for e in (py, cpp)]
    same(*out)
    return out[0]


def test_colour_edges_where_plain_plastic_has_no_depth():
    for T in (pose([0.006, -0.004, 0.150], yaw=12.0, tilt=2.0),
              pose([0.004, 0.010, 0.280], yaw=-20.0, tilt=3.0)):
        depth, bgr = colour_scene(T, hole=_plain_plastic)
        _, valid, q = run_both(depth, bgr, perturbed(T, 4.0, 2.0))
        assert valid and q['edge_source'] == 'colour', q['reason']


def test_the_marker_seeded_fast_path_with_distortion():
    T = pose([0.004, -0.003, 0.150], yaw=10.0, tilt=2.0)
    depth, bgr = colour_scene(T)
    run_both(depth, bgr, perturbed(T, 0.6, 0.4), dist=DIST, err=0.0015, depth_fallback=False)


def test_a_dark_band_and_the_patch_near_the_outline():
    T = pose([0.0, 0.0, 0.250], yaw=4.0)
    depth, bgr = colour_scene(T, band_mm=2.5)
    run_both(depth, bgr, perturbed(T, 3.0, 1.5))


def test_no_colour_contrast_falls_back_or_is_reported():
    T = pose([0.003, 0.0, 0.150], yaw=8.0)
    depth, bgr = colour_scene(T, plain=True)
    _, _, q = run_both(depth, bgr, perturbed(T, 3.0, 1.5))
    assert q['edge_source'] == 'depth'
    _, valid, q = run_both(depth, bgr, perturbed(T, 3.0, 1.5), depth_fallback=False)
    assert not valid and q['reason'].startswith('no colour outline')


def test_depth_only_from_a_perturbed_prior():
    T = pose([0.010, -0.005, 0.150], yaw=20.0, tilt=4.0)
    run_both(cube_scene(T, noise=0.0002), None, perturbed(T, 8.0, 4.0))


def test_an_angled_view_and_the_outline_off():
    R_side = rpy_matrix([35.0, 0.0, 0.0]) @ R_DOWN
    T = pose([0.0, 0.02, 0.170], yaw=15.0, R_base=R_side)
    depth = cube_scene(T, noise=0.0002)
    run_both(depth, None, perturbed(T, 6.0, 3.0))
    _, valid, _ = run_both(cube_scene(pose([0.0, 0.0, 0.150], yaw=10.0)), None,
                           perturbed(pose([0.0, 0.0, 0.150], yaw=10.0), 3.0, 2.0),
                           use_outline=False)
    assert not valid                                  # weak in-plane: the same verdict


def test_an_occluder_and_junk():
    T = pose([0.0, 0.0, 0.150], yaw=5.0)
    Vf, Ff = box_mesh([0.015, 0.060, 0.010])
    Vf = Vf + [0.022, 0.0, 0.015]
    depth = cube_scene(T, extra=[(Vf @ T[:3, :3].T + T[:3, 3], Ff, False)])
    run_both(depth, None, perturbed(T, 4.0, 2.0))
    empty = raycast([], (T[:3, 2], float(T[:3, 2] @ (T[:3, 3] - 0.085 * T[:3, 2]))))
    _, valid, q = run_both(empty, None, T)
    assert not valid and q['reason']


def test_priors_out_of_view_or_behind_the_camera():
    depth = cube_scene(pose([0.0, 0.0, 0.150]))
    for prior in (pose([0.5, 0.0, 0.150]), pose([0.0, 0.0, -0.1])):
        _, valid, q = run_both(depth, None, prior)
        assert not valid and q['reason'] in ('prior out of view', 'prior behind the camera',
                                             'prior renders empty')


def test_the_symmetry_snap():
    T = pose([0.0, 0.004, 0.140], yaw=30.0)
    run_both(cube_scene(T), None, pose([0.0, 0.004, 0.140], yaw=30.0 + 86.0))


def test_a_box_that_is_not_a_cube():
    """Nothing in the port is specific to the cube."""
    part = {'V': box_mesh([0.04, 0.025, 0.012])[0], 'F': box_mesh([0.04, 0.025, 0.012])[1],
            'sym_order': 2}
    T = pose([0.002, 0.001, 0.160], yaw=25.0, tilt=3.0)
    V, F = part['V'], part['F']
    depth = raycast([(V @ T[:3, :3].T + T[:3, 3], F, False)],
                    (T[:3, 2], float(T[:3, 2] @ (T[:3, 3] - 0.012 * T[:3, 2]))))
    py, cpp = ObjectPoseEstimator(part), CppObjectPoseEstimator(part)
    prior = perturbed(T, 2.0, 1.0)
    same(py.process(depth, None, K, None, prior), cpp.process(depth, None, K, None, prior))
    assert CUBE > 0.04                                # (the cube's scenes above)


def test_bad_input_never_crashes_and_matches():
    """A crash in C++ would take the vision process down with it: NaN and
    infinite depth, noise, a tiny image and odd priors must come back as
    rejections, and as the Python ones."""
    rng = np.random.default_rng(3)
    T = pose([0.0, 0.0, 0.150], yaw=5.0)
    depth, bgr = colour_scene(T)
    nan = depth.copy()
    nan[rng.random(nan.shape) < 0.3] = np.nan
    nan[rng.random(nan.shape) < 0.05] = np.inf
    noise = rng.uniform(0.0, 0.5, depth.shape)
    cases = [(nan, bgr, perturbed(T, 2.0, 1.0)), (noise, bgr, T),
             (np.zeros_like(depth), bgr, T),
             (depth[200:240, 300:340].copy(), bgr[200:240, 300:340].copy(), T),
             (depth, rng.integers(0, 256, bgr.shape, dtype=np.uint8), T),
             (depth, bgr, pose([0.0, 0.0, 0.02], yaw=5.0)),
             (depth.astype(np.float32), bgr, perturbed(T, 1.0, 0.5))]
    for d, c, prior in cases:
        run_both(d, c, prior)
