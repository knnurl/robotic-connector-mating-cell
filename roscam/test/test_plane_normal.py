#!/usr/bin/env python3
"""Tests for depth plane fitting and IPPE flip disambiguation."""

import numpy as np
from roscam.plane_normal import (disambiguate_by_normal, fit_plane,
                                 fit_plane_robust, marker_plane_normal,
                                 quad_mask, range_scale, solve_square_by_depth)

FX = FY = 389.0
CX, CY = 316.1, 236.1        # measured D405 intrinsics at 640x480


def synth_depth(normal, dist, shape=(480, 640), fx=FX, fy=FY,
                cx=CX, cy=CY, noise=0.0, seed=0):
    """Depth image of a single plane  n . X = dist  (metres)."""
    rng = np.random.default_rng(seed)
    h, w = shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    dx = (us - cx) / fx
    dy = (vs - cy) / fy
    denom = normal[0] * dx + normal[1] * dy + normal[2]
    denom[np.abs(denom) < 1e-9] = np.nan
    z = dist / denom
    z[~np.isfinite(z)] = 0.0
    z[z <= 0] = 0.0
    if noise:
        z = np.where(z > 0, z + rng.normal(0, noise, z.shape), z)
    return z


def quad(cx_px, cy_px, half):
    return np.array([[cx_px - half, cy_px - half], [cx_px + half, cy_px - half],
                     [cx_px + half, cy_px + half], [cx_px - half, cy_px + half]],
                    dtype=float)


def test_fit_plane_recovers_exact_normal():
    rng = np.random.default_rng(1)
    n_true = np.array([0.1, -0.2, 1.0])
    n_true /= np.linalg.norm(n_true)
    basis = np.linalg.svd(n_true.reshape(1, 3))[2][1:]
    pts = (np.array([0.0, 0.0, 0.3])
           + rng.normal(0, 0.02, (200, 2)) @ basis)
    n, c, rms = fit_plane(pts)
    assert abs(abs(n @ n_true) - 1.0) < 1e-8
    assert rms < 1e-9


def test_fit_plane_rejects_degenerate():
    assert fit_plane(np.zeros((2, 3))) is None          # too few
    line = np.linspace(0, 1, 50)[:, None] * np.array([1.0, 0, 0])
    assert fit_plane(line) is None                      # collinear


def test_robust_fit_beats_plain_fit_with_outliers():
    rng = np.random.default_rng(2)
    n_true = np.array([0.0, 0.0, -1.0])
    pts = np.column_stack([rng.uniform(-.02, .02, 300),
                           rng.uniform(-.02, .02, 300),
                           np.full(300, 0.10)])
    # a slab of background 3 cm behind, as the card edge would give
    out = np.column_stack([rng.uniform(-.02, .02, 40),
                           rng.uniform(.015, .02, 40),
                           np.full(40, 0.13)])
    allp = np.vstack([pts, out])
    n_plain = fit_plane(allp)[0]
    n_rob = fit_plane_robust(allp)[0]
    err_plain = np.degrees(np.arccos(min(1, abs(n_plain @ n_true))))
    err_rob = np.degrees(np.arccos(min(1, abs(n_rob @ n_true))))
    assert err_rob < err_plain
    assert err_rob < 1.0


def test_quad_mask_scales_about_centroid():
    m1 = quad_mask((480, 640), quad(320, 240, 20), scale=1.0)
    m2 = quad_mask((480, 640), quad(320, 240, 20), scale=2.0)
    assert m2.sum() > m1.sum() * 3        # area scales ~4x
    assert m1[240, 320] and m2[240, 320]


def test_marker_plane_normal_matches_synthetic_tilt():
    for tilt_deg in (0.0, 5.0, 10.0, 20.0):
        t = np.radians(tilt_deg)
        n_true = np.array([np.sin(t), 0.0, np.cos(t)])   # tilt about image Y
        depth = synth_depth(n_true, 0.10, noise=0.0002)
        got = marker_plane_normal(depth, quad(320, 240, 38), FX, FY, CX, CY)
        assert got is not None, f'no fit at {tilt_deg} deg'
        n = got['normal']
        assert n[2] < 0, 'normal must face the camera'
        ang = np.degrees(np.arccos(min(1, abs(n @ n_true))))
        assert ang < 1.5, f'{tilt_deg} deg tilt -> {ang:.2f} deg error'


def test_marker_plane_normal_rejects_empty_depth():
    assert marker_plane_normal(None, quad(320, 240, 30),
                               FX, FY, CX, CY) is None
    assert marker_plane_normal(np.zeros((480, 640)), quad(320, 240, 30),
                               FX, FY, CX, CY) is None


def test_disambiguate_picks_the_matching_mirror_solution():
    """The real failure: two IPPE solutions, same tilt, mirrored direction."""
    t = np.radians(8.0)
    a = np.array([np.sin(t), 0.0, -np.cos(t)])      # true
    b = np.array([-np.sin(t), 0.0, -np.cos(t)])     # mirror
    # depth says the tilt leans the 'a' way
    i, agree = disambiguate_by_normal([a, b], a + np.array([0, 0.001, 0]))
    assert i == 0 and agree > 0.99
    i, agree = disambiguate_by_normal([a, b], b)
    assert i == 1 and agree > 0.99


def test_disambiguate_uses_sign_not_magnitude():
    """abs() would make the two mirror solutions indistinguishable."""
    t = np.radians(8.0)
    a = np.array([np.sin(t), 0.0, -np.cos(t)])
    b = np.array([-np.sin(t), 0.0, -np.cos(t)])
    assert abs(abs(a @ a) - abs(b @ a)) > 1e-6 or True   # documents intent
    i, _ = disambiguate_by_normal([b, a], a)
    assert i == 1, 'must pick the true one even when listed second'


def test_disambiguate_handles_bad_input():
    assert disambiguate_by_normal([], [0, 0, 1]) is None
    assert disambiguate_by_normal([[0, 0, 1]], [0, 0, 0]) is None


def test_flip_sequence_is_stabilised():
    """Replay the measured flip pattern; disambiguation must lock one branch.

    Directions taken from autoconverge_20260911_180721: the in-plane
    component alternated sign on most frames.
    """
    t = np.radians(8.0)
    true_n = np.array([0.017, 0.147, -0.989])
    true_n /= np.linalg.norm(true_n)
    mirror = np.array([-true_n[0], -true_n[1], true_n[2]])
    mirror /= np.linalg.norm(mirror)
    # solver offers them in arbitrary order each frame
    order = [0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0]
    picked = []
    for o in order:
        cands = [true_n, mirror] if o == 0 else [mirror, true_n]
        i, _ = disambiguate_by_normal(cands, true_n)
        picked.append(cands[i])
    P = np.array(picked)
    ang = np.degrees(np.arctan2(P[:, 1], P[:, 0]))
    spread = ang.max() - ang.min()
    assert spread < 1.0, f'still flipping: {spread:.1f} deg spread'
    assert np.allclose(P, true_n, atol=1e-9), 'must lock the TRUE branch'
    assert abs(t) > 0                                    # tilt is nonzero


def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def test_fuse_orientation_is_a_proper_rotation():
    from roscam.plane_normal import fuse_orientation
    R = _rotz(np.radians(30)) @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
    n = np.array([0.1, -0.05, -0.99])
    F = fuse_orientation(R, n)
    assert F is not None
    assert np.allclose(F.T @ F, np.eye(3), atol=1e-9), 'not orthonormal'
    assert abs(np.linalg.det(F) - 1.0) < 1e-9, 'not right-handed'


def test_fuse_orientation_takes_normal_from_depth():
    from roscam.plane_normal import fuse_orientation
    R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])     # tilt 0
    t = np.radians(9.0)
    n = np.array([np.sin(t), 0.0, -np.cos(t)])              # depth says 9 deg
    F = fuse_orientation(R, n)
    got = np.degrees(np.arccos(min(1, abs(F[:, 2] @ [0, 0, 1.0]))))
    assert abs(got - 9.0) < 1e-6, f'normal not adopted: {got:.3f} deg'


def test_fuse_orientation_preserves_in_plane_rotation():
    from roscam.plane_normal import fuse_orientation
    for deg in (0.0, 17.0, -42.0, 88.0):
        R = _rotz(np.radians(deg)) @ np.array([[1, 0, 0], [0, -1, 0],
                                               [0, 0, -1.0]])
        n = np.array([0.08, 0.03, -0.996])
        F = fuse_orientation(R, n)
        # in-plane angle is preserved to within the tilt-induced projection
        a_in = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        f_in = np.degrees(np.arctan2(F[1, 0], F[0, 0]))
        d = abs(((a_in - f_in) + 180) % 360 - 180)
        assert d < 1.0, f'in-plane drifted {d:.2f} deg at {deg} deg'


def test_fuse_orientation_fixes_a_biased_tilt_floor():
    """The real bug: ArUco reads a tilt floor, depth reads the truth."""
    from roscam.plane_normal import fuse_orientation
    true_t = np.radians(1.0)
    n_depth = np.array([np.sin(true_t), 0.0, -np.cos(true_t)])
    bias_t = np.radians(7.0)                      # what IPPE reported
    R_aruco = np.array([[np.cos(bias_t), 0, np.sin(bias_t)],
                        [0, -1, 0],
                        [np.sin(bias_t), 0, -np.cos(bias_t)]])
    before = np.degrees(np.arccos(min(1, abs(R_aruco[:, 2] @ [0, 0, 1.0]))))
    F = fuse_orientation(R_aruco, n_depth)
    after = np.degrees(np.arccos(min(1, abs(F[:, 2] @ [0, 0, 1.0]))))
    assert before > 6.0, f'setup wrong: {before:.2f}'
    assert after < 1.1, f'still reading the bias: {after:.2f} deg'


def test_fuse_orientation_rejects_bad_input():
    from roscam.plane_normal import fuse_orientation
    assert fuse_orientation(np.eye(3), [0, 0, 0]) is None
    assert fuse_orientation(np.eye(2), [0, 0, 1]) is None
    # X parallel to the normal is degenerate
    assert fuse_orientation(np.eye(3), [1, 0, 0]) is None


def test_inplane_angle_matches_optical_convention():
    from roscam.plane_normal import inplane_angle
    # marker X along camera +X -> red points RIGHT -> 0 deg
    assert abs(inplane_angle(np.eye(3)) - 0.0) < 1e-9
    # marker X along camera +Y -> red points DOWN -> +90 (optical Y is down)
    R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
    assert abs(inplane_angle(R) - 90.0) < 1e-9
    R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])
    assert abs(inplane_angle(R) + 90.0) < 1e-9


def test_inplane_angle_rejects_bad_shape():
    from roscam.plane_normal import inplane_angle
    assert inplane_angle(np.eye(2)) is None


def test_wrap_deg():
    from roscam.plane_normal import wrap_deg
    for a, want in ((0, 0), (180, 180), (-180, 180), (190, -170),
                    (-190, 170), (360, 0), (450, 90)):
        assert abs(wrap_deg(a) - want) < 1e-9, f'{a} -> {wrap_deg(a)}'


def test_inplane_correction_takes_the_short_way():
    """The measured case: +83.67 -> +90 is 6.33 deg, NOT 353.67."""
    from roscam.plane_normal import inplane_correction
    c = inplane_correction(83.67, 90.0)
    assert abs(c + 6.33) < 1e-6, c
    # and the long way round is never chosen
    assert abs(inplane_correction(-179.0, 179.0)) <= 2.0 + 1e-9


def test_inplane_correction_clamps():
    from roscam.plane_normal import inplane_correction
    assert abs(inplane_correction(0.0, 90.0, limit_deg=5.0) + 5.0) < 1e-9
    assert abs(inplane_correction(90.0, 0.0, limit_deg=5.0) - 5.0) < 1e-9
    # already there -> no motion
    assert abs(inplane_correction(45.0, 45.0, limit_deg=5.0)) < 1e-12


def test_inplane_correction_converges_when_iterated():
    from roscam.plane_normal import inplane_correction, wrap_deg
    cur, target, lim = 83.67, 90.0, 2.0
    for _ in range(50):
        d = inplane_correction(cur, target, lim)
        if abs(d) < 1e-9:
            break
        cur = wrap_deg(cur - d)
    assert abs(wrap_deg(cur - target)) < 1e-6, cur


def test_a_static_target_marker_is_solved_on_the_depth_side_of_the_mirror():
    """The place target: IPPE's two mirrored tilts, decided by depth alone."""
    import cv2
    K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
    half = 0.0105
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
                   dtype=np.float32)
    for tilt_deg in (8.0, -8.0, 15.0):
        R = cv2.Rodrigues(np.array([np.radians(tilt_deg), 0.0, 0.0]))[0] @ np.diag([1, -1, -1])
        t = np.array([0.01, -0.005, 0.22])
        img, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], t, K, None)
        n_cam = R[:, 2]                                  # marker z, toward the camera
        depth = synth_depth(-n_cam, float(-n_cam @ t))   # the plane the marker lies on
        sol = solve_square_by_depth(obj, img.reshape(-1, 2).astype(np.float32), K, None, depth)
        assert sol is not None
        R_got = cv2.Rodrigues(sol[0])[0]
        assert float(R_got[:, 2] @ n_cam) > np.cos(np.radians(1.0)), tilt_deg
        assert np.allclose(sol[1].reshape(3), t, atol=2e-3)
        assert float(np.asarray(sol[2]) @ n_cam) > np.cos(np.radians(1.0))   # the depth normal
        # the returned plane puts a 5% long ArUco distance back on the truth
        s_ = range_scale(1.05 * sol[1].reshape(3), sol[2], sol[3])
        assert np.allclose(1.05 * s_ * sol[1].reshape(3), t, atol=2e-4), tilt_deg
    assert solve_square_by_depth(obj, img.reshape(-1, 2).astype(np.float32), K, None,
                                 None) is None          # no depth: never guess


def test_the_target_solve_follows_depth_not_opencvs_order():
    """With noise-free corners OpenCV's first IPPE branch is the true one, so
    a solver that ignored depth would still pass the test above. Here the
    depth plane is built from the MIRROR branch: the solve must follow it."""
    import cv2
    K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
    half = 0.0105
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
                   dtype=np.float32)
    R = cv2.Rodrigues(np.array([np.radians(10.0), 0.0, 0.0]))[0] @ np.diag([1, -1, -1])
    t = np.array([0.0, 0.0, 0.22])
    img = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], t, K, None)[0].reshape(-1, 2)
    img = img.astype(np.float32)
    n_sol, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, K, None,
                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
    assert n_sol == 2
    first, mirror = (cv2.Rodrigues(r)[0][:, 2] for r in rvecs)
    assert float(first @ mirror) < np.cos(np.radians(5.0))      # the branches really differ
    depth = synth_depth(-mirror, float(-mirror @ tvecs[1].reshape(3)))
    sol = solve_square_by_depth(obj, img, K, None, depth)
    assert sol is not None
    assert float(cv2.Rodrigues(sol[0])[0][:, 2] @ mirror) > np.cos(np.radians(1.0))


def test_range_scale_puts_the_ray_on_the_plane():
    n = np.array([0.0, 0.0, -1.0])                      # facing the camera, 100 mm away
    t_true = np.array([0.012, -0.020, 0.100])
    s = range_scale(1.04 * t_true, n, [0.3, -0.2, 0.100])   # any point on the plane
    assert np.allclose(s * 1.04 * t_true, t_true)
    tilt = np.radians(30.0)                             # a tilted plane through t_true
    n = np.array([0.0, np.sin(tilt), -np.cos(tilt)])
    s = range_scale(0.97 * t_true, n, t_true)
    assert np.allclose(s * 0.97 * t_true, t_true)
    assert np.isclose(range_scale(t_true, -n, t_true), 1.0)  # the normal's sign is irrelevant


def test_range_scale_refuses_a_grazing_ray_or_a_plane_behind():
    t = np.array([0.0, 0.0, 0.1])
    assert range_scale(t, [1.0, 0.0, 0.01], [0.0, 0.0, 0.1]) is None     # ray almost in-plane
    assert range_scale(t, [0.0, 0.0, -1.0], [0.0, 0.0, -0.1]) is None    # behind the camera
    assert range_scale(t, [0.0, 0.0, 0.0], [0.0, 0.0, 0.1]) is None      # no normal
    assert range_scale([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 0.0, 0.1]) is None
