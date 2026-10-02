#!/usr/bin/env python3
"""object_pose on ray-cast depth: the cube on a black stand (no depth) over
a table, seen from above as TRACK and GRIP see it, and at an angle."""

import numpy as np

from roscam.object_pose import (ObjectPoseEstimator, box_mesh, load_part, pose_error,
                                rpy_matrix)

FX = FY = 389.0
CX, CY = 316.1, 236.1
K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
CUBE = 0.055
PART = {'V': box_mesh([CUBE] * 3)[0], 'F': box_mesh([CUBE] * 3)[1], 'sym_order': 4}
R_DOWN = np.diag([1.0, -1.0, -1.0])       # object Z toward a camera looking down


def pose(t, yaw=0.0, tilt=0.0, R_base=R_DOWN):
    T = np.eye(4)
    T[:3, :3] = R_base @ rpy_matrix([tilt, 0.0, yaw])
    T[:3, 3] = t
    return T


def raycast(meshes, table=None, shape=(480, 640), noise=0.0, seed=0):
    """Depth of the nearest hit through each pixel centre (pinhole).
    meshes: [(V_cam, F, blind)]; a blind mesh (the black stand) hides what
    is behind it and returns no depth. table: (n, d), the plane n.X = d."""
    h, w = shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    d = np.stack([(us - CX) / FX, (vs - CY) / FY, np.ones((h, w))], -1).reshape(-1, 3)
    best = np.full(len(d), np.inf)
    blind = np.zeros(len(d), bool)
    for V, F, is_blind in meshes:
        for tri in F:
            v0, v1, v2 = V[tri]
            e1, e2 = v1 - v0, v2 - v0
            p = np.cross(d, e2)
            det = p @ e1
            ok = np.abs(det) > 1e-12
            inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
            s = -v0
            u = (p @ s) * inv
            qv = np.cross(s, e1)
            v = (d @ qv) * inv
            z = (e2 @ qv) * inv
            hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (z > 0) & (z < best)
            best[hit] = z[hit]
            blind[hit] = is_blind
    if table is not None:
        n, dd = table
        zt = dd / (d @ n)
        hit = (zt > 0) & (zt < best)
        best[hit] = zt[hit]
        blind[hit] = False
    z = np.where(np.isfinite(best) & ~blind, best, 0.0)
    if noise:
        z = np.where(z > 0, z + np.random.default_rng(seed).normal(0, noise, z.shape), 0.0)
    return z.reshape(h, w)


def cube_scene(T, stand=True, table=True, noise=0.0, extra=()):
    """The cube at T, on a 55 x 110 x 30 mm black stand, on a table."""
    V, F = box_mesh([CUBE] * 3)
    meshes = [(V @ T[:3, :3].T + T[:3, 3], F, False)]
    if stand:
        Vs, Fs = box_mesh([CUBE, 2 * CUBE, 0.03])
        Vs = Vs + [0.0, 0.0, -CUBE]
        meshes.append((Vs @ T[:3, :3].T + T[:3, 3], Fs, True))
    meshes += list(extra)
    tb = None
    if table:
        n = T[:3, 2]                                   # object Z = the table normal
        tb = (n, float(n @ (T[:3, 3] + T[:3, :3] @ [0.0, 0.0, -CUBE - 0.03])))
    return raycast(meshes, tb, noise=noise)


def perturbed(T, mm, deg, axis=(0.3, -0.5, 0.8), rot_axis=(0.2, 0.9, 0.4)):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    r = np.asarray(rot_axis, float) / np.linalg.norm(rot_axis)
    P = np.eye(4)
    import cv2
    P[:3, :3] = cv2.Rodrigues(r * np.radians(deg))[0]
    P[:3, 3] = a * mm / 1000.0
    return T @ P


def check(T_est, T_true, mm=0.3, deg=0.3):
    dxyz, tilt, yaw = pose_error(T_est, T_true, 4)
    assert np.linalg.norm(dxyz) * 1e3 < mm, (dxyz * 1e3, tilt, yaw)
    assert tilt < deg and abs(yaw) < deg, (dxyz * 1e3, tilt, yaw)


def test_the_box_faces_point_outward():
    V, F = box_mesh([0.055, 0.04, 0.03])
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    n = np.cross(b - a, c - a)
    centre = V.mean(axis=0)
    assert np.all(np.einsum('ij,ij->i', n, (a + b + c) / 3 - centre) > 0)
    assert np.isclose(V[:, 2].max(), 0.0) and np.isclose(V[:, 2].min(), -0.03)


def test_a_part_file_takes_a_box_or_an_stl(tmp_path):
    V, F = box_mesh([55.0, 55.0, 55.0])                 # mm, as a CAD export would be
    lines = ['solid box']
    for tri in F:
        lines += ['facet normal 0 0 0', 'outer loop']
        lines += [f'vertex {x} {y} {z}' for x, y, z in V[tri]]
        lines += ['endloop', 'endfacet']
    (tmp_path / 'box.stl').write_text('\n'.join(lines + ['endsolid box']) + '\n')
    (tmp_path / 'stl.yaml').write_text(
        'mesh: {stl: box.stl, scale: 0.001}\n'
        'symmetry: {order: 4}\n'
        'T_marker_object: {xyz_mm: [1, 2, 0], rpy_deg: [0, 0, 90]}\n')
    (tmp_path / 'box.yaml').write_text('mesh: {box_mm: [55, 55, 55]}\n')
    a, b = load_part(tmp_path / 'stl.yaml'), load_part(tmp_path / 'box.yaml')
    assert np.allclose(np.sort(a['V'], axis=0), np.sort(b['V'], axis=0))
    assert a['sym_order'] == 4 and b['sym_order'] == 1
    assert np.allclose(a['T_marker_object'][:3, 3], [0.001, 0.002, 0.0])
    assert np.allclose(a['T_marker_object'][:3, 0], [0.0, 1.0, 0.0])


def test_top_down_from_a_perturbed_prior():
    T = pose([0.010, -0.005, 0.150], yaw=20.0, tilt=4.0)
    depth = cube_scene(T, noise=0.0002)
    est = ObjectPoseEstimator(PART)
    T_est, valid, q = est.process(depth, None, K, None, perturbed(T, 8.0, 4.0))
    assert valid, q['reason']
    assert not q['weak_dof']
    check(T_est, T)


def test_the_outline_is_what_pins_a_flat_face():
    """Without the outline term a face seen head on can slide and spin in
    its plane: weak_dof must say so, and the pose must not pass."""
    T = pose([0.0, 0.0, 0.150], yaw=10.0)
    depth = cube_scene(T)
    est = ObjectPoseEstimator(PART, use_outline=False)
    _, valid, q = est.process(depth, None, K, None, perturbed(T, 3.0, 2.0))
    assert not valid
    assert {'tx', 'ty', 'rz'} <= set(q['weak_dof']), q['weak_dof']


def test_the_symmetry_snap_follows_the_prior():
    """A prior a quarter turn off lands on the equivalent pose nearest it."""
    T = pose([0.0, 0.004, 0.140], yaw=30.0)
    depth = cube_scene(T)
    prior = pose([0.0, 0.004, 0.140], yaw=30.0 + 86.0)
    T_est, valid, q = ObjectPoseEstimator(PART).process(depth, None, K, None, prior)
    assert valid, q['reason']
    check(T_est, T)                                   # the same cube, modulo 90 deg
    assert float(T_est[:3, 0] @ prior[:3, 0]) > np.cos(np.radians(5.0))


def test_an_angled_view_with_two_faces():
    R_side = rpy_matrix([35.0, 0.0, 0.0]) @ R_DOWN    # camera tilted over one edge
    T = pose([0.0, 0.02, 0.170], yaw=15.0, R_base=R_side)
    depth = cube_scene(T, noise=0.0002)
    T_est, valid, q = ObjectPoseEstimator(PART).process(depth, None, K, None,
                                                        perturbed(T, 6.0, 3.0))
    assert valid, q['reason']
    check(T_est, T, mm=0.4)


def test_frames_without_the_part_are_rejected():
    T = pose([0.0, 0.0, 0.150])
    V, F = box_mesh([CUBE] * 3)
    empty = raycast([], (T[:3, 2], float(T[:3, 2] @ (T[:3, 3] - 0.085 * T[:3, 2]))))
    _, valid, q = ObjectPoseEstimator(PART).process(empty, None, K, None, T)
    assert not valid and q['reason']


def test_an_occluder_is_never_a_wrong_valid_pose():
    """A finger 15 mm above the top face, over one edge: the pose is either
    rejected or still right - never confidently wrong."""
    T = pose([0.0, 0.0, 0.150], yaw=5.0)
    Vf, Ff = box_mesh([0.015, 0.060, 0.010])
    Vf = Vf + [0.022, 0.0, 0.015]                   # over the +x edge, above the face
    depth = cube_scene(T, extra=[(Vf @ T[:3, :3].T + T[:3, 3], Ff, False)])
    T_est, valid, q = ObjectPoseEstimator(PART).process(depth, None, K, None,
                                                        perturbed(T, 4.0, 2.0))
    if valid:
        check(T_est, T, mm=1.0, deg=1.0)


def test_a_far_prior_converges_but_is_gated():
    """20 mm off: the search still finds the cube (the basin), and the
    shift gate still refuses to publish so large a jump."""
    T = pose([0.0, 0.0, 0.120], yaw=12.0)
    depth = cube_scene(T)
    T_est, valid, q = ObjectPoseEstimator(PART).process(depth, None, K, None,
                                                        perturbed(T, 20.0, 5.0),
                                                        prior_err_m=0.020)
    check(T_est, T)
    assert not valid and 'shift' in q['reason']


# ------------------------------------------------------- colour (stage B)

YELLOW, WHITE, BLUE, INK = (40, 200, 215), (235, 235, 235), (200, 120, 60), (30, 30, 30)
SIDE, STAND, GREY = (20, 110, 130), (25, 25, 25), (150, 150, 150)


def trace(meshes, table, us, vs):
    """Nearest hit through pixel positions us, vs (pinhole): depth, which
    mesh (-1 the table, -2 nothing), which triangle, and the ray."""
    d = np.stack([(us - CX) / FX, (vs - CY) / FY, np.ones(us.shape)], -1).reshape(-1, 3)
    best = np.full(len(d), np.inf)
    mid = np.full(len(d), -2)
    fid = np.full(len(d), -1)
    for k, (V, F, _) in enumerate(meshes):
        for fi, tri in enumerate(F):
            v0, v1, v2 = V[tri]
            e1, e2 = v1 - v0, v2 - v0
            p = np.cross(d, e2)
            det = p @ e1
            ok = np.abs(det) > 1e-12
            inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
            u = (p @ -v0) * inv
            qv = np.cross(-v0, e1)
            v = (d @ qv) * inv
            z = (e2 @ qv) * inv
            hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (z > 0) & (z < best)
            best[hit], mid[hit], fid[hit] = z[hit], k, fi
    n, dd = table
    zt = dd / (d @ n)
    hit = (zt > 0) & (zt < best)
    best[hit], mid[hit] = zt[hit], -1
    return best, mid, fid, d


def colour_scene(T, hole=None, patch=True, plain=False, band_mm=0.0, ss=2, seed=0, extra=()):
    """(depth, bgr) of the cube on its black stand over a striped table.
    The top face is yellow with a white label, the black marker and, when
    patch, a blue patch ending 2 mm short of the +x edge. hole(x, y) (top
    face, object frame, m) marks plain plastic: no depth there. plain: one
    grey everywhere (no colour contrast at all). band_mm: a dark band that
    wide in the image just outside the -x edge (seen on the cell from 200 mm
    up: the stand and its shadow coming into view beside the cube). extra:
    more boxes, (size xyz, T_camera_box with its origin at the box's
    top-face centre, bgr, hole or None), e.g. a finger or a second cube."""
    V, F = box_mesh([CUBE] * 3)
    Vs, Fs = box_mesh([CUBE, 2 * CUBE, 0.03])
    Vs = Vs + [0.0, 0.0, -CUBE]
    meshes = [(V @ T[:3, :3].T + T[:3, 3], F, False), (Vs @ T[:3, :3].T + T[:3, 3], Fs, True)]
    for size, Tx, _, _ in extra:
        Vx, Fx = box_mesh(size)
        meshes.append((Vx @ Tx[:3, :3].T + Tx[:3, 3], Fx, False))
    n = T[:3, 2]
    table = (n, float(n @ (T[:3, 3] + T[:3, :3] @ [0.0, 0.0, -CUBE - 0.03])))
    Ti = np.linalg.inv(T)
    h, w = 480, 640

    def obj(z, d):
        return (d * z[:, None]) @ Ti[:3, :3].T + Ti[:3, 3]

    # Colour: ss x ss samples per pixel, averaged.
    sub = lambda n: (np.arange(n * ss) + 0.5) / ss - 0.5      # noqa: E731
    us, vs = np.meshgrid(sub(w), sub(h))
    z, mid, fid, d = trace(meshes, table, us, vs)
    Po = obj(np.where(np.isfinite(z), z, 0.0), d)
    col = np.zeros((len(z), 3))
    top = (mid == 0) & (fid <= 1)
    col[(mid == 0) & ~top] = SIDE
    col[mid == 1] = STAND
    stripes = (np.sin(Po[:, 1] / 0.003) > 0)[:, None]
    col[mid == -1] = np.where(stripes, (180, 180, 180), (105, 105, 105))[mid == -1]
    x, y = Po[:, 0], Po[:, 1]
    face = np.tile(YELLOW, (len(z), 1)).astype(float)
    face[(np.abs(x - 0.001) < 0.014) & (np.abs(y) < 0.016)] = WHITE
    face[(np.abs(x) < 0.0105) & (np.abs(y) < 0.0105)] = INK
    if patch:
        face[(x > 0.018) & (x < 0.0255) & (np.abs(y) < 0.010)] = BLUE
    col[top] = face[top]
    for k, (_, _, c, _) in enumerate(extra):
        col[mid == 2 + k] = c
    if band_mm:
        # where each ray meets the top face's plane, object frame
        do = d @ Ti[:3, :3].T
        s = -Ti[2, 3] / np.where(np.abs(do[:, 2]) > 1e-12, do[:, 2], 1e-12)
        xp, yp = Ti[0, 3] + s * do[:, 0], Ti[1, 3] + s * do[:, 1]
        inb = ((mid != 0) & (xp < -CUBE / 2) & (xp > -CUBE / 2 - band_mm / 1000.0)
               & (np.abs(yp) < CUBE / 2))
        col[inb] = (35, 35, 35)
    if plain:
        col[:] = GREY
    bgr = col.reshape(h, ss, w, ss, 3).mean(axis=(1, 3))
    bgr = bgr + np.random.default_rng(seed).normal(0.0, 2.0, bgr.shape)
    bgr = np.clip(bgr, 0, 255).astype(np.uint8)

    # Depth at pixel centres; the stand gives none, nor does plain plastic.
    us, vs = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    z, mid, fid, d = trace(meshes, table, us, vs)
    depth = np.where(np.isfinite(z) & (mid != 1) & (mid != -2), z, 0.0)
    if hole is not None:
        Po = obj(np.where(np.isfinite(z), z, 0.0), d)
        depth[(mid == 0) & (fid <= 1) & hole(Po[:, 0], Po[:, 1])] = 0.0
    zz = np.where(np.isfinite(z), z, 0.0)
    for k, (_, Tx, _, hx) in enumerate(extra):
        if hx is not None:
            Tix = np.linalg.inv(Tx)
            Px = (d * zz[:, None]) @ Tix[:3, :3].T + Tix[:3, 3]
            depth[(mid == 2 + k) & (fid <= 1) & hx(Px[:, 0], Px[:, 1])] = 0.0
    return depth.reshape(h, w), bgr


def _plain_plastic(x, y):
    """No depth along most of the +y edge and the lower -x edge, as on the
    real cube's bare yellow (the D405 needs texture)."""
    return ((y > 0.018) & (np.abs(x) < 0.02)) | ((x < -0.020) & (y < 0.0))


def _islands(x, y):
    """No depth along a cross through the top face, 8 mm wide: plain plastic
    and paper as the D405 sees them up close (09-29, 70-110 mm), leaving the
    face's depth in four islands of a fifth of it each."""
    return (np.abs(x) < 0.004) | (np.abs(y) < 0.004)


def test_a_face_broken_into_islands_is_still_the_part():
    """3A step 3: the size check measures the part's extent, not the one
    island nearest the prior, nor how much of it returned depth."""
    T = pose([0.004, -0.003, 0.100], yaw=12.0, tilt=2.0)
    depth, bgr = colour_scene(T, hole=_islands)
    T_est, valid, q = ObjectPoseEstimator(PART, depth_fallback=False).process(
        depth, bgr, K, None, perturbed(T, 0.6, 0.4), prior_err_m=0.0015)
    assert valid, q['reason']
    assert 0.9 < q['size_ratio'] < 1.1
    check(T_est, T, mm=0.5, deg=0.5)


def _judged(**q):
    base = {'n_pts': 500, 'rms_mm': 0.5, 'inlier_frac': 0.97, 'outline_frac': 0.9,
            'outline_frac_all': 0.9, 'rim_in_view': 1.0, 'agree_mm': 0.3, 'agree_deg': 0.2,
            'weak_dof': []}
    base.update(q)
    return ObjectPoseEstimator(PART)._judge(base)


def test_the_outline_gate_counts_only_points_in_the_image_when_most_are_in_view():
    """3A step 3 on 10-02: a cube partly out of view, or a hand over one
    edge, read as 'no colour outline'. What passed before still passes."""
    assert _judged(outline_frac=0.65, outline_frac_all=0.62, rim_in_view=0.5)[0]   # as before
    assert _judged(outline_frac=0.7, outline_frac_all=0.56, rim_in_view=0.8)[0]    # in-image
    assert not _judged(outline_frac=0.7, outline_frac_all=0.53, rim_in_view=0.75)[0]
    # one edge hidden, a good surface fit
    assert _judged(outline_frac=0.56, outline_frac_all=0.56, inlier_frac=0.96)[0]
    assert not _judged(outline_frac=0.56, outline_frac_all=0.56, inlier_frac=0.94)[0]
    assert not _judged(outline_frac=0.5, outline_frac_all=0.5, inlier_frac=0.99)[0]


def _fragment_case():
    """A 100 x 100 px label image for _fragments, with the prior at the
    object frame (R0 = I, t0 = 0), so a pixel's point is in the object frame:
    label 1 (the anchor) a plate 6 mm over the top face, labels 2 and 3
    islands on the face, all inside the prior's silhouette."""
    lab = np.zeros((100, 100), np.int32)
    lab[45:55, 30:70] = 1
    lab[25:40, 25:45] = 2
    lab[60:75, 55:75] = 3
    prior = np.zeros((100, 100), bool)
    prior[20:80, 20:80] = True

    def points(sel):
        ys, xs = np.nonzero(sel)                 # row-major, as _fragments reads lab[sel]
        z = np.where(lab[ys, xs] == 1, 0.006, 0.0)
        return np.column_stack([(xs - 50) * 0.0005, (ys - 50) * 0.0005, z])
    return lab, prior, points


def test_an_anchor_off_the_part_merges_nothing():
    """Review 2026-09-30: a finger over the face, picked as the anchor, must
    not pull the part's islands into its region."""
    lab, prior, points = _fragment_case()
    args = (lab, 4, 1, prior, points, np.eye(3), np.zeros(3), 0.0015, 400.0, 0.1)
    region = ObjectPoseEstimator(PART)._fragments(*args)
    assert np.array_equal(region, lab == 1)
    loose = ObjectPoseEstimator(PART, fragment_on_part_m=0.05)._fragments(*args)
    assert loose[lab == 2].all() and loose[lab == 3].all()     # what the gate stops


def test_an_anchor_on_the_part_merges_its_islands():
    lab, prior, points = _fragment_case()

    def on_face(sel):
        p = points(sel)
        p[:, 2] = 0.0
        return p
    region = ObjectPoseEstimator(PART)._fragments(
        lab, 4, 1, prior, on_face, np.eye(3), np.zeros(3), 0.0015, 400.0, 0.1)
    assert region[lab > 0].all()


def _half_face(cut):
    return lambda x, y: x > cut


def test_half_a_face_is_not_the_part():
    """Review 2026-09-30: the size lower bound is 0.7 of the prior's extent;
    at the old 0.5 half a face (58 % here) passed."""
    T = pose([0.004, -0.003, 0.100], yaw=12.0, tilt=2.0)
    depth, bgr = colour_scene(T, hole=_half_face(0.005), patch=False)
    prior = perturbed(T, 0.6, 0.4)
    _, valid, q = ObjectPoseEstimator(PART, depth_fallback=False).process(
        depth, bgr, K, None, prior, prior_err_m=0.0015)
    assert not valid and q['reason'].startswith('size')
    assert 0.5 < q['size_ratio'] < 0.7
    _, valid, _ = ObjectPoseEstimator(PART, depth_fallback=False, size_ratio=(0.5, 1.6)).process(
        depth, bgr, K, None, prior, prior_err_m=0.0015)
    assert valid                                               # what the bound stops


def _flush_neighbour(T):
    nb_pose = T.copy()
    nb_pose[:3, 3] = T[:3, 3] + T[:3, 1] * CUBE
    return colour_scene(T, hole=_islands, extra=[([CUBE] * 3, nb_pose, YELLOW, _islands)])


def test_a_prior_straddling_a_flush_neighbour_is_refused():
    """Review 2026-09-30: with a second cube flush against the first and the
    prior 20 mm towards it, the islands of both lie inside the prior's
    silhouette. The top plane running on past the silhouette (the spill
    ring) stops the merge; without it the pose came out 23 mm off."""
    T = pose([0.004, -0.003, 0.100], yaw=12.0, tilt=2.0)
    depth, bgr = _flush_neighbour(T)
    prior = T.copy()
    prior[:3, 3] = T[:3, 3] + T[:3, 1] * 0.020
    _, valid, q = ObjectPoseEstimator(PART, depth_fallback=True).process(
        depth, bgr, K, None, prior, prior_err_m=0.0015)
    assert not valid, q
    no_ring = ObjectPoseEstimator(PART, depth_fallback=True, fragment_spill_max=1.01)
    T_bad, valid, _ = no_ring.process(depth, bgr, K, None, prior, prior_err_m=0.0015)
    assert valid and np.linalg.norm(pose_error(T_bad, T, 4)[0]) > 0.015   # what the ring stops


def test_colour_edges_take_over_where_plain_plastic_has_no_depth():
    for T in (pose([0.006, -0.004, 0.150], yaw=12.0, tilt=2.0),
              pose([0.004, 0.010, 0.280], yaw=-20.0, tilt=3.0)):
        depth, bgr = colour_scene(T, hole=_plain_plastic)
        est = ObjectPoseEstimator(PART)
        prior = perturbed(T, 4.0, 2.0)
        T_d, _, _ = est.process(depth, None, K, None, prior)
        T_c, valid, q = est.process(depth, bgr, K, None, prior)
        assert valid and q['edge_source'] == 'colour', q['reason']
        check(T_c, T, mm=0.15, deg=0.1)
        err_d = np.linalg.norm(pose_error(T_d, T, 4)[0])
        err_c = np.linalg.norm(pose_error(T_c, T, 4)[0])
        assert err_c <= err_d + 1e-5


def test_the_outermost_edge_wins_over_a_patch_near_the_outline():
    """The blue patch's edge 2 mm inside the +x edge is the stronger edge;
    the silhouette is the outer one."""
    T = pose([0.0, 0.0, 0.150], yaw=0.0)
    depth, bgr = colour_scene(T, patch=True)
    T_c, valid, q = ObjectPoseEstimator(PART).process(depth, bgr, K, None,
                                                      perturbed(T, 3.0, 1.5))
    assert valid and q['edge_source'] == 'colour', q['reason']
    check(T_c, T, mm=0.15, deg=0.1)


def test_no_colour_contrast_falls_back_to_the_depth_outline():
    T = pose([0.003, 0.0, 0.150], yaw=8.0)
    depth, bgr = colour_scene(T, plain=True)
    T_c, valid, q = ObjectPoseEstimator(PART).process(depth, bgr, K, None,
                                                      perturbed(T, 3.0, 1.5))
    assert valid and q['edge_source'] == 'depth', q['reason']
    check(T_c, T)


def test_without_the_depth_fallback_a_missing_colour_outline_is_reported():
    T = pose([0.003, 0.0, 0.150], yaw=8.0)
    depth, bgr = colour_scene(T, plain=True)
    est = ObjectPoseEstimator(PART, depth_fallback=False)
    T_c, valid, q = est.process(depth, bgr, K, None, perturbed(T, 3.0, 1.5))
    assert T_c is None and not valid and q['reason'].startswith('no colour outline')


def test_a_dark_band_beside_the_outline_is_not_the_part():
    """A dark band just outside the -x edge (the stand and its shadow, seen
    from further up) and the blue patch just inside the +x edge: the
    silhouette is the outermost edge whose inner side looks like the part."""
    for T in (pose([0.0, 0.0, 0.150], yaw=0.0), pose([0.004, -0.006, 0.280], yaw=6.0)):
        depth, bgr = colour_scene(T, patch=True, band_mm=2.5)
        T_c, valid, q = ObjectPoseEstimator(PART).process(depth, bgr, K, None,
                                                          perturbed(T, 3.0, 1.5))
        assert valid and q['edge_source'] == 'colour', q['reason']
        check(T_c, T, mm=0.15, deg=0.1)


def test_a_narrow_face_in_its_own_colour_is_still_the_part():
    """A side face seen at 60-75 deg is a strip a few px wide in its own
    colour (the cube's blue tape, 10-02): the deep samples are mostly the
    top face's yellow, whose spread fills the commonest bins. Its colour
    counts where the depth confirms it at the outline, not otherwise."""
    rng = np.random.default_rng(0)
    yellow = np.array(YELLOW, float) + rng.normal(0.0, 12.0, (900, 3))
    blue = np.array(BLUE, float) + rng.normal(0.0, 3.0, (100, 3))
    pool = np.vstack([yellow, blue])
    inner = np.array([[BLUE]], float)                   # one candidate, blue on its inner side
    pk = np.ones((1, 1), bool)
    like = ObjectPoseEstimator._part_like
    assert not like(pool, inner, pk)[0, 0]              # outranked by yellow's spread
    assert like(pool, inner, pk, at_rim=blue)[0, 0]
    assert not like(pool, inner, pk, at_rim=yellow)[0, 0]


# ---------------------------------------------------------------- acquisition (Phase 6)

PART_BOX = dict(PART, box_m=np.array([CUBE] * 3))


def test_acquire_finds_the_cube_with_no_prior_at_any_range():
    for z, yaw, hole in ((0.10, 12.0, None), (0.25, 40.0, None), (0.08, -20.0, _islands)):
        T = pose([0.01, -0.02, z], yaw=yaw, tilt=2.0)
        depth, bgr = colour_scene(T, hole=hole)
        T_est, valid, q = ObjectPoseEstimator(PART_BOX, depth_fallback=False).acquire(
            depth, bgr, K, None)
        assert valid, q['reason']
        check(T_est, T, mm=0.2, deg=0.2)


def test_acquire_with_nothing_there_or_a_finger_over_the_face_finds_nothing_false():
    T = pose([0.01, -0.02, 0.15], yaw=12.0, tilt=2.0)
    depth, bgr = colour_scene(T)
    empty = np.where(depth < 0.2, 0.0, depth)                # only the table left
    _, valid, q = ObjectPoseEstimator(PART_BOX).acquire(empty, bgr, K, None)
    assert not valid and 'nothing standing' in q['reason']
    over = T.copy()
    over[:3, 3] += T[:3, 2] * 0.012
    depth, bgr = colour_scene(T, extra=[([0.018, 0.07, 0.01], over, (60, 60, 60), None)])
    T_est, valid, q = ObjectPoseEstimator(PART_BOX, depth_fallback=False).acquire(
        depth, bgr, K, None)
    assert not valid or np.linalg.norm(pose_error(T_est, T, 4)[0]) < 0.001


def test_acquire_needs_a_box_part():
    _, valid, q = ObjectPoseEstimator(PART).acquire(np.ones((480, 640)), None, K, None)
    assert not valid and 'box parts only' in q['reason']


def test_a_colour_at_the_outline_is_kept_as_its_own_bin_without_noise():
    """Review 2026-10-02: the box-summed peak is flat-topped, and the kept
    bin sat one off the tape's own colour, so an exact colour was refused."""
    pool = np.tile([[40.0, 200.0, 215.0]], (900, 1))                 # the yellow top
    blue = np.array([200.0, 120.0, 60.0])
    inner, pk = np.tile(blue, (5, 4, 1)), np.ones((5, 4), bool)
    ok = ObjectPoseEstimator._part_like(pool, inner, pk, at_rim=np.tile(blue, (60, 1)))
    assert ok.all()


def test_the_patch_inside_the_edge_stays_out_of_the_outline_at_range():
    """Review 2026-10-02: a fixed 3 px rim band reached the blue patch 2 mm
    inside the edge from ~0.5 m; it is about 1.5 mm at the part's distance."""
    for z in (0.40, 0.60):
        T = pose([0.004, -0.003, z], yaw=8.0, tilt=2.0)
        depth, bgr = colour_scene(T)
        T_est, valid, q = ObjectPoseEstimator(PART, depth_fallback=False).process(
            depth, bgr, K, None, perturbed(T, 0.6, 0.4), prior_err_m=0.0015)
        assert valid, q['reason']
        check(T_est, T, mm=0.1, deg=0.2)
