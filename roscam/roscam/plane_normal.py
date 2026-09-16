#!/usr/bin/env python3
"""Unambiguous plane normal from depth, used to resolve the ArUco pose flip.

WHY THIS EXISTS
===============
A small, nearly fronto-parallel square marker has TWO IPPE pose solutions
that project almost identically: same tilt magnitude, mirrored tilt
direction. `cv2.solvePnP(..., SOLVEPNP_IPPE_SQUARE)` returns only one of
them, and which one wins is decided by near-identical reprojection errors,
so it flips frame to frame. Measured on this cell: the in-plane direction
of the marker normal flipped >120 deg on 62% of consecutive frames while
its magnitude held ~8 deg. Any orientation control loop fed that signal
commands alternating corrections and cannot converge.

Depth does not share the ambiguity: a plane fitted to the depth samples on
the marker has exactly one normal. It is not used as the pose itself (the
ArUco corner fit is far better for position and in-plane rotation) - only
to pick WHICH of the two IPPE solutions is real.

Pure numpy/cv2, no ROS, so it unit-tests standalone.
"""

import cv2
import numpy as np


def fit_plane(points):
    """Least-squares plane through (N,3) points.

    Returns (normal unit (3,), centroid (3,), rms distance) or None if
    degenerate. The normal sign is arbitrary here; callers orient it.
    """
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[0] < 3:
        return None
    c = p.mean(axis=0)
    q = p - c
    try:
        _, s, vt = np.linalg.svd(q, full_matrices=False)
    except np.linalg.LinAlgError:
        return None
    if s[1] < 1e-12:           # collinear: no unique plane
        return None
    n = vt[2]
    nn = np.linalg.norm(n)
    if nn < 1e-12:
        return None
    n = n / nn
    rms = float(np.sqrt(np.mean((q @ n) ** 2)))
    return n, c, rms


def fit_plane_robust(points, trims=3, keep_frac=0.8, sigma=2.5,
                     min_points=12):
    """fit_plane with trimming passes that always make progress.

    Depth patches pick up the card edge and the surface behind it, and a
    plain least-squares fit tilts toward those outliers - which is exactly
    the quantity we are trying to measure.

    Trimming is by QUANTILE, not by sigma. A coherent outlier block (a slab
    of background 30 mm behind the card, say) inflates the RMS enough that a
    2.5*rms gate keeps every point and the refit never moves. Dropping the
    worst (1-keep_frac) each pass is guaranteed to converge on the dominant
    plane. A final sigma pass then cleans up the remaining spread.
    """
    p = np.asarray(points, dtype=float)
    res = fit_plane(p)
    if res is None:
        return None
    for _ in range(max(0, int(trims))):
        n, c, _ = res
        if len(p) <= min_points:
            break
        d = np.abs((p - c) @ n)
        thr = np.quantile(d, keep_frac)
        keep = d <= thr
        if keep.sum() < max(min_points, 3):
            break
        nxt = fit_plane(p[keep])
        if nxt is None:
            break
        p, res = p[keep], nxt
    # final sigma pass on the (now mostly clean) support
    n, c, rms = res
    if rms > 0 and len(p) > min_points:
        d = np.abs((p - c) @ n)
        keep = d <= sigma * rms
        if keep.sum() >= max(min_points, 3) and not keep.all():
            nxt = fit_plane(p[keep])
            if nxt is not None:
                p, res = p[keep], nxt
    n, c, rms = res
    return n, c, rms, len(p)


def quad_mask(shape, corners_px, scale=1.0):
    """Filled mask of the marker quad, optionally scaled about its centroid.

    A 21 mm marker at 100 mm covers only ~75 px, so scale>1 borrows
    surrounding co-planar card to condition the fit. Keep it modest or the
    patch runs off the card onto the background.
    """
    pts = np.asarray(corners_px, dtype=float).reshape(-1, 2)
    if len(pts) < 3:
        return None
    if scale != 1.0:
        c = pts.mean(axis=0)
        pts = c + (pts - c) * float(scale)
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.round(pts).astype(np.int32), 1)
    return mask.astype(bool)


def marker_plane_normal(depth_m, corners_px, fx, fy, cx, cy,
                        scale=1.6, min_points=40, max_rms_m=0.004,
                        stride=1, max_points=600):
    """Normal of the plane the marker lies on, from depth.

    Returns dict(normal, centroid, rms_m, n_points) with the normal a unit
    vector in the camera optical frame pointing TOWARD the camera (nz < 0,
    matching the convention of an ArUco marker facing the lens), or None if
    the patch is too sparse or too non-planar to trust.
    """
    if depth_m is None:
        return None
    d = np.asarray(depth_m)
    if d.ndim != 2:
        return None
    pts_px = np.asarray(corners_px, dtype=float).reshape(-1, 2)
    if len(pts_px) < 3:
        return None

    # Work inside the quad's bounding box only. Masking the whole frame and
    # scanning it costs ~12 ms at 640x480 for a ~120 px patch - which at
    # 90 fps is most of the frame budget. Cropping first makes it ~0.3 ms.
    ctr = pts_px.mean(axis=0)
    scaled = ctr + (pts_px - ctr) * float(scale)
    h, w = d.shape
    x0 = max(0, int(np.floor(scaled[:, 0].min())))
    y0 = max(0, int(np.floor(scaled[:, 1].min())))
    x1 = min(w, int(np.ceil(scaled[:, 0].max())) + 1)
    y1 = min(h, int(np.ceil(scaled[:, 1].max())) + 1)
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None

    sub = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
    cv2.fillConvexPoly(sub, np.round(scaled - [x0, y0]).astype(np.int32), 1)
    mask = sub.astype(bool)
    if stride > 1:
        keep = np.zeros_like(mask)
        keep[::stride, ::stride] = True
        mask &= keep
    vs, us = np.nonzero(mask)
    if len(us) < min_points:
        return None
    z = d[vs + y0, us + x0].astype(float)
    us = us + x0                      # back to full-frame pixel coords
    vs = vs + y0
    ok = np.isfinite(z) & (z > 0.0)
    if ok.sum() < min_points:
        return None
    z, us, vs = z[ok], us[ok], vs[ok]
    # Cap the support. A plane normal is over-determined by a few hundred
    # samples at 0.1 mm depth noise, and the SVD passes are O(N): without a
    # cap the cost grows as the marker fills the frame on approach.
    if max_points and len(z) > max_points:
        sel = np.linspace(0, len(z) - 1, int(max_points)).astype(int)
        z, us, vs = z[sel], us[sel], vs[sel]
    pts = np.column_stack([(us - cx) * z / fx, (vs - cy) * z / fy, z])

    res = fit_plane_robust(pts, min_points=min(min_points, 12))
    if res is None:
        return None
    n, c, rms, kept = res
    if kept < min_points or rms > max_rms_m:
        return None
    if n[2] > 0:                 # face the camera
        n = -n
    return {'normal': n, 'centroid': c, 'rms_m': float(rms),
            'n_points': int(kept)}


def fuse_orientation(R_aruco, normal):
    """Take the plane normal from depth, the in-plane rotation from ArUco.

    Each sensor is used for what it is actually good at on this cell:

      ArUco corners  position      7-10 micron
                     in-plane rot  0.18 deg   <- excellent
                     out-of-plane  biased ~2.6 deg high, and ambiguous
      depth plane    out-of-plane  ~0.18 deg at 0.11 mm fit rms, unbiased

    Driving a control loop on the ArUco out-of-plane angle bottoms out on
    its bias (observed: a tilt floor that would not go below ~7 deg). This
    rebuilds the rotation with the depth normal as the marker Z axis while
    preserving ArUco's in-plane orientation about it.

    Returns a proper rotation matrix (columns = marker X, Y, Z in the camera
    frame) or None if degenerate.
    """
    R = np.asarray(R_aruco, dtype=float)
    n = np.asarray(normal, dtype=float)
    nn = np.linalg.norm(n)
    if R.shape != (3, 3) or nn < 1e-12:
        return None
    n = n / nn
    if float(n @ R[:, 2]) < 0.0:       # same hemisphere as the ArUco normal
        n = -n
    x = R[:, 0] - float(R[:, 0] @ n) * n      # project X into the plane
    xn = np.linalg.norm(x)
    if xn < 1e-6:                      # X almost parallel to the normal
        return None
    x = x / xn
    y = np.cross(n, x)
    yn = np.linalg.norm(y)
    if yn < 1e-9:
        return None
    return np.column_stack([x, y / yn, n])


def inplane_angle(R_cam_marker):
    """Angle of the marker X axis in the image, degrees.

    This is rotation about the optical axis - the 6th DOF, and the one an
    alignment loop that only nulls position and tilt leaves untouched.

    Optical convention: +X is right on screen, +Y is DOWN. So 0 deg puts the
    marker X axis (the RED axis OpenCV draws) pointing right, +90 straight
    down, -90 straight up.

    Measured on this cell at 0.01 deg std - by far the best-conditioned
    quantity the marker gives, because it comes from the corner positions
    rather than from foreshortening.
    """
    R = np.asarray(R_cam_marker, dtype=float)
    if R.shape != (3, 3):
        return None
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def wrap_deg(a):
    """Wrap to (-180, 180]."""
    w = (float(a) + 180.0) % 360.0 - 180.0
    # the modulo lands exactly 180 on -180; keep the documented half-open end
    return 180.0 if w == -180.0 else w


def inplane_correction(current_deg, target_deg, limit_deg=None):
    """Rotation about the optical axis that moves current -> target.

    Returns degrees to rotate the CAMERA about its own Z, clamped to
    limit_deg. Takes the short way round, so a target of +90 from +83.67 is
    a 6.33 deg move, not 353.67.
    """
    err = wrap_deg(float(current_deg) - float(target_deg))
    if limit_deg is not None:
        lim = abs(float(limit_deg))
        err = max(-lim, min(lim, err))
    return err


def disambiguate_by_normal(candidate_normals, reference_normal):
    """Pick the candidate whose normal best agrees with the reference.

    Returns (index, agreement) where agreement is the dot product of the
    winner with the reference (1.0 = identical). Compared WITHOUT taking
    absolute value: the whole point is that the two IPPE solutions differ
    in the SIGN of their in-plane tilt, and abs() would discard exactly
    that information.
    """
    ref = np.asarray(reference_normal, dtype=float)
    rn = np.linalg.norm(ref)
    if rn < 1e-12 or not candidate_normals:
        return None
    ref = ref / rn
    dots = []
    for n in candidate_normals:
        n = np.asarray(n, dtype=float)
        nn = np.linalg.norm(n)
        dots.append(-np.inf if nn < 1e-12 else float((n / nn) @ ref))
    i = int(np.argmax(dots))
    return i, dots[i]
