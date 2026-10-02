#!/usr/bin/env python3
"""Compare the D405's depth quality across capture settings (PERCEPTION_PLAN
Phase 0), from frame_recorder sessions.

    python3 tools/fr3/vision/depth_quality.py <session_dir> [<session_dir> ...]
    python3 tools/fr3/vision/depth_quality.py $FR3_LOG_DIR/<date>/vision --csv dq.csv
    python3 tools/fr3/vision/depth_quality.py <session_dir> --cube-mm 55

A path is a session (it holds session.json) or a directory of sessions.
Record one session per capture setting and distance, with the marker cube
static under the camera. The markdown table has one row per session and
distance bucket (80/100/150/200/300 mm +/- 15%, nearest wins; '-' outside
them); --csv also writes the rows unrounded.

Every frame with depth and an /aruco/pose_raw pose is measured against the
marker (ground truth in this phase):

  fill      % of pixels with depth (> 0) inside the marker quad scaled x1.6
            about its centre: the ring cam_pub fits its plane to.
  rms       depth scatter about the top-face plane in that ring, mm. The
            plane is fit_plane_robust's; the rms is over the points within
            3 sigma of it (sigma = 1.4826 * median |residual|). The rms
            fit_plane_robust returns is of its trimmed core and reads ~0.4x
            the real noise, so it is not the figure reported.
  flying    at the silhouette, % of the pixels with depth in the band
            +/- 2 mm (in the marker plane, projected) around the cube's top
            outline: a square of --cube-mm centred on the marker and aligned
            with it. A pixel is flying when its point is > 3 mm off the top
            plane AND > 3 mm above the table plane: on neither surface. The
            table is the dominant plane (fit_plane_robust) of the points
            > 20 mm below the top plane in a window twice the cube's size.
            It counts as found when those points cover >= 10% of the window
            (a ring of flying pixels alone would not). Otherwise a stand-in
            23 mm below the top, parallel to it, is used: points 3-20 mm
            below the top plane (or > 3 mm above it) count. 'table %' is
            how often the real table was found. Frames whose camera is not
            over the top face (a side face in view) are left out, since a
            side face would count here. The outline assumes the sticker is
            centred and square on the top face: a sticker d mm off centre
            moves the band d mm.
  bias      (median depth in 5x5 px at the marker centre - tvec z) / tvec z,
            %. tvec scales with the marker size in session.json, so a wrong
            marker size reads as a depth-scale bias here too. The median
            moves in whole depth units (D405: 0.1 mm = 0.1% at 100 mm).
  dist      marker tvec z, mm.

Per row: the capture settings, frames used (depth and pose) / frames in
the session, medians over the frames (fill also its 10th percentile), the
raw pose rate (over consecutive /aruco/pose_raw stamps within the bucket; a
missed detection or a frame the recorder dropped reads as a lower rate) and
the raw pose's position sigma (3-D rms about its mean, mm; only meaningful
for a static session).

Geometry: corners are projected with the recorded intrinsics and the
distortion coefficients taken as OpenCV's, as cam_pub solved the pose;
depth pixels are back-projected with the same model. With all-zero
coefficients (a rectified stream) both are plain pinhole.

No ROS: reads the recordings only (roscam/roscam/frame_recorder.py).
"""

import argparse
import csv
import pathlib
import sys

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / 'roscam'))
from roscam.frame_recorder import load_session  # noqa: E402
from roscam.plane_normal import fit_plane_robust, quad_mask  # noqa: E402

RAW_TOPIC = '/aruco/pose_raw'
RING_SCALE = 1.6            # cam_pub's tilt_depth_scale
BAND_M = 0.002              # silhouette band half-width, in the marker plane
OFF_PLANE_M = 0.003         # flying: this far off the top AND above the table
TABLE_BELOW_M = 0.020       # table candidates lie this far below the top
TABLE_MIN_POINTS = 100
TABLE_MIN_SHARE = 0.10      # ... covering this share of the window's pixels
TABLE_MAX_POINTS = 5000     # plenty for a plane; keeps the fit cheap up close
BIAS_WINDOW_PX = 5
MIN_PX = 20
BUCKETS_MM = (80, 100, 150, 200, 300)
BUCKET_TOL = 0.15

# (row key, markdown label, markdown format; None = as is)
COLUMNS = (('session', 'session', None), ('res', 'res', None), ('preset', 'preset', None),
           ('spatial', 'spatial', None), ('exposure', 'exposure', None),
           ('bucket_mm', 'bucket mm', '{:.0f}'), ('frames', 'frames', None),
           ('dist_mm', 'dist mm', '{:.1f}'), ('fill_pct', 'fill %', '{:.1f}'),
           ('fill_p10_pct', 'fill p10 %', '{:.1f}'), ('rms_mm', 'rms mm', '{:.3f}'),
           ('flying_pct', 'flying %', '{:.2f}'), ('table_pct', 'table %', '{:.0f}'),
           ('bias_pct', 'bias %', '{:+.2f}'), ('raw_hz', 'raw Hz', '{:.1f}'),
           ('sigma_mm', 'sigma mm', '{:.3f}'))


def q2R(q):
    """Rotation matrix of a quaternion (x, y, z, w)."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def square(R, t, half):
    """(4, 3) camera-frame corners of a square centred on the marker, in its
    plane and aligned with it, in cam_pub's corner order."""
    local = np.array([[-half, half, 0.0], [half, half, 0.0],
                      [half, -half, 0.0], [-half, -half, 0.0]])
    return local @ R.T + t


def project(points, K, dist):
    px, _ = cv2.projectPoints(np.asarray(points, dtype=float).reshape(-1, 1, 3),
                              np.zeros(3), np.zeros(3), K, dist)
    return px.reshape(-1, 2)


def region(shape, points, K, dist, scale=1.0):
    """Pixel mask of a camera-frame polygon; None if it reaches behind the
    camera."""
    if np.any(points[:, 2] <= 0):
        return None
    return quad_mask(shape, project(points, K, dist), scale=scale)


def back_project(mask, depth_m, K, dist, max_points=None):
    """(N, 3) optical-frame points of the pixels in mask."""
    vs, us = np.nonzero(mask)
    if max_points and len(us) > max_points:
        sel = np.linspace(0, len(us) - 1, int(max_points)).astype(int)
        vs, us = vs[sel], us[sel]
    z = depth_m[vs, us].astype(float)
    xy = cv2.undistortPoints(np.column_stack([us, vs]).astype(float).reshape(-1, 1, 2),
                             K, dist).reshape(-1, 2)
    return np.column_stack([xy[:, 0] * z, xy[:, 1] * z, z])


def plane(points):
    """fit_plane_robust as (normal toward the camera, centroid), or None."""
    res = fit_plane_robust(points)
    if res is None:
        return None
    n, c = res[0], res[1]
    return (-n if n[2] > 0 else n), c


def clipped_rms(residuals):
    """rms of the residuals within 3 sigma, sigma from the median |r|."""
    r = np.abs(np.asarray(residuals, dtype=float))
    gate = 3.0 * 1.4826 * np.median(r)
    return float(np.sqrt(np.mean(r[r <= gate] ** 2)))


def frame_metrics(depth_m, pose, K, dist, marker_m, cube_m):
    """The per-frame figures (module docstring); None where not measurable."""
    R, t = q2R(pose['q']), np.asarray(pose['t'], dtype=float)
    m = {'dist_mm': t[2] * 1e3, 'fill': None, 'rms_mm': None, 'flying': None,
         'table': None, 'bias_pct': None}
    shape = depth_m.shape
    valid = depth_m > 0

    if t[2] <= 0:
        return m
    u, v = np.round(project(t, K, dist)[0]).astype(int)
    h = BIAS_WINDOW_PX // 2
    win = depth_m[max(0, v - h):max(0, v + h + 1), max(0, u - h):max(0, u + h + 1)]
    win = win[win > 0]
    if win.size:
        m['bias_pct'] = float((np.median(win) - t[2]) / t[2] * 100.0)

    ring = region(shape, square(R, t, marker_m / 2), K, dist, scale=RING_SCALE)
    if ring is None or ring.sum() < MIN_PX:
        return m
    m['fill'] = float(valid[ring].mean())
    if (ring & valid).sum() < MIN_PX:
        return m
    pts = back_project(ring & valid, depth_m, K, dist)
    top = plane(pts)
    if top is None:
        return m
    n, c = top
    m['rms_mm'] = clipped_rms((pts - c) @ n) * 1e3

    half = cube_m / 2
    cam = -R.T @ t                              # camera centre, marker coordinates
    if max(abs(cam[0]), abs(cam[1])) > half:
        return m                                # a side face is in view
    outer = region(shape, square(R, t, half + BAND_M), K, dist)
    inner = region(shape, square(R, t, half - BAND_M), K, dist)
    window = region(shape, square(R, t, cube_m), K, dist)
    if outer is None or inner is None or window is None:
        return m
    band = outer & ~inner & valid
    if band.sum() < MIN_PX:
        return m
    seen = window & valid
    cand = back_project(seen, depth_m, K, dist, TABLE_MAX_POINTS)
    below = (cand - c) @ n < -TABLE_BELOW_M
    # a table covers a real share of the window, not just a silhouette ring
    found = (below.sum() >= TABLE_MIN_POINTS
             and below.mean() * seen.sum() >= TABLE_MIN_SHARE * window.sum())
    table = plane(cand[below]) if found else None
    m['table'] = table is not None
    nt, ct = table if table is not None else (n, c - (TABLE_BELOW_M + OFF_PLANE_M) * n)
    p = back_project(band, depth_m, K, dist)
    flying = (np.abs((p - c) @ n) > OFF_PLANE_M) & ((p - ct) @ nt > OFF_PLANE_M)
    m['flying'] = float(flying.mean())
    return m


def bucket(z_mm):
    """The nearest distance bucket within +/- BUCKET_TOL, or None."""
    near = [b for b in BUCKETS_MM if abs(z_mm - b) <= BUCKET_TOL * b]
    return min(near, key=lambda b: abs(z_mm - b) / b) if near else None


def _med(values, scale=1.0, pct=50):
    return float(np.percentile(values, pct)) * scale if values else None


def session_rows(session_dir, cube_m):
    """(rows, meta) for one session: one row per distance bucket."""
    meta, frames = load_session(session_dir)
    intr = meta.get('intrinsics')
    if not intr:
        raise SystemExit(f'{session_dir}: session.json has no intrinsics')
    K = np.array([[intr['fx'], 0.0, intr['cx']], [0.0, intr['fy'], intr['cy']],
                  [0.0, 0.0, 1.0]])
    dist = np.asarray(intr.get('coeffs') or [0.0] * 5, dtype=float)
    marker_m = float(meta['marker_size_m'])
    groups, total, prev = {}, 0, None           # prev: (bucket, stamp) of the last raw pose
    for rec in frames:
        total += 1
        pose = rec.line.get('poses', {}).get(RAW_TOPIC)
        if pose is None:
            continue
        b = bucket(pose['t'][2] * 1e3)
        g = groups.setdefault(b, {'t': [], 'dt': [], 'm': []})
        g['t'].append(pose['t'])
        if prev is not None and prev[0] == b:
            g['dt'].append(pose['stamp'] - prev[1])
        prev = (b, pose['stamp'])
        if rec.depth_m is not None:
            g['m'].append(frame_metrics(rec.depth_m, pose, K, dist, marker_m, cube_m))

    cap = meta.get('capture') or {}
    rows = []
    for b in sorted(groups, key=lambda k: (k is None, k or 0)):
        g = groups[b]

        def col(key):
            return [x[key] for x in g['m'] if x[key] is not None]
        tmm = np.asarray(g['t'], dtype=float) * 1e3
        span = sum(g['dt'])
        rows.append({
            'session': pathlib.Path(session_dir).name,
            'res': f"{cap.get('width', '?')}x{cap.get('height', '?')}",
            'preset': cap.get('preset', '?'), 'spatial': cap.get('spatial_filter', '?'),
            'exposure': cap.get('exposure_us', '?'), 'bucket_mm': b,
            'frames': f"{len(g['m'])}/{total}", 'dist_mm': float(np.median(tmm[:, 2])),
            'fill_pct': _med(col('fill'), 100.0), 'fill_p10_pct': _med(col('fill'), 100.0, 10),
            'rms_mm': _med(col('rms_mm')), 'flying_pct': _med(col('flying'), 100.0),
            'table_pct': float(np.mean(col('table'))) * 100.0 if col('table') else None,
            'bias_pct': _med(col('bias_pct')),
            'raw_hz': len(g['dt']) / span if span > 0 else None,
            'sigma_mm': (float(np.sqrt(tmm.var(axis=0, ddof=1).sum()))
                         if len(tmm) > 1 else None)})
    return rows, meta


def markdown(rows):
    def cell(row, key, fmt):
        v = row[key]
        return '-' if v is None else (fmt.format(v) if fmt else str(v))
    out = ['| ' + ' | '.join(label for _, label, _ in COLUMNS) + ' |',
           '|' + '---|' * len(COLUMNS)]
    out += ['| ' + ' | '.join(cell(r, k, f) for k, _, f in COLUMNS) + ' |' for r in rows]
    return '\n'.join(out)


def write_csv(rows, path):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=[k for k, _, _ in COLUMNS])
        w.writeheader()
        w.writerows(rows)


def session_dirs(paths):
    for p in map(pathlib.Path, paths):
        if (p / 'session.json').is_file():
            yield p
            continue
        found = sorted(s.parent for s in p.glob('*/session.json'))
        if not found:
            raise SystemExit(f'{p}: no session.json here or one level down')
        yield from found


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+', help='session directories, or directories of them')
    ap.add_argument('--cube-mm', type=float, default=55.0,
                    help='side of the cube whose top face carries the marker (default 55)')
    ap.add_argument('--csv', help='also write the rows, unrounded, to this CSV file')
    args = ap.parse_args(argv)
    rows, notes = [], []
    for s in session_dirs(args.sessions):
        got, meta = session_rows(s, args.cube_mm / 1000.0)
        rows += got
        if meta.get('frames_dropped'):
            notes.append(f'{s.name}: the recorder dropped {meta["frames_dropped"]} frames '
                         '(raw Hz reads low)')
        if not got:
            notes.append(f'{s.name}: no frame with an {RAW_TOPIC} pose')
    print(markdown(rows))
    if notes:
        print('\n' + '\n'.join(f'- {n}' for n in notes))
    if args.csv:
        write_csv(rows, args.csv)
    return 0


if __name__ == '__main__':
    sys.exit(main())
