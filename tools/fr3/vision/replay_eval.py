#!/usr/bin/env python3
"""Score roscam.object_pose on recorded sessions against the marker
(PERCEPTION_PLAN Phase 2).

    python3 tools/fr3/vision/replay_eval.py <session_dir> [...] [--part cube55.yaml]
            [--levels 0/0,5/5,10/5,20/5] [--every 5] [--csv frames.csv]

Run from src/ with fr3_env.sh sourced (the reference needs rclpy). A path
is a frame_recorder session (session.json) or a directory of them.

Reference: every frame is replayed through the real cam_pub process_frame
(range_source 'depth', the marker pose as the cell now publishes it), then
composed with the part file's T_marker_object: T_ref = T_cam_marker o
T_marker_object. Until the sticker's offset is measured, T_marker_object
is identity and a constant difference is the sticker's offset, printed as
the implied T_marker_object; what matters then is whether the difference
changes with distance, and its scatter.

Priors: level 0 is the reference itself; the others move it by L mm in a
random direction and turn it by D deg about a random axis (seeded per
frame), to map the convergence basin. Level 0 runs on every frame, the
others on every --every-th.

Per session and distance bucket (the reference's camera distance), level 0:
  avail      % of frames with a valid estimate
  colour %   of the valid ones, how many took the outline from colour edges
             (the rest fell back to the depth outline)
  bias       mean of the estimate in the reference object frame: x, y, z mm,
             tilt deg, in-plane deg (modulo the part's symmetry)
  sd         the same, standard deviation: the marker's noise and the
             estimator's together
  table      angle between the estimate's Z and the support plane's normal
             (deg), and the top face's height above that plane (mm): checks
             that use neither the marker nor hand-eye. On this cell the cube
             sits on a stand, so the height is 55 mm + the stand.
  ms         compute p50 / p95, single-threaded BLAS as the live vision
             process runs (pin with taskset -c 12 to time one E-core)
Per perturbed level: valid %, and how often it reached the same answer as
the unperturbed prior on the same frame (0.5 mm, 0.5 deg): the basin,
independent of the unmeasured sticker offset.
"""

import argparse
import csv
import os
import pathlib
import sys

# One BLAS thread: the solves here are tiny, and OpenBLAS would otherwise
# spread each one over every core (measured: 17 cores busy) - both slower
# and not what the live vision process may use.
for _var in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_var, '1')

import numpy as np  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parents[2]
sys.path.insert(0, str(SRC / 'roscam'))
from roscam.frame_recorder import load_session  # noqa: E402
from roscam.object_pose import ObjectPoseEstimator, load_part, pose_error  # noqa: E402

BUCKETS_MM = (80, 100, 150, 200, 250, 300)


def session_dirs(paths):
    for p in map(pathlib.Path, paths):
        if (p / 'session.json').is_file():
            yield p
            continue
        found = sorted(s.parent for s in p.glob('*/session.json'))
        if not found:
            raise SystemExit(f'{p}: no session.json here or one level down')
        yield from found


def quat_matrix(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def marker_reference(meta, recs):
    """{frame i: T_cam_marker} from the real cam_pub, range_source depth."""
    from builtin_interfaces.msg import Time
    from rclpy.parameter import Parameter
    from std_msgs.msg import Header

    from roscam.cam_pub import ArucoPosePublisher
    node = ArucoPosePublisher(parameter_overrides=[
        Parameter('source', value='external'),
        Parameter('marker_id', value=int(meta.get('marker_id', 0))),
        Parameter('marker_size_m', value=float(meta['marker_size_m'])),
        Parameter('target_marker_id', value=-1), Parameter('range_source', value='depth'),
        Parameter('publish_debug_image', value=False)])
    it = meta['intrinsics']
    node.set_intrinsics(it['fx'], it['fy'], it['cx'], it['cy'], it['coeffs'])
    ref = {}
    try:
        for rec in recs:
            stamp = rec.line['stamp']
            h = Header(frame_id=meta.get('optical_frame', 'camera_color_optical_frame'),
                       stamp=Time(sec=int(stamp), nanosec=int(round((stamp % 1) * 1e9))))
            got = {}
            node.on_publish = lambda topic, header, t, q: got.setdefault(topic, (t, q))
            node.process_frame(rec.bgr.copy(), h, depth_m=rec.depth_m)
            if '/aruco/pose_raw' in got:
                t, q = got['/aruco/pose_raw']
                T = np.eye(4)
                T[:3, :3], T[:3, 3] = quat_matrix(q), t
                ref[rec.i] = T
    finally:
        node.destroy_node()
    return ref


def perturb(T, mm, deg, rng):
    if not mm and not deg:
        return T
    import cv2
    a = rng.normal(size=3)
    r = rng.normal(size=3)
    P = np.eye(4)
    P[:3, :3] = cv2.Rodrigues(r / np.linalg.norm(r) * np.radians(deg))[0]
    P[:3, 3] = a / np.linalg.norm(a) * mm / 1000.0
    return T @ P


def bucket(z_mm):
    near = [b for b in BUCKETS_MM if abs(z_mm - b) <= 0.15 * b]
    return min(near, key=lambda b: abs(z_mm - b)) if near else None


def table_checks(T, support):
    """(angle of the part's Z to the support normal, deg; height of the
    part's origin above the support plane, mm) or (None, None)."""
    if support is None:
        return None, None
    n, d = np.asarray(support[0]), support[1]
    ang = float(np.degrees(np.arccos(np.clip(abs(T[:3, 2] @ n), -1.0, 1.0))))
    return ang, float((T[:3, 3] @ n - d) * 1e3)


def evaluate(session, est, part, levels, every, rows, colour=True):
    meta, recs = load_session(session)
    recs = [r for r in recs if r.depth_m is not None]
    it = meta['intrinsics']
    K = np.array([[it['fx'], 0.0, it['cx']], [0.0, it['fy'], it['cy']], [0.0, 0.0, 1.0]])
    dist = np.asarray(it.get('coeffs') or [0.0] * 5, dtype=float)
    ref = marker_reference(meta, recs)
    T_mo = part['T_marker_object']
    out = []
    for rec in recs:
        if rec.i not in ref:
            continue
        T_ref = ref[rec.i] @ T_mo
        for li, (mm, deg) in enumerate(levels):
            if li and rec.i % every:
                continue
            rng = np.random.default_rng(rec.i * 7919 + li)
            # the prior's expected error: the perturbation, and at least the
            # marker's own few mm (sticker offset, marker noise)
            T, valid, q = est.process(rec.depth_m, rec.bgr if colour else None, K, dist,
                                      perturb(T_ref, mm, deg, rng),
                                      prior_err_m=max(0.003, mm / 1000.0))
            row = {'session': session.name, 'i': rec.i, 'level': f'{mm}/{deg}',
                   'z_mm': float(ref[rec.i][2, 3] * 1e3), 'valid': bool(valid),
                   'reason': q['reason'], 'ms': q['compute_ms'],
                   'weak': ','.join(q['weak_dof']), 'edge': q.get('edge_source') or ''}
            if T is not None:
                d, tilt, yaw = pose_error(T, T_ref, est.sym_order)
                ang, height = table_checks(T, q.get('support'))
                D = np.linalg.inv(ref[rec.i]) @ T               # marker -> estimate
                row.update(dx=d[0] * 1e3, dy=d[1] * 1e3, dz=d[2] * 1e3, tilt=tilt, yaw=yaw,
                           table_deg=ang, height_mm=height,
                           mo_x=D[0, 3] * 1e3, mo_y=D[1, 3] * 1e3, mo_z=D[2, 3] * 1e3,
                           mo_yaw=float(np.degrees(np.arctan2(D[1, 0], D[0, 0]))))
            out.append(row)
    rows.extend(out)
    return meta, len(recs), len(ref)


def _stat(vals, f):
    v = [x for x in vals if x is not None]
    return f(v) if v else None


def fmt(v, spec):
    return '-' if v is None else format(v, spec)


def report(rows, levels, sym_order):
    print('\n### Level 0 (prior = the marker reference), per session and distance\n')
    hdr = ('| session | bucket mm | frames | avail % | colour % | bias x/y/z mm '
           '| bias tilt/yaw deg | sd x/y/z mm | sd tilt/yaw deg | table deg | height mm (sd) '
           '| ms p50/p95 |')
    print(hdr)
    print('|' + '---|' * (hdr.count('|') - 1))
    zero = [r for r in rows if r['level'] == f'{levels[0][0]}/{levels[0][1]}']
    keys = sorted({(r['session'], bucket(r['z_mm'])) for r in zero},
                  key=lambda k: (k[0], k[1] is None, k[1] or 0))
    for s, b in keys:
        g = [r for r in zero if r['session'] == s and bucket(r['z_mm']) == b]
        v = [r for r in g if r['valid']]

        def col(k, rs=v):
            return [r[k] for r in rs if r.get(k) is not None]
        mean = (lambda k: _stat(col(k), np.mean))
        sd = (lambda k: _stat(col(k), np.std))
        ms = col('ms', g)
        p50 = _stat(ms, np.median)
        p95 = _stat(ms, lambda x: np.percentile(x, 95))
        colour = 100.0 * sum(r.get('edge') == 'colour' for r in v) / len(v) if v else None
        print(f"| {s} | {'-' if b is None else b} | {len(g)} | {100.0 * len(v) / len(g):.0f} | "
              f"{fmt(colour, '.0f')} | "
              f"{fmt(mean('dx'), '+.2f')}/{fmt(mean('dy'), '+.2f')}/{fmt(mean('dz'), '+.2f')} | "
              f"{fmt(mean('tilt'), '.2f')}/{fmt(mean('yaw'), '+.2f')} | "
              f"{fmt(sd('dx'), '.2f')}/{fmt(sd('dy'), '.2f')}/{fmt(sd('dz'), '.2f')} | "
              f"{fmt(sd('tilt'), '.2f')}/{fmt(sd('yaw'), '.2f')} | "
              f"{fmt(mean('table_deg'), '.2f')} | "
              f"{fmt(mean('height_mm'), '.1f')} ({fmt(sd('height_mm'), '.1f')}) | "
              f"{fmt(p50, '.0f')}/{fmt(p95, '.0f')} |")

    # Basin: does a perturbed prior reach the answer the unperturbed one
    # gave on the same frame? (Independent of the unmeasured sticker offset.)
    base = {(r['session'], r['i']): r for r in zero if r['valid']}
    print('\n### Convergence per prior level (all sessions), against the level-0 estimate\n')
    print('| level mm/deg | frames | valid % | same answer % (0.5 mm, 0.5 deg) |')
    print('|---|---|---|---|')
    for mm, deg in levels[1:]:
        g = [r for r in rows if r['level'] == f'{mm}/{deg}' and (r['session'], r['i']) in base]
        if not g:
            continue
        same = 0
        for r in g:
            b = base[(r['session'], r['i'])]
            if r.get('dx') is not None and np.linalg.norm(
                    [r['dx'] - b['dx'], r['dy'] - b['dy'], r['dz'] - b['dz']]) < 0.5 \
                    and abs(r['tilt'] - b['tilt']) < 0.5 and abs(r['yaw'] - b['yaw']) < 0.5:
                same += 1
        print(f"| {mm}/{deg} | {len(g)} | {100.0 * sum(r['valid'] for r in g) / len(g):.1f} | "
              f"{100.0 * same / len(g):.1f} |")

    v = [r for r in zero if r['valid']]
    if v:
        per = 360.0 / max(1, sym_order)
        yaw = [(r['mo_yaw'] + per / 2) % per - per / 2 for r in v]
        mo = [np.median([r[k] for r in v]) for k in ('mo_x', 'mo_y', 'mo_z')]
        print(f"\nImplied T_marker_object (median over {len(v)} valid level-0 frames): "
              f"xyz {mo[0]:+.2f} {mo[1]:+.2f} {mo[2]:+.2f} mm, in-plane {np.median(yaw):+.2f} deg "
              f"(modulo {per:.0f}) - calipers decide; this only says what depth sees.")
    reasons = {}
    for r in zero:
        if not r['valid']:
            key = r['reason'].split(' ')[0] if r['reason'] else '?'
            reasons[key] = reasons.get(key, 0) + 1
    if reasons:
        print('\nLevel-0 rejections by first reason: '
              + ', '.join(f'{k} {n}' for k, n in sorted(reasons.items(), key=lambda kv: -kv[1])))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+')
    ap.add_argument('--part', default=str(SRC / 'tools' / 'fr3' / 'parts' / 'cube55.yaml'))
    ap.add_argument('--levels', default='0/0,5/5,10/5,20/5',
                    help='prior perturbations, mm/deg, comma separated (first = the report level)')
    ap.add_argument('--every', type=int, default=5, help='perturbed levels on every N-th frame')
    ap.add_argument('--csv', help='per-frame rows to this CSV file')
    ap.add_argument('--depth-only', action='store_true',
                    help='no colour frames: the outline from the depth region only')
    args = ap.parse_args(argv)
    levels = [tuple(float(x) for x in s.split('/')) for s in args.levels.split(',')]
    levels = [tuple(int(v) if v.is_integer() else v for v in lv) for lv in levels]
    part = load_part(args.part)
    est = ObjectPoseEstimator(part)
    import rclpy
    rclpy.init(domain_id=88)                           # never the cell's domain
    rows = []
    try:
        for s in session_dirs(args.sessions):
            meta, n, n_ref = evaluate(s, est, part, levels, max(1, args.every), rows,
                                      colour=not args.depth_only)
            cap = meta.get('capture', {})
            print(f"{s.name}: {n} frames with depth, {n_ref} with a marker reference "
                  f"({cap.get('preset', '?')}, spatial {cap.get('spatial_filter', '?')})",
                  flush=True)
    finally:
        rclpy.shutdown()
    measured = np.any(part['T_marker_object'] != np.eye(4))
    print(f"\npart {part['name']}, T_marker_object "
          f"{'from the part file' if measured else 'identity (not measured)'}")
    report(rows, levels, est.sym_order)
    if args.csv:
        keys = sorted({k for r in rows for k in r})
        with open(args.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)


if __name__ == '__main__':
    main()
