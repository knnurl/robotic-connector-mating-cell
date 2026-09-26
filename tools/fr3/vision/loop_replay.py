#!/usr/bin/env python3
"""Replay recorded sessions through the live vision loop, offline
(PERCEPTION_PLAN Phases 3 and 4).

    taskset -c 13 python3 tools/fr3/vision/loop_replay.py <session_dir> [...]
            [--source marker|depth_checked] [--part cube55.yaml] [--csv frames.csv]

Run from src/ with fr3_env.sh sourced; it uses DDS domain 88, never the
cell's. Every recorded frame goes through vision_standalone.handle_frame
with the objects the cell builds:
- cam_pub's ArucoPosePublisher, with range_source depth as the cell runs;
- the ObjectContract, with object_source set to --source;
- the DepthRunner, with the estimator on (shadow mode for marker, driving
  for depth_checked).

Each frame keeps its recorded stamp, and the filters run in fr3_link0 on the
TF of the session's bag (../../bag_<same time>). So which frames give a pose,
and when, is what the cell would have seen.

Table 1, the estimate against the marker, per session and distance bucket
(the marker's camera distance):
  marker %   frames with a raw marker pose (each seeds one estimate)
  est %      of those, estimated (the rest skipped: budget, held, no depth)
  valid %    of the estimated, passed the estimator's gates
  bias, sd   the estimate against marker o T_marker_object, in the part
             frame: x, y, z mm; tilt about the part's x and y, signed, deg
             (the mean of an unsigned tilt angle would read noise as bias);
             in-plane deg, modulo the symmetry
  agree      the contract's agree_mm / agree_deg, p95
  ms         the whole frame (marker + estimate + quality) p50 / p95 / max,
             and how many frames overran the camera's frame period. Pin to
             one E-core (taskset -c 13) to time it as the cell runs. Frames
             outside the bag's TF (its first and last few) wait out the 50 ms
             lookup timeout, which the cell does not: they are left out of the
             timing and counted apart.

Table 2, what the consumers get on /object/* with --source:
  raw %      frames with a /object/pose_raw, of those with a marker
  veto %     depth_checked: valid estimates the marker vetoed
  pose sd    /object/pose in fr3_link0 over the bucket, x/y/z mm. The cube is
             still, so this is the filtered pose's scatter, plus the
             hand-eye's as the camera moves between plateaus.
  pub ms     from picking the frame up to /object/pose_raw going out, p50 /
             p95 (the TRACK row in PERCEPTION_PLAN section 1: <= 40 ms with
             the TF wait)
The raw gaps over 0.2 s (TRACK holds after 0.25 s) are listed per session.
"""

import argparse
import csv
import os
import pathlib
import re
import sys
import time

for _var in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_var, '1')

import numpy as np  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parents[2]
sys.path.insert(0, str(SRC / 'roscam'))
sys.path.insert(0, str(HERE))
from replay_eval import bucket, fmt, session_dirs  # noqa: E402
from roscam.frame_recorder import load_session  # noqa: E402
from roscam.object_pose import pose_error  # noqa: E402
from roscam.object_shadow import pose_matrix  # noqa: E402


def bag_tf_buffer(bag_dir):
    """A tf2_ros Buffer holding the bag's /tf and /tf_static, or None."""
    import rosbag2_py
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    from tf2_ros.buffer import Buffer

    if not (bag_dir / 'metadata.yaml').is_file():
        return None
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    span_s = reader.get_metadata().duration.nanoseconds * 1e-9
    buf = Buffer(cache_time=Duration(seconds=span_s + 60.0))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static']))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        for tf in deserialize_message(data, TFMessage).transforms:
            if topic == '/tf_static':
                buf.set_transform_static(tf, 'bag')
            else:
                buf.set_transform(tf, 'bag')
    return buf


def replay(session, part_file, source, rows, impl='cpp'):
    import cv2
    from builtin_interfaces.msg import Time
    from rclpy.parameter import Parameter
    from std_msgs.msg import Header

    from roscam.cam_pub import ArucoPosePublisher
    from roscam.object_contract import ObjectContract
    from roscam.rs_capture import Frame
    from roscam.vision_standalone import DepthRunner, handle_frame

    meta, recs = load_session(session)
    bag = bag_tf_buffer(session.parents[1] / session.name.replace('vision_', 'bag_'))
    fps = int(meta.get('capture', {}).get('fps', 15))
    node = ArucoPosePublisher(parameter_overrides=[
        Parameter('source', value='external'),
        Parameter('marker_id', value=int(meta.get('marker_id', 0))),
        Parameter('marker_size_m', value=float(meta['marker_size_m'])),
        Parameter('target_marker_id', value=-1), Parameter('range_source', value='depth'),
        Parameter('publish_debug_image', value=False), Parameter('capture_fps', value=fps),
        Parameter('filter_frame', value='fr3_link0' if bag is not None else ''),
        Parameter('object_part', value=str(part_file)), Parameter('object_shadow', value=True),
        Parameter('object_source', value=source), Parameter('object_pose_impl', value=impl)])
    if bag is not None:
        node.tf_buffer = bag                        # the TF the cell had, from the bag
    it = meta['intrinsics']
    node.set_intrinsics(it['fx'], it['fy'], it['cx'], it['cy'], it['coeffs'])
    contract = ObjectContract(node, estimator=True)
    runner = DepthRunner(node, contract)
    runner.depth.warm(node.camera_matrix, node.dist_coeffs, (it['height'], it['width']))
    quality, raw, out = [], {}, {}
    contract.quality_pub.publish = quality.append
    contract.pubs['raw'].publish = lambda m: out.__setitem__('raw', (time.perf_counter(), m))
    contract.pubs['pose'].publish = lambda m: out.__setitem__('pose', m)

    def keep_raw(topic, _h, t, q):
        if topic == '/aruco/pose_raw':
            raw['m'] = (t, q)
    node.publish_hooks.append(keep_raw)
    T_mo = contract.part['T_marker_object']
    sym = int(contract.part['sym_order'])
    optical = meta.get('optical_frame', 'camera_color_optical_frame')
    n = 0
    try:
        for rec in recs:
            if rec.depth_m is None:
                continue
            s = rec.line['stamp']
            h = Header(frame_id=optical,
                       stamp=Time(sec=int(s), nanosec=int(round((s % 1) * 1e9))))
            node.self_stamped_header = lambda h=h: h
            raw.clear()
            out.clear()
            t0 = time.perf_counter()
            handle_frame(node, Frame(rec.bgr, rec.depth_m, rec.line.get('t_host', 0.0)),
                         None, runner)
            loop_ms = (time.perf_counter() - t0) * 1e3
            kv = {k.key: k.value for k in quality[-1].status[0].values}
            row = {'session': session.name, 'i': rec.i, 'stamp': s, 'loop_ms': loop_ms,
                   'marker_ms': float(kv['compute_ms']), 'marker': 'm' in raw,
                   'seeded_from': kv.get('seeded_from'), 'reason': kv.get('depth_reason', ''),
                   'valid': kv.get('depth_valid') == 'true', 'depth_ms': kv.get('depth_ms'),
                   'edge': kv.get('edge_source', ''), 'tf_wait_ms': kv.get('tf_wait_ms'),
                   'agree_mm': kv.get('agree_mm'), 'agree_deg': kv.get('agree_deg'),
                   'check': kv.get('check', ''),
                   'tf_gap': float(kv.get('tf_wait_ms', 0.0)) >= 50.0,
                   'raw_pub': 'raw' in out,
                   'exposure_us': rec.line.get('exposure_us'), 'gain': rec.line.get('gain'),
                   'pub_ms': (out['raw'][0] - t0) * 1e3 if 'raw' in out else None}
            row['over'] = loop_ms > 1000.0 / fps and not row['tf_gap']
            if 'pose' in out:
                pp = out['pose'].pose.position
                row.update(px=pp.x * 1e3, py=pp.y * 1e3, pz=pp.z * 1e3)
            if 'm' in raw:
                T_ref = pose_matrix(*raw['m']) @ T_mo
                row['z_mm'] = float(raw['m'][0][2] * 1e3)
                # both poses of this frame, optical frame (t, q), for offline joins
                row['m_pose'] = ','.join(f'{v:.6f}' for v in list(raw['m'][0]) + list(raw['m'][1]))
                row['c_pose'] = kv.get('cand_pose', '')
                if 'cand_pose' in kv:
                    v = [float(x) for x in kv['cand_pose'].split(',')]
                    T_est = pose_matrix(v[:3], v[3:])
                    d, _, yaw = pose_error(T_est, T_ref, sym)
                    D = np.linalg.inv(T_ref) @ T_est
                    r = np.degrees(cv2.Rodrigues(D[:3, :3])[0].ravel())
                    row.update(dx=d[0] * 1e3, dy=d[1] * 1e3, dz=d[2] * 1e3, tx=float(r[0]),
                               ty=float(r[1]), yaw=yaw)
            rows.append(row)
            n += 1
    finally:
        listener = getattr(node, 'tf_listener', None)
        if listener is not None and getattr(listener, 'executor', None) is not None:
            listener.executor.shutdown()            # its own spin thread, before rclpy's
            listener.dedicated_listener_thread.join(timeout=2.0)
        node.destroy_node()
    return meta, n, bag is not None


def _groups(rows):
    keys = sorted({(r['session'], bucket(r['z_mm']) if 'z_mm' in r else None) for r in rows},
                  key=lambda k: (k[0], k[1] is None, k[1] or 0))
    for s, b in keys:
        yield s, b, [r for r in rows if r['session'] == s
                     and (bucket(r['z_mm']) if 'z_mm' in r else None) == b]


def _stats(rs):
    def st(k, f):
        x = [r[k] for r in rs if r.get(k) is not None]
        return f(np.asarray(x, dtype=float)) if x else None
    return (lambda k: st(k, np.mean), lambda k: st(k, np.std),
            lambda k: st(k, lambda x: float(np.percentile(x, 95))))


def _pct(a, c):
    return f'{100.0 * len(a) / len(c):.0f}' if c else '-'


def report(rows, fps, source):
    print('\n### 1. The estimate against the marker\n')
    print('| session | bucket mm | frames | marker % | est % | valid % | bias x/y/z mm '
          '| bias tilt x/y deg | bias yaw deg | sd x/y/z mm | sd tilt x/y deg | sd yaw deg '
          '| agree p95 mm/deg | frame ms p50/p95/max | over period |')
    print('|' + '---|' * 15)
    for s, b, g in _groups(rows):
        m = [r for r in g if r['marker']]
        e = [r for r in m if r['seeded_from'] == 'marker']
        v = [r for r in e if r['valid']]
        mean, sd, p95 = _stats(v)
        ms = [r['loop_ms'] for r in g if not r['tf_gap']] or [float('nan')]
        print(f"| {s} | {'-' if b is None else b} | {len(g)} | {_pct(m, g)} | {_pct(e, m)} | "
              f"{_pct(v, e)} | "
              f"{fmt(mean('dx'), '+.2f')}/{fmt(mean('dy'), '+.2f')}/{fmt(mean('dz'), '+.2f')} | "
              f"{fmt(mean('tx'), '+.2f')}/{fmt(mean('ty'), '+.2f')} | "
              f"{fmt(mean('yaw'), '+.2f')} | "
              f"{fmt(sd('dx'), '.2f')}/{fmt(sd('dy'), '.2f')}/{fmt(sd('dz'), '.2f')} | "
              f"{fmt(sd('tx'), '.2f')}/{fmt(sd('ty'), '.2f')} | {fmt(sd('yaw'), '.2f')} | "
              f"{fmt(p95('agree_mm'), '.2f')}/{fmt(p95('agree_deg'), '.2f')} | "
              f"{np.median(ms):.0f}/{np.percentile(ms, 95):.0f}/{max(ms):.0f} | "
              f"{sum(r['over'] for r in g)} |")

    print(f'\n### 2. What the consumers get on /object/*, source {source}\n')
    print('| session | bucket mm | marker frames | raw % | veto % | pose sd x/y/z mm '
          '| pub ms p50/p95 |')
    print('|' + '---|' * 7)
    for s, b, g in _groups(rows):
        m = [r for r in g if r['marker']]
        pub = [r for r in m if r['raw_pub']]
        valid = [r for r in g if r['valid']]
        vetoed = [r for r in valid if r['check'].startswith('vetoed')]
        _, sd, _ = _stats(g)
        lat = [r['pub_ms'] for r in pub if not r['tf_gap']]
        print(f"| {s} | {'-' if b is None else b} | {len(m)} | {_pct(pub, m)} | "
              f"{_pct(vetoed, valid) if source == 'depth_checked' else '-'} | "
              f"{fmt(sd('px'), '.2f')}/{fmt(sd('py'), '.2f')}/{fmt(sd('pz'), '.2f')} | "
              + (f"{np.median(lat):.0f}/{np.percentile(lat, 95):.0f} |" if lat else '- |'))
    print()
    for s in sorted({r['session'] for r in rows}):
        stamps = [r['stamp'] for r in rows if r['session'] == s and r['raw_pub']]
        gaps = np.diff(stamps) if len(stamps) > 1 else np.array([])
        long = gaps[gaps > 0.2]
        print(f"{s}: {len(stamps)} raw poses; gaps over 0.2 s: {len(long)}"
              + (f", longest {gaps.max():.2f} s" if len(gaps) else ''))

    over = [r for r in rows if r['over']]
    gap = sum(r['tf_gap'] for r in rows)
    print(f"\nFrame period {1000.0 / fps:.1f} ms: {len(over)} of {len(rows) - gap} frames "
          f"overran it ({gap} outside the bag's TF left out)"
          + (': ' + ', '.join(f"{r['session'][-6:]}#{r['i']} {r['loop_ms']:.0f} ms"
                              for r in over[:8]) if over else '.'))
    skipped = {}
    for r in rows:
        if r['marker'] and r['seeded_from'] != 'marker':
            key = r['reason'].split(':')[0]
            skipped[key] = skipped.get(key, 0) + 1
    if skipped:
        print('Marker frames not estimated: ' + ', '.join(f'{k} {n}' for k, n in skipped.items()))
    rejected = {}
    for r in rows:
        if r['seeded_from'] == 'marker' and not r['valid']:
            key = re.sub(r'[-+\d.]+', '', r['reason'].split(';')[0].split(' (')[0]).strip()
            rejected[key or '?'] = rejected.get(key or '?', 0) + 1
    if rejected:
        print('Estimates rejected by first reason: '
              + ', '.join(f'{k} {n}' for k, n in sorted(rejected.items(), key=lambda kv: -kv[1])))
    if source == 'depth_checked':
        checks = {}
        for r in rows:
            if r['marker'] and not r['raw_pub']:
                key = re.sub(r'[-+\d.,/]+', '', r['check'].split(':')[0]).strip() or '?'
                checks[key] = checks.get(key, 0) + 1
        if checks:
            print('Marker frames without a raw pose, by check: '
                  + ', '.join(f'{k} {n}' for k, n in
                              sorted(checks.items(), key=lambda kv: -kv[1])))
    tf = [float(r['tf_wait_ms']) for r in rows if r.get('tf_wait_ms') is not None]
    if tf:
        print(f'tf_wait_ms p50/p95/max (a buffer filled from the bag, not live): '
              f'{np.median(tf):.2f}/{np.percentile(tf, 95):.2f}/{max(tf):.2f}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+')
    ap.add_argument('--source', default='marker', choices=('marker', 'depth_checked'))
    ap.add_argument('--part', default=str(SRC / 'tools' / 'fr3' / 'parts' / 'cube55.yaml'))
    ap.add_argument('--csv', help='per-frame rows to this CSV file')
    ap.add_argument('--impl', default='cpp', choices=('cpp', 'python'),
                    help='the depth estimator (object_pose_cpp, or the Python reference)')
    args = ap.parse_args(argv)
    import rclpy
    rclpy.init(domain_id=88)                           # never the cell's domain
    rows, fps = [], 15
    try:
        for s in session_dirs(args.sessions):
            meta, n, tf = replay(s, pathlib.Path(args.part).resolve(), args.source, rows,
                                 args.impl)
            fps = int(meta.get('capture', {}).get('fps', 15))
            print(f"{s.name}: {n} frames, filters in "
                  f"{'fr3_link0 (bag TF)' if tf else 'the optical frame (no bag)'}", flush=True)
    finally:
        rclpy.shutdown()
    report(rows, fps, args.source)
    if args.csv:
        keys = sorted({k for r in rows for k in r})
        with open(args.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)


if __name__ == '__main__':
    main()
