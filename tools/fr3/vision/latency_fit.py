#!/usr/bin/env python3
"""Measure the camera's true capture latency from a bag (PERCEPTION_PLAN
Phase 0).

    python3 tools/fr3/vision/latency_fit.py <bag_dir>
    python3 tools/fr3/vision/latency_fit.py <bag_dir> --session <vision session dir>
    python3 tools/fr3/vision/latency_fit.py <bag_dir> --capture-latency 0.02 --min-speed 20

<bag_dir> is a bag from the panel's REC button (actions.Recorder.TOPICS:
/aruco/pose_raw, /tf, /tf_static, ...). --session takes capture_latency_s
from a frame_recorder session.json; otherwise --capture-latency (default
0.02 s, cam_pub's default) is the value the bag's stamps were made with.

Why: cam_pub / vision_standalone stamp each frame  node_now - capture_latency_s.
If the true latency differs by delta, every pose is stamped delta off, and
while the arm moves the eye-in-hand camera puts a STATIC marker at a base
position that shifts with camera velocity.

Recording: the marker lies still, the panel records, and the arm is jogged
BACK AND FORTH over the same path (100-200 mm/s, some rotation too), marker
in view, a few times. Back and forth matters: hand-eye and ArUco errors
depend on where the camera is; visiting each place in both directions keeps
them uncorrelated with velocity, so they raise the floor of the objective
but do not move its minimum.

Method (fit_stamp_offset): for each offset d on a grid (default -100..+100
ms, 1 ms) every raw marker observation p_cam, stamped s, is mapped to the
base frame through T_base_optical(s + d) (tf2 BufferCore filled from the
bag). The objective J(d) is the mean squared deviation of those base
positions from their mean, over the frames where the camera moved faster
than --min-speed. The speed is that of the point the camera carries at the
marker (translation plus rotation times lever arm), i.e. how fast a stamp
error moves the marker in the base frame. d* = argmin J; to first order
this is the regression of base-frame scatter on that velocity, without the
linearisation. A bootstrap over frames gives the 95% CI; the curvature of
J at d* says how sharp the minimum is (m^2/s^2 = mm^2/ms^2: J rises by
curvature/2 mm^2 at 1 ms off the minimum).

Sign: with assumed latency L0 = capture_latency_s and true latency L, a
frame captured at t_c reaches the node at t_c + L and is stamped
s = t_c + L - L0. The fit finds s + d* = t_c, so d* = L0 - L and

    true latency L = capture_latency_s - d*

d* < 0: the stamps are later than the capture, the latency is longer than
assumed. d is measured against the /tf stamps (the joint states' clock), so
any lag in those is folded in; that is the offset that matters, since the
pipeline looks up TF at the image stamp.

The bootstrap resamples frames as independent. Consecutive frames are
correlated (same place, same hand-eye error), so the CI is optimistic;
repeat the recording to see the real spread.
"""

import argparse
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / 'roscam'))
from roscam.frame_recorder import load_session  # noqa: E402

SPEED_HALF_WINDOW_S = 0.01      # central difference for the marker-point speed
MIN_FRAMES = 10                 # fewer moving frames: d is not determined
SHARPNESS_WINDOW_S = 0.010      # quadratic fit of J within this of d*


def default_deltas(span_s=0.1, step_s=0.001):
    """-span..+span in step_s steps, 0 included exactly."""
    n = int(round(span_s / step_s))
    return np.arange(-n, n + 1) * step_s


def _apply(T, p):
    return None if T is None else T[:3, :3] @ p + T[:3, 3]


def fit_stamp_offset(stamps, p_cam, T_base_optical_at, deltas=None, min_speed=0.02,
                     n_boot=1000, seed=0, capture_latency_s=None):
    """Stamp offset d that makes a static marker most static in the base frame.

    stamps (N,) image stamps, s; p_cam (N, 3) marker positions in the optical
    frame, m; T_base_optical_at(t) -> 4x4 (optical -> base) or None where TF
    does not cover t; min_speed in m/s. Frames without TF over the whole grid
    are dropped. Returns a dict; 'd_star' is None, with a 'reason', when the
    data cannot determine d. With capture_latency_s, also the implied true
    latency and its CI (L = capture_latency_s - d)."""
    stamps = np.asarray(stamps, dtype=float)
    p_cam = np.asarray(p_cam, dtype=float).reshape(-1, 3)
    deltas = default_deltas() if deltas is None else np.asarray(deltas, dtype=float)
    n = len(stamps)
    P = np.full((len(deltas), n, 3), np.nan)
    n_tf = 0
    for i, (s, p) in enumerate(zip(stamps, p_cam)):
        a, b = (_apply(T_base_optical_at(s + h), p)
                for h in (-SPEED_HALF_WINDOW_S, SPEED_HALF_WINDOW_S))
        if a is None or b is None:
            continue
        n_tf += 1
        if np.linalg.norm(b - a) / (2 * SPEED_HALF_WINDOW_S) < min_speed:
            continue
        col = [_apply(T_base_optical_at(s + d), p) for d in deltas]
        if all(q is not None for q in col):
            P[:, i] = col
    used = ~np.isnan(P[0, :, 0])
    res = {'n_frames': n, 'n_tf': n_tf, 'n_moving': int(used.sum()), 'd_star': None}
    if res['n_moving'] < MIN_FRAMES:
        res['reason'] = (f"{res['n_moving']} of {n_tf} frames with TF moved faster than "
                         f"{min_speed * 1e3:.0f} mm/s at the marker (need {MIN_FRAMES}): "
                         'the data cannot determine d')
        return res

    P = P[:, used]
    P = P - P.mean(axis=(0, 1))                 # small numbers: no cancellation below
    m = P.shape[1]
    sq = (P ** 2).sum(axis=2)                   # (G, M)
    J = sq.mean(axis=1) - (P.mean(axis=1) ** 2).sum(axis=1)
    k = int(np.argmin(J))

    # Bootstrap: resampling frames = multinomial weights on them.
    W = np.random.default_rng(seed).multinomial(m, np.full(m, 1.0 / m), size=n_boot) / m
    Jb = W @ sq.T - (np.tensordot(W, P, axes=([1], [1])) ** 2).sum(axis=2)
    d_boot = deltas[np.argmin(Jb, axis=1)]
    lo, hi = np.percentile(d_boot, [2.5, 97.5])

    win = np.abs(deltas - deltas[k]) <= SHARPNESS_WINDOW_S + 1e-9
    curvature = 2.0 * np.polyfit(deltas[win], J[win], 2)[0] if win.sum() >= 3 else float('nan')
    zero = np.flatnonzero(np.isclose(deltas, 0.0))
    res.update(d_star=float(deltas[k]), ci95=(float(lo), float(hi)),
               curvature=float(curvature),
               rms_min_mm=float(np.sqrt(max(J[k], 0.0)) * 1e3),
               rms_zero_mm=float(np.sqrt(max(J[zero[0]], 0.0)) * 1e3) if len(zero) else None,
               at_edge=k in (0, len(deltas) - 1), deltas=deltas, objective=J)
    if capture_latency_s is not None:
        res['latency'] = capture_latency_s - res['d_star']
        res['latency_ci95'] = (capture_latency_s - hi, capture_latency_s - lo)
    return res


def _rot(x, y, z, w):
    q = np.array([x, y, z, w], dtype=float)
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_bag(bag_dir, base_frame='fr3_link0', pose_topic='/aruco/pose_raw'):
    """(stamps (N,), p_cam (N, 3), optical frame id, T_base_optical_at) from a
    rosbag2 directory. TF comes from the bag's /tf and /tf_static through a
    tf2 BufferCore (interpolated as tf2 does); lookups outside its coverage
    return None."""
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import BufferCore, TransformException

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    span_s = reader.get_metadata().duration.nanoseconds * 1e-9
    buf = BufferCore(Duration(seconds=span_s + 60.0))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static', pose_topic]))
    stamps, pts, frames = [], [], set()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == pose_topic:
            msg = deserialize_message(data, PoseStamped)
            stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            p = msg.pose.position
            pts.append((p.x, p.y, p.z))
            frames.add(msg.header.frame_id)
        else:
            for tf in deserialize_message(data, TFMessage).transforms:
                if topic == '/tf_static':
                    buf.set_transform_static(tf, 'bag')
                else:
                    buf.set_transform(tf, 'bag')
    if not stamps:
        raise ValueError(f'no {pose_topic} messages in {bag_dir}')
    if len(frames) != 1:
        raise ValueError(f'{pose_topic} comes in several frames: {sorted(frames)}')
    optical = frames.pop()

    def lookup(t):
        return buf.lookup_transform_core(base_frame, optical,
                                         Time(nanoseconds=int(round(t * 1e9))))

    try:
        lookup(stamps[len(stamps) // 2])
    except TransformException as e:
        raise ValueError(f'no TF {base_frame} <- {optical} in the bag: {e}') from None

    def T_base_optical_at(t):
        try:
            tr = lookup(t).transform
        except TransformException:
            return None
        T = np.eye(4)
        T[:3, :3] = _rot(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
        T[:3, 3] = (tr.translation.x, tr.translation.y, tr.translation.z)
        return T

    return np.array(stamps), np.array(pts), optical, T_base_optical_at


def report(res, capture_latency_s):
    ms = 1e3
    lines = [f"frames: {res['n_frames']}, with TF: {res['n_tf']}, moving: {res['n_moving']}"]
    if res['d_star'] is None:
        return '\n'.join(lines + [res['reason']])
    lo, hi = res['ci95']
    lines.append(f"stamp offset d* = {res['d_star'] * ms:+.1f} ms "
                 f"(95% CI {lo * ms:+.1f} .. {hi * ms:+.1f})")
    if 'latency' in res:
        llo, lhi = res['latency_ci95']
        lines.append(f"true capture latency = {res['latency'] * ms:.1f} ms "
                     f"(95% CI {llo * ms:.1f} .. {lhi * ms:.1f}); "
                     f"stamps used {capture_latency_s * ms:.1f} ms")
    zero = ('' if res['rms_zero_mm'] is None
            else f", {res['rms_zero_mm']:.2f} mm as stamped (d = 0)")
    lines.append(f"static-marker rms scatter: {res['rms_min_mm']:.2f} mm at d*{zero}")
    lines.append(f"sharpness: J'' = {res['curvature']:.4f} mm^2/ms^2 at d*")
    if res['at_edge']:
        lines.append('WARNING: the minimum is at the edge of the grid - widen --span-ms')
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='rosbag2 directory from the panel REC button')
    ap.add_argument('--base-frame', default='fr3_link0')
    ap.add_argument('--topic', default='/aruco/pose_raw', help='raw optical-frame marker pose')
    ap.add_argument('--min-speed', type=float, default=20.0,
                    help='mm/s at the marker; slower frames are left out (default 20)')
    ap.add_argument('--span-ms', type=float, default=100.0, help='search d in +-span (100)')
    ap.add_argument('--step-ms', type=float, default=1.0, help='grid step (1)')
    ap.add_argument('--bootstrap', type=int, default=1000, help='resamples for the CI')
    src = ap.add_mutually_exclusive_group()
    src.add_argument('--capture-latency', type=float,
                     help='s: the capture_latency_s the stamps were made with (default 0.02)')
    src.add_argument('--session', help='frame_recorder session dir: capture_latency_s '
                                       'from its session.json')
    args = ap.parse_args(argv)

    if args.session:
        meta, _ = load_session(args.session)
        latency0 = float(meta['capture_latency_s'])
    else:
        latency0 = 0.02 if args.capture_latency is None else args.capture_latency
    try:
        stamps, p_cam, optical, T_at = load_bag(args.bag, args.base_frame, args.topic)
    except ValueError as e:
        sys.exit(str(e))
    print(f'{args.bag}: {len(stamps)} poses in {optical}, base {args.base_frame}')
    res = fit_stamp_offset(stamps, p_cam, T_at,
                           deltas=default_deltas(args.span_ms / 1e3, args.step_ms / 1e3),
                           min_speed=args.min_speed / 1e3, n_boot=args.bootstrap,
                           capture_latency_s=latency0)
    print(report(res, latency0))


if __name__ == '__main__':
    main()
