#!/usr/bin/env python3
"""Tilt reference (PERCEPTION_PLAN 3A step 1): how far the marker and the
depth estimate read a cube standing on the floor as tilted.

The floor is in plane with the robot base (operator, 2026-09-30), so the
true normal of a cube standing on it is base +z: any tilt read is error.

    python3 tools/fr3/vision/tilt_reference.py <bag_dir> [<bag_dir> ...]

For each stretch of >= 2 s with the TCP still (< 1 mm/s, from
/cell_panel/robot_state), the tilt of /aruco/pose (the marker, cam_pub's
fr3_link0 pose) and /object/pose (the contract, depth when the source is
depth_checked) against base z: mean, and the direction it leans.
"""

import math
import sys

import numpy as np


def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while r.has_next():
        name, data, t_ns = r.read_next()
        if name in out:
            out[name].append((t_ns * 1e-9, deserialize_message(data, get_message(types[name]))))
    return out


def z_axis(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def tilt(n):
    """(deg from base z, deg azimuth of the lean) of a normal."""
    n = n / np.linalg.norm(n)
    return math.degrees(math.acos(max(-1.0, min(1.0, n[2])))), math.degrees(math.atan2(n[1], n[0]))


def still_stretches(t, p, min_s=2.0, mm_s=1.0):
    v = np.zeros(len(t))
    v[1:-1] = np.linalg.norm(p[2:] - p[:-2], axis=1) / np.maximum(t[2:] - t[:-2], 1e-6)
    runs, i0 = [], None
    for i, s in enumerate(v * 1000.0 < mm_s):
        if s and i0 is None:
            i0 = i
        if (not s or i == len(v) - 1) and i0 is not None:
            i1 = i if s else i - 1
            if t[i1] - t[i0] >= min_s:
                runs.append((t[i0], t[i1], float(p[i0:i1 + 1, 2].mean())))
            i0 = None
    return runs


def main(bags):
    rows = []
    for bag in bags:
        d = read_bag(bag, ['/cell_panel/robot_state', '/aruco/pose', '/object/pose'])
        rs = d['/cell_panel/robot_state']
        if not rs:
            print(f'{bag}: no /cell_panel/robot_state')
            continue
        t = np.array([x[0] for x in rs])
        p = np.array([[m.o_t_ee.pose.position.x, m.o_t_ee.pose.position.y,
                       m.o_t_ee.pose.position.z] for _, m in rs])
        print(f'== {bag}')
        for t0, t1, zt in still_stretches(t, p):
            out = [f'{t1 - t0:5.1f} s  TCP z {zt * 1000:5.0f} mm']
            for name, tag in (('/aruco/pose', 'marker'), ('/object/pose', 'object')):
                sel = [m for tt, m in d[name] if t0 <= tt <= t1]
                if len(sel) < 10:
                    out.append(f'{tag} -')
                    continue
                N = np.array([z_axis(m.pose.orientation) for m in sel])
                tl, az = tilt(N.mean(axis=0))
                sd = float(np.std([tilt(n)[0] for n in N]))
                out.append(f'{tag} {tl:4.2f} deg (sd {sd:.2f}) toward {az:4.0f}')
                if tag == 'marker':
                    rows.append((zt, tl))
            print('  ' + '  |  '.join(out))
    if rows:
        tl = np.array([r[1] for r in rows])
        print(f'marker tilt against the floor over {len(rows)} stills: mean {tl.mean():.2f}, '
              f'max {tl.max():.2f} deg')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1:])
