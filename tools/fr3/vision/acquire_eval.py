#!/usr/bin/env python3
"""Marker-free acquisition (PERCEPTION_PLAN Phase 6), scored on recordings.

    python3 tools/fr3/vision/acquire_eval.py <session_dir> [...] [--every N]
            [--mask-marker] [--impl cpp|python] [--csv out.csv]

Run from src/ with fr3_env.sh sourced. For every Nth recorded frame that
carries the cube's marker pose, ObjectPoseEstimator.acquire() runs with NO
prior and is scored against the marker (its pose composed with the part
file's T_marker_object):
  correct   valid and within 3 mm / 3 deg, modulo the part's symmetry;
  false     valid and further off: a false acquisition;
  none      no valid candidate (q['reason'] says why).
--mask-marker blanks the marker's square out of the depth image, roughly as
a bare cube would read (the colour image keeps it: the outline uses the
cube's edges, not the marker's).
"""
import argparse
import csv
import json
import pathlib
import sys
import time

import cv2
import numpy as np

SRC = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SRC / 'roscam'))

from roscam.object_pose import (CppObjectPoseEstimator, ObjectPoseEstimator,  # noqa: E402
                                load_part, pose_error)


def pose_matrix(t, q):
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sessions', nargs='+')
    ap.add_argument('--every', type=int, default=5)
    ap.add_argument('--mask-marker', action='store_true')
    ap.add_argument('--impl', default='cpp', choices=('cpp', 'python'))
    ap.add_argument('--part', default=str(SRC / 'tools' / 'fr3' / 'parts' / 'cube55.yaml'))
    ap.add_argument('--csv')
    a = ap.parse_args()
    part = load_part(a.part)
    est = (CppObjectPoseEstimator if a.impl == 'cpp' else ObjectPoseEstimator)(
        part, depth_fallback=False)
    rows = []
    for ses in map(pathlib.Path, a.sessions):
        meta = json.loads((ses / 'session.json').read_text())
        it = meta['intrinsics']
        K = np.array([[it['fx'], 0, it['cx']], [0, it['fy'], it['cy']], [0, 0, 1.0]])
        dist = np.asarray(it['coeffs'], float)
        scale = float(meta['capture']['depth_scale'])
        half = float(meta.get('marker_size_m', 0.021)) / 2.0
        counts = {'correct': 0, 'false': 0, 'none': 0}
        reasons, ms = {}, []
        for text in (ses / 'frames.jsonl').open():
            line = json.loads(text)
            p = line['poses'].get('/aruco/pose_raw')
            if p is None or not line.get('depth') or line['i'] % a.every:
                continue
            name = f"{line['i']:06d}.png"
            raw = cv2.imread(str(ses / 'depth' / name), cv2.IMREAD_UNCHANGED)
            depth = raw.astype(np.float64) * scale
            bgr = cv2.imread(str(ses / 'color' / name))
            T_marker = pose_matrix(p['t'], p['q'])
            T_ref = T_marker @ part['T_marker_object']
            if a.mask_marker:
                sq = np.array([[-half, -half, 0], [half, -half, 0],
                               [half, half, 0], [-half, half, 0]])
                uv, _ = cv2.projectPoints(sq @ T_marker[:3, :3].T + T_marker[:3, 3],
                                          np.zeros(3), np.zeros(3), K, dist)
                m = np.zeros(depth.shape, np.uint8)
                cv2.fillConvexPoly(m, np.round(uv.reshape(-1, 2)).astype(np.int32), 1)
                depth[m > 0] = 0.0
            t0 = time.perf_counter()
            T, valid, q = est.acquire(depth, bgr, K, dist)
            ms.append((time.perf_counter() - t0) * 1e3)
            row = {'session': ses.name, 'i': line['i'], 'stamp': line['stamp'],
                   'range_mm': float(p['t'][2]) * 1e3,
                   'valid': bool(valid), 'reason': q['reason'],
                   'n_cand': len(q.get('candidates', [])),
                   'whys': ' | '.join(
                       f"{[round(x) for x in c.get('side_mm', [])]} {c.get('why', '')}"
                       for c in q.get('acquire', []))}
            if valid:
                d, tilt, yaw = pose_error(T, T_ref, part['sym_order'])
                err = float(np.linalg.norm(d)) * 1e3
                rot = max(abs(float(tilt)), abs(float(yaw)))
                row.update(err_mm=err, tilt_deg=float(tilt), yaw_deg=float(yaw))
                if q.get('support_normal') is not None:     # tilt against the floor it stands on
                    c = abs(float(np.dot(T[:3, 2], q['support_normal'])))
                    row['tilt_vs_support_deg'] = float(np.degrees(np.arccos(min(1.0, c))))
                    c = abs(float(np.dot(T_ref[:3, 2], q['support_normal'])))
                    row['marker_tilt_vs_support_deg'] = float(np.degrees(np.arccos(min(1.0, c))))
                cat = 'correct' if err <= 3.0 and rot <= 3.0 else 'false'
            else:
                cat = 'none'
                key = q['reason'].split(':')[-1].strip()[:40]
                reasons[key] = reasons.get(key, 0) + 1
            row['cat'] = cat
            counts[cat] += 1
            rows.append(row)
        n = sum(counts.values())
        if not n:
            print(f'{ses.name}: no frames with the marker')
            continue
        errs = [r['err_mm'] for r in rows if r['session'] == ses.name and r['cat'] == 'correct']
        print(f"{ses.name}{' (marker masked)' if a.mask_marker else ''}: {n} frames, "
              f"correct {100 * counts['correct'] / n:.1f} %, FALSE {counts['false']}, "
              f"none {100 * counts['none'] / n:.1f} %; err p50/p95 "
              f"{np.percentile(errs, 50) if errs else float('nan'):.2f}/"
              f"{np.percentile(errs, 95) if errs else float('nan'):.2f} mm; "
              f"ms p50/p95 {np.percentile(ms, 50):.0f}/{np.percentile(ms, 95):.0f}")
        if reasons:
            print('   none, by reason:', dict(sorted(reasons.items(), key=lambda kv: -kv[1])[:6]))
    if a.csv and rows:
        keys = sorted({k for r in rows for k in r})
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)


if __name__ == '__main__':
    main()
