#!/usr/bin/env python3
"""Summarise an align_gui auto-converge trace.

    python3 tools/fr3/analyse_trace.py                  # newest trace
    python3 tools/fr3/analyse_trace.py logs/foo.jsonl

Built to answer one question first: when a level step is commanded, does the
tilt actually change, and which joints moved? If commanded_deg is nonzero but
achieved_rot_deg is ~0, the rotation never reached the robot. If both are
nonzero but tilt_change is positive, the correction has the wrong sign. If
tilt just wanders while both look right, the marker's orientation estimate is
the problem, not the controller.
"""

import json
import pathlib
import sys

import numpy as np

ARM = [f'fr3_joint{i}' for i in range(1, 8)]


def normal_flip_check(iters):
    """Detect the planar-marker pose ambiguity.

    A small, nearly fronto-parallel planar marker has two poses that project
    almost identically - tilted the same amount in OPPOSITE directions. The
    solver picks one arbitrarily per frame, so the tilt MAGNITUDE looks
    stable while its in-plane DIRECTION flips ~180 deg. Every correction
    then undoes the previous one and levelling cannot converge, which shows
    up as the wrist joint alternating sign on every step.
    """
    N = np.array([r['marker_normal_cam'] for r in iters
                  if 'marker_normal_cam' in r])
    if len(N) < 4:
        return ''
    ang = np.degrees(np.arctan2(N[:, 1], N[:, 0]))
    d = np.abs(np.diff(ang))
    d = np.minimum(d, 360 - d)                  # wrap to 0..180
    flips = int((d > 120).sum())
    frac = flips / len(d)
    out = [f'  normal in-plane direction: {flips}/{len(d)} '
           f'frame-to-frame flips >120 deg ({frac*100:.0f}%)',
           f'  nz std {N[:, 2].std():.4f} (stable) vs '
           f'ny std {N[:, 1].std():.4f} (flipping)']
    if frac > 0.3:
        out.append('  *** PLANAR POSE AMBIGUITY: the marker normal is '
                   'flipping between the two')
        out.append('      mirror solutions. Tilt magnitude is meaningless '
                   'and levelling CANNOT')
        out.append('      converge. Fix the MEASUREMENT (bigger marker / '
                   'ArUco board / depth')
        out.append('      plane fit) - the controller is not at fault.')
    return '\n'.join(out)


def load(path):
    recs = []
    for line in pathlib.Path(path).read_text().splitlines():
        line = line.strip()
        if line:
            try:
                recs.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return recs


def main():
    if len(sys.argv) > 1:
        path = pathlib.Path(sys.argv[1])
    else:
        d = pathlib.Path(__file__).with_name('logs')
        files = sorted(d.glob('autoconverge_*.jsonl')) if d.is_dir() else []
        if not files:
            print(f'no traces in {d}')
            return
        path = files[-1]
    recs = load(path)
    if not recs:
        print(f'{path}: empty')
        return
    print(f'=== {path.name}  ({len(recs)} records) ===')

    start = next((r for r in recs if r.get('rec') == 'run_start'), {})
    end = next((r for r in recs if r.get('rec') == 'run_end'), {})
    for k in ('target_mm', 'pos_tol_mm', 'rot_tol_deg', 'step_mm',
              'rot_step_deg', 'speed_pct', 'z_floor_mm', 'cap'):
        if k in start:
            print(f'  {k:14s} {start[k]}')
    print(f'  outcome        {end.get("outcome", "?")}')

    iters = [r for r in recs if r.get('rec') == 'iter']
    if iters:
        e = np.array([r['err_mm'] for r in iters])
        t = np.array([r['tilt_deg'] for r in iters])
        print(f'\n-- {len(iters)} iterations --')
        print(f'  err  mm : first {e[0]:7.2f}  last {e[-1]:7.2f}  '
              f'min {e.min():7.2f}')
        print(f'  tilt deg: first {t[0]:7.2f}  last {t[-1]:7.2f}  '
              f'min {t.min():7.2f}  max {t.max():7.2f}')
        if len(t) > 3:
            slope = np.polyfit(np.arange(len(t)), t, 1)[0]
            print(f'  tilt trend: {slope:+.4f} deg/iter '
                  f'({"reducing" if slope < -0.01 else "NOT reducing"})')
        flip = normal_flip_check(iters)
        if flip:
            print(flip)

    levels = [r for r in recs if r.get('rec') == 'level']
    print(f'\n-- {len(levels)} level steps --')
    if levels:
        print(f'  {"cmd":>7} {"achieved":>9} {"tilt_before":>12}'
              f' {"tilt_after":>11} {"change":>8}  {"dJ5":>7} {"dJ6":>7}'
              f' {"dJ7":>7}  ok')
        for r in levels:
            jb, ja = r.get('joints_before', {}), r.get('joints_after', {})
            dj = {n: (ja.get(n, float("nan")) - jb.get(n, float("nan")))
                  for n in ARM}
            print(f'  {r.get("clamped_cmd_deg", float("nan")):7.3f}'
                  f' {r.get("achieved_rot_deg", float("nan")):9.3f}'
                  f' {r.get("tilt_before_deg", float("nan")):12.3f}'
                  f' {r.get("tilt_after_deg", float("nan")):11.3f}'
                  f' {r.get("tilt_change_deg", float("nan")):8.3f}'
                  f'  {np.degrees(dj["fr3_joint5"]):7.3f}'
                  f' {np.degrees(dj["fr3_joint6"]):7.3f}'
                  f' {np.degrees(dj["fr3_joint7"]):7.3f}  {r.get("ok")}')
        cmd = np.array([r.get('clamped_cmd_deg', np.nan) for r in levels])
        ach = np.array([r.get('achieved_rot_deg', np.nan) for r in levels])
        chg = np.array([r.get('tilt_change_deg', np.nan) for r in levels])
        print(f'\n  mean commanded {np.nanmean(cmd):.3f} deg, '
              f'mean achieved {np.nanmean(ach):.3f} deg '
              f'(ratio {np.nanmean(ach)/max(np.nanmean(cmd), 1e-9):.2f})')
        print(f'  mean tilt change {np.nanmean(chg):+.3f} deg '
              f'(want NEGATIVE)')
        # verdict
        # order matters: "barely moves" must be tested before "got worse",
        # or a tiny positive drift is misreported as a sign error
        if np.nanmean(ach) < 0.1 * np.nanmean(cmd):
            print('  VERDICT: rotation commanded but NOT executed '
                  '-> planner/controller is dropping the orientation change')
        elif abs(np.nanmean(chg)) < 0.1 * np.nanmean(cmd):
            print('  VERDICT: rotation executed but tilt barely moves '
                  '-> marker normal estimate likely dominated by noise/bias')
        elif np.nanmean(chg) > 0:
            print('  VERDICT: rotation executed but tilt got WORSE '
                  '-> sign/frame error in the correction')
        else:
            print('  VERDICT: levelling is working')

    trans = [r for r in recs if r.get('rec') == 'translate']
    print(f'\n-- {len(trans)} translate steps --')
    if trans:
        ach = np.array([r.get('achieved_trans_mm', np.nan) for r in trans])
        cmd = np.array([np.linalg.norm(r.get('cmd_d_base_mm', [np.nan] * 3))
                        for r in trans])
        print(f'  mean commanded {np.nanmean(cmd):.2f} mm, '
              f'mean achieved {np.nanmean(ach):.2f} mm '
              f'(ratio {np.nanmean(ach)/max(np.nanmean(cmd), 1e-9):.2f})')
        bad = [r for r in trans if not r.get('ok')]
        if bad:
            print(f'  {len(bad)} failed: '
                  f'{sorted({r.get("msg", "?") for r in bad})}')


if __name__ == '__main__':
    main()
