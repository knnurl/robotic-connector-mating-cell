#!/usr/bin/env python3
"""Visual regression for the window: five fixed scenes, captured offscreen.

    python3 tools/fr3/cell/visual.py             # recapture and diff (exit 1 on a change)
    python3 tools/fr3/cell/visual.py --update    # accept the current look as baseline
    python3 tools/fr3/cell/visual.py --out DIR   # keep captures and diffs in DIR

Scenes: idle, aligning (position mode), tracking (torque mode), reflex,
vision stale. Each is a logic.Snap the mock cell produces (mock_smoke.py
reaches every one of these states live); here they are frozen - fixed
clock, fixed plots, fixed log - so a pixel change means the window changed.
No ROS, no robot, no display: QT_QPA_PLATFORM=offscreen.

Baselines depend on the installed fonts (Lato, DejaVu Sans Mono): recapture
them with --update on a machine with different fonts.
"""

import argparse
import math
import os
import pathlib
import sys
import tempfile

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

import logic as L  # noqa: E402

BASELINES = HERE / 'baselines'
SIZE = (1440, 960)
NOW = 1000.0
PIXEL_TOL = 24          # per-channel difference that counts as changed
AREA_TOL = 0.002        # fraction of changed pixels that fails the check
ARM, IMP = L.ARM, L.IMP
CALIB = yaml.safe_load((HERE.parent / 'calib' / 'handeye.yaml').read_text())
COMMISSION = {'k_xy': 150.0, 'k_z': 800.0, 'k_rp': 10.0, 'k_yaw': 20.0, 'zeta': 1.0}
TRACK = {'k_xy': 1500.0, 'k_z': 1500.0, 'k_rp': 90.0, 'k_yaw': 90.0, 'zeta': 0.5}


def image(marker=True, angle=70.0, scale=1.0, off=(0, 0)):
    """A drawn camera frame: numpy only, so it is the same everywhere."""
    h, w = 480, 640
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = 38
    img[:, :, 1] += np.linspace(0, 18, w, dtype=np.uint8)[None, :]
    if marker:
        yy, xx = np.mgrid[0:h, 0:w]
        cx, cy = w / 2 + off[0], h / 2 + off[1]
        a = math.radians(angle)
        u = (xx - cx) * math.cos(a) + (yy - cy) * math.sin(a)
        v = -(xx - cx) * math.sin(a) + (yy - cy) * math.cos(a)
        r = 60 * scale
        img[(abs(u) < r) & (abs(v) < r)] = 235
        img[(abs(u) < r * 0.72) & (abs(v) < r * 0.72)] = 15
        img[(abs(v) < 2) & (u > 0) & (u < r * 1.4)] = (230, 0, 0)
        img[(abs(u) < 2) & (v > 0) & (v < r * 1.4)] = (0, 200, 0)
    return img


def series(n, f):
    return [(NOW - 30 + 30 * i / n, f(i / n)) for i in range(n)]


def marker(err, tilt, ip, dist=None):
    return {'dist_mm': dist if dist is not None else 100 + err * 0.7, 'lat_mm': err * 0.6,
            'tilt_deg': tilt, 'ip_deg': 90.0 - ip, 'ip_err_deg': ip, 'err_mm': err,
            'err_xyz_mm': [0.0, 0.0, 0.0], 'ok': err < 2 and tilt < 1 and ip < 0.5}


BASE = dict(state_age=0.02, robot_mode=L.MODE_MOVE, rt_rate=0.999, tcp=(0.452, 0.021, 0.418),
            force=(0.3, -0.2, -0.4), dq_max=0.001, joint_pct=(4, 42.0),
            controllers={ARM: 'active', IMP: 'inactive'}, floating=False, params_ok=True,
            moveit_up=True, recover_ready=True, marker_age=0.041, image_age=0.12,
            calib='loaded', track_age=0.1, track={'state': 'idle', 'policy': 'hold'},
            track_node_up=True, inplane_target=90.0,
            poses={'home': True, 'pre_align': False})


def scenes():
    torque = dict(controllers={ARM: 'inactive', IMP: 'active'}, preflight='done',
                  thresholds={'contact_n': 20.0, 'reflex_n': 40.0})
    return {
        'idle': dict(
            snap=L.Snap(**{**BASE, 'marker': marker(173.4, 3.0, 20.0, dist=269.9)}),
            hist=series(60, lambda x: (173.4, 3.0, 20.0)), image=image(scale=0.45, off=(-60, 20)),
            log=['11:02:14  Ready. Position moves run on the arm controller; TORQUE starts '
                 'with PRE-FLIGHT.', '11:02:15  calibration loaded (handeye.yaml)',
                 '11:02:15  robot mode None -> MOVE']),
        'aligning': dict(
            snap=L.Snap(**{**BASE, 'busy': 'auto_converge', 'tcp': (0.47, 0.03, 0.33),
                           'dq_max': 0.12, 'marker': marker(41.3, 0.8, 6.2)}),
            pending={'auto_converge': 12.4},
            hist=series(120, lambda x: (173 * math.exp(-2.2 * x), 3 * math.exp(-3 * x),
                                        20 * math.exp(-1.6 * x))),
            image=image(angle=84.0, scale=0.8, off=(-12, 6)),
            log=['11:04:02  [07] |e|=  52.10 mm  tilt= 1.02 deg  in-plane=+83.70 deg',
                 '11:04:03    -> executed', '11:04:04  [08] |e|=  41.30 mm  tilt= 0.80 deg'
                 '  in-plane=+83.80 deg']),
        'tracking': dict(
            snap=L.Snap(**{**BASE, **torque, 'tracking': True, 'force': (1.1, -0.8, -2.6),
                           'tcp': (0.481, 0.035, 0.247), 'marker': marker(2.1, 0.3, 0.2),
                           'track': {'state': 'tracking', 'reason': '', 'policy': 'hold',
                                     'pos_err_mm': '2.1', 'rot_err_deg': '0.31',
                                     'lead_mm': '3.4', 'lead_deg': '0.12'}}),
            fhist=series(150, lambda x: (3.0 + 0.8 * math.sin(9 * x),
                                         3.2 + 1.1 * math.sin(6 * x + 1))),
            gains=TRACK, slew=(0.025, 0.1), image=image(angle=90.0, scale=1.6),
            log=['11:06:40  hold: holding', '11:06:41  TRACKING: tracking - the arm follows '
                 'the marker. STOP NOW or END TRACK ends it.', '11:06:41  track: tracking']),
        'reflex': dict(
            snap=L.Snap(**{**BASE, **torque, 'robot_mode': L.MODE_REFLEX,
                           'errors': ('cartesian_reflex',), 'last_errors': ('cartesian_reflex',),
                           'force': (4.0, 31.0, -24.0), 'marker': marker(3.0, 0.4, 0.3),
                           'rt_rate': 0.0}),
            fhist=series(150, lambda x: (3 + 38 * max(0.0, x - 0.93) / 0.07, 2.0)),
            gains=COMMISSION, slew=(0.05, 0.2), image=image(angle=90.0, scale=1.6),
            log=['11:08:12  setpoint -20 mm along tool Z (stroke) - lead now 20.0 mm',
                 '11:08:13  robot mode MOVE -> REFLEX  (11:08:13.402)',
                 '11:08:13  a reflex stops the robot - it does not free it']),
        'vision_stale': dict(
            snap=L.Snap(**{**BASE, 'marker_age': 0.82, 'marker': None, 'image_age': 0.9,
                           'marker_why': None}),
            hist=series(60, lambda x: (12.0, 0.6, 1.5)), image=image(marker=False),
            log=['11:10:03  [03] |e|=  12.00 mm  tilt= 0.60 deg  in-plane=+88.50 deg',
                 '11:10:04  ABORT: marker lost / stale', '11:10:04  camera view enlarged']),
    }


class SceneBackend:
    """A frozen Backend for view.MainWindow (see its docstring)."""

    settings_path = None                   # frozen scenes never read the operator's file

    def __init__(self, sc):
        self.sc = sc

    def snap(self):
        return self.sc['snap']

    def now(self):
        return NOW

    def hist(self):
        return [(t, *v) for t, v in self.sc.get('hist', [])]

    def fhist(self):
        return [(t, *v) for t, v in self.sc.get('fhist', [])]

    def image(self):
        return 1, self.sc['image']

    def calib(self):
        return CALIB, 'loaded'

    def applied_gains(self):
        return dict(self.sc.get('gains', COMMISSION))

    def applied_slew(self):
        return self.sc.get('slew', (0.05, 0.2))

    def pose_info(self):
        return {'home': ('2026-09-23T10:12:00', (312.0, 24.0)), 'pre_align': (None, None)}

    def busy(self):
        return self.sc['snap'].busy

    def __getattr__(self, _name):          # command, set_param, heartbeat, ...: no-ops
        return lambda *a, **k: None


def capture(out_dir):
    import view
    from PySide6.QtCore import QCoreApplication
    st = L.load_settings(HERE / 'config' / 'settings.yaml')
    presets = yaml.safe_load((HERE / 'config' / 'gain_presets.yaml').read_text())['presets']
    app = view.make_app([])
    paths = {}
    for name, sc in scenes().items():
        win = view.MainWindow(SceneBackend(sc), st, presets)
        win.render_timer.stop()
        win.hb_timer.stop()
        win.resize(*SIZE)
        win.show()
        for k, age in sc.get('pending', {}).items():
            win.pending[k] = NOW - age
        for _ in range(4):
            QCoreApplication.processEvents()
            win.render()
        win.log_box.setPlainText('\n'.join(sc['log']))
        QCoreApplication.processEvents()
        path = out_dir / f'{name}.png'
        win.grab().save(str(path))
        paths[name] = path
        win.may_close = True
        win.close()
    app.processEvents()
    return paths


def compare(paths, out_dir):
    from PIL import Image, ImageChops
    failed = []
    for name, path in paths.items():
        base = BASELINES / f'{name}.png'
        if not base.exists():
            print(f'NO BASELINE {name} - run with --update')
            failed.append(name)
            continue
        a, b = Image.open(base).convert('RGB'), Image.open(path).convert('RGB')
        if a.size != b.size:
            print(f'FAIL {name}: size {b.size} != baseline {a.size}')
            failed.append(name)
            continue
        diff = np.asarray(ImageChops.difference(a, b)).max(axis=2)
        frac = float((diff > PIXEL_TOL).mean())
        ok = frac <= AREA_TOL
        if not ok:
            mask = Image.fromarray(((diff > PIXEL_TOL) * 255).astype(np.uint8))
            mask.save(out_dir / f'{name}_diff.png')
            failed.append(name)
        print(f'{"PASS" if ok else "FAIL"} {name}: {frac*100:.3f}% of pixels changed')
    return failed


def main():
    ap = argparse.ArgumentParser(description='cell panel visual regression')
    ap.add_argument('--update', action='store_true', help='write the baselines')
    ap.add_argument('--out', help='where captures and diffs go (default: a temp dir)')
    args = ap.parse_args()
    out = pathlib.Path(args.out or tempfile.mkdtemp(prefix='fr3_cell_visual_'))
    out.mkdir(parents=True, exist_ok=True)
    if args.update:
        BASELINES.mkdir(exist_ok=True)
        for name, path in capture(BASELINES).items():
            print(f'baseline {name}: {path}')
        return 0
    failed = compare(capture(out), out)
    print(f'captures in {out}' + (f' - FAILED: {", ".join(failed)}' if failed else ''))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
