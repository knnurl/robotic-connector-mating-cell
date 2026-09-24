#!/usr/bin/env python3
"""FR3 Cell Control - one page for align, impedance and tracking.

    fr3_cell                 # terminal 2 (fr3_env.sh): tools/fr3/fr3_cell.launch.py
    fr3_cell mock:=true      # no robot: mock_cell.py on the isolated domain 88
    python3 tools/fr3/cell/cell.py [--mock]   # the window alone

It runs as the node 'cell_panel' and refuses to start while any other
cell_panel is up: two panels must never drive one arm.

Every motion is still a button press, nothing here moves on its own, and
none of it replaces the hardware E-stop or the enabling device.
"""

import argparse
import dataclasses
import pathlib
import signal
import sys
import threading
import time

HERE = pathlib.Path(__file__).resolve().parent
DISCOVERY_S = 2.0            # wait this long for the graph before the one-panel check


def load_presets(path, limits):
    import yaml
    import logic
    doc = yaml.safe_load(path.read_text())
    problems = logic.validate_presets(doc['presets'], limits)
    if problems:
        raise SystemExit(f'{path}: ' + '; '.join(problems))
    return doc['presets']


class LiveBackend:
    """The window's data source and command sink, on the real ROS graph."""

    def __init__(self, node, cell, mock):
        from std_msgs.msg import String
        self.n, self.cell, self.mock = node, cell, mock
        self._String = String
        self.fault_pub = node.create_publisher(String, '/mock_cell/fault', 10) if mock else None
        import persist
        self.settings_path = persist.path(mock)       # the drawer, kept across restarts
        self._autoload_at = 0.0
        self.camera(True)            # the marker view is on by default

    def attach(self, window):
        self.cell.emit = window.post

    # ---- data
    def snap(self):
        s = self.cell.snapshot()
        self.cell.tick(s)
        if s.calib != 'loaded' and time.monotonic() - self._autoload_at > 2.0:
            self._autoload_at = time.monotonic()
            self.cell.try_autoload()
        return s

    def now(self):
        return time.monotonic()

    def hist(self):
        return list(self.cell.hist)

    def fhist(self):
        return list(self.cell.fhist)

    def image(self):
        return self.n.image()

    def calib(self):
        return self.cell.calib_meta, self.cell.calib_state

    def applied_gains(self):
        return self.cell.applied_gains()

    def applied_slew(self):
        p = self.n.applied_params()
        return None if p is None else (p['setpoint_slew_mps'], p['setpoint_slew_rps'])

    def pose_info(self):
        import actions
        out = {}
        for name, v in actions.load_poses_cached().items():
            out[name] = (None, None) if not v else (v.get('taught', '?'),
                                                    self.cell.pose_distance(name))
        return out

    def busy(self):
        return self.cell.busy

    # ---- inputs
    def set_param(self, name, value):
        self.cell.params = dataclasses.replace(self.cell.params, **{name: value})

    def set_settings(self, st):
        self.cell.st = st

    def set_pending_gains(self, values):
        self.cell.pending_gains = values

    def heartbeat(self):
        self.n.heartbeat()

    def camera(self, on):
        self.cell.camera_on = self.n.camera(on)

    def fault(self, name):
        if self.fault_pub is not None:
            self.fault_pub.publish(self._String(data=name))

    def command(self, name, *a):
        c = self.cell
        runs = {'preflight': (c.preflight_run,), 'float': (c.float_on,),
                'hold': (c.hold_on,), 'setpoint_minus': (c.setpoint_step, -1.0),
                'setpoint_plus': (c.setpoint_step, 1.0), 'hold_here': (c.hold_here,),
                'track': (c.start_tracking,), 'release': (c.release,),
                'recover': (c.recover,), 'reload_calib': (c.load_calib,),
                'auto_floor': (c.auto_floor,)}
        if name in ('translate', 'level', 'inplane', 'auto_converge'):
            c.run(name, c.start_align, name)
        elif name.startswith('goto:'):
            c.run(name, c.goto_pose, name.split(':', 1)[1])
        elif name.startswith('teach:'):
            c.run(name, c.teach, name.split(':', 1)[1])
        elif name in runs:
            c.run(name, *runs[name])
        elif name == 'end_track':
            c._thread('track', c.stop_tracking)       # a stop: never behind busy
        elif name == 'apply_gains':
            c.run(name, c.apply_gains, dict(c.pending_gains or {}))
        elif name == 'speed':
            c.run(name, c.write_speed, a[0])
        elif name == 'track_speed':
            # Stored for the next START; while tracking it is written live.
            self.set_param('track_speed_pct', float(a[0]))
            if c.tracking or c._track_state() in ('starting', 'tracking', 'holding'):
                c.run('track_speed', c.write_speed, float(a[0]))
        elif name == 'stop_now':
            c.stop_now()
        elif name == 'pause':
            c.pause()
        elif name == 'stop_after':
            c.stop_after()
        elif name == 'record':
            c._thread('record', c.toggle_record)
        elif name == 'policy':
            self.set_param('over_lead', a[0])
            threading.Thread(target=c.write_policy, args=(a[0],), daemon=True).start()
        elif name == 'gate':
            c.set_gate(a[0])
        elif name == 'torque_attempt':
            c.note_torque_attempt()
        elif name == 'dismiss':
            c.failure = None
        elif name == 'close_handoff':
            c._thread('close_handoff', self._close_handoff)
        else:
            c.say(f'unknown command {name}')

    def _close_handoff(self):
        """the Tk panel's on_close: never leave the arm compliant behind a closed window."""
        import core
        self.cell.recorder.stop()
        result = core.release_if_active(self.n, self.cell.say)
        st = self.n.state()
        age = float('inf') if st is None else st[5]
        if result is False or (result is None and age <= core.DRIVER_DOWN_S):
            return False, ('cannot confirm the impedance controller is released - NOT '
                           'closing. Press RELEASE, or use the E-stop')
        self.cell.close_trace()
        return True, ''


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--mock', action='store_true',
                    help='talk to mock_cell.py on the isolated domain 88')
    args = ap.parse_args()
    if args.mock:
        import isolate
        isolate.isolate()

    import actions
    import ros_node
    import logic
    import view
    import core

    st = logic.load_settings(HERE / 'config' / 'settings.yaml')
    presets = load_presets(HERE / 'config' / 'gain_presets.yaml', core.GAIN_LIMITS)
    node, executor, spin = ros_node.spin_up()
    time.sleep(DISCOVERY_S)
    others = node.other_panels()
    if others:
        # Refuse WITHOUT the exit handoff: it would release an arm the other
        # panel holds.
        print(f'cell: {others} other cell_panel node(s) on the graph - another panel is '
              'running. Close it first.',
              file=sys.stderr)
        node.close()
        executor.shutdown()
        rclpy_shutdown()
        return 2

    cell = actions.Cell(node, st)
    backend = LiveBackend(node, cell, args.mock)
    app = view.make_app(sys.argv)
    win = view.MainWindow(backend, st, presets, mock=args.mock)
    backend.attach(win)
    esc = view.EscapeFilter(win)
    app.installEventFilter(esc)

    def on_signal(_sig, _frame):
        from PySide6.QtCore import QTimer
        QTimer.singleShot(0, win.close)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, on_signal)
    from PySide6.QtCore import QTimer
    keepalive = QTimer(interval=200, timeout=lambda: None)   # lets Python see signals
    keepalive.start()

    win.log('Ready. Position moves run on the arm controller; TORQUE starts with '
            'PRE-FLIGHT. Esc = STOP NOW from anywhere.' + ('  MOCK CELL - no robot.'
                                                         if args.mock else ''))
    if node.relay_error:
        win.log(f'WARNING: robot state relay failed ({node.relay_error})')
    win.show()
    try:
        app.exec()
    finally:
        cell.recorder.stop()
        core.shutdown_ros(node, cell, executor, spin)
    return 0


def rclpy_shutdown():
    import rclpy
    rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
