#!/usr/bin/env python3
"""Headless run of the panel's ROS side against the mock cell: every command path.

    python3 tools/fr3/cell/mock_smoke.py      # test_v2_mock_smoke.py runs it

The real V2Node and Cell (no window) drive mock_cell.py through ALIGN,
PRE-FLIGHT, HOLD, SETPOINT, gains, speed, TRACK, marker loss, STOP NOW,
RELEASE, a reflex and its recovery, a driver restart, the recorder and a
saved pose - checking the decisions (logic.py) at each step, as the window
would see them. Node-level plumbing is invisible to the unit tests; this is
where it shows (the tracking_smoke.py lesson).

Isolated on domain 88 (isolate.py); traces, bags and taught poses go to a
temp directory, never the repo. Exit code 0 only if every check passes.
"""

import isolate
isolate.isolate()

import os  # noqa: E402
import pathlib  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import tempfile  # noqa: E402
import time  # noqa: E402

TMP = tempfile.mkdtemp(prefix='fr3_cell_smoke_')
os.environ['FR3_LOG_DIR'] = TMP
HERE = pathlib.Path(__file__).resolve().parent

import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import actions  # noqa: E402
import ros_node  # noqa: E402
import logic as L  # noqa: E402

actions.POSES_PATH = pathlib.Path(TMP) / 'poses.yaml'
actions.POSES_PATH.write_text('# smoke\nhome: null\n')

results = []


def check(name, ok, detail=''):
    results.append(bool(ok))
    print(('PASS ' if ok else 'FAIL ') + name + (f'  [{detail}]' if detail else ''),
          flush=True)


def main():
    mock_log = open(pathlib.Path(TMP) / 'mock_cell.out', 'w')
    mock = subprocess.Popen([sys.executable, str(HERE / 'mock_cell.py')],
                            stdout=mock_log, stderr=subprocess.STDOUT)
    st = L.load_settings(HERE / 'config' / 'settings.yaml')
    events = []
    node, executor, spin = ros_node.spin_up()
    cell = actions.Cell(node, st, emit=lambda *e: events.append(e))
    fault = node.create_publisher(String, '/mock_cell/fault', 10)
    snap = [None]

    def tick():
        s = cell.snapshot()
        cell.tick(s)
        snap[0] = s
        return s

    def wait_for(pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = tick()
            if pred(s):
                return True
            time.sleep(0.1)
        return False

    def run(name, fn, *a, timeout=60.0):
        """Start through Cell.run, as a button press does; wait for done."""
        events.clear()
        if not cell.run(name, fn, *a):
            return False, 'refused: busy'
        wait_for(lambda s: any(e[0] == 'done' and e[1] == name for e in events), timeout)
        done = [e for e in events if e[0] == 'done' and e[1] == name]
        return (done[0][2], done[0][3]) if done else (False, 'no done event')

    def say(f):
        fault.publish(String(data=f))

    def en(name):
        return L.enable(snap[0], st)[name]

    try:
        up = wait_for(lambda s: L.fresh(s, st) and s.controllers is not None
                      and s.marker is not None and s.moveit_up, 30)
        check('mock up: robot state, controllers, marker, MoveIt', up,
              L.banner(snap[0], st).detail)
        check('status bar quiet on a healthy cell',
              all(c.level == L.NORMAL for c in L.chips(snap[0], st)
                  if c.label != 'PRE-FLIGHT'), str(L.chips(snap[0], st)))
        check('heartbeat publishes', node.count_publishers('/cell_panel/heartbeat') >= 1)
        node.heartbeat()
        check('calibration loads once TF is up',
              wait_for(lambda s: (cell.try_autoload() or True) and s.calib == 'loaded', 10),
              cell.calib_state)
        check('joint limits from /robot_description',
              wait_for(lambda s: node.joint_limits()[1] == '/robot_description', 5),
              str(node.joint_limits()[1]))
        s = tick()
        check('mode POSITION, PRE-FLIGHT is the next step',
              L.mode(s) == 'POSITION' and L.next_step(s, st, L.enable(s, st)) == 'preflight',
              L.mode(s))
        check('TRACK refused before torque', not en('track').ok, en('track').why)

        # ---- ALIGN
        cell.params = actions.Params(speed_pct=100.0)
        e0 = snap[0].marker['err_mm']
        ok, msg = run('translate', cell.start_align, 'translate')
        e1 = tick().marker['err_mm']
        check('one translate step closes the error', ok and e1 < e0 - 5, f'{e0:.1f} -> {e1:.1f} mm, {msg}')
        ok, msg = run('auto_converge', cell.start_align, 'auto_converge', timeout=120)
        s = tick()
        check('AUTO-CONVERGE converges', ok and msg == 'converged' and s.marker['err_mm'] < 3.0,
              f'{msg}, |e| {s.marker["err_mm"]:.2f} mm tilt {s.marker["tilt_deg"]:.2f}')
        check('converge trace written', any(pathlib.Path(TMP).rglob('cell_*.jsonl')))

        # ---- PRE-FLIGHT and HOLD
        check('PRE-FLIGHT enabled with the arm still', wait_for(lambda s: en('preflight').ok, 5),
              en('preflight').why)
        ok, msg = run('preflight', cell.preflight_run)
        check('PRE-FLIGHT done', ok and cell.preflight == 'done', msg)
        check('arm controller restored after PRE-FLIGHT',
              wait_for(lambda s: L.mode(s) == 'POSITION', 5), L.mode(snap[0]))
        ok, msg = run('hold', cell.hold_on)
        check('HOLD activates impedance', ok and wait_for(lambda s: L.mode(s) == 'TORQUE', 5), msg)
        check('HOLD reads back float_mode false',
              wait_for(lambda s: s.floating is False, 5), str(snap[0].floating))
        check('activation writes the speed, capped at the confirm threshold (25 %)',
              wait_for(lambda s: abs((node.applied_params() or {}).get(
                  'setpoint_slew_mps', 0) - 0.0625) < 1e-9, 5), str(node.applied_params()))
        cell.params = actions.Params(speed_pct=20.0)
        ok, msg = run('speed', cell.write_speed, 20.0)
        check('speed slider writes the controller slew',
              wait_for(lambda s: abs(node.applied_params()['setpoint_slew_mps'] - 0.05) < 1e-9, 5),
              str(node.applied_params()))
        s = tick()
        check('TRACK is the next step', L.next_step(s, st, L.enable(s, st)) == 'track',
              L.enable(s, st)['track'].why)

        # ---- SETPOINT, hold HERE, gains
        z0 = tick().tcp[2]
        ok, msg = run('setpoint_plus', cell.setpoint_step, 1.0)
        check('SETPOINT + moves the arm up 10 mm',
              ok and wait_for(lambda s: s.tcp[2] - z0 > 0.009, 5),
              f'{(snap[0].tcp[2]-z0)*1000:.1f} mm, {msg}')
        ok, msg = run('hold_here', cell.hold_here)
        check('hold HERE', ok, msg)
        g = {'k_xy': 300.0, 'k_z': 1200.0, 'k_rp': 20.0, 'k_yaw': 30.0, 'zeta': 0.9}
        cell.pending_gains = dict(g)
        check('edited gains read as pending', tick().gains_pending)
        ok, msg = run('apply_gains', cell.apply_gains, g)
        check('gains applied and read back',
              ok and wait_for(lambda s: not s.gains_pending, 5), str(cell.applied_gains()))
        say('push')
        check('a 12 N push blocks presets',
              wait_for(lambda s: not L.enable(s, st)['preset'].ok, 3), en('preset').why)
        say('clear')
        wait_for(lambda s: L.enable(s, st)['preset'].ok, 3)

        # ---- TRACK, marker loss, STOP NOW
        pre_track = node.applied_params()['setpoint_slew_mps']
        ok, msg = run('track', cell.start_tracking)
        check('TRACK starts', ok and wait_for(lambda s: s.track.get('state') == 'tracking', 5),
              msg)
        want = L.speed_torque(cell.params.track_speed_pct, st)['setpoint_slew_mps']
        check('TRACK SPEED overrides the node profile (100 mm/s) after START',
              wait_for(lambda s: abs(node.applied_params()['setpoint_slew_mps'] - want) < 1e-9,
                       3), f'{node.applied_params()["setpoint_slew_mps"]} vs {want}')
        cell.params = actions.Params(track_speed_pct=5.0)
        ok, msg = run('track_speed', cell.write_speed, 5.0)
        check('TRACK SPEED is live while tracking',
              ok and wait_for(lambda s: abs(node.applied_params()['setpoint_slew_mps']
                                            - 0.0125) < 1e-9, 3), msg)
        check('gains and speed lock while tracking',
              not en('preset').ok and not en('speed').ok, en('speed').why)
        cell.params = actions.Params(marker_loss='stop', marker_loss_ms=500.0)
        say('marker_lost')
        check('marker loss -> GUI-side stop after 500 ms',
              wait_for(lambda s: s.track.get('state') == 'idle' and not cell.tracking, 6),
              str(snap[0].track.get('state')))
        check('STOP puts the pre-TRACK slew back',
              wait_for(lambda s: abs(node.applied_params()['setpoint_slew_mps']
                                     - pre_track) < 1e-9, 3),
              f'{node.applied_params()["setpoint_slew_mps"]} vs {pre_track}')
        say('clear')
        wait_for(lambda s: s.marker is not None, 3)
        ok, msg = run('track', cell.start_tracking)
        wait_for(lambda s: s.track.get('state') == 'tracking', 5)
        cell.stop_now()
        check('STOP NOW ends tracking',
              wait_for(lambda s: s.track.get('state') == 'idle' and not cell.tracking, 25),
              str(snap[0].track.get('state')))
        check('STOP NOW leaves the arm held on impedance', L.mode(tick()) == 'TORQUE')

        # ---- RELEASE
        ok, msg = run('release', cell.release)
        check('RELEASE hands the arm back', ok and wait_for(lambda s: L.mode(s) == 'POSITION', 5),
              msg)

        # ---- reflex and recovery
        say('reflex')
        check('reflex -> banner with RECOVER',
              wait_for(lambda s: L.banner(s, st).key == 'reflex'
                       and L.banner(s, st).action == 'recover', 5),
              L.banner(snap[0], st).title)
        check('reflex greys out ALIGN', not en('translate').ok, en('translate').why)
        ok, msg = run('recover', cell.recover)
        check('RECOVER clears the reflex',
              ok and wait_for(lambda s: not L.robot_error(s, st), 5), msg)

        # ---- driver restart voids PRE-FLIGHT
        say('driver_down')
        check('driver down -> banner', wait_for(lambda s: L.banner(s, st).key == 'driver_down', 8),
              L.banner(snap[0], st).key)
        check('driver down voids PRE-FLIGHT', cell.preflight == 'needed', cell.preflight)
        say('clear')
        check('driver back -> READY', wait_for(lambda s: L.banner(s, st).key == 'ready', 15),
              L.banner(snap[0], st).detail)

        # ---- vision events
        say('pose_jump')
        check('a pose jump is a vision event',
              wait_for(lambda s: (L.vision_event(s, st) or '').startswith('marker pose jumped'), 3),
              str(snap[0].jump_mm))
        say('vision_stale')
        check('stale vision -> banner', wait_for(lambda s: L.banner(s, st).key == 'vision', 3),
              L.banner(snap[0], st).key)
        say('clear')
        wait_for(lambda s: L.marker_fresh(s, st), 3)

        # ---- saved pose
        ok, msg = run('teach', cell.teach, 'home')
        check('TEACH home', ok and tick().poses.get('home'), msg)
        cell.params = actions.Params(speed_pct=100.0)
        ok, msg = run('setpoint', lambda: cell.n.move([0.0, 0.03, 0.02],
                                                        slowdown=5.0), timeout=20)
        dist = cell.pose_distance('home')
        ok, msg = run('goto:home', cell.goto_pose, 'home', timeout=30)
        back = cell.pose_distance('home')
        check('go to home returns there', ok and back[0] < 2.0,
              f'{dist[0]:.0f} mm away -> {back[0]:.1f} mm, {msg}')

        # ---- recorder
        ok, msg = cell.toggle_record()
        time.sleep(2.0)
        ok2, msg2 = cell.toggle_record()
        bags = list(pathlib.Path(TMP).rglob('bag_*'))
        check('rosbag records and closes', ok and ok2 and bags, msg2)
    finally:
        cell.recorder.stop()
        mock.terminate()
        try:
            mock.wait(timeout=10)
        except subprocess.TimeoutExpired:
            mock.kill()
        node.close()
        executor.shutdown()
        rclpy.try_shutdown()
    check('mock cell alive throughout', mock.returncode in (None, 0, -15),
          f'exit {mock.returncode}')
    print(f'{sum(results)}/{len(results)} checks passed (logs: {TMP})')
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
