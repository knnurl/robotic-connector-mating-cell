#!/usr/bin/env python3
"""Align the wrist camera to a target pose above an ArUco marker.

No hand-eye calibration needed. Three small probe moves recover the
camera->base ROTATION empirically, which is all a position servo requires:
the unknown camera->TCP translation cancels out of the translation math.

    camera moves by d (base coords)  =>  marker shifts by  -R @ d  in the
    camera frame, where R = R_cam_base. Probing along each base axis by h
    gives  col_i(R) = -(p_after - p_before) / h.

    To drive the marker to target t:  d = R.T @ (p_marker - t)

Every motion needs an explicit button press. Steps are clamped, the TCP is
bounded by an absolute base-frame Z floor, and speed is capped.

Run (needs vision publishing /aruco/pose, and MoveIt up):

    python3 tools/fr3/align_gui.py
"""

import datetime
import json
import pathlib
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext, ttk

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

# Shared dark-dashboard palette - single source of truth is the operator
# panel, so the two GUIs cannot drift apart.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'gui'))
from mating_panel import THEME as T, mix, text_on          # noqa: E402

# ---- selectable step sizes ----------------------------------------------
STEP_MM_CHOICES = ['0.5', '1', '2', '5', '10', '20', '30', '50']
ROT_DEG_CHOICES = ['0.25', '0.5', '1', '2', '3', '5']
PROBE_MM_CHOICES = ['5', '10', '15', '20', '30']
STEP_MM_DEFAULT, ROT_DEG_DEFAULT, PROBE_MM_DEFAULT = '30', '3', '15'

# ---- selectable speed ----------------------------------------------------
# Percentage of the trajectory speed MoveIt planned. Applied by stretching
# time_from_start (this GetCartesianPath has no velocity-scaling field, so
# setting one in the request would silently do nothing).
SPEED_PCT_CHOICES = ['1', '2', '5', '10', '20']
SPEED_PCT_DEFAULT = '5'
CEIL_SPEED_PCT = 20.0     # never faster than this, whatever is selected

# ---- target standoff + convergence --------------------------------------
TARGET_MM_CHOICES = ['80', '100', '150', '200', '250', '300']
TARGET_MM_DEFAULT = '100'
POS_TOL_MM_CHOICES = ['1', '2', '3', '5']
POS_TOL_MM_DEFAULT = '2'
ROT_TOL_DEG = 1.0         # 'parallel' tolerance for auto-converge
MAX_ITERS_ABS = 400       # absolute backstop; the real cap is step-scaled
NO_PROGRESS_LIMIT = 4     # abort if the error stops improving

# ---- hard safety ceilings (a selection can never exceed these) ----------
CEIL_STEP_M = 0.050
CEIL_ROT_DEG = 5.0
CEIL_PROBE_M = 0.030

MIN_FRACTION = 0.95       # reject incomplete Cartesian paths
POSE_STALE_S = 0.5        # marker measurement must be fresher than this
FLOOR_MARGIN_M = 0.020    # extra clearance under the predicted end pose

# Saved calibration. Holds the camera->TCP ROTATION, which is a rigid
# mounting property: unlike R_cam_base it does not change when the robot
# moves, so it stays valid across sessions and arm poses.
CALIB_PATH = pathlib.Path(__file__).with_name('handeye_rotation.json')

# Per-iteration JSONL trace of auto-converge: what was measured, what was
# commanded, what the robot actually did. One file per run.
LOG_DIR = pathlib.Path(__file__).with_name('logs')


def q2R(x, y, z, w):
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = np.array([x, y, z, w]) / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R2q(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
            y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
            y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
            y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def axis_angle_R(axis, ang):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


class AlignNode(Node):
    def __init__(self):
        super().__init__('camera_align_gui')
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('tcp_link', 'fr3_hand_tcp')
        self.declare_parameter('group', 'fr3_arm')
        self.base = self.get_parameter('base_frame').value
        self.tcp = self.get_parameter('tcp_link').value
        self.group = self.get_parameter('group').value

        self._lock = threading.Lock()
        self._pose = None          # (pos(3), R_cam_marker, stamp_s)
        self._joints = None        # (names, positions)
        self.create_subscription(PoseStamped, '/aruco/pose', self._cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_cb, 10)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.cart = self.create_client(GetCartesianPath,
                                       '/compute_cartesian_path')
        self.exec_ac = ActionClient(self, ExecuteTrajectory,
                                    '/execute_trajectory')
        # MoveIt's TrajectoryExecutionManager subscribes here and calls
        # stopExecution() on "stop" - this halts a trajectory MID-motion,
        # unlike cancelling the action goal which can let it run out.
        self.exec_event = self.create_publisher(
            String, '/trajectory_execution_event', 10)
        self._goal_handle = None
        self.last_cmd = {}         # filled by move(), read by the trace log

    def _cb(self, m):
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        o = m.pose.orientation
        with self._lock:
            self._pose = (p, q2R(o.x, o.y, o.z, o.w),
                          self.get_clock().now().nanoseconds * 1e-9)

    def _joint_cb(self, m):
        with self._lock:
            self._joints = (list(m.name), list(m.position))

    def joints(self):
        """Arm joint positions as {name: rad}, or {} if not yet seen."""
        with self._lock:
            j = self._joints
        return {} if j is None else dict(zip(j[0], j[1]))

    def marker(self):
        """Fresh marker measurement, or None if missing/stale."""
        with self._lock:
            if self._pose is None:
                return None
            p, R, t = self._pose
        if self.get_clock().now().nanoseconds * 1e-9 - t > POSE_STALE_S:
            return None
        return p, R

    def tcp_pose(self):
        tf = self.tf_buf.lookup_transform(self.base, self.tcp,
                                          rclpy.time.Time())
        tr, ro = tf.transform.translation, tf.transform.rotation
        return (np.array([tr.x, tr.y, tr.z]),
                q2R(ro.x, ro.y, ro.z, ro.w))

    def move(self, d_base, R_delta=None, z_floor=None, slowdown=20.0):
        """Cartesian move of the TCP. Returns (ok, message).

        z_floor is an absolute base-frame minimum for the TCP, checked
        against the goal BEFORE planning, so a bad vision scale or a sign
        error cannot drive the arm down past it.
        """
        pos, Rt = self.tcp_pose()
        target = Pose()
        tp = pos + np.asarray(d_base)
        self.last_cmd = {
            'tcp_pos_before': pos.tolist(),
            'tcp_quat_before': R2q(Rt).tolist(),
            'joints_before': self.joints(),
            'goal_pos': tp.tolist(),
        }
        if z_floor is not None and tp[2] < z_floor:
            return False, (f'BLOCKED by Z floor: goal z={tp[2]*1000:.1f} mm '
                           f'< floor {z_floor*1000:.1f} mm')
        target.position.x, target.position.y, target.position.z = tp
        Rgoal = (R_delta @ Rt) if R_delta is not None else Rt
        q = R2q(Rgoal)
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = q
        self.last_cmd['goal_quat'] = q.tolist()
        # how much reorientation was actually requested, in degrees
        self.last_cmd['goal_rot_deg'] = float(np.degrees(np.arccos(
            np.clip((np.trace(Rgoal @ Rt.T) - 1.0) / 2.0, -1, 1))))

        if not self.cart.wait_for_service(timeout_sec=3.0):
            return False, 'compute_cartesian_path unavailable'
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base
        req.group_name = self.group
        req.link_name = self.tcp
        req.waypoints = [target]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        fut = self.cart.call_async(req)
        if not self._wait(fut, 10.0):
            return False, 'Cartesian planning timed out'
        res = fut.result()
        if res is None:
            return False, 'Cartesian planning call failed'
        if res.fraction < MIN_FRACTION:
            return False, f'path only {res.fraction * 100:.0f}% complete'

        traj = res.solution
        for pt in traj.joint_trajectory.points:
            tot = (pt.time_from_start.sec
                   + pt.time_from_start.nanosec * 1e-9) * slowdown
            pt.time_from_start.sec = int(tot)
            pt.time_from_start.nanosec = int((tot - int(tot)) * 1e9)
            pt.velocities = [v / slowdown for v in pt.velocities]
            pt.accelerations = [a / slowdown ** 2 for a in pt.accelerations]

        if not self.exec_ac.wait_for_server(timeout_sec=3.0):
            return False, 'execute_trajectory unavailable'
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        gfut = self.exec_ac.send_goal_async(goal)
        if not self._wait(gfut, 10.0):
            return False, 'goal send timed out'
        gh = gfut.result()
        if gh is None or not gh.accepted:
            return False, 'trajectory goal rejected'
        self._goal_handle = gh
        rfut = gh.get_result_async()
        ok = self._wait(rfut, 300.0)
        self._goal_handle = None
        if not ok or rfut.result() is None:
            return False, 'no result (timeout)'
        code = rfut.result().result.error_code.val
        try:
            apos, aRt = self.tcp_pose()
            self.last_cmd['tcp_pos_after'] = apos.tolist()
            self.last_cmd['tcp_quat_after'] = R2q(aRt).tolist()
            self.last_cmd['joints_after'] = self.joints()
            # achieved vs requested reorientation - the key diagnostic for
            # "the level step ran but tilt did not change"
            self.last_cmd['achieved_rot_deg'] = float(np.degrees(np.arccos(
                np.clip((np.trace(aRt @ Rt.T) - 1.0) / 2.0, -1, 1))))
            self.last_cmd['achieved_trans_mm'] = float(
                np.linalg.norm(apos - pos) * 1000)
        except Exception:                                   # noqa: BLE001
            pass
        return (code == 1), ('executed' if code == 1
                             else f'execution error code {code}')

    def stop(self):
        """Graceful: cancel the goal (may still finish the current segment)."""
        gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()
            return True
        return False

    def stop_now(self):
        """Instant: halt MoveIt execution mid-trajectory, then cancel.

        Publishing "stop" makes TrajectoryExecutionManager call
        stopExecution(), which stops the controller where it is instead of
        letting the queued trajectory play out. NOT a substitute for the
        hardware E-stop.
        """
        msg = String()
        msg.data = 'stop'
        for _ in range(3):          # cheap, and the topic is best-effort
            self.exec_event.publish(msg)
        gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()
        return True

    @staticmethod
    def _wait(fut, timeout_s):
        """Wait on a future completed by the background spinner."""
        end = time.time() + timeout_s
        while time.time() < end:
            if fut.done():
                return True
            time.sleep(0.02)
        return False


class Gui:
    def __init__(self, node):
        self.n = node
        self.R = None                 # R_cam_base once calibrated
        self.cols = {}                # axis index -> column of R
        self.busy = False
        self.abort = False            # set by STOP, polled by auto_converge
        self.tracef = None            # open trace file during auto-converge
        self.tracepath = None

        self.root = tk.Tk()
        self.root.title('FR3 Camera Alignment')
        self.root.configure(bg=T['page'])

        base = tkfont.nametofont('TkDefaultFont').actual()['family']
        self.f_caption = (base, 9)
        self.f_label = (base, 10)
        self.f_value = ('DejaVu Sans Mono', 17, 'bold')
        self.f_small = ('DejaVu Sans Mono', 10)
        self.f_button = (base, 10, 'bold')

        self._init_combo_style()

        outer = tk.Frame(self.root, bg=T['page'])
        outer.pack(fill='both', expand=True, padx=16, pady=14)
        tk.Label(outer, text='FR3 CAMERA <-> MARKER ALIGNMENT',
                 font=(base, 10, 'bold'), fg=T['muted'], bg=T['page'],
                 anchor='w').pack(fill='x')

        self._build_readout(outer)
        cfg = tk.Frame(outer, bg=T['page'])
        cfg.pack(fill='x', pady=(12, 0))
        self._build_target(cfg)
        self._build_motion(cfg)
        self._build_calib(outer)
        self._build_align(outer)

        stoprow = tk.Frame(outer, bg=T['page'])
        stoprow.pack(fill='x', pady=(12, 0))
        self.estopb = self._button(stoprow, 'STOP NOW', T['critical'],
                                   self.stop_now, font=(base, 14, 'bold'),
                                   pady=14)
        self.estopb.pack(side='left', expand=True, fill='both', padx=(0, 8))
        self.stopb = self._button(stoprow, 'stop after\ncurrent move',
                                  T['serious'], self.stop,
                                  font=(base, 10, 'bold'), pady=8)
        self.stopb.pack(side='left', fill='both')

        logcard = self._card(outer, 'LOG')
        logcard.pack(fill='both', expand=True, pady=(12, 0))
        self.log = scrolledtext.ScrolledText(
            logcard, height=10, font=self.f_small, bg=T['page'],
            fg=T['ink2'], insertbackground=T['ink'], relief='flat',
            bd=0, highlightthickness=0)
        self.log.pack(fill='both', expand=True, padx=10, pady=(0, 10))

        self.status = tk.Label(outer, text='ready', anchor='w',
                               fg=T['muted'], bg=T['page'],
                               font=self.f_caption)
        self.status.pack(fill='x', pady=(6, 0))

        self.say('Ready. Calibrate all three probes before aligning.')
        self.root.after(1500, self._autoload)
        self.relabel()
        self.tick()

    # ------------------------------------------------------------ chrome

    def _init_combo_style(self):
        st = ttk.Style()
        try:
            st.theme_use('clam')
        except tk.TclError:
            pass
        st.configure('Dark.TCombobox', fieldbackground=T['surface'],
                     background=T['grid'], foreground=T['ink'],
                     arrowcolor=T['ink2'], bordercolor=T['grid'],
                     lightcolor=T['grid'], darkcolor=T['grid'])
        st.map('Dark.TCombobox',
               fieldbackground=[('readonly', T['surface'])],
               foreground=[('readonly', T['ink'])])
        for k, v in (('background', T['surface']), ('foreground', T['ink']),
                     ('selectBackground', T['series']),
                     ('selectForeground', T['ink'])):
            self.root.option_add(f'*TCombobox*Listbox.{k}', v)

    def _card(self, parent, title):
        frame = tk.Frame(parent, bg=T['surface'],
                         highlightbackground=T['grid'], highlightthickness=1)
        tk.Label(frame, text=title, font=self.f_caption, fg=T['muted'],
                 bg=T['surface'], anchor='w').pack(fill='x', padx=10,
                                                   pady=(8, 2))
        return frame

    def _button(self, parent, text, color, cmd, font=None, pady=9):
        fg = text_on(color)
        b = tk.Button(parent, text=text, font=font or self.f_button,
                      bg=color, fg=fg,
                      activebackground=mix(color, '#ffffff', .15),
                      activeforeground=fg, relief='flat', bd=0,
                      highlightthickness=0, cursor='hand2', padx=14,
                      pady=pady, command=cmd,
                      disabledforeground=T['muted'])
        b.bind('<Enter>', lambda e, w=b, c=color:
               w['state'] == 'normal' and w.config(bg=mix(c, '#ffffff', .12)))
        b.bind('<Leave>', lambda e, w=b, c=color: w.config(bg=c))
        return b

    def _combo(self, parent, var, vals, width=6):
        cb = ttk.Combobox(parent, textvariable=var, values=vals, width=width,
                          state='readonly', style='Dark.TCombobox',
                          font=self.f_small)
        cb.bind('<<ComboboxSelected>>', lambda _e: self.relabel())
        return cb

    def _row(self, parent, label, var, vals, r, c):
        tk.Label(parent, text=label, font=self.f_caption, fg=T['ink2'],
                 bg=T['surface'], anchor='e').grid(row=r, column=c * 2,
                                                   padx=(10, 4), pady=4,
                                                   sticky='e')
        self._combo(parent, var, vals).grid(row=r, column=c * 2 + 1,
                                            padx=(0, 10), pady=4, sticky='w')

    # ------------------------------------------------------------ layout

    def _build_readout(self, parent):
        card = self._card(parent, 'MARKER IN CAMERA FRAME')
        card.pack(fill='x', pady=(8, 0))
        body = tk.Frame(card, bg=T['surface'])
        body.pack(fill='x', padx=10, pady=(0, 10))
        self.tiles = {}
        for i, (key, unit) in enumerate((('dist', 'mm'), ('lateral', 'mm'),
                                         ('tilt', 'deg'))):
            col = tk.Frame(body, bg=T['surface'])
            col.grid(row=0, column=i, sticky='w', padx=(0, 26))
            head = tk.Frame(col, bg=T['surface'])
            head.pack(anchor='w')
            dot = tk.Canvas(head, width=10, height=10, bg=T['surface'],
                            highlightthickness=0)
            dot.pack(side='left', pady=(0, 2))
            did = dot.create_oval(2, 2, 9, 9, fill=T['muted'], outline='')
            lbl = tk.Label(head, text=key.upper(), font=self.f_caption,
                           fg=T['muted'], bg=T['surface'])
            lbl.pack(side='left', padx=(5, 0))
            vrow = tk.Frame(col, bg=T['surface'])
            vrow.pack(anchor='w')
            val = tk.Label(vrow, text='--', font=self.f_value, fg=T['ink'],
                           bg=T['surface'])
            val.pack(side='left')
            tk.Label(vrow, text=unit, font=self.f_caption, fg=T['muted'],
                     bg=T['surface']).pack(side='left', padx=(4, 0),
                                           pady=(0, 3))
            self.tiles[key] = {'val': val, 'dot': dot, 'id': did}
        self.sub = tk.Label(card, text='', font=self.f_small, fg=T['muted'],
                            bg=T['surface'], anchor='w', justify='left')
        self.sub.pack(fill='x', padx=10, pady=(0, 10))

    def _build_target(self, parent):
        card = self._card(parent, 'TARGET + SAFETY FLOOR')
        card.pack(side='left', fill='both', expand=True)
        g = tk.Frame(card, bg=T['surface'])
        g.pack(fill='x', pady=(0, 8))
        self.v_target = tk.StringVar(value=TARGET_MM_DEFAULT)
        self.v_tol = tk.StringVar(value=POS_TOL_MM_DEFAULT)
        self.v_floor = tk.StringVar(value='')
        self._row(g, 'standoff (mm)', self.v_target, TARGET_MM_CHOICES, 0, 0)
        self._row(g, 'pos tol (mm)', self.v_tol, POS_TOL_MM_CHOICES, 1, 0)
        tk.Label(g, text='TCP Z floor (mm)', font=self.f_caption,
                 fg=T['ink2'], bg=T['surface'], anchor='e').grid(
            row=2, column=0, padx=(10, 4), pady=4, sticky='e')
        tk.Entry(g, textvariable=self.v_floor, width=8, font=self.f_small,
                 bg=T['page'], fg=T['ink'], insertbackground=T['ink'],
                 relief='flat', highlightthickness=1,
                 highlightbackground=T['grid']).grid(row=2, column=1,
                                                     sticky='w', pady=4)
        self._button(card, 'Auto floor from here', T['grid'],
                     self.auto_floor).pack(fill='x', padx=10, pady=(0, 10))

    def _build_motion(self, parent):
        card = self._card(parent, 'MOTION LIMITS')
        card.pack(side='left', fill='both', expand=True, padx=(12, 0))
        g = tk.Frame(card, bg=T['surface'])
        g.pack(fill='x', pady=(0, 10))
        self.v_step = tk.StringVar(value=STEP_MM_DEFAULT)
        self.v_rot = tk.StringVar(value=ROT_DEG_DEFAULT)
        self.v_probe = tk.StringVar(value=PROBE_MM_DEFAULT)
        self.v_speed = tk.StringVar(value=SPEED_PCT_DEFAULT)
        self._row(g, 'translate (mm)', self.v_step, STEP_MM_CHOICES, 0, 0)
        self._row(g, 'level (deg)', self.v_rot, ROT_DEG_CHOICES, 1, 0)
        self._row(g, 'probe (mm)', self.v_probe, PROBE_MM_CHOICES, 2, 0)
        self._row(g, 'speed (%)', self.v_speed, SPEED_PCT_CHOICES, 3, 0)

    def _build_calib(self, parent):
        card = self._card(parent, '1. CALIBRATE ROTATION (3 PROBES)')
        card.pack(fill='x', pady=(12, 0))
        row = tk.Frame(card, bg=T['surface'])
        row.pack(fill='x', padx=10, pady=(0, 10))
        self.pbtn = []
        for i, ax in enumerate('XYZ'):
            b = self._button(row, f'Probe +{ax}', T['grid'],
                             lambda i=i: self.go(self.probe, i))
            b.pack(side='left', expand=True, fill='x', padx=(0, 6))
            self.pbtn.append(b)
        for txt, cmd in (('Reset', self.reset), ('Save calib', self.save_calib),
                         ('Load calib', self.load_calib)):
            self._button(row, txt, T['grid'], cmd).pack(
                side='left', expand=True, fill='x', padx=(0, 6))

    def _build_align(self, parent):
        card = self._card(parent, '2. ALIGN')
        card.pack(fill='x', pady=(12, 0))
        row = tk.Frame(card, bg=T['surface'])
        row.pack(fill='x', padx=10, pady=(0, 6))
        self.tb = self._button(row, '', T['grid'],
                               lambda: self.go(self.translate))
        self.tb.pack(side='left', expand=True, fill='x', padx=(0, 6))
        self.lb = self._button(row, '', T['grid'],
                               lambda: self.go(self.level))
        self.lb.pack(side='left', expand=True, fill='x')
        self.ab = self._button(card, '', T['good'],
                               lambda: self.go(self.auto_converge),
                               font=(self.f_button[0], 11, 'bold'), pady=11)
        self.ab.pack(fill='x', padx=10, pady=(0, 10))

    # ------------------------------------------------------------ helpers

    def say(self, m):
        self.log.insert('end', m + '\n')
        self.log.see('end')

    def trace(self, rec):
        """Append one JSON record to the current run's trace file."""
        if self.tracef is None:
            return
        rec['t'] = time.time()
        try:
            self.tracef.write(json.dumps(rec, default=float) + '\n')
            self.tracef.flush()
        except Exception as e:                              # noqa: BLE001
            self.say(f'trace write failed: {e}')

    def open_trace(self, header):
        LOG_DIR.mkdir(exist_ok=True)
        name = ('autoconverge_'
                + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.jsonl')
        self.tracepath = LOG_DIR / name
        self.tracef = self.tracepath.open('w')
        self.trace({'rec': 'run_start', **header})
        self.say(f'trace -> tools/fr3/logs/{name}')

    def close_trace(self, outcome):
        if self.tracef is None:
            return
        self.trace({'rec': 'run_end', 'outcome': outcome})
        self.tracef.close()
        self.tracef = None

    def set_status(self, m, color=None):
        self.status.config(text=m, fg=color or T['muted'])

    def step_m(self):
        return min(float(self.v_step.get()) / 1000.0, CEIL_STEP_M)

    def rot_deg(self):
        return min(float(self.v_rot.get()), CEIL_ROT_DEG)

    def probe_m(self):
        return min(float(self.v_probe.get()) / 1000.0, CEIL_PROBE_M)

    def target_m(self):
        return float(self.v_target.get()) / 1000.0

    def pos_tol_m(self):
        return float(self.v_tol.get()) / 1000.0

    def slowdown(self):
        """Trajectory time-stretch factor from the selected speed percent."""
        pct = min(float(self.v_speed.get()), CEIL_SPEED_PCT)
        return 100.0 / pct

    def z_floor(self):
        s = self.v_floor.get().strip()
        if not s:
            return None
        try:
            return float(s) / 1000.0
        except ValueError:
            return None

    def relabel(self):
        self.tb.config(text=f'Translate step ({self.step_m()*1000:g} mm)')
        self.lb.config(text=f'Level step ({self.rot_deg():g} deg)')
        self.ab.config(text=f'AUTO-CONVERGE  ->  {self.v_target.get()} mm')

    def reset(self):
        self.R, self.cols = None, {}
        self.say('Calibration cleared (saved file untouched).')

    def stop(self):
        """Graceful: end the loop, let the in-flight segment finish."""
        self.abort = True
        self.say('STOP (graceful) - cancelling goal; current move may finish.'
                 if self.n.stop()
                 else 'STOP (graceful) - loop will not continue.')
        self.set_status('stopping after current move', T['serious'])
        self.trace({'rec': 'stop_graceful'})

    def stop_now(self):
        """Instant: halt the trajectory where it is."""
        self.abort = True
        self.n.stop_now()
        self.say('*** STOP NOW - halting trajectory mid-motion ***')
        self.set_status('STOPPED (instant)', T['critical'])
        self.trace({'rec': 'stop_now'})

    # ------------------------------------------------------------ calib io

    def save_calib(self):
        if self.R is None:
            self.say('nothing to save - calibrate first')
            return
        try:
            _, R_base_tcp = self.n.tcp_pose()
        except Exception as e:                              # noqa: BLE001
            self.say(f'save failed (no TCP transform): {e}')
            return
        R_cam_tcp = self.R @ R_base_tcp
        q_opt = R2q(R_cam_tcp.T)     # TCP -> optical, the launch-arg direction
        CALIB_PATH.write_text(json.dumps({
            'R_cam_tcp': R_cam_tcp.tolist(),
            'quat_cam_tcp_xyzw': R2q(R_cam_tcp).tolist(),
            'handeye_quat_xyzw': q_opt.tolist(),
            'saved_utc': datetime.datetime.utcnow().isoformat(
                timespec='seconds'),
            'note': 'ROTATION ONLY, from align_gui probe calibration. '
                    'handeye_xyz (translation) is NOT determined by this.',
        }, indent=2) + '\n')
        self.say(f'saved -> {CALIB_PATH.name}')
        self.say('  launch arg:  handeye_quat:="'
                 f'{q_opt[0]:.6f} {q_opt[1]:.6f} {q_opt[2]:.6f} '
                 f'{q_opt[3]:.6f}"')
        self.say('  (rotation only - handeye_xyz still needs handeye_calib)')
        self.set_status(f'calibration saved to {CALIB_PATH.name}', T['good'])

    def _autoload(self, attempt=0):
        """Auto-load the saved calibration once TF is actually available.

        The TF buffer needs a moment to receive /tf and /tf_static, so a
        single early attempt races the listener and fails with
        "fr3_link0 ... does not exist". Retry instead of giving up.
        """
        if self.R is not None or not CALIB_PATH.exists():
            return
        try:
            self.n.tcp_pose()
        except Exception:                                   # noqa: BLE001
            if attempt < 8:
                self.root.after(1500, lambda: self._autoload(attempt + 1))
            else:
                self.say('auto-load gave up waiting for TF - '
                         'press "Load calib" once MoveIt is publishing')
            return
        self.load_calib(quiet=True)

    def load_calib(self, quiet=False):
        """Rebuild R_cam_base for the CURRENT arm pose from the saved file."""
        if not CALIB_PATH.exists():
            if not quiet:
                self.say(f'no saved calibration at {CALIB_PATH.name}')
            return
        try:
            data = json.loads(CALIB_PATH.read_text())
            R_cam_tcp = np.array(data['R_cam_tcp'], dtype=float)
            _, R_base_tcp = self.n.tcp_pose()
        except Exception as e:                              # noqa: BLE001
            self.say(f'load failed: {e}')
            return
        # R_cam_base = R_cam_tcp @ R_tcp_base, rebased to where the arm is
        self.R = R_cam_tcp @ R_base_tcp.T
        self.cols = {}      # R is set; probing again recalibrates from scratch
        self.say(f'loaded {CALIB_PATH.name} (saved '
                 f'{data.get("saved_utc", "?")}) and re-based to current pose')

    def auto_floor(self):
        """Predict the end pose from the CURRENT measurement, floor below it.

        The optical axis is near-vertical here, so the TCP descends about
        (measured - target). Needs no hand-eye translation, and it bounds a
        runaway from a bad marker scale or a sign error.
        """
        m = self.n.marker()
        if m is None:
            self.say('auto floor: marker not visible')
            return
        try:
            pos, _ = self.n.tcp_pose()
        except Exception as e:                              # noqa: BLE001
            self.say(f'auto floor failed: {e}')
            return
        descent = float(m[0][2]) - self.target_m()
        floor = pos[2] - max(descent, 0.0) - FLOOR_MARGIN_M
        self.v_floor.set(f'{floor * 1000:.1f}')
        self.say(f'auto floor: TCP z now {pos[2]*1000:.1f} mm, expected '
                 f'descent {descent*1000:.1f} mm -> floor '
                 f'{floor*1000:.1f} mm ({FLOOR_MARGIN_M*1000:g} mm margin)')

    # ------------------------------------------------------------ live view

    def tick(self):
        m = self.n.marker()
        if m is None:
            for k in self.tiles:
                self.tiles[k]['val'].config(text='--', fg=T['critical'])
                self.tiles[k]['dot'].itemconfig(self.tiles[k]['id'],
                                                fill=T['critical'])
            self.sub.config(text='marker NOT VISIBLE / STALE')
        else:
            p, Rm = m
            mz = Rm[:, 2]
            tilt = float(np.degrees(np.arccos(
                np.clip(abs(mz @ np.array([0, 0, 1.0])), -1, 1))))
            err = p - np.array([0, 0, self.target_m()])
            tol = self.pos_tol_m()
            lat = float(np.hypot(p[0], p[1]))
            for k, v, good in (
                    ('dist', p[2] * 1000, abs(err[2]) <= tol),
                    ('lateral', lat * 1000, lat <= tol),
                    ('tilt', tilt, tilt <= ROT_TOL_DEG)):
                col = T['good'] if good else (
                    T['warning'] if k != 'tilt' or tilt < 5 else T['serious'])
                self.tiles[k]['val'].config(text=f'{v:.2f}', fg=T['ink'])
                self.tiles[k]['dot'].itemconfig(self.tiles[k]['id'], fill=col)
            cal = ('calibrated' if self.R is not None
                   else f'NOT calibrated ({len(self.cols)}/3 probes)')
            fl = self.z_floor()
            self.sub.config(text=(
                f'err  x={err[0]*1000:+7.2f}  y={err[1]*1000:+7.2f}  '
                f'z={err[2]*1000:+7.2f} mm   |e|='
                f'{np.linalg.norm(err)*1000:.2f} mm\n'
                f'rotation {cal}   speed {self.v_speed.get()}%   '
                f'Z floor {"NOT SET" if fl is None else f"{fl*1000:.1f} mm"}'))
        self.root.after(150, self.tick)

    # ------------------------------------------------------------ actions

    def go(self, fn, *a):
        """Run a motion action in a worker thread, buttons disabled."""
        if self.busy:
            return
        self.busy = True
        btns = self.pbtn + [self.tb, self.lb, self.ab]
        for b in btns:
            b.config(state='disabled')
        self.set_status('moving...', T['warning'])

        def run():
            try:
                fn(*a)
            except Exception as e:                      # noqa: BLE001
                self.say(f'ERROR: {e}')
                self.set_status(f'error: {e}', T['critical'])
            finally:
                self.busy = False
                self.root.after(0, lambda: [b.config(state='normal')
                                            for b in btns])
        threading.Thread(target=run, daemon=True).start()

    def probe(self, i):
        before = self.n.marker()
        if before is None:
            self.say('probe aborted: marker not visible')
            return
        h = self.probe_m()
        d = np.zeros(3)
        d[i] = h
        self.say(f'probe {"XYZ"[i]}: moving {h*1000:g} mm ...')
        ok, msg = self.n.move(d, z_floor=self.z_floor(),
                              slowdown=self.slowdown())
        if not ok:
            self.say(f'probe {"XYZ"[i]} FAILED: {msg}')
            return
        time.sleep(0.6)
        after = self.n.marker()
        if after is None:
            self.say('probe aborted: marker lost after move')
            return
        shift = after[0] - before[0]
        # each column is Re_i regardless of h, so probe sizes may differ
        self.cols[i] = -shift / h
        self.say(f'probe {"XYZ"[i]}: marker shifted '
                 f'[{shift[0]*1000:+.2f} {shift[1]*1000:+.2f} '
                 f'{shift[2]*1000:+.2f}] mm')
        if len(self.cols) == 3 and all(self.cols[k] is not None
                                       for k in (0, 1, 2)):
            M = np.column_stack([self.cols[0], self.cols[1], self.cols[2]])
            u, s, vt = np.linalg.svd(M)       # nearest rotation
            self.R = u @ vt
            if np.linalg.det(self.R) < 0:
                u[:, -1] *= -1
                self.R = u @ vt
            self.say(f'CALIBRATED. singular values {np.round(s, 3)} '
                     '(want ~1,1,1)')
            self.say(f'R_cam_base =\n{np.round(self.R, 3)}')
            self.set_status('rotation calibrated - Save calib to keep it',
                            T['good'])

    def translate(self, quiet=False):
        if self.R is None:
            self.say('calibrate first (3 probes)')
            return False
        m = self.n.marker()
        if m is None:
            self.say('marker not visible')
            return False
        err = m[0] - np.array([0, 0, self.target_m()])
        d = self.R.T @ err
        nrm = np.linalg.norm(d)
        lim = self.step_m()
        if nrm < 1e-4:
            self.say('already within 0.1 mm - nothing to do')
            return True
        if nrm > lim:
            d *= lim / nrm
        if not quiet:
            self.say(f'translate: base delta [{d[0]*1000:+.2f} '
                     f'{d[1]*1000:+.2f} {d[2]*1000:+.2f}] mm '
                     f'(|e|={nrm*1000:.2f} mm)')
        ok, msg = self.n.move(d, z_floor=self.z_floor(),
                              slowdown=self.slowdown())
        self.say(f'  -> {msg}')
        self.trace({'rec': 'translate', 'ok': ok, 'msg': msg,
                    'err_cam_mm': (err * 1000).tolist(),
                    'err_norm_mm': float(nrm * 1000),
                    'cmd_d_base_mm': (d * 1000).tolist(),
                    'marker_pos_cam': m[0].tolist(),
                    **self.n.last_cmd})
        return ok

    def level(self, quiet=False):
        if self.R is None:
            self.say('calibrate first (3 probes)')
            return False
        m = self.n.marker()
        if m is None:
            self.say('marker not visible')
            return False
        mz = m[1][:, 2]
        tgt = np.array([0, 0, -1.0]) if mz[2] < 0 else np.array([0, 0, 1.0])
        ang = np.arccos(np.clip(mz @ tgt, -1, 1))
        if np.degrees(ang) < 0.15:
            self.say('already square within 0.15 deg')
            return True
        axis = np.cross(mz, tgt)
        if np.linalg.norm(axis) < 1e-8:
            self.say('degenerate rotation axis - skipping')
            return False
        tilt_before = float(np.degrees(np.arccos(
            np.clip(abs(mz @ np.array([0, 0, 1.0])), -1, 1))))
        full_ang = float(np.degrees(ang))
        ang = min(ang, np.radians(self.rot_deg()))
        Rc = axis_angle_R(axis, ang).T          # camera-frame correction
        D = self.R.T @ Rc @ self.R              # same rotation, base frame
        if not quiet:
            self.say(f'level: rotating {np.degrees(ang):.2f} deg')
        ok, msg = self.n.move(np.zeros(3), R_delta=D, z_floor=self.z_floor(),
                              slowdown=self.slowdown())
        self.say(f'  -> {msg}')
        rec = {'rec': 'level', 'ok': ok, 'msg': msg,
               'tilt_before_deg': tilt_before,
               'full_correction_deg': full_ang,
               'clamped_cmd_deg': float(np.degrees(ang)),
               'marker_normal_cam': mz.tolist(),
               'axis_cam': (axis / np.linalg.norm(axis)).tolist(),
               'R_delta_base_quat': R2q(D).tolist(),
               **self.n.last_cmd}
        time.sleep(0.6)                 # let the filter settle before reading
        m2 = self.n.marker()
        if m2 is not None:
            mz2 = m2[1][:, 2]
            rec['tilt_after_deg'] = float(np.degrees(np.arccos(
                np.clip(abs(mz2 @ np.array([0, 0, 1.0])), -1, 1))))
            rec['tilt_change_deg'] = rec['tilt_after_deg'] - tilt_before
        self.trace(rec)
        if ok:
            self.R = Rc.T @ self.R
        return ok

    def auto_converge(self):
        """Translate/level until the camera is at the target standoff and
        parallel. Aborts on STOP, marker loss, Z-floor block, plan failure,
        or lack of progress."""
        if self.R is None:
            self.say('calibrate first (3 probes)')
            return
        if self.z_floor() is None:
            self.say('REFUSING: set a TCP Z floor first '
                     '("Auto floor from here")')
            self.set_status('refused: no Z floor set', T['critical'])
            return
        self.abort = False
        tol = self.pos_tol_m()
        m0 = self.n.marker()
        if m0 is None:
            self.say('marker not visible')
            return
        # Iteration cap scaled to the distance/step actually asked for: a
        # 245 mm descent in 2 mm steps legitimately needs >100 iterations.
        err0 = float(np.linalg.norm(m0[0] - np.array([0, 0, self.target_m()])))
        cap = int(min(MAX_ITERS_ABS, max(60, 3 * err0 / self.step_m() + 30)))
        self.say(f'=== AUTO-CONVERGE to {self.target_m()*1000:g} mm, '
                 f'tol {tol*1000:g} mm / {ROT_TOL_DEG:g} deg, '
                 f'cap {cap} iters, speed {self.v_speed.get()}% ===')
        self.open_trace({
            'target_mm': self.target_m() * 1000,
            'pos_tol_mm': tol * 1000, 'rot_tol_deg': ROT_TOL_DEG,
            'step_mm': self.step_m() * 1000, 'rot_step_deg': self.rot_deg(),
            'speed_pct': float(self.v_speed.get()),
            'z_floor_mm': self.z_floor() * 1000,
            'R_cam_base': self.R.tolist(),
            'cap': cap,
        })
        outcome = 'exception'
        try:
            outcome = self._converge_loop(tol, cap)
        finally:
            self.close_trace(outcome)
            self.say(f'trace written: {self.tracepath}')

    def _converge_loop(self, tol, cap):
        """The loop body. Returns an outcome string; the caller closes the
        trace so every exit path (including exceptions) is recorded."""
        phase, prev, stale = None, np.inf, 0
        for it in range(1, cap + 1):
            if self.abort:
                self.say('ABORTED by STOP')
                return 'stopped'
            m = self.n.marker()
            if m is None:
                self.say('ABORT: marker lost / stale')
                self.set_status('aborted: marker lost', T['critical'])
                return 'marker_lost'
            err = float(np.linalg.norm(m[0] - np.array([0, 0,
                                                        self.target_m()])))
            mz = m[1][:, 2]
            tilt = float(np.degrees(np.arccos(
                np.clip(abs(mz @ np.array([0, 0, 1.0])), -1, 1))))
            self.say(f'[{it:02d}] |e|={err*1000:7.2f} mm  tilt={tilt:5.2f} deg')
            self.trace({'rec': 'iter', 'it': it, 'err_mm': err * 1000,
                        'tilt_deg': tilt,
                        'marker_pos_cam': m[0].tolist(),
                        'marker_normal_cam': mz.tolist(),
                        'marker_quat': R2q(m[1]).tolist(),
                        'joints': self.n.joints()})

            if err <= tol and tilt <= ROT_TOL_DEG:
                self.say(f'=== CONVERGED: {err*1000:.2f} mm, {tilt:.2f} deg '
                         f'in {it} iterations ===')
                self.set_status(f'converged: {err*1000:.2f} mm, '
                                f'{tilt:.2f} deg', T['good'])
                return 'converged'
            # Progress watchdog. Compared against the PREVIOUS iteration, not
            # the best ever, and reset when the phase switches: a level step
            # legitimately increases translation error (it rotates about the
            # TCP, and the camera swings on the unknown lever arm), so a
            # best-ever test would false-abort right after every level.
            cur = 't' if err > tol else 'r'
            if cur != phase:
                phase, prev, stale = cur, np.inf, 0
            metric = err if cur == 't' else tilt
            if metric < prev - (1e-5 if cur == 't' else 1e-3):
                stale = 0
            else:
                stale += 1
                if stale >= NO_PROGRESS_LIMIT:
                    self.say(f'ABORT: no progress in {stale} iterations of '
                             f'phase "{cur}" (reduce step size; or the '
                             'orientation estimate is too noisy to level)')
                    self.set_status('aborted: no progress', T['serious'])
                    return f'no_progress_{cur}'
            prev = metric
            ok = (self.translate(quiet=True) if cur == 't'
                  else self.level(quiet=True))
            if not ok:
                self.say('ABORT: step failed (see message above)')
                self.set_status('aborted: step failed', T['critical'])
                return 'step_failed'
            time.sleep(0.5)          # let the filter settle after motion
        self.say(f'ABORT: hit iteration cap ({cap})')
        self.set_status('aborted: iteration cap', T['serious'])
        return 'iter_cap'


def main():
    rclpy.init()
    node = AlignNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    gui = Gui(node)
    try:
        gui.root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
