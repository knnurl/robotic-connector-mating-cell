#!/usr/bin/env python3
"""FR3 Cartesian-impedance commissioning panel.

Walks the fr3_mating_controllers README ladder one button per rung, with the
arm's own state in front of you. That controller has never run on hardware
and it commands TORQUE, so this exists to make first contact boring:

  0. PRE-FLIGHT  set the payload (camera + bracket) and the collision reflex
                 thresholds. Franka accepts these only with no controller
                 active, so the panel releases fr3_arm_controller (the robot
                 goes idle), sets both, checks each reply, and re-activates
                 it - judged by the controller manager's answer, not by how
                 the release call ended. Needed once per driver session; the
                 ladder stays locked until it succeeds. It reports |F ext| at
                 rest: that bias eats into the reflex margin.
  1. FLOAT       activate in float mode - the RT-loop proof. The arm
                 free-floats under gravity compensation. Push gently (keep
                 |F ext| under PUSH_LIMIT_N) and watch the RT pill
                 (control_command_success_rate). Anything ragged here is a
                 network/RT problem, not a gain problem. A slow sag means the
                 payload is wrong.
  2. HOLD        float off. The controller re-seeds its equilibrium where the
                 arm is NOW, so this cannot snap it back to the activation
                 pose. Pressing HOLD again while holding changes nothing (use
                 "hold HERE" to re-seed). Push the TCP gently: ~15 N per 10 cm sideways, ~12 N per
                 1.5 cm along tool Z. One small overshoot on release is normal:
                 D = 2*zeta*sqrt(K) is critical for 1 kg, about 0.5-0.7 for
                 the arm. Oscillation is not.
  3. SETPOINT    step the equilibrium a few cm, UP first; the arm should glide
                 there at the slew limit. No step may take the equilibrium
                 more than FLOOR_BELOW_HOLD_MM below where HOLD started - the
                 camera bracket hangs below the flange.
  4. RELEASE     hand the arm back to fr3_arm_controller.

Nothing moves without a button press. RELEASE is one click away whenever no
other panel action is running; actions take milliseconds and only stall when
the controller manager is not answering - and then RELEASE could not get
through either, so use the E-stop. Whether the impedance controller is live is
asked of the controller manager, never taken from the panel's memory: closing
the window, Ctrl+C and SIGTERM all take the same guarded close path, which
refuses to close while a release cannot be confirmed; a panel restarted
mid-session can still release.

Both controllers claim the EFFORT interface, so swapping them is a single
combined switch_controller call - franka_hardware then changes no command
mode and torque control is never interrupted. (Cross-mode swaps are the ones
that bite; see align_gui.switch_controllers.)

Any libfranka reflex kills ros2_control_node on this cell (franka_hardware
does not catch it). That STOPS the robot; it does not free it. The panel then
shows DRIVER DOWN: relaunch the stack and run PRE-FLIGHT again.

Run with ROS, ~/franka_ros2_ws and tools/fr3/fr3_env.sh sourced in EVERY
terminal - the controller manager must see this workspace's plugin:

    ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=$FR3_ROBOT_IP
    ros2 run controller_manager spawner cartesian_impedance_stroke_controller \\
        --inactive --param-file \\
        "$(pwd)/fr3_mating_controllers/config/cartesian_impedance_stroke.yaml"
    python3 tools/fr3/impedance_panel.py
"""

import datetime
import json
import pathlib
import signal
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext, ttk

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from franka_msgs.msg import FrankaRobotState
from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetLoad
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from state_relay import start_throttle, stop_throttle          # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'gui'))
from mating_panel import (THEME as T, mix, rounded_rect,       # noqa: E402
                          text_on)

IMPEDANCE_CONTROLLER = 'cartesian_impedance_stroke_controller'
ARM_CONTROLLER = 'fr3_arm_controller'
ROBOT_STATE_TOPIC = '/franka_robot_state_broadcaster/robot_state'
ROBOT_STATE_RELAY = '/impedance_panel/robot_state'
EQUILIBRIUM_TOPIC = f'/{IMPEDANCE_CONTROLLER}/equilibrium_pose'
STATE_STALE_S = 0.3
DRIVER_DOWN_S = 2.0       # robot state silent this long: the driver is gone

MODE_IDLE, MODE_MOVE, MODE_REFLEX = 1, 2, 4
ROBOT_MODES = {0: 'OTHER', 1: 'IDLE', 2: 'MOVE', 3: 'GUIDING', 4: 'REFLEX',
               5: 'USER_STOPPED', 6: 'ERROR_RECOVERY'}

# ---- pre-flight -------------------------------------------------------------
# Collision reflex thresholds. Contact thresholds only raise flags in the robot
# state; collision thresholds stop the robot - and on this cell a reflex also
# kills ros2_control_node. The upper Cartesian values sit about 10 N above the
# controller's force ceiling (max_force_n 30 N; max_torque_nm 10 Nm), while a
# shove above 40 N still reflexes. The reflex watches libfranka's ESTIMATED
# external wrench, so the margin shrinks by the estimate's bias - PRE-FLIGHT
# reports |F ext| at rest and warns above REST_FORCE_WARN_N. libfranka's own
# cartesian_impedance_control example sets 100 everywhere; these stay well
# below that.
CONTACT_TORQUE_NM = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
COLLISION_TORQUE_NM = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0]
CONTACT_WRENCH = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]      # N x3, Nm x3
COLLISION_WRENCH = [40.0, 40.0, 40.0, 40.0, 40.0, 40.0]
CONTROLLER_MAX_FORCE_N = 30.0   # max_force_n in the controller yaml
PUSH_LIMIT_N = 10.0             # what the ladder asks the operator to stay under
PREFLIGHT_MODE_TIMEOUT_S = 5.0
REST_FORCE_WARN_N = 5.0         # |F ext| bias at rest worth fixing before FLOAT
EXIT_WAIT_S = 20.0              # at exit, how long to let a running action finish
SPIN_JOIN_S = 2.0               # at exit, how long to wait for the spin to stop
# Payload inertia is modelled as a small block. It barely matters at
# commissioning speeds; mass and centre of mass do (gravity compensation).
PAYLOAD_GYRATION_M = 0.03
PAYLOAD_MAX_KG = 2.0

# ---- setpoints ---------------------------------------------------------------
# The equilibrium is a spring anchor. In free space the controller slews it at
# 5 cm/s and the arm keeps up, so MAX_LEAD_MM mostly caps how far one run of
# commands can carry the arm; if the arm is blocked it also caps the spring
# force on the soft axes (150 N/m x 60 mm = 9 N). Along stiff tool Z the
# controller's max_force_n (30 N) is the real force bound.
STEP_MM_CHOICES = ['5', '10', '20', '50']
STEP_MM_DEFAULT = '10'
AXIS_CHOICES = ['base Z (up)', 'base X', 'base Y', 'tool Z (stroke)']
MAX_LEAD_MM = 60.0
FLOOR_BELOW_HOLD_MM = 30.0

# ---- gains -------------------------------------------------------------------
# Live-tunable within the SAME limits the controller enforces (GainLimits in
# fr3_mating_controllers/include/fr3_mating_controllers/impedance_detail.hpp;
# test_impedance_panel checks they agree). The controller rejects anything
# outside them; the panel refuses first so the operator sees why. zeta 0 is an
# undamped spring and a negative value injects energy - the force ceiling
# bounds how hard the arm pushes, not whether it oscillates.
TUNE_FIELDS = [('k lateral', 'k_xy', '150'), ('k tool Z', 'k_z', '800'),
               ('k roll/pitch', 'k_rp', '10'), ('k yaw', 'k_yaw', '20'),
               ('damping zeta', 'zeta', '1.0')]
GAIN_LIMITS = {'k_xy': (0.0, 3000.0), 'k_z': (0.0, 3000.0),
               'k_rp': (0.0, 300.0), 'k_yaw': (0.0, 300.0),
               'zeta': (0.1, 2.0)}

LOG_DIR = pathlib.Path(__file__).with_name('logs')


def q2R(x, y, z, w):
    n = float(np.linalg.norm([x, y, z, w]))
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = np.array([x, y, z, w]) / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def release_if_active(node, say=print):
    """Hand the arm back if the CONTROLLER MANAGER says impedance is active.

    Used on window close and at process exit, so the panel's own memory never
    decides whether the arm is left compliant. Returns True when nothing is
    left active, False when a release was needed and failed, None when the
    controller manager cannot be asked.
    """
    states = node.controllers()
    if states is None:
        return None
    if states.get(IMPEDANCE_CONTROLLER) != 'active':
        return True
    ok, msg = node.switch([ARM_CONTROLLER], [IMPEDANCE_CONTROLLER])
    say(f'released {IMPEDANCE_CONTROLLER} on exit' if ok else
        f'*** could NOT release {IMPEDANCE_CONTROLLER} on exit ({msg}) - '
        'use RELEASE again or the robot E-stop ***')
    return ok


def exit_handoff(node, panel, say=print, wait_s=None):
    """The last word before the process exits, whatever ended the panel.

    Lets a running action finish (bounded), so its own restore can run; hands
    the arm back if the impedance controller is still active; and restores
    fr3_arm_controller if PRE-FLIGHT released it and it is still down. Ctrl+C
    and SIGTERM reach Panel.on_close first; an exception out of mainloop comes
    straight here. kill -9 cannot be caught at all.
    """
    wait_s = EXIT_WAIT_S if wait_s is None else wait_s
    if panel is not None:
        end = time.monotonic() + wait_s
        while panel.busy and time.monotonic() < end:
            time.sleep(0.05)
        if panel.busy:
            say('*** exiting while a panel action is still running - check '
                'the controllers: ros2 control list_controllers ***')
    result = release_if_active(node, say)
    if result is None:
        say('*** could not ask the controller manager on exit - if the '
            'impedance controller is active the arm is still compliant: '
            'RELEASE from a new panel, or use the E-stop ***')
    if panel is not None and getattr(panel, 'arm_released', False):
        states = node.controllers()
        if (states is not None and states.get(ARM_CONTROLLER) != 'active'
                and states.get(IMPEDANCE_CONTROLLER) != 'active'):
            ok, msg = node.switch([ARM_CONTROLLER], [])
            say(f'restored {ARM_CONTROLLER}, which PRE-FLIGHT had released'
                if ok else f'*** {ARM_CONTROLLER} is NOT active ({msg}) - '
                'activate it before moving the robot ***')
    return result


class ImpedanceNode(Node):
    """ROS side: robot state in, equilibrium out, controllers switched."""

    def __init__(self):
        super().__init__('impedance_panel')
        self._lock = threading.Lock()
        self._state = None       # (pos, quat, force, mode, rate, stamp)
        self.on_sample = None    # Panel hook, called in the spin thread
        self.relay_error = None
        try:
            self._relay = start_throttle(ROBOT_STATE_TOPIC, ROBOT_STATE_RELAY,
                                         node_name='impedance_panel_relay')
        except Exception as e:                              # noqa: BLE001
            self._relay, self.relay_error = None, str(e)
        self.create_subscription(
            FrankaRobotState, ROBOT_STATE_RELAY, self._state_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.eq_pub = self.create_publisher(PoseStamped, EQUILIBRIUM_TOPIC, 1)
        self.switch_cli = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.list_cli = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        # ATOMIC: plain set_parameters applies each parameter on its own, so a
        # rejected value could leave the rest of a gain set live while the
        # panel reported it refused.
        self.param_cli = self.create_client(
            SetParametersAtomically,
            f'/{IMPEDANCE_CONTROLLER}/set_parameters_atomically')
        self.load_cli = self.create_client(SetLoad, '/service_server/set_load')
        self.collision_cli = self.create_client(
            SetForceTorqueCollisionBehavior,
            '/service_server/set_force_torque_collision_behavior')

    def close(self):
        stop_throttle(self._relay)

    def _state_cb(self, m):
        p = m.o_t_ee.pose.position
        o = m.o_t_ee.pose.orientation
        f = m.o_f_ext_hat_k.wrench.force
        mode = int(m.robot_mode)
        rate = float(m.control_command_success_rate)
        with self._lock:
            self._state = (np.array([p.x, p.y, p.z]),
                           np.array([o.x, o.y, o.z, o.w]),
                           np.array([f.x, f.y, f.z]), mode, rate,
                           time.monotonic())
        hook = self.on_sample
        if hook is not None:
            hook({'pos': [p.x, p.y, p.z], 'quat': [o.x, o.y, o.z, o.w],
                  'force': [f.x, f.y, f.z],
                  'q': list(m.measured_joint_state.position),
                  'mode': mode, 'success_rate': rate})

    def state(self):
        """(pos, quat, force, mode, rate, age_s) or None before any message."""
        with self._lock:
            s = self._state
        return None if s is None else (*s[:5], time.monotonic() - s[5])

    def cm_reachable(self):
        return self.list_cli.service_is_ready()

    def publish_equilibrium(self, pos, quat):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'fr3_link0'
        (m.pose.position.x, m.pose.position.y, m.pose.position.z) = \
            [float(v) for v in pos]
        (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
         m.pose.orientation.w) = [float(v) for v in quat]
        self.eq_pub.publish(m)

    @staticmethod
    def _wait(fut, timeout_s):
        end = time.time() + timeout_s
        while time.time() < end:
            if fut.done():
                return True
            time.sleep(0.02)
        return False

    def switch(self, activate, deactivate, timeout_s=10.0):
        """One STRICT switch. Between the impedance and arm controllers both
        sides are effort, so franka_hardware never stops the robot; with an
        empty activate list it releases a controller and the robot idles."""
        if not self.switch_cli.wait_for_service(timeout_sec=3.0):
            return False, 'controller_manager not available'
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        fut = self.switch_cli.call_async(req)
        if not self._wait(fut, timeout_s):
            return False, 'switch_controller timed out'
        res = fut.result()
        ok = bool(res is not None and res.ok)
        return ok, ('switched' if ok else 'controller_manager refused')

    def controllers(self, timeout_s=3.0):
        """{name: state}, or None if the controller manager did not answer -
        which must never be read as 'nothing is active'."""
        if not self.list_cli.service_is_ready():
            return None
        fut = self.list_cli.call_async(ListControllers.Request())
        if not self._wait(fut, timeout_s) or fut.result() is None:
            return None
        return {c.name: c.state for c in fut.result().controller}

    def set_params(self, values, timeout_s=5.0):
        """values: {name: bool | float | [float, float, float]}, applied
        all-or-nothing by the controller."""
        if not self.param_cli.wait_for_service(timeout_sec=3.0):
            return False, f'{IMPEDANCE_CONTROLLER} parameters unavailable'
        req = SetParametersAtomically.Request()
        for name, v in values.items():
            p = Parameter()
            p.name = name
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = v
            elif isinstance(v, (list, tuple)):
                pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                pv.double_array_value = [float(x) for x in v]
            else:
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(v)
            p.value = pv
            req.parameters.append(p)
        fut = self.param_cli.call_async(req)
        if not self._wait(fut, timeout_s) or fut.result() is None:
            return False, 'set_parameters_atomically timed out'
        res = fut.result().result
        return bool(res.successful), ('applied' if res.successful
                                      else (res.reason or 'refused'))

    def _call_franka(self, cli, req, timeout_s=5.0):
        """franka's parameter services reply success=False (never crash) when
        the robot rejects a command, e.g. because a controller is active."""
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, f'{cli.srv_name} not available'
        fut = cli.call_async(req)
        if not self._wait(fut, timeout_s) or fut.result() is None:
            return False, f'{cli.srv_name} timed out'
        res = fut.result()
        return bool(res.success), (res.error or 'ok')

    def set_load(self, mass, com_m, inertia_diag):
        req = SetLoad.Request()
        req.mass = float(mass)
        req.center_of_mass = [float(v) for v in com_m]    # flange frame, m
        inertia = [0.0] * 9                               # column-major
        inertia[0], inertia[4], inertia[8] = [float(v) for v in inertia_diag]
        req.load_inertia = inertia
        return self._call_franka(self.load_cli, req)

    def set_collision_behavior(self):
        req = SetForceTorqueCollisionBehavior.Request()
        req.lower_torque_thresholds_nominal = list(CONTACT_TORQUE_NM)
        req.upper_torque_thresholds_nominal = list(COLLISION_TORQUE_NM)
        req.lower_force_thresholds_nominal = list(CONTACT_WRENCH)
        req.upper_force_thresholds_nominal = list(COLLISION_WRENCH)
        return self._call_franka(self.collision_cli, req)


class Panel:
    """The ladder, one rung per button."""

    def __init__(self, node):
        self.n = node
        self.active = False          # impedance controller active
        self.floating = False
        self.busy = False
        self.setpoint = None         # (pos, quat) last published
        self.z_floor = None          # lowest equilibrium z allowed, m
        self.preflight_ok = False    # payload + thresholds set this session
        self.arm_released = False    # PRE-FLIGHT released fr3_arm_controller
        self.driver_down_logged = False
        self.tracef = None
        self._trace_lock = threading.Lock()

        self.root = tk.Tk()
        self.root.title('FR3 Impedance Commissioning')
        self.root.configure(bg=T['page'])
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        base = tkfont.nametofont('TkDefaultFont').actual()['family']
        self.f_caption = (base, 9)
        self.f_value = ('DejaVu Sans Mono', 15, 'bold')
        self.f_small = ('DejaVu Sans Mono', 9)
        self.f_button = (base, 10, 'bold')
        self.f_banner = (base, 18, 'bold')
        self.f_pill = tkfont.Font(family=base, size=9)
        self.f_pill_b = tkfont.Font(family=base, size=9, weight='bold')
        self._init_combo_style()

        outer = tk.Frame(self.root, bg=T['page'])
        outer.pack(fill='both', expand=True, padx=16, pady=12)
        head = tk.Frame(outer, bg=T['page'])
        head.pack(fill='x')
        tk.Label(head, text='FR3  CARTESIAN IMPEDANCE  -  COMMISSIONING',
                 font=(base, 10, 'bold'), fg=T['muted'], bg=T['page'],
                 anchor='w').pack(side='left')
        self.pills = tk.Canvas(head, height=26, width=10, bg=T['page'],
                               highlightthickness=0)
        self.pills.pack(side='right')
        self.banner = tk.Canvas(outer, height=62, bg=T['page'],
                                highlightthickness=0)
        self.banner.pack(fill='x', pady=(6, 10))

        body = tk.Frame(outer, bg=T['page'])
        body.pack(fill='both', expand=True)
        left = tk.Frame(body, bg=T['page'])
        left.pack(side='left', fill='both', expand=True)
        right = tk.Frame(body, bg=T['page'])
        right.pack(side='left', fill='y', padx=(12, 0))
        self._build_readout(left)
        self._build_log(left)
        self._build_ladder(right)
        self._build_tuning(right)

        self.status = tk.Label(outer, text='ready', anchor='w', fg=T['muted'],
                               bg=T['page'], font=self.f_caption)
        self.status.pack(fill='x', pady=(6, 0))
        if node.relay_error:
            self.say(f'WARNING: state relay failed ({node.relay_error}) - '
                     'no robot state, so nothing will activate')
        self.say('Ladder: 0 PRE-FLIGHT -> 1 FLOAT (RT proof) -> 2 HOLD -> '
                 '3 SETPOINT -> 4 RELEASE. Read the arm, not the screen.')
        node.on_sample = self._sample
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
        b = tk.Button(parent, text=text, font=font or self.f_button, bg=color,
                      fg=fg, activebackground=mix(color, '#ffffff', .15),
                      activeforeground=fg, relief='flat', bd=0,
                      highlightthickness=0, cursor='hand2', padx=12,
                      pady=pady, command=cmd, disabledforeground=T['muted'])
        b.bind('<Enter>', lambda e, w=b, c=color:
               w['state'] == 'normal' and w.config(bg=mix(c, '#ffffff', .12)))
        b.bind('<Leave>', lambda e, w=b, c=color: w.config(bg=c))
        return b

    def install_signal_handlers(self):
        """Ctrl+C and SIGTERM take the guarded close path. The default Ctrl+C
        raises inside a Tk callback, where tkinter swallows it: nothing is
        released and the live view silently freezes."""
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self._on_signal)

    def _on_signal(self, _signum, _frame):
        self.root.after(0, self.on_close)

    def _entry(self, parent, var, width):
        return tk.Entry(parent, textvariable=var, width=width,
                        font=self.f_small, bg=T['page'], fg=T['ink'],
                        insertbackground=T['ink'], relief='flat',
                        highlightthickness=1, highlightbackground=T['grid'])

    def _build_readout(self, parent):
        card = self._card(parent, 'ARM')
        card.pack(fill='x')
        row = tk.Frame(card, bg=T['surface'])
        row.pack(fill='x', padx=10, pady=(0, 6))
        self.tiles = {}
        for key, unit in (('TCP Z', 'mm'), ('|F| ext', 'N'), ('Fz ext', 'N'),
                          ('spring lead', 'mm')):
            col = tk.Frame(row, bg=T['surface'])
            col.pack(side='left', padx=(0, 18))
            tk.Label(col, text=key.upper(), font=self.f_caption,
                     fg=T['muted'], bg=T['surface'], anchor='w').pack(
                fill='x')
            v = tk.Label(col, text='--', font=self.f_value, fg=T['ink'],
                         bg=T['surface'], anchor='w')
            v.pack(fill='x')
            tk.Label(col, text=unit, font=self.f_caption, fg=T['muted'],
                     bg=T['surface'], anchor='w').pack(fill='x')
            self.tiles[key] = v
        self.sub = tk.Label(card, text='', font=self.f_small, fg=T['muted'],
                            bg=T['surface'], anchor='w', justify='left')
        self.sub.pack(fill='x', padx=10, pady=(0, 10))

    def _build_log(self, parent):
        card = self._card(parent, 'LOG')
        card.pack(fill='both', expand=True, pady=(12, 0))
        self.log = scrolledtext.ScrolledText(
            card, height=16, width=64, font=self.f_small, bg=T['page'],
            fg=T['ink2'], insertbackground=T['ink'], relief='flat', bd=0,
            highlightthickness=0)
        self.log.pack(fill='both', expand=True, padx=10, pady=(0, 10))

    def _build_ladder(self, parent):
        card = self._card(parent, 'LADDER  |  fr3_mating_controllers README')
        card.pack(fill='x')

        pay = tk.Frame(card, bg=T['surface'])
        pay.pack(fill='x', padx=10, pady=(0, 4))
        tk.Label(pay, text='payload = camera + bracket, NOT the hand '
                           '(0 if Desk already has them)',
                 font=self.f_caption, fg=T['muted'], bg=T['surface'],
                 anchor='w').grid(row=0, column=0, columnspan=8, sticky='w')
        self.v_mass = tk.StringVar(value='')
        self.v_com = [tk.StringVar(value='') for _ in range(3)]
        for c, (label, var, w) in enumerate((
                ('kg   CoM from flange:', self.v_mass, 6),
                ('x', self.v_com[0], 4), ('y', self.v_com[1], 4),
                ('z mm', self.v_com[2], 4))):
            self._entry(pay, var, w).grid(row=1, column=c * 2, sticky='w',
                                          pady=2)
            tk.Label(pay, text=label, font=self.f_caption, fg=T['ink2'],
                     bg=T['surface']).grid(row=1, column=c * 2 + 1,
                                           sticky='w', padx=(2, 6))
        self.b_pre = self._button(card, '0.  PRE-FLIGHT  (payload + reflex '
                                        'thresholds)', T['warning'],
                                  lambda: self.go(self.preflight))
        self.b_pre.pack(fill='x', padx=10, pady=(2, 6))
        self.b_float = self._button(card, '1.  FLOAT  (RT proof)',
                                    T['series'], lambda: self.go(self.float_on))
        self.b_float.pack(fill='x', padx=10, pady=(0, 6))
        self.b_hold = self._button(card, '2.  HOLD  (float off, holds here)',
                                   T['good'], lambda: self.go(self.hold_on))
        self.b_hold.pack(fill='x', padx=10, pady=(0, 6))

        step = tk.Frame(card, bg=T['surface'])
        step.pack(fill='x', padx=10, pady=(0, 4))
        self.v_step = tk.StringVar(value=STEP_MM_DEFAULT)
        self.v_axis = tk.StringVar(value=AXIS_CHOICES[0])
        ttk.Combobox(step, textvariable=self.v_step, values=STEP_MM_CHOICES,
                     width=4, state='readonly', style='Dark.TCombobox',
                     font=self.f_small).pack(side='left')
        tk.Label(step, text='mm along', font=self.f_caption, fg=T['ink2'],
                 bg=T['surface']).pack(side='left', padx=4)
        ttk.Combobox(step, textvariable=self.v_axis, values=AXIS_CHOICES,
                     width=15, state='readonly', style='Dark.TCombobox',
                     font=self.f_small).pack(side='left')
        row = tk.Frame(card, bg=T['surface'])
        row.pack(fill='x', padx=10, pady=(0, 6))
        self.b_minus = self._button(row, '3.  SETPOINT  -', T['grid'],
                                    lambda: self.go(self.setpoint_step, -1.0))
        self.b_minus.pack(side='left', expand=True, fill='x', padx=(0, 6))
        self.b_plus = self._button(row, 'SETPOINT  +', T['grid'],
                                   lambda: self.go(self.setpoint_step, 1.0))
        self.b_plus.pack(side='left', expand=True, fill='x')
        self.b_here = self._button(card, 'hold HERE (equilibrium = arm)',
                                   T['grid'], lambda: self.go(self.hold_here),
                                   pady=6)
        self.b_here.pack(fill='x', padx=10, pady=(0, 8))
        self.b_release = self._button(card, '4.  RELEASE  ->  arm controller',
                                      T['critical'],
                                      lambda: self.go(self.release),
                                      font=(self.f_button[0], 13, 'bold'),
                                      pady=13)
        self.b_release.pack(fill='x', padx=10, pady=(0, 10))

    def _build_tuning(self, parent):
        card = self._card(parent, 'GAINS  |  live, within the controller '
                                  'limits')
        card.pack(fill='x', pady=(12, 0))
        g = tk.Frame(card, bg=T['surface'])
        g.pack(fill='x', padx=10, pady=(0, 6))
        self.tune = {}
        for r, (label, key, default) in enumerate(TUNE_FIELDS):
            lo, hi = GAIN_LIMITS[key]
            tk.Label(g, text=label, font=self.f_caption, fg=T['ink2'],
                     bg=T['surface'], anchor='e', width=12).grid(
                row=r, column=0, sticky='e', pady=2)
            var = tk.StringVar(value=default)
            self._entry(g, var, 8).grid(row=r, column=1, sticky='w', padx=6,
                                        pady=2)
            tk.Label(g, text=f'{lo:g} - {hi:g}', font=self.f_caption,
                     fg=T['muted'], bg=T['surface']).grid(row=r, column=2,
                                                          sticky='w')
            self.tune[key] = var
        self._button(card, 'apply gains', T['grid'],
                     lambda: self.go(self.apply_gains), pady=6).pack(
            fill='x', padx=10, pady=(0, 10))

    # ------------------------------------------------------------ helpers

    def say(self, m):
        stamp = datetime.datetime.now().strftime('%H:%M:%S')
        self.log.insert('end', f'{stamp}  {m}\n')
        self.log.see('end')

    def set_status(self, m, color=None):
        self.status.config(text=m, fg=color or T['muted'])

    def trace(self, rec):
        """Append one record. Called from worker, Tk and spin threads."""
        with self._trace_lock:
            if self.tracef is None:
                return
            rec['t'] = time.time()
            try:
                self.tracef.write(json.dumps(rec, default=float) + '\n')
                self.tracef.flush()
            except Exception as e:                          # noqa: BLE001
                print(f'impedance_panel: trace write failed: {e}',
                      file=sys.stderr)

    def open_trace(self):
        with self._trace_lock:
            if self.tracef is not None:
                return
            LOG_DIR.mkdir(exist_ok=True)
            name = ('impedance_' + datetime.datetime.now().strftime(
                '%Y%m%d_%H%M%S') + '.jsonl')
            self.tracef = (LOG_DIR / name).open('w')
        self.trace({'rec': 'session_start', 'gains': self.gains()})
        self.say(f'trace -> tools/fr3/logs/{name}')

    def close_trace(self):
        self.trace({'rec': 'session_end'})
        with self._trace_lock:
            if self.tracef is not None:
                self.tracef.close()
                self.tracef = None

    def _sample(self, s):
        """Spin thread, every relayed state message (50 Hz): enough to see
        the hold-test overshoot and short RT dips, with joints for drift."""
        if self.tracef is None or not self.active:
            return
        anchor = self.setpoint
        self.trace({'rec': 'sample', **s,
                    'anchor': None if anchor is None else list(anchor[0]),
                    'floating': self.floating})

    def gains(self):
        """The gain entries as numbers, or None if one is not a number."""
        try:
            return {k: float(v.get()) for k, v in self.tune.items()}
        except ValueError:
            return None

    @staticmethod
    def gain_error(g):
        """Why a gain set is unacceptable, or None."""
        for key, (lo, hi) in GAIN_LIMITS.items():
            v = g[key]
            if not np.isfinite(v) or not lo <= v <= hi:
                return f'{key} must be within [{lo:g}, {hi:g}]'
        return None

    def payload(self):
        """(mass_kg, com_m) from the entries, or the reason they are unusable."""
        raw = [self.v_mass.get().strip()] + [v.get().strip()
                                             for v in self.v_com]
        if any(r == '' for r in raw):
            return ('enter the payload first: measured mass of camera + '
                    'bracket and its centre of mass from the flange '
                    '(0 kg if Desk already includes them)')
        try:
            vals = [float(r) for r in raw]
        except ValueError:
            return 'payload fields must be numbers'
        mass, com = vals[0], np.array(vals[1:]) / 1000.0
        if not np.isfinite(mass) or not 0.0 <= mass <= PAYLOAD_MAX_KG:
            return (f'payload mass must be within 0-{PAYLOAD_MAX_KG:g} kg '
                    '(camera + bracket is roughly 0.1-0.3 kg)')
        if not np.all(np.isfinite(com)) or np.any(np.abs(com) > 0.3):
            return 'centre of mass must be within 300 mm of the flange'
        return mass, com

    def _state_age(self):
        st = self.n.state()
        return float('inf') if st is None else st[5]

    def blocked(self):
        """Why the robot may not be driven right now, or None."""
        st = self.n.state()
        if st is None or st[5] > STATE_STALE_S:
            return 'no robot state (is franka_ros2 running?)'
        if st[3] == MODE_REFLEX:
            return 'robot in REFLEX - run error recovery'
        if st[3] != MODE_MOVE:
            return f'robot is {ROBOT_MODES.get(st[3], st[3])}, not MOVE'
        return None

    def _wait_mode(self, modes, timeout_s):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            st = self.n.state()
            if st is not None and st[5] <= STATE_STALE_S and st[3] in modes:
                return True
            time.sleep(0.05)
        return False

    def go(self, fn, *a):
        """Run one action in a worker thread; buttons disabled meanwhile."""
        if self.busy:
            return
        self.busy = True
        btns = [self.b_pre, self.b_float, self.b_hold, self.b_minus,
                self.b_plus, self.b_here, self.b_release]
        for b in btns:
            b.config(state='disabled')

        def run():
            try:
                fn(*a)
            except Exception as e:                          # noqa: BLE001
                self.say(f'ERROR: {e}')
                self.set_status(f'error: {e}', T['critical'])
            finally:
                self.busy = False
                self.root.after(0, lambda: [b.config(state='normal')
                                            for b in btns])
        threading.Thread(target=run, daemon=True).start()

    def _clear_run_state(self):
        self.active, self.floating = False, False
        self.setpoint, self.z_floor = None, None

    def _driver_down(self):
        self.say('DRIVER DOWN - the controller manager is unreachable and '
                 'robot state has stopped. The robot is not being commanded: '
                 'a reflex or crash STOPS it, it does not free it. Relaunch '
                 'the stack, then run PRE-FLIGHT again.')
        self.set_status('DRIVER DOWN - relaunch the stack', T['critical'])
        self._clear_run_state()
        self.preflight_ok = False
        self.trace({'rec': 'driver_down'})

    # ------------------------------------------------------------ ladder

    def preflight(self):
        """Payload + collision thresholds, with the robot idle as franka
        requires, and the arm controller restored whatever happens."""
        self.preflight_ok = False
        pl = self.payload()
        if isinstance(pl, str):
            self.say(f'REFUSING: {pl}')
            self.set_status('refused: payload missing', T['serious'])
            return
        mass, com = pl
        states = self.n.controllers()
        if states is None:
            self.say('REFUSING: the controller manager is not answering - is '
                     'the stack up?')
            return
        if states.get(IMPEDANCE_CONTROLLER) == 'active':
            self.say('REFUSING: RELEASE the impedance controller before '
                     'PRE-FLIGHT')
            return
        if states.get(ARM_CONTROLLER) != 'active':
            self.say(f'REFUSING: {ARM_CONTROLLER} is not active '
                     f'({states.get(ARM_CONTROLLER)}) - the panel will only '
                     'release what it can restore')
            return
        why = self.blocked()
        if why is not None:
            self.say(f'REFUSING: {why}')
            return
        self.open_trace()
        self.say(f'PRE-FLIGHT: releasing {ARM_CONTROLLER} so the robot goes '
                 'idle - franka accepts these settings only then.')
        load_ok = collision_ok = restored = False
        # Flag BEFORE the call: a release that times out on our side can still
        # complete in the controller manager afterwards.
        self.arm_released = True
        try:
            ok, msg = self.n.switch([], [ARM_CONTROLLER])
            if not ok:
                self.say(f'PRE-FLIGHT aborted: could not release '
                         f'{ARM_CONTROLLER} ({msg})')
                return
            if not self._wait_mode({MODE_IDLE}, PREFLIGHT_MODE_TIMEOUT_S):
                self.say('PRE-FLIGHT aborted: the robot never reported IDLE')
                return
            inertia = [mass * PAYLOAD_GYRATION_M ** 2] * 3
            load_ok, load_msg = self.n.set_load(mass, com, inertia)
            self.say(f'  payload {mass:.3f} kg at [{com[0]*1000:.0f} '
                     f'{com[1]*1000:.0f} {com[2]*1000:.0f}] mm from the '
                     'flange: ' + ('set' if load_ok else
                                   f'FAILED ({load_msg})'))
            collision_ok, collision_msg = self.n.set_collision_behavior()
            self.say(f'  collision reflex at {COLLISION_WRENCH[0]:.0f} N / '
                     f'{COLLISION_WRENCH[3]:.0f} Nm Cartesian, contact flag at '
                     f'{CONTACT_WRENCH[0]:.0f} N: ' +
                     ('set' if collision_ok else f'FAILED ({collision_msg})'))
        finally:
            restored = self._restore_arm_controller()
            self.preflight_ok = bool(load_ok and collision_ok and restored)
            self.trace({'rec': 'preflight', 'ok': self.preflight_ok,
                        'mass_kg': mass, 'com_m': [float(v) for v in com],
                        'load_ok': load_ok, 'collision_ok': collision_ok,
                        'restored': restored,
                        'contact_torque_nm': CONTACT_TORQUE_NM,
                        'collision_torque_nm': COLLISION_TORQUE_NM,
                        'contact_wrench': CONTACT_WRENCH,
                        'collision_wrench': COLLISION_WRENCH})
        if self.preflight_ok:
            st = self.n.state()
            rest = (float('nan') if st is None
                    else float(np.linalg.norm(st[2])))
            self.say(f'PRE-FLIGHT done - the ladder is unlocked. |F ext| at '
                     f'rest {rest:.1f} N' +
                     (' - that bias eats into the reflex margin: check the '
                      'payload before FLOAT' if not rest <= REST_FORCE_WARN_N
                      else '') +
                     f'. Keep hand pushes under {PUSH_LIMIT_N:.0f} N.')
            self.set_status('pre-flight done', T['good'])
            self.trace({'rec': 'preflight_rest_force', 'force_n': rest})
        else:
            self.say('PRE-FLIGHT failed - the ladder stays locked')

    def _restore_arm_controller(self):
        """Bring fr3_arm_controller back, judged by the controller manager's
        answer rather than by how the release call ended. True once it is
        active and the robot is back in MOVE."""
        states = self.n.controllers()
        if states is not None and states.get(ARM_CONTROLLER) == 'active':
            back, back_msg = True, 'already active'
        else:
            back, back_msg = self.n.switch([ARM_CONTROLLER], [])
        restored = back and self._wait_mode({MODE_MOVE}, PREFLIGHT_MODE_TIMEOUT_S)
        if restored:
            self.arm_released = False
            self.say(f'  {ARM_CONTROLLER} active again')
        else:
            self.say(f'*** {ARM_CONTROLLER} did NOT come back '
                     f'({back_msg if not back else "robot not in MOVE"}) '
                     '- relaunch the stack before anything else ***')
            self.set_status('arm controller NOT restored', T['critical'])
        return restored

    def _activate(self, float_mode):
        """Put the impedance controller in charge, in the wanted mode."""
        if not self.preflight_ok:
            self.say('REFUSING: run 0. PRE-FLIGHT first - payload and '
                     'collision thresholds must be set for this driver '
                     'session')
            self.set_status('refused: pre-flight not done', T['serious'])
            return False
        why = self.blocked()
        if why is not None:
            self.say(f'REFUSING: {why}')
            self.set_status(f'refused: {why}', T['serious'])
            return False
        states = self.n.controllers()
        if states is None:
            self.say('REFUSING: the controller manager is not answering')
            return False
        if states.get(IMPEDANCE_CONTROLLER) is None:
            self.say(f'REFUSING: {IMPEDANCE_CONTROLLER} is not loaded - '
                     'spawn it inactive first (see the module docstring)')
            self.set_status('refused: controller not loaded', T['critical'])
            return False
        # float_mode BEFORE activation: on_activate reads it, and activating
        # in hold mode when you meant float is the wrong surprise.
        ok, msg = self.n.set_params({'float_mode': bool(float_mode)})
        if not ok:
            self.say(f'could not set float_mode: {msg}')
            return False
        if states.get(IMPEDANCE_CONTROLLER) == 'active':
            # already live (e.g. a restarted panel): adopt it, so RELEASE works
            self.active = True
            self.floating = bool(float_mode)
            return True
        ok, msg = self.n.switch([IMPEDANCE_CONTROLLER], [ARM_CONTROLLER])
        if not ok:
            self.say(f'could not activate {IMPEDANCE_CONTROLLER}: {msg}')
            self.set_status('activation failed', T['critical'])
            return False
        self.active = True
        self.floating = bool(float_mode)
        self.setpoint = None
        self.say(f'{IMPEDANCE_CONTROLLER} ACTIVE - the arm is compliant now')
        self.trace({'rec': 'activate', 'float_mode': self.floating})
        return True

    def float_on(self):
        if not self._activate(True):
            return
        self.say('FLOAT: move the arm gently by hand - smooth, no buzz, no '
                 f'kicks. Keep |F| ext under {PUSH_LIMIT_N:.0f} N. RT pill '
                 'below ~99% is a network/RT problem, not a gain problem; a '
                 'slow sag means the payload is wrong.')
        self.set_status('floating - RT proof', T['series'])
        self.trace({'rec': 'float_on'})

    def hold_on(self):
        if self.active and not self.floating:
            # Already holding. The controller re-seeds only on float -> hold, so
            # nothing would change in the arm - and resetting the floor or the
            # anchor here would let repeated presses walk the floor down and
            # blank the spring-lead readout while the arm is still gliding.
            self.say('already holding - nothing to change. To re-seed the '
                     'equilibrium where the arm is, use "hold HERE".')
            return
        if not self.active:
            if not self._activate(False):
                return
        else:
            ok, msg = self.n.set_params({'float_mode': False})
            if not ok:
                self.say(f'float_mode not cleared: {msg}')
                return
            self.floating = False
        self.setpoint = None
        st = self.n.state()
        if st is not None:
            here = float(st[0][2]) - FLOOR_BELOW_HOLD_MM / 1000.0
            # One floor per activation (RELEASE clears it), never lowered: a
            # FLOAT -> HOLD cycle may raise it, but cannot walk it down.
            self.z_floor = here if self.z_floor is None else max(self.z_floor,
                                                                 here)
        floor = ('--' if self.z_floor is None else
                 f'{self.z_floor*1000:.0f} mm')
        self.say('HOLD: the controller re-seeded its equilibrium where the '
                 'arm is now - it must not move. Push the TCP gently: soft '
                 'sideways, stiffer along tool Z. One small overshoot on '
                 'release is normal; ringing, buzz or drift is not. Keep '
                 f'|F| ext under {PUSH_LIMIT_N:.0f} N. Setpoint floor: {floor}.')
        self.set_status('holding', T['good'])
        self.trace({'rec': 'hold_on', 'gains': self.gains(),
                    'z_floor': self.z_floor})

    def setpoint_step(self, sign):
        if not self.active or self.floating:
            self.say('SETPOINT needs the controller holding (step 2 first)')
            return
        if self.z_floor is None:
            self.say('REFUSING: no Z floor - press 2. HOLD first')
            return
        st = self.n.state()
        why = self.blocked()
        if why is not None or st is None:
            self.say(f'REFUSING: {why}')
            return
        pos, quat = st[0], st[1]
        try:
            step = min(float(self.v_step.get()), 50.0) / 1000.0 * sign
        except ValueError:
            self.say('step size is not a number')
            return
        axis = self.v_axis.get()
        if axis.startswith('tool Z'):
            d = q2R(*quat)[:, 2] * step
        else:
            idx = {'base X': 0, 'base Y': 1}.get(axis, 2)
            d = np.zeros(3)
            d[idx] = step
        anchor = (self.setpoint[0] if self.setpoint is not None else pos) + d
        # Only a step that LOWERS the anchor below the floor is refused, so an
        # arm floated down past it can still be stepped back up.
        if anchor[2] < self.z_floor and d[2] < 0.0:
            self.say(f'REFUSING: that puts the equilibrium at z '
                     f'{anchor[2]*1000:.0f} mm, below the floor '
                     f'{self.z_floor*1000:.0f} mm ({FLOOR_BELOW_HOLD_MM:.0f} mm '
                     'under where HOLD started) - the camera bracket hangs '
                     'below the flange')
            self.set_status('refused: below Z floor', T['serious'])
            return
        lead = float(np.linalg.norm(anchor - pos))
        if lead * 1000.0 > MAX_LEAD_MM:
            self.say(f'REFUSING: that would put the equilibrium '
                     f'{lead*1000:.0f} mm from the arm (cap {MAX_LEAD_MM:.0f} '
                     'mm) - wait for the arm to catch up')
            self.set_status('refused: equilibrium too far', T['serious'])
            return
        self.setpoint = (anchor, quat)
        self.n.publish_equilibrium(anchor, quat)
        self.say(f'setpoint {step*1000:+.0f} mm along {axis} - lead now '
                 f'{lead*1000:.1f} mm; the arm glides at the slew limit')
        self.trace({'rec': 'setpoint', 'axis': axis, 'step_mm': step * 1000,
                    'anchor': anchor.tolist(), 'lead_mm': lead * 1000})

    def hold_here(self):
        if not self.active or self.floating:
            self.say('nothing to re-seed: the controller is not holding')
            return
        st = self.n.state()
        if st is None:
            self.say('no robot state')
            return
        self.setpoint = (st[0], st[1])
        self.n.publish_equilibrium(st[0], st[1])
        self.say('equilibrium re-seeded at the current pose (zero lead)')
        self.trace({'rec': 'hold_here', 'anchor': st[0].tolist()})

    def release(self):
        """Hand the arm back, deciding from the controller manager's state."""
        if not self.n.cm_reachable():
            if self._state_age() > DRIVER_DOWN_S:
                self._driver_down()
                self.close_trace()
            else:
                self.say('the controller manager is not answering - press '
                         'RELEASE again in a moment')
            return
        states = self.n.controllers()
        if states is None:
            self.say('the controller manager did not answer - press RELEASE '
                     'again')
            return
        if states.get(IMPEDANCE_CONTROLLER) != 'active':
            self.say('impedance controller is not active'
                     + (' - panel state corrected' if self.active else ''))
            self._clear_run_state()
            return
        ok, msg = self.n.switch([ARM_CONTROLLER], [IMPEDANCE_CONTROLLER])
        if not ok:
            self.say(f'*** RELEASE FAILED ({msg}) - the arm is still on the '
                     'impedance controller; use the robot E-stop if it is '
                     'not behaving ***')
            self.set_status('RELEASE FAILED', T['critical'])
            self.trace({'rec': 'release', 'ok': False, 'msg': msg})
            return
        self._clear_run_state()
        self.say(f'released - {ARM_CONTROLLER} holds the arm again')
        self.set_status('released', T['muted'])
        self.trace({'rec': 'release', 'ok': True})
        self.close_trace()

    def apply_gains(self):
        g = self.gains()
        if g is None:
            self.say('REFUSING: gains must be numbers')
            return
        err = self.gain_error(g)
        if err is not None:
            self.say(f'REFUSING: {err} - zeta 0 is an undamped spring, and '
                     'the force ceiling does not stop oscillation')
            self.set_status('refused: gain out of range', T['serious'])
            return
        ok, msg = self.n.set_params({
            'k_pos_tool': [g['k_xy'], g['k_xy'], g['k_z']],
            'k_rot_tool': [g['k_rp'], g['k_rp'], g['k_yaw']],
            'damping_ratio': g['zeta']})
        self.say(f'gains {"applied" if ok else "REFUSED"}: k_pos '
                 f'[{g["k_xy"]:.0f} {g["k_xy"]:.0f} {g["k_z"]:.0f}] N/m, '
                 f'k_rot [{g["k_rp"]:.0f} {g["k_rp"]:.0f} {g["k_yaw"]:.0f}] '
                 f'Nm/rad, zeta {g["zeta"]:.2f}'
                 + ('' if ok else f' ({msg})'))
        self.trace({'rec': 'gains', 'ok': ok, **g})

    # ------------------------------------------------------------ live view

    def tick(self):
        # Reschedule FIRST. tkinter swallows exceptions raised in callbacks, so
        # a tick that re-armed itself only at the end would stop the live view
        # for good on one error, leaving a stale banner that looks live.
        self.root.after(150, self.tick)
        try:
            self._refresh()
        except Exception as e:                              # noqa: BLE001
            print(f'impedance_panel: live view update failed: {e}',
                  file=sys.stderr)

    def _refresh(self):
        st = self.n.state()
        if st is None or st[5] > DRIVER_DOWN_S:
            if not self.driver_down_logged:
                if not self.n.cm_reachable():
                    if self.preflight_ok or self.active:
                        self.say('robot state stopped and the controller '
                                 'manager does not answer - the driver is '
                                 'down. Relaunch the stack, then run '
                                 'PRE-FLIGHT again.')
                    self.preflight_ok = False
                    self.driver_down_logged = True
                elif self.active:
                    self.say('robot state stopped but the controller manager '
                             'still answers - the state relay may be stuck. '
                             'Impedance may be live: press RELEASE. Do not '
                             'relaunch while the controller manager is alive.')
                    self.driver_down_logged = True
        else:
            self.driver_down_logged = False
        if st is None or st[5] > STATE_STALE_S:
            for v in self.tiles.values():
                v.config(text='--', fg=T['critical'])
            self.sub.config(text='no robot state')
        else:
            pos, quat, force, mode, rate, _ = st
            fn = float(np.linalg.norm(force))
            lead = (0.0 if self.setpoint is None
                    else float(np.linalg.norm(self.setpoint[0] - pos)) * 1000)
            for key, val, col in (
                    ('TCP Z', f'{pos[2]*1000:.1f}', T['ink']),
                    ('|F| ext', f'{fn:.1f}',
                     T['warning'] if fn > PUSH_LIMIT_N else T['ink']),
                    ('Fz ext', f'{force[2]:+.1f}', T['ink']),
                    ('spring lead', f'{lead:.1f}',
                     T['warning'] if lead > 20 else T['ink'])):
                self.tiles[key].config(text=val, fg=col)
            floor = ('--' if self.z_floor is None else
                     f'{self.z_floor*1000:.0f} mm')
            self.sub.config(text=(
                f'TCP  x {pos[0]*1000:8.1f}   y {pos[1]*1000:8.1f}   '
                f'z {pos[2]*1000:8.1f} mm   Z floor {floor}\n'
                f'robot {ROBOT_MODES.get(mode, mode)}   '
                f'control success {rate*100:.1f}%   '
                f'external force [{force[0]:+.1f} {force[1]:+.1f} '
                f'{force[2]:+.1f}] N'))
        self.draw_pills()
        self.draw_banner()

    def _pill_states(self):
        st = self.n.state()
        if st is None or st[5] > STATE_STALE_S:
            robot, rt = ('no state', T['critical']), ('--', T['muted'])
        else:
            robot = (ROBOT_MODES.get(st[3], str(st[3])),
                     T['good'] if st[3] == MODE_MOVE
                     else T['critical'] if st[3] == MODE_REFLEX
                     else T['warning'])
            rate = st[4] * 100.0
            rt = (f'{rate:.0f}%', T['good'] if rate >= 99.0
                  else T['warning'] if rate >= 95.0 else T['critical'])
        pre = ('done', T['good']) if self.preflight_ok else \
            ('needed', T['warning'])
        if self.active:
            ctrl = ('FLOAT', T['series']) if self.floating else \
                ('impedance', T['good'])
        else:
            ctrl = ('arm controller', T['muted'])
        return [('FRANKA',) + robot, ('RT',) + rt, ('PRE-FLIGHT',) + pre,
                ('CONTROL',) + ctrl]

    def draw_pills(self):
        c = self.pills
        c.delete('all')
        items = []
        for label, value, col in self._pill_states():
            wl, wv = self.f_pill.measure(label), self.f_pill_b.measure(value)
            items.append((label, value, col, wl, 26 + wl + 6 + wv + 12))
        total = sum(i[-1] for i in items) + 8 * (len(items) - 1) + 2
        if int(c['width']) != total:
            c.config(width=total)
        x = 1
        for label, value, col, wl, w in items:
            rounded_rect(c, x, 2, x + w, 24, 11, fill=T['surface'],
                         outline=mix(col, T['surface'], 0.55))
            c.create_oval(x + 11, 9, x + 19, 17, fill=col, outline='')
            c.create_text(x + 26, 13, text=label, anchor='w',
                          font=self.f_pill, fill=T['muted'])
            c.create_text(x + 26 + wl + 6, 13, text=value, anchor='w',
                          font=self.f_pill_b,
                          fill=T['ink2'] if col == T['muted'] else col)
            x += w + 8

    def _banner_state(self):
        if self.active and self._state_age() > DRIVER_DOWN_S:
            if not self.n.cm_reachable():
                return ('DRIVER DOWN', T['critical'],
                        'robot state stopped and the controller manager is '
                        'gone - the robot is not being commanded; relaunch '
                        'the stack, then PRE-FLIGHT')
            return ('NO ROBOT STATE', T['critical'],
                    'state relay silent but the controller manager answers - '
                    'impedance may still be live: press RELEASE')
        why = self.blocked()
        if not self.active:
            if why:
                return ('NOT READY', T['critical'], why)
            if not self.preflight_ok:
                return ('PRE-FLIGHT NEEDED', T['warning'],
                        'enter the payload, then press 0. PRE-FLIGHT - it sets '
                        'payload and collision thresholds')
            return ('INACTIVE', T['grid'],
                    'arm on fr3_arm_controller  -  press 1. FLOAT to start '
                    'the ladder')
        if why:
            return ('CHECK THE ROBOT', T['critical'],
                    f'{why}  -  press RELEASE')
        if self.floating:
            return ('FLOATING', T['series'],
                    'arm is free under gravity compensation  -  move it by '
                    'hand, then press 2. HOLD')
        lead = 0.0
        st = self.n.state()
        if self.setpoint is not None and st is not None:
            lead = float(np.linalg.norm(self.setpoint[0] - st[0])) * 1000
        if lead > 1.0:
            return ('MOVING', T['warning'],
                    f'equilibrium {lead:.1f} mm from the arm  -  gliding at '
                    'the slew limit')
        return ('HOLDING', T['good'],
                'compliant hold  -  push the TCP gently to feel the spring')

    def draw_banner(self):
        c = self.banner
        c.delete('all')
        w = max(c.winfo_width(), 400)
        state, color, sub = self._banner_state()
        rounded_rect(c, 0, 0, w, 60, 12, fill=color, outline='')
        ink = text_on(color)
        c.create_text(20, 23, text=state, anchor='w', font=self.f_banner,
                      fill=ink)
        c.create_text(20, 47, text=sub, anchor='w', font=self.f_caption,
                      fill=mix(ink, color, 0.3))

    def on_close(self):
        """Never leave the arm on the impedance controller."""
        if self.busy:
            self.say('an action is still running - close again once it has '
                     'finished, so the arm is never left mid-switch')
            return
        self.busy = True
        result = release_if_active(self.n, self.say)
        if result is False or (result is None
                               and self._state_age() <= DRIVER_DOWN_S):
            # False: a release was needed and failed. None while robot state
            # is fresh: the driver is alive but the controller manager did not
            # answer, so impedance may still be live. Only a dead driver (no
            # answer AND no state) lets the window close unconfirmed.
            self.say('*** cannot confirm the impedance controller is released '
                     '- NOT closing. Press RELEASE, or use the E-stop ***')
            self.busy = False
            return
        self.busy = False
        self.close_trace()
        self.root.destroy()


def shutdown_ros(node, panel, executor, spin, say=print):
    """Teardown, in an order that is the whole point.

    The spin MUST stop before the rclpy context goes away. A thread still
    inside spin() when rclpy.shutdown() runs takes a DDS thread down with the
    context - "terminate called without an active exception", i.e. the process
    ABORTS, in the very path whose job is handing the arm back. Observed
    2026-09-16 on a normal window close: the abort landed after the handoff by
    luck of the race, not by construction. So: hand back, stop the spin, join
    it, and only then drop the node and the context.
    """
    try:
        # Window closed (on_close has already released), or an exception out
        # of mainloop: ask the controller manager, not the panel, what is live.
        exit_handoff(node, panel, say)
    finally:
        if panel is not None:
            panel.close_trace()
        node.close()
        executor.shutdown()
        spin.join(timeout=SPIN_JOIN_S)
        node.destroy_node()
        rclpy.shutdown()


def main():
    # rclpy's own signal handling is off: Ctrl+C must reach Panel.on_close,
    # which needs ROS alive to hand the arm back.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ImpedanceNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    panel = None
    try:
        panel = Panel(node)
        panel.install_signal_handlers()
        panel.root.mainloop()
    finally:
        shutdown_ros(node, panel, executor, spin)


if __name__ == '__main__':
    main()
