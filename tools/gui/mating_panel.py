#!/usr/bin/env python3
"""Operator panel for the connector-mating cell.

Everything user-facing is data, not code: topics, tolerances, phase colours,
plot window, and every button come from panel_config.yaml (or --config).
Buttons support two actions:
  service_trigger  - call a std_srvs/Trigger service (e.g. the mating reset)
  param_toggle     - flip a boolean parameter on a node (e.g. enable_insertion)

Run (any machine on the ROS 2 network, sourced environment):
    python3 tools/gui/mating_panel.py [--config path/to/config.yaml]

Uses only stdlib tkinter + rclpy — no Qt, no extra installs.
The panel is a convenience, NOT a safety device; the e-stop stays hardware.
"""

import argparse
import collections
import os
import threading
import time
import tkinter as tk

import numpy as np
import rclpy
import yaml
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'panel_config.yaml')


class PanelNode(Node):
    """ROS side: subscriptions + button actions. Thread-safe via self.lock."""

    def __init__(self, cfg):
        super().__init__('mating_panel')
        self.cfg = cfg
        self.lock = threading.Lock()
        self.phase = '---'
        self.errors = {
            'pos': collections.deque(maxlen=2000),   # (t, mm)
            'rot': collections.deque(maxlen=2000),   # (t, deg)
        }
        self.image_rgb = None          # numpy RGB, downscaled
        self.last_msg_time = 0.0

        # Pre-create service/parameter clients for all configured buttons
        # (rclpy already uses Node._clients internally; use our own dict).
        self._srv_clients = {}
        for btn in cfg.get('buttons', []):
            if btn.get('type') == 'service_trigger':
                self._client(Trigger, btn['service'])
            elif btn.get('type') == 'param_toggle':
                self._client(GetParameters, f"{btn['node']}/get_parameters")
                self._client(SetParameters, f"{btn['node']}/set_parameters")

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, cfg['phase_topic'],
                                 self._phase_cb, latched)
        self.create_subscription(Float64, cfg['error_pos_topic'],
                                 lambda m: self._err_cb('pos', m), 10)
        self.create_subscription(Float64, cfg['error_rot_topic'],
                                 lambda m: self._err_cb('rot', m), 10)
        if cfg.get('image_topic'):
            self.create_subscription(Image, cfg['image_topic'],
                                     self._image_cb, 2)

    def _phase_cb(self, msg):
        with self.lock:
            self.phase = msg.data
            self.last_msg_time = time.monotonic()

    def _err_cb(self, key, msg):
        with self.lock:
            self.errors[key].append((time.monotonic(), msg.data))
            self.last_msg_time = time.monotonic()

    def _image_cb(self, msg):
        enc = msg.encoding.lower()
        if enc not in ('bgr8', 'rgb8'):
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape(msg.height, msg.step)[:, :msg.width * 3]
        arr = arr.reshape(msg.height, msg.width, 3)
        if enc == 'bgr8':
            arr = arr[:, :, ::-1]
        max_w = int(self.cfg.get('image_max_width', 480))
        stride = max(1, int(np.ceil(msg.width / max_w)))
        with self.lock:
            self.image_rgb = np.ascontiguousarray(arr[::stride, ::stride])

    # ------------------------------------------------------------ actions

    def _client(self, srv_type, name):
        if name not in self._srv_clients:
            self._srv_clients[name] = self.create_client(srv_type, name)
        return self._srv_clients[name]

    def call_trigger(self, service, done_cb):
        cli = self._client(Trigger, service)
        if not cli.service_is_ready():
            done_cb(False, f'{service}: not available')
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: done_cb(
            f.result() is not None and f.result().success,
            f.result().message if f.result() else 'call failed'))

    def toggle_param(self, node_name, param, done_cb):
        get_cli = self._client(GetParameters, f'{node_name}/get_parameters')
        if not get_cli.service_is_ready():
            done_cb(False, f'{node_name}: parameter service not available')
            return
        req = GetParameters.Request(names=[param])

        def after_get(f):
            res = f.result()
            if res is None or not res.values:
                done_cb(False, f'could not read {param}')
                return
            new_value = not bool(res.values[0].bool_value)
            set_cli = self._client(SetParameters, f'{node_name}/set_parameters')
            p = Parameter(name=param, value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=new_value))
            fut = set_cli.call_async(SetParameters.Request(parameters=[p]))
            fut.add_done_callback(lambda g: done_cb(
                g.result() is not None and g.result().results[0].successful,
                f'{param} = {new_value}'))

        get_cli.call_async(req).add_done_callback(after_get)


class PanelUI:
    def __init__(self, root, node, cfg):
        self.root = root
        self.node = node
        self.cfg = cfg
        self.photo = None  # keep a reference or tk garbage-collects it

        root.title(cfg.get('title', 'Connector Mating'))
        root.configure(bg='#202020')

        self.phase_label = tk.Label(root, text='---', font=('DejaVu Sans', 32, 'bold'),
                                    fg='white', bg='gray25', width=18, pady=10)
        self.phase_label.grid(row=0, column=0, columnspan=2, sticky='ew',
                              padx=8, pady=8)

        self.err_labels = {}
        self.plots = {}
        for col, (key, title, tol_key, unit) in enumerate([
                ('pos', 'position error', 'pos_tol_mm', 'mm'),
                ('rot', 'rotation error', 'rot_tol_deg', 'deg')]):
            frame = tk.Frame(root, bg='#202020')
            frame.grid(row=1, column=col, padx=8, sticky='n')
            lbl = tk.Label(frame, text=f'{title}: ---', font=('DejaVu Sans Mono', 14),
                           fg='white', bg='#202020')
            lbl.pack()
            canvas = tk.Canvas(frame, width=300, height=110, bg='#101010',
                               highlightthickness=0)
            canvas.pack(pady=4)
            self.err_labels[key] = lbl
            self.plots[key] = (canvas, float(cfg.get(tol_key, 1.0)), unit)

        self.image_label = tk.Label(root, bg='#101010')
        self.image_label.grid(row=2, column=0, columnspan=2, padx=8, pady=8)

        btn_frame = tk.Frame(root, bg='#202020')
        btn_frame.grid(row=3, column=0, columnspan=2, pady=4)
        per_row = int(cfg.get('buttons_per_row', 4))
        for i, btn in enumerate(cfg.get('buttons', [])):
            color = btn.get('color', 'gray85')
            tk.Button(btn_frame, text=btn['label'],
                      font=('DejaVu Sans', 12, 'bold'),
                      bg=color, activebackground=color,
                      fg=btn.get('text_color', 'black'),
                      activeforeground=btn.get('text_color', 'black'),
                      command=lambda b=btn: self.run_button(b),
                      width=18, height=2).grid(
                row=i // per_row, column=i % per_row, padx=6, pady=4)

        self.status = tk.Label(root, text='', anchor='w', fg='gray70',
                               bg='#202020', font=('DejaVu Sans', 10))
        self.status.grid(row=4, column=0, columnspan=2, sticky='ew', padx=8)

        self.refresh()

    button_log = None

    def run_button(self, btn):
        def done(ok, message):
            if self.button_log is None:
                self.button_log = {}
            self.button_log[btn['label']] = ok
            self.root.after(0, lambda: self.status.config(
                text=f'{btn["label"]}: {"OK" if ok else "FAILED"} - {message}',
                fg='pale green' if ok else 'salmon'))
        if btn.get('type') == 'service_trigger':
            self.node.call_trigger(btn['service'], done)
        elif btn.get('type') == 'param_toggle':
            self.node.toggle_param(btn['node'], btn['param'], done)
        else:
            done(False, f'unknown button type {btn.get("type")}')

    def draw_plot(self, key):
        canvas, tol, unit = self.plots[key]
        canvas.delete('all')
        w = int(canvas['width'])
        h = int(canvas['height'])
        window = float(self.cfg.get('plot_seconds', 30))
        now = time.monotonic()
        with self.node.lock:
            data = [(t, v) for t, v in self.node.errors[key] if now - t < window]
        top = max([v for _, v in data] + [tol * 2.0]) * 1.1
        # tolerance line
        y_tol = h - (tol / top) * h
        canvas.create_line(0, y_tol, w, y_tol, fill='#3fa34d', dash=(4, 3))
        canvas.create_text(4, y_tol - 8, anchor='w', fill='#3fa34d',
                           text=f'tol {tol:g} {unit}', font=('DejaVu Sans', 8))
        if len(data) >= 2:
            pts = []
            for t, v in data:
                x = w - (now - t) / window * w
                y = h - min(v / top, 1.0) * h
                pts += [x, y]
            canvas.create_line(*pts, fill='#e0e0e0', width=2)

    def refresh(self):
        with self.node.lock:
            phase = self.node.phase
            image = self.node.image_rgb
            stale = time.monotonic() - self.node.last_msg_time > \
                float(self.cfg.get('stale_after_s', 3.0))
            latest = {k: (d[-1][1] if d else None)
                      for k, d in self.node.errors.items()}

        colors = self.cfg.get('phase_colors', {})
        self.phase_label.config(
            text=phase + (' (stale)' if stale and phase != '---' else ''),
            bg=colors.get(phase, 'gray25'))

        for key, (_, tol, unit) in self.plots.items():
            v = latest[key]
            lbl = self.err_labels[key]
            if v is None:
                lbl.config(text=f'{key}: ---', fg='gray60')
            else:
                lbl.config(text=f'{v:7.2f} {unit}',
                           fg='pale green' if v < tol else 'orange')
            self.draw_plot(key)

        if image is not None:
            hgt, wid = image.shape[:2]
            header = f'P6 {wid} {hgt} 255 '.encode()
            self.photo = tk.PhotoImage(data=header + image.tobytes())
            self.image_label.config(image=self.photo)

        self.root.after(int(1000 / float(self.cfg.get('refresh_hz', 10))),
                        self.refresh)


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def spin_quietly(node):
    try:
        rclpy.spin(node)
    except Exception:
        if rclpy.ok():
            raise  # real error; shutdown races are expected and silent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=DEFAULT_CONFIG_PATH,
                        help='panel configuration YAML')
    parser.add_argument('--test-seconds', type=float, default=0.0,
                        help='auto-close after N seconds (smoke testing)')
    args, ros_args = parser.parse_known_args()
    cfg = load_config(args.config)

    rclpy.init(args=ros_args)
    node = PanelNode(cfg)
    spin_thread = threading.Thread(target=spin_quietly, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    ui = PanelUI(root, node, cfg)
    if args.test_seconds > 0:
        # exercise every configured button once, staggered
        for i, btn in enumerate(cfg.get('buttons', [])):
            root.after(2000 + i * 800, lambda b=btn: ui.run_button(b))
        root.after(int(args.test_seconds * 1000), root.destroy)
    try:
        root.mainloop()
    finally:
        with node.lock:
            snapshot = (node.phase, {k: len(d) for k, d in node.errors.items()},
                        node.image_rgb is not None)
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        if args.test_seconds > 0:
            print(f'SMOKE OK phase={snapshot[0]} samples={snapshot[1]} '
                  f'image={snapshot[2]} button_results={ui.button_log}')


if __name__ == '__main__':
    main()
