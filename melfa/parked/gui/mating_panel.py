#!/usr/bin/env python3
"""Operator panel for the connector-mating cell.

Everything user-facing is data, not code: topics, tolerances, phase colours,
plot window, and every button come from panel_config.yaml (or --config).
Buttons support two actions:
  service_trigger  - call a std_srvs/Trigger service (e.g. stop/pause/reset)
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
import tkinter.font as tkfont

import numpy as np
import rclpy
import yaml
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'panel_config.yaml')

# Dark-surface chart chrome + status palette (validated set; see repo docs).
THEME = {
    'page': '#0d0d0d',        # window plane
    'surface': '#1a1a19',     # cards / chart surface
    'ink': '#ffffff',         # primary text
    'ink2': '#c3c2b7',        # secondary text
    'muted': '#898781',       # axis labels, captions
    'grid': '#2c2c2a',        # hairline gridlines / card border
    'baseline': '#383835',    # axis baseline
    'series': '#3987e5',      # plot line (single series per plot)
    'good': '#0ca30c',
    'warning': '#fab219',
    'serious': '#ec835a',
    'critical': '#d03b3b',
}


def _hex_to_rgb(h):
    h = h.lstrip('#')
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def text_on(color):
    """Black or white ink, whichever reads on the given fill."""
    r, g, b = _hex_to_rgb(color)
    return '#0b0b0b' if (0.299 * r + 0.587 * g + 0.114 * b) > 150 else '#ffffff'


def mix(color, other, t):
    """Linear blend color->other by t (for hover shades)."""
    a, b = _hex_to_rgb(color), _hex_to_rgb(other)
    return '#%02x%02x%02x' % tuple(int(round(a[i] + (b[i] - a[i]) * t))
                                   for i in range(3))


def rounded_rect(canvas, x0, y0, x1, y1, r, **kw):
    pts = [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1,
           x1 - r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0]
    return canvas.create_polygon(pts, smooth=True, **kw)


class PanelNode(Node):
    """ROS side: subscriptions + button actions. Thread-safe via self.lock."""

    def __init__(self, cfg):
        super().__init__('mating_panel')
        self.cfg = cfg
        self.lock = threading.Lock()
        self.phase = '---'
        self.paused = False
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
        # Operational pause is a separate latched Bool; the phase topic
        # always carries the real state-machine phase.
        self.create_subscription(Bool,
                                 cfg.get('paused_topic', '/mating/paused'),
                                 self._paused_cb, latched)
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

    def _paused_cb(self, msg):
        with self.lock:
            self.paused = bool(msg.data)
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
    button_log = None

    def __init__(self, root, node, cfg):
        self.root = root
        self.node = node
        self.cfg = cfg
        self.photo = None  # keep a reference or tk garbage-collects it
        t = THEME

        root.title(cfg.get('title', 'Connector Mating'))
        root.configure(bg=t['page'])

        base = tkfont.nametofont('TkDefaultFont').actual()['family']
        self.f_caption = (base, 9)
        self.f_label = (base, 10)
        self.f_value = ('DejaVu Sans Mono', 18, 'bold')
        self.f_phase = (base, 26, 'bold')
        self.f_button = (base, 11, 'bold')

        outer = tk.Frame(root, bg=t['page'])
        outer.pack(fill='both', expand=True, padx=16, pady=14)

        # ---- header: title + phase banner ------------------------------
        header = tk.Frame(outer, bg=t['page'])
        header.pack(fill='x')
        tk.Label(header, text=cfg.get('title', 'Connector Mating').upper(),
                 font=(base, 10, 'bold'), fg=t['muted'], bg=t['page'],
                 anchor='w').pack(fill='x')

        self.phase_canvas = tk.Canvas(outer, height=76, bg=t['page'],
                                      highlightthickness=0)
        self.phase_canvas.pack(fill='x', pady=(6, 12))

        # ---- middle: camera card (left) + two plot cards (right) -------
        mid = tk.Frame(outer, bg=t['page'])
        mid.pack(fill='both', expand=True)

        cam_card = self._card(mid, 'CAMERA')
        cam_card.pack(side='left', fill='both', expand=True)
        self.image_label = tk.Label(cam_card, bg=t['surface'],
                                    fg=t['muted'], font=self.f_label,
                                    text='waiting for image…')
        self.image_label.pack(fill='both', expand=True, padx=10, pady=(0, 10))

        right = tk.Frame(mid, bg=t['page'])
        right.pack(side='left', fill='y', padx=(12, 0))

        self.plots = {}
        for key, title, tol_key, unit in (
                ('pos', 'POSITION ERROR', 'pos_tol_mm', 'mm'),
                ('rot', 'ROTATION ERROR', 'rot_tol_deg', 'deg')):
            card = self._card(right, title)
            card.pack(fill='x', pady=(0, 12) if key == 'pos' else 0)
            row = tk.Frame(card, bg=t['surface'])
            row.pack(fill='x', padx=10)
            dot = tk.Canvas(row, width=10, height=10, bg=t['surface'],
                            highlightthickness=0)
            dot.pack(side='left', pady=6)
            dot_id = dot.create_oval(2, 2, 9, 9, fill=t['muted'], outline='')
            val = tk.Label(row, text='--', font=self.f_value, fg=t['ink'],
                           bg=t['surface'], anchor='w')
            val.pack(side='left', padx=(6, 2))
            tk.Label(row, text=unit, font=self.f_caption, fg=t['muted'],
                     bg=t['surface'], anchor='sw').pack(side='left', pady=(0, 4))
            canvas = tk.Canvas(card, width=330, height=110, bg=t['surface'],
                               highlightthickness=0)
            canvas.pack(padx=10, pady=(2, 10))
            self.plots[key] = {
                'canvas': canvas, 'value': val, 'dot': dot, 'dot_id': dot_id,
                'tol': float(cfg.get(tol_key, 1.0)), 'unit': unit,
            }

        # ---- buttons ----------------------------------------------------
        btn_frame = tk.Frame(outer, bg=t['page'])
        btn_frame.pack(fill='x', pady=(14, 4))
        per_row = int(cfg.get('buttons_per_row', 5))
        for i, btn in enumerate(cfg.get('buttons', [])):
            color = btn.get('color', t['grid'])
            fg = btn.get('text_color', text_on(color))
            b = tk.Button(btn_frame, text=btn['label'], font=self.f_button,
                          bg=color, fg=fg, activebackground=mix(color, '#ffffff', 0.15),
                          activeforeground=fg, relief='flat', bd=0,
                          highlightthickness=0, cursor='hand2',
                          padx=18, pady=10,
                          command=lambda b_=btn: self.run_button(b_))
            b.grid(row=i // per_row, column=i % per_row, padx=(0, 10), pady=4,
                   sticky='ew')
            btn_frame.grid_columnconfigure(i % per_row, weight=1)
            b.bind('<Enter>', lambda e, w=b, c=color:
                   w.config(bg=mix(c, '#ffffff', 0.12)))
            b.bind('<Leave>', lambda e, w=b, c=color: w.config(bg=c))

        # ---- status bar --------------------------------------------------
        self.status = tk.Label(outer, text='ready', anchor='w', fg=t['muted'],
                               bg=t['page'], font=self.f_caption)
        self.status.pack(fill='x', pady=(6, 0))

        self.refresh()

    def _card(self, parent, title):
        """Bordered surface card with a muted caption; returns the body."""
        t = THEME
        frame = tk.Frame(parent, bg=t['surface'],
                         highlightbackground=t['grid'], highlightthickness=1)
        tk.Label(frame, text=title, font=self.f_caption, fg=t['muted'],
                 bg=t['surface'], anchor='w').pack(fill='x', padx=10,
                                                   pady=(8, 2))
        return frame

    # ------------------------------------------------------------ actions

    def run_button(self, btn):
        def done(ok, message):
            if self.button_log is None:
                self.button_log = {}
            self.button_log[btn['label']] = ok
            color = THEME['good'] if ok else THEME['serious']
            self.root.after(0, lambda: self.status.config(
                text=f'{btn["label"]}: {"ok" if ok else "failed"} — {message}',
                fg=color))
        if btn.get('type') == 'service_trigger':
            self.node.call_trigger(btn['service'], done)
        elif btn.get('type') == 'param_toggle':
            self.node.toggle_param(btn['node'], btn['param'], done)
        else:
            done(False, f'unknown button type {btn.get("type")}')

    # ------------------------------------------------------------ drawing

    def draw_phase(self, phase, stale):
        t = THEME
        c = self.phase_canvas
        c.delete('all')
        w = max(c.winfo_width(), 200)
        color = self.cfg.get('phase_colors', {}).get(phase, t['grid'])
        if phase == '---':
            color = t['grid']
        rounded_rect(c, 0, 0, w, 74, 12, fill=color, outline='')
        ink = text_on(color)
        c.create_text(w / 2, 34, text=phase.replace('_', ' '),
                      font=self.f_phase, fill=ink)
        sub = 'no data' if phase == '---' else (
            'STALE — no updates from controller' if stale else 'live')
        c.create_text(w / 2, 60, text=sub, font=self.f_caption,
                      fill=mix(ink, color, 0.35))

    def draw_plot(self, key):
        t = THEME
        p = self.plots[key]
        c = p['canvas']
        c.delete('all')
        w, h = int(c['width']), int(c['height'])
        ml, mr, mt, mb = 34, 10, 8, 16  # margins: room for y labels + x caption
        pw, ph = w - ml - mr, h - mt - mb
        window = float(self.cfg.get('plot_seconds', 30))
        now = time.monotonic()
        with self.node.lock:
            data = [(ts, v) for ts, v in self.node.errors[key]
                    if now - ts < window]
        tol = p['tol']
        top = max([v for _, v in data] + [tol * 2.0]) * 1.12

        def x(ts):
            return ml + pw - (now - ts) / window * pw

        def y(v):
            return mt + ph - min(v / top, 1.0) * ph

        # recessive hairline grid: quarters of the scale
        for frac in (0.25, 0.5, 0.75):
            gy = mt + ph * frac
            c.create_line(ml, gy, ml + pw, gy, fill=t['grid'])
        # baseline + tolerance reference
        c.create_line(ml, mt + ph, ml + pw, mt + ph, fill=t['baseline'])
        ty = y(tol)
        c.create_line(ml, ty, ml + pw, ty, fill=t['muted'], dash=(2, 4))
        # y tick labels (muted, tabular): 0, tol, top
        for vy, txt in ((mt + ph, '0'), (ty, f'{tol:g}'), (mt + 4, f'{top:.3g}')):
            c.create_text(ml - 5, vy, text=txt, anchor='e', fill=t['muted'],
                          font=('DejaVu Sans Mono', 8))
        c.create_text(ml + pw, h - 3, text=f'last {window:g} s', anchor='se',
                      fill=t['muted'], font=('DejaVu Sans Mono', 8))

        if len(data) >= 2:
            pts = [coord for ts, v in data for coord in (x(ts), y(v))]
            c.create_line(*pts, fill=t['series'], width=2,
                          joinstyle='round', capstyle='round')
            # direct label on the latest point (selective, not every point)
            lx, ly = x(data[-1][0]), y(data[-1][1])
            c.create_oval(lx - 3, ly - 3, lx + 3, ly + 3,
                          fill=t['series'], outline=t['surface'], width=2)

    def refresh(self):
        t = THEME
        with self.node.lock:
            phase = self.node.phase
            paused = self.node.paused
            image = self.node.image_rgb
            stale = time.monotonic() - self.node.last_msg_time > \
                float(self.cfg.get('stale_after_s', 3.0))
            latest = {k: (d[-1][1] if d else None)
                      for k, d in self.node.errors.items()}

        # Compose the display state: the pause flag overlays the phase.
        # (phase == 'PAUSED' kept for controllers predating /mating/paused.)
        if paused and phase != '---':
            phase = 'PAUSED'
        self.draw_phase(phase, stale)

        for key, p in self.plots.items():
            v = latest[key]
            if v is None:
                p['value'].config(text='--', fg=t['ink2'])
                p['dot'].itemconfig(p['dot_id'], fill=t['muted'])
            else:
                p['value'].config(text=f'{v:.2f}', fg=t['ink'])
                p['dot'].itemconfig(
                    p['dot_id'],
                    fill=t['good'] if v < p['tol'] else t['serious'])
            self.draw_plot(key)

        if image is not None:
            hgt, wid = image.shape[:2]
            header = f'P6 {wid} {hgt} 255 '.encode()
            self.photo = tk.PhotoImage(data=header + image.tobytes())
            self.image_label.config(image=self.photo, text='')

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
