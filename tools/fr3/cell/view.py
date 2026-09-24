"""FR3 Cell Control - the window.

One page, no process tabs: status bar (the same seven chips always), one
banner (the single highest-priority fault, logic.banner), a narrow left
column (camera, telemetry, force, plots), the POSITION and TORQUE sections
with the speed slider, a settings drawer, and a stop bar that never moves.

The window draws what a Backend hands it and nothing else:
  snap() -> logic.Snap        hist() / fhist()     image() -> (seq, RGB) | None
  now()   calib() -> (meta, state)   applied_gains()   applied_slew()
  pose_info() -> {name: (taught, (mm, deg) | None)}
  command(name, *args)   set_param(name, value)   set_settings(st)
  set_pending_gains(d)   heartbeat()   camera(on)   fault(name)   busy()
LiveBackend (cell.py) wires these to ROS; visual.py's scenes are fixed.
Worker threads reach the window only through post(), a queued Qt signal.
"""

import dataclasses
import datetime
import math
import time

from PySide6.QtCore import QEvent, QObject, QPointF, QRectF, QSize, Qt, QTimer, Signal
from PySide6.QtGui import (QColor, QFont, QFontMetrics, QImage, QPainter, QPainterPath,
                           QPen, QPixmap, QPolygonF)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
                               QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
                               QMainWindow, QPlainTextEdit, QPushButton, QScrollArea,
                               QSizePolicy, QSlider, QVBoxLayout, QWidget)

import choices as C
import logic
import persist
from palette import LEVEL, MONO_FONT, QSS, M, S, T

PLOT_WINDOW_S = 30.0
AUTO_CLOSE_S = 3.0           # an auto-opened camera view closes once vision is healthy this long
CONFIRM_S = 6.0              # an armed confirm button disarms after this
STOP_CONTROLS = ('stop_now', 'pause', 'stop_after')
TORQUE_ENTRY = ('float', 'hold')
TEXT = {
    'translate': 'Translate', 'level': 'Level', 'inplane': 'In-plane',
    'auto_converge': 'AUTO-CONVERGE', 'preflight': 'PRE-FLIGHT',
    'float': 'FLOAT', 'hold': 'HOLD', 'setpoint_minus': 'SETPOINT  −',
    'setpoint_plus': 'SETPOINT  +', 'hold_here': 'hold HERE', 'track': 'TRACK',
    'track_fast': 'FAST',
    'release': 'RELEASE  →  arm controller', 'apply_gains': 'APPLY GAINS',
    'recover': 'RECOVER', 'stop_now': 'STOP NOW', 'pause': 'PAUSE',
    'stop_after': 'stop after\ncurrent move',
}
STOP_TIP = ('Software stop (Esc, from anywhere in the window): halts MoveIt '
            'mid-move, ends tracking, pins the impedance equilibrium where the '
            'arm is. It is NOT an emergency stop and carries no safety rating - '
            'the hardware E-stop and the enabling device are the safety functions.')


# ---------------------------------------------------------------- helpers

def restyle(w, **props):
    changed = False
    for k, v in props.items():
        if w.property(k) != v:
            w.setProperty(k, v)
            changed = True
    if changed:
        w.style().unpolish(w)
        w.style().polish(w)


def lab(text='', role=None, wrap=False):
    w = QLabel(text)
    if role:
        w.setProperty('role', role)
    if role == 'section':
        f = w.font()
        f.setLetterSpacing(QFont.AbsoluteSpacing, 1.2)
        w.setFont(f)
    w.setWordWrap(wrap)
    return w


def panel(name='panel'):
    f = QFrame()
    f.setObjectName(name)
    return f


def fmt(v, spec, dash='—'):
    return dash if v is None or (isinstance(v, float) and math.isnan(v)) else format(v, spec)


def set_text(w, text):
    if w.text() != text:
        w.setText(text)


def draw_shape(p, kind, cx, cy, r, color):
    p.setPen(Qt.NoPen)
    p.setBrush(QColor(color))
    if kind == 'circle':
        p.drawEllipse(QPointF(cx, cy), r, r)
    elif kind == 'square':
        p.drawRect(QRectF(cx - r, cy - r, 2 * r, 2 * r))
    else:
        p.drawPolygon(QPolygonF([QPointF(cx, cy - r * 1.15), QPointF(cx + r * 1.2, cy + r),
                                 QPointF(cx - r * 1.2, cy + r)]))


TEXT_COLOR = {'normal': T['ink'], 'active': T['green'], 'warn': T['amber'],
              'fault': '#f0675e'}


# ---------------------------------------------------------------- widgets

class Chip(QWidget):
    """One status-bar indicator: shape + label + value, never colour alone."""

    def __init__(self, label):
        super().__init__()
        self.label, self.value, self.level = label, '--', 'normal'
        self.f_label, self.f_value = QFont(self.font()), QFont(self.font())
        self.f_label.setPixelSize(S)
        self.f_value.setPixelSize(M)
        self.f_value.setBold(True)
        self.setFixedHeight(30)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    def set(self, value, level):
        if (value, level) != (self.value, self.level):
            self.value, self.level = value, level
            self.setToolTip(f'{self.label}: {value} ({level})')
            self.updateGeometry()
            self.update()

    def sizeHint(self):
        lw = QFontMetrics(self.f_label).horizontalAdvance(self.label) + 5 if self.label else 0
        w = 22 + lw + QFontMetrics(self.f_value).horizontalAdvance(self.value) + 9
        return QSize(w, 30)

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        tok, shape = LEVEL[self.level]
        r = self.rect().adjusted(0, 3, -1, -3)
        p.setPen(QPen(QColor(T['line'] if self.level == 'normal' else T[tok]), 1))
        p.setBrush(QColor(T['sunken']))
        p.drawRoundedRect(r, 5, 5)
        draw_shape(p, shape, 12, self.height() / 2, 4.5, T[tok])
        p.setFont(self.f_label)
        p.setPen(QColor(T['muted']))
        x = 22
        p.drawText(QRectF(x, 0, 200, self.height()), Qt.AlignVCenter | Qt.AlignLeft, self.label)
        if self.label:
            x += QFontMetrics(self.f_label).horizontalAdvance(self.label) + 5
        p.setFont(self.f_value)
        p.setPen(QColor(TEXT_COLOR[self.level]))
        p.drawText(QRectF(x, 0, 400, self.height()), Qt.AlignVCenter | Qt.AlignLeft, self.value)


class ActionButton(QPushButton):
    """A command button. Blocked buttons stay clickable so a press is never
    silent: it explains why (log) instead of doing nothing. Pending shows
    from the press until the command answers."""

    def __init__(self, name, window, text=None, kind=None, height=None):
        super().__init__(text or TEXT.get(name, name))
        self.name, self.w = name, window
        self.base = text or TEXT.get(name, name)
        self.state, self.why = None, ''
        self.setFocusPolicy(Qt.NoFocus)
        self.setCursor(Qt.PointingHandCursor)
        if kind:
            self.setProperty('kind', kind)
        if height:
            self.setMinimumHeight(height)
        self.clicked.connect(self._click)
        self.show_state('idle')

    def _click(self):
        if self.state == 'blocked':
            self.w.on_blocked(self.name, self.why)
        elif self.state == 'pending' and self.name not in STOP_CONTROLS:
            self.w.log(f'{self.base.splitlines()[0]}: still waiting for the last press')
        else:
            self.w.on_press(self.name)

    def show_state(self, state, why='', text=None):
        restyle(self, state=state)
        self.state, self.why = state, why
        set_text(self, text or self.base)
        tip = (f'Not available: {why}' if state == 'blocked'
               else 'The next step' if state == 'next' else '')
        if self.name == 'stop_now':
            tip = STOP_TIP
        if self.toolTip() != tip:
            self.setToolTip(tip)


class DetentSlider(QWidget):
    """Named stops ordered soft -> stiff. No handle = Custom (hand-edited)."""

    picked = Signal(int)
    refused = Signal(str)

    def __init__(self, names, placeholders):
        super().__init__()
        self.names, self.ph = names, placeholders
        self.index, self.blocked, self.why = None, False, ''
        self.custom_text = 'CUSTOM'
        self.setMinimumHeight(52)
        self.setMinimumWidth(260)
        self.setCursor(Qt.PointingHandCursor)

    def set_index(self, i):
        if i != self.index:
            self.index = i
            self.update()

    def set_blocked(self, blocked, why):
        if (blocked, why) != (self.blocked, self.why):
            self.blocked, self.why = blocked, why
            self.setToolTip(f'Not available: {why}' if blocked else
                            'Gain presets, soft to stiff. * = placeholder, tune on the cell.')
            self.update()

    def _x(self, i):
        n = len(self.names)
        return 18 + i * (self.width() - 36) / max(1, n - 1)

    def mousePressEvent(self, e):
        if self.blocked:
            self.refused.emit(self.why)
            return
        x = e.position().x()
        i = min(range(len(self.names)), key=lambda k: abs(self._x(k) - x))
        self.picked.emit(i)

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        y = 22
        on = not self.blocked
        p.setPen(QPen(QColor(T['line'] if on else T['line_dim']), 3))
        p.drawLine(QPointF(self._x(0), y), QPointF(self._x(len(self.names) - 1), y))
        f = QFont(self.font())
        f.setPixelSize(S)
        p.setFont(f)
        for i, name in enumerate(self.names):
            x = self._x(i)
            sel = i == self.index
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(T['ink'] if sel and on else T['off_ink'] if not on
                              else T['ink2'] if sel else T['line']))
            p.drawEllipse(QPointF(x, y), 7 if sel else 4, 7 if sel else 4)
            p.setPen(QColor(T['ink'] if sel and on else T['muted']))
            p.drawText(QRectF(x - 45, y + 10, 90, 18), Qt.AlignHCenter,
                       name + ('*' if self.ph[i] else ''))
        if self.index is None:
            p.setPen(QColor(T['muted'] if self.custom_text == 'TRACK PROFILE' else T['amber']))
            p.drawText(QRectF(0, 0, self.width(), 13), Qt.AlignLeft, self.custom_text)


class RollingPlot(QWidget):
    """Last 30 s of one quantity, with labelled reference lines."""

    def __init__(self, title, unit):
        super().__init__()
        self.title, self.unit = title, unit
        self.data, self.refs, self.now = [], [], 0.0
        self.setMinimumHeight(58)

    def set_data(self, data, refs, now):
        self.data, self.refs, self.now = data, refs, now
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        f = QFont(MONO_FONT)
        f.setPixelSize(S)
        p.setFont(f)
        ml, mr, mt, mb = 34, 6, 16, 6
        w, h = self.width() - ml - mr, self.height() - mt - mb
        p.setPen(QColor(T['muted']))
        p.drawText(QRectF(0, 0, self.width(), 14), Qt.AlignLeft,
                   f'{self.title}  {self.unit}')
        pts = [(t, v) for t, v in self.data
               if v is not None and self.now - t < PLOT_WINDOW_S and not math.isnan(v)]
        lowest = min([r[0] for r in self.refs], default=1e-6)
        top = max([v for _, v in pts] + [lowest * 2.0, 1e-6]) * 1.12
        p.fillRect(QRectF(ml, mt, w, h), QColor(T['sunken']))

        def py(v):
            return mt + h - min(max(v / top, 0.0), 1.0) * h

        above = [text for v, text in self.refs if v > top]
        if above:
            p.drawText(QRectF(ml + 4, mt + 1, w - 8, 12), Qt.AlignLeft, ', '.join(above) + ' ↑')
        for v, text in self.refs:
            if v > top:
                continue
            y = py(v)
            p.setPen(QPen(QColor(T['muted']), 1, Qt.DashLine))
            p.drawLine(QPointF(ml, y), QPointF(ml + w, y))
            p.drawText(QRectF(ml, y - 13, w - 3, 12), Qt.AlignRight, text)
        p.setPen(QColor(T['muted']))
        p.drawText(QRectF(0, mt - 4, ml - 4, 12), Qt.AlignRight, f'{top:.3g}')
        p.drawText(QRectF(0, mt + h - 10, ml - 4, 12), Qt.AlignRight, '0')
        if len(pts) >= 2:
            path = QPainterPath()
            for k, (t, v) in enumerate(pts):
                pt = QPointF(ml + w - (self.now - t) / PLOT_WINDOW_S * w, py(v))
                path.moveTo(pt) if k == 0 else path.lineTo(pt)
            p.setPen(QPen(QColor(T['ink2']), 1.6))
            p.setBrush(Qt.NoBrush)
            p.drawPath(path)


class ForceBar(QWidget):
    """|F| ext against what this session's PRE-FLIGHT set."""

    def __init__(self, st):
        super().__init__()
        self.st, self.force, self.known = st, None, None
        self.setMinimumHeight(52)

    def set(self, force, thresholds):
        if (force, thresholds) != (self.force, self.known):
            self.force, self.known = force, thresholds
            self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        f = QFont(MONO_FONT)
        f.setPixelSize(S)
        p.setFont(f)
        full = 45.0
        x0, w, y, h = 2, self.width() - 4, 4, 12

        def px(v):
            return x0 + min(v / full, 1.0) * w

        p.fillRect(QRectF(x0, y, w, h), QColor(T['sunken']))
        if self.force is not None:
            warn = self.force > self.st.push_limit_n
            p.fillRect(QRectF(x0, y, px(self.force) - x0, h),
                       QColor(T['amber'] if warn else T['ink2']))
        marks = [(self.st.push_limit_n, 'push'), (self.st.controller_max_force_n, 'ctrl')]
        if self.known:
            marks += [(self.known['contact_n'], 'contact'), (self.known['reflex_n'], 'reflex')]
        for v, name in marks:
            x = px(v)
            p.setPen(QPen(QColor(T['ink'] if name == 'reflex' else T['muted']),
                          1.5 if name == 'reflex' else 1))
            p.drawLine(QPointF(x, y - 2), QPointF(x, y + h + 2))
            p.setPen(QColor(T['muted']))
            p.drawText(QRectF(x - 20, y + h + 2, 40, 14), Qt.AlignHCenter, f'{v:.0f}')
        legend = (f'N:  push {self.st.push_limit_n:g} · ctrl {self.st.controller_max_force_n:g}'
                  + (f' · contact {self.known["contact_n"]:g} · reflex {self.known["reflex_n"]:g}'
                     if self.known else ' · reflex: run PRE-FLIGHT'))
        lf = QFont(self.font())
        lf.setPixelSize(S)
        p.setFont(lf)
        p.drawText(QRectF(x0, y + h + 17, w, 14), Qt.AlignLeft, legend)


class CameraThumb(QLabel):
    clicked = Signal()

    def __init__(self, w, h):
        super().__init__()
        self.setFixedSize(w, h)
        self.setAlignment(Qt.AlignCenter)
        self.setCursor(Qt.PointingHandCursor)
        self.setStyleSheet(f'background: {T["sunken"]}; border: 1px solid {T["line_dim"]};'
                           f' color: {T["muted"]}; font-size: {S}px;')
        self.setToolTip('Click to enlarge')

    def mousePressEvent(self, _e):
        self.clicked.emit()


class Section(QFrame):
    """A titled panel. Inactive = dashed and dim, with the reason on hover;
    its entry controls stay live (PRE-FLIGHT and HOLD enter TORQUE)."""

    def __init__(self, title):
        super().__init__()
        self.setObjectName('panel')
        self.v = QVBoxLayout(self)
        self.v.setContentsMargins(12, 10, 12, 12)
        self.v.setSpacing(8)
        head = QHBoxLayout()
        self.title = lab(title, 'section')
        self.hint = lab('', 'caption')
        self.hint.setWordWrap(True)
        self.hint.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        head.addWidget(self.title)
        head.addWidget(self.hint, 1)
        self.v.addLayout(head)

    def set_inactive(self, inactive, why):
        restyle(self, inactive='true' if inactive else 'false')
        set_text(self.hint, why if inactive else '')
        tip = f'Inactive: {why}' if inactive else ''
        if self.toolTip() != tip:
            self.setToolTip(tip)


# ---------------------------------------------------------------- window

class MainWindow(QMainWindow):
    posted = Signal(object)

    def __init__(self, backend, settings, presets, mock=False):
        super().__init__()
        self.b, self.st, self.presets, self.mock = backend, settings, presets, mock
        self.pending = {}
        self.buttons = {}
        self.confirm = None          # (name, args, deadline)
        self.overlay_auto = False
        self.overlay_ok_since = None
        self.pending_gains = None
        self.speed_dragging = False
        self.tspeed_dragging = False
        self._image_seq = None
        self._qimg = None
        self.may_close = False
        self.closing = False
        self._last = None
        self.posted.connect(self._on_event, Qt.QueuedConnection)
        self.setWindowTitle('FR3 Cell Control' + ('  -  MOCK' if mock else ''))
        self.setStyleSheet(QSS)
        self._build()
        self.resize(1440, 960)
        self.render_timer = QTimer(self, interval=100, timeout=self.render)
        self.render_timer.start()
        self.hb_timer = QTimer(self, interval=100, timeout=self.b.heartbeat)
        self.hb_timer.start()
        # The drawer survives restarts (persist.py); only a live backend has
        # a file, so the frozen scenes and tests never read the operator's.
        self.settings_path = getattr(backend, 'settings_path', None)
        self._save_timer = QTimer(self, singleShot=True, interval=400,
                                  timeout=self._save_settings)
        if self.settings_path is not None:
            self._restore_settings()
            self._watch_drawer()

    # ------------------------------------------------------------ plumbing

    def post(self, *event):
        """Any thread: hand an event to the GUI thread (queued signal)."""
        self.posted.emit(event)

    def _on_event(self, e):
        kind = e[0]
        if kind == 'log':
            self.log(e[1])
        elif kind == 'pending':
            self.pending[e[1]] = self.b.now()
        elif kind == 'done':
            _, name, ok, msg = e
            self.pending.pop(name, None)
            if msg:
                self.log(f'{name}: {msg}' if ok else f'{name} FAILED: {msg}')
            if name == 'close_handoff':
                if ok:
                    self.may_close = True
                    self.close()
                else:
                    self.closing = False
        elif kind == 'gains_applied':
            if self.settings_path is not None:
                self._save_timer.start()
        elif kind == 'floor_mm':
            self.floor_edit.setText(f'{e[1]:.1f}')
            self._floor_changed()
            if self.settings_path is not None:
                self._save_timer.start()

    def log(self, msg):
        stamp = datetime.datetime.now().strftime('%H:%M:%S')
        self.log_box.appendPlainText(f'{stamp}  {msg}')

    # ------------------------------------------------------------ build

    def _build(self):
        central = QWidget()
        central.setObjectName('central')
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_status_bar())
        bw = QWidget()
        bl = QVBoxLayout(bw)
        bl.setContentsMargins(12, 10, 12, 0)
        bl.addWidget(self._build_banner())
        root.addWidget(bw)
        body = QHBoxLayout()
        body.setContentsMargins(12, 10, 12, 10)
        body.setSpacing(12)
        body.addWidget(self._build_left())
        body.addWidget(self._build_center(), 1)
        self.drawer = self._build_drawer()
        self.drawer.setVisible(False)
        body.addWidget(self.drawer)
        root.addLayout(body, 1)
        root.addWidget(self._build_stop_bar())
        self.overlay = self._build_overlay(central)

    def _build_status_bar(self):
        bar = panel('bar')
        h = QHBoxLayout(bar)
        h.setContentsMargins(12, 6, 12, 6)
        h.setSpacing(6)
        self.chips = [Chip(n) for n in ('FRANKA', 'MOVEIT', 'RT', 'CONTROLLER',
                                         'PRE-FLIGHT', 'VISION', 'GATE')]
        for c in self.chips:
            h.addWidget(c)
        h.addStretch(1)
        if self.mock:
            badge = lab('MOCK', 'caption')
            badge.setToolTip('Mock cell on the isolated DDS domain 88 - no robot')
            badge.setStyleSheet(f'border: 1px solid {T["line"]}; border-radius: 4px;'
                                f' padding: 3px 8px; color: {T["ink2"]};')
            h.addWidget(badge)
        self.calib_chip = Chip('CALIB')
        self.calib_chip.setCursor(Qt.PointingHandCursor)
        self.calib_chip.setToolTip('Hand-eye calibration residuals - click for details')
        self.calib_chip.mousePressEvent = lambda _e: self._toggle_drawer(focus='calib')
        h.addWidget(self.calib_chip)
        self.rec = Chip('REC')
        self.rec.setCursor(Qt.PointingHandCursor)
        self.rec.mousePressEvent = lambda _e: self.on_press('record')
        self.rec.setToolTip('Record a rosbag (auto-named in the day log folder)')
        h.addWidget(self.rec)
        return bar

    def _build_banner(self):
        self.banner = panel('banner')
        self.banner.setMinimumHeight(58)
        h = QHBoxLayout(self.banner)
        h.setContentsMargins(16, 8, 12, 8)
        v = QVBoxLayout()
        v.setSpacing(2)
        self.banner_title = QLabel('')
        self.banner_title.setObjectName('bannerTitle')
        self.banner_detail = QLabel('')
        self.banner_detail.setObjectName('bannerDetail')
        self.banner_detail.setWordWrap(True)
        v.addWidget(self.banner_title)
        v.addWidget(self.banner_detail)
        h.addLayout(v, 1)
        self.banner_btn = ActionButton('recover', self, 'RECOVER', height=40)
        self.banner_btn.setObjectName('bannerAction')
        self.banner_btn.setMinimumWidth(130)
        self.banner_btn.setVisible(False)
        h.addWidget(self.banner_btn)
        return self.banner

    def _build_left(self):
        col = QWidget()
        col.setFixedWidth(288)
        v = QVBoxLayout(col)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(8)
        cam = panel()
        cv = QVBoxLayout(cam)
        cv.setContentsMargins(12, 8, 12, 8)
        cv.setSpacing(4)
        hdr = QHBoxLayout()
        hdr.addWidget(lab('CAMERA', 'section'))
        hdr.addStretch(1)
        self.cam_age = lab('', 'caption')
        hdr.addWidget(self.cam_age)
        cv.addLayout(hdr)
        self.thumb = CameraThumb(240, 180)
        self.thumb.clicked.connect(lambda: self._open_overlay(False, 'opened by hand'))
        cv.addWidget(self.thumb, 0, Qt.AlignHCenter)
        v.addWidget(cam)

        tel = panel()
        self.tel_panel = tel
        g = QGridLayout(tel)
        g.setContentsMargins(12, 8, 12, 8)
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(1)
        self.tiles = {}
        rows = [('MARKER · CAMERA FRAME', None),
                (('dist', 'mm'), ('lateral', 'mm')), (('tilt', 'deg'), ('in-plane', 'deg')),
                ('ARM · BASE FRAME', None),
                (('TCP Z', 'mm'), ('lead', 'mm')), (('|F| ext', 'N'), ('Fz ext', 'N'))]
        r = 0
        for row in rows:
            if row[1] is None:
                head = lab(row[0], 'section')
                head.setContentsMargins(0, 4 if r else 0, 0, 3)
                g.addWidget(head, r, 0, 1, 2)
                r += 1
                continue
            for c, (key, unit) in enumerate(row):
                cell = QWidget()
                cl = QVBoxLayout(cell)
                cl.setContentsMargins(0, 0, 0, 2)
                cl.setSpacing(0)
                cl.addWidget(lab(key.upper(), 'caption'))
                vr = QHBoxLayout()
                vr.setSpacing(4)
                val = lab('—', 'value')
                vr.addWidget(val)
                vr.addWidget(lab(unit, 'unit'), 0, Qt.AlignBottom)
                vr.addStretch(1)
                cl.addLayout(vr)
                g.addWidget(cell, r, c)
                self.tiles[key] = val
            r += 1
        self.force_bar = ForceBar(self.st)
        g.addWidget(self.force_bar, r, 0, 1, 2)
        r += 1
        self.joints_lbl = lab('joints —', 'mono')
        g.addWidget(self.joints_lbl, r, 0, 1, 2)
        v.addWidget(tel)

        plots = panel()
        pv = QVBoxLayout(plots)
        pv.setContentsMargins(12, 8, 12, 8)
        pv.setSpacing(2)
        self.plot_title = lab('', 'section')
        pv.addWidget(self.plot_title)
        self.plots = [RollingPlot('', '') for _ in range(3)]
        for pl in self.plots:
            pv.addWidget(pl)
        v.addWidget(plots, 1)
        return col

    def _combo(self, values, current, on_change, width=None):
        cb = QComboBox()
        cb.addItems(values)
        cb.setCurrentText(current)
        cb.setFocusPolicy(Qt.NoFocus)
        if width:
            cb.setFixedWidth(width)
        cb.currentTextChanged.connect(on_change)
        return cb

    def _build_center(self):
        col = QWidget()
        v = QVBoxLayout(col)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)

        top = panel()
        tg = QGridLayout(top)
        tg.setContentsMargins(12, 10, 12, 10)
        tg.setHorizontalSpacing(12)
        tg.addWidget(lab('MODE', 'section'), 0, 0)
        self.mode_lbl = lab('', 'mode')
        tg.addWidget(self.mode_lbl, 0, 1, 1, 2)
        self.activity = Chip('')
        self.activity.setVisible(False)
        tg.addWidget(self.activity, 0, 3)
        self.drawer_btn = QPushButton('SETTINGS  ▸')
        self.drawer_btn.setProperty('kind', 'quiet')
        self.drawer_btn.setFocusPolicy(Qt.NoFocus)
        self.drawer_btn.clicked.connect(lambda: self._toggle_drawer())
        tg.addWidget(self.drawer_btn, 0, 6)
        tg.setColumnStretch(4, 1)

        tg.addWidget(lab('MOTION SPEED', 'section'), 1, 0)
        self.speed = QSlider(Qt.Horizontal)
        self.speed.setRange(1, 100)
        self.speed.setValue(int(self.st.speed_default_pct))
        self.speed.setFocusPolicy(Qt.NoFocus)
        self.speed.setMinimumWidth(200)
        self.speed.valueChanged.connect(self._speed_moved)
        self.speed.sliderPressed.connect(lambda: setattr(self, 'speed_dragging', True))
        self.speed.sliderReleased.connect(self._speed_released)
        tg.addWidget(self.speed, 1, 1, 1, 4)
        self.speed_val = lab('', 'mono')
        self.speed_val.setMinimumWidth(52)
        tg.addWidget(self.speed_val, 1, 5)
        self.speed_txt = lab('', 'caption', wrap=True)
        tg.addWidget(self.speed_txt, 2, 1, 1, 4)
        self.speed_confirm = QPushButton('')
        self.speed_confirm.setFocusPolicy(Qt.NoFocus)
        restyle(self.speed_confirm, state='confirm')
        self.speed_confirm.setVisible(False)
        self.speed_confirm.clicked.connect(self._speed_confirmed)
        self.speed_confirm.setMaximumWidth(260)
        tg.addWidget(self.speed_confirm, 2, 5, 1, 2)
        v.addWidget(top)

        # ---- POSITION
        self.pos_sec = Section('POSITION  ·  MoveIt + fr3_arm_controller  ·  align is optional')
        r1 = QHBoxLayout()
        r1.setSpacing(8)
        for n in ('translate', 'level', 'inplane'):
            b = self._btn(n, height=40)
            r1.addWidget(b, 1)
        self.pos_sec.v.addLayout(r1)
        r2 = QHBoxLayout()
        r2.setSpacing(8)
        r2.addWidget(self._btn('auto_converge', height=46), 3)
        r2.addSpacing(16)
        r2.addWidget(lab('SAVED POSES', 'section'))
        self.pose_btns = {}
        for name in ('home', 'pre_align'):
            b = self._btn(f'goto:{name}', text=f'go  {name.replace("_", "-")}', height=40)
            r2.addWidget(b, 1)
            self.pose_btns[name] = b
        self.pos_sec.v.addLayout(r2)
        self.pose_confirm = QPushButton('')
        self.pose_confirm.setFocusPolicy(Qt.NoFocus)
        restyle(self.pose_confirm, state='confirm')
        self.pose_confirm.setVisible(False)
        self.pose_confirm.clicked.connect(self._confirmed)
        self.pos_sec.v.addWidget(self.pose_confirm)
        v.addWidget(self.pos_sec)

        # ---- TORQUE
        self.tq_sec = Section('TORQUE  ·  cartesian_impedance_stroke_controller')
        self.tq_sec.v.addWidget(self._btn('preflight', height=40,
                                          text='PRE-FLIGHT   ·   reflex thresholds, FCI payload'
                                               ' zeroed   ·   entry gate'))
        r = QHBoxLayout()
        r.setSpacing(8)
        r.addWidget(self._btn('float', height=44), 1)
        r.addWidget(self._btn('hold', height=44), 1)
        r.addSpacing(24)
        r.addWidget(self._btn('track', height=44), 2)
        r.addWidget(lab('over lead', 'caption'))
        self.policy = self._combo(C.OVER_LEAD, C.OVER_LEAD_DEFAULT,
                                  lambda t: self.b.command('policy', t), 80)
        self.policy.setToolTip('What tracking does when the goal is beyond the lead cap: '
                               'hold = wait, stop = end, clamp = follow capped')
        r.addWidget(self.policy)
        self.tq_sec.v.addLayout(r)
        r = QHBoxLayout()
        r.setSpacing(8)
        r.addWidget(lab('TRACK SPEED', 'section'))
        self.tspeed = QSlider(Qt.Horizontal)
        self.tspeed.setRange(1, 100)
        self.tspeed.setValue(int(self.st.track_speed_default_pct))
        self.tspeed.setFocusPolicy(Qt.NoFocus)
        self.tspeed.setMinimumWidth(160)
        self.tspeed.valueChanged.connect(self._tspeed_moved)
        self.tspeed.sliderPressed.connect(lambda: setattr(self, 'tspeed_dragging', True))
        self.tspeed.sliderReleased.connect(self._tspeed_released)
        r.addWidget(self.tspeed, 2)
        self.tspeed_val = lab('', 'mono')
        self.tspeed_val.setMinimumWidth(52)
        r.addWidget(self.tspeed_val)
        r.addWidget(self._btn('track_fast'))
        self.tspeed_txt = lab('', 'caption', wrap=True)
        r.addWidget(self.tspeed_txt, 3)
        self.tspeed_confirm = QPushButton('')
        self.tspeed_confirm.setFocusPolicy(Qt.NoFocus)
        self.tspeed_confirm.setMaximumWidth(240)
        restyle(self.tspeed_confirm, state='confirm')
        self.tspeed_confirm.setVisible(False)
        self.tspeed_confirm.clicked.connect(self._confirmed)
        r.addWidget(self.tspeed_confirm)
        self.tq_sec.v.addLayout(r)
        r = QHBoxLayout()
        r.setSpacing(8)
        r.addWidget(self._btn('setpoint_minus', height=36), 1)
        self.sp_mm = self._combo(C.SETPOINT_MM, C.SETPOINT_MM_DEFAULT,
                                 lambda t: self.b.set_param('setpoint_mm', float(t)), 64)
        r.addWidget(self.sp_mm)
        r.addWidget(lab('mm along', 'caption'))
        self.sp_axis = self._combo(C.AXES, C.AXES[0], lambda t: self.b.set_param('axis', t), 140)
        r.addWidget(self.sp_axis)
        r.addWidget(self._btn('setpoint_plus', height=36), 1)
        r.addSpacing(16)
        r.addWidget(self._btn('hold_here', height=36), 1)
        self.tq_sec.v.addLayout(r)

        gh = QHBoxLayout()
        gh.addWidget(lab('GAIN PRESET  ·  soft → stiff  ·  free space only', 'section'))
        self.gain_state = lab('', 'caption', wrap=True)
        self.gain_state.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        gh.addWidget(self.gain_state, 1)
        self.tq_sec.v.addSpacing(4)
        self.tq_sec.v.addLayout(gh)
        gains = QHBoxLayout()
        gains.setSpacing(12)
        names = [p['name'] for p in self.presets]
        self.preset = DetentSlider(names, [p.get('placeholder', False) for p in self.presets])
        self.preset.picked.connect(self._preset_picked)
        self.preset.refused.connect(lambda why: self.on_blocked('preset', why))
        gains.addWidget(self.preset, 1)
        gains.addWidget(self._btn('apply_gains', height=36))
        self.tq_sec.v.addLayout(gains)
        self.gain_lbls = {}
        gg = QGridLayout()
        gg.setHorizontalSpacing(8)
        gg.setVerticalSpacing(0)
        for i, k in enumerate(logic.GAIN_KEYS):
            gg.addWidget(lab(f'{C.GAIN_LABELS[k]}  {C.GAIN_UNITS[k]}', 'caption'), 0, i)
            val = lab('—', 'mono')
            gg.addWidget(val, 1, i)
            gg.setColumnStretch(i, 1)
            self.gain_lbls[k] = val
        self.tq_sec.v.addLayout(gg)

        rr = QHBoxLayout()
        self.torque_note = lab('', 'caption')
        rr.addWidget(self.torque_note, 1)
        rel = self._btn('release', height=34)
        rel.setFixedWidth(250)
        rr.addWidget(rel)
        self.tq_sec.v.addSpacing(6)
        self.tq_sec.v.addLayout(rr)
        v.addWidget(self.tq_sec)
        v.addStretch(1)
        return col

    def _field(self, text, on_edit, width=70):
        e = QLineEdit(text)
        e.setFixedWidth(width)
        e.editingFinished.connect(on_edit)
        return e

    def _group(self, v, title):
        v.addSpacing(6)
        v.addWidget(lab(title, 'section'))
        g = QGridLayout()
        g.setHorizontalSpacing(8)
        g.setVerticalSpacing(5)
        v.addLayout(g)
        return g

    def _build_drawer(self):
        area = QScrollArea()
        area.setFixedWidth(360)
        area.setWidgetResizable(True)
        inner = panel()
        v = QVBoxLayout(inner)
        v.setContentsMargins(14, 12, 14, 12)
        v.setSpacing(4)
        st = self.st

        g = self._group(v, 'TARGET + SAFETY')
        self.target = self._combo(C.TARGET_MM, C.TARGET_MM_DEFAULT,
                                  lambda t: self.b.set_param('target_m', float(t) / 1000))
        self.tol = self._combo(C.POS_TOL_MM, C.POS_TOL_MM_DEFAULT,
                               lambda t: self.b.set_param('tol_m', float(t) / 1000))
        self.inplane = self._combo(C.INPLANE, C.INPLANE_DEFAULT, self._inplane_changed)
        self.floor_edit = self._field(C.FLOOR_MM_DEFAULT, self._floor_changed)
        for r, (name, w) in enumerate((('standoff  mm', self.target), ('pos tol  mm', self.tol),
                                       ('in-plane  deg', self.inplane),
                                       ('ALIGN Z floor  mm', self.floor_edit))):
            g.addWidget(lab(name, 'caption'), r, 0)
            g.addWidget(w, r, 1)
        af = self._btn('auto_floor', text='Auto floor from here')
        af.setProperty('kind', 'quiet')
        g.addWidget(af, 4, 0, 1, 2)
        self.gate_cb = QCheckBox('robot-state gate (move only in MOVE)')
        self.gate_cb.setChecked(True)
        self.gate_cb.setFocusPolicy(Qt.NoFocus)
        self.gate_cb.toggled.connect(lambda on: self.b.command('gate', on))
        g.addWidget(self.gate_cb, 5, 0, 1, 2)
        g.addWidget(lab('workspace box  (fr3_link0, mm; floor above)', 'caption'), 6, 0, 1, 2)
        self.box = {}
        for r, (key, lo, hi) in enumerate((('x', *st.box_x), ('y', *st.box_y))):
            self.box[key] = (self._field(f'{lo*1000:.0f}', self._box_changed, 60),
                             self._field(f'{hi*1000:.0f}', self._box_changed, 60))
            row = QHBoxLayout()
            row.addWidget(self.box[key][0])
            row.addWidget(lab('to', 'caption'))
            row.addWidget(self.box[key][1])
            row.addStretch(1)
            g.addWidget(lab(f'{key}', 'caption'), 7 + r, 0)
            g.addLayout(row, 7 + r, 1)
        self.box['z'] = self._field(f'{st.box_z_max*1000:.0f}', self._box_changed, 60)
        g.addWidget(lab('z max', 'caption'), 9, 0)
        g.addWidget(self.box['z'], 9, 1)

        g = self._group(v, 'MOTION  (position)')
        self.step = self._combo(C.STEP_MM, C.STEP_MM_DEFAULT,
                                lambda t: self.b.set_param('step_m', float(t) / 1000))
        self.rot = self._combo(C.ROT_DEG, C.ROT_DEG_DEFAULT,
                               lambda t: self.b.set_param('rot_deg', float(t)))
        g.addWidget(lab('translate step  mm', 'caption'), 0, 0)
        g.addWidget(self.step, 0, 1)
        g.addWidget(lab('level / in-plane step  deg', 'caption'), 1, 0)
        g.addWidget(self.rot, 1, 1)

        g = self._group(v, 'TRACK')
        self.entry_mm = self._spin(st.track_entry_mm, 1, 200, 1, 'track_entry_mm')
        self.entry_deg = self._spin(st.track_entry_deg, 0.5, 30, 1, 'track_entry_deg')
        self.loss = self._combo(list(logic.MARKER_LOSS_POLICIES), st.marker_loss_policy,
                                lambda t: self.b.set_param('marker_loss', t))
        self.loss.setToolTip('hold = tracking_node holds by itself; stop / release after N ms'
                             ' are done by this GUI (TODO C2: move into the node)')
        self.loss_ms = self._spin(st.marker_loss_ms, 100, 10000, 0, None)
        self.loss_ms.valueChanged.connect(lambda x: self.b.set_param('marker_loss_ms', x))
        self.jump = self._spin(st.pose_jump_mm, 1, 200, 0, 'pose_jump_mm')
        for r, (name, w) in enumerate((('entry  |e| below  mm', self.entry_mm),
                                       ('entry  tilt below  deg', self.entry_deg),
                                       ('marker loss', self.loss), ('...after  ms', self.loss_ms),
                                       ('pose jump alarm  mm', self.jump))):
            g.addWidget(lab(name, 'caption'), r, 0)
            g.addWidget(w, r, 1)
        # An interlock relaxed, so never kept: off at every launch.
        self.blind_cb = QCheckBox('TRACK may start without the marker')
        self.blind_cb.setFocusPolicy(Qt.NoFocus)
        self.blind_cb.setToolTip('Skips every marker check at START (visible, entry, lead cap). '
                                 'The node holds until it sees the marker, then approaches it at '
                                 'TRACK SPEED - or holds past its 60 mm lead cap under over lead '
                                 "'hold'. Off at every launch.")
        self.blind_cb.toggled.connect(lambda on: self.b.set_param('track_blind', bool(on)))
        g.addWidget(self.blind_cb, 5, 0, 1, 2)

        g = self._group(v, 'GAINS  (numeric - pending until APPLY)')
        self.gain_edits = {}
        for r, k in enumerate(logic.GAIN_KEYS):
            lo, hi = C.GAIN_LIMITS[k]
            e = self._field('', self._gain_edited, 80)
            e.setToolTip(f'{C.GAIN_LABELS[k]}  [{lo:g}, {hi:g}] {C.GAIN_UNITS[k]}')
            self.gain_edits[k] = e
            g.addWidget(lab(f'{C.GAIN_LABELS[k]}  {C.GAIN_UNITS[k]}', 'caption'), r, 0)
            g.addWidget(e, r, 1)
            g.addWidget(lab(f'{lo:g}-{hi:g}', 'caption'), r, 2)

        g = self._group(v, 'SAVED POSES  (TCP, through ALIGN\'s path)')
        self.pose_lbls = {}
        for r, name in enumerate(('home', 'pre_align')):
            b = self._btn(f'teach:{name}', text=f'TEACH {name.replace("_", "-")}')
            b.setProperty('kind', 'quiet')
            g.addWidget(b, r, 0)
            self.pose_lbls[name] = lab('', 'caption')
            g.addWidget(self.pose_lbls[name], r, 1)

        g = self._group(v, 'CALIBRATION  ·  hand-eye')
        self.calib_lbls = {}
        for r, k in enumerate(('source', 'frames', 'residual', 'validated', 'status')):
            g.addWidget(lab(k, 'caption'), r, 0)
            self.calib_lbls[k] = lab('—', 'caption')
            self.calib_lbls[k].setWordWrap(True)
            g.addWidget(self.calib_lbls[k], r, 1)
        rb = self._btn('reload_calib', text='Reload calibration')
        rb.setProperty('kind', 'quiet')
        g.addWidget(rb, 5, 0, 1, 2)

        g = self._group(v, 'CAMERA')
        self.cam_cb = QCheckBox('show the marker view (off = no frames on DDS)')
        self.cam_cb.setChecked(True)
        self.cam_cb.setFocusPolicy(Qt.NoFocus)
        self.cam_cb.toggled.connect(lambda on: self.b.camera(on))
        g.addWidget(self.cam_cb, 0, 0, 1, 2)

        if self.mock:
            g = self._group(v, 'MOCK FAULTS  (mock cell only)')
            for i, f in enumerate(('reflex', 'driver_down', 'vision_stale', 'marker_lost',
                                   'pose_jump', 'rt_dip', 'user_stop', 'push',
                                   'moveit_down', 'clear')):
                b = QPushButton(f)
                b.setProperty('kind', 'quiet')
                b.setFocusPolicy(Qt.NoFocus)
                b.clicked.connect(lambda _c=False, f=f: self.b.fault(f))
                g.addWidget(b, i // 2, i % 2)
        v.addStretch(1)
        area.setWidget(inner)
        return area

    def _spin(self, value, lo, hi, decimals, setting):
        sp = QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setDecimals(decimals)
        sp.setValue(value)
        sp.setFixedWidth(90)
        sp.setFocusPolicy(Qt.ClickFocus)
        if setting:
            sp.valueChanged.connect(lambda x, k=setting: self._set_setting(k, x))
        return sp

    def _build_stop_bar(self):
        bar = panel('stopbar')
        h = QHBoxLayout(bar)
        h.setContentsMargins(12, 8, 12, 8)
        h.setSpacing(10)
        stop = self._btn('stop_now', text='STOP NOW   (Esc)')
        stop.setObjectName('stopNow')
        stop.setFixedSize(280, 60)
        h.addWidget(stop)
        pause = self._btn('pause')
        pause.setFixedSize(120, 60)
        h.addWidget(pause)
        after = self._btn('stop_after', kind='quiet')
        after.setFixedSize(140, 60)
        h.addWidget(after)
        h.addSpacing(14)
        lv = QVBoxLayout()
        lv.setSpacing(2)
        lh = QHBoxLayout()
        lh.addWidget(lab('LOG', 'section'))
        lh.addStretch(1)
        self.log_toggle = QPushButton('expand ▴')
        self.log_toggle.setProperty('kind', 'quiet')
        self.log_toggle.setFocusPolicy(Qt.NoFocus)
        self.log_toggle.clicked.connect(self._toggle_log)
        lh.addWidget(self.log_toggle)
        lv.addLayout(lh)
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(5000)
        self.log_box.setFocusPolicy(Qt.ClickFocus)
        self._log_lines(3)
        lv.addWidget(self.log_box)
        h.addLayout(lv, 1)
        return bar

    def _log_lines(self, n):
        fm = self.log_box.fontMetrics()
        self.log_box.setFixedHeight(int(fm.lineSpacing() * n + 12))

    def _toggle_log(self):
        big = self.log_toggle.text().startswith('expand')
        self._log_lines(16 if big else 3)
        self.log_toggle.setText('collapse ▾' if big else 'expand ▴')
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())

    def _build_overlay(self, parent):
        ov = QFrame(parent)
        ov.setStyleSheet(f'background: {T["surface"]}; border: 2px solid {T["line"]};'
                         ' border-radius: 6px;')
        v = QVBoxLayout(ov)
        v.setContentsMargins(10, 8, 10, 10)
        self.ov_reason = QLabel('')
        self.ov_reason.setStyleSheet(f'color: {T["amber"]}; font-weight: bold; border: none;')
        v.addWidget(self.ov_reason)
        self.ov_img = QLabel('')
        self.ov_img.setFixedSize(400, 300)
        self.ov_img.setAlignment(Qt.AlignCenter)
        self.ov_img.setStyleSheet(f'background: {T["sunken"]}; border: none;'
                                  f' color: {T["muted"]};')
        v.addWidget(self.ov_img)
        hint = QLabel('click to close')
        hint.setStyleSheet(f'color: {T["muted"]}; border: none; font-size: {S}px;')
        v.addWidget(hint)
        ov.mousePressEvent = lambda _e: self._close_overlay()
        ov.setVisible(False)
        return ov

    def _btn(self, name, text=None, kind=None, height=None):
        b = ActionButton(name, self, text, kind, height)
        self.buttons[name] = b
        return b

    # ------------------------------------------------------------ inputs

    def on_press(self, name):
        """A live button: start its command (the backend marks it pending)."""
        if name == 'track' and self._last is not None and logic.tracking(self._last, self.st):
            self.b.command('end_track')
            return
        if name == 'track_fast':
            self.b.command(name, not (self._last is not None and self._last.track_fast))
            return
        if name.startswith('goto:'):
            pose = name.split(':', 1)[1]
            info = self.b.pose_info().get(pose)
            dist = info[1] if info else None
            if dist is not None and dist[0] > self.st.pose_confirm_mm:
                self._arm_confirm(self.pose_confirm, name, (),
                                  f'CONFIRM: go {dist[0]:.0f} mm / {dist[1]:.0f} deg to '
                                  f'{pose.replace("_", "-")}  (straight line, '
                                  f'{self.speed.value()} % speed)')
                return
        self.b.command(name)

    def on_blocked(self, name, why):
        label = TEXT.get(name, name).splitlines()[0]
        self.log(f'{label}: not available - {why}')
        if name in TORQUE_ENTRY + ('setpoint_minus', 'setpoint_plus', 'hold_here', 'track'):
            self.b.command('torque_attempt')

    def _arm_confirm(self, button, name, args, text, hold=False):
        """Show an inline confirm; it disarms after CONFIRM_S unless hold
        (a pending torque speed stays offered until it is applied)."""
        deadline = float('inf') if hold else time.monotonic() + CONFIRM_S
        self.confirm = (name, args, deadline, button)
        button.setText(text)
        button.setVisible(True)

    def _confirmed(self):
        if self.confirm is None:
            return
        name, args, _deadline, button = self.confirm
        self.confirm = None
        button.setVisible(False)
        self.b.command(name, *args)

    def _speed_moved(self, v):
        self.b.set_param('speed_pct', float(v))
        if not self.speed_dragging and not self.speed.isSliderDown():
            self._speed_released()

    def _speed_released(self):
        self.speed_dragging = False
        s = self._last
        if s is None or logic.mode(s) != 'TORQUE':
            return
        pct = float(self.speed.value())
        if logic.speed_needs_confirm(pct, self.st):
            t = logic.speed_torque(pct, self.st)
            self._arm_confirm(self.speed_confirm, 'speed', (pct,),
                              f'CONFIRM {pct:.0f} %  →  {t["setpoint_slew_mps"]*1000:.0f} mm/s')
            return
        self.speed_confirm.setVisible(False)
        self.b.command('speed', pct)

    def _tspeed_moved(self, _v):
        if not self.tspeed_dragging and not self.tspeed.isSliderDown():
            self._tspeed_released()

    def _tspeed_released(self):
        """TRACK SPEED: stored for the next START, written live while
        tracking; above the confirm threshold only after CONFIRM."""
        self.tspeed_dragging = False
        pct = float(self.tspeed.value())
        if logic.speed_needs_confirm(pct, self.st):
            t = logic.speed_torque(pct, self.st)
            self._arm_confirm(self.tspeed_confirm, 'track_speed', (pct,),
                              f'CONFIRM {pct:.0f} %  →  {t["setpoint_slew_mps"]*1000:.0f} mm/s')
            return
        self.tspeed_confirm.setVisible(False)
        if self.confirm and self.confirm[0] == 'track_speed':
            self.confirm = None
        self.b.command('track_speed', pct)

    def _speed_confirmed(self):
        self._confirmed()

    def _inplane_changed(self, t):
        self.b.set_param('inplane_target', None if t == 'off' else float(t))

    def _floor_changed(self):
        t = self.floor_edit.text().strip()
        try:
            self.b.set_param('floor_m', float(t) / 1000 if t else None)
            restyle(self.floor_edit, pending='false')
        except ValueError:
            restyle(self.floor_edit, pending='true')

    def _box_changed(self, quiet=False):
        try:
            box_x = (float(self.box['x'][0].text()) / 1000, float(self.box['x'][1].text()) / 1000)
            box_y = (float(self.box['y'][0].text()) / 1000, float(self.box['y'][1].text()) / 1000)
            z = float(self.box['z'].text()) / 1000
        except ValueError:
            self.log('workspace box: not a number - unchanged')
            return
        self.st = dataclasses.replace(self.st, box_x=box_x, box_y=box_y, box_z_max=z)
        self.b.set_settings(self.st)
        if not quiet:
            self.log(f'workspace box x {box_x} y {box_y} z max {z} m')

    def _set_setting(self, key, value):
        self.st = dataclasses.replace(self.st, **{key: float(value)})
        self.b.set_settings(self.st)

    def _preset_picked(self, i):
        p = self.presets[i]
        self._set_pending({k: float(p[k]) for k in logic.GAIN_KEYS})
        self.log(f'preset {p["name"]}' + (' (placeholder values)' if p.get('placeholder')
                                          else '') + ' - pending until APPLY GAINS')

    def _gain_edited(self):
        vals = dict(self.pending_gains or {})
        for k, e in self.gain_edits.items():
            try:
                vals[k] = float(e.text())
            except ValueError:
                pass
        self._set_pending(vals)

    def _set_pending(self, vals):
        self.pending_gains = vals
        self.b.set_pending_gains(dict(vals))
        for k, e in self.gain_edits.items():
            if k in vals and e.text() != f'{vals[k]:g}':
                e.setText(f'{vals[k]:g}')

    def _toggle_drawer(self, focus=None):
        show = not self.drawer.isVisible() or focus == 'calib'
        self.drawer.setVisible(show)
        self.drawer_btn.setText('SETTINGS  ◂' if show else 'SETTINGS  ▸')
        if focus == 'calib' and show:
            QApplication.processEvents()        # lay the drawer out before scrolling it
            self.drawer.ensureWidgetVisible(self.calib_lbls['status'], 0, 120)

    def _open_overlay(self, auto, reason):
        self.overlay_auto, self.overlay_ok_since = auto, None
        self.ov_reason.setText(('VISION EVENT: ' if auto else '') + reason)
        self._place_overlay()
        self.overlay.setVisible(True)
        self.overlay.raise_()
        self._paint_image(force=True)

    def _close_overlay(self):
        self.overlay.setVisible(False)
        self.overlay_auto = False

    def _place_overlay(self):
        self.overlay.adjustSize()
        g = self.tel_panel.mapTo(self.centralWidget(), self.tel_panel.rect().topLeft())
        self.overlay.move(g.x(), g.y())

    def closeEvent(self, e):
        """Never leave the arm on the impedance controller (the Tk panel's on_close):
        the handoff runs off the GUI thread, then the window closes."""
        if self.may_close:
            if self.settings_path is not None:
                self._save_settings()
            e.accept()
            return
        e.ignore()
        if self.closing:
            return
        if self.b.busy():
            self.log(f'an action is still running ({self.b.busy()}) - close again once it '
                     'has finished, so the arm is never left mid-switch')
            return
        self.closing = True
        self.log('closing: handing the arm back if impedance is active ...')
        self.b.command('close_handoff')

    def resizeEvent(self, e):
        super().resizeEvent(e)
        if self.overlay.isVisible():
            self._place_overlay()

    # ------------------------------------------------------------ persistence

    def _drawer_state(self):
        def num(e):
            try:
                return float(e.text())
            except ValueError:
                return e.text().strip()
        return {'standoff_mm': self.target.currentText(), 'pos_tol_mm': self.tol.currentText(),
                'inplane': self.inplane.currentText(), 'floor_mm': self.floor_edit.text().strip(),
                'box_x_mm': [num(self.box['x'][0]), num(self.box['x'][1])],
                'box_y_mm': [num(self.box['y'][0]), num(self.box['y'][1])],
                'box_z_max_mm': num(self.box['z']),
                'step_mm': self.step.currentText(), 'rot_deg': self.rot.currentText(),
                'track_entry_mm': self.entry_mm.value(), 'track_entry_deg': self.entry_deg.value(),
                'marker_loss': self.loss.currentText(), 'marker_loss_ms': self.loss_ms.value(),
                'pose_jump_mm': self.jump.value(), 'camera': self.cam_cb.isChecked(),
                **({'gains': dict(g)} if (g := self.b.session_gains()) else {})}

    def _restore_settings(self):
        """Put the drawer back as it was left. Every value goes through the
        widget, so its normal handler updates the backend; anything not
        valid for its widget is skipped and said so."""
        vals, problem = persist.load(self.settings_path)
        if problem:
            self.log(f'settings: {problem}')
        if not vals:
            return
        changed, skipped = [], []

        def combo(key, cb, label, unit):
            if key not in vals:
                return
            v = str(vals[key])
            if v not in [cb.itemText(i) for i in range(cb.count())]:
                skipped.append(key)
            elif v != cb.currentText():
                cb.setCurrentText(v)
                changed.append(f'{label} {v}{unit}')

        def spin(key, sp, label, unit):
            if key not in vals:
                return
            try:
                v = float(vals[key])
            except (TypeError, ValueError):
                skipped.append(key)
                return
            if abs(v - sp.value()) > 1e-9:
                sp.setValue(v)
                changed.append(f'{label} {sp.value():g}{unit}')

        combo('standoff_mm', self.target, 'standoff', ' mm')
        combo('pos_tol_mm', self.tol, 'pos tol', ' mm')
        combo('inplane', self.inplane, 'in-plane', '')
        combo('step_mm', self.step, 'translate step', ' mm')
        combo('rot_deg', self.rot, 'level step', ' deg')
        combo('marker_loss', self.loss, 'marker loss', '')
        spin('track_entry_mm', self.entry_mm, 'TRACK entry', ' mm')
        spin('track_entry_deg', self.entry_deg, 'TRACK entry tilt', ' deg')
        spin('marker_loss_ms', self.loss_ms, 'marker loss after', ' ms')
        spin('pose_jump_mm', self.jump, 'pose jump alarm', ' mm')
        if 'floor_mm' in vals:
            f = str(vals['floor_mm']).strip()
            try:
                if f:
                    float(f)
                if f != self.floor_edit.text().strip():
                    self.floor_edit.setText(f)
                    self._floor_changed()
                    changed.append(f'ALIGN Z floor {f or "NOT SET"} mm')
            except ValueError:
                skipped.append('floor_mm')
        try:
            box = [vals.get('box_x_mm'), vals.get('box_y_mm')]
            fields = [(self.box['x'], box[0]), (self.box['y'], box[1])]
            before = self._drawer_state()
            for (lo_e, hi_e), v in fields:
                if v is not None:
                    lo_e.setText(f'{float(v[0]):g}')
                    hi_e.setText(f'{float(v[1]):g}')
            if vals.get('box_z_max_mm') is not None:
                self.box['z'].setText(f'{float(vals["box_z_max_mm"]):g}')
            self._box_changed(quiet=True)
            after = self._drawer_state()
            if [before[k] for k in ('box_x_mm', 'box_y_mm', 'box_z_max_mm')] != \
                    [after[k] for k in ('box_x_mm', 'box_y_mm', 'box_z_max_mm')]:
                changed.append(f'workspace box x {after["box_x_mm"]} y {after["box_y_mm"]}'
                               f' z max {after["box_z_max_mm"]:g} mm')
        except (TypeError, ValueError, IndexError):
            skipped.append('workspace box')
        g = vals.get('gains')
        if isinstance(g, dict):
            try:
                g = {k: float(g[k]) for k in logic.GAIN_KEYS}
                if logic.gain_problem(g, C.GAIN_LIMITS):
                    raise ValueError
                self.b.set_session_gains(g)
                self._set_pending(g)
                changed.append('gains {k_xy:g}/{k_z:g}/{k_rp:g}/{k_yaw:g}/zeta {zeta:g} '
                               '(written at the next FLOAT/HOLD)'.format(**g))
            except (KeyError, TypeError, ValueError):
                skipped.append('gains')
        if 'camera' in vals and bool(vals['camera']) != self.cam_cb.isChecked():
            self.cam_cb.setChecked(bool(vals['camera']))
            changed.append('camera view ' + ('on' if vals['camera'] else 'OFF'))
        self.log(f'settings restored from {self.settings_path}: '
                 + (', '.join(changed) if changed else 'all at their defaults')
                 + (f'  (ignored: {", ".join(skipped)})' if skipped else ''))

    def _watch_drawer(self):
        kick = lambda *_: self._save_timer.start()           # noqa: E731
        for cb in (self.target, self.tol, self.inplane, self.step, self.rot, self.loss):
            cb.currentTextChanged.connect(kick)
        for sp in (self.entry_mm, self.entry_deg, self.loss_ms, self.jump):
            sp.valueChanged.connect(kick)
        for e in (self.floor_edit, self.box['x'][0], self.box['x'][1], self.box['y'][0],
                  self.box['y'][1], self.box['z']):
            e.editingFinished.connect(kick)
        self.cam_cb.toggled.connect(kick)

    def _save_settings(self):
        try:
            persist.save(self.settings_path, self._drawer_state())
        except OSError as e:
            self.log(f'settings NOT saved: {e}')

    # ------------------------------------------------------------ render

    def render(self):
        """GUI thread, 10 Hz: one snapshot, every widget from it."""
        s = self.b.snap()
        st = self.st
        now = self.b.now()
        self._last = s
        en = logic.enable(s, st)
        nxt = logic.next_step(s, st, en)
        m = logic.mode(s)
        trk = logic.tracking(s, st)

        for chip, c in zip(self.chips, logic.chips(s, st)):
            chip.set(c.value, c.level)
        rec = s.recording
        self.rec.set('off' if rec is None else f'{int(rec // 60):02d}:{int(rec % 60):02d}',
                     'normal' if rec is None else 'active')

        b = logic.banner(s, st)
        for w in (self.banner, self.banner_title, self.banner_detail):
            restyle(w, level=b.level)       # children are not re-polished with the frame
        set_text(self.banner_title, b.title)
        set_text(self.banner_detail, b.detail)
        if b.action == 'recover':
            self.banner_btn.name, self.banner_btn.base = 'recover', 'RECOVER'
            e = en['recover']
            self.banner_btn.show_state('pending' if 'recover' in self.pending else
                                       'next' if e.ok else 'blocked', e.why)
            self.banner_btn.setVisible(True)
        elif b.action == 'dismiss':
            self.banner_btn.name, self.banner_btn.base = 'dismiss', 'DISMISS'
            self.banner_btn.show_state('idle')
            self.banner_btn.setVisible(True)
        else:
            self.banner_btn.setVisible(False)

        ctl = logic.controller(s)
        mode_text = {'POSITION': 'POSITION   ·   arm controller',
                     'TORQUE': 'TORQUE   ·   impedance, ' + ('tracking' if trk else
                                                             'floating' if s.floating
                                                             else 'holding'),
                     'NONE': 'NO CONTROLLER ACTIVE', 'CONFLICT': 'CONTROLLER CONFLICT',
                     'UNKNOWN': 'CONTROLLER UNKNOWN'}[m]
        set_text(self.mode_lbl, mode_text)
        act = ('ALIGNING' if s.busy in ('auto_converge', 'translate', 'level', 'inplane')
               or (s.busy or '').startswith('goto:') else 'TRACKING' if trk else None)
        self.activity.setVisible(act is not None)
        if act:
            self.activity.set(act, 'active')
        meta, cstate = self.b.calib()
        res = (f'{meta["residual_mm"]:.2f} mm / {meta.get("residual_deg", 0):.2f} deg'
               if 'residual_mm' in meta else cstate)
        self.calib_chip.set(res, {'loaded': 'normal', 'waiting': 'warn'}.get(cstate, 'fault'))
        self._render_calib(meta, cstate)

        self._render_speed(s, en, m)
        self._render_tspeed(s, en, trk)
        self._render_buttons(s, en, nxt, now, trk)
        self.pos_sec.set_inactive(m != 'POSITION',
                                  'impedance holds the arm - RELEASE returns here'
                                  if m == 'TORQUE' else 'no position control right now')
        self.tq_sec.set_inactive(m != 'TORQUE',
                                 'arm controller holds the arm - PRE-FLIGHT, then HOLD, enters'
                                 if m == 'POSITION' else 'impedance controller not active')
        self._render_gains(s, en)
        self._render_telemetry(s, st, now, m)
        self._render_camera(s, st, now, ctl, trk)
        if self.confirm and (time.monotonic() > self.confirm[2]
                             or (self.confirm[0] == 'speed' and m != 'TORQUE')):
            self.confirm[3].setVisible(False)
            self.confirm = None
        note = ('slider value is written to the controller on each activation'
                if m != 'TORQUE' else '')
        set_text(self.torque_note, note)

    def _render_buttons(self, s, en, nxt, now, trk):
        if s.busy and s.busy in self.buttons and s.busy not in self.pending:
            self.pending[s.busy] = now        # in flight: never idle, even unannounced
        for name, btn in self.buttons.items():
            if name in self.pending:
                t0 = self.pending[name]
                label = 'STOPPING' if name == 'stop_now' else btn.base.splitlines()[0]
                btn.show_state('pending', text=f'{label}  …  {now - t0:.1f} s')
                continue
            key = name
            if name.startswith('teach:'):
                key = 'teach'
            e = en.get(key, logic.Enable(True, ''))
            text = None
            if name == 'track' and trk:
                text = 'END TRACK'
            elif name == 'pause' and s.user_paused:
                text = 'RESUME'
            elif name == 'translate':
                text = f'Translate   {self.step.currentText()} mm'
            elif name == 'level':
                text = f'Level   {self.rot.currentText()} deg'
            elif name == 'inplane':
                text = (f'In-plane  →  {self.inplane.currentText()} deg'
                        if self.inplane.currentText() != 'off' else 'In-plane  (off)')
            elif name == 'auto_converge':
                text = f'AUTO-CONVERGE   →   {self.target.currentText()} mm standoff'
            elif name == 'track_fast':
                fast = logic.speed_torque(self.st.track_fast_pct, self.st)
                text = (('▲ FAST ON  ' if s.track_fast else 'FAST  ')
                        + f'{fast["setpoint_slew_mps"]*1000:.0f} mm/s')
            state = 'blocked' if not e.ok else 'next' if name == nxt else 'idle'
            if name == 'track_fast' and e.ok and s.track_fast:
                state = 'warn'                # raised speed: amber, shape and word
            btn.show_state(state, e.why, text)

    def _render_speed(self, s, en, m):
        pct = self.speed.value()
        set_text(self.speed_val, f'{pct:3d} %')
        e = en['speed']
        self.speed.setEnabled(e.ok)
        self.speed.setToolTip('' if e.ok else f'Not available: {e.why}')
        if not e.ok and self.confirm and self.confirm[0] == 'speed':
            self.confirm[3].setVisible(False)
            self.confirm = None
        text = logic.speed_text(pct, 'TORQUE' if m == 'TORQUE' else 'POSITION', self.st)
        slew = self.b.applied_slew()
        if m == 'TORQUE' and logic.tracking(s, self.st):
            text = ('during TRACK the TRACK SPEED slider (TORQUE section) sets the limit'
                    + ('' if slew is None else f': {slew[0]*1000:.0f} mm/s, '
                       f'{math.degrees(slew[1]):.1f} deg/s'))
        elif m == 'TORQUE' and slew is not None:
            want = logic.speed_torque(pct, self.st)['setpoint_slew_mps']
            differs = abs(slew[0] - want) > 1e-6
            text += (f'   ·   controller {slew[0]*1000:.0f} mm/s'
                     + ('  (pending)' if differs else ''))
            if (differs and logic.speed_needs_confirm(pct, self.st) and self.confirm is None
                    and not self.speed_dragging and 'speed' not in self.pending):
                self._arm_confirm(self.speed_confirm, 'speed', (float(pct),),
                                  f'CONFIRM {pct} %  →  {want*1000:.0f} mm/s', hold=True)
        elif m != 'TORQUE':
            text += '   ·   applies to the next planned move'
        set_text(self.speed_txt, text)

    def _render_tspeed(self, s, en, trk):
        pct = self.tspeed.value()
        set_text(self.tspeed_val, f'{pct:3d} %')
        e = en['track_speed']
        self.tspeed.setEnabled(e.ok)
        self.tspeed.setToolTip('Slew limit while tracking; tracking_node\'s own profile is '
                               '100 mm/s. Written right after START and live during TRACK.'
                               if e.ok else f'Not available: {e.why}')
        t = logic.speed_torque(pct, self.st)
        text = (f'{t["setpoint_slew_mps"]*1000:.0f} mm/s   '
                f'{math.degrees(t["setpoint_slew_rps"]):.1f} deg/s')
        slew = self.b.applied_slew()
        if trk and slew is not None:
            if s.track_fast:
                t = logic.speed_torque(self.st.track_fast_pct, self.st)
            text += (f'   ·   in force {slew[0]*1000:.0f} mm/s'
                     + (' FAST' if s.track_fast else '')
                     + ('' if abs(slew[0] - t['setpoint_slew_mps']) < 1e-6 else '  (pending)'))
        elif not trk:
            text += '   ·   applied when TRACK starts'
        set_text(self.tspeed_txt, text)

    def _render_gains(self, s, en):
        applied = self.b.applied_gains()
        trk = logic.tracking(s, self.st)
        if self.pending_gains is None and applied is not None and not trk:
            self._set_pending(dict(applied))
        pend = self.pending_gains or {}
        set_text(self.gain_state, 'tracking profile in force - yours come back at END TRACK' if trk else 'applied' + ('  →  pending (amber)' if s.gains_pending
                                                         and applied else ''))
        for k, lbl in self.gain_lbls.items():
            a, p = (applied or {}).get(k), pend.get(k)
            if trk:
                p = a
            f = '.2f' if k == 'zeta' else '.0f'
            if a is None:
                txt = fmt(p, f) + ' ?'
            elif p is not None and abs(p - a) > 1e-9:
                txt = f'{a:{f}} → {p:{f}}'
            else:
                txt = f'{a:{f}}'
            set_text(lbl, txt)
            pending = a is not None and p is not None and abs(p - a) > 1e-9
            lbl.setStyleSheet(f'color: {T["amber"] if pending else T["ink"]};')
            restyle(self.gain_edits[k], pending='true' if pending else 'false')
        idx = logic.preset_index(pend, self.presets) if pend and not trk else None
        self.preset.set_index(idx)
        self.preset.custom_text = 'TRACK PROFILE' if trk else 'CUSTOM'
        e = en['preset']
        self.preset.set_blocked(not e.ok, e.why)

    def _render_telemetry(self, s, st, now, m):
        mk = s.marker
        vals = {'dist': (mk and mk['dist_mm'], '.1f'), 'lateral': (mk and mk['lat_mm'], '.2f'),
                'tilt': (mk and mk['tilt_deg'], '.2f'), 'in-plane': (mk and mk['ip_deg'], '+.1f')}
        for k, (v, f) in vals.items():
            set_text(self.tiles[k], fmt(v, f))
            restyle(self.tiles[k], level='off' if v is None else 'normal')
        force = logic.force_n(s)
        lead = logic.lead_mm(s, st)
        arm = {'TCP Z': (s.tcp and s.tcp[2] * 1000, '.1f', None),
               '|F| ext': (force, '.1f', st.push_limit_n),
               'Fz ext': (s.force and s.force[2], '+.1f', None),
               'lead': (lead, '.1f', st.lead_warn_mm)}
        for k, (v, f, warn) in arm.items():
            set_text(self.tiles[k], fmt(v, f))
            level = ('off' if v is None else
                     'warn' if warn is not None and abs(v) > warn else 'normal')
            restyle(self.tiles[k], level=level)
        self.force_bar.set(force, s.thresholds)
        e = mk['err_xyz_mm'] if mk else None
        tip = ('TCP  x {:.1f}  y {:.1f}  z {:.1f} mm'.format(*[v * 1000 for v in s.tcp])
               if s.tcp else 'TCP  —')
        tip += ('\nF ext  [{:+.1f} {:+.1f} {:+.1f}] N'.format(*s.force) if s.force else '')
        tip += ('\nALIGN error  x {:+.2f}  y {:+.2f}  z {:+.2f} mm   |e| {:.2f} mm'.format(
            *e, mk['err_mm']) if e else '')
        if self.tel_panel.toolTip() != tip:
            self.tel_panel.setToolTip(tip)
        jp = s.joint_pct
        set_text(self.joints_lbl, 'joints  —' if jp is None else
                 f'joints  worst J{jp[0]}  {jp[1]:4.0f} % of range')
        self.joints_lbl.setStyleSheet(
            f'color: {T["amber"] if jp and jp[1] >= st.joint_warn_pct else T["ink2"]};')
        if m == 'TORQUE':
            set_text(self.plot_title, 'FORCE + SPRING LEAD  ·  30 s')
            fh = self.b.fhist()
            refs = [(st.push_limit_n, f'push {st.push_limit_n:g}')]
            if s.thresholds:
                refs.append((s.thresholds['reflex_n'], f'reflex {s.thresholds["reflex_n"]:g}'))
            self.plots[0].title, self.plots[0].unit = '|F| ext', 'N'
            self.plots[0].set_data([(t, f) for t, f, _ in fh], refs, now)
            self.plots[1].title, self.plots[1].unit = 'spring lead', 'mm'
            self.plots[1].set_data([(t, ld) for t, _, ld in fh],
                                   [(st.lead_warn_mm, f'warn {st.lead_warn_mm:g}'),
                                    (C.MAX_LEAD_MM, f'cap {C.MAX_LEAD_MM:g}')], now)
            self.plots[2].setVisible(False)
        else:
            set_text(self.plot_title, 'CONVERGENCE  ·  30 s  ·  dashed = tol')
            h = self.b.hist()
            tol = float(self.tol.currentText())
            for pl, (title, unit, idx, ref) in zip(self.plots, (
                    ('|e|', 'mm', 1, tol), ('tilt', 'deg', 2, st.rot_tol_deg),
                    ('in-plane error', 'deg', 3, st.inplane_tol_deg))):
                pl.title, pl.unit = title, unit
                pl.set_data([(r[0], r[idx]) for r in h], [(ref, f'tol {ref:g}')], now)
            self.plots[2].setVisible(True)

    def _render_calib(self, meta, cstate):
        self.calib_lbls['source'].setText(f'{meta.get("method", "?")}, {meta.get("poses", "?")}'
                                          f' poses, {meta.get("calibrated", "?")}')
        self.calib_lbls['frames'].setText(f'{meta.get("parent_frame", "?")} → '
                                          f'{meta.get("child_frame", "?")}')
        if 'residual_mm' in meta:
            self.calib_lbls['residual'].setText(f'{meta["residual_mm"]:.2f} mm / '
                                                f'{meta.get("residual_deg", 0):.2f} deg')
        if 'validated_scatter_mm' in meta:
            self.calib_lbls['validated'].setText(f'{meta["validated_scatter_mm"]:.2f} mm '
                                                 'static-marker scatter')
        self.calib_lbls['status'].setText({'loaded': 'loaded, re-based to the current pose',
                                           'waiting': 'waiting for TF - is MoveIt up?',
                                           'missing': 'handeye.yaml not found',
                                           'error': 'could not read the file'}.get(cstate, cstate))
        for name, (taught, dist) in self.b.pose_info().items():
            if name in self.pose_lbls:
                self.pose_lbls[name].setText('not taught' if not taught else
                                             f'taught {taught}' + ('' if dist is None else
                                                                   f', {dist[0]:.0f} mm away'))

    def _render_camera(self, s, st, now, ctl, trk):
        age = s.image_age
        set_text(self.cam_age, 'off' if not s.camera_on else
                 'no frames' if age is None else f'frame {age*1000:.0f} ms')
        self._paint_image()
        ev = logic.vision_event(s, st)
        relevant = ctl == 'arm' or trk or logic.holding(s)
        if ev and relevant and s.camera_on and not self.overlay.isVisible():
            self._open_overlay(True, ev)
            self.log(f'camera view enlarged: {ev}')
        elif self.overlay.isVisible() and self.overlay_auto:
            if ev:
                self.overlay_ok_since = None
                set_text(self.ov_reason, 'VISION EVENT: ' + ev)
            elif self.overlay_ok_since is None:
                self.overlay_ok_since = now
            elif now - self.overlay_ok_since > AUTO_CLOSE_S:
                self._close_overlay()

    def _paint_image(self, force=False):
        got = self.b.image()
        if got is None:
            self.thumb.setText('no frames yet\nis cam_pub running?')
            return
        seq, img = got
        if seq == self._image_seq and not force:
            return
        self._image_seq = seq
        h, w = img.shape[:2]
        self._qimg = QImage(img.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        pix = QPixmap.fromImage(self._qimg)
        self.thumb.setPixmap(pix.scaled(self.thumb.size(), Qt.KeepAspectRatio,
                                        Qt.SmoothTransformation))
        if self.overlay.isVisible():
            self.ov_img.setPixmap(pix.scaled(self.ov_img.size(), Qt.KeepAspectRatio,
                                             Qt.SmoothTransformation))


class EscapeFilter(QObject):
    """Esc = STOP NOW from anywhere: an application-wide filter sees the key
    before any widget or open popup can keep it."""

    def __init__(self, window):
        super().__init__()
        self.w = window

    def eventFilter(self, obj, e):
        # Consumed here, so Qt does not propagate it to the parents (one stop
        # per press); a held key's auto-repeat adds nothing.
        if e.type() == QEvent.KeyPress and e.key() == Qt.Key_Escape:
            if not e.isAutoRepeat():
                self.w.on_press('stop_now')
            return True
        return False


def make_app(argv):
    app = QApplication.instance() or QApplication(argv)
    app.setStyle('Fusion')
    f = QFont('Lato')
    f.setPixelSize(M)
    app.setFont(f)
    return app
