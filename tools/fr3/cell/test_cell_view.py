"""The window's behaviour, offscreen, on visual.py's frozen scenes: Esc
from anywhere, blocked presses that explain themselves, pending state,
exactly one blue step, confirm clicks. No ROS, robot or display."""

import dataclasses
import os
import pathlib

import pytest
import yaml

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('PySide6.QtWidgets')
from PySide6.QtCore import QCoreApplication, Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402

import logic as L  # noqa: E402
import view  # noqa: E402
import visual  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
ST = L.load_settings(HERE / 'config' / 'settings.yaml')
PRESETS = yaml.safe_load((HERE / 'config' / 'gain_presets.yaml').read_text())['presets']


class Recorder(visual.SceneBackend):
    def __init__(self, sc):
        super().__init__(sc)
        self.cmds = []

    def command(self, name, *a):
        self.cmds.append((name, *a))

    def names(self):
        return [c[0] for c in self.cmds]


@pytest.fixture(scope='module')
def app():
    return view.make_app([])


@pytest.fixture
def make(app):
    made = []

    def build(scene='idle', **snap):
        sc = dict(visual.scenes()[scene])
        if snap:
            sc['snap'] = dataclasses.replace(sc['snap'], **snap)
        b = Recorder(sc)
        w = view.MainWindow(b, ST, PRESETS)
        w.render_timer.stop()
        w.hb_timer.stop()
        w.show()
        for _ in range(2):
            QCoreApplication.processEvents()
            w.render()
        made.append(w)
        return w, b
    yield build
    for w in made:
        w.may_close = True
        w.close()


def test_escape_is_stop_now_whatever_has_focus(app, make):
    w, b = make()
    esc = view.EscapeFilter(w)
    app.installEventFilter(esc)
    try:
        w._toggle_drawer()
        w.floor_edit.setFocus()
        QTest.keyClick(w.floor_edit, Qt.Key_Escape)
        assert b.names().count('stop_now') == 1       # once, however it propagates
        w.target.showPopup()                          # an open dropdown keeps no key
        QTest.keyClick(w.target.view(), Qt.Key_Escape)
        assert b.names().count('stop_now') == 2
    finally:
        app.removeEventFilter(esc)


def test_blocked_press_explains_and_runs_nothing(make):
    w, b = make('idle')                               # PRE-FLIGHT not done
    assert w.buttons['hold'].state == 'blocked'
    assert 'PRE-FLIGHT' in w.buttons['hold'].toolTip()
    w.buttons['hold'].click()
    assert 'hold' not in b.names() and 'torque_attempt' in b.names()
    assert 'not available' in w.log_box.toPlainText()


def test_next_step_press_runs(make):
    w, b = make('idle')
    assert w.buttons['preflight'].state == 'next'
    w.buttons['preflight'].click()
    assert b.names() == ['preflight']


def test_pending_from_press_until_done(make):
    w, _b = make('idle')
    w.post('pending', 'preflight')
    QCoreApplication.processEvents()
    w.render()
    btn = w.buttons['preflight']
    assert btn.state == 'pending' and '…' in btn.text()
    w.post('done', 'preflight', False, 'the robot never reported IDLE')
    QCoreApplication.processEvents()
    w.render()
    assert btn.state != 'pending'
    assert 'preflight FAILED' in w.log_box.toPlainText()


def test_stops_stay_live_while_busy(make):
    w, _b = make('aligning')
    assert w.buttons['auto_converge'].state == 'pending'
    for name in ('stop_now', 'pause', 'stop_after'):
        assert w.buttons[name].state != 'blocked', name
    assert w.buttons['translate'].state == 'blocked'


@pytest.mark.parametrize('scene', list(visual.scenes()))
def test_at_most_one_blue_step(make, scene):
    w, _b = make(scene)
    blue = [n for n, btn in w.buttons.items() if btn.state == 'next']
    if w.banner_btn.isVisible() and w.banner_btn.state == 'next':
        blue.append('banner:' + w.banner_btn.name)
    assert len(blue) <= 1, blue


def test_stop_now_is_the_only_red_control(make):
    w, _b = make('tracking')
    red = [n for n, btn in w.buttons.items() if btn.objectName() == 'stopNow']
    assert red == ['stop_now']
    assert 'NOT an emergency stop' in w.buttons['stop_now'].toolTip()
    assert w.buttons['release'].objectName() != 'stopNow'


def test_torque_speed_above_threshold_needs_a_confirm(make):
    w, b = make('tracking', tracking=False, track={'state': 'idle', 'policy': 'hold'})
    w.speed.setValue(60)
    assert 'speed' not in b.names()
    assert w.speed_confirm.isVisible() and '60' in w.speed_confirm.text()
    w.speed_confirm.click()
    assert ('speed', 60.0) in b.cmds


def test_speed_locked_while_tracking(make):
    w, _b = make('tracking')
    assert not w.speed.isEnabled()
    assert 'TRACK SPEED' in w.speed.toolTip()


def test_track_speed_is_live_while_tracking(make):
    w, b = make('tracking')
    assert w.tspeed.isEnabled()
    w.tspeed.setValue(8)
    assert ('track_speed', 8.0) in b.cmds
    w.tspeed.setValue(40)                             # above 25 %: confirm first
    assert ('track_speed', 40.0) not in b.cmds and w.tspeed_confirm.isVisible()
    w.tspeed_confirm.click()
    assert ('track_speed', 40.0) in b.cmds


def test_presets_refused_under_load(make):
    w, b = make('tracking', tracking=False, track={'state': 'idle'}, force=(0.0, 0.0, 12.0))
    assert w.preset.blocked and '|F| ext' in w.preset.why
    QTest.mouseClick(w.preset, Qt.LeftButton)
    assert 'not available' in w.log_box.toPlainText()


def test_inactive_section_says_why(make):
    w, _b = make('idle')
    assert w.tq_sec.property('inactive') == 'true'
    assert 'PRE-FLIGHT' in w.tq_sec.toolTip()
    assert w.pos_sec.property('inactive') == 'false'


def test_a_far_saved_pose_needs_a_confirm(make):
    w, b = make('idle')                                # home is 312 mm away
    w.buttons['goto:home'].click()
    assert 'goto:home' not in b.names() and w.pose_confirm.isVisible()
    w.pose_confirm.click()
    assert 'goto:home' in b.names()


def test_failure_surfaces_in_the_banner(make):
    w, _b = make('idle', failure=('translate', 'BLOCKED by the workspace box'))
    assert 'TRANSLATE FAILED' in w.banner_title.text()
    assert w.banner_btn.isVisible() and w.banner_btn.name == 'dismiss'


def test_fast_switch_toggles_and_shows_its_speed(make):
    w, b = make('tracking')
    fast = w.buttons['track_fast']
    assert fast.text() == 'FAST  100 mm/s' and fast.state == 'idle'
    fast.click()
    assert ('track_fast', True) in b.cmds          # the labelled click is the confirm
    w, b = make('tracking', track_fast=True)
    fast = w.buttons['track_fast']
    assert fast.text().startswith('▲ FAST ON') and fast.state == 'warn'
    fast.click()
    assert ('track_fast', False) in b.cmds


def test_fast_blocked_before_track_says_why(make):
    w, b = make('tracking', tracking=False, track={'state': 'idle'})
    w.buttons['track_fast'].click()
    assert not any(c[0] == 'track_fast' for c in b.cmds)
    assert 'only while tracking' in w.log_box.toPlainText()


def test_tracking_on_the_operators_gains_says_so(make):
    track = {'state': 'tracking', 'reason': '', 'policy': 'hold', 'gains': 'operator'}
    w, b = make('tracking', track=track)
    assert w.gain_state.text().startswith('your gains in force')
    assert w.preset.custom_text == 'CUSTOM'
    w, b = make('tracking')                                # an older node: its profile
    assert w.gain_state.text().startswith('tracking profile in force')
