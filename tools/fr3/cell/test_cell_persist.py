"""The settings drawer survives a restart; the speed sliders and the gate
deliberately do not."""

import dataclasses
import os
import pathlib
import types

import pytest
import yaml

import persist

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
HERE = pathlib.Path(__file__).resolve().parent


def test_round_trip_and_unknown_keys(tmp_path):
    p = tmp_path / 'fr3_cell' / 'settings.yaml'
    persist.save(p, {'standoff_mm': '150', 'track_entry_mm': 45.0, 'speed_pct': 90})
    vals, problem = persist.load(p)
    assert problem is None
    assert vals == {'standoff_mm': '150', 'track_entry_mm': 45.0}   # speed not kept
    assert p.read_text().startswith('# FR3 Cell Control')
    assert not list(p.parent.glob('*.tmp'))


def test_missing_or_broken_file_is_never_fatal(tmp_path):
    assert persist.load(tmp_path / 'nope.yaml') == ({}, None)
    bad = tmp_path / 'bad.yaml'
    bad.write_text('standoff_mm: [unclosed')
    vals, problem = persist.load(bad)
    assert vals == {} and 'could not read' in problem
    bad.write_text('- just\n- a list\n')
    assert persist.load(bad)[0] == {}


def test_mock_and_real_are_separate_files(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    assert persist.path(False) != persist.path(True)
    assert persist.path(True).name == 'settings_mock.yaml'
    assert str(persist.path(False)).startswith(str(tmp_path))


pytest.importorskip('PySide6.QtWidgets')
from PySide6.QtCore import QCoreApplication  # noqa: E402

import logic as L  # noqa: E402
import view  # noqa: E402
import visual  # noqa: E402

ST = L.load_settings(HERE / 'config' / 'settings.yaml')
PRESETS = yaml.safe_load((HERE / 'config' / 'gain_presets.yaml').read_text())['presets']


class Live(visual.SceneBackend):
    """A backend with a settings file, recording what the window sets."""

    def __init__(self, path):
        super().__init__(visual.scenes()['idle'])
        self.settings_path = path
        self.params, self.st, self.cam = {}, None, None

    def set_param(self, name, value):
        self.params[name] = value

    def set_settings(self, st):
        self.st = st

    def camera(self, on):
        self.cam = on


@pytest.fixture(scope='module')
def app():
    return view.make_app([])


def _window(backend):
    w = view.MainWindow(backend, ST, PRESETS)
    w.render_timer.stop()
    w.hb_timer.stop()
    QCoreApplication.processEvents()
    return w


def test_drawer_values_survive_a_restart(app, tmp_path):
    p = tmp_path / 'settings.yaml'
    w = _window(Live(p))
    w.target.setCurrentText('150')
    w.entry_mm.setValue(45)
    w.loss.setCurrentText('stop')
    w.box['z'].setText('700')
    w.cam_cb.setChecked(False)
    w._save_settings()                       # what the 0.4 s timer does
    w.may_close = True
    w.close()

    b = Live(p)
    w2 = _window(b)
    assert w2.target.currentText() == '150' and b.params['target_m'] == pytest.approx(0.15)
    assert w2.entry_mm.value() == 45 and b.st.track_entry_mm == 45
    assert w2.loss.currentText() == 'stop' and b.params['marker_loss'] == 'stop'
    assert b.st.box_z_max == pytest.approx(0.7)
    assert b.cam is False
    log = w2.log_box.toPlainText()
    assert 'settings restored' in log and 'TRACK entry 45 mm' in log
    # the speed sliders and the gate start at their launch defaults regardless
    assert w2.speed.value() == int(ST.speed_default_pct)
    assert w2.tspeed.value() == int(ST.track_speed_default_pct)
    assert w2.gate_cb.isChecked()
    w2.may_close = True
    w2.close()


def test_invalid_saved_values_are_skipped(app, tmp_path):
    p = tmp_path / 'settings.yaml'
    persist.save(p, {'standoff_mm': '123', 'track_entry_mm': 'lots'})
    w = _window(Live(p))
    assert w.target.currentText() == '100'                # not one of the choices
    assert 'ignored: standoff_mm, track_entry_mm' in w.log_box.toPlainText()
    w.may_close = True
    w.close()


def test_frozen_scenes_never_touch_the_operators_file(app, monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    persist.save(persist.path(False), {'standoff_mm': '300'})
    w = _window(visual.SceneBackend(visual.scenes()['idle']))
    assert w.settings_path is None and w.target.currentText() == '100'
    w.may_close = True
    w.close()
    assert dataclasses.asdict(ST) == dataclasses.asdict(L.load_settings(
        HERE / 'config' / 'settings.yaml'))


def test_applied_gains_are_restored_and_written_at_activation(app, tmp_path):
    """A relaunch reloads the yaml gains; the last APPLIED set comes back and
    goes out with float_mode, before the switch (zero spring force)."""
    import actions
    import core
    g = {'k_xy': 300.0, 'k_z': 1200.0, 'k_rp': 20.0, 'k_yaw': 30.0, 'zeta': 0.9}
    p = tmp_path / 'settings.yaml'
    persist.save(p, {'gains': g})
    b = Live(p)
    got = {}
    b.set_session_gains = lambda v: got.update(v)
    w = _window(b)
    assert got == g and 'gains 300/1200/20/30' in w.log_box.toPlainText()
    w.may_close = True
    w.close()

    calls = []
    node = types.SimpleNamespace(
        state=lambda: (None, None, None, core.MODE_MOVE, 1.0, 0.0),
        controllers=lambda: {core.ARM_CONTROLLER: 'active', core.IMPEDANCE_CONTROLLER: 'inactive'},
        set_params=lambda v: calls.append(('params', dict(v))) or (True, 'ok'),
        switch=lambda a, d: calls.append(('switch', a, d)) or (True, 'switched'),
        refresh_now=lambda: None)
    cell = types.SimpleNamespace(n=node, st=ST, preflight='done', session_gains=g,
                                 params=actions.Params(), setpoint=None, floating=None,
                                 say=lambda m: None, trace=lambda r: None,
                                 note_torque_attempt=lambda: None,
                                 write_speed=lambda pct: (True, 'ok'))
    cell.blocked = types.MethodType(actions.Cell.blocked, cell)
    ok, _ = actions.Cell._activate(cell, False)
    assert ok
    assert calls[0][0] == 'params' and calls[0][1]['k_pos_tool'] == [300.0, 300.0, 1200.0]
    assert calls[0][1]['damping_ratio'] == 0.9 and calls[0][1]['float_mode'] is False
    assert calls[1][0] == 'switch'                  # gains land BEFORE activation


def test_applied_gains_are_saved(app, tmp_path):
    """The other half: APPLY GAINS ('gains_applied') lands in the file."""
    g = {'k_xy': 800.0, 'k_z': 1200.0, 'k_rp': 50.0, 'k_yaw': 50.0, 'zeta': 0.9}
    p = tmp_path / 'settings.yaml'
    b = Live(p)
    b.session_gains = lambda: g
    w = _window(b)
    w._on_event(('gains_applied', g))
    assert w._save_timer.isActive()
    w._save_settings()                       # what the 0.4 s timer does
    assert persist.load(p)[0]['gains'] == g
    w.may_close = True
    w.close()


def test_the_grip_drawer_survives_a_restart(app, tmp_path):
    p = tmp_path / 'settings.yaml'
    w = _window(Live(p))
    w.grip_cube.setValue(48)
    w.grip_force.setValue(30)
    w._save_settings()
    w.may_close = True
    w.close()
    b = Live(p)
    w2 = _window(b)
    assert w2.grip_cube.value() == 48 and b.params['grip_cube_mm'] == 48.0
    assert w2.grip_force.value() == 30 and b.params['grip_force_n'] == 30.0
    w2.may_close = True
    w2.close()
