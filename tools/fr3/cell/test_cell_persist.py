"""The settings drawer survives a restart; the speed sliders and the gate
deliberately do not."""

import dataclasses
import os
import pathlib

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
