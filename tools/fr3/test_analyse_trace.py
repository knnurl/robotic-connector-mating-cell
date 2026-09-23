#!/usr/bin/env python3
"""analyse_trace against tiny logs written the way the real writers write
them: cell_panel's trace (CellPanel.trace, one record per event) and
tracking_node's per-tick log (log_record, null where nothing was computed).

    python3 -m pytest tools/fr3/test_analyse_trace.py -q
"""

import json
import os

import numpy as np

import analyse_trace as at

T0 = 1790103751.0
HOLD_REASON = ('the equilibrium would sit {} mm from the arm (cap 10.0 mm) - '
               'wait for the arm to catch up')
HOLD_KIND = HOLD_REASON.format('#').replace('10.0', '#')


def _write(path, recs, tail=''):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r) + '\n' for r in recs) + tail)
    return path


def _cell(path, tail=''):
    return _write(path, [
        {'rec': 'session_start', 't': T0, 'tab': 'ALIGN', 'target_mm': 100.0,
         'R_cam_base': np.eye(3).tolist()},
        {'rec': 'iter', 't': T0 + 1, 'it': 1, 'err_mm': 50.0,
         'tilt_deg': 4.0, 'inplane_deg': 80.0, 'inplane_err_deg': 10.0},
        {'rec': 'translate', 't': T0 + 1.5, 'ok': True,
         'cmd_d_base_mm': [3.0, 4.0, 0.0], 'achieved_trans_mm': 4.5},
        {'rec': 'level', 't': T0 + 2, 'ok': False, 'clamped_cmd_deg': 2.0,
         'achieved_rot_deg': 0.1},
        {'rec': 'iter', 't': T0 + 3, 'it': 2, 'err_mm': 1.5,
         'tilt_deg': 0.5, 'inplane_deg': None, 'inplane_err_deg': 0.0},
        {'rec': 'sample', 't': T0 + 4, 'force': [3.0, 4.0, 0.0],
         'success_rate': 0.97, 'mode': 2},
        {'rec': 'sample', 't': T0 + 4.02, 'force': [0.0, 0.0, 1.0],
         'success_rate': 1.0, 'mode': 2},
        {'rec': 'track_start', 't': T0 + 5, 'ok': True, 'msg': 'tracking'},
        {'rec': 'track_status', 't': T0 + 5, 'state': 'tracking',
         'reason': '', 'policy': 'hold', 'level': 0},
        {'rec': 'track_status', 't': T0 + 7, 'state': 'holding',
         'reason': HOLD_REASON.format('12.3'), 'policy': 'hold', 'level': 1},
        {'rec': 'track_status', 't': T0 + 8, 'state': 'holding',
         'reason': HOLD_REASON.format('14.1'), 'policy': 'hold', 'level': 1},
        {'rec': 'track_policy', 't': T0 + 8.5, 'policy': 'clamp', 'ok': True},
        {'rec': 'track_status', 't': T0 + 9, 'state': 'tracking',
         'reason': '', 'policy': 'clamp', 'level': 0},
        {'rec': 'track_stop', 't': T0 + 10, 'ok': True, 'msg': 'stopped'},
        {'rec': 'track_status', 't': T0 + 10, 'state': 'idle', 'reason': '',
         'policy': 'clamp', 'level': 0},
        {'rec': 'session_end', 't': T0 + 11, 'outcome': 'converged'},
    ], tail)


def _tick(t, published, pos, policy='hold', reason=''):
    return {'t': float(t), 'stamp': 100.0 + t, 'goal': None, 'meas': None,
            'pos_err_mm': pos,
            'rot_err_deg': None if pos is None else pos / 10,
            'lead_mm': 0.5 * t, 'lead_deg': 0.1, 'published': published,
            'policy': policy, 'reason': reason}


def _tracking(path):
    clamp = 'clamped: ' + HOLD_REASON.format('11.0').split(' - ')[0]
    return _write(path, [_tick(t, True, t + 1.0) for t in range(5)] + [
        _tick(5, False, 6.0, reason=HOLD_REASON.format('12.0')),
        _tick(6, False, 7.0, reason=HOLD_REASON.format('13.5')),
        _tick(7, True, 8.0, 'clamp', clamp),
        _tick(8, False, None, 'clamp', 'marker stale'),     # null geometry
        _tick(9, True, 10.0, 'clamp')])


def _text(path):
    return '\n'.join(at.summarise(path))


def test_cell_trace_summary(tmp_path):
    out = _text(_cell(tmp_path / 'cell_20260923_101500.jsonl'))
    assert 'outcome    converged' in out
    assert 'target_mm 100.0' in out and 'R_cam_base' not in out
    assert 'samples    2, control success min 97.0%, |F ext| max 5.0 N' in out
    assert '-- ALIGN: 2 iterations --' in out
    assert ('|e|        first   50.00  last    1.50  min    1.50 mm'
            in out)
    assert ('translate  1 steps, 0 failed; mean commanded 5.00 mm, '
            'achieved 4.50 mm') in out
    assert 'level      1 steps, 1 failed' in out
    assert 'START      1 (1 ok), STOP 1' in out
    assert 'tracking 3.0 s, holding 2.0 s, idle 1.0 s' in out
    assert f'     2  {HOLD_KIND}' in out
    assert 'policy     clamp' in out


def test_a_trace_the_panel_never_closed_says_so(tmp_path):
    """A stack death leaves no session_end, and can tear the last line."""
    path = tmp_path / 'cell_20260923_101500.jsonl'
    _write(path, [{'rec': 'session_start', 't': T0, 'tab': 'IMPEDANCE'},
                  {'rec': 'activate', 't': T0 + 1, 'float_mode': True}],
           tail='{"rec": "sample", "pos": [0.38')
    out = _text(path)
    assert '(cell_panel, 2 records)' in out
    assert 'no session_end' in out
    assert '-- ALIGN' not in out and '-- TRACK' not in out


def test_tracking_log_summary(tmp_path):
    out = _text(_tracking(tmp_path / 'tracking_20260923_091500.jsonl'))
    assert '(tracking_node, 10 ticks)' in out
    assert 'duration   9.0 s' in out
    assert 'tracking   6.3 s (70%), 1 ticks clamped' in out
    assert 'holding    2.7 s (30%)' in out
    assert f'     2  {HOLD_KIND}' in out
    assert '     1  marker stale' in out
    pos = [p for p in range(1, 11) if p != 9]     # tick 8 computed nothing
    p50, p95 = np.percentile(pos, [50, 95])
    assert f'pos error  p50 {p50:.2f}  p95 {p95:.2f}  max 10.00 mm' in out
    assert 'lead       p50' in out and 'max 4.50 mm' in out
    assert 'policy     hold, 7.0 s -> clamp' in out


def test_newest_log_prefers_fr3_log_dir_and_its_day_folders(
        tmp_path, monkeypatch):
    runs, legacy = tmp_path / 'runs', tmp_path / 'logs'
    monkeypatch.setattr(at, 'LOG_DIR', legacy)
    old = _cell(runs / '2026-09-22' / 'cell_20260922_100000.jsonl')
    new = _tracking(runs / '2026-09-23' / 'tracking_20260923_090000.jsonl')
    other = _cell(runs / '2026-09-23' / 'notes.jsonl')     # not a log
    here = _cell(legacy / 'cell_20260923_120000.jsonl')
    for path, mtime in ((old, 1000), (new, 2000), (other, 3000),
                        (here, 4000)):
        os.utime(path, (mtime, mtime))
    monkeypatch.setenv('FR3_LOG_DIR', str(runs))
    assert at.newest_log() == new
    monkeypatch.setenv('FR3_LOG_DIR', str(tmp_path / 'empty'))
    assert at.newest_log() == here
    monkeypatch.delenv('FR3_LOG_DIR')
    assert at.newest_log() == here


def test_no_argument_summarises_the_newest_or_says_there_is_none(
        tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(at, 'LOG_DIR', tmp_path / 'logs')
    monkeypatch.setenv('FR3_LOG_DIR', str(tmp_path / 'runs'))
    assert at.main([]) == 1
    assert 'no cell_ or tracking_ logs' in capsys.readouterr().out
    _tracking(tmp_path / 'runs' / '2026-09-23' / 'tracking_x.jsonl')
    assert at.main([]) == 0
    assert 'tracking_x.jsonl  (tracking_node, 10 ticks)' in \
        capsys.readouterr().out
