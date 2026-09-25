#!/usr/bin/env python3
"""frame_recorder: lossless round trip, drop counting, session metadata."""

import json
import threading
import time

import numpy as np
import pytest

from roscam.frame_recorder import FrameRecorder, load_session
from roscam.rs_capture import Frame


def frame(i, depth=True):
    rng = np.random.default_rng(i)
    bgr = rng.integers(0, 256, (48, 64, 3), dtype=np.uint8)
    raw = rng.integers(0, 65535, (48, 64), dtype=np.uint16) if depth else None
    return Frame(bgr, None if raw is None else raw * 1e-4, 1000.0 + i, 5e6 + i * 66.7,
                 'hardware_clock', raw, 1e-4)


def test_frames_and_poses_round_trip_exactly(tmp_path):
    rec = FrameRecorder()
    rec.start(tmp_path / 's1', {'capture': {'depth_scale': 1e-4, 'width': 64}})
    sent = [frame(i) for i in range(5)]
    for i, f in enumerate(sent):
        rec.record(f, 50.0 + i, {'/aruco/pose_raw': (50.0 + i, [0.1, 0.2, 0.3 + i],
                                                     [0.0, 0.0, 0.0, 1.0])})
    summary = rec.stop()
    assert summary['written'] == 5 and summary['dropped'] == 0 and summary['error'] is None
    meta, frames = load_session(tmp_path / 's1')
    assert meta['frames_written'] == 5 and meta['capture']['width'] == 64
    got = list(frames)
    assert [g.i for g in got] == list(range(5))
    for g, f in zip(got, sent):
        assert np.array_equal(g.bgr, f.bgr)
        assert np.array_equal(g.depth_raw, f.depth_raw) and g.depth_raw.dtype == np.uint16
        assert np.allclose(g.depth_m, f.depth_raw * 1e-4)
        assert g.line['t_hw'] == f.t_hw and g.line['t_domain'] == 'hardware_clock'
    assert got[3].line['poses']['/aruco/pose_raw']['t'] == [0.1, 0.2, 3.3]
    assert got[3].line['stamp'] == 53.0


def test_a_full_queue_drops_and_counts_instead_of_blocking(tmp_path, monkeypatch):
    """The camera loop must never wait on the disk: with the writer stalled,
    record() returns at once and every frame is written or counted dropped."""
    import roscam.frame_recorder as fr
    gate = threading.Event()
    real = fr.cv2.imwrite
    monkeypatch.setattr(fr.cv2, 'imwrite', lambda *a: gate.wait(5) and real(*a))
    rec = FrameRecorder(max_queue=2)
    rec.start(tmp_path / 's2', {})
    t0 = time.monotonic()
    for i in range(10):
        rec.record(frame(i, depth=False), float(i), {})
    assert time.monotonic() - t0 < 0.5
    assert rec.dropped >= 7                              # one in the writer, two queued
    gate.set()
    summary = rec.stop()
    assert summary['written'] + summary['dropped'] == 10 and summary['written'] >= 1


def test_frames_without_depth_are_recorded_colour_only(tmp_path):
    rec = FrameRecorder()
    rec.start(tmp_path / 's4', {})
    rec.record(frame(0, depth=False), 1.0, {})
    rec.stop()
    _, frames = load_session(tmp_path / 's4')
    (g,) = list(frames)
    assert g.depth_raw is None and g.line['depth'] is False
    assert not any((tmp_path / 's4' / 'depth').iterdir())


def test_a_session_directory_is_never_overwritten(tmp_path):
    rec = FrameRecorder()
    rec.start(tmp_path / 's5', {})
    rec.stop()
    with pytest.raises(FileExistsError):
        rec.start(tmp_path / 's5', {})
    assert json.loads((tmp_path / 's5' / 'session.json').read_text())['frames_written'] == 0


def test_a_frame_that_raced_stop_never_lands_in_the_next_session(tmp_path):
    rec = FrameRecorder()
    rec.start(tmp_path / 'a', {})
    old_q = rec._q
    rec.stop()
    old_q.put_nowait((99, frame(99, depth=False), 9.0, {}))   # the late frame
    rec.start(tmp_path / 'b', {})
    rec.record(frame(1, depth=False), 1.0, {})
    rec.stop()
    _, frames = load_session(tmp_path / 'b')
    assert [g.i for g in frames] == [0]
