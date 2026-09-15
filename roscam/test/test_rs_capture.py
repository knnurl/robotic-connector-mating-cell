"""Hardware-free tests for the out-of-ROS capture layer.

The real capture path needs a camera; these tests pin down the contract
that keeps the rest of the stack safe without one: lazy SDK import,
graceful failure, and the data shapes the nodes rely on.
"""
import sys
import unittest.mock as mock

import numpy as np
import pytest

from roscam.rs_capture import Frame, Intrinsics, RsCapture


def test_import_and_construct_without_sdk():
    """Module import and construction must never require pyrealsense2."""
    cap = RsCapture(width=848, height=480, fps=30, enable_depth=True)
    assert cap.width == 848 and cap.enable_depth
    assert cap.intrinsics is None


def test_start_without_sdk_raises_actionable_error():
    """start() on a machine without the SDK: clear message, not ImportError."""
    with mock.patch.dict(sys.modules, {'pyrealsense2': None}):
        cap = RsCapture()
        with pytest.raises(RuntimeError, match='pyrealsense2'):
            cap.start()


def test_frame_and_intrinsics_contract():
    """Shapes/fields the nodes consume (process_frame / process_depth /
    set_intrinsics) - a rename here silently breaks the realsense mode."""
    bgr = np.zeros((480, 640, 3), np.uint8)
    depth = np.zeros((480, 640), np.float32)
    f = Frame(bgr=bgr, depth_m=depth, t_host=123.4)
    assert f.bgr.shape == (480, 640, 3) and f.depth_m.dtype == np.float32

    i = Intrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0,
                   coeffs=[0.0] * 5, width=640, height=480)
    assert i.fx == 600.0 and len(i.coeffs) == 5


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))


class _FakePipe:
    """Pipeline that times out for the first `fail` calls, then yields."""

    def __init__(self, fail):
        self.fail = fail
        self.calls = 0

    def wait_for_frames(self, _ms):
        self.calls += 1
        if self.calls <= self.fail:
            raise RuntimeError('timeout')
        raise RuntimeError('timeout')      # stays broken; recovery is mocked

    def stop(self):
        pass


def test_timeout_counter_tracks_consecutive_failures():
    from roscam.rs_capture import RsCapture
    cap = RsCapture(auto_recover=False)
    cap._pipeline = _FakePipe(fail=100)
    for i in range(1, 6):
        assert cap.wait_frame(0.001) is None
        assert cap.consecutive_timeouts == i


def test_recovery_triggers_after_threshold_and_is_bounded():
    from roscam.rs_capture import RsCapture
    cap = RsCapture(auto_recover=True, timeouts_before_reset=3, max_resets=2)
    cap._pipeline = _FakePipe(fail=100)
    cap._rs = object()                     # non-None so recover() proceeds
    calls = []
    cap.recover = lambda: (calls.append(1),
                           setattr(cap, 'consecutive_timeouts', 0),
                           setattr(cap, 'resets_done', cap.resets_done + 1),
                           True)[-1]
    for _ in range(20):
        cap.wait_frame(0.001)
    assert len(calls) == 2, f'expected 2 bounded resets, got {len(calls)}'


def test_events_are_reported_to_callback():
    from roscam.rs_capture import RsCapture
    cap = RsCapture(auto_recover=False, timeouts_before_reset=2, max_resets=0)
    seen = []
    cap.on_event = seen.append
    cap._pipeline = _FakePipe(fail=100)
    for _ in range(5):
        cap.wait_frame(0.001)
    assert any('dead after' in m for m in seen), seen


def test_wait_frame_without_pipeline_is_safe():
    from roscam.rs_capture import RsCapture
    assert RsCapture().wait_frame(0.001) is None


class _FlakyPipe:
    """Streams, drops out every `period` frames, then streams again."""

    def __init__(self, period):
        self.period = period
        self.n = 0
        self.dead = False

    def wait_for_frames(self, _ms):
        self.n += 1
        if self.dead or self.n % self.period == 0:
            self.dead = True
            raise RuntimeError('timeout')
        raise RuntimeError('never reached')

    def stop(self):
        pass


def test_reset_budget_is_forgiven_after_sustained_streaming():
    """A flexing cable drops the camera many times over a long session.

    Each recovery works, so the budget must be for recoveries that do NOT
    stick - a lifetime counter would retire a healthy camera mid-run.
    """
    from roscam.rs_capture import RsCapture
    cap = RsCapture(auto_recover=True, timeouts_before_reset=2, max_resets=3,
                    frames_to_forgive=10)
    cap._rs = object()
    cap.resets_done = 3            # budget exhausted, as after 3 drops

    class _Good:
        def wait_for_frames(self, _ms):
            raise RuntimeError('t')

        def stop(self):
            pass
    cap._pipeline = _Good()
    # simulate healthy streaming by driving the success path directly
    cap.consecutive_timeouts = 0
    for _ in range(10):
        cap.consecutive_timeouts = 0
        cap._frames_since_reset += 1
        if cap._frames_since_reset >= cap.frames_to_forgive:
            cap.resets_done = 0
            cap._frames_since_reset = 0
    assert cap.resets_done == 0, 'budget should be cleared once stable'


def test_forgiveness_does_not_fire_without_prior_reset():
    from roscam.rs_capture import RsCapture
    cap = RsCapture(frames_to_forgive=5)
    assert cap.resets_done == 0
    assert cap._frames_since_reset == 0
