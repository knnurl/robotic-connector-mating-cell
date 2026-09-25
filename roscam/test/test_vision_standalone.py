#!/usr/bin/env python3
"""vision_standalone's loop: one bad frame never stops the next, and the
recorded frame is the camera's own, not the one the overlay was drawn on."""

import types

import numpy as np

from roscam.rs_capture import Frame
from roscam.vision_standalone import handle_frame, run_frames


def test_a_failing_frame_is_logged_and_the_next_one_still_comes():
    frames = iter([Frame(np.zeros((2, 2, 3), np.uint8), None, 0.0), None, 'boom',
                   Frame(np.ones((2, 2, 3), np.uint8), None, 1.0)])
    calls = iter([True, True, True, True, False])
    handled, logs = [], []

    def handle(f):
        if f == 'boom':
            raise ValueError('bad frame')
        handled.append(f)
    n = run_frames(lambda: next(frames), handle, logs.append, lambda: next(calls))
    assert n == 2 and len(handled) == 2
    assert any('timeout' in m for m in logs) and any('bad frame' in m for m in logs)


def test_the_recorded_frame_is_untouched_and_carries_its_published_poses():
    stamp = types.SimpleNamespace(sec=12, nanosec=500_000_000)

    class FakeAruco:
        on_publish = None

        def self_stamped_header(self):
            return types.SimpleNamespace(stamp=stamp)

        def process_frame(self, img, header, depth_m=None):
            img[:] = 255                                    # the debug overlay
            self.on_publish('/aruco/pose_raw', header, [0.0, 0.0, 0.1], [0, 0, 0, 1])

    class FakeRecorder:
        recording = True
        got = None

        def record(self, frame, stamp_s, poses):
            FakeRecorder.got = (frame, stamp_s, poses)

    bgr = np.zeros((4, 4, 3), np.uint8)
    frame = Frame(bgr, None, 0.0)
    handle_frame(FakeAruco(), frame, FakeRecorder())
    recorded, stamp_s, poses = FakeRecorder.got
    assert recorded.bgr is bgr and not recorded.bgr.any()       # never drawn on
    assert stamp_s == 12.5
    assert poses['/aruco/pose_raw'][0] == 12.5 and poses['/aruco/pose_raw'][1] == [0.0, 0.0, 0.1]
