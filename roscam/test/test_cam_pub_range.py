#!/usr/bin/env python3
"""cam_pub range_source: through the real process_frame, the published
distance is the depth plane's when ArUco's is off, and ArUco's when the
plane disagrees too much or there is no depth.

A marker_size_m set larger than the rendered marker makes ArUco read long
by the same factor: the measured error on the cell (plane_normal.
range_scale) is a few percent at 200-300 mm."""

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter

from roscam.cam_pub import ArucoPosePublisher
from test_plane_normal import CX, CY, FX, FY, synth_depth

SIZE = 0.021
K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
R_TRUE = cv2.Rodrigues(np.array([np.radians(8.0), 0.0, 0.0]))[0] @ np.diag([1.0, -1.0, -1.0])
T_TRUE = np.array([0.008, -0.004, 0.150])


@pytest.fixture(scope='module')
def ros():
    rclpy.init(domain_id=88)                        # never the cell's domain
    yield
    rclpy.shutdown()


def render(shape=(480, 640), side=240):
    """BGR image of DICT_6X6_250 id 0 (SIZE, pose R_TRUE/T_TRUE) on white."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    make = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    marker = make(d, 0, side)
    h = SIZE / 2.0
    obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    px = cv2.projectPoints(obj, cv2.Rodrigues(R_TRUE)[0], T_TRUE, K, None)[0].reshape(4, 2)
    H = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [side, 0], [side, side], [0, side]]), px.astype(np.float32))
    gray = cv2.warpPerspective(marker, H, (shape[1], shape[0]), flags=cv2.INTER_AREA,
                               borderValue=255)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def published_t(size_m, range_source, depth=True):
    """The /aruco/pose_raw position cam_pub publishes for one frame."""
    node = ArucoPosePublisher(parameter_overrides=[
        Parameter('source', value='external'), Parameter('marker_id', value=0),
        Parameter('marker_size_m', value=size_m), Parameter('range_source', value=range_source),
        Parameter('target_marker_id', value=-1), Parameter('publish_debug_image', value=False)])
    try:
        node.set_intrinsics(FX, FY, CX, CY)
        got = {}
        node.on_publish = lambda topic, header, t, q: got.setdefault(topic, np.array(t))
        n = R_TRUE[:, 2]
        node.process_frame(render(), node.self_stamped_header(),
                           depth_m=synth_depth(-n, float(-n @ T_TRUE)) if depth else None)
        assert '/aruco/pose_raw' in got, 'the marker was not detected'
        return got['/aruco/pose_raw']
    finally:
        node.destroy_node()


def test_the_distance_comes_from_the_depth_plane(ros):
    t = published_t(1.05 * SIZE, 'depth')
    assert np.linalg.norm(t - T_TRUE) < 0.5e-3, t


def test_aruco_mode_keeps_the_long_aruco_distance(ros):
    t = published_t(1.05 * SIZE, 'aruco')
    assert np.isclose(t[2], 1.05 * T_TRUE[2], atol=1e-3), t


def test_a_plane_too_far_off_is_not_trusted(ros):
    t = published_t(1.15 * SIZE, 'depth')             # 13% off > range_depth_max_rel 8%
    assert np.isclose(t[2], 1.15 * T_TRUE[2], atol=1.5e-3), t


def test_no_depth_keeps_the_aruco_distance(ros):
    t = published_t(1.05 * SIZE, 'depth', depth=False)
    assert np.isclose(t[2], 1.05 * T_TRUE[2], atol=1e-3), t


class _SlowTf:
    """A TF buffer whose transform arrives ready_s after the first lookup."""

    def __init__(self, ready_s):
        self.ready_s, self.t0 = ready_s, None

    def lookup_transform_core(self, target, source, stamp):
        import time
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import LookupException
        now = time.perf_counter()
        self.t0 = now if self.t0 is None else self.t0
        if self.ready_s is None or now - self.t0 < self.ready_s:
            raise LookupException('not yet')
        tf = TransformStamped()
        tf.transform.rotation.w = 1.0
        return tf


def _filter_frame_wait(ready_s):
    from types import SimpleNamespace
    from std_msgs.msg import Header
    ns = SimpleNamespace(tf_buffer=_SlowTf(ready_s), filter_frame='fr3_link0',
                         get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: None),
                         last_tf_wait_ms=None, last_tf=None)
    header = Header(frame_id='camera_color_optical_frame')
    out = ArucoPosePublisher._to_filter_frame(
        ns, (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])), header)
    return out, ns.last_tf_wait_ms


def test_the_tf_lookup_answers_within_a_few_ms_of_the_transform_arriving():
    """3A step 3 on 10-02: tf2_ros's own timeout sleeps in 20 ms steps, so a
    transform 3 ms late cost 20 ms. Polled finely, it costs about 3."""
    out, wait = _filter_frame_wait(0.003)
    assert out is not None and wait < 10.0
    out, wait = _filter_frame_wait(None)                    # never: refused after 50 ms
    assert out is None and 45.0 <= wait < 80.0
