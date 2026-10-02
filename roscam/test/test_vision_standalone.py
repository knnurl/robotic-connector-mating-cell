#!/usr/bin/env python3
"""vision_standalone's loop: one bad frame never stops the next, the
recorded frame is the camera's own, not the one the overlay was drawn on;
in shadow mode the depth estimate joins that frame's quality status, and in
depth_checked it drives /object/* once acquired, the marker vetoing it."""

import pathlib
import time
import types

import numpy as np
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.parameter import Parameter

from roscam.cam_pub import ArucoPosePublisher, rotation_matrix_to_quaternion
from roscam.object_contract import ObjectContract
from roscam.rs_capture import Frame
from roscam.vision_standalone import DepthRunner, handle_frame, run_frames
from test_object_pose import CX, CY, FX, FY, colour_scene, pose

PART_FILE = pathlib.Path(__file__).resolve().parents[2] / 'tools' / 'fr3' / 'parts' / 'cube55.yaml'


@pytest.fixture(scope='module')
def ros():
    rclpy.init(domain_id=88)                        # never the cell's domain
    yield
    rclpy.shutdown()


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


def test_the_estimator_draws_on_the_copy_and_estimates_on_the_camera_frame():
    stamp = types.SimpleNamespace(sec=3, nanosec=0)
    order = []

    class FakeAruco:
        on_publish = None

        def self_stamped_header(self):
            return types.SimpleNamespace(stamp=stamp)

        def process_frame(self, img, header, depth_m=None):
            img[:] = 7                                      # the overlays, on its copy
            order.append(('process',))
            self.on_publish('/aruco/pose_raw', header, [0.0, 0.0, 0.1], [0, 0, 0, 1])

    class FakeDepth:
        def after(self, frame, header, poses, t_start):
            order.append(('after', frame.bgr.any(), sorted(poses), t_start <= time.perf_counter()))

    frame = Frame(np.zeros((4, 4, 3), np.uint8), None, 0.0)
    handle_frame(FakeAruco(), frame, None, FakeDepth())
    assert order == [('process',), ('after', False, ['/aruco/pose_raw'], True)]


def test_the_debug_overlay_goes_on_after_the_markers_were_detected(ros):
    """An outline drawn across a marker must not change its detection: the
    debug hooks run on the outgoing debug image only."""
    from test_cam_pub_range import SIZE, render
    node = ArucoPosePublisher(parameter_overrides=[
        Parameter('source', value='external'), Parameter('marker_id', value=0),
        Parameter('marker_size_m', value=SIZE), Parameter('target_marker_id', value=-1)])
    node.set_intrinsics(FX, FY, CX, CY)
    raws, images = [], []
    node.pose_raw_pub.publish = raws.append
    node.debug_pub.publish = images.append
    node.debug_pub.get_subscription_count = lambda: 1       # someone watches
    node.debug_hooks.append(lambda img: img.__setitem__(slice(None), 255))
    try:
        node.process_frame(render(), node.self_stamped_header())
    finally:
        node.destroy_node()
    assert len(raws) == 1                                   # detected on clean pixels
    assert len(images) == 1 and bytes(images[0].data) == b'\xff' * len(images[0].data)


T_CUBE = pose([0.004, -0.003, 0.150], yaw=10.0, tilt=2.0)


def depth_node(*params):
    """The node, contract and DepthRunner vision_standalone builds, with
    the publishers captured, on a rendered cube; one(poses) runs a frame
    after process_frame (whose frame hook one() plays) and returns its
    quality keys."""
    node = ArucoPosePublisher(parameter_overrides=[
        Parameter('source', value='external'), Parameter('target_marker_id', value=-1),
        Parameter('publish_debug_image', value=False),
        Parameter('object_part', value=str(PART_FILE))] + list(params))
    node.set_intrinsics(FX, FY, CX, CY)
    contract = ObjectContract(node, estimator=True)
    runner = DepthRunner(node, contract)
    runner.period_ms = 1e9                  # no frame budget here (test_object_shadow has it)
    got = {'quality': [], 'raw': [], 'pose': []}
    contract.quality_pub.publish = got['quality'].append
    contract.pubs['raw'].publish = got['raw'].append
    contract.pubs['pose'].publish = got['pose'].append
    depth, bgr = colour_scene(T_CUBE)
    frame = Frame(bgr, depth, 0.0)
    t0 = [100.0]

    def one(poses):
        t0[0] += 1.0 / 15.0
        h = node.self_stamped_header()
        h.stamp.sec, h.stamp.nanosec = int(t0[0]), int(round((t0[0] % 1) * 1e9))
        contract._on_frame(h, 2.0)                  # process_frame's frame hook
        runner.after(frame, h, poses, time.perf_counter())
        return {kv.key: kv.value for kv in got['quality'][-1].status[0].values}
    return node, contract, runner, got, one


def marker_at(T):
    return {'/aruco/pose_raw': (3.0, T[:3, 3], rotation_matrix_to_quaternion(T[:3, :3]))}


def test_the_shadow_estimate_joins_the_frames_quality(ros):
    node, contract, shadow, got, one = depth_node(Parameter('object_shadow', value=True))
    raw = marker_at(T_CUBE)
    got = got['quality']
    try:
        v = one(raw)
        assert v['source'] == 'marker' and v['seeded_from'] == 'marker'
        assert v['depth_valid'] == 'true' and float(v['agree_mm']) < 0.3, v['depth_reason']
        assert len(v['cand_pose'].split(',')) == 7
        assert one({})['depth_reason'] == 'no marker prior'
        shadow._grip_cb(DiagnosticStatus(values=[KeyValue(key='holding', value='true')]))
        assert one(raw)['depth_reason'] == 'HELD'
        shadow._grip_cb(DiagnosticStatus(values=[KeyValue(key='holding', value='false')]))
        shadow.depth.est.process = lambda *a, **k: 1 / 0       # the estimator fails
        v = one(raw)                                            # the status still goes out
        assert v['depth_valid'] == 'false' and 'ZeroDivisionError' in v['depth_reason']
        node.set_parameters([Parameter('object_shadow', value=False)])
        assert 'depth_valid' not in one(raw)                    # off: the marker's keys only
        assert len(got) == 5
    finally:
        node.destroy_node()


def test_depth_checked_drives_once_acquired_and_the_marker_vetoes_it(ros):
    node, contract, runner, got, one = depth_node()
    raw = marker_at(T_CUBE)
    try:
        assert contract.sources == ('marker', 'depth_checked')
        assert node.set_parameters([Parameter('object_source', value='depth_checked')])[0] \
            .successful
        q = [one(raw) for _ in range(7)]
        assert [v['check'] for v in q[:4]] == [f'acquiring {i}/5' for i in range(1, 5)]
        assert [v['valid'] for v in q] == ['false'] * 5 + ['true'] * 2   # the switch gap: 5
        assert all(v['source'] == 'depth_checked' for v in q)
        assert len(got['raw']) == 2 and len(got['pose']) == 2
        p = got['raw'][-1].pose.position
        assert np.linalg.norm(np.array([p.x, p.y, p.z]) - T_CUBE[:3, 3]) < 0.0003
        assert got['raw'][-1].header.stamp == got['quality'][-1].header.stamp
        off = T_CUBE.copy()
        off[0, 3] += 0.004                                      # the marker 4 mm away
        v = one(marker_at(off))
        assert v['valid'] == 'false' and len(got['raw']) == 2, v
        assert v['check'].startswith(('vetoed by the marker', 'depth invalid')), v['check']
        assert one({})['check'] == 'no marker prior'
        runner._grip_cb(DiagnosticStatus(values=[KeyValue(key='holding', value='true')]))
        assert one(raw)['check'] == 'HELD'
        assert len(got['raw']) == 2
    finally:
        node.destroy_node()


def test_depth_checked_puts_the_pose_in_the_filter_frame_with_the_markers_tf(ros):
    from roscam.depth_checked import pose7
    node, contract, runner, got, one = depth_node(
        Parameter('filter_frame', value='fr3_link0'),
        Parameter('object_source', value='depth_checked'))
    T_bc = np.eye(4)                                        # the camera in fr3_link0
    T_bc[:3, :3] = np.diag([1.0, -1.0, -1.0])
    T_bc[:3, 3] = [0.5, -0.1, 0.45]
    raw = marker_at(T_CUBE)
    try:
        for _ in range(6):
            node.last_tf = pose7(T_bc)                      # cam_pub's lookup, this frame
            v = one(raw)
        assert v['valid'] == 'true', v
        p = got['pose'][-1]
        assert p.header.frame_id == 'fr3_link0'
        t = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
        assert np.linalg.norm(t - (T_bc @ T_CUBE)[:3, 3]) < 0.0005
        node.last_tf = None                                 # no TF for this frame
        v = one(raw)
        assert v['valid'] == 'false' and v['check'] == 'no TF at the image stamp'
    finally:
        listener = node.tf_listener
        listener.executor.shutdown()
        listener.dedicated_listener_thread.join(timeout=2.0)
        node.destroy_node()


def test_shadow_mode_needs_a_part_file(ros):
    node = ArucoPosePublisher(parameter_overrides=[
        Parameter('source', value='external'), Parameter('publish_debug_image', value=False),
        Parameter('object_shadow', value=True)])
    try:
        shadow = DepthRunner(node, ObjectContract(node, estimator=True))
        assert not shadow.enabled                               # refused at start
        res = node.set_parameters([Parameter('object_shadow', value=True)])
        assert not res[0].successful and not shadow.enabled
    finally:
        node.destroy_node()


def test_exposure_and_gain_apply_live_to_the_camera_this_process_owns(ros):
    node = ArucoPosePublisher(parameter_overrides=[
        Parameter('source', value='external'), Parameter('publish_debug_image', value=False)])
    try:
        res = node.set_parameters([Parameter('capture_exposure_us', value=3000)])
        assert not res[0].successful                        # no camera in this process
        calls = []
        node.live_capture = types.SimpleNamespace(set_exposure=lambda v: calls.append(('e', v)),
                                                  set_gain=lambda v: calls.append(('g', v)))
        assert node.set_parameters([Parameter('capture_exposure_us', value=3000)])[0].successful
        assert node.set_parameters([Parameter('capture_gain', value=16)])[0].successful
        assert calls == [('e', 3000), ('g', 16)]
    finally:
        node.destroy_node()
