#!/usr/bin/env python3
"""The object pose contract, source 'marker': through the real cam_pub, every
/object/* message is the /aruco/* one unchanged (or composed with a measured
T_marker_object), and one quality status goes out per processed frame - held,
in shadow mode, until the depth estimate's keys join it."""

import pathlib

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter

from roscam.cam_pub import ArucoPosePublisher
from roscam.object_contract import (POSE_TOPIC, QUALITY_TOPIC, RAW_POSE_TOPIC,
                                    ObjectContract)
from roscam.pose_kf import compose_pose, quat_from_rotvec
from test_cam_pub_range import R_TRUE, SIZE, T_TRUE, render
from test_plane_normal import CX, CY, FX, FY, synth_depth


@pytest.fixture(scope='module')
def ros():
    rclpy.init(domain_id=88)                        # never the cell's domain
    yield
    rclpy.shutdown()


def capture(pub, into):
    pub.publish = lambda msg: into.append(msg)


def node_with_contract(**params):
    node = ArucoPosePublisher(parameter_overrides=[
        Parameter('source', value='external'), Parameter('marker_id', value=0),
        Parameter('marker_size_m', value=SIZE), Parameter('target_marker_id', value=-1),
        Parameter('publish_debug_image', value=False)]
        + [Parameter(k, value=v) for k, v in params.items()])
    node.set_intrinsics(FX, FY, CX, CY)
    contract = ObjectContract(node)
    got = {k: [] for k in ('aruco_raw', 'aruco', 'raw', 'pose', 'quality')}
    capture(node.pose_raw_pub, got['aruco_raw'])
    capture(node.pose_pub, got['aruco'])
    capture(contract.pubs['raw'], got['raw'])
    capture(contract.pubs['pose'], got['pose'])
    capture(contract.quality_pub, got['quality'])
    return node, contract, got


def test_the_marker_source_is_mirrored_unchanged(ros):
    node, contract, got = node_with_contract()
    try:
        n = R_TRUE[:, 2]
        depth = synth_depth(-n, float(-n @ T_TRUE))
        img = render()
        for _ in range(3):
            node.process_frame(img.copy(), node.self_stamped_header(), depth_m=depth)
        node.process_frame(np.full_like(img, 255), node.self_stamped_header(), depth_m=depth)
    finally:
        node.destroy_node()
    assert len(got['aruco_raw']) == 3 and got['raw'] == got['aruco_raw']
    assert len(got['aruco']) >= 3 and got['pose'] == got['aruco']   # the 4th: a prediction
    q = got['quality']
    assert len(q) == 4                                              # one per processed frame
    vals = [{kv.key: kv.value for kv in a.status[0].values} for a in q]
    assert [v['valid'] for v in vals] == ['true', 'true', 'true', 'false']
    assert all(v['source'] == 'marker' for v in vals)
    assert q[3].status[0].level == q[3].status[0].WARN
    assert q[0].header.stamp == got['aruco_raw'][0].header.stamp


def test_the_topics_are_the_contracts(ros):
    node, contract, _ = node_with_contract()
    try:
        assert contract.pubs['raw'].topic_name == RAW_POSE_TOPIC == '/object/pose_raw'
        assert contract.pubs['pose'].topic_name == POSE_TOPIC == '/object/pose'
        assert contract.quality_pub.topic_name == QUALITY_TOPIC == '/object/pose_quality'
    finally:
        node.destroy_node()


def test_an_unknown_source_is_refused(ros):
    node, contract, _ = node_with_contract()
    try:
        res = node.set_parameters([Parameter('object_source', value='depth')])
        assert not res[0].successful and contract.source == 'marker'
    finally:
        node.destroy_node()


PART_FILE = pathlib.Path(__file__).resolve().parents[2] / 'tools' / 'fr3' / 'parts' / 'cube55.yaml'


def test_depth_checked_is_offered_only_where_the_estimator_runs(ros):
    for estimator, part, offered in ((False, True, False), (True, False, False),
                                     (True, True, True)):
        node = ArucoPosePublisher(parameter_overrides=[
            Parameter('source', value='external'), Parameter('publish_debug_image', value=False)]
            + ([Parameter('object_part', value=str(PART_FILE))] if part else []))
        try:
            contract = ObjectContract(node, estimator=estimator)
            res = node.set_parameters([Parameter('object_source', value='depth_checked')])
            assert res[0].successful == offered
            assert contract.source == ('depth_checked' if offered else 'marker')
        finally:
            node.destroy_node()


def test_a_source_switch_leaves_five_frames_without_a_pose(ros):
    node, contract, got = node_with_contract(object_part=str(PART_FILE))
    contract.sources = ('marker', 'depth_checked')      # as with the estimator
    try:
        _frames(node, 2)
        for src in ('depth_checked', 'marker'):          # there and straight back
            assert node.set_parameters([Parameter('object_source', value=src)])[0].successful
        _frames(node, 7)
    finally:
        node.destroy_node()
    assert contract.switches == 2
    assert len(got['aruco_raw']) == 9 and len(got['raw']) == 4   # frames 3-7 dropped
    vals = [{kv.key: kv.value for kv in a.status[0].values} for a in got['quality']]
    assert [v['valid'] for v in vals] == ['true'] * 2 + ['false'] * 5 + ['true'] * 2
    assert got['quality'][2].status[0].message == 'source switch: 5 frame(s) without a pose'


def _tq(msg):
    p, o = msg.pose.position, msg.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w])


_CLOCK = [1000.0]


def _frames(node, n=3, hz=15.0):
    """n rendered marker frames through the real process_frame, stamped 1/hz
    apart (a switch gap is judged on image stamps)."""
    n_ = R_TRUE[:, 2]
    depth = synth_depth(-n_, float(-n_ @ T_TRUE))
    for _ in range(n):
        _CLOCK[0] += 1.0 / hz
        h = node.self_stamped_header()
        h.stamp.sec, h.stamp.nanosec = int(_CLOCK[0]), int(round((_CLOCK[0] % 1) * 1e9))
        node.process_frame(render(), h, depth_m=depth)


def _stamps(msgs):
    return [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs]


def test_a_switch_inside_a_frame_still_leaves_a_third_of_a_second(ros):
    """The switch lands right after a raw went out, mid-frame."""
    node, contract, got = node_with_contract(object_part=str(PART_FILE))
    contract.sources = ('marker', 'depth_checked')
    publish = contract.pubs['raw'].publish

    def publish_then_switch(msg):
        publish(msg)
        if len(got['raw']) == 3:
            for src in ('depth_checked', 'marker'):
                assert node.set_parameters([Parameter('object_source', value=src)])[0] \
                    .successful
    contract.pubs['raw'].publish = publish_then_switch
    try:
        _frames(node, 12)
    finally:
        node.destroy_node()
    t = _stamps(got['raw'])
    assert len(t) > 3 and t[3] - t[2] >= 0.34


def test_at_a_high_frame_rate_the_switch_gap_is_still_a_third_of_a_second(ros):
    node, contract, got = node_with_contract(object_part=str(PART_FILE))
    contract.sources = ('marker', 'depth_checked')
    try:
        _frames(node, 2, hz=90.0)
        for src in ('depth_checked', 'marker'):
            assert node.set_parameters([Parameter('object_source', value=src)])[0].successful
        _frames(node, 40, hz=90.0)
    finally:
        node.destroy_node()
    t = _stamps(got['raw'])
    assert len(t) > 2 and t[2] - t[1] >= 0.34


def test_a_measured_T_marker_object_moves_the_pose_to_the_part(ros, tmp_path):
    part = tmp_path / 'part.yaml'
    part.write_text('name: t\nmesh: {box_mm: [55, 55, 55]}\nsymmetry: {order: 4}\n'
                    'T_marker_object: {xyz_mm: [3.0, -2.0, 0.0], rpy_deg: [0.0, 0.0, 5.0]}\n')
    node, contract, got = node_with_contract(object_part=str(part))
    try:
        _frames(node)
    finally:
        node.destroy_node()
    mo = (np.array([0.003, -0.002, 0.0]), quat_from_rotvec([0.0, 0.0, np.radians(5.0)]))
    assert len(got['raw']) == len(got['aruco_raw']) == 3
    for kind, ref in (('raw', 'aruco_raw'), ('pose', 'aruco')):
        for o, a in zip(got[kind], got[ref]):
            t, q = compose_pose(*_tq(a), *mo)
            to, qo = _tq(o)
            assert np.allclose(to, t, atol=1e-12) and np.allclose(qo, q, atol=1e-12)
            assert o.header == a.header


def test_the_held_status_waits_for_the_shadow_keys(ros):
    node, contract, got = node_with_contract()
    contract.hold_quality = True
    try:
        _frames(node, 1)
        assert got['quality'] == []                         # held for the shadow
        contract.publish_quality({'agree_mm': '0.400'})
        contract.publish_quality({'agree_mm': '9'})         # nothing waiting: nothing
    finally:
        node.destroy_node()
    vals = [{kv.key: kv.value for kv in a.status[0].values} for a in got['quality']]
    assert len(vals) == 1 and vals[0]['agree_mm'] == '0.400' and vals[0]['valid'] == 'true'
    assert got['quality'][0].header.stamp == got['aruco_raw'][0].header.stamp


def test_the_tf_wait_goes_into_the_quality(ros):
    node, contract, got = node_with_contract()
    try:
        node.last_tf_wait_ms = 3.24                 # cam_pub's lookup, this frame
        contract._on_frame(node.self_stamped_header(), 1.0)
        _frames(node, 1)                            # filter_frame '': no lookup
    finally:
        node.destroy_node()
    vals = [{kv.key: kv.value for kv in a.status[0].values} for a in got['quality']]
    assert vals[0]['tf_wait_ms'] == '3.2' and 'tf_wait_ms' not in vals[1]
