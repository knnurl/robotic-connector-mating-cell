#!/usr/bin/env python3
"""The object pose contract, source 'marker': through the real cam_pub, every
/object/* message is the /aruco/* one unchanged, and one quality status goes
out per processed frame."""

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter

from roscam.cam_pub import ArucoPosePublisher
from roscam.object_contract import (POSE_TOPIC, QUALITY_TOPIC, RAW_POSE_TOPIC,
                                    ObjectContract)
from test_cam_pub_range import R_TRUE, SIZE, T_TRUE, render
from test_plane_normal import CX, CY, FX, FY, synth_depth


@pytest.fixture(scope='module')
def ros():
    rclpy.init(domain_id=88)                        # never the cell's domain
    yield
    rclpy.shutdown()


def capture(pub, into):
    pub.publish = lambda msg: into.append(msg)


def node_with_contract():
    node = ArucoPosePublisher(parameter_overrides=[
        Parameter('source', value='external'), Parameter('marker_id', value=0),
        Parameter('marker_size_m', value=SIZE), Parameter('target_marker_id', value=-1),
        Parameter('publish_debug_image', value=False)])
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
