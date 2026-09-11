"""Tests for the auto-teach math (ROS-free: pure transforms + averaging)."""
import math
import sys

import numpy as np
import pytest

from roscam.handeye_calib import to_homogeneous
from roscam.teach_offsets import average_offsets, offsets_from_pair


def rot_z(deg):
    r = math.radians(deg)
    return np.array([[math.cos(r), -math.sin(r), 0.0],
                     [math.sin(r), math.cos(r), 0.0],
                     [0.0, 0.0, 1.0]])


def rot_x(deg):
    r = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, math.cos(r), -math.sin(r)],
                     [0.0, math.sin(r), math.cos(r)]])


def test_offsets_round_trip_known_transform():
    """Compose a known marker->connector offset into camera-frame poses;
    the tool must recover it exactly, regardless of the camera viewpoint."""
    T_marker_connector = to_homogeneous(rot_z(25.0), [0.031, -0.012, 0.004])
    # Arbitrary camera->marker viewpoint (tilted, offset).
    T_cam_marker = to_homogeneous(rot_x(160.0) @ rot_z(40.0), [0.05, 0.02, 0.31])
    T_cam_connector = T_cam_marker @ T_marker_connector

    xyz, yaw = offsets_from_pair(T_cam_marker, T_cam_connector)
    assert np.allclose(xyz, [0.031, -0.012, 0.004], atol=1e-12)
    assert abs(yaw - 25.0) < 1e-9


def test_average_rejects_nothing_but_reports_spread():
    rng = np.random.default_rng(7)
    truth_xyz = np.array([0.02, 0.01, 0.0])
    samples = [(truth_xyz + rng.normal(0, 0.0005, 3),
                10.0 + rng.normal(0, 0.3)) for _ in range(200)]
    xyz, xyz_std, yaw, yaw_std = average_offsets(samples)
    assert np.allclose(xyz, truth_xyz, atol=0.0003)
    assert abs(yaw - 10.0) < 0.2
    assert np.all(xyz_std < 0.001)
    assert yaw_std < 0.5


def test_yaw_circular_mean_across_wraparound():
    """Samples straddling +/-180 deg must not average toward zero."""
    samples = [(np.zeros(3), 179.0), (np.zeros(3), -179.0)] * 20
    _, _, yaw, yaw_std = average_offsets(samples)
    assert abs(abs(yaw) - 180.0) < 1e-6
    assert yaw_std < 1.1


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
