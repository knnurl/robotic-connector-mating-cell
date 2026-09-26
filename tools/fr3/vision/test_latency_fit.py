#!/usr/bin/env python3
"""latency_fit against a synthetic eye-in-hand camera: a known stamp error
e (frames captured at t, stamped t + e) must come back as d* = -e.

    python3 -m pytest tools/fr3/vision/test_latency_fit.py -q
"""

import numpy as np
import pytest

import latency_fit as lf

T0 = 1790000000.0                               # epoch-sized stamps, as on the cell
FPS, SECONDS = 15.0, 8.0
E = 0.030                                       # stamps 30 ms late: d* must be -30 ms
MARKER = np.array([0.40, 0.00, 0.20])           # static, in the base frame
DELTAS = lf.default_deltas(0.06, 0.001)
BASE, HAND, OPTICAL = 'fr3_link0', 'fr3_hand', 'camera_color_optical_frame'


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def camera(t, moving=True):
    """T_base_optical(t): looking down at the marker from ~150 mm, weaving at
    up to ~200 mm/s with a few degrees of tilt and yaw."""
    a, u = float(moving), t - T0
    T = np.eye(4)
    T[:3, 3] = (0.40 + a * 0.025 * np.sin(2 * np.pi * 1.0 * u),
                a * 0.030 * np.sin(2 * np.pi * 0.7 * u + 0.3),
                0.35 + a * 0.010 * np.sin(2 * np.pi * 1.3 * u))
    T[:3, :3] = (np.diag([1.0, -1.0, -1.0])                 # optical z down
                 @ _rx(a * np.radians(6) * np.sin(2 * np.pi * 0.8 * u))
                 @ _rz(a * np.radians(8) * np.sin(2 * np.pi * 0.5 * u)))
    return T


def observations(moving=True, noise_mm=None, seed=1):
    """(stamps, p_cam): captured at t, stamped t + E."""
    t = T0 + np.arange(int(FPS * SECONDS)) / FPS
    p = np.array([np.linalg.solve(camera(ti, moving), np.append(MARKER, 1.0))[:3] for ti in t])
    if noise_mm is not None:
        p = p + np.random.default_rng(seed).normal(0.0, 1e-3, p.shape) * noise_mm
    return t + E, p


def fit(stamps, p, moving=True):
    return lf.fit_stamp_offset(stamps, p, lambda t: camera(t, moving), deltas=DELTAS,
                               n_boot=300, capture_latency_s=0.02)


@pytest.fixture(scope='module')
def clean():
    return fit(*observations())


def test_the_synthetic_camera_moves_as_the_fit_needs():
    s, _ = observations()
    v = [np.linalg.norm(camera(t + 0.005)[:3, 3] - camera(t - 0.005)[:3, 3]) / 0.01
         for t in s]
    assert 0.10 < np.percentile(v, 50) and max(v) < 0.26


def test_recovers_minus_the_stamp_error(clean):
    assert clean['d_star'] == pytest.approx(-E, abs=0.002)
    assert clean['ci95'][0] <= -E + 1e-9 <= clean['ci95'][1] + 2e-3
    assert 0.8 * clean['n_frames'] < clean['n_moving'] <= clean['n_frames']
    # static marker, exact TF: the scatter vanishes at d* and not as stamped
    assert clean['rms_min_mm'] < 0.1 < 1.0 < clean['rms_zero_mm']
    assert clean['curvature'] > 0 and not clean['at_edge']


def test_true_latency_is_the_assumed_one_minus_d(clean):
    # stamped with 20 ms but 30 ms late: the camera really lags 50 ms
    assert clean['latency'] == pytest.approx(0.050, abs=0.002)
    lo, hi = clean['latency_ci95']
    assert lo <= clean['latency'] <= hi


def test_static_camera_cannot_determine_d():
    res = fit(*observations(moving=False), moving=False)
    assert res['d_star'] is None and res['n_moving'] == 0
    assert 'cannot determine' in res['reason']
    assert 'latency' not in res


def test_noise_widens_the_ci_but_keeps_d(clean):
    res = fit(*observations(noise_mm=np.array([1.0, 1.0, 3.0])))   # ArUco: z worst
    assert res['d_star'] == pytest.approx(-E, abs=0.002)
    lo, hi = res['ci95']
    assert lo <= -E <= hi
    assert hi - lo > (clean['ci95'][1] - clean['ci95'][0]) + 0.0015
    assert res['rms_min_mm'] > clean['rms_min_mm'] + 1.0


def test_frames_without_tf_are_dropped():
    s, p = observations()
    lo_t, hi_t = s[0] + 0.5, s[-1] - 0.5
    res = lf.fit_stamp_offset(s, p, lambda t: camera(t) if lo_t <= t <= hi_t else None,
                              deltas=DELTAS, n_boot=50)
    assert res['n_tf'] < res['n_frames'] and res['n_moving'] < res['n_tf']
    assert res['d_star'] == pytest.approx(-E, abs=0.002)


def _hand_optical():
    T = np.eye(4)
    T[:3, :3] = _rz(np.radians(-45)) @ _rx(np.radians(10))
    T[:3, 3] = (0.05, -0.01, 0.04)
    return T


def _quat(R):
    """(x, y, z, w) of a rotation matrix, any angle (base -> hand is near 180)."""
    d = np.diag(R)
    x, y, z, w = (np.sqrt(max(0.0, 1.0 + s @ d)) / 2
                  for s in np.array([[1, -1, -1], [-1, 1, -1], [-1, -1, 1], [1, 1, 1]]))
    return (np.copysign(x, R[2, 1] - R[1, 2]), np.copysign(y, R[0, 2] - R[2, 0]),
            np.copysign(z, R[1, 0] - R[0, 1]), w)


def _tf(parent, child, T, t):
    from geometry_msgs.msg import TransformStamped
    m = TransformStamped()
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(int(round(t * 1e9)), 10 ** 9)
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = T[:3, 3]
    r = m.transform.rotation
    r.x, r.y, r.z, r.w = _quat(T[:3, :3])
    return m


def test_bag_round_trip(tmp_path, monkeypatch):
    """A bag laid out like the REC button's: fr3_link0 -> fr3_hand on /tf at
    200 Hz, the hand-eye on /tf_static, raw poses in the optical frame. Pins
    the lookup direction and the quaternion convention."""
    rosbag2_py = pytest.importorskip('rosbag2_py')
    # rosbag2 splits paths on '\' too, and tmp_path holds the user name
    # (DOMAIN\user here): hand it a relative path.
    monkeypatch.chdir(tmp_path)
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import serialize_message
    from tf2_msgs.msg import TFMessage

    X = _hand_optical()
    bag = 'bag'
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name, kind in (('/tf', 'tf2_msgs/msg/TFMessage'), ('/tf_static', 'tf2_msgs/msg/TFMessage'),
                       ('/aruco/pose_raw', 'geometry_msgs/msg/PoseStamped')):
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format='cdr'))
    w.write('/tf_static', serialize_message(TFMessage(transforms=[_tf(HAND, OPTICAL, X, T0)])),
            int(T0 * 1e9))
    stamps, p = observations()
    events = [(t, 'tf') for t in T0 + np.arange(int(200 * SECONDS)) / 200.0]
    events += [(s, i) for i, s in enumerate(stamps)]
    for t, what in sorted(events, key=lambda e: e[0]):
        if what == 'tf':
            msg, topic = TFMessage(transforms=[
                _tf(BASE, HAND, camera(t) @ np.linalg.inv(X), t)]), '/tf'
        else:
            msg, topic = PoseStamped(), '/aruco/pose_raw'
            msg.header.stamp = _tf('', '', np.eye(4), t).header.stamp
            msg.header.frame_id = OPTICAL
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p[what]
            msg.pose.orientation.w = 1.0
        w.write(topic, serialize_message(msg), int(round(t * 1e9)))
    del w

    s, p_cam, optical, T_at = lf.load_bag(bag)
    assert optical == OPTICAL and len(s) == len(stamps)
    for t in s[[17, 38, 71, 90]] - E:   # between TF samples, yaw and tilt non-zero
        assert np.allclose(T_at(t), camera(t), atol=1e-5)   # interpolation: ~3 um
    assert T_at(s[-1] + 1.0) is None
    res = lf.fit_stamp_offset(s, p_cam, T_at, deltas=DELTAS, n_boot=50)
    assert res['d_star'] == pytest.approx(-E, abs=0.002)
