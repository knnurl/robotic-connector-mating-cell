"""Unit tests for the 6-DOF pose Kalman filter."""
import math

import numpy as np
import pytest

from roscam.pose_kf import (PoseKF, compose_pose, quat_from_rotvec,
                            quat_multiply, quat_rotate)

RATE = 30.0
DT = 1.0 / RATE
POS_NOISE = 0.002   # 2 mm, matches default meas_std_pos
ROT_NOISE = math.radians(1.0)


def trajectory(t):
    """Smooth relative marker-camera motion (robot servoing)."""
    pos = np.array([0.05 * math.sin(0.8 * t),
                    0.03 * math.cos(0.5 * t),
                    0.30 + 0.02 * math.sin(0.3 * t)])
    q = quat_from_rotvec(np.array([0.1 * math.sin(0.4 * t), 0.05 * t % 0.2, 0.0]))
    return pos, q


def noisy(pos, q, rng):
    n_pos = pos + rng.normal(0.0, POS_NOISE, 3)
    n_q = quat_multiply(quat_from_rotvec(rng.normal(0.0, ROT_NOISE, 3)), q)
    return n_pos, n_q / np.linalg.norm(n_q)


def run_filter(kf, rng, seconds=5.0, dropout=None):
    """Feed the trajectory; returns (filtered_errs, raw_errs) position errors."""
    filt_errs, raw_errs = [], []
    steps = int(seconds * RATE)
    for i in range(steps):
        t = i * DT
        pos, q = trajectory(t)
        if kf.initialized:
            kf.predict(DT)
        if dropout and dropout[0] <= t < dropout[1]:
            continue  # no measurement, prediction only
        z_pos, z_q = noisy(pos, q, rng)
        kf.update(z_pos, z_q)
        if t > 1.0:  # after convergence
            filt_errs.append(np.linalg.norm(kf.position - pos))
            raw_errs.append(np.linalg.norm(z_pos - pos))
    return np.array(filt_errs), np.array(raw_errs)


def test_smoothing_beats_raw_noise():
    kf = PoseKF()
    rng = np.random.default_rng(1)
    filt, raw = run_filter(kf, rng)
    assert filt.mean() < 0.7 * raw.mean(), \
        f'filter RMS {filt.mean() * 1000:.2f} mm not < 70% of raw {raw.mean() * 1000:.2f} mm'


def test_outlier_rejected_and_state_unaffected():
    kf = PoseKF()
    rng = np.random.default_rng(2)
    run_filter(kf, rng, seconds=2.0)
    pos_before = kf.position.copy()
    kf.predict(DT)
    accepted = kf.update(pos_before + np.array([0.10, 0.0, 0.0]),
                         kf.quaternion)  # 10 cm jump
    assert not accepted
    assert np.linalg.norm(kf.position - pos_before) < 0.005


def test_prediction_bridges_short_dropout():
    kf = PoseKF()
    rng = np.random.default_rng(3)
    # 0.3 s dropout mid-run; filter predicts through it
    steps = int(4.0 * RATE)
    max_pred_err = 0.0
    for i in range(steps):
        t = i * DT
        pos, q = trajectory(t)
        if kf.initialized:
            kf.predict(DT)
        if 2.0 <= t < 2.3:
            max_pred_err = max(max_pred_err, np.linalg.norm(kf.position - pos))
            continue
        kf.update(*noisy(pos, q, rng))
    assert max_pred_err < 0.010, f'prediction drifted {max_pred_err * 1000:.1f} mm in 0.3 s'


def test_reacquire_after_reset():
    kf = PoseKF()
    rng = np.random.default_rng(4)
    run_filter(kf, rng, seconds=1.0)
    kf.reset()
    assert not kf.initialized
    new_pos = np.array([1.0, 2.0, 3.0])
    assert kf.update(new_pos, np.array([0.0, 0.0, 0.0, 1.0]))
    assert np.allclose(kf.position, new_pos)


def test_gate_loosens_with_covariance_growth():
    """After a long prediction gap the gate must accept a valid measurement."""
    kf = PoseKF()
    rng = np.random.default_rng(5)
    run_filter(kf, rng, seconds=2.0)
    for _ in range(30):  # 1 s of pure prediction
        kf.predict(DT)
    pos, q = trajectory(3.0)
    assert kf.update(*noisy(pos, q, rng)), \
        'valid measurement rejected after covariance growth'


def test_compose_pose_matches_manual_chain():
    """(a<-b) o (b<-c): rotate-then-translate must match manual composition."""
    q_ab = quat_from_rotvec(np.array([0.0, 0.0, math.pi / 2.0]))  # 90 deg yaw
    p_ab = np.array([1.0, 0.0, 0.0])
    q_bc = quat_from_rotvec(np.array([0.2, -0.1, 0.3]))
    p_bc = np.array([0.0, 0.5, 0.2])

    p_ac, q_ac = compose_pose(p_ab, q_ab, p_bc, q_bc)

    # +90 deg yaw maps (0, 0.5, 0.2) -> (-0.5, 0, 0.2), plus (1, 0, 0).
    assert np.allclose(p_ac, [0.5, 0.0, 0.2], atol=1e-12)
    assert np.allclose(q_ac, quat_multiply(q_ab, q_bc), atol=1e-12)
    # Rotations compose: rotating a probe vector through q_ac must equal
    # rotating it through q_bc then q_ab.
    v = np.array([0.3, -0.7, 0.9])
    assert np.allclose(quat_rotate(q_ac, v),
                       quat_rotate(q_ab, quat_rotate(q_bc, v)), atol=1e-12)


def test_fixed_frame_filtering_survives_camera_step():
    """A static marker seen by a stepping eye-in-hand camera.

    Filtered in the OPTICAL frame, a coarse robot step (5 cm between frames)
    appears as marker motion and must trip the innovation gate on a good
    detection. Filtered in a FIXED frame (measurements re-expressed via TF,
    as cam_pub's filter_frame does), the same detections are constant and
    every one must pass. This is the failure mode filter_frame exists for.
    """
    kf_optical = PoseKF()
    kf_fixed = PoseKF()
    rng = np.random.default_rng(6)

    marker_fixed = np.array([0.40, 0.10, 0.0])
    q_marker = np.array([0.0, 0.0, 0.0, 1.0])

    def camera_pos(i):
        return (np.array([0.0, 0.0, 0.30]) if i < 10
                else np.array([0.05, 0.0, 0.30]))  # coarse-align step

    optical_rejected = False
    fixed_all_accepted = True
    for i in range(20):
        # Translation-only camera: optical measurement = marker - camera.
        z_optical = marker_fixed - camera_pos(i) + rng.normal(0.0, POS_NOISE, 3)
        z_fixed = z_optical + camera_pos(i)  # re-expressed in the fixed frame

        if kf_optical.initialized:
            kf_optical.predict(DT)
        if not kf_optical.update(z_optical, q_marker):
            optical_rejected = True

        if kf_fixed.initialized:
            kf_fixed.predict(DT)
        if not kf_fixed.update(z_fixed, q_marker):
            fixed_all_accepted = False

    assert optical_rejected, \
        'optical-frame filter accepted a 5 cm apparent jump - gate broken?'
    assert fixed_all_accepted, \
        'fixed-frame filter rejected a static-marker detection'
    assert np.linalg.norm(kf_fixed.position - marker_fixed) < 0.005


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
