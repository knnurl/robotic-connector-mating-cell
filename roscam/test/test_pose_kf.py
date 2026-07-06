"""Unit tests for the 6-DOF pose Kalman filter."""
import math

import numpy as np
import pytest

from roscam.pose_kf import PoseKF, quat_from_rotvec, quat_multiply

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


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
