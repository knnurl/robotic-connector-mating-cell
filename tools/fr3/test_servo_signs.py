#!/usr/bin/env python3
"""Closed-loop checks for align_gui's servo backend - run before hardware.

A sign error in a velocity servo does not fail gracefully: it drives the arm
AWAY from the target, faster the further it gets. The three discrete steps
this backend is derived from (translate, level, in-plane) do not share a
sign convention, and two errors of exactly that kind were caught here before
they reached the robot. These tests simulate the loop against a static
marker and assert it converges - and that each of those original bugs, if
re-introduced, makes it fail.

    python3 -m pytest tools/fr3/test_servo_signs.py -q

No ROS needed: conftest.py loads the GUI module with its ROS imports stubbed,
and only the pure control law (Gui._servo_error, Gui._clamp) is exercised.
"""

import numpy as np
import pytest

# Measured 2026-09-15 by handeye_calib (T_tcp->camera_optical). Servo turns
# the TCP, not the camera, so rotation drags the camera through this lever
# arm - a coupling the camera-frame control law does not compensate for.
HANDEYE_T = np.array([0.061126, -0.011144, -0.046550])
HANDEYE_Q = (0.000855, 0.003126, 0.706706, 0.707500)


class _Var:
    def __init__(self, v):
        self.v = v

    def get(self):
        return self.v


class _Gui:
    """Just the attributes _servo_error reads."""

    def __init__(self, profile):
        self.v_servo = _Var(profile)
        self.target_m = lambda: 0.100


def _expR(w):
    w = np.asarray(w, dtype=float)
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    a = w / th
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def _q2R(x, y, z, w):
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = np.array([x, y, z, w]) / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def simulate(ag, err_mm, tilt_deg, ip_deg, tgt_ip, seed, profile='moderate',
             lever=False, error_fn=None, max_s=60.0):
    """Integrate the servo loop against a static marker.

    Returns (converged, seconds, history[(err, tilt, inplane_err, |cmd|)]).
    With lever=True the twist is applied at the TCP, as servo really does,
    so rotation also translates the camera by omega x r.
    """
    rng = np.random.default_rng(seed)
    gui = _Gui(profile)
    fn = error_fn or ag.Gui._servo_error
    P = ag.SERVO_PROFILES[profile]
    Rx = _q2R(*HANDEYE_Q)

    M_R, M_p = np.eye(3), np.zeros(3)
    ax = rng.normal(size=3)
    ax[2] = 0.0
    ax /= np.linalg.norm(ax)
    C_R = (_expR([0, 0, np.radians(ip_deg)]) @ _expR(ax * np.radians(tilt_deg))
           @ np.diag([1.0, -1.0, -1.0]))
    off = rng.normal(size=3)
    off /= np.linalg.norm(off)
    C_p = M_p - C_R @ np.array([0, 0, 0.100]) + off * err_mm / 1000.0

    dt, hist = 1.0 / ag.SERVO_RATE_HZ, []
    for k in range(int(max_s / dt)):
        R_cm, p_cm = C_R.T @ M_R, C_R.T @ (M_p - C_p)
        lin, ang, err, tilt, ip = fn(gui, (p_cm, R_cm), tgt_ip)
        lin = ag.Gui._clamp(lin, P['lin'])
        ang = ag.Gui._clamp(ang, np.radians(P['ang']))
        ipe = 0.0 if tgt_ip is None else abs(ag.wrap_deg(ip - tgt_ip))
        hist.append((err, tilt, ipe, float(np.linalg.norm(lin))))
        if err < 0.002 and tilt < ag.ROT_TOL_DEG and ipe < ag.INPLANE_TOL_DEG:
            return True, k * dt, hist
        v = C_R @ lin
        if lever:
            # camera origin relative to TCP, world frame: C_R = T_R @ Rx
            r_w = C_R @ Rx.T @ HANDEYE_T
            v = v + np.cross(C_R @ ang, r_w)
        C_p = C_p + v * dt
        C_R = C_R @ _expR(ang * dt)
    return False, max_s, hist


CASES = [
    (150, 18, 0, None, 'in-plane off'),
    (150, 18, 84, 90, 'the measured 84->90 correction'),
    (40, 5, -30, 0, 'small offset, in-plane -30->0'),
    (200, 25, 170, -170, 'wrap across +/-180'),
    (5, 0.5, 89, 90, 'already near target'),
]


@pytest.mark.parametrize('lever', [False, True], ids=['no-lever', 'lever'])
@pytest.mark.parametrize('err_mm,tilt,ip,tgt,name', CASES,
                         ids=[c[4] for c in CASES])
def test_servo_converges_all_dof(ag, err_mm, tilt, ip, tgt, name, lever):
    ok, secs, hist = simulate(ag, err_mm, tilt, ip, tgt, seed=7, lever=lever)
    assert ok, (f'{name} (lever={lever}) did not converge in {secs:.0f}s: '
                f'err {hist[-1][0]*1000:.2f} mm, tilt {hist[-1][1]:.2f} deg, '
                f'in-plane {hist[-1][2]:.2f} deg')
    assert hist[-1][0] < hist[0][0] or err_mm < 10, 'error grew'
    assert secs < 30.0, f'{name} too slow: {secs:.1f}s'


@pytest.mark.parametrize('profile', ['conservative', 'moderate', 'brisk'])
def test_speed_cap_is_honoured(ag, profile):
    ok, _, hist = simulate(ag, 150, 18, 84, 90, seed=9, profile=profile)
    cap = ag.SERVO_PROFILES[profile]['lin']
    peak = max(h[3] for h in hist)
    assert ok
    assert peak <= cap + 1e-9, f'{profile}: {peak*1000:.1f} > {cap*1000:.0f} mm/s'


def test_linear_sign_bug_would_be_caught(ag):
    """The original bug: linear command negated -> drives AWAY."""
    orig = ag.Gui._servo_error

    def bug(self, m, tgt_ip):
        lin, ang, err, tilt, ip = orig(self, m, tgt_ip)
        return -lin, ang, err, tilt, ip
    ok, _, hist = simulate(ag, 150, 18, 84, 90, seed=1, error_fn=bug,
                           max_s=10.0)
    assert not ok
    assert hist[-1][0] > hist[0][0], 'a sign-flipped servo must diverge'


def test_inplane_sign_bug_would_be_caught(ag):
    """The original bug: in-plane shared tilt's negation -> spins away."""
    orig = ag.Gui._servo_error

    def bug(self, m, tgt_ip):
        lin, ang, err, tilt, ip = orig(self, m, tgt_ip)
        if tgt_ip is not None:
            g = ag.SERVO_PROFILES[self.v_servo.get()]['gain']
            d = np.radians(ag.wrap_deg(ip - tgt_ip)) * g
            ang = ang - 2.0 * np.array([0, 0, 1.0]) * d
        return lin, ang, err, tilt, ip
    ok, _, hist = simulate(ag, 150, 18, 84, 90, seed=1, error_fn=bug,
                           max_s=15.0)
    assert not ok
    assert hist[-1][2] > 90.0, 'in-plane error should run away, not settle'


def test_clamp_preserves_direction(ag):
    v = np.array([3.0, 4.0, 0.0])
    c = ag.Gui._clamp(v, 1.0)
    assert abs(np.linalg.norm(c) - 1.0) < 1e-12
    assert np.allclose(c / np.linalg.norm(c), v / np.linalg.norm(v))
    small = np.array([0.1, 0.0, 0.0])
    assert np.allclose(ag.Gui._clamp(small, 1.0), small)
