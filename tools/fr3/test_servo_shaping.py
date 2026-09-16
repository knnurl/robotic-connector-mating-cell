#!/usr/bin/env python3
"""Servo command shaping, checked against a sim with REAL-WORLD imperfections.

test_servo_signs.py proves the control law converges - but that sim had no
latency and no measurement noise, and it passed while the real FR3 shook
itself into cartesian_reflex. This file adds what that sim missed:

  * 80 ms measurement latency and a 60 ms first-order actuator lag
  * the per-frame noise measured on this cell (position 0.1 mm, tilt
    0.31 deg, in-plane 0.08 deg) and the 78 mm camera-TCP lever arm

With those in, the RAW command reproduces the roughness seen on hardware
(single-frame angular jumps ~11.5 deg/s vs 11.7 measured), which is what
makes the shaper assertions meaningful.

The sim still omits the arm's structural resonance, so it can show the
shaper removes the excitation - not that a given speed is safe on metal.

    python3 -m pytest tools/fr3/test_servo_shaping.py -q
"""

import collections

import numpy as np
import pytest

HANDEYE_T = np.array([0.061126, -0.011144, -0.046550])
HANDEYE_Q = (0.000855, 0.003126, 0.706706, 0.707500)
SIG_POS = 0.0001
SIG_TILT = np.radians(0.44 / np.sqrt(2))   # measured per-frame diff std
SIG_IP = np.radians(0.12 / np.sqrt(2))
DELAY_S = 0.08
TAU_S = 0.06


class _Var:
    def __init__(self, v):
        self.v = v

    def get(self):
        return self.v


class _Gui:
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


def simulate(ag, profile, shaped, seed=3, max_s=30.0, sub_hz=300):
    """Run the servo loop with latency, lag, noise and lever arm.

    Returns dict(ok, t, lin_steps, ang_steps [per-control-frame command
    change, m/s and rad/s], watchdog_tripped).
    """
    rng = np.random.default_rng(seed)
    gui, P = _Gui(profile), ag.SERVO_PROFILES[profile]
    Rx = _q2R(*HANDEYE_Q)
    C_R = (_expR([0, 0, np.radians(101.8)])
           @ _expR(np.array([1.0, 0.3, 0]) / np.linalg.norm([1.0, 0.3, 0])
                   * np.radians(7.0))
           @ np.diag([1.0, -1.0, -1.0]))
    C_p = -C_R @ np.array([0, 0, 0.1]) + np.array([0.13, -0.12, 0.28])

    sub, ctl = 1.0 / sub_hz, 1.0 / ag.SERVO_RATE_HZ
    hist = collections.deque(maxlen=int(DELAY_S / sub) + 2)
    v_act, w_act = np.zeros(3), np.zeros(3)
    shaper = ag.CommandShaper()
    lin_cmd, ang_cmd = np.zeros(3), np.zeros(3)
    lin_steps, ang_steps, tripped = [], [], False
    next_ctl = 0.0
    for k in range(int(max_s / sub)):
        tnow = k * sub
        hist.append((C_R.copy(), C_p.copy()))
        if tnow >= next_ctl:
            next_ctl += ctl
            R_d, p_d = hist[0]
            R_cm = R_d.T @ _expR(rng.normal(0, [SIG_TILT, SIG_TILT, SIG_IP]))
            p_cm = R_d.T @ (-p_d) + rng.normal(0, SIG_POS, 3)
            lin, ang, err, tilt, ip = ag.Gui._servo_error(gui, (p_cm, R_cm),
                                                          90.0)
            ipe = abs(ag.wrap_deg(ip - 90.0))
            if (tnow > 1.0 and err < 0.002 and tilt < ag.ROT_TOL_DEG
                    and ipe < ag.INPLANE_TOL_DEG):
                return dict(ok=True, t=tnow, lin_steps=lin_steps,
                            ang_steps=ang_steps, watchdog_tripped=tripped)
            if shaped:
                lin, ang = shaper.filter(lin, ang, tilt, ipe)
            lin = ag.Gui._clamp(lin, P['lin'])
            ang = ag.Gui._clamp(ang, np.radians(P['ang']))
            if shaped:
                lin, ang = shaper.limit(lin, ang, ctl)
                tripped = tripped or shaper.oscillating()
            lin_steps.append(float(np.linalg.norm(lin - lin_cmd)))
            ang_steps.append(float(np.linalg.norm(ang - ang_cmd)))
            lin_cmd, ang_cmd = lin, ang
        v_act += (C_R @ lin_cmd - v_act) * (sub / TAU_S)
        w_act += (C_R @ ang_cmd - w_act) * (sub / TAU_S)
        r_w = C_R @ Rx.T @ HANDEYE_T
        C_p = C_p + (v_act + np.cross(w_act, r_w)) * sub
        C_R = _expR(w_act * sub) @ C_R
    return dict(ok=False, t=max_s, lin_steps=lin_steps, ang_steps=ang_steps,
                watchdog_tripped=tripped)


PROFILES = ['conservative', 'moderate', 'brisk']


def test_sim_reproduces_measured_hardware_roughness(ag):
    """Model sanity: the RAW command must be as rough as the real trace.

    Hardware conservative run: max single-frame angular jump 11.7 deg/s.
    If the raw sim were smooth, 'the shaper fixes it' would prove nothing.
    """
    raw = simulate(ag, 'conservative', shaped=False)
    peak = np.degrees(max(raw['ang_steps']))
    assert peak > 8.0, f'sim too clean ({peak:.1f} deg/s) - model is wrong'


@pytest.mark.parametrize('profile', PROFILES)
def test_shaped_servo_converges_without_tripping_watchdog(ag, profile):
    r = simulate(ag, profile, shaped=True)
    assert r['ok'], f'{profile} shaped did not converge in {r["t"]:.0f}s'
    assert not r['watchdog_tripped'], f'{profile}: watchdog false alarm'


@pytest.mark.parametrize('profile', PROFILES)
def test_shaped_command_obeys_acceleration_limits(ag, profile):
    r = simulate(ag, profile, shaped=True)
    dt = 1.0 / ag.SERVO_RATE_HZ
    assert max(r['lin_steps']) <= ag.SHAPER_LIN_ACC * dt + 1e-9
    assert max(r['ang_steps']) <= np.radians(ag.SHAPER_ANG_ACC_DEG) * dt + 1e-9


@pytest.mark.parametrize('profile', PROFILES)
def test_shaper_cuts_roughness_without_slowing_convergence(ag, profile):
    raw = simulate(ag, profile, shaped=False)
    shp = simulate(ag, profile, shaped=True)
    assert max(shp['ang_steps']) * 4 <= max(raw['ang_steps'])
    assert max(shp['lin_steps']) * 3 <= max(raw['lin_steps'])
    assert shp['ok'] and raw['ok']
    assert shp['t'] <= raw['t'] * 1.25, 'shaping should not cost much time'


def test_first_command_ramps_from_standstill(ag):
    sh = ag.CommandShaper()
    lin, ang = sh.limit(np.array([1.0, 0, 0]), np.array([0, 0, 1.0]), 1 / 30)
    assert np.linalg.norm(lin) <= ag.SHAPER_LIN_ACC / 30 + 1e-12
    assert np.linalg.norm(ang) <= np.radians(ag.SHAPER_ANG_ACC_DEG) / 30 + 1e-12


def test_watchdog_trips_on_oscillation(ag):
    sh = ag.CommandShaper()
    for i in range(ag.OSC_WINDOW + 2):
        s = 1.0 if i % 2 else -1.0
        # dt large enough that the limiter lets the command fully reverse
        sh.limit(np.array([0.05 * s, 0, 0]), np.array([0, 0, 0.5 * s]), 10.0)
    assert sh.oscillating()


def test_watchdog_quiet_on_steady_command(ag):
    sh = ag.CommandShaper()
    for _ in range(ag.OSC_WINDOW * 3):
        sh.limit(np.array([0.03, 0, 0]), np.array([0, 0, 0.1]), 1 / 30)
    assert not sh.oscillating()
    assert sh.flip_rate() == 0.0


def test_deadband_zeroes_orientation_near_target(ag):
    sh = ag.CommandShaper()
    _, ang = sh.filter(np.zeros(3), np.array([0.2, 0.1, 0.05]),
                       ag.DEAD_TILT_DEG * 0.5, ag.DEAD_IP_DEG * 0.5)
    assert np.allclose(ang, 0.0)
    _, ang = sh.filter(np.zeros(3), np.array([0.2, 0.1, 0.05]),
                       ag.DEAD_TILT_DEG * 2.0, 0.0)
    assert np.linalg.norm(ang) > 0.0, 'outside the deadband must still act'


def test_reset_returns_to_rest(ag):
    sh = ag.CommandShaper()
    for _ in range(10):
        sh.limit(np.array([0.05, 0, 0]), np.array([0, 0, 0.3]), 1 / 30)
    sh.reset()
    assert np.allclose(sh.lin, 0.0) and np.allclose(sh.ang, 0.0)
    assert sh.flip_rate() == 0.0
