#!/usr/bin/env python3
"""Servo stall watchdog, checked against the real stall it exists for.

On 2026-09-15 a conservative servo run commanded ~10 mm/s and 7 deg/s for
the last 90 s of a 144 s run while the arm did not move (error flat at
12.7 mm / 8.8 deg). testdata/servo_stall_20260915_174518.csv.gz is that
run's trace: t, position error, tilt, in-plane error, one row per command.

    python3 -m pytest tools/fr3/test_servo_stall.py -q
"""

import csv
import gzip
import pathlib

import numpy as np

TRACE = (pathlib.Path(__file__).resolve().parent / 'testdata'
         / 'servo_stall_20260915_174518.csv.gz')


def _rows():
    with gzip.open(TRACE, 'rt') as f:
        return np.array([[float(r[k]) for k in
                          ('t', 'err_mm', 'tilt_deg', 'inplane_err_deg')]
                         for r in csv.DictReader(f)])


def _first_trip(ag, rows):
    w = ag.StallWatchdog()
    w.reset(rows[0, 0])
    for t, e, tilt, ipe in rows:
        if w.update(t, e / 1000.0, tilt, ipe):
            return t
    return None


def test_real_hardware_stall_is_caught(ag):
    """Error fell 183 -> 16 mm in the first 40 s (real progress, must not
    trip), then crept to a flat 12.7 mm from ~50 s (must trip)."""
    t = _first_trip(ag, _rows())
    assert t is not None, 'the 90 s hardware stall went unnoticed'
    assert 40.0 <= t <= 75.0, f'tripped at {t:.1f} s'


def test_noisy_healthy_convergence_never_trips(ag):
    """Conservative profile shape: capped at 30 mm/s, then exponential
    (gain 0.8 -> tau 1.25 s), with this cell's measured noise. Watched only
    while clearly far from target, as the servo loop does."""
    rng = np.random.default_rng(1)
    w = ag.StallWatchdog()
    w.reset(0.0)
    err, tilt, ipe, dt = 183.0, 20.0, 3.0, 1 / 30
    for k in range(int(60 / dt)):
        t = k * dt
        err -= min(30.0, 0.8 * err) * dt
        tilt -= min(15.0, 0.8 * tilt) * dt
        ipe -= min(15.0, 0.8 * ipe) * dt
        if err < 4.0 and tilt < 2.0 and ipe < 1.0:
            return
        m = (err + rng.normal(0, 0.1), tilt + abs(rng.normal(0, 0.31)),
             ipe + abs(rng.normal(0, 0.08)))
        assert not w.update(t, m[0] / 1000.0, m[1], m[2]), f'false trip {t:.1f}s'
    raise AssertionError('synthetic run did not converge - test is broken')


def test_noisy_plateau_trips_after_the_window(ag):
    rng = np.random.default_rng(2)
    w = ag.StallWatchdog()
    w.reset(0.0)
    dt, tripped = 1 / 30, None
    for k in range(int(20 / dt)):
        t = k * dt
        if w.update(t, (12.7 + rng.normal(0, 0.1)) / 1000.0,
                    8.8 + rng.normal(0, 0.31), 2.6 + rng.normal(0, 0.08)):
            tripped = t
            break
    assert tripped is not None
    assert ag.SERVO_STALL_S <= tripped <= ag.SERVO_STALL_S + 2.0, tripped


def test_diverging_counts_as_no_progress(ag):
    w = ag.StallWatchdog()
    w.reset(0.0)
    trips = [w.update(k / 30, (20 + k * 0.05) / 1000.0, 5.0, 1.0)
             for k in range(int(10 * 30))]
    assert any(trips)


def test_reset_restarts_the_window(ag):
    w = ag.StallWatchdog()
    w.reset(0.0)
    for k in range(int(3.5 * 30)):
        assert not w.update(k / 30, 0.0127, 8.8, 2.6)
    w.reset(3.5)
    for k in range(int(3.5 * 30), int(7.0 * 30)):
        assert not w.update(k / 30, 0.0127, 8.8, 2.6), 'reset was ignored'
