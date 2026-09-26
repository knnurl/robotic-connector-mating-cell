#!/usr/bin/env python3
"""object_source 'depth_checked' rules (depth_checked.py): raw is the frame's
depth estimate and nothing else, only after five accepted estimates on one
track, never when the marker vetoes it (position, tilt and in-plane each
bounded), a prediction or while the part is held; a restart always costs a
fresh acquisition, a dropped frame only pauses one."""

import numpy as np

from roscam.depth_checked import ACQUIRE_FRAMES, DepthChecked, pose7

DT = 1.0 / 15.0
R_DOWN = np.diag([1.0, -1.0, -1.0])


def T_at(x_mm=0.0):
    T = np.eye(4)
    T[:3, :3] = R_DOWN
    T[:3, 3] = [x_mm / 1000.0, 0.0, 0.150]
    return T


# the camera 400 mm above the base, looking down, 0.5 m out
T_FC = np.eye(4)
T_FC[:3, :3] = R_DOWN
T_FC[:3, 3] = [0.5, 0.0, 0.4]


def est(T, valid=True, mm=0.5, tilt=0.3, inp=0.2, reason=''):
    return (T, valid, {'agree_mm': mm, 'agree_tilt_deg': tilt, 'agree_inplane_deg': inp,
                       'reason': reason})


class Run:
    """Frames at 15 Hz through one DepthChecked."""

    def __init__(self, **kw):
        self.dc, self.t = DepthChecked(**kw), 100.0

    def __call__(self, e, T_fc=T_FC, holding=False, why=''):
        self.t += DT
        return self.dc.step(self.t, e, T_fc, holding=holding, why=why)

    def acquire(self, T=None):
        T = T_at() if T is None else T
        return [self(est(T)) for _ in range(ACQUIRE_FRAMES)]


def test_raw_comes_after_five_agreeing_frames_and_is_the_estimate_itself():
    run = Run()
    out = run.acquire()
    assert [o[0] is None for o in out] == [True] * 4 + [False]
    assert [o[2] for o in out[:4]] == [f'acquiring {i}/5' for i in range(1, 5)]
    T = T_at(1.0)
    raw, pose, check = run(est(T))
    assert raw is T and check == 'ok'                      # the estimate, not the filter
    t_meas = (T_FC @ T)[:3, 3]
    assert np.linalg.norm(pose[0] - t_meas) < 0.002        # the filter, in the filter frame


def test_a_veto_stops_raw_the_pose_coasts_briefly_then_the_track_restarts():
    run = Run()
    run.acquire()
    coasted = []
    for _ in range(6):                                      # 0.4 s of marker disagreement
        raw, pose, check = run(est(T_at(), mm=3.4))
        assert raw is None and check.startswith('vetoed by the marker: 3.4 mm')
        coasted.append(pose is not None)
    assert coasted == [True] * 4 + [False] * 2              # 0.3 s of prediction, then silent
    out = run.acquire()                                     # agreement is back: a new acquisition
    assert [o[0] is None for o in out] == [True] * 4 + [False]


def test_the_veto_bounds_position_tilt_and_in_plane_each():
    run = Run()
    run.acquire()
    assert run(est(T_at(), tilt=3.9))[0] is not None        # the marker's tilt is the noisy one
    raw, _, check = run(est(T_at(), tilt=4.1))
    assert raw is None and 'tilt 4.1 deg' in check
    assert run(est(T_at(), inp=-1.9))[0] is not None
    raw, _, check = run(est(T_at(), inp=-2.1))
    assert raw is None and 'in-plane -2.1 deg' in check
    assert run(est(T_at(), mm=3.1))[0] is None


def test_raw_never_comes_from_a_prediction_or_without_an_estimate():
    run = Run()
    run.acquire()
    for why in ('no marker prior', 'budget: 20 ms left', ''):
        raw, pose, check = run(None, why=why)
        assert raw is None and check == (why or 'no estimate')
    raw, pose, check = run(est(T_at(), valid=False, reason='rms 2.1'))
    assert raw is None and check == 'depth invalid: rms 2.1'


def test_holding_suspends_raw_and_the_release_restarts_the_track():
    run = Run()
    run.acquire()
    assert run(est(T_at()), holding=True) == (None, None, 'HELD')
    out = run.acquire()
    assert [o[0] is None for o in out] == [True] * 4 + [False]


def test_a_reset_forces_a_fresh_acquisition():
    run = Run()
    run.acquire()
    run.dc.reset()                                          # a source switch
    out = run.acquire()
    assert [o[0] is None for o in out] == [True] * 4 + [False]


def test_a_jump_is_gated_until_it_is_acquired_as_a_new_pose():
    run = Run()
    run.acquire()
    T_new = T_at(40.0)                                      # the part moved 40 mm
    raws = [run(est(T_new))[0] for _ in range(2 * ACQUIRE_FRAMES)]
    first = next(i for i, r in enumerate(raws) if r is not None)
    assert first >= ACQUIRE_FRAMES                          # never before a new acquisition
    assert all(r is T_new for r in raws[first:])


def test_no_tf_gives_no_raw():
    run = Run()
    run.acquire()
    raw, _, check = run(est(T_at()), T_fc=None)
    assert raw is None and check == 'no TF at the image stamp'


def test_the_veto_rate_counts_valid_estimates_only():
    run = Run()
    for e in [est(T_at())] * 6 + [est(T_at(), mm=5.0)] * 2 + [est(T_at(), valid=False)] * 5:
        run(e)
    run(None)
    assert run.dc.veto_pct() == 25.0


def test_a_dropped_frame_pauses_an_acquisition_and_only_the_lost_track_limit_restarts_it():
    run = Run()
    why = 'budget: 20 ms left, the last took 30'
    out = [run(est(T_at())) if i % 2 == 0 else run(None, why=why) for i in range(9)]
    assert all(o[0] is None for o in out[:8]) and out[8][0] is not None
    assert out[1][2] == f'{why} (acquiring 1/5)'
    raw, pose, check = run(None, why=why)                   # acquired: it coasts
    assert raw is None and pose is not None and check == why
    run = Run()
    for _ in range(3):
        run(est(T_at()))
    assert run(est(T_at(), mm=3.5))[2].endswith('(acquiring 3/5)')   # a veto pauses too
    for _ in range(4):
        run(None, why='no marker prior')                    # 0.33 s since the last accepted
    assert run(est(T_at()))[2] == 'acquiring 1/5'           # the track was lost: from 1


def test_a_kf_gate_rejection_while_acquiring_restarts_from_that_estimate():
    run = Run()
    for _ in range(2):
        run(est(T_at()))
    T_new = T_at(40.0)
    out = [run(est(T_new)) for _ in range(ACQUIRE_FRAMES)]
    assert [o[2] for o in out[:4]] == [f'acquiring {i}/5' for i in range(1, 5)]
    assert out[4][0] is T_new


def test_a_non_finite_estimate_is_refused():
    run = Run()
    run.acquire()
    T = T_at()
    T[0, 3] = np.nan
    for e in (est(T), est(T_at(), mm=float('nan')), est(T_at(), tilt=float('nan'))):
        raw, _, check = run(e)
        assert raw is None and check == 'depth invalid: not finite'


def test_a_gross_disagreement_counts_as_a_veto_even_when_the_estimate_is_invalid():
    run = Run()
    for _ in range(3):
        run(est(T_at()))
    raw, _, check = run(est(T_at(), valid=False, mm=12.0, reason='shift 12.0 mm / 1.0 deg'))
    assert raw is None and check.startswith('vetoed by the marker: 12.0 mm')
    assert run.dc.veto_pct() == 25.0
    run.dc.reset()
    assert run.dc.veto_pct() == 25.0                        # a lost track keeps the rate
    run.dc.reset(clear_rate=True)                           # a source switch does not
    assert run.dc.veto_pct() is None


def test_pose7_round_trips_a_half_turn():
    t, q = pose7(T_FC)
    assert np.allclose(t, T_FC[:3, 3]) and np.isclose(np.linalg.norm(q), 1.0)
    assert np.isclose(abs(q[0]), 1.0)                       # 180 deg about x
