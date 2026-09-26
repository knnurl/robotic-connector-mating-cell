#!/usr/bin/env python3
"""object_source 'depth_checked' (PERCEPTION_PLAN Phase 4): the depth
estimate drives, and the marker in the same frame can veto it.

ROS-free: the rules the vision node applies to every frame, so they are
tested without a camera or a graph. The estimate comes from object_shadow's
DepthShadow, seeded from the marker, so it exists only on frames that gave
a marker pose. Per frame:

  raw    the frame's depth estimate itself (optical frame, image stamp), only
         when it passed the estimator's gates, agrees with marker o
         T_marker_object from the same frame (the veto, below), passed the
         object KF's innovation gate, and the track is acquired. Never a
         prediction, a prior or a fallback.
  veto   within check_mm in position, check_inplane_deg about the part's Z
         (the estimator snapped it to the symmetry member nearest the
         marker, so modulo the symmetry) and check_tilt_deg of Z itself.
         The tilt bound is the looser one: the marker's tilt comes from the
         depth ring around its small square and is the noisier of the two
         (runs/2026-09-25/analysis, which_degrades.py).
  pose   the object KF in the filter frame (fr3_link0), updated by each
         accepted estimate. It coasts on its prediction for at most
         max_prediction_s after the last one; beyond that the track is lost
         and restarts, so the next raw needs a fresh acquisition.
  held   while GRIP holds the part it rides in the fingers: nothing, and the
         track restarts after the release.

Acquisition: acquire_frames accepted estimates on one KF track before the
first raw. A frame without one (no marker, vetoed, invalid, skipped for the
time budget) pauses the count; only the lost-track limit restarts it. A KF
gate rejection before the track is acquired restarts it from that estimate:
an early track the new data disagrees with is not one to finish.

reset() restarts the track (a source switch calls it). The KF is PoseKF with
the marker's settings, so /object/pose behaves as it did with the marker and
only the measurement changes.
"""

import collections

import numpy as np

from roscam.object_pose import _quat
from roscam.pose_kf import PoseKF

ACQUIRE_FRAMES = 5          # accepted estimates on one track before the first raw


def pose7(T):
    """(t, q (x, y, z, w)) of a 4x4."""
    return T[:3, 3].copy(), _quat(T[:3, :3])


class DepthChecked:
    def __init__(self, check_mm=3.0, check_tilt_deg=4.0, check_inplane_deg=2.0,
                 acquire_frames=ACQUIRE_FRAMES, max_prediction_s=0.3,
                 rejects_before_reacquire=5, veto_window=150, **kf):
        self.check_mm = float(check_mm)
        self.check_tilt_deg = float(check_tilt_deg)
        self.check_inplane_deg = float(check_inplane_deg)
        self.acquire_frames = int(acquire_frames)
        self.max_prediction_s = float(max_prediction_s)
        self.rejects_before_reacquire = int(rejects_before_reacquire)
        self.kf = PoseKF(**kf)
        self.vetoes = collections.deque(maxlen=int(veto_window))   # per valid estimate
        self.reset()

    def reset(self, clear_rate=False):
        """Restart the track: the next raw needs a fresh acquisition.
        clear_rate: also forget the veto rate (a source switch)."""
        if clear_rate:
            self.vetoes.clear()
        self.kf.reset()
        self.agreeing = 0           # accepted estimates on this track
        self.last_accept = None     # image stamp of the last accepted estimate
        self.last_stamp = None
        self.rejects = 0            # KF gate rejections in a row

    @property
    def acquired(self):
        return self.agreeing >= self.acquire_frames

    def veto_pct(self):
        """Vetoed share of the recent valid estimates, %, or None."""
        return 100.0 * sum(self.vetoes) / len(self.vetoes) if self.vetoes else None

    def step(self, stamp_s, est, T_filter_cam, holding=False, why=''):
        """One frame. est: (T_cam_object 4x4, valid, quality) from the
        estimator, or None (none this frame; why says why). T_filter_cam: the
        camera in the filter frame at the image stamp, or None (no TF).
        Returns (raw, pose, check): raw, the T_cam_object to publish, or None;
        pose, (t, q) in the filter frame, or None; check, 'ok' or why there
        is no raw this frame."""
        if holding:
            self.reset()
            return None, None, 'HELD'
        if self.kf.initialized and self.last_stamp is not None:
            self.kf.predict(stamp_s - self.last_stamp)
        if self.last_accept is not None and stamp_s - self.last_accept > self.max_prediction_s:
            self.reset()                                    # silent too long: lost
        self.last_stamp = stamp_s

        check = self._check(est, T_filter_cam, why)
        if check is None:
            t, q = pose7(T_filter_cam @ est[0])
            ok = self.kf.update(t, q)
            if not ok:
                self.rejects += 1
                check = f'KF gate ({self.rejects} in a row)'
                if not self.acquired or self.rejects >= self.rejects_before_reacquire:
                    # Not acquired yet, or a persistent disagreement (the part
                    # really moved): restart the track from this estimate.
                    self.reset()
                    self.last_stamp = stamp_s
                    ok = self.kf.update(t, q)
            if ok:
                self.rejects = 0
                self.agreeing += 1
                self.last_accept = stamp_s
                if self.acquired:
                    return est[0], (self.kf.position, self.kf.quaternion), 'ok'
                return None, None, f'acquiring {self.agreeing}/{self.acquire_frames}'
        if self.acquired:                   # coast on the prediction, never as raw
            return None, (self.kf.position, self.kf.quaternion), check
        if self.agreeing:                   # acquiring: the count pauses
            return None, None, f'{check} (acquiring {self.agreeing}/{self.acquire_frames})'
        return None, None, check

    def _check(self, est, T_filter_cam, why):
        """None if est may update the KF, else why not."""
        if est is None:
            return why or 'no estimate'
        T, valid, q = est
        mm, tilt, inp = q.get('agree_mm'), q.get('agree_tilt_deg'), q.get('agree_inplane_deg')
        known = mm is not None and tilt is not None and inp is not None
        if T is not None and valid and not (known and np.all(np.isfinite(T))
                                            and np.isfinite([mm, tilt, inp]).all()):
            return 'depth invalid: not finite'              # NaN passes > and the KF gate
        vetoed = known and (mm > self.check_mm or tilt > self.check_tilt_deg
                            or abs(inp) > self.check_inplane_deg)
        why_veto = (f'vetoed by the marker: {mm:.1f} mm, tilt {tilt:.1f} deg, '
                    f'in-plane {inp:+.1f} deg' if vetoed else '')
        if T is None or not valid:
            if not vetoed:
                return f"depth invalid: {q.get('reason') or '?'}"
            # A gross disagreement fails the estimator's own shift gate
            # too: it still counts against the depth in the veto rate.
            self.vetoes.append(True)
            return f"{why_veto} (and depth invalid: {q.get('reason') or '?'})"
        self.vetoes.append(vetoed)
        if vetoed:
            return why_veto
        if T_filter_cam is None:
            return 'no TF at the image stamp'
        return None
