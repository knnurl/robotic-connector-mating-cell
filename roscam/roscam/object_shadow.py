#!/usr/bin/env python3
"""Shadow mode (PERCEPTION_PLAN Phase 3): the depth estimator runs beside the
marker on every frame and only reports. The marker still drives.

ROS-free. vision_standalone calls step() after cam_pub's process_frame, on
the camera's own colour frame, with the marker pose that frame published:

  seed    marker o T_marker_object (the part file), when the marker gave a
          raw pose this frame. No marker, no estimate: the agreement is the
          point of this phase. The Phase 2 replay put the estimate within
          ~0.5 mm and ~1 deg of the marker, so the prior is taken as good to
          MARKER_PRIOR_ERR_M, which puts it on the estimator's fast path
          (surface, then colour edges) from ~100 mm out.
  held    while /grip_node/status says holding, the part rides in the
          fingers: nothing is estimated (reason HELD).
  budget  the frame loop stamps a frame when it picks it up, so a loop that
          overruns the camera's frame period stamps the next frame late -
          and TRACK looks the arm up at that stamp. A frame with less of its
          period left than the last estimate took is skipped, never two in a
          row, so the loop catches up. For the same reason there is no
          depth-outline fallback: where the colour outline is not found
          (motion blur), finishing from depth took 55-90 ms in all and was
          most of the replay's overruns; the frame is reported instead.

step() returns the keys the contract adds to /object/pose_quality
(PERCEPTION_PLAN section 2), as strings; cand_pose is filled whenever an
estimate was computed, valid or not, so a shadow bag carries the per-axis
data. draw_outline() puts the last estimate's silhouette on the debug image,
one frame stale: the debug image goes out inside process_frame, before this
frame's estimate exists.
"""

import cv2
import numpy as np

from roscam.object_pose import CppObjectPoseEstimator, ObjectPoseEstimator

MARKER_PRIOR_ERR_M = 0.0015
VALID_BGR, INVALID_BGR = (0, 200, 0), (0, 140, 255)


def pose_matrix(t, q):
    """4x4 from a translation and a quaternion (x, y, z, w)."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = np.asarray(t, dtype=float).ravel()
    return T


class DepthShadow:
    """impl: 'cpp' (object_pose_cpp, the same answers several times faster)
    or 'python' (roscam/object_pose.py, the reference)."""

    def __init__(self, part, impl='python', **estimator_kwargs):
        cls = {'cpp': CppObjectPoseEstimator, 'python': ObjectPoseEstimator}[impl]
        self.impl = impl
        self.est = cls(part, **{'depth_fallback': False, **estimator_kwargs})
        self.T_mo = np.asarray(part['T_marker_object'], dtype=float)
        self.holding = False
        self.last = None            # (T_cam_object, valid) of the last frame, or None
        self.result = None          # this frame's (T_cam_object, valid, quality), or None
        self.last_ms = None         # the last estimate's compute time
        self._rested = True         # the last frame ran no estimate
        # Each mesh edge's two end points (edge_pts holds each edge's points
        # together, in edge order): the silhouette is drawn edge by edge.
        eid = self.est.edge_id
        n = int(eid.max()) + 1
        self._ends = (self.est.edge_pts[np.searchsorted(eid, np.arange(n))],
                      self.est.edge_pts[np.searchsorted(eid, np.arange(n), side='right') - 1])

    def warm(self, K, dist, shape):
        """Build the estimator's per-camera pixel grid now (~35 ms on an
        E-core), not inside the first frame it estimates."""
        self.est.warm(K, dist, shape)

    def step(self, depth_m, bgr_clean, K, dist, T_cam_marker, budget_ms=None):
        """One frame. T_cam_marker: the raw marker pose this frame published
        (4x4, optical frame) or None; budget_ms: what is left of the frame
        period, or None for no limit. Returns the quality keys."""
        out = {'holding': 'true' if self.holding else 'false'}
        skip = None
        if self.holding:
            skip = 'HELD'
        elif T_cam_marker is None:
            skip = 'no marker prior'
        elif depth_m is None:
            skip = 'no depth'
        elif (budget_ms is not None and self.last_ms is not None and not self._rested
              and self.last_ms > budget_ms):
            skip = f'budget: {budget_ms:.0f} ms left, the last took {self.last_ms:.0f}'
        if skip is not None:
            self.last, self.result, self._rested = None, None, True
            out.update(seeded_from='none', depth_valid='false', depth_reason=skip)
            return out

        T, valid, q = self.est.process(depth_m, bgr_clean, K, dist, T_cam_marker @ self.T_mo,
                                       prior_err_m=MARKER_PRIOR_ERR_M)
        self.last = None if T is None else (T, bool(valid))
        self.result = (T, bool(valid), q)
        self.last_ms, self._rested = q['compute_ms'], False
        out.update(seeded_from='marker', depth_valid='true' if valid else 'false',
                   depth_reason=q['reason'], depth_ms=f"{q['compute_ms']:.1f}",
                   n_pts=str(q['n_pts']), weak_dof=','.join(q['weak_dof']),
                   sym_index=str(q['sym_index']), edge_source=q['edge_source'] or '')
        for k in ('rms_mm', 'inlier_frac', 'agree_mm', 'agree_deg', 'agree_tilt_deg',
                  'agree_inplane_deg'):
            if q.get(k) is not None:
                out[k] = f'{q[k]:.3f}'
        if q['cand_pose'] is not None:
            out['cand_pose'] = ','.join(f'{v:.6f}' for v in q['cand_pose'])
        return out

    def draw_outline(self, img, K, dist):
        """The last estimate's silhouette onto img (the debug copy): green
        when it passed the gates, orange when not."""
        if self.last is None:
            return
        T, valid = self.last
        R, t = T[:3, :3], T[:3, 3]
        facing = self.est._facing(R, t)
        f0, f1 = self.est.edge_faces[:, 0], self.est.edge_faces[:, 1]
        a = facing[f0]
        b = np.where(f1 >= 0, facing[np.maximum(f1, 0)], False)
        sil = (a != b) | (f1 == -2)
        if not sil.any():
            return
        P = np.concatenate([self._ends[0][sil], self._ends[1][sil]]) @ R.T + t
        if np.any(P[:, 2] <= 1e-3):
            return
        uv = cv2.projectPoints(P, np.zeros(3), np.zeros(3), K,
                               np.zeros(5) if dist is None else dist)[0].reshape(-1, 2)
        n = int(sil.sum())
        lines = np.round(np.stack([uv[:n], uv[n:]], 1) * 16).astype(np.int32)
        cv2.polylines(img, list(lines), False, VALID_BGR if valid else INVALID_BGR, 1,
                      cv2.LINE_AA, shift=4)


ACQUIRE_BGR = (255, 200, 0)                     # cyan: the marker-free acquisition


class AcquireShadow:
    """Phase 6 in shadow: ObjectPoseEstimator.acquire() (no prior, no marker)
    on the newest frame at most every period_s, in a background thread with
    its own estimator, so the frame loop only pays for a copy of the frame
    (the C++ refinement releases the GIL). It never drives anything: it
    reports its agreement with the same frame's marker (quality() keys
    acquire_*) and draws its outline in cyan on the debug image. Nothing is
    acquired while GRIP holds the part (holding)."""

    def __init__(self, part, impl='python', period_s=1.0, log=None):
        import threading
        cls = {'cpp': CppObjectPoseEstimator, 'python': ObjectPoseEstimator}[impl]
        self.est = cls(part, depth_fallback=False)
        self.T_mo = np.asarray(part.get('T_marker_object', np.eye(4)), dtype=float)
        self.sym = int(part.get('sym_order', 1))
        self.period_s = float(period_s)
        self.log = log
        self.holding = False
        self.last = None                 # dict of the newest result
        self._job = None
        self._next_t = 0.0
        self._cv = threading.Condition()
        self._stop = False
        self._thread = threading.Thread(target=self._run, name='acquire_shadow', daemon=True)
        self._thread.start()

    def close(self):
        with self._cv:
            self._stop = True
            self._cv.notify()

    def offer(self, now_s, depth_m, bgr_clean, K, dist, T_cam_marker):
        """Hand this frame over if one is due and none is in hand. Cheap."""
        if self.holding or depth_m is None or now_s < self._next_t:
            return False
        with self._cv:
            if self._job is not None:
                return False
            self._job = (now_s, np.array(depth_m, copy=True),
                         None if bgr_clean is None else np.array(bgr_clean, copy=True),
                         np.array(K, dtype=float), None if dist is None else np.array(dist),
                         None if T_cam_marker is None else np.array(T_cam_marker))
            self._next_t = now_s + self.period_s
            self._cv.notify()
        return True

    def _run(self):
        import time
        from roscam.object_pose import pose_error
        while True:
            with self._cv:
                while self._job is None and not self._stop:
                    self._cv.wait()
                if self._stop:
                    return
                stamp, depth, bgr, K, dist, T_marker = self._job
            t0 = time.perf_counter()
            try:
                T, valid, q = self.est.acquire(depth, bgr, K, dist)
                out = {'stamp': stamp, 'T': T, 'valid': bool(valid), 'reason': q['reason'],
                       'n': len(q.get('candidates', [])),
                       'ms': (time.perf_counter() - t0) * 1e3, 'K': K, 'dist': dist}
                if valid and T_marker is not None:
                    d, tilt, yaw = pose_error(T, T_marker @ self.T_mo, self.sym)
                    out.update(mm=float(np.linalg.norm(d)) * 1e3, tilt=float(tilt),
                               yaw=float(yaw))
            except Exception as e:                          # noqa: BLE001
                out = {'stamp': stamp, 'T': None, 'valid': False, 'reason': f'error: {e!r}',
                       'n': 0, 'ms': (time.perf_counter() - t0) * 1e3}
                if self.log is not None:
                    self.log(f'acquire shadow failed: {e!r}')
            self.last = out
            with self._cv:
                self._job = None

    def quality(self, now_s):
        """The newest result as /object/pose_quality keys (strings)."""
        r = self.last
        if r is None:
            return {}
        out = {'acquire_valid': 'true' if r['valid'] else 'false',
               'acquire_reason': r['reason'], 'acquire_age_s': f"{now_s - r['stamp']:.2f}",
               'acquire_ms': f"{r['ms']:.0f}", 'acquire_n': str(r['n'])}
        for k in ('mm', 'tilt', 'yaw'):
            if r.get(k) is not None:
                out[f'acquire_{k}'] = f'{r[k]:.2f}'
        return out

    def draw(self, img, K, dist, now_s, max_age_s=2.0):
        """The last acquisition's outline (cyan) while it is recent."""
        r = self.last
        if r is None or r['T'] is None or now_s - r['stamp'] > max_age_s:
            return
        T = r['T']
        P = self.est.V @ T[:3, :3].T + T[:3, 3]
        if np.any(P[:, 2] <= 1e-3):
            return
        uv = cv2.projectPoints(P, np.zeros(3), np.zeros(3), K,
                               np.zeros(5) if dist is None else dist)[0].reshape(-1, 2)
        hull = cv2.convexHull(np.round(uv * 16).astype(np.int32))
        cv2.polylines(img, [hull], True, ACQUIRE_BGR, 2, cv2.LINE_AA, shift=4)
