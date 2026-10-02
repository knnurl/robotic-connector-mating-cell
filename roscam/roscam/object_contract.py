#!/usr/bin/env python3
"""The object pose contract (PERCEPTION_PLAN section 2): the part's pose on
topics that do not depend on where it came from, so TRACK, GRIP and ALIGN
switch sources with a parameter, not a code change.

  /object/pose_raw      PoseStamped, optical frame, image stamp. A
                        measurement that passed every gate of the current
                        source, the filter's innovation gate included. Never
                        a prediction, prior or fallback; no message = no
                        valid pose.
  /object/pose          PoseStamped, the filter frame (fr3_link0 on the
                        cell). Filtered; may coast for max_prediction_s.
  /object/pose_quality  DiagnosticArray, one status per processed frame,
                        stamped with the image: level, reason, and keys
                        (source, valid, compute_ms, tf_wait_ms, and in
                        shadow mode the depth estimate's: object_shadow.py).

/aruco/* stays as it is (hand-eye calibration, teach_offsets, target B, the
ground truth). Sources, chosen by the vision node's object_source parameter:

  marker         every raw, filtered and predicted marker publish, composed
                 with T_marker_object from the part file (object_part), so
                 the pose is the part's frame whichever source gives it.
                 Identity (no part file, or the sticker taken as centred
                 until it is measured) is mirrored unchanged: /object/* is
                 then bit-identical to /aruco/*.
  depth_checked  the depth estimate, the marker in the same frame able to
                 veto it (PERCEPTION_PLAN Phase 4, depth_checked.py). Only
                 where the estimator runs (vision_standalone, estimator=True
                 with a part file); vision_standalone publishes it through
                 publish_object().

Switching source publishes nothing on /object/* for ACQUIRE_FRAMES frames
and at least SWITCH_GAP_S after the last raw pose (image stamps), longer
than the consumers' raw timeout (tracking_raw_timeout_s 0.25 s): they hold
across a switch, whoever made it and at any frame rate.

Both camera owners attach it (cam_pub for vision_source realsense or topic,
vision_standalone for standalone), so rolling back the camera owner never
leaves the consumers without their topics. Both read the part file the
launch passes: an unreadable one stops the vision node at start, on
purpose - a measured T_marker_object moves the pose the arm acts on, so
falling back to identity would be a silent error. vision_standalone sets
hold_quality and calls publish_quality() once its depth estimate for the
frame is in (PERCEPTION_PLAN Phase 3).
"""

import threading

import cv2
import numpy as np

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header

from roscam.depth_checked import ACQUIRE_FRAMES
from roscam.pose_kf import compose_pose, quat_from_rotvec

POSE_TOPIC = '/object/pose'
RAW_POSE_TOPIC = '/object/pose_raw'
QUALITY_TOPIC = '/object/pose_quality'
SOURCES = ('marker', 'depth_checked')
SWITCH_GAP_S = 0.34

_MARKER_TO_OBJECT = {'/aruco/pose_raw': 'raw', '/aruco/pose': 'pose'}


class ObjectContract:
    """Attach to an ArucoPosePublisher: declares object_source and publishes
    the /object/* topics through the node's publish and frame hooks.
    estimator: this process runs the depth estimator (vision_standalone),
    so depth_checked is on offer when there is a part file."""

    def __init__(self, node, estimator=False):
        self.node = node
        # The part file (tools/fr3/parts/*.yaml): T_marker_object, and the
        # mesh for the shadow estimator. '' = none (T_marker_object identity).
        node.declare_parameter('object_part', '')
        path = str(node.get_parameter('object_part').value)
        self.part = None
        self._T_mo = None           # (t, q) of T_marker_object, None = identity
        if path:
            from roscam.object_pose import load_part     # scipy: only with a part file
            try:
                self.part = load_part(path)
            except Exception as e:                          # noqa: BLE001
                # Loud, not silent identity: a measured T_marker_object
                # moves the pose TRACK and GRIP act on.
                raise RuntimeError(f'object_part {path}: {e!r}') from e
            T_mo = self.part['T_marker_object']
            if not np.array_equal(T_mo, np.eye(4)):
                self._T_mo = (T_mo[:3, 3].copy(),
                              quat_from_rotvec(cv2.Rodrigues(T_mo[:3, :3])[0].ravel()))
        self.sources = SOURCES if estimator and self.part is not None else ('marker',)
        node.declare_parameter('object_source', 'marker')
        self.source = str(node.get_parameter('object_source').value)
        if self.source not in self.sources:
            raise RuntimeError(
                f"object_source must be one of {self.sources}, got '{self.source}'")
        self.switches = 0           # source changes so far (the estimator's owner restarts on it)
        self._gap = 0               # frames left with nothing on /object/* after a switch
        self._gap_until = 0.0       # ... and no image stamp before this (s)
        self._last_raw_s = None     # image stamp of the last raw pose out (s)
        self._lock = threading.Lock()                      # the switch vs the frame loop
        self.hold_quality = False   # True: the frame's status waits for publish_quality()
        self._pending = None
        self.pubs = {'raw': node.create_publisher(PoseStamped, RAW_POSE_TOPIC, 10),
                     'pose': node.create_publisher(PoseStamped, POSE_TOPIC, 10)}
        self.quality_pub = node.create_publisher(DiagnosticArray, QUALITY_TOPIC, 10)
        self._raw_this_frame = False
        node.publish_hooks.append(self._on_publish)
        node.frame_hooks.append(self._on_frame)
        node.add_on_set_parameters_callback(self._on_set)

    def _on_set(self, params):
        for p in params:
            if p.name == 'object_source':
                if p.value not in self.sources:
                    reason = (f'{p.value} needs the depth estimator: vision_source:=standalone '
                              'with a part file (object_part)' if p.value in SOURCES
                              else f'object_source must be one of {self.sources}')
                    return SetParametersResult(successful=False, reason=reason)
                with self._lock:
                    if p.value != self.source:
                        self.source = p.value
                        self.switches += 1
                        self._gap = ACQUIRE_FRAMES
                        self._gap_until = (self._last_raw_s or 0.0) + SWITCH_GAP_S
        return SetParametersResult(successful=True)

    def _on_publish(self, topic, header, t, q):
        """A marker publish: mirrored while the source is the marker."""
        kind = _MARKER_TO_OBJECT.get(topic)
        if kind is None or self.source != 'marker':
            return
        if self._T_mo is not None:
            t, q = compose_pose(t, q, *self._T_mo)
        self.publish_object(kind, header, t, q)

    def publish_object(self, kind, header, t, q):
        """/object/pose_raw ('raw') or /object/pose ('pose'), unless a source
        switch is still in its gap."""
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        with self._lock:
            if self._gap > 0 or stamp < self._gap_until:
                return
            if kind == 'raw':
                self._last_raw_s = stamp
        out = PoseStamped()
        out.header = header
        (out.pose.position.x, out.pose.position.y, out.pose.position.z) = map(float, t)
        (out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z,
         out.pose.orientation.w) = map(float, q)
        self.pubs[kind].publish(out)
        if kind == 'raw':
            self._raw_this_frame = True

    def _on_frame(self, header, compute_ms):
        """One quality status per processed frame (held, if hold_quality,
        until publish_quality adds the estimator's keys)."""
        pending = (header, compute_ms, getattr(self.node, 'last_tf_wait_ms', None))
        if self.hold_quality:
            self._pending = pending
        else:
            self._publish_quality(*pending)

    def publish_quality(self, extra=None):
        """The held status of the last processed frame, with extra keys
        ({key: str}); nothing if no frame is waiting."""
        pending, self._pending = self._pending, None
        if pending is not None:
            self._publish_quality(*pending, extra=extra)

    def _publish_quality(self, header, compute_ms, tf_wait_ms, extra=None):
        """The frame's status; valid = a raw pose went out for it. It ends
        the frame: a switch gap counts down here."""
        valid = self._raw_this_frame
        self._raw_this_frame = False
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        with self._lock:
            gap = self._gap
            self._gap = max(0, gap - 1)
            if stamp < self._gap_until:
                gap = max(gap, 1)
        st = DiagnosticStatus()
        st.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        st.name = 'object_pose'
        st.hardware_id = self.source
        st.message = ('' if valid else f'source switch: {gap} frame(s) without a pose' if gap
                      else 'no raw pose this frame')
        st.values = [KeyValue(key='source', value=self.source),
                     KeyValue(key='valid', value='true' if valid else 'false'),
                     KeyValue(key='compute_ms', value=f'{compute_ms:.1f}')]
        if tf_wait_ms is not None:
            st.values.append(KeyValue(key='tf_wait_ms', value=f'{tf_wait_ms:.1f}'))
        st.values += [KeyValue(key=k, value=v) for k, v in (extra or {}).items()]
        arr = DiagnosticArray()
        arr.header = Header(stamp=header.stamp, frame_id=header.frame_id)
        arr.status = [st]
        self.quality_pub.publish(arr)
