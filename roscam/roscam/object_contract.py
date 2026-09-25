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
                        (source, valid, compute_ms, ...).

/aruco/* stays as it is (hand-eye calibration, teach_offsets, target B, the
ground truth). Sources, chosen by the vision node's object_source parameter:

  marker  every raw, filtered and predicted marker publish, mirrored
          unchanged (T_marker_object is identity until Phase 3), so the
          /object/* stream is bit-identical to /aruco/*.

Both camera owners attach it (cam_pub for vision_source realsense or topic,
vision_standalone for standalone), so rolling back the camera owner never
leaves the consumers without their topics.
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header

POSE_TOPIC = '/object/pose'
RAW_POSE_TOPIC = '/object/pose_raw'
QUALITY_TOPIC = '/object/pose_quality'
SOURCES = ('marker',)

_MARKER_TO_OBJECT = {'/aruco/pose_raw': 'raw', '/aruco/pose': 'pose'}


class ObjectContract:
    """Attach to an ArucoPosePublisher: declares object_source and publishes
    the /object/* topics through the node's publish and frame hooks."""

    def __init__(self, node):
        self.node = node
        node.declare_parameter('object_source', 'marker')
        self.source = str(node.get_parameter('object_source').value)
        if self.source not in SOURCES:
            raise RuntimeError(f"object_source must be one of {SOURCES}, got '{self.source}'")
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
                if p.value not in SOURCES:
                    return SetParametersResult(
                        successful=False, reason=f'object_source must be one of {SOURCES}')
                self.source = p.value
        return SetParametersResult(successful=True)

    def _on_publish(self, topic, header, t, q):
        """A marker publish: mirrored while the source is the marker."""
        kind = _MARKER_TO_OBJECT.get(topic)
        if kind is None or self.source != 'marker':
            return
        out = PoseStamped()
        out.header = header
        (out.pose.position.x, out.pose.position.y, out.pose.position.z) = map(float, t)
        (out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z,
         out.pose.orientation.w) = map(float, q)
        self.pubs[kind].publish(out)
        if kind == 'raw':
            self._raw_this_frame = True

    def _on_frame(self, header, compute_ms):
        """One quality status per processed frame."""
        valid = self._raw_this_frame
        self._raw_this_frame = False
        st = DiagnosticStatus()
        st.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        st.name = 'object_pose'
        st.hardware_id = self.source
        st.message = '' if valid else 'no raw pose this frame'
        st.values = [KeyValue(key='source', value=self.source),
                     KeyValue(key='valid', value='true' if valid else 'false'),
                     KeyValue(key='compute_ms', value=f'{compute_ms:.1f}')]
        arr = DiagnosticArray()
        arr.header = Header(stamp=header.stamp, frame_id=header.frame_id)
        arr.status = [st]
        self.quality_pub.publish(arr)
