#!/usr/bin/env python3
"""Single-process vision stack: camera -> poses, zero image topics.

One process owns the RealSense (a device allows only one pipeline) and runs
BOTH vision pipelines on the same frames, entirely outside the DDS graph:

    RsCapture ──bgr──▶ ArucoPosePublisher.process_frame ──▶ /aruco/pose[_raw]
        └─────depth──▶ ConnectorPoseNode.process_depth  ──▶ /connector/pose

Only poses (~2 KB/s) and the throttled, subscribe-gated debug image ever
touch ROS - the ~13-45 MB/s of raw colour/depth stays in this process, so
nothing can contend with the FR3's 1 kHz FCI loop. This is the "camera feed
outside ROS 2, only useful end-data in" pattern as one executable.

The ICP half activates only when a template is given; without it this is a
marker-only tracker (drop-in replacement for `cam_pub source:=realsense`):

    ros2 run roscam vision_standalone                       # marker only
    ros2 run roscam vision_standalone --ros-args \\
        -p template_stl:=/path/connector.stl \\
        -p marker_t_connector_xyz:="[0.05, 0.0, 0.0]" \\
        -p filter_frame:=fr3_link0                          # marker + ICP

All cam_pub / connector_pose parameters apply (they are the same nodes,
fed externally). `source` is forced to 'external' on both.

fr3_cell.launch.py runs it with vision_source:=standalone (PERCEPTION_PLAN
Phase 0); vision_source:=realsense (cam_pub's own capture) stays the rollback.

Recording (Phase 0): set the parameter record_dir to a directory and every
frame goes to disk (frame_recorder.py: lossless colour, raw depth,
timestamps, and the poses published for that frame); set it to '' to stop.
The panel's REC button does this beside its bag. process_frame draws its
debug overlay on a COPY, so the recorded frame (and later the estimator's)
is the camera's own.

Capture settings for the depth-quality comparison: capture_preset (a D405
visual preset name, '' = device default), capture_spatial_filter (bool),
capture_exposure_us (-1 = auto) and capture_gain (-1 = the device's),
besides capture_width/height/fps. Exposure and gain change live (ros2 param
set), and every recorded frame carries the exposure and gain it was shot
with.

The depth estimator (needs object_part), on the untouched frame after the
marker and seeded from it (object_shadow.py), adds its estimate and its
agreement with the marker to that frame's /object/pose_quality, and the
debug image shows the previous frame's estimated outline:
  - shadow mode (PERCEPTION_PLAN Phase 3): object_shadow true (live), with
    object_source marker; the marker still drives;
  - object_source depth_checked (Phase 4, depth_checked.py): the estimate
    drives /object/*, and the same frame's marker can veto it.
"""

import datetime
import pathlib
import threading
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from roscam.cam_pub import ArucoPosePublisher
from roscam.connector_pose import ConnectorPoseNode
from roscam.frame_recorder import FrameRecorder
from roscam.depth_checked import DepthChecked, pose7
from roscam.object_contract import ObjectContract
from roscam.object_shadow import DepthShadow, pose_matrix
from roscam.rs_capture import RsCapture

GRIP_STATUS_TOPIC = '/grip_node/status'


class DepthRunner:
    """The depth estimator in this process (object_shadow.DepthShadow),
    seeded from the marker on each frame it runs:

      shadow mode    object_shadow true, object_source marker: it only
                     reports (PERCEPTION_PLAN Phase 3);
      depth_checked  object_source depth_checked: it drives /object/*, and
                     the same frame's marker can veto it (depth_checked.py,
                     Phase 4). It runs whatever object_shadow says.

    Also wired here: /grip_node/status (holding), the frame budget, the
    contract's per-frame quality (out after the estimate), and a source
    switch, which restarts the depth track in the frame loop's own thread."""

    MARGIN_MS = 5.0             # after the estimate: the recorder hand-off, the quality

    def __init__(self, aruco, contract):
        self.aruco, self.contract = aruco, contract
        contract.hold_quality = True
        # 'cpp': the estimator in object_pose_cpp (colcon-built, the same
        # answers several times faster); 'python': roscam/object_pose.py.
        aruco.declare_parameter('object_pose_impl', 'cpp')
        self.depth = None
        if contract.part is not None:
            impl = str(aruco.get_parameter('object_pose_impl').value)
            if impl not in ('cpp', 'python'):
                raise RuntimeError(f"object_pose_impl must be cpp|python, got '{impl}'")
            try:
                self.depth = DepthShadow(contract.part, impl=impl)
            except ImportError:
                aruco.get_logger().error(
                    'object_pose_cpp is not built (colcon build --packages-select '
                    'object_pose_cpp) - the Python estimator instead: the same poses, slower')
                self.depth = DepthShadow(contract.part, impl='python')
        aruco.declare_parameter('object_shadow', False)
        # The marker's veto in depth_checked (PERCEPTION_PLAN Phase 4). Set at
        # launch only: widening it under a running TRACK would loosen the
        # one check depth answers to without restarting the track. The tilt
        # bound is looser: the marker's own tilt is the noisier.
        aruco.declare_parameter('depth_check_mm', 3.0)
        aruco.declare_parameter('depth_check_tilt_deg', 4.0)
        aruco.declare_parameter('depth_check_inplane_deg', 2.0)
        self.enabled = bool(aruco.get_parameter('object_shadow').value)
        if self.enabled and self.depth is None:
            aruco.get_logger().error('object_shadow needs object_part (a part file) - off')
            self.enabled = False
        self.checked = None
        if self.depth is not None:
            prm = lambda n: float(aruco.get_parameter(n).value)          # noqa: E731
            # The object KF takes the marker filter's settings: /object/pose
            # behaves as with the marker, only the measurement changes.
            self.checked = DepthChecked(
                check_mm=prm('depth_check_mm'), check_tilt_deg=prm('depth_check_tilt_deg'),
                check_inplane_deg=prm('depth_check_inplane_deg'),
                max_prediction_s=aruco.max_prediction_s,
                rejects_before_reacquire=aruco.rejects_before_reacquire,
                sigma_accel=prm('sigma_accel'), sigma_rot_rate_deg=prm('sigma_rot_rate_deg'),
                meas_std_pos=prm('meas_std_pos'), meas_std_rot_deg=prm('meas_std_rot_deg'),
                gate_sigma=prm('gate_sigma'))
        self._switches = contract.switches
        self.period_ms = 1000.0 / max(1, int(aruco.get_parameter('capture_fps').value))
        aruco.debug_hooks.append(self.draw)
        aruco.add_on_set_parameters_callback(self._on_set)
        aruco.create_subscription(
            DiagnosticStatus, GRIP_STATUS_TOPIC, self._grip_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_set(self, params):
        for prm in params:
            if prm.name == 'object_shadow':
                if prm.value and self.depth is None:
                    return SetParametersResult(successful=False,
                                               reason='object_shadow needs object_part')
                self.enabled = bool(prm.value)
                self.aruco.get_logger().info(f'shadow mode {"on" if self.enabled else "off"}')
            elif prm.name.startswith('depth_check_'):
                return SetParametersResult(successful=False,
                                           reason=f'{prm.name} is set at launch only')
        return SetParametersResult(successful=True)

    def _grip_cb(self, msg):
        if self.depth is not None:
            self.depth.holding = any(kv.key == 'holding' and kv.value == 'true'
                                     for kv in msg.values)

    def _running(self, source):
        return self.depth is not None and (self.enabled or source == 'depth_checked')

    def draw(self, img):
        """cam_pub's debug hook: the previous estimate's outline onto the
        debug image as it goes out, after this frame's markers were detected
        on clean pixels (an outline across a marker would change its
        detection whenever someone watched)."""
        if self._running(self.contract.source):
            self.depth.draw_outline(img, self.aruco.camera_matrix, self.aruco.dist_coeffs)

    def after(self, frame, header, poses, t_start):
        """This frame's estimate (shadow mode or depth_checked), the
        depth_checked decision, then the frame's quality status. A failure
        costs the estimate (and in depth_checked the track), never the
        frame's status or its recording."""
        source = self.contract.source
        if self.contract.switches != self._switches:        # a switch: restart the track
            self._switches = self.contract.switches
            if self.checked is not None:
                self.checked.reset(clear_rate=True)
        extra = None
        if self._running(source):
            raw = poses.get('/aruco/pose_raw')
            left = self.period_ms - self.MARGIN_MS - (time.perf_counter() - t_start) * 1e3
            try:
                extra = self.depth.step(frame.depth_m, frame.bgr, self.aruco.camera_matrix,
                                        self.aruco.dist_coeffs,
                                        None if raw is None else pose_matrix(raw[1], raw[2]),
                                        budget_ms=left)
                if source == 'depth_checked':
                    extra.update(self._drive(header, extra))
            except Exception as e:                          # noqa: BLE001
                self.depth.last = self.depth.result = None
                if self.checked is not None:
                    self.checked.reset()                    # no raw before a fresh acquisition
                extra = {'seeded_from': 'marker', 'depth_valid': 'false',
                         'depth_reason': f'error: {e!r}'}
                if source == 'depth_checked':
                    extra['check'] = f'error: {e!r}'
                self.aruco.get_logger().warn(f'depth estimate failed: {e!r}',
                                             throttle_duration_sec=5.0)
        self.contract.publish_quality(extra)

    def _drive(self, header, extra):
        """depth_checked: this frame's decision, published through the
        contract; returns its quality keys."""
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        raw, pose, check = self.checked.step(stamp, self.depth.result, self._T_filter_cam(),
                                             holding=self.depth.holding,
                                             why=extra.get('depth_reason', ''))
        if raw is not None:
            self.contract.publish_object('raw', header, *pose7(raw))
        if pose is not None:
            self.contract.publish_object('pose', self.aruco._pose_header(header), *pose)
        out = {'check': check, 'acquired': 'true' if self.checked.acquired else 'false',
               'check_mm': f'{self.checked.check_mm:g}',
               'check_tilt_deg': f'{self.checked.check_tilt_deg:g}',
               'check_inplane_deg': f'{self.checked.check_inplane_deg:g}'}
        veto = self.checked.veto_pct()
        if veto is not None:
            out['veto_pct'] = f'{veto:.1f}'
        return out

    def _T_filter_cam(self):
        """The camera in the filter frame at this frame's stamp: cam_pub's
        own lookup for the marker (one per frame), or None."""
        if not self.aruco.filter_frame:
            return np.eye(4)                                # filtering in the optical frame
        if self.aruco.last_tf is None:
            return None
        return pose_matrix(*self.aruco.last_tf)


def handle_frame(aruco, frame, recorder, depth=None):
    """One frame: detect and publish (the overlays go on a copy), estimate
    from depth on the untouched frame (DepthRunner), then record it with the
    poses published for it."""
    t_start = time.perf_counter()
    header = aruco.self_stamped_header()
    poses = {}
    aruco.on_publish = lambda topic, hdr, t, q: poses.__setitem__(
        topic, (hdr.stamp.sec + hdr.stamp.nanosec * 1e-9, t, q))
    aruco.process_frame(frame.bgr.copy(), header, depth_m=frame.depth_m)
    if depth is not None:
        depth.after(frame, header, poses, t_start)
    if recorder is not None and recorder.recording:
        recorder.record(frame, header.stamp.sec + header.stamp.nanosec * 1e-9, poses)


def run_frames(next_frame, handle, log, keep_going):
    """The capture loop: a failure on one frame is logged and the next frame
    still comes (as cam_pub's own capture loop does). Returns frames handled."""
    n = 0
    while keep_going():
        frame = next_frame()
        if frame is None:
            log('RealSense frame timeout')
            continue
        try:
            handle(frame)
            n += 1
        except Exception as e:                              # noqa: BLE001
            log(f'frame processing failed: {e!r}')
    return n


def recorder_params(aruco, capture, recorder):
    """record_dir: a directory starts a session in <dir>/<time>; '' stops it."""
    aruco.declare_parameter('record_dir', '')

    def on_set(params):
        for prm in params:
            if prm.name != 'record_dir':
                continue
            try:
                if prm.value:
                    session = pathlib.Path(str(prm.value)) / datetime.datetime.now().strftime(
                        'vision_%Y%m%d_%H%M%S')
                    intr = capture.intrinsics
                    recorder.start(session, {
                        'capture': capture.settings(),
                        'intrinsics': dict(intr._asdict()) if intr is not None else None,
                        'optical_frame': aruco.optical_frame_id,
                        'capture_latency_s': aruco.capture_latency,
                        'marker_id': aruco.marker_id, 'marker_size_m': aruco.marker_size,
                        'target_marker_id': aruco.target_id,
                        'target_marker_size_m': float(aruco.target_obj_points[1, 0] * 2)})
                    aruco.get_logger().info(f'recording frames -> {session}')
                else:
                    summary = recorder.stop()
                    if summary:
                        aruco.get_logger().info(
                            f'recording stopped: {summary["written"]} frames written, '
                            f'{summary["dropped"]} dropped -> {summary["dir"]}')
            except Exception as e:                          # noqa: BLE001
                return SetParametersResult(successful=False, reason=f'record_dir: {e}')
        return SetParametersResult(successful=True)
    aruco.add_on_set_parameters_callback(on_set)


def main(args=None):
    rclpy.init(args=args)
    external = [Parameter('source', value='external')]

    aruco = ArucoPosePublisher(parameter_overrides=external)
    contract = ObjectContract(aruco, estimator=True)   # /object/* (PERCEPTION_PLAN 2)
    depth = DepthRunner(aruco, contract)   # the estimator: shadow mode, depth_checked

    icp_node = None
    try:
        icp_node = ConnectorPoseNode(parameter_overrides=external)
        aruco.get_logger().info('ICP refinement active (template_stl set).')
    except RuntimeError:
        aruco.get_logger().info(
            'No template_stl - running marker-only (set template_stl to '
            'enable connector-level ICP refinement).')

    # Depth is needed by the ICP node AND by cam_pub's IPPE disambiguation
    # (tilt_disambiguation: depth), so enable it for either consumer.
    want_depth = (icp_node is not None
                  or str(aruco.get_parameter('tilt_disambiguation').value
                         ).lower() == 'depth')
    capture = RsCapture(
        width=int(aruco.get_parameter('capture_width').value),
        height=int(aruco.get_parameter('capture_height').value),
        fps=int(aruco.get_parameter('capture_fps').value),
        enable_depth=want_depth,
        preset=str(aruco.get_parameter('capture_preset').value),
        spatial_filter=bool(aruco.get_parameter('capture_spatial_filter').value),
        exposure_us=int(aruco.get_parameter('capture_exposure_us').value),
        gain=int(aruco.get_parameter('capture_gain').value))
    capture.on_event = lambda m: aruco.get_logger().warn(m)
    intr = capture.start()
    aruco.live_capture = capture          # capture_exposure_us / capture_gain apply live
    aruco.set_intrinsics(intr.fx, intr.fy, intr.cx, intr.cy, intr.coeffs)
    if depth.depth is not None:
        depth.depth.warm(aruco.camera_matrix, aruco.dist_coeffs, (intr.height, intr.width))
    if icp_node is not None:
        icp_node.set_intrinsics(intr.fx, intr.fy, intr.cx, intr.cy)
    recorder = FrameRecorder()
    recorder_params(aruco, capture, recorder)
    aruco.get_logger().info(f'capture settings: {capture.settings()}')
    aruco.get_logger().info(
        f'object sources {contract.sources}, now {contract.source}; shadow mode '
        f'{"on" if depth.enabled else "off"}'
        + (f" (part {contract.part['name']}, estimator {depth.depth.impl})"
           if depth.depth is not None else ' (no object_part)'))
    aruco.get_logger().info(
        f'Camera up: {intr.width}x{intr.height} @ '
        f'{aruco.get_parameter("capture_fps").value} fps, depth '
        f'{"on" if want_depth else "off"}'
        f'{" (ICP)" if icp_node is not None else ""}'
        f'{" (tilt disambiguation)" if want_depth and icp_node is None else ""}'
        ' - no image topics on the graph.')

    # Subscriptions (marker prior) + services need spinning; frames are
    # processed in the main thread, never in a callback.
    executor = MultiThreadedExecutor()
    executor.add_node(aruco)
    if icp_node is not None:
        executor.add_node(icp_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    def handle(frame):
        handle_frame(aruco, frame, recorder, depth)
        if icp_node is not None and frame.depth_m is not None:
            icp_node.process_depth(frame.depth_m, icp_node.self_stamped_header())

    try:
        run_frames(lambda: capture.wait_frame(timeout_s=1.0), handle,
                   lambda m: aruco.get_logger().warn(m, throttle_duration_sec=5.0),
                   rclpy.ok)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        capture.stop()
        executor.shutdown()
        aruco.destroy_node()
        if icp_node is not None:
            icp_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
