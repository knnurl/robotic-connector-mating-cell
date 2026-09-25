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
visual preset name, '' = device default), capture_spatial_filter (bool) and
capture_exposure_us (-1 = auto), besides capture_width/height/fps.
"""

import datetime
import pathlib
import threading

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from roscam.cam_pub import ArucoPosePublisher
from roscam.connector_pose import ConnectorPoseNode
from roscam.frame_recorder import FrameRecorder
from roscam.rs_capture import RsCapture


def handle_frame(aruco, frame, recorder):
    """One frame: detect and publish (drawing on a copy), then record the
    untouched frame with the poses published for it."""
    header = aruco.self_stamped_header()
    poses = {}
    aruco.on_publish = lambda topic, hdr, t, q: poses.__setitem__(
        topic, (hdr.stamp.sec + hdr.stamp.nanosec * 1e-9, t, q))
    aruco.process_frame(frame.bgr.copy(), header, depth_m=frame.depth_m)
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
    aruco.declare_parameter('capture_preset', '')
    aruco.declare_parameter('capture_spatial_filter', False)
    aruco.declare_parameter('capture_exposure_us', -1)
    capture = RsCapture(
        width=int(aruco.get_parameter('capture_width').value),
        height=int(aruco.get_parameter('capture_height').value),
        fps=int(aruco.get_parameter('capture_fps').value),
        enable_depth=want_depth,
        preset=str(aruco.get_parameter('capture_preset').value),
        spatial_filter=bool(aruco.get_parameter('capture_spatial_filter').value),
        exposure_us=int(aruco.get_parameter('capture_exposure_us').value))
    capture.on_event = lambda m: aruco.get_logger().warn(m)
    intr = capture.start()
    aruco.set_intrinsics(intr.fx, intr.fy, intr.cx, intr.cy, intr.coeffs)
    if icp_node is not None:
        icp_node.set_intrinsics(intr.fx, intr.fy, intr.cx, intr.cy)
    recorder = FrameRecorder()
    recorder_params(aruco, capture, recorder)
    aruco.get_logger().info(f'capture settings: {capture.settings()}')
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
        handle_frame(aruco, frame, recorder)
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
