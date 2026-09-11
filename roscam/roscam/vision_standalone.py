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

NOT wired into any launch file by default - the topic-mode camera-driver
wiring keeps working untouched. Switch by running this instead of the
camera driver + cam_pub pair.
"""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from roscam.cam_pub import ArucoPosePublisher
from roscam.connector_pose import ConnectorPoseNode
from roscam.rs_capture import RsCapture


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
    capture = RsCapture(
        width=int(aruco.get_parameter('capture_width').value),
        height=int(aruco.get_parameter('capture_height').value),
        fps=int(aruco.get_parameter('capture_fps').value),
        enable_depth=want_depth)
    capture.on_event = lambda m: aruco.get_logger().warn(m)
    intr = capture.start()
    aruco.set_intrinsics(intr.fx, intr.fy, intr.cx, intr.cy, intr.coeffs)
    if icp_node is not None:
        icp_node.set_intrinsics(intr.fx, intr.fy, intr.cx, intr.cy)
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

    try:
        while rclpy.ok():
            frame = capture.wait_frame(timeout_s=1.0)
            if frame is None:
                aruco.get_logger().warn('RealSense frame timeout',
                                        throttle_duration_sec=5.0)
                continue
            aruco.process_frame(frame.bgr, aruco.self_stamped_header(),
                                depth_m=frame.depth_m)
            if icp_node is not None and frame.depth_m is not None:
                icp_node.process_depth(frame.depth_m,
                                       icp_node.self_stamped_header())
    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()
        executor.shutdown()
        aruco.destroy_node()
        if icp_node is not None:
            icp_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
