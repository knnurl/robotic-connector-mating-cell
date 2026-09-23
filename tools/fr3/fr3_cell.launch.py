"""Connector-mating stack for a Franka FR3 - the CELL side only.

Hand-eye static TF, ArUco vision (in-process D405 capture, filtered in
fr3_link0) and the tracking node, all with tools/fr3/fr3_params.yaml. The
ROBOT side (FCI driver + move_group) is launched separately; the bring-up
order is GUIDE.md section 2, the only maintained copy.

The zero-argument launch is the panel-driven one: the tracking node comes up
idle and publishes nothing until an operator presses TRACK on
tools/fr3/cell_panel.py. The autonomous phase machine (mating_node) moves the
arm by itself, so it is opt-in: start_mating_node:=true.

vision_source:=topic instead expects a separate realsense2_camera driver
(tools/fr3/realsense_low_bw.yaml) and gives cam_pub colour only - no depth,
so the depth tilt and mirror disambiguation fall back to IPPE.

Tracking traces go to $FR3_LOG_DIR/YYYY-MM-DD/ when fr3_env.sh has set it.
"""

import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    params_file = LaunchConfiguration('params_file')
    filter_frame = LaunchConfiguration('filter_frame')
    vision_source = LaunchConfiguration('vision_source')
    start_mating_node = LaunchConfiguration('start_mating_node')
    marker_id = LaunchConfiguration('marker_id')
    capture_fps = LaunchConfiguration('capture_fps')

    args = [
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.3',
                              description='FR3 FCI IP (only used to render the URDF)'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(THIS_DIR, 'fr3_params.yaml')),
        DeclareLaunchArgument('filter_frame', default_value='fr3_link0',
                              description="KF frame for cam_pub ('' = optical/legacy)"),
        # mating_node is the autonomous phase machine: given a marker it
        # aligns the arm by itself. That is deliberate for a mating run and
        # wrong for panel-driven work, where every motion is behind a button.
        # It also needs a working MoveIt, which this machine does not have.
        # The default is the documented cell marker (SETUP_AND_CALIBRATION
        # 2.1: DICT_6X6_250 id 11, 21 mm). Override it when the connector
        # carries a different one - cam_pub decodes every marker it sees and
        # then DISCARDS any whose id does not match, so a mismatch looks
        # exactly like "no marker": debug_image flows, /aruco/pose is silent.
        DeclareLaunchArgument('marker_id', default_value='11',
                              description='ArUco id to track (DICT_6X6_250)'),
        DeclareLaunchArgument('start_mating_node', default_value='false',
                              description='start the autonomous phase '
                                          'machine (needs MoveIt; it moves '
                                          'the arm on its own)'),
        DeclareLaunchArgument('vision_source', default_value='realsense',
                              description='realsense = in-process capture with depth, no '
                                          'image topics (default); topic = a separate camera '
                                          'driver publishes colour only'),
        DeclareLaunchArgument('capture_fps', default_value='15',
                              description='D405 capture rate in realsense mode; '
                                          '/aruco/pose is published at this rate'),
        # Hand-eye TCP->optical transform. MEASURED 2026-09-15 with
        # roscam.handeye_calib: 21 poses, Tsai selected (all four solvers
        # agreed to 0.05 mm / 0.01 deg), residual 3.17 mm / 1.57 deg.
        #
        # The previous defaults were a dry-run guess and the ROTATION was
        # wrong by 89.94 deg: it assumed identity, but the camera is mounted
        # rotated ~90 deg about the optical axis, so camera X/Y were
        # effectively swapped for anything that trusted this transform. The
        # translation guess was close (13 mm out); the rotation was not.
        #
        # Re-run handeye_calib if the bracket is reprinted or reseated - see
        # hardware/camera_mount/. Cross-checks: the camera optical axis comes
        # out 0.37 deg off TCP Z, consistent with a mount designed to look
        # straight down the tool axis, and |translation| = 78 mm is plausible
        # for that bracket's envelope.
        DeclareLaunchArgument('handeye_xyz',
                              default_value='0.061126 -0.011144 -0.046550'),
        DeclareLaunchArgument('handeye_quat',
                              default_value=('0.000855 0.003126 '
                                             '0.706706 0.707500'),
                              description='qx qy qz qw'),
    ]

    # Robot model for the controller's MoveGroupInterface (same xacro the
    # upstream moveit.launch.py renders; ros2_control content is inert here).
    franka_xacro = [FindPackageShare('franka_description'), '/robots/fr3/']
    robot_description = {'robot_description': ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', *franka_xacro, 'fr3.urdf.xacro',
        ' hand:=true ros2_control:=true',
        ' robot_ip:=', robot_ip,
        ' use_fake_hardware:=', use_fake_hardware,
    ]), value_type=str)}
    robot_description_semantic = {'robot_description_semantic': ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', *franka_xacro, 'fr3.srdf.xacro',
        ' hand:=true',
    ]), value_type=str)}
    kinematics = {'robot_description_kinematics': {'fr3_arm': {
        'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_search_resolution': 0.005,
        'kinematics_solver_timeout': 0.05,
    }}}

    handeye_reminder = LogInfo(msg=(
        'fr3_mating: hand-eye TF is the launch-arg value. If you have not '
        'run handeye_calib yet, roll/pitch alignment WILL be off by degrees.'))

    return LaunchDescription(args + [handeye_reminder,
                                     _static_tf(),
                                     _vision(filter_frame, vision_source,
                                             marker_id, capture_fps),
                                     TimerAction(period=3.0, actions=[
                                         _controller(
                                             robot_description,
                                             robot_description_semantic,
                                             kinematics, params_file,
                                             condition=IfCondition(
                                                 start_mating_node)),
                                         _tracking(params_file)])])


def _static_tf():
    # static_transform_publisher needs individual tokens; split the two
    # space-separated launch args into positional arguments at launch time.
    from launch.actions import OpaqueFunction

    def make(context):
        xyz = LaunchConfiguration('handeye_xyz').perform(context).split()
        quat = LaunchConfiguration('handeye_quat').perform(context).split()
        return [Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='handeye_tcp_to_optical',
            arguments=['--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                       '--qx', quat[0], '--qy', quat[1], '--qz', quat[2],
                       '--qw', quat[3],
                       '--frame-id', 'fr3_hand_tcp',
                       '--child-frame-id', 'camera_color_optical_frame'],
        )]

    return OpaqueFunction(function=make)


def _vision(filter_frame, vision_source, marker_id, capture_fps):
    return Node(
        package='roscam',
        executable='cam_pub',
        output='screen',
        parameters=[{
            # topic mode: RealSense D405 driver defaults (remap via -p).
            # realsense mode ignores these and captures in-process -
            # zero image topics on the graph (see README defence 0).
            'image_topic': '/camera/camera/color/image_rect_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'source': vision_source,
            'filter_frame': filter_frame,
            'marker_id': ParameterValue(marker_id, value_type=int),
            'capture_fps': ParameterValue(capture_fps, value_type=int),
            # Debug image is only encoded/published while something
            # subscribes - keep GUI subscriptions off the robot NIC.
            'publish_debug_image': True,
        }],
    )


def _tracking(params_file):
    # fr3_params.yaml reaches a node only through params_file, so without
    # this the tracking_* values never arrive. Idle until START TRACKING.
    # FR3_LOG_DIR (fr3_env.sh) overrides tracking_log_dir so every TRACK run
    # is recorded; the node does not create directories, so make the day
    # folder here.
    overrides = {}
    log_root = os.environ.get('FR3_LOG_DIR')
    if log_root:
        day_dir = os.path.join(log_root, datetime.date.today().isoformat())
        os.makedirs(day_dir, exist_ok=True)
        overrides['tracking_log_dir'] = day_dir
    return Node(
        package='mating_controller',
        executable='tracking_node',
        output='screen',
        parameters=[params_file, overrides],
    )


def _controller(robot_description, robot_description_semantic, kinematics,
                params_file, condition=None):
    return Node(
        condition=condition,
        package='mating_controller',
        executable='mating_node',
        output='screen',
        parameters=[robot_description,
                    robot_description_semantic,
                    kinematics,
                    params_file],
    )
