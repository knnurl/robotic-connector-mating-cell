"""Connector-mating stack for a Franka FR3 (real or fake hardware).

Launches the CELL side only: hand-eye static TF, ArUco vision (filtered in
fr3_link0), and the mating controller with FR3 parameters. The ROBOT side
(FCI driver + move_group) is launched separately with upstream tooling:

  # Terminal 1 - robot driver + MoveIt (add use_fake_hardware:=true for dry runs)
  source ~/franka_ros2_ws/install/setup.sh
  ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.3

  # Terminal 2 - camera (LOW-BANDWIDTH config - see README.md, this matters)
  ros2 launch realsense2_camera rs_launch.py config_file:=$(pwd)/tools/fr3/realsense_low_bw.yaml

  # Terminal 3 - this file
  source /opt/ros/humble/setup.sh && source ~/franka_ros2_ws/install/setup.sh
  export AMENT_PREFIX_PATH="$PWD/install/melfa_rv5as_masterclass:\
$PWD/install/roscam:$AMENT_PREFIX_PATH"
  ros2 launch tools/fr3/fr3_mating.launch.py robot_ip:=172.16.0.3

Run tools/fr3/fr3_preflight.sh FIRST - it checks the RT kernel, DDS
interface isolation, and camera bandwidth traps that stop the 1 kHz FCI
loop (communication_constraints_violation).

The hand-eye defaults below are the dry-run GUESS. Calibrate with
`ros2 run roscam handeye_calib --ros-args -p base_frame:=fr3_link0
-p tcp_frame:=fr3_hand_tcp` and pass the printed values as launch args.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
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

    args = [
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.3',
                              description='FR3 FCI IP (only used to render the URDF)'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(THIS_DIR, 'fr3_params.yaml')),
        DeclareLaunchArgument('filter_frame', default_value='fr3_link0',
                              description="KF frame for cam_pub ('' = optical/legacy)"),
        DeclareLaunchArgument('vision_source', default_value='topic',
                              description='topic = camera driver publishes images (default); '
                                          'realsense = in-process capture, no image topics '
                                          '(skip the camera-driver terminal)'),
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
                                     _vision(filter_frame, vision_source),
                                     TimerAction(period=3.0, actions=[
                                         _controller(robot_description,
                                                     robot_description_semantic,
                                                     kinematics, params_file)])])


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


def _vision(filter_frame, vision_source):
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
            # Debug image is only encoded/published while something
            # subscribes - keep GUI subscriptions off the robot NIC.
            'publish_debug_image': True,
        }],
    )


def _controller(robot_description, robot_description_semantic, kinematics,
                params_file):
    return Node(
        package='melfa_rv5as_masterclass',
        executable='move_l',
        output='screen',
        parameters=[robot_description,
                    robot_description_semantic,
                    kinematics,
                    params_file],
    )
