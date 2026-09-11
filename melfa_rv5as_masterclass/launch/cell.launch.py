"""Consolidated cell launch: vision + hand-eye TF + mating controller.

One command for the whole CELL side. The ROBOT side (driver + move_group)
stays on the vendor's own launch files - see SETUP_AND_CALIBRATION.md §4.

  # MELFA RV-5AS defaults, camera driver publishing image topics:
  ros2 launch melfa_rv5as_masterclass cell.launch.py \
      handeye_xyz:="<x> <y> <z>" handeye_quat:="<qx> <qy> <qz> <qw>"

  # in-process capture (no image topics), fixed-frame KF, depth-ICP:
  ros2 launch melfa_rv5as_masterclass cell.launch.py \
      vision_source:=realsense filter_frame:=rv5as_base \
      template_stl:=/path/connector.stl

Anything robot-specific is an argument; for a different robot pass
robot_name/moveit_config_package/params_file exactly as move_l.launch.py.
The hand-eye arguments MUST come from handeye_calib (setup guide §2.3) -
there is deliberately no default: a guessed transform is the dominant
roll/pitch error source and should never be launched silently.

When template_stl is set, connector_pose runs and the controller is
expected to use pose_topic: /connector/pose in its params file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    moveit_config_package = \
        LaunchConfiguration('moveit_config_package').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    xyz = LaunchConfiguration('handeye_xyz').perform(context).split()
    quat = LaunchConfiguration('handeye_quat').perform(context).split()

    moveit_config = MoveItConfigsBuilder(
        robot_name, package_name=moveit_config_package).to_moveit_configs()

    controller = Node(
        package='melfa_rv5as_masterclass',
        executable='move_l',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            params_file,
        ],
    )

    hand_eye_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='handeye_tcp_to_camera',
        arguments=['--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                   '--qx', quat[0], '--qy', quat[1], '--qz', quat[2],
                   '--qw', quat[3],
                   '--frame-id',
                   LaunchConfiguration('tcp_frame').perform(context),
                   '--child-frame-id',
                   LaunchConfiguration('camera_frame').perform(context)],
    )
    return [controller, hand_eye_tf]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('melfa_rv5as_masterclass'),
        'config', 'rv5as_params.yaml')

    vision = Node(
        package='roscam',
        executable='cam_pub',
        output='screen',
        parameters=[{
            'source': LaunchConfiguration('vision_source'),
            'filter_frame': LaunchConfiguration('filter_frame'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        }],
    )

    # Optional connector-level ICP refinement (setup guide §6).
    connector_pose = Node(
        package='roscam',
        executable='connector_pose',
        output='screen',
        condition=LaunchConfigurationNotEquals('template_stl', ''),
        parameters=[{
            'template_stl': LaunchConfiguration('template_stl'),
            'source': LaunchConfiguration('vision_source'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='rv5as',
            description='Robot name as used by the MoveIt config package'),
        DeclareLaunchArgument(
            'moveit_config_package', default_value='melfa_rv5as_moveit_config',
            description='MoveIt config package for the target robot'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Controller parameter file (frames, offsets, speeds)'),
        DeclareLaunchArgument(
            'handeye_xyz',
            description='Calibrated TCP->camera translation "x y z" (m). '
                        'REQUIRED - run handeye_calib, do not guess.'),
        DeclareLaunchArgument(
            'handeye_quat',
            description='Calibrated TCP->camera rotation "qx qy qz qw". '
                        'REQUIRED - run handeye_calib, do not guess.'),
        DeclareLaunchArgument('tcp_frame', default_value='rv5as_default_tcp'),
        DeclareLaunchArgument(
            'camera_frame', default_value='camera_link',
            description='Child frame of the hand-eye TF (RealSense: '
                        'camera_link; other cameras: the image frame_id)'),
        DeclareLaunchArgument(
            'vision_source', default_value='topic',
            description="topic = camera driver publishes images; realsense "
                        "= in-process capture, no image topics"),
        DeclareLaunchArgument(
            'filter_frame', default_value='',
            description="Fixed frame for the vision KF ('' = optical)"),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/camera/color/image_rect_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument(
            'template_stl', default_value='',
            description='Connector CAD STL: enables depth-ICP refinement '
                        '(set pose_topic: /connector/pose in the params file)'),
        vision,
        connector_pose,
        OpaqueFunction(function=launch_setup),
    ])
