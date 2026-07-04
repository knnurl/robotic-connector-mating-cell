"""Launch the connector-mating controller.

Robot-agnostic: point it at any MoveIt config package.

  ros2 launch melfa_rv5as_masterclass move_l.launch.py                 # MELFA RV-5AS
  ros2 launch melfa_rv5as_masterclass move_l.launch.py \
      robot_name:=ur5e moveit_config_package:=ur5e_moveit_config \
      params_file:=/path/to/my_robot_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    moveit_config_package = LaunchConfiguration('moveit_config_package').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    moveit_config = MoveItConfigsBuilder(
        robot_name, package_name=moveit_config_package).to_moveit_configs()

    controller_node = Node(
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
    return [controller_node]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('melfa_rv5as_masterclass'),
        'config', 'rv5as_params.yaml')

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
        OpaqueFunction(function=launch_setup),
    ])
