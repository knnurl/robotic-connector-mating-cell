
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('melfa_rv5as_masterclass'),
        'config',
        'rv5as_params.yaml'
    )

    moveit_config = MoveItConfigsBuilder("rv5as", package_name="melfa_rv5as_moveit_config").to_moveit_configs()

    pnp_node = Node(
        package="melfa_rv5as_masterclass",
        executable="move_l",  # Ensure this matches your compiled executable name
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            config
        ],
    )

    return LaunchDescription([pnp_node])
