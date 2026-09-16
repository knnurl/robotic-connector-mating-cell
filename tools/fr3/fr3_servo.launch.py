#!/usr/bin/env python3
"""Start moveit_servo for the FR3 cell.

Run this ALONGSIDE an already-running franka_fr3_moveit_config
moveit.launch.py - it adds only the servo node, using the same xacro and
SRDF the upstream launch renders so the two cannot disagree about the model.

    source tools/fr3/fr3_env.sh
    ros2 launch tools/fr3/fr3_servo.launch.py

Then align_gui's "servo" backend streams TwistStamped on
/servo_node/delta_twist_cmds and servo converts it to a continuous joint
POSITION stream on /fr3_servo_position_controller/commands.

This launch also loads that controller, INACTIVE. align_gui activates it
(deactivating fr3_arm_controller) for the duration of a servo run and always
switches back afterwards, because franka_hardware allows exactly one command
mode at a time:
  * fr3_arm_controller           effort, trajectories from move_group
                                 (the cartesian backend)
  * fr3_servo_position_controller  position, the servo stream
Streaming into the effort trajectory controller is what left the arm stalled
and buzzing - see fr3_servo_controllers.yaml for the measurements.

Servo starts PAUSED: nothing moves until /servo_node/start_servo is called
(align_gui does this when you select the servo backend).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    try:
        with open(os.path.join(get_package_share_directory(package_name),
                               file_path), 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    franka_desc = get_package_share_directory('franka_description')

    # Same xacro invocation as franka_fr3_moveit_config/launch/moveit.launch.py
    # (hand:=true), so servo's model matches move_group's exactly.
    urdf = Command([FindExecutable(name='xacro'), ' ',
                    os.path.join(franka_desc, 'robots', 'fr3',
                                 'fr3.urdf.xacro'),
                    ' hand:=true'])
    srdf = Command([FindExecutable(name='xacro'), ' ',
                    os.path.join(franka_desc, 'robots', 'fr3',
                                 'fr3.srdf.xacro'),
                    ' hand:=true'])

    here = os.path.dirname(os.path.abspath(__file__))
    servo_yaml = os.path.join(here, 'fr3_servo.yaml')
    with open(servo_yaml, 'r') as f:
        servo_params = yaml.safe_load(f)['/servo_node']['ros__parameters']
    controllers_yaml = os.path.join(here, 'fr3_servo_controllers.yaml')

    return LaunchDescription([
        # Loaded and configured but NOT activated: fr3_arm_controller keeps
        # the arm until align_gui swaps them for a servo run.
        Node(
            package='controller_manager',
            executable='spawner',
            name='fr3_servo_position_controller_spawner',
            output='screen',
            arguments=[
                'fr3_servo_position_controller',
                '--inactive',
                '-t', 'position_controllers/JointGroupPositionController',
                '-p', controllers_yaml,
                '--controller-manager-timeout', '30',
            ],
        ),
        Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            output='screen',
            parameters=[
                servo_params,
                {'robot_description': ParameterValue(urdf, value_type=str)},
                {'robot_description_semantic':
                    ParameterValue(srdf, value_type=str)},
                load_yaml('franka_fr3_moveit_config',
                          'config/kinematics.yaml') or {},
            ],
        ),
    ])
