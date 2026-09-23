"""Headless dry run of the connector-mating controller on a mock Franka FR3.

Proves the controller end-to-end without any robot hardware:
  mock ros2_control FR3 + move_group (OMPL) + fake hand-eye TF
  + synthetic marker vision (fake_marker_pub.py) + mating_node controller.

Run:
  source /opt/ros/humble/setup.sh
  source ~/franka_ros2_ws/install/setup.sh
  ros2 launch <this file>

Expected: phases WAIT_FOR_VISION -> ALIGN_COARSE -> ALIGN_FINE -> INSERT ->
MATED in the mating_node log, with the alignment error converging to < 2.5 mm / 1 deg.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def load_yaml(package_name, file_path):
    with open(os.path.join(get_package_share_directory(package_name), file_path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    franka_desc = get_package_share_directory('franka_description')

    robot_description = {'robot_description': ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(franka_desc, 'robots', 'fr3', 'fr3.urdf.xacro'),
        ' hand:=true robot_ip:=127.0.0.1 use_fake_hardware:=true ros2_control:=true',
    ]), value_type=str)}

    robot_description_semantic = {'robot_description_semantic': ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(franka_desc, 'robots', 'fr3', 'fr3.srdf.xacro'),
        ' hand:=true',
    ]), value_type=str)}

    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    ompl_config = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters':
            'default_planner_request_adapters/AddTimeOptimalParameterization '
            'default_planner_request_adapters/ResolveConstraintFrames '
            'default_planner_request_adapters/FixWorkspaceBounds '
            'default_planner_request_adapters/FixStartStateBounds '
            'default_planner_request_adapters/FixStartStateCollision '
            'default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1,
    }
    ompl_config.update(load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml'))
    # Modern multi-pipeline layout so the pipeline is addressable as 'ompl'
    ompl_pipeline = {'planning_pipelines': ['ompl'], 'ompl': ompl_config}

    arm_joints = [f'fr3_joint{i}' for i in range(1, 8)]
    moveit_controllers = {
        'moveit_simple_controller_manager': {
            'controller_names': ['fr3_arm_controller'],
            'fr3_arm_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': True,
                'joints': arm_joints,
            },
        },
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_pipeline,
            moveit_controllers,
            {
                'moveit_manage_controllers': True,
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
            },
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description,
                    os.path.join(THIS_DIR, 'fr3_dryrun_controllers.yaml')],
        output='screen',
    )

    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[name, '--controller-manager', '/controller_manager'])
        for name in ('joint_state_broadcaster', 'fr3_arm_controller')
    ]

    # Fake hand-eye: camera rigidly on the hand, looking along the tool Z.
    hand_eye_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.06', '--y', '0.0', '--z', '-0.04',
                   '--frame-id', 'fr3_hand_tcp',
                   '--child-frame-id', 'camera_color_optical_frame'],
    )

    fake_marker = ExecuteProcess(
        cmd=['python3', os.path.join(THIS_DIR, 'fake_marker_pub.py')],
        output='screen',
    )

    controller = Node(
        package='mating_controller',
        executable='mating_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {
                'planning_group': 'fr3_arm',
                'EEF_FRAME_ID': 'fr3_hand_tcp',
                'planning_pipeline': 'ompl',
                'planner_id': '',
                'pose_topic': '/aruco/pose',
                'control_period_s': 0.4,
                'vision_timeout_s': 0.6,
                'connector_offset_x': 0.0,
                'connector_offset_y': 0.0,
                'connector_offset_z': 0.0,
                'tool_yaw_offset_deg': 0.0,
                'standoff_height_m': 0.10,
                'insertion_depth_m': 0.05,
                'coarse_max_step_m': 0.05,
                'coarse_max_step_deg': 5.0,
                'coarse_speed': 0.25,
                'coarse_pos_tol_m': 0.02,
                'coarse_rot_tol_deg': 5.0,
                'fine_max_step_m': 0.01,
                'fine_max_step_deg': 1.5,
                'fine_speed': 0.08,
                'fine_pos_tol_m': 0.0025,
                'fine_rot_tol_deg': 1.0,
                'align_hold_cycles': 3,
                'insert_speed': 0.03,
                'accel_scaling': 0.1,
                'enable_insertion': True,
                'max_joint_jump_rad': 0.8,
                'max_consecutive_plan_failures': 5,
            },
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        *spawners,
        move_group_node,
        hand_eye_tf,
        TimerAction(period=6.0, actions=[fake_marker]),
        TimerAction(period=10.0, actions=[controller]),
    ])
