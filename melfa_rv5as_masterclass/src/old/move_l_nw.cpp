// COPYRIGHT (C) 2024 Mitsubishi Electric Corporation

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp> // Change to Float32

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");

// Function for linear downward motion
void move_downward(
    moveit::planning_interface::MoveGroupInterface &move_group,
    double distance = 0.03)
{
    auto current_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(LOGGER, "Current pose: [x: %.2f, y: %.2f, z: %.2f]",
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z);

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.z -= distance;

    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN");
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RCLCPP_INFO(LOGGER, "Executing downward motion...");
        move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "Downward motion complete.");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Downward motion planning failed.");
    }
}

// Function for linear upward motion
void move_upward(
    moveit::planning_interface::MoveGroupInterface &move_group,
    double distance = 0.03)
{
    auto current_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(LOGGER, "Current pose: [x: %.2f, y: %.2f, z: %.2f]",
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z);

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.z += distance;

    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN");
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RCLCPP_INFO(LOGGER, "Executing upward motion...");
        move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "Upward motion complete.");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Upward motion planning failed.");
    }
}

void distance_callback(
    const std_msgs::msg::Float32::SharedPtr msg, // Change to Float32
    moveit::planning_interface::MoveGroupInterface &move_group)
{
    float distance = msg->data; // Use float
    RCLCPP_INFO(LOGGER, "Received distance: %.2f", distance);

    if (distance < 0.3)
    {
        RCLCPP_INFO(LOGGER, "Distance below 0.3, moving upward...");
        move_upward(move_group, 0.03);
    }
    else if (distance > 0.45)
    {
        RCLCPP_INFO(LOGGER, "Distance above 0.45, moving downward...");
        move_downward(move_group, 0.03);
    }
    else
    {
        RCLCPP_INFO(LOGGER, "Distance within range, no movement.");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Initialize node and MoveGroupInterface
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_node", node_options);

    std::string param_planning_group = move_group_node->get_parameter("planning_group").as_string();
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, param_planning_group);

    move_group.setMaxVelocityScalingFactor(0.08);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalJointTolerance(0.001);

    RCLCPP_INFO(LOGGER, "Starting distance listener...");

    auto subscription = move_group_node->create_subscription<std_msgs::msg::Float32>(
        "distance", 10, // Change to Float32
        [&move_group](const std_msgs::msg::Float32::SharedPtr msg) { // Change to Float32
            distance_callback(msg, move_group);
        });

    rclcpp::spin(move_group_node);
    rclcpp::shutdown();
    return 0;
}

