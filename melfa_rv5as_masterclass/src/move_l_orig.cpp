// COPYRIGHT (C) 2024 Mitsubishi Electric Corporation

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");

// Function for linear downward motion
void move_downward(
    moveit::planning_interface::MoveGroupInterface &move_group,
    double distance =0.05) // Default: Move down 5cm
{
    // Get the current pose
    auto current_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(LOGGER, "Current pose: [x: %.2f, y: %.2f, z: %.2f]", 
                current_pose.position.x, 
                current_pose.position.y, 
                current_pose.position.z);

    // Modify the pose to move downward
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.z -= distance;

    // Configure Pilz LIN planner
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN");

    // Set the target pose
    move_group.setPoseTarget(target_pose);

    // Plan and execute
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

// Function for linear downward motion
void move_upward(
    moveit::planning_interface::MoveGroupInterface &move_group,
    double distance =0.05) // Default: Move down 5cm
{
    // Get the current pose
    auto current_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(LOGGER, "Current pose: [x: %.2f, y: %.2f, z: %.2f]", 
                current_pose.position.x, 
                current_pose.position.y, 
                current_pose.position.z);

    // Modify the pose to move downward
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.z += distance;

    // Configure Pilz LIN planner
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN");

    // Set the target pose
    move_group.setPoseTarget(target_pose);

    // Plan and execute
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
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Initialize node and MoveGroupInterface
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_node", node_options);

    auto executor = rclcpp::executors::MultiThreadedExecutor();
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Define the planning group
    std::string param_planning_group = move_group_node->get_parameter("planning_group").as_string();
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, param_planning_group);

    // Set up scaling factors and tolerances
    move_group.setMaxVelocityScalingFactor(0.08);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalJointTolerance(0.001);

    RCLCPP_INFO(LOGGER, "Starting downward motion demo...");

    // Perform downward motion
    move_downward(move_group, 0.05); // Move down 5cm
    RCLCPP_INFO(LOGGER, "Downward motion completed. Going up.");
    
    rclcpp::sleep_for(std::chrono::seconds(1));
      
    move_upward(move_group, 0.05); // Move up 5cm
    RCLCPP_INFO(LOGGER, "Upward motion completed. Returning to home position.");

    // Return to home position
    move_group.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (move_group.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group.execute(home_plan);
        RCLCPP_INFO(LOGGER, "Returned to home position.");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Failed to return to home position.");
    }

    rclcpp::shutdown();
    return 0;
}

