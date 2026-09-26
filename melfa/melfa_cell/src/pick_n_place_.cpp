#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// Function to perform linear downward motion
void move_downward(
    moveit::planning_interface::MoveGroupInterface &move_group,
    double distance = 0.05) // Default: Move down 5cm
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
    moveit::planning_interface::MoveGroupInterface::Plan plan_down;
    if (move_group.plan(plan_down) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(LOGGER, "Executing downward motion...");
        move_group.execute(plan_down);
        RCLCPP_INFO(LOGGER, "Downward motion complete.");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Failed to plan downward motion.");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Initialize node and MoveGroupInterface
    auto move_group_node =
        rclcpp::Node::make_shared("move_group_interface_tutorial");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP_ARM = "rv5as";

    moveit::planning_interface::MoveGroupInterface move_group_arm(
        move_group_node, PLANNING_GROUP_ARM);

    // Step 1: Move to the starting pose using OMPL planner
    move_group_arm.setPlanningPipelineId("ompl");
    move_group_arm.setMaxVelocityScalingFactor(0.04);
    move_group_arm.setMaxAccelerationScalingFactor(0.1);

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = 0.2;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.5;
    start_pose.orientation.w = 0;
    start_pose.orientation.x = 1;
    start_pose.orientation.y = 0;
    start_pose.orientation.z = 0;

    move_group_arm.setPoseTarget(start_pose);

    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    if (move_group_arm.plan(start_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(LOGGER, "Moving to the starting pose...");
        move_group_arm.execute(start_plan);
        RCLCPP_INFO(LOGGER, "Reached the starting pose.");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Failed to move to the starting pose.");
        return 1;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Step 2: Perform a 5cm downward motion using Pilz LIN planner
    move_downward(move_group_arm, 0.05);

    rclcpp::shutdown();
    return 0;
}

