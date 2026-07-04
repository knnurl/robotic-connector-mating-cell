#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp> // Include for subscribing to cam/distance

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");

// Function for linear downward motion
void move_downward(
    moveit::planning_interface::MoveGroupInterface &move_group,
    double distance = 0.05) // Default: Move down 0.05m
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
    double distance = 0.05) // Default: Move down 0.05m
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

class DistanceSubscriber : public rclcpp::Node
{
public:
    DistanceSubscriber(moveit::planning_interface::MoveGroupInterface &move_group)
        : Node("distance_subscriber"), move_group_(move_group)
    {
        // Subscription to cam/distance topic
        distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "cam/distance", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                RCLCPP_INFO(LOGGER, "Received distance: %.2f", msg->data);
                if (msg->data > 0.5)
                {
                    RCLCPP_INFO(LOGGER, "Distance exceeded threshold. Moving downward...");
                    move_downward(move_group_, 0.025); // Move down 5cm
                }
                else if (msg->data < 0.35)
                {
                    RCLCPP_INFO(LOGGER, "Distance exceeded threshold. Moving upward...");
                    move_upward(move_group_, 0.025); // Move down 5cm        
                }
                else
                {
                    RCLCPP_INFO(LOGGER, "Distance within threshold. Not moving ...");
                }
            });
    }

private:
    moveit::planning_interface::MoveGroupInterface &move_group_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Initialize node and MoveGroupInterface
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_node", node_options);

    auto executor = rclcpp::executors::MultiThreadedExecutor();
    executor.add_node(move_group_node);

    // Define the planning group
    std::string param_planning_group = move_group_node->get_parameter("planning_group").as_string();
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, param_planning_group);

    // Set up scaling factors and tolerances
    move_group.setMaxVelocityScalingFactor(0.08);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalJointTolerance(0.001);

    RCLCPP_INFO(LOGGER, "Starting downward motion demo...");

    // Add DistanceSubscriber node
    auto distance_subscriber_node = std::make_shared<DistanceSubscriber>(move_group);
    executor.add_node(distance_subscriber_node);

    // Start executor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

