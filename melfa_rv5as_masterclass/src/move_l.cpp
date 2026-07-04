#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <algorithm> // For std::max
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp> // Assuming /cam topic publishes this type

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");

int closeEnough = 0;
int finalDown = 0;
int finalDownTriggered = 0;
bool mated = false;

// Function for linear motion
void move(
    moveit::planning_interface::MoveGroupInterface &move_group,
    double x = 0.00, double y = 0.00, 
    double z = 0.00, double distance = 0.05) // Default: Move by 0.05m
{
    // Get the current pose
    auto current_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(LOGGER, "Current pose: [x: %.2f, y: %.2f, z: %.2f]", 
                current_pose.position.x, 
                current_pose.position.y, 
                current_pose.position.z);

    // Modify the pose
    geometry_msgs::msg::Pose target_pose = current_pose;
    
    if (x > 0.03) {
        target_pose.position.x -= distance;
        RCLCPP_INFO(LOGGER, "X exceeded positive threshold. Moving backward...");
    } else if (x < -0.03) {
        target_pose.position.x += distance;
        RCLCPP_INFO(LOGGER, "X exceeded negative threshold. Moving forward...");
    }

    if (y > 0.03) {
        target_pose.position.y -= distance;
        RCLCPP_INFO(LOGGER, "Y exceeded positive threshold. Moving right...");
    } else if (y < -0.03) {
        target_pose.position.y += distance;
        RCLCPP_INFO(LOGGER, "Y exceeded negative threshold. Moving left...");
    }

    if (z > 0.32) {
        target_pose.position.z -= distance;
        RCLCPP_INFO(LOGGER, "Z exceeded upper threshold. Moving downward...");
    } else if (z < 0.27) {
        target_pose.position.z += distance;
        RCLCPP_INFO(LOGGER, "Z fell below lower threshold. Moving upward...");
    }

    // Configure Pilz LIN planner
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN");

    // Set the target pose
    move_group.setPoseTarget(target_pose);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        RCLCPP_INFO(LOGGER, "Executing motion...");
        move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "Motion complete.");
    } else {
        RCLCPP_ERROR(LOGGER, "Motion planning failed.");
    }
}

class CamSubscriber : public rclcpp::Node
{
public:
    CamSubscriber(moveit::planning_interface::MoveGroupInterface &move_group)
        : Node("cam_subscriber"), move_group_(move_group)
    {
        // Subscription to /cam topic
        cam_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/cam", 10,
            [this, &move_group](geometry_msgs::msg::Point::SharedPtr msg) { // Capture move_group by reference
                double x = msg->x;
                double y = msg->y;
                double z = msg->z; // Assuming 'z' field represents distance
                RCLCPP_INFO(LOGGER, "Received data - x: %.2f, y: %.2f, z: %.2f", x, y, z);

				double smallest = std::min(x, std::min(y, z));
				
                // Check if any value is outside thresholds
                if (closeEnough == 0 && (std::abs(x) > 0.03 || std::abs(y) > 0.03 || z > 0.32 || z < 0.27)) {
                    RCLCPP_INFO(LOGGER, "Values outside thresholds. Triggering motion...");
                    move(move_group, x, y, z, fabs(std::min(0.1, smallest))); // Move with adjustments
                } else {
                    RCLCPP_INFO(LOGGER, "Values within thresholds. No motion triggered.");
                    closeEnough = 1;
                }

                if (closeEnough == 1) {
                    closeEnough = 0; // Reset closeEnough
                    finalDown = 1;
                    RCLCPP_INFO(LOGGER, "close enough triggered.");

                    // Get the current pose
                    auto current_pose = move_group.getCurrentPose().pose;
                    RCLCPP_INFO(LOGGER, "Current pose before movement: [x: %.2f, y: %.2f, z: %.2f]", 
                                current_pose.position.x, 
                                current_pose.position.y, 
                                current_pose.position.z);

                    // Modify the pose: Move 12 cm (0.12 m) forward and 3 cm (0.03 m) downward
                    geometry_msgs::msg::Pose target_pose = current_pose;
                    target_pose.position.x += 0.12; // Move forward
                    target_pose.position.z -= 0.03; // Move downward

                    // Configure Pilz LIN planner
                    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
                    move_group.setPlannerId("LIN");

                    // Set the target pose
                    move_group.setPoseTarget(target_pose);

                    // Plan and execute
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    if (success) {
                        RCLCPP_INFO(LOGGER, "Executing movement: 12 cm forward and 3 cm down...");
                        move_group.execute(plan);
                        RCLCPP_INFO(LOGGER, "Pre-final movement complete.");
                    } else {
                        RCLCPP_ERROR(LOGGER, "Pre-final movement planning failed.");
                    }
                }
                
                if (finalDown == 1 && finalDownTriggered == 0) {
                    finalDown = 0; // Reset finalDown
                    finalDownTriggered = 1;
                    
                    RCLCPP_INFO(LOGGER, "finalDown triggered.");

                    // Get the current pose
                    auto current_pose = move_group.getCurrentPose().pose;
                    RCLCPP_INFO(LOGGER, "Current pose before movement: [x: %.2f, y: %.2f, z: %.2f]", 
                                current_pose.position.x, 
                                current_pose.position.y, 
                                current_pose.position.z);

                    // Modify the pose: Move 1.5 cm (0.015 m) downward
                    geometry_msgs::msg::Pose target_pose = current_pose;

                    target_pose.position.z -= 0.06; // Move downward

                    // Configure Pilz LIN planner
                    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
                    move_group.setPlannerId("LIN");

                    // Set the target pose
                    move_group.setPoseTarget(target_pose);

                    // Plan and execute
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    if (success) {
                        RCLCPP_INFO(LOGGER, "Executing movement:3 cm down...");
                        move_group.execute(plan);
                        RCLCPP_INFO(LOGGER, "Final movement complete.");
                    } else {
                        RCLCPP_ERROR(LOGGER, "Final movement planning failed.");
                    }
                }
            });
    }

private:
    moveit::planning_interface::MoveGroupInterface &move_group_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cam_subscription_;
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

    RCLCPP_INFO(LOGGER, "Starting motion demo...");

    // Add CamSubscriber node
    auto cam_subscriber_node = std::make_shared<CamSubscriber>(move_group);
    executor.add_node(cam_subscriber_node);
 
    // Start executor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
