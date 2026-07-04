// Connector-mating controller for the MELFA RV-5AS.
//
// Consumes the marker pose published by roscam (/aruco/pose, PoseStamped in
// the camera optical frame), transforms it into the planning frame via TF2,
// and drives a phased mating sequence:
//
//   WAIT_FOR_VISION -> ALIGN_COARSE -> ALIGN_FINE -> INSERT -> MATED
//
// ALIGN phases servo the full 6-DOF error (X, Y, Z, roll, pitch, yaw) toward
// a standoff pose above the connector, with per-cycle step clamps so each
// commanded LIN goal stays close to the current configuration. INSERT is a
// single committed slow LIN stroke along the tool axis, taken only while
// vision is fresh and alignment is within fine tolerance; once MATED the arm
// is latched and no further motion is commanded.
//
// The /aruco/pose subscription only stores the latest sample; all planning
// and execution happens in a dedicated control thread, never in a callback.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("connector_mating");

enum class Phase { WAIT_FOR_VISION, ALIGN_COARSE, ALIGN_FINE, INSERT, MATED, FAULT };

static const char *phase_name(Phase p)
{
    switch (p) {
    case Phase::WAIT_FOR_VISION: return "WAIT_FOR_VISION";
    case Phase::ALIGN_COARSE:    return "ALIGN_COARSE";
    case Phase::ALIGN_FINE:      return "ALIGN_FINE";
    case Phase::INSERT:          return "INSERT";
    case Phase::MATED:           return "MATED";
    case Phase::FAULT:           return "FAULT";
    }
    return "?";
}

struct Params
{
    std::string pose_topic{"/aruco/pose"};
    double control_period_s{0.4};
    double vision_timeout_s{0.6};

    // Where the connector sits relative to the marker, expressed in the
    // marker frame (X/Y in the marker plane, Z out of the surface). Teach
    // these on the physical cell.
    double connector_offset_x{0.0};
    double connector_offset_y{0.0};
    double connector_offset_z{0.0};
    // Extra tool yaw about the marker normal, degrees.
    double tool_yaw_offset_deg{0.0};

    // Standoff height above the connector where alignment happens, and the
    // committed insertion stroke length from that standoff.
    double standoff_height_m{0.08};
    double insertion_depth_m{0.06};

    // Per-cycle step clamps and phase speeds.
    double coarse_max_step_m{0.05};
    double coarse_max_step_deg{5.0};
    double coarse_speed{0.25};
    double fine_max_step_m{0.01};
    double fine_max_step_deg{1.5};
    double fine_speed{0.08};
    double insert_speed{0.03};

    // Tolerances. Coarse tolerance switches ALIGN_COARSE -> ALIGN_FINE;
    // fine tolerance (held for align_hold_cycles) arms the insertion.
    double coarse_pos_tol_m{0.02};
    double coarse_rot_tol_deg{5.0};
    double fine_pos_tol_m{0.0025};
    double fine_rot_tol_deg{1.0};
    int align_hold_cycles{3};

    // Reject any plan whose joint-space motion is suspiciously large
    // (IK branch flip / wrist flip guard).
    double max_joint_jump_rad{0.8};
    int max_consecutive_plan_failures{5};
};

class ConnectorMatingController
{
public:
    ConnectorMatingController(const rclcpp::Node::SharedPtr &node,
                              moveit::planning_interface::MoveGroupInterface &move_group)
        : node_(node), move_group_(move_group),
          tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
    {
        load_params();

        sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            params_.pose_topic, rclcpp::QoS(1),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                latest_pose_ = *msg;
                latest_pose_arrival_ = std::chrono::steady_clock::now();
            });

        planning_frame_ = move_group_.getPlanningFrame();
        eef_link_ = move_group_.getEndEffectorLink();
        RCLCPP_INFO(LOGGER, "Planning frame: %s, end effector: %s",
                    planning_frame_.c_str(), eef_link_.c_str());

        move_group_.setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_.setPlannerId("LIN");
        move_group_.setMaxAccelerationScalingFactor(0.1);
        move_group_.setGoalPositionTolerance(0.001);
        move_group_.setGoalOrientationTolerance(0.01);
    }

    void run()
    {
        RCLCPP_INFO(LOGGER, "Connector mating controller running.");
        while (rclcpp::ok()) {
            const auto cycle_start = std::chrono::steady_clock::now();
            try {
                step();
            } catch (const std::exception &e) {
                RCLCPP_ERROR(LOGGER, "Control cycle exception: %s", e.what());
            }
            const auto period =
                std::chrono::duration<double>(params_.control_period_s);
            std::this_thread::sleep_until(cycle_start + period);
        }
    }

private:
    void load_params()
    {
        auto &p = params_;
        auto get = [this](const std::string &name, auto &field) {
            node_->get_parameter_or(name, field, field);
        };
        get("pose_topic", p.pose_topic);
        get("control_period_s", p.control_period_s);
        get("vision_timeout_s", p.vision_timeout_s);
        get("connector_offset_x", p.connector_offset_x);
        get("connector_offset_y", p.connector_offset_y);
        get("connector_offset_z", p.connector_offset_z);
        get("tool_yaw_offset_deg", p.tool_yaw_offset_deg);
        get("standoff_height_m", p.standoff_height_m);
        get("insertion_depth_m", p.insertion_depth_m);
        get("coarse_max_step_m", p.coarse_max_step_m);
        get("coarse_max_step_deg", p.coarse_max_step_deg);
        get("coarse_speed", p.coarse_speed);
        get("fine_max_step_m", p.fine_max_step_m);
        get("fine_max_step_deg", p.fine_max_step_deg);
        get("fine_speed", p.fine_speed);
        get("insert_speed", p.insert_speed);
        get("coarse_pos_tol_m", p.coarse_pos_tol_m);
        get("coarse_rot_tol_deg", p.coarse_rot_tol_deg);
        get("fine_pos_tol_m", p.fine_pos_tol_m);
        get("fine_rot_tol_deg", p.fine_rot_tol_deg);
        get("align_hold_cycles", p.align_hold_cycles);
        get("max_joint_jump_rad", p.max_joint_jump_rad);
        get("max_consecutive_plan_failures", p.max_consecutive_plan_failures);
    }

    void set_phase(Phase next)
    {
        if (next != phase_) {
            RCLCPP_INFO(LOGGER, "Phase: %s -> %s", phase_name(phase_), phase_name(next));
            phase_ = next;
        }
    }

    // Latest marker pose if it is fresh enough, else nullopt.
    std::optional<geometry_msgs::msg::PoseStamped> fresh_marker_pose()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!latest_pose_) {
            return std::nullopt;
        }
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - latest_pose_arrival_).count();
        if (age > params_.vision_timeout_s) {
            return std::nullopt;
        }
        return latest_pose_;
    }

    // Marker pose -> desired TCP standoff pose, in the planning frame.
    std::optional<tf2::Transform> compute_standoff_goal(
        const geometry_msgs::msg::PoseStamped &marker_msg)
    {
        geometry_msgs::msg::PoseStamped marker_in_planning;
        try {
            const auto tf = tf_buffer_.lookupTransform(
                planning_frame_, marker_msg.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(marker_msg, marker_in_planning, tf);
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
                                 "TF %s -> %s unavailable: %s",
                                 marker_msg.header.frame_id.c_str(),
                                 planning_frame_.c_str(), e.what());
            return std::nullopt;
        }

        tf2::Transform t_planning_marker;
        tf2::fromMsg(marker_in_planning.pose, t_planning_marker);

        // Goal in the marker frame: hover standoff_height above the connector
        // point, tool Z pointing into the surface (marker frame rotated by
        // 180 deg about X), optionally yawed about the marker normal.
        tf2::Vector3 goal_position(
            params_.connector_offset_x,
            params_.connector_offset_y,
            params_.connector_offset_z + params_.standoff_height_m);
        tf2::Quaternion q_flip, q_yaw;
        q_flip.setRPY(M_PI, 0.0, 0.0);
        q_yaw.setRPY(0.0, 0.0, params_.tool_yaw_offset_deg * M_PI / 180.0);
        tf2::Transform t_marker_goal(q_yaw * q_flip, goal_position);

        return t_planning_marker * t_marker_goal;
    }

    tf2::Transform current_tcp_pose()
    {
        tf2::Transform t;
        tf2::fromMsg(move_group_.getCurrentPose(eef_link_).pose, t);
        return t;
    }

    // 6-DOF error between current TCP pose and goal.
    static void pose_error(const tf2::Transform &current, const tf2::Transform &goal,
                           tf2::Vector3 &pos_err, tf2::Quaternion &rot_err, double &rot_angle)
    {
        pos_err = goal.getOrigin() - current.getOrigin();
        rot_err = goal.getRotation() * current.getRotation().inverse();
        rot_err.normalize();
        rot_angle = rot_err.getAngle();  // in [0, 2*pi]
        if (rot_angle > M_PI) {
            rot_angle = 2.0 * M_PI - rot_angle;
        }
    }

    // Clamp the pose error to per-cycle limits and return the commanded pose.
    static tf2::Transform clamped_target(const tf2::Transform &current,
                                         const tf2::Vector3 &pos_err,
                                         const tf2::Quaternion &rot_err,
                                         double max_step_m, double max_step_rad)
    {
        tf2::Vector3 step = pos_err;
        const double dist = step.length();
        if (dist > max_step_m) {
            step *= max_step_m / dist;
        }

        double angle = rot_err.getAngle();
        tf2::Vector3 axis = rot_err.getAxis();
        if (angle > M_PI) {  // take the short way
            angle = 2.0 * M_PI - angle;
            axis = -axis;
        }
        tf2::Quaternion q_step;
        if (angle > 1e-6) {
            q_step.setRotation(axis, std::min(angle, max_step_rad));
        } else {
            q_step = tf2::Quaternion::getIdentity();
        }

        tf2::Transform target;
        target.setOrigin(current.getOrigin() + step);
        target.setRotation((q_step * current.getRotation()).normalized());
        return target;
    }

    bool plan_is_sane(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        const auto &points = plan.trajectory_.joint_trajectory.points;
        for (size_t i = 1; i < points.size(); ++i) {
            for (size_t j = 0; j < points[i].positions.size(); ++j) {
                const double jump =
                    std::fabs(points[i].positions[j] - points[i - 1].positions[j]);
                if (jump > params_.max_joint_jump_rad) {
                    RCLCPP_ERROR(LOGGER,
                                 "Rejecting plan: joint %zu jumps %.2f rad between "
                                 "trajectory points (limit %.2f).",
                                 j, jump, params_.max_joint_jump_rad);
                    return false;
                }
            }
        }
        return true;
    }

    bool move_lin(const tf2::Transform &target, double speed)
    {
        geometry_msgs::msg::Pose target_msg;
        tf2::toMsg(target, target_msg);

        move_group_.setMaxVelocityScalingFactor(speed);
        move_group_.setPoseTarget(target_msg, eef_link_);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(LOGGER, "LIN planning failed.");
            return false;
        }
        if (!plan_is_sane(plan)) {
            return false;
        }
        if (move_group_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(LOGGER, "LIN execution failed.");
            return false;
        }
        return true;
    }

    void note_plan_result(bool ok)
    {
        if (ok) {
            consecutive_plan_failures_ = 0;
            return;
        }
        if (++consecutive_plan_failures_ >= params_.max_consecutive_plan_failures) {
            RCLCPP_FATAL(LOGGER, "%d consecutive planning/execution failures - holding.",
                         consecutive_plan_failures_);
            set_phase(Phase::FAULT);
        }
    }

    // One state-machine cycle.
    void step()
    {
        if (phase_ == Phase::MATED || phase_ == Phase::FAULT) {
            RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 10000,
                                 "Phase %s - no motion will be commanded.",
                                 phase_name(phase_));
            return;
        }

        const auto marker = fresh_marker_pose();
        if (!marker) {
            if (phase_ != Phase::WAIT_FOR_VISION) {
                RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
                                     "Marker not visible / stale - holding position.");
            }
            aligned_cycles_ = 0;
            return;
        }

        const auto goal = compute_standoff_goal(*marker);
        if (!goal) {
            return;
        }

        if (phase_ == Phase::WAIT_FOR_VISION) {
            set_phase(Phase::ALIGN_COARSE);
        }

        const tf2::Transform current = current_tcp_pose();
        tf2::Vector3 pos_err;
        tf2::Quaternion rot_err;
        double rot_angle = 0.0;
        pose_error(current, *goal, pos_err, rot_err, rot_angle);

        const double pos_dist = pos_err.length();
        const double rot_deg = rot_angle * 180.0 / M_PI;
        RCLCPP_INFO(LOGGER, "[%s] err: %.1f mm, %.2f deg",
                    phase_name(phase_), pos_dist * 1000.0, rot_deg);

        switch (phase_) {
        case Phase::ALIGN_COARSE: {
            if (pos_dist < params_.coarse_pos_tol_m &&
                rot_deg < params_.coarse_rot_tol_deg) {
                set_phase(Phase::ALIGN_FINE);
                return;
            }
            const auto target = clamped_target(
                current, pos_err, rot_err,
                params_.coarse_max_step_m,
                params_.coarse_max_step_deg * M_PI / 180.0);
            note_plan_result(move_lin(target, params_.coarse_speed));
            return;
        }
        case Phase::ALIGN_FINE: {
            if (pos_dist < params_.fine_pos_tol_m &&
                rot_deg < params_.fine_rot_tol_deg) {
                if (++aligned_cycles_ >= params_.align_hold_cycles) {
                    set_phase(Phase::INSERT);
                }
                return;
            }
            aligned_cycles_ = 0;
            if (pos_dist > params_.coarse_pos_tol_m * 2.0) {
                // Target drifted far away (e.g. operator moved the jig).
                set_phase(Phase::ALIGN_COARSE);
                return;
            }
            const auto target = clamped_target(
                current, pos_err, rot_err,
                params_.fine_max_step_m,
                params_.fine_max_step_deg * M_PI / 180.0);
            note_plan_result(move_lin(target, params_.fine_speed));
            return;
        }
        case Phase::INSERT: {
            // Committed stroke: alignment was verified with fresh vision for
            // align_hold_cycles. Descend along the tool Z axis with the
            // orientation locked; the gripper occluding the marker from here
            // on is expected and does not abort the stroke.
            RCLCPP_INFO(LOGGER, "Committing insertion: %.1f mm along tool Z at %.0f%% speed.",
                        params_.insertion_depth_m * 1000.0, params_.insert_speed * 100.0);
            const tf2::Vector3 tool_z =
                tf2::quatRotate(current.getRotation(), tf2::Vector3(0, 0, 1));
            tf2::Transform target = current;
            target.setOrigin(current.getOrigin() + tool_z * params_.insertion_depth_m);
            if (move_lin(target, params_.insert_speed)) {
                RCLCPP_INFO(LOGGER, "Insertion stroke complete - connector mated.");
                set_phase(Phase::MATED);
            } else {
                note_plan_result(false);
                // Not yet in contact according to plan failure; re-verify
                // alignment before trying again.
                aligned_cycles_ = 0;
                if (phase_ != Phase::FAULT) {
                    set_phase(Phase::ALIGN_FINE);
                }
            }
            return;
        }
        default:
            return;
        }
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface &move_group_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

    Params params_;
    std::string planning_frame_;
    std::string eef_link_;

    std::mutex pose_mutex_;
    std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
    std::chrono::steady_clock::time_point latest_pose_arrival_;

    Phase phase_{Phase::WAIT_FOR_VISION};
    int aligned_cycles_{0};
    int consecutive_plan_failures_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("connector_mating_node", node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    std::string planning_group = "rv5as";
    node->get_parameter_or("planning_group", planning_group, planning_group);
    moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

    std::string eef_link;
    if (node->get_parameter_or("EEF_FRAME_ID", eef_link, std::string()) && !eef_link.empty()) {
        move_group.setEndEffectorLink(eef_link);
    }

    ConnectorMatingController controller(node, move_group);
    controller.run();  // blocks until rclcpp::ok() is false

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
