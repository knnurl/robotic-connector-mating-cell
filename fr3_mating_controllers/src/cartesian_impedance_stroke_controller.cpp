#include "fr3_mating_controllers/cartesian_impedance_stroke_controller.hpp"

#include <algorithm>
#include <cmath>

#include <franka/model.h>

namespace fr3_mating_controllers
{

controller_interface::InterfaceConfiguration
CartesianImpedanceStrokeController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= kNumJoints; ++i) {
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
    return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceStrokeController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= kNumJoints; ++i) {
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    }
    for (int i = 1; i <= kNumJoints; ++i) {
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    }
    for (const auto &name : franka_robot_model_->get_state_interface_names()) {
        config.names.push_back(name);
    }
    return config;
}

CartesianImpedanceStrokeController::CallbackReturn
CartesianImpedanceStrokeController::on_init()
{
    try {
        auto_declare<std::string>("arm_id", arm_id_);
        auto_declare<bool>("float_mode", float_mode_);
        auto_declare<std::vector<double>>(
            "k_pos_tool", {k_pos_tool_.x(), k_pos_tool_.y(), k_pos_tool_.z()});
        auto_declare<std::vector<double>>(
            "k_rot_tool", {k_rot_tool_.x(), k_rot_tool_.y(), k_rot_tool_.z()});
        auto_declare<double>("damping_ratio", damping_ratio_);
        auto_declare<double>("nullspace_stiffness", nullspace_stiffness_);
        auto_declare<double>("setpoint_slew_mps", setpoint_slew_mps_);
        auto_declare<double>("setpoint_slew_rps", setpoint_slew_rps_);
        auto_declare<double>("tau_rate_limit", tau_rate_limit_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CartesianImpedanceStrokeController::CallbackReturn
CartesianImpedanceStrokeController::on_configure(const rclcpp_lifecycle::State &)
{
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    float_mode_ = get_node()->get_parameter("float_mode").as_bool();
    const auto kp = get_node()->get_parameter("k_pos_tool").as_double_array();
    const auto kr = get_node()->get_parameter("k_rot_tool").as_double_array();
    if (kp.size() != 3 || kr.size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "k_pos_tool / k_rot_tool must each have 3 entries");
        return CallbackReturn::ERROR;
    }
    k_pos_tool_ = Eigen::Vector3d(kp[0], kp[1], kp[2]);
    k_rot_tool_ = Eigen::Vector3d(kr[0], kr[1], kr[2]);
    damping_ratio_ = get_node()->get_parameter("damping_ratio").as_double();
    nullspace_stiffness_ = get_node()->get_parameter("nullspace_stiffness").as_double();
    setpoint_slew_mps_ = get_node()->get_parameter("setpoint_slew_mps").as_double();
    setpoint_slew_rps_ = get_node()->get_parameter("setpoint_slew_rps").as_double();
    tau_rate_limit_ = get_node()->get_parameter("tau_rate_limit").as_double();

    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        franka_semantic_components::FrankaRobotModel(
            arm_id_ + "/" + k_robot_model_interface_name,
            arm_id_ + "/" + k_robot_state_interface_name));

    pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/equilibrium_pose", rclcpp::QoS(1),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(target_mutex_);
            target_position_ = Eigen::Vector3d(msg->pose.position.x,
                                               msg->pose.position.y,
                                               msg->pose.position.z);
            target_orientation_ = Eigen::Quaterniond(
                msg->pose.orientation.w, msg->pose.orientation.x,
                msg->pose.orientation.y, msg->pose.orientation.z).normalized();
            have_target_ = true;
        });

    RCLCPP_INFO(get_node()->get_logger(),
                "Configured: arm_id=%s float_mode=%s K_pos_tool=[%.0f %.0f %.0f] N/m "
                "K_rot_tool=[%.1f %.1f %.1f] Nm/rad zeta=%.2f",
                arm_id_.c_str(), float_mode_ ? "true" : "false",
                k_pos_tool_.x(), k_pos_tool_.y(), k_pos_tool_.z(),
                k_rot_tool_.x(), k_rot_tool_.y(), k_rot_tool_.z(), damping_ratio_);
    return CallbackReturn::SUCCESS;
}

CartesianImpedanceStrokeController::CallbackReturn
CartesianImpedanceStrokeController::on_activate(const rclcpp_lifecycle::State &)
{
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    update_joint_states();
    q_nullspace_ = q_;

    // Seed the equilibrium at the CURRENT end-effector pose (fr3_backend
    // pattern): the first cycle commands zero spring force, and torque
    // rate limiting ramps in from zero - no step, no reflex.
    const std::array<double, 16> pose =
        franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    const Eigen::Map<const Eigen::Matrix4d> t(pose.data());
    equilibrium_position_ = t.block<3, 1>(0, 3);
    equilibrium_orientation_ = Eigen::Quaterniond(t.block<3, 3>(0, 0)).normalized();
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_position_ = equilibrium_position_;
        target_orientation_ = equilibrium_orientation_;
        have_target_ = false;  // hold here until a setpoint arrives
    }
    tau_last_.setZero();

    RCLCPP_INFO(get_node()->get_logger(),
                "Active%s - holding at [%.3f %.3f %.3f].",
                float_mode_ ? " (FLOAT MODE: coriolis only)" : "",
                equilibrium_position_.x(), equilibrium_position_.y(),
                equilibrium_position_.z());
    return CallbackReturn::SUCCESS;
}

CartesianImpedanceStrokeController::CallbackReturn
CartesianImpedanceStrokeController::on_deactivate(const rclcpp_lifecycle::State &)
{
    franka_robot_model_->release_interfaces();
    return CallbackReturn::SUCCESS;
}

void CartesianImpedanceStrokeController::update_joint_states()
{
    for (int i = 0; i < kNumJoints; ++i) {
        q_(i) = state_interfaces_.at(i).get_value();
        dq_(i) = state_interfaces_.at(kNumJoints + i).get_value();
    }
}

void CartesianImpedanceStrokeController::slew_equilibrium(double dt)
{
    Eigen::Vector3d target_p;
    Eigen::Quaterniond target_q;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (!have_target_) {
            return;  // no setpoint yet: keep holding the activation pose
        }
        target_p = target_position_;
        target_q = target_orientation_;
    }

    // Translation: bounded step toward the target.
    const Eigen::Vector3d dp = target_p - equilibrium_position_;
    const double dist = dp.norm();
    const double max_step = setpoint_slew_mps_ * dt;
    equilibrium_position_ += (dist <= max_step || dist < 1e-12)
                                 ? dp
                                 : Eigen::Vector3d(dp * (max_step / dist));

    // Rotation: bounded-angle slerp toward the target.
    if (target_q.coeffs().dot(equilibrium_orientation_.coeffs()) < 0.0) {
        target_q.coeffs() = -target_q.coeffs();  // shortest path
    }
    const double angle = equilibrium_orientation_.angularDistance(target_q);
    const double max_angle = setpoint_slew_rps_ * dt;
    equilibrium_orientation_ =
        (angle <= max_angle || angle < 1e-12)
            ? target_q
            : equilibrium_orientation_.slerp(max_angle / angle, target_q);
    equilibrium_orientation_.normalize();
}

CartesianImpedanceStrokeController::Vector7d
CartesianImpedanceStrokeController::saturate_torque_rate(const Vector7d &tau_desired)
{
    Vector7d tau;
    for (int i = 0; i < kNumJoints; ++i) {
        const double delta = tau_desired(i) - tau_last_(i);
        tau(i) = tau_last_(i) + std::clamp(delta, -tau_rate_limit_, tau_rate_limit_);
    }
    tau_last_ = tau;
    return tau;
}

controller_interface::return_type
CartesianImpedanceStrokeController::update(const rclcpp::Time &,
                                           const rclcpp::Duration &period)
{
    update_joint_states();

    const std::array<double, 7> coriolis_array =
        franka_robot_model_->getCoriolisForceVector();
    const Eigen::Map<const Vector7d> coriolis(coriolis_array.data());

    if (float_mode_) {
        // Build-step-1 skeleton, kept as a commissioning switch: the arm
        // free-floats (libfranka adds gravity; we add coriolis only).
        const Vector7d tau = saturate_torque_rate(coriolis);
        for (int i = 0; i < kNumJoints; ++i) {
            command_interfaces_[i].set_value(tau(i));
        }
        return controller_interface::return_type::OK;
    }

    const std::array<double, 42> jacobian_array =
        franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    const Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    const std::array<double, 16> pose_array =
        franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    const Eigen::Map<const Eigen::Matrix4d> transform(pose_array.data());
    const Eigen::Vector3d position = transform.block<3, 1>(0, 3);
    const Eigen::Quaterniond orientation(transform.block<3, 3>(0, 0));

    slew_equilibrium(period.seconds());

    // Pose error (base frame). Rotation error as a rotation vector taking
    // the current orientation to the equilibrium, shortest way.
    Vector6d error;
    error.head(3) = equilibrium_position_ - position;
    Eigen::Quaterniond err_q = equilibrium_orientation_ * orientation.inverse();
    if (err_q.w() < 0.0) {
        err_q.coeffs() = -err_q.coeffs();
    }
    const Eigen::AngleAxisd err_aa(err_q);
    error.tail(3) = err_aa.axis() * err_aa.angle();

    // Stiffness/damping: diagonal in the TOOL axes of the equilibrium
    // orientation, rotated into the base frame each cycle. Soft lateral
    // X/Y + roll/pitch = self-alignment; firm Z = the stroke axis.
    const Eigen::Matrix3d r_d = equilibrium_orientation_.toRotationMatrix();
    Matrix6d stiffness = Matrix6d::Zero();
    Matrix6d damping = Matrix6d::Zero();
    stiffness.topLeftCorner(3, 3) =
        r_d * k_pos_tool_.asDiagonal() * r_d.transpose();
    stiffness.bottomRightCorner(3, 3) =
        r_d * k_rot_tool_.asDiagonal() * r_d.transpose();
    const Eigen::Vector3d d_pos =
        2.0 * damping_ratio_ * k_pos_tool_.cwiseSqrt();
    const Eigen::Vector3d d_rot =
        2.0 * damping_ratio_ * k_rot_tool_.cwiseSqrt();
    damping.topLeftCorner(3, 3) = r_d * d_pos.asDiagonal() * r_d.transpose();
    damping.bottomRightCorner(3, 3) = r_d * d_rot.asDiagonal() * r_d.transpose();

    const Vector6d velocity = jacobian * dq_;
    const Vector7d tau_task =
        jacobian.transpose() * (stiffness * error - damping * velocity);

    // Nullspace posture hold: keep the elbow near the activation posture
    // without fighting the task (damped-pseudoinverse projector).
    const Eigen::Matrix<double, 6, 6> jjt =
        jacobian * jacobian.transpose() +
        1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 7, 6> jt_pinv =
        jacobian.transpose() * jjt.inverse();
    const Eigen::Matrix<double, 7, 7> nullspace_projector =
        Eigen::Matrix<double, 7, 7>::Identity() - jt_pinv * jacobian;
    const Vector7d tau_nullspace =
        nullspace_projector *
        (nullspace_stiffness_ * (q_nullspace_ - q_) -
         2.0 * std::sqrt(nullspace_stiffness_) * dq_);

    const Vector7d tau = saturate_torque_rate(tau_task + tau_nullspace + coriolis);
    for (int i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(tau(i));
    }
    return controller_interface::return_type::OK;
}

}  // namespace fr3_mating_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fr3_mating_controllers::CartesianImpedanceStrokeController,
                       controller_interface::ControllerInterface)
