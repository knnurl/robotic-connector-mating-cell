#include "fr3_mating_controllers/cartesian_impedance_stroke_controller.hpp"

#include <algorithm>
#include <cmath>

#include <franka/model.h>

namespace fr3_mating_controllers
{

namespace
{
// Read at configure time only. Accepting them while configured would leave
// the node reporting values the loop is not using.
constexpr std::array<const char *, 7> kConfigureOnlyParams{
    "arm_id", "max_force_n", "max_torque_nm", "tau_max_nm", "tau_rate_limit",
    "setpoint_slew_mps", "setpoint_slew_rps"};
}  // namespace

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
        auto_declare<double>("max_force_n", max_force_n_);
        auto_declare<double>("max_torque_nm", max_torque_nm_);
        auto_declare<std::vector<double>>(
            "tau_max_nm", {tau_max_nm_(0), tau_max_nm_(1), tau_max_nm_(2),
                           tau_max_nm_(3), tau_max_nm_(4), tau_max_nm_(5),
                           tau_max_nm_(6)});
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CartesianImpedanceStrokeController::CallbackReturn
CartesianImpedanceStrokeController::on_configure(const rclcpp_lifecycle::State &)
{
    const auto logger = get_node()->get_logger();
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    float_mode_ = get_node()->get_parameter("float_mode").as_bool();
    const auto kp = get_node()->get_parameter("k_pos_tool").as_double_array();
    const auto kr = get_node()->get_parameter("k_rot_tool").as_double_array();
    if (kp.size() != 3 || kr.size() != 3) {
        RCLCPP_ERROR(logger, "k_pos_tool / k_rot_tool must each have 3 entries");
        return CallbackReturn::ERROR;
    }
    k_pos_tool_ = Eigen::Vector3d(kp[0], kp[1], kp[2]);
    k_rot_tool_ = Eigen::Vector3d(kr[0], kr[1], kr[2]);
    damping_ratio_ = get_node()->get_parameter("damping_ratio").as_double();
    nullspace_stiffness_ = get_node()->get_parameter("nullspace_stiffness").as_double();
    setpoint_slew_mps_ = get_node()->get_parameter("setpoint_slew_mps").as_double();
    setpoint_slew_rps_ = get_node()->get_parameter("setpoint_slew_rps").as_double();
    tau_rate_limit_ = get_node()->get_parameter("tau_rate_limit").as_double();
    max_force_n_ = get_node()->get_parameter("max_force_n").as_double();
    max_torque_nm_ = get_node()->get_parameter("max_torque_nm").as_double();

    const std::string gains_error = detail::validate_gains(
        k_pos_tool_, k_rot_tool_, damping_ratio_, nullspace_stiffness_);
    if (!gains_error.empty()) {
        RCLCPP_ERROR(logger, "refusing to configure: %s", gains_error.c_str());
        return CallbackReturn::ERROR;
    }
    const auto tmax = get_node()->get_parameter("tau_max_nm").as_double_array();
    if (tmax.size() != static_cast<size_t>(kNumJoints)) {
        RCLCPP_ERROR(logger, "tau_max_nm must have 7 entries");
        return CallbackReturn::ERROR;
    }
    std::array<double, 7> tau_max{};
    std::copy(tmax.begin(), tmax.end(), tau_max.begin());
    const std::string limits_error = detail::validate_limits(
        max_force_n_, max_torque_nm_, tau_rate_limit_, setpoint_slew_mps_,
        setpoint_slew_rps_, tau_max);
    if (!limits_error.empty()) {
        RCLCPP_ERROR(logger, "refusing to configure: %s", limits_error.c_str());
        return CallbackReturn::ERROR;
    }
    for (int i = 0; i < kNumJoints; ++i) {
        tau_max_nm_(i) = tau_max[i];
    }

    pending_k_pos_tool_ = k_pos_tool_;
    pending_k_rot_tool_ = k_rot_tool_;
    pending_damping_ratio_ = damping_ratio_;
    pending_nullspace_stiffness_ = nullspace_stiffness_;
    params_dirty_ = false;

    // Live tuning: the ladder adjusts stiffness by +/-50 N/m and toggles
    // float mode between steps, and deactivating to reconfigure would drop
    // the arm to the trajectory controller in between. The RT loop picks
    // these up through adopt_pending_params().
    param_cb_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            std::lock_guard<std::mutex> lock(params_mutex_);

            // Build the candidate set and apply NOTHING until all of it is
            // valid: a half-applied set would run gains nobody asked for.
            Eigen::Vector3d kp_new = pending_k_pos_tool_;
            Eigen::Vector3d kr_new = pending_k_rot_tool_;
            double zeta_new = pending_damping_ratio_;
            double ns_new = pending_nullspace_stiffness_;
            bool gains_touched = false;
            int float_request = -1;  // -1 untouched, 0 off, 1 on

            for (const auto &p : params) {
                const auto &name = p.get_name();
                for (const char *fixed : kConfigureOnlyParams) {
                    if (name == fixed) {
                        result.successful = false;
                        result.reason = name + " is read at configure time only; "
                                               "clean up the controller, set it, then configure again";
                        return result;
                    }
                }
                if (name == "float_mode") {
                    float_request = p.as_bool() ? 1 : 0;
                } else if (name == "damping_ratio") {
                    zeta_new = p.as_double();
                    gains_touched = true;
                } else if (name == "nullspace_stiffness") {
                    ns_new = p.as_double();
                    gains_touched = true;
                } else if (name == "k_pos_tool" || name == "k_rot_tool") {
                    const auto v = p.as_double_array();
                    if (v.size() != 3) {
                        result.successful = false;
                        result.reason = name + " must have 3 entries";
                        return result;
                    }
                    (name == "k_pos_tool" ? kp_new : kr_new) = Eigen::Vector3d(v[0], v[1], v[2]);
                    gains_touched = true;
                }
            }

            if (gains_touched) {
                const std::string err = detail::validate_gains(kp_new, kr_new, zeta_new, ns_new);
                if (!err.empty()) {
                    result.successful = false;
                    result.reason = err;
                    return result;
                }
                pending_k_pos_tool_ = kp_new;
                pending_k_rot_tool_ = kr_new;
                pending_damping_ratio_ = zeta_new;
                pending_nullspace_stiffness_ = ns_new;
                params_dirty_ = true;
            }
            if (float_request >= 0) {
                const bool on = float_request == 1;
                // Logged here, off the RT thread, not where the loop acts on it.
                if (on != float_mode_.load()) {
                    RCLCPP_INFO(get_node()->get_logger(),
                                on ? "float mode ON - coriolis only, the arm is free"
                                   : "float mode OFF - holding wherever the arm is when "
                                     "the loop sees the change");
                }
                float_mode_ = on;
            }
            return result;
        });

    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        franka_semantic_components::FrankaRobotModel(
            arm_id_ + "/" + k_robot_model_interface_name,
            arm_id_ + "/" + k_robot_state_interface_name));

    pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/equilibrium_pose", rclcpp::QoS(1),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            const Eigen::Vector3d p(msg->pose.position.x, msg->pose.position.y,
                                    msg->pose.position.z);
            const Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                                       msg->pose.orientation.y, msg->pose.orientation.z);
            // An unset orientation is all zeros, and normalising it gives NaN,
            // which would zero the task torque and let the arm float mid-hold.
            // Drop anything that is not a real pose.
            const double qn = q.norm();
            if (!p.allFinite() || !std::isfinite(qn) || std::abs(qn - 1.0) > 0.1) {
                RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                     "ignoring malformed equilibrium_pose (finite position: "
                                     "%s, |q| = %.3f)", p.allFinite() ? "yes" : "no", qn);
                return;
            }
            targets_.publish(p, q.normalized());
        });

    RCLCPP_INFO(logger,
                "Configured: arm_id=%s float_mode=%s K_pos_tool=[%.0f %.0f %.0f] N/m "
                "K_rot_tool=[%.1f %.1f %.1f] Nm/rad zeta=%.2f ceilings %.0f N / %.0f Nm",
                arm_id_.c_str(), float_mode_ ? "true" : "false",
                k_pos_tool_.x(), k_pos_tool_.y(), k_pos_tool_.z(),
                k_rot_tool_.x(), k_rot_tool_.y(), k_rot_tool_.z(), damping_ratio_,
                max_force_n_, max_torque_nm_);
    return CallbackReturn::SUCCESS;
}

CartesianImpedanceStrokeController::CallbackReturn
CartesianImpedanceStrokeController::on_activate(const rclcpp_lifecycle::State &)
{
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    // Equilibrium at the CURRENT pose (fr3_backend pattern): the first
    // cycle commands zero spring force, and torque rate limiting ramps in
    // from zero - no step, no reflex.
    seed_equilibrium_here();
    was_floating_ = float_mode_.load();
    tau_last_.setZero();

    RCLCPP_INFO(get_node()->get_logger(),
                "Active%s - holding at [%.3f %.3f %.3f]. Ceilings: %.0f N, "
                "%.0f Nm.",
                was_floating_ ? " (FLOAT MODE: coriolis only)" : "",
                equilibrium_position_.x(), equilibrium_position_.y(),
                equilibrium_position_.z(), max_force_n_, max_torque_nm_);
    return CallbackReturn::SUCCESS;
}

CartesianImpedanceStrokeController::CallbackReturn
CartesianImpedanceStrokeController::on_deactivate(const rclcpp_lifecycle::State &)
{
    // Leave nothing behind: a stale torque in the command interface would
    // otherwise be whatever the last cycle asked for.
    for (int i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(0.0);
    }
    tau_last_.setZero();
    franka_robot_model_->release_interfaces();
    return CallbackReturn::SUCCESS;
}

CartesianImpedanceStrokeController::CallbackReturn
CartesianImpedanceStrokeController::on_cleanup(const rclcpp_lifecycle::State &)
{
    // Drop the live-parameter guard and the setpoint subscription, so the
    // configure-time parameters can be changed before the next configure,
    // which re-validates everything.
    param_cb_.reset();
    pose_sub_.reset();
    return CallbackReturn::SUCCESS;
}

void CartesianImpedanceStrokeController::seed_equilibrium_here()
{
    update_joint_states();
    q_nullspace_ = q_;

    const std::array<double, 16> pose =
        franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    const Eigen::Map<const Eigen::Matrix4d> t(pose.data());
    equilibrium_position_ = t.block<3, 1>(0, 3);
    equilibrium_orientation_ = Eigen::Quaterniond(t.block<3, 3>(0, 0)).normalized();
    target_position_ = equilibrium_position_;
    target_orientation_ = equilibrium_orientation_;
    have_target_ = false;     // hold here until a NEW setpoint arrives
    targets_.invalidate();    // anything published before now is stale
}

void CartesianImpedanceStrokeController::adopt_pending_params()
{
    if (!params_dirty_.load()) {
        return;
    }
    std::unique_lock<std::mutex> lock(params_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        return;  // writer is mid-update; try again next cycle
    }
    k_pos_tool_ = pending_k_pos_tool_;
    k_rot_tool_ = pending_k_rot_tool_;
    damping_ratio_ = pending_damping_ratio_;
    nullspace_stiffness_ = pending_nullspace_stiffness_;
    params_dirty_ = false;
}

void CartesianImpedanceStrokeController::write_torque(const Vector7d &tau)
{
    for (int i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(tau(i));
    }
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
    Eigen::Vector3d taken_p;
    Eigen::Quaterniond taken_q;
    if (targets_.try_take(taken_p, taken_q)) {
        target_position_ = taken_p;
        target_orientation_ = taken_q;
        have_target_ = true;
    }
    if (!have_target_) {
        return;  // no setpoint yet: keep holding the seeded pose
    }
    const Eigen::Vector3d target_p = target_position_;
    Eigen::Quaterniond target_q = target_orientation_;

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
    adopt_pending_params();

    const std::array<double, 7> coriolis_array =
        franka_robot_model_->getCoriolisForceVector();
    const Eigen::Map<const Vector7d> coriolis(coriolis_array.data());

    // Never build a torque out of a bad number: one non-finite value would
    // otherwise land in tau_last_ and poison every later cycle.
    if (!q_.allFinite() || !dq_.allFinite() || !coriolis.allFinite()) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                              1000, "non-finite robot state - commanding zero");
        write_torque(saturate_torque_rate(Vector7d::Zero()));
        return controller_interface::return_type::OK;
    }

    const bool floating = float_mode_.load();
    if (was_floating_ && !floating) {
        // float -> hold, the ladder's first real risk: the arm has been
        // moved by hand, so the activation equilibrium is somewhere else.
        // Hold HERE instead of snapping back to it. (Logged by the
        // parameter callback, not here in the 1 kHz loop.)
        seed_equilibrium_here();
    }
    was_floating_ = floating;

    if (floating) {
        // Build-step-1 skeleton, kept as a commissioning switch: the arm
        // free-floats (libfranka adds gravity; we add coriolis only).
        write_torque(saturate_torque_rate(clamp_joint_torque(coriolis)));
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

    // The commanded wrench, BOUNDED. Stiffness times error alone is
    // unbounded: hold the arm still while the equilibrium slews away at
    // 5 cm/s and 800 N/m reaches 80 N in two seconds and keeps climbing.
    // The clamp covers the TOTAL (damping included) on purpose: clamping only
    // the spring and adding damping afterwards would let D*v exceed the
    // ceiling at speed. With the default gains it only engages beyond
    // ~3.75 cm of tool-Z error or ~20 cm laterally.
    Vector6d wrench;
    wrench.head(3) = stiffness.topLeftCorner(3, 3) * error.head(3) -
                     damping.topLeftCorner(3, 3) * velocity.head(3);
    wrench.tail(3) = stiffness.bottomRightCorner(3, 3) * error.tail(3) -
                     damping.bottomRightCorner(3, 3) * velocity.tail(3);
    const double force = wrench.head(3).norm();
    if (force > max_force_n_ && force > 0.0) {
        wrench.head(3) *= max_force_n_ / force;
    }
    const double moment = wrench.tail(3).norm();
    if (moment > max_torque_nm_ && moment > 0.0) {
        wrench.tail(3) *= max_torque_nm_ / moment;
    }
    const Vector7d tau_task = jacobian.transpose() * wrench;

    // Nullspace posture hold: keep the elbow near the activation posture
    // without fighting the task (damped-pseudoinverse projector; its norm
    // stays <= 1 in every pose, so it cannot amplify near a singularity).
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

    Vector7d tau_raw = tau_task + tau_nullspace + coriolis;
    if (!tau_raw.allFinite()) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                              1000, "non-finite torque - commanding zero");
        tau_raw.setZero();
    }
    write_torque(saturate_torque_rate(clamp_joint_torque(tau_raw)));
    return controller_interface::return_type::OK;
}

CartesianImpedanceStrokeController::Vector7d
CartesianImpedanceStrokeController::clamp_joint_torque(const Vector7d &tau) const
{
    Vector7d out;
    for (int i = 0; i < kNumJoints; ++i) {
        out(i) = std::clamp(tau(i), -tau_max_nm_(i), tau_max_nm_(i));
    }
    return out;
}

}  // namespace fr3_mating_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fr3_mating_controllers::CartesianImpedanceStrokeController,
                       controller_interface::ControllerInterface)
