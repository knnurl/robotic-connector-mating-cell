// Cartesian-impedance insertion backend for the connector-mating cell.
//
// A franka_ros2 ControllerInterface plugin: claims the seven joint effort
// command interfaces plus Franka's robot_model/robot_state semantic state
// interfaces, and runs at the hardware update rate (1 kHz over FCI):
//
//   tau = J^T ( K (x_d - x)  -  D (J dq) )  +  tau_nullspace  +  coriolis
//   [libfranka adds gravity compensation on top of any commanded torque]
//
// K and D are diagonal in the TOOL frame of the desired pose and rotated
// into the base frame each cycle: soft lateral X/Y + roll/pitch lets the
// connector self-align into its socket under contact, firm tool-Z drives
// the stroke. D = 2*zeta*sqrt(K): critical damping for a 1 kg mass. The
// FR3's apparent Cartesian mass is a few kg, so the effective damping ratio
// is about 0.5-0.7 - one small overshoot is expected, oscillation is not.
//
// The equilibrium pose arrives on ~/equilibrium_pose (PoseStamped, base
// frame) and is SLEW-LIMITED before use - a pattern ported from the
// fr3_backend testbed's joint-impedance loop: any topic jump becomes a
// smooth bounded torque ramp, never a reflex. Malformed setpoints
// (non-finite, or an orientation that is not a rotation) are dropped.
//
// Bounds that hold whatever the gains are: the commanded wrench is capped
// (max_force_n / max_torque_nm), joint torques are capped (tau_max_nm, never
// above the FR3 limits) and rate-limited (<= 1 Nm per 1 ms cycle, the FCI
// discontinuity limit), and non-finite state or torque commands zero.
//
// float_mode:=true skips the task law entirely (coriolis only): the arm
// free-floats under gravity compensation. That is build-step 1's "prove
// the RT loop" skeleton, kept as a permanent commissioning switch. It is
// live-tunable, and switching it OFF re-seeds the equilibrium where the arm
// is NOW - after floating the arm by hand, the activation equilibrium is
// somewhere else and holding to it would snap the arm back.
//
// The gains are live-tunable too, within detail::GainLimits
// (impedance_detail.hpp), as is the setpoint slew pair within
// detail::ConfigLimits; a set with any value out of range is rejected whole.
// The five configure-time parameters - arm_id, max_force_n, max_torque_nm,
// tau_max_nm, tau_rate_limit - cannot be changed while the controller is
// configured: clean it up, set them, configure again. Nothing in
// update() blocks on a lock a non-RT thread holds, and nothing in it logs
// outside fault paths.
//
// The phase machine in mating_node stays the brain: it activates this
// controller for INSERT (insert_backend: impedance), ramps the setpoint
// along the tool axis, watches the external wrench, and switches back to
// the trajectory controller afterwards. This controller knows nothing
// about connectors - it tracks an equilibrium, compliantly.
#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_mating_controllers/impedance_detail.hpp"

namespace fr3_mating_controllers
{

class CartesianImpedanceStrokeController
    : public controller_interface::ControllerInterface
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

    [[nodiscard]] controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

private:
    void update_joint_states();
    // Advance the slewed equilibrium toward the latest commanded target by
    // at most (slew limit * dt) - translation and rotation independently.
    void slew_equilibrium(double dt);
    Vector7d saturate_torque_rate(const Vector7d &tau_desired);
    // Put the equilibrium (and the posture seed) where the arm is NOW, and
    // discard every setpoint published before this moment. Used on
    // activation and whenever float mode is switched off.
    void seed_equilibrium_here();
    // Adopt live gain and slew changes, RT-safely: try_lock, and simply keep
    // the previous values for one cycle if the writer holds the lock.
    void adopt_pending_params();
    void write_torque(const Vector7d &tau);
    Vector7d clamp_joint_torque(const Vector7d &tau) const;

    static constexpr int kNumJoints = 7;
    const std::string k_robot_model_interface_name{"robot_model"};
    const std::string k_robot_state_interface_name{"robot_state"};

    std::string arm_id_{"fr3"};
    std::atomic<bool> float_mode_{false};
    bool was_floating_{false};
    Eigen::Vector3d k_pos_tool_{150.0, 150.0, 800.0};   // N/m   (tool x, y, z)
    Eigen::Vector3d k_rot_tool_{10.0, 10.0, 20.0};      // Nm/rad (tool r, p, y)
    double damping_ratio_{1.0};
    double nullspace_stiffness_{5.0};                   // Nm/rad
    double setpoint_slew_mps_{0.05};
    double setpoint_slew_rps_{0.5};
    double tau_rate_limit_{1.0};                        // Nm per cycle (1 ms)
    // Hard ceilings on what the task law may ask for, whatever the pose
    // error is. Without these a blocked arm and a slewing equilibrium make
    // force grow without bound (800 N/m x 10 cm = 80 N, and climbing).
    double max_force_n_{30.0};
    double max_torque_nm_{10.0};
    // Per-joint ceiling, default = the FR3's own limits, so a command can
    // never exceed spec even if everything else is wrong.
    Vector7d tau_max_nm_ =
        (Vector7d() << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0).finished();

    // Live-tunable set (see adopt_pending_params). Written by the parameter
    // callback under params_mutex_, read by the RT loop.
    std::mutex params_mutex_;
    std::atomic<bool> params_dirty_{false};
    Eigen::Vector3d pending_k_pos_tool_{k_pos_tool_};
    Eigen::Vector3d pending_k_rot_tool_{k_rot_tool_};
    double pending_damping_ratio_{damping_ratio_};
    double pending_nullspace_stiffness_{nullspace_stiffness_};
    double pending_setpoint_slew_mps_{setpoint_slew_mps_};
    double pending_setpoint_slew_rps_{setpoint_slew_rps_};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

    Vector7d q_ = Vector7d::Zero();
    Vector7d dq_ = Vector7d::Zero();
    Vector7d q_nullspace_ = Vector7d::Zero();  // posture seed (activation q)
    Vector7d tau_last_ = Vector7d::Zero();

    // Setpoints: published by the subscription thread, taken by the RT loop
    // without blocking (see detail::TargetHandoff).
    detail::TargetHandoff targets_;
    // RT-thread only: the latest target taken from targets_.
    Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_orientation_ = Eigen::Quaterniond::Identity();
    bool have_target_{false};

    Eigen::Vector3d equilibrium_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond equilibrium_orientation_ = Eigen::Quaterniond::Identity();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace fr3_mating_controllers
