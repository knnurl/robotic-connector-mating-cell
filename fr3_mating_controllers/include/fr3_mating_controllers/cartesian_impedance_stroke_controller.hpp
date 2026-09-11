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
// the stroke. D is derived from K by damping_ratio (critically damped by
// default), so tuning is a single stiffness vector.
//
// The equilibrium pose arrives on ~/equilibrium_pose (PoseStamped, base
// frame) and is SLEW-LIMITED before use - a pattern ported from the
// fr3_backend testbed's joint-impedance loop: any topic jump becomes a
// smooth bounded torque ramp, never a reflex. Torques are additionally
// rate-limited (<= 1 Nm per 1 ms cycle, the FCI discontinuity limit).
//
// float_mode:=true skips the task law entirely (coriolis only): the arm
// free-floats under gravity compensation. That is build-step 1's "prove
// the RT loop" skeleton, kept as a permanent commissioning switch.
//
// The phase machine in move_l stays the brain: it activates this
// controller for INSERT (insert_backend: impedance), ramps the setpoint
// along the tool axis, watches the external wrench, and switches back to
// the trajectory controller afterwards. This controller knows nothing
// about connectors - it tracks an equilibrium, compliantly.
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

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

private:
    void update_joint_states();
    // Advance the slewed equilibrium toward the latest commanded target by
    // at most (slew limit * dt) - translation and rotation independently.
    void slew_equilibrium(double dt);
    Vector7d saturate_torque_rate(const Vector7d &tau_desired);

    static constexpr int kNumJoints = 7;
    const std::string k_robot_model_interface_name{"robot_model"};
    const std::string k_robot_state_interface_name{"robot_state"};

    std::string arm_id_{"fr3"};
    bool float_mode_{false};
    Eigen::Vector3d k_pos_tool_{150.0, 150.0, 800.0};   // N/m   (tool x, y, z)
    Eigen::Vector3d k_rot_tool_{10.0, 10.0, 20.0};      // Nm/rad (tool r, p, y)
    double damping_ratio_{1.0};
    double nullspace_stiffness_{5.0};                   // Nm/rad
    double setpoint_slew_mps_{0.05};
    double setpoint_slew_rps_{0.5};
    double tau_rate_limit_{1.0};                        // Nm per cycle (1 ms)

    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

    Vector7d q_ = Vector7d::Zero();
    Vector7d dq_ = Vector7d::Zero();
    Vector7d q_nullspace_ = Vector7d::Zero();  // posture seed (activation q)
    Vector7d tau_last_ = Vector7d::Zero();

    // Latest commanded target (subscription thread) -> slewed equilibrium
    // (RT thread). The mutex guards only the small target copy.
    std::mutex target_mutex_;
    Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_orientation_ = Eigen::Quaterniond::Identity();
    bool have_target_{false};

    Eigen::Vector3d equilibrium_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond equilibrium_orientation_ = Eigen::Quaterniond::Identity();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace fr3_mating_controllers
