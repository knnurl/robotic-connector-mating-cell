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
// vision is fresh, alignment is within fine tolerance, and raw (unpredicted)
// detections confirm the marker is actually seen; once MATED the arm is
// latched and no further motion is commanded. A FAULT that interrupts the
// stroke refuses reset until ~/retract has pulled back along the tool axis.
//
// Optional per-robot upgrades (all off by default, see Params):
//   wrench_topic     force-aware insertion - contact force seats/aborts the
//                    stroke instead of running blind to full depth
//   insert_planner   "cartesian" = computeCartesianPath stroke, path-
//                    guaranteed on robots without Pilz LIN (e.g. FR3)
//   align_mode       "servo" = ALIGN phases publish Cartesian twists for
//                    moveit_servo (continuous tracking); INSERT stays a
//                    discrete committed stroke
//
// The pose subscriptions only store the latest sample; all planning and
// execution happens in a dedicated control thread, never in a callback.
// State topics: /mating/phase (latched String), /mating/paused (latched
// Bool), /mating/error_mm, /mating/error_deg, /diagnostics.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "melfa_rv5as_masterclass/mating_geometry.hpp"
#include "melfa_rv5as_masterclass/mating_phase_machine.hpp"

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("connector_mating");

// The sequencing logic itself is pure and unit-tested; see
// mating_phase_machine.hpp and test/test_phase_machine.cpp. This node
// supplies its inputs (time, poses, TF, forces) and executes its actions
// (planning, motion, twist publishing).
using mating_phase_machine::Action;
using mating_phase_machine::Outcome;
using mating_phase_machine::Phase;
using mating_phase_machine::PhaseMachine;
using mating_phase_machine::phase_name;

struct Params
{
    std::string pose_topic{"/aruco/pose"};
    // Unfiltered detections. The filtered pose may be a Kalman prediction
    // during brief dropouts; arming the committed insertion stroke requires
    // a real detection on this topic within vision_timeout_s.
    std::string raw_pose_topic{"/aruco/pose_raw"};
    double control_period_s{0.4};
    double vision_timeout_s{0.6};
    // How long to wait for TF at the image timestamp before falling back to
    // the latest transform.
    double tf_lookup_timeout_s{0.1};

    // Motion backend. Any MoveIt planning pipeline works for alignment, but
    // ALIGN/INSERT assume straight-line TCP motion: prefer a Cartesian/LIN
    // planner (Pilz LIN where available).
    std::string planning_pipeline{"pilz_industrial_motion_planner"};
    std::string planner_id{"LIN"};
    double accel_scaling{0.1};

    // Calibration/teach mode: when false the sequence stops after fine
    // alignment is achieved and only reports; no insertion stroke.
    bool enable_insertion{true};

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

    // --- Force guard for the insertion stroke (empty topic = disabled).
    // Subscribes geometry_msgs/WrenchStamped (e.g. the FR3's estimated
    // external wrench). Axial reaction >= contact_force_n means contact:
    // MATED if at least min_contact_depth_m of stroke was travelled, jam
    // (FAULT + retract required) if earlier. Lateral force >=
    // max_lateral_force_n at any point stops the stroke immediately.
    std::string wrench_topic{""};
    double contact_force_n{8.0};
    double max_lateral_force_n{12.0};
    double min_contact_depth_m{0.0};
    double wrench_timeout_s{0.2};

    // --- Insertion stroke planner. "pipeline" uses the configured MoveIt
    // pipeline (Pilz LIN preferred); "cartesian" uses computeCartesianPath,
    // giving a path-guaranteed straight stroke on Pilz-less robots (FR3).
    std::string insert_planner{"pipeline"};
    double cartesian_eef_step_m{0.002};

    // --- Alignment mode. "step": plan+execute one clamped LIN goal per
    // cycle (default). "servo": publish Cartesian twists for moveit_servo
    // (continuous tracking; INSERT remains a discrete committed stroke).
    // The velocity envelope reuses the step clamps: max_step / period.
    std::string align_mode{"step"};
    std::string servo_twist_topic{"/servo_node/delta_twist_cmds"};
    double servo_pos_gain{1.5};  // (m/s per m of error)
    double servo_rot_gain{1.5};  // (rad/s per rad of error)

    // --- INSERT execution backend. "moveit" (default): plan+execute the
    // stroke via MoveIt (see insert_planner). "impedance": switch to a
    // Cartesian-impedance controller (fr3_mating_controllers), ramp its
    // equilibrium along the tool axis, and judge the outcome purely by
    // the wrench guard - compliant, self-aligning insertion. Requires
    // wrench_topic (no force source = no outcome judgment = refused) and
    // a planning frame equal to the robot base frame the impedance
    // controller works in.
    std::string insert_backend{"moveit"};
    std::string impedance_controller{"cartesian_impedance_stroke_controller"};
    std::string trajectory_controller{"fr3_arm_controller"};
    std::string controller_switch_service{"/controller_manager/switch_controller"};
    std::string equilibrium_topic{
        "/cartesian_impedance_stroke_controller/equilibrium_pose"};
    double impedance_stroke_mps{0.005};   // equilibrium ramp speed
    // Equilibrium lead past full depth: the spring preload that generates
    // the seating force (lead * k_pos_tool_z newtons at full lag).
    double impedance_overdrive_m{0.010};
    double impedance_settle_s{1.0};       // hold after the ramp before judging
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

        mating_phase_machine::Config mc;
        mc.coarse_pos_tol_m = params_.coarse_pos_tol_m;
        mc.coarse_rot_tol_deg = params_.coarse_rot_tol_deg;
        mc.fine_pos_tol_m = params_.fine_pos_tol_m;
        mc.fine_rot_tol_deg = params_.fine_rot_tol_deg;
        mc.align_hold_cycles = params_.align_hold_cycles;
        mc.max_consecutive_plan_failures = params_.max_consecutive_plan_failures;
        machine_.emplace(mc);

        sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            params_.pose_topic, rclcpp::QoS(1),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                latest_pose_ = *msg;
                latest_pose_arrival_ = std::chrono::steady_clock::now();
            });

        // Raw (unpredicted) detections: only used as an arming gate so the
        // insertion stroke can never be committed on filter predictions.
        raw_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            params_.raw_pose_topic, rclcpp::QoS(1),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                latest_raw_stamp_ = rclcpp::Time(msg->header.stamp);
                latest_raw_arrival_ = std::chrono::steady_clock::now();
                has_raw_ = true;
            });

        // External wrench (e.g. FR3 estimated end-effector wrench): turns
        // the insertion stroke from open-loop into force-aware. Optional -
        // robots without a force source leave wrench_topic empty.
        if (!params_.wrench_topic.empty()) {
            wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
                params_.wrench_topic, rclcpp::SensorDataQoS(),
                [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(wrench_mutex_);
                    latest_wrench_ = *msg;
                    latest_wrench_arrival_ = std::chrono::steady_clock::now();
                });
        }

        if (params_.align_mode == "servo") {
            twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                params_.servo_twist_topic, 10);
        }

        if (params_.insert_backend == "impedance") {
            equilibrium_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                params_.equilibrium_topic, rclcpp::QoS(1));
            switch_client_ =
                node_->create_client<controller_manager_msgs::srv::SwitchController>(
                    params_.controller_switch_service);
        }

        // Machine-readable state for dashboards/logging. Phase is
        // transient_local so late-joining GUIs get the current value.
        // /mating/phase always carries the real state-machine phase;
        // the operational pause flag is a separate topic.
        phase_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/mating/phase", rclcpp::QoS(1).transient_local());
        paused_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/mating/paused", rclcpp::QoS(1).transient_local());
        err_pos_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/mating/error_mm", 10);
        err_rot_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/mating/error_deg", 10);
        diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);
        publish_phase();

        // Operator recovery: unlatch FAULT/MATED and restart the sequence
        // (e.g. after clearing a jam or removing the mated connector).
        // Refused while an insertion stroke is interrupted: the tool may be
        // partially engaged, and the restarted sequence would command
        // lateral alignment motion while inside the connector.
        reset_srv_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/reset",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                if (insert_interrupted_) {
                    resp->success = false;
                    resp->message =
                        "Refused: FAULT latched mid-INSERT - the tool may be "
                        "partially engaged. Call ~/retract first to pull back "
                        "along the tool axis.";
                    RCLCPP_ERROR(LOGGER,
                                 "Reset refused: interrupted insertion stroke. "
                                 "Retract first (~/retract).");
                    return;
                }
                reset_requested_ = true;
                resp->success = true;
                resp->message = "Reset requested: sequence restarts at WAIT_FOR_VISION.";
                RCLCPP_WARN(LOGGER, "Reset requested via service.");
            });

        // Pull straight back along the tool axis to the stroke start (or by
        // insertion_depth_m if unknown), then restart at WAIT_FOR_VISION.
        // Only available in FAULT: it is the recovery path for an
        // interrupted insertion, not a general jog command.
        retract_srv_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/retract",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                if (phase_ != Phase::FAULT) {
                    resp->success = false;
                    resp->message = "Refused: retract is only available in FAULT.";
                    RCLCPP_WARN(LOGGER, "Retract refused: phase is %s, not FAULT.",
                                phase_name(phase_));
                    return;
                }
                retract_requested_ = true;
                resp->success = true;
                resp->message = "Retract requested: pulling back along tool -Z, "
                                "then restarting at WAIT_FOR_VISION.";
                RCLCPP_WARN(LOGGER, "Retract requested via service.");
            });

        // Operational stop (NOT a safety e-stop - that stays hardware):
        // halts the current trajectory immediately and latches FAULT.
        stop_srv_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/stop",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                stop_requested_ = true;
                move_group_.stop();  // cancel in-flight trajectory now
                resp->success = true;
                resp->message = "STOP: trajectory halted, FAULT latched (reset to recover).";
                RCLCPP_ERROR(LOGGER, "Software STOP via service - halting and latching FAULT.");
            });

        // Pause: halt and hold position; resume continues the sequence.
        pause_srv_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/pause",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                paused_ = true;
                move_group_.stop();
                publish_phase();
                resp->success = true;
                resp->message = "Paused: holding position until resume.";
                RCLCPP_WARN(LOGGER, "Paused via service.");
            });
        resume_srv_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/resume",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                paused_ = false;
                // (arming count was already zeroed by each paused cycle)
                publish_phase();
                resp->success = true;
                resp->message = "Resumed.";
                RCLCPP_WARN(LOGGER, "Resumed via service.");
            });

        planning_frame_ = move_group_.getPlanningFrame();
        eef_link_ = move_group_.getEndEffectorLink();
        RCLCPP_INFO(LOGGER, "Planning frame: %s, end effector: %s",
                    planning_frame_.c_str(), eef_link_.c_str());

        move_group_.setPlanningPipelineId(params_.planning_pipeline);
        move_group_.setPlannerId(params_.planner_id);
        move_group_.setMaxAccelerationScalingFactor(params_.accel_scaling);
        move_group_.setGoalPositionTolerance(0.001);
        move_group_.setGoalOrientationTolerance(0.01);

        if (params_.enable_insertion &&
            params_.connector_offset_x == 0.0 && params_.connector_offset_y == 0.0 &&
            params_.connector_offset_z == 0.0 && params_.tool_yaw_offset_deg == 0.0) {
            RCLCPP_WARN(LOGGER,
                        "All connector offsets are zero with insertion enabled - the "
                        "tool will aim at the MARKER CENTRE. Teach connector_offset_x/y/z "
                        "(setup guide 2.4), or set enable_insertion: false for teaching.");
        }

        RCLCPP_INFO(LOGGER, "Insert backend: %s (planner %s)%s. Alignment mode: %s.",
                    params_.insert_backend.c_str(),
                    params_.insert_planner.c_str(),
                    params_.wrench_topic.empty()
                        ? ", force guard OFF (no wrench_topic)"
                        : (", force-guarded via " + params_.wrench_topic).c_str(),
                    params_.align_mode.c_str());
        if (params_.insert_backend == "impedance") {
            std::string base_frame;
            node_->get_parameter_or("BASE_FRAME_ID", base_frame, std::string());
            if (!base_frame.empty() && base_frame != planning_frame_) {
                RCLCPP_WARN(LOGGER,
                            "insert_backend: impedance publishes equilibria in the "
                            "planning frame '%s', but BASE_FRAME_ID is '%s'. The "
                            "impedance controller works in the robot base frame - "
                            "these must be the SAME frame.",
                            planning_frame_.c_str(), base_frame.c_str());
            }
        }
        if (params_.align_mode == "servo") {
            RCLCPP_WARN(LOGGER,
                        "Servo alignment publishes twists on %s - a moveit_servo "
                        "node must be running, and control_period_s (%.2f s) must "
                        "be shorter than its command timeout.",
                        params_.servo_twist_topic.c_str(), params_.control_period_s);
        }

        RCLCPP_INFO(LOGGER,
                    "Timing budget: control %.2f s, vision timeout %.2f s, arming "
                    "%d cycles (%.2f s); insertion arming additionally requires raw "
                    "detections on %s.",
                    params_.control_period_s, params_.vision_timeout_s,
                    params_.align_hold_cycles,
                    params_.control_period_s * params_.align_hold_cycles,
                    params_.raw_pose_topic.c_str());
        if (params_.vision_timeout_s >=
            params_.control_period_s * params_.align_hold_cycles) {
            RCLCPP_WARN(LOGGER,
                        "vision_timeout_s (%.2f) spans the whole arming window "
                        "(%.2f s): a single stale pose could persist across every "
                        "hold cycle. Lower vision_timeout_s or raise align_hold_cycles.",
                        params_.vision_timeout_s,
                        params_.control_period_s * params_.align_hold_cycles);
        }
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
        get("raw_pose_topic", p.raw_pose_topic);
        get("control_period_s", p.control_period_s);
        get("vision_timeout_s", p.vision_timeout_s);
        get("tf_lookup_timeout_s", p.tf_lookup_timeout_s);
        get("planning_pipeline", p.planning_pipeline);
        get("planner_id", p.planner_id);
        get("accel_scaling", p.accel_scaling);
        get("enable_insertion", p.enable_insertion);
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
        get("wrench_topic", p.wrench_topic);
        get("contact_force_n", p.contact_force_n);
        get("max_lateral_force_n", p.max_lateral_force_n);
        get("min_contact_depth_m", p.min_contact_depth_m);
        get("wrench_timeout_s", p.wrench_timeout_s);
        get("insert_planner", p.insert_planner);
        get("cartesian_eef_step_m", p.cartesian_eef_step_m);
        get("align_mode", p.align_mode);
        get("servo_twist_topic", p.servo_twist_topic);
        get("servo_pos_gain", p.servo_pos_gain);
        get("servo_rot_gain", p.servo_rot_gain);
        get("insert_backend", p.insert_backend);
        get("impedance_controller", p.impedance_controller);
        get("trajectory_controller", p.trajectory_controller);
        get("controller_switch_service", p.controller_switch_service);
        get("equilibrium_topic", p.equilibrium_topic);
        get("impedance_stroke_mps", p.impedance_stroke_mps);
        get("impedance_overdrive_m", p.impedance_overdrive_m);
        get("impedance_settle_s", p.impedance_settle_s);
        if (p.insert_planner != "pipeline" && p.insert_planner != "cartesian") {
            throw std::runtime_error("insert_planner must be pipeline|cartesian");
        }
        if (p.align_mode != "step" && p.align_mode != "servo") {
            throw std::runtime_error("align_mode must be step|servo");
        }
        if (p.insert_backend != "moveit" && p.insert_backend != "impedance") {
            throw std::runtime_error("insert_backend must be moveit|impedance");
        }
        if (p.insert_backend == "impedance" && p.wrench_topic.empty()) {
            throw std::runtime_error(
                "insert_backend: impedance requires wrench_topic - without a "
                "force source the stroke outcome cannot be judged");
        }
    }

    // Mirror the machine's state into the atomics the service callbacks
    // read (they run on the executor thread), log/publish transitions.
    void sync_phase_from_machine()
    {
        insert_interrupted_ = machine_->insert_interrupted();
        const Phase next = machine_->phase();
        if (next != phase_) {
            RCLCPP_INFO(LOGGER, "Phase: %s -> %s", phase_name(phase_), phase_name(next));
            phase_ = next;
            publish_phase();
        }
    }

    void publish_phase()
    {
        std_msgs::msg::String msg;
        msg.data = phase_name(phase_);
        phase_pub_->publish(msg);
        std_msgs::msg::Bool paused_msg;
        paused_msg.data = paused_;
        paused_pub_->publish(paused_msg);
    }

    // Cell status for dashboards: phase, live error, health flags.
    void publish_status(bool vision_fresh, double pos_err_mm, double rot_err_deg)
    {
        std_msgs::msg::Float64 f;
        if (std::isfinite(pos_err_mm)) {
            f.data = pos_err_mm;
            err_pos_pub_->publish(f);
            f.data = rot_err_deg;
            err_rot_pub_->publish(f);
        }

        diagnostic_msgs::msg::DiagnosticArray arr;
        arr.header.stamp = node_->get_clock()->now();
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = "connector_mating";
        st.hardware_id = params_.pose_topic;
        if (phase_ == Phase::FAULT) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            st.message = "FAULT latched - operator reset required";
        } else if (!vision_fresh && phase_ != Phase::MATED) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "marker not visible / stale - holding";
        } else {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            st.message = phase_name(phase_);
        }
        auto kv = [&st](const std::string &k, const std::string &v) {
            diagnostic_msgs::msg::KeyValue e;
            e.key = k;
            e.value = v;
            st.values.push_back(e);
        };
        kv("phase", phase_name(phase_));
        kv("paused", paused_ ? "true" : "false");
        kv("vision_fresh", vision_fresh ? "true" : "false");
        kv("position_error_mm",
           std::isfinite(pos_err_mm) ? std::to_string(pos_err_mm) : "n/a");
        kv("rotation_error_deg",
           std::isfinite(rot_err_deg) ? std::to_string(rot_err_deg) : "n/a");
        kv("aligned_cycles", std::to_string(machine_->aligned_cycles()));
        kv("consecutive_plan_failures",
           std::to_string(machine_->consecutive_plan_failures()));
        kv("insertion_enabled", insertion_enabled() ? "true" : "false");
        arr.status.push_back(st);
        diag_pub_->publish(arr);
    }

    // Read fresh each cycle so a GUI/param toggle takes effect immediately.
    bool insertion_enabled()
    {
        bool enabled = params_.enable_insertion;
        node_->get_parameter_or("enable_insertion", enabled, enabled);
        return enabled;
    }

    // Age of a pose: from its header stamp (capture time) when usable, from
    // arrival time otherwise. Header stamps are authoritative - arrival time
    // hides DDS/executor latency and lies under bag replay - but zero or
    // future stamps (unsynchronized camera clock) fall back to arrival.
    double pose_age_s(const rclcpp::Time &stamp,
                      const std::chrono::steady_clock::time_point &arrival)
    {
        const double arrival_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - arrival).count();
        if (stamp.nanoseconds() == 0) {
            return arrival_age;
        }
        double stamp_age = NAN;
        try {
            stamp_age = (node_->get_clock()->now() - stamp).seconds();
        } catch (const std::exception &) {
            return arrival_age;  // mismatched clock types (e.g. sim time)
        }
        if (stamp_age < -0.05) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 10000,
                                 "Pose header stamp is %.3f s in the future - using "
                                 "arrival time (unsynchronized camera clock?).",
                                 -stamp_age);
            return arrival_age;
        }
        return stamp_age;
    }

    // Latest marker pose if it is fresh enough, else nullopt.
    std::optional<geometry_msgs::msg::PoseStamped> fresh_marker_pose()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!latest_pose_) {
            return std::nullopt;
        }
        const double age = pose_age_s(rclcpp::Time(latest_pose_->header.stamp),
                                      latest_pose_arrival_);
        if (age > params_.vision_timeout_s) {
            return std::nullopt;
        }
        return latest_pose_;
    }

    // True when a raw (unpredicted) detection arrived recently. The filtered
    // pose can be a Kalman prediction during dropouts; committing insertion
    // demands evidence the marker was actually SEEN.
    bool fresh_raw_detection()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!has_raw_) {
            return false;
        }
        return pose_age_s(latest_raw_stamp_, latest_raw_arrival_) <=
               params_.vision_timeout_s;
    }

    // Marker pose -> desired TCP standoff pose, in the planning frame.
    std::optional<tf2::Transform> compute_standoff_goal(
        const geometry_msgs::msg::PoseStamped &marker_msg)
    {
        geometry_msgs::msg::PoseStamped marker_in_planning;
        try {
            // Transform at the image timestamp: the marker pose was captured
            // at that instant, so it must compose with the robot's TF from
            // the same instant. "Latest" is wrong by however far the arm
            // moved between capture and this control cycle (eye-in-hand).
            geometry_msgs::msg::TransformStamped tf;
            const rclcpp::Time stamp(marker_msg.header.stamp);
            if (stamp.nanoseconds() == 0) {
                tf = tf_buffer_.lookupTransform(
                    planning_frame_, marker_msg.header.frame_id, tf2::TimePointZero);
            } else {
                try {
                    tf = tf_buffer_.lookupTransform(
                        planning_frame_, marker_msg.header.frame_id, stamp,
                        rclcpp::Duration::from_seconds(params_.tf_lookup_timeout_s));
                } catch (const tf2::TransformException &e) {
                    RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 5000,
                                         "TF at image stamp unavailable (%s) - "
                                         "falling back to latest transform.", e.what());
                    tf = tf_buffer_.lookupTransform(
                        planning_frame_, marker_msg.header.frame_id, tf2::TimePointZero);
                }
            }
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

        return mating_geometry::standoff_goal(
            t_planning_marker,
            params_.connector_offset_x, params_.connector_offset_y,
            params_.connector_offset_z, params_.standoff_height_m,
            params_.tool_yaw_offset_deg * M_PI / 180.0);
    }

    tf2::Transform current_tcp_pose()
    {
        tf2::Transform t;
        tf2::fromMsg(move_group_.getCurrentPose(eef_link_).pose, t);
        return t;
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

    // Latest external force re-expressed in the planning frame (rotation
    // only - force is a free vector), or nullopt when disabled/stale.
    std::optional<tf2::Vector3> fresh_force_in_planning()
    {
        geometry_msgs::msg::WrenchStamped w;
        {
            std::lock_guard<std::mutex> lock(wrench_mutex_);
            if (!latest_wrench_) {
                return std::nullopt;
            }
            const double age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - latest_wrench_arrival_).count();
            if (age > params_.wrench_timeout_s) {
                return std::nullopt;
            }
            w = *latest_wrench_;
        }
        tf2::Vector3 f(w.wrench.force.x, w.wrench.force.y, w.wrench.force.z);
        if (w.header.frame_id.empty() || w.header.frame_id == planning_frame_) {
            return f;
        }
        try {
            const auto tf = tf_buffer_.lookupTransform(
                planning_frame_, w.header.frame_id, tf2::TimePointZero);
            tf2::Quaternion q;
            tf2::fromMsg(tf.transform.rotation, q);
            return tf2::quatRotate(q, f);
        } catch (const tf2::TransformException &) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 5000,
                                 "Wrench frame '%s' not in TF - force guard inactive.",
                                 w.header.frame_id.c_str());
            return std::nullopt;
        }
    }

    // Plan the committed stroke (INSERT or retract). "cartesian" bypasses
    // the planning pipeline entirely: interpolated Cartesian waypoints,
    // time-parameterized at the stroke speed - a path-guaranteed straight
    // line on robots without Pilz LIN.
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
    plan_stroke(const tf2::Transform &target, double speed)
    {
        geometry_msgs::msg::Pose target_msg;
        tf2::toMsg(target, target_msg);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (params_.insert_planner == "cartesian") {
            moveit_msgs::msg::RobotTrajectory traj;
            const double fraction = move_group_.computeCartesianPath(
                {target_msg}, params_.cartesian_eef_step_m, 0.0, traj);
            if (fraction < 0.995) {
                RCLCPP_WARN(LOGGER, "Cartesian stroke only %.0f%% feasible.",
                            fraction * 100.0);
                return std::nullopt;
            }
            const auto state = move_group_.getCurrentState();
            robot_trajectory::RobotTrajectory rt(move_group_.getRobotModel(),
                                                 move_group_.getName());
            rt.setRobotTrajectoryMsg(*state, traj);
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            if (!totg.computeTimeStamps(rt, speed, params_.accel_scaling)) {
                RCLCPP_WARN(LOGGER, "Cartesian stroke time-parameterization failed.");
                return std::nullopt;
            }
            rt.getRobotTrajectoryMsg(plan.trajectory_);
            moveit::core::robotStateToRobotStateMsg(*state, plan.start_state_);
        } else {
            move_group_.setMaxVelocityScalingFactor(speed);
            move_group_.setPoseTarget(target_msg, eef_link_);
            if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(LOGGER, "Stroke planning failed.");
                return std::nullopt;
            }
        }
        if (!plan_is_sane(plan)) {
            return std::nullopt;
        }
        return plan;
    }

    enum class StrokeResult { DONE, CONTACT, LATERAL_ABORT, FAILED };

    // Execute the stroke. Without a wrench source this is the plain
    // blocking execute. With one, the trajectory runs asynchronously while
    // the wrench is watched at 100 Hz: axial reaction = contact (stop and
    // let the caller classify seated-vs-jam), lateral load = snag (stop,
    // caller latches FAULT). The stroke axis is taken from stroke_pose.
    StrokeResult execute_stroke(
        const moveit::planning_interface::MoveGroupInterface::Plan &plan,
        const tf2::Transform &stroke_pose, const tf2::Transform &target)
    {
        if (params_.wrench_topic.empty()) {
            return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS
                       ? StrokeResult::DONE
                       : StrokeResult::FAILED;
        }

        const auto &pts = plan.trajectory_.joint_trajectory.points;
        const double planned_s = pts.empty()
            ? 0.0
            : pts.back().time_from_start.sec +
                  pts.back().time_from_start.nanosec * 1e-9;
        if (move_group_.asyncExecute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            return StrokeResult::FAILED;
        }
        const tf2::Vector3 tool_z =
            tf2::quatRotate(stroke_pose.getRotation(), tf2::Vector3(0, 0, 1));
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(planned_s * 1.5 + 2.0));
        int tick = 0;
        while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
            if (stop_requested_ || paused_) {
                return StrokeResult::FAILED;  // operator interrupt: step() handles it
            }
            if (const auto f = fresh_force_in_planning()) {
                double axial = 0.0, lateral = 0.0;
                mating_geometry::wrench_axial_lateral(*f, tool_z, axial, lateral);
                if (lateral >= params_.max_lateral_force_n) {
                    move_group_.stop();
                    RCLCPP_ERROR(LOGGER, "Lateral force %.1f N (limit %.1f) - stroke stopped.",
                                 lateral, params_.max_lateral_force_n);
                    return StrokeResult::LATERAL_ABORT;
                }
                if (axial >= params_.contact_force_n) {
                    move_group_.stop();
                    RCLCPP_INFO(LOGGER, "Contact: axial reaction %.1f N (threshold %.1f).",
                                axial, params_.contact_force_n);
                    return StrokeResult::CONTACT;
                }
            }
            if (++tick % 5 == 0 &&
                (current_tcp_pose().getOrigin() - target.getOrigin()).length() < 0.0015) {
                return StrokeResult::DONE;  // reached the stroke end early
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        // Ran to the deadline: either completed quietly or stalled short of
        // the target; the caller re-checks depth/pose either way.
        return StrokeResult::DONE;
    }

    // Servo-mode alignment command. The velocity envelope reuses the step
    // clamps (max step per control period), so servo mode can never move
    // faster than step mode was allowed to.
    void publish_align_twist(const tf2::Vector3 &pos_err, const tf2::Quaternion &rot_err,
                             double max_step_m, double max_step_deg)
    {
        tf2::Vector3 lin, ang;
        mating_geometry::servo_twist(
            pos_err, rot_err, params_.servo_pos_gain, params_.servo_rot_gain,
            max_step_m / params_.control_period_s,
            max_step_deg * M_PI / 180.0 / params_.control_period_s, lin, ang);
        geometry_msgs::msg::TwistStamped t;
        t.header.stamp = node_->get_clock()->now();
        t.header.frame_id = planning_frame_;
        t.twist.linear.x = lin.x();
        t.twist.linear.y = lin.y();
        t.twist.linear.z = lin.z();
        t.twist.angular.x = ang.x();
        t.twist.angular.y = ang.y();
        t.twist.angular.z = ang.z();
        twist_pub_->publish(t);
    }

    // Explicit hold for servo mode (also acts as the deadman refresh).
    // No-op in step mode.
    void publish_zero_twist()
    {
        if (!twist_pub_) {
            return;
        }
        geometry_msgs::msg::TwistStamped t;
        t.header.stamp = node_->get_clock()->now();
        t.header.frame_id = planning_frame_;
        twist_pub_->publish(t);
    }

    // Feed an align-step result to the machine, preserving the "operator
    // halted an in-flight move" exemption (not a planner fault).
    void note_align_result(bool ok)
    {
        if (!ok && (paused_ || stop_requested_)) {
            return;
        }
        machine_->note_result(ok ? Outcome::SUCCESS : Outcome::FAILURE);
        if (!ok && machine_->phase() == Phase::FAULT) {
            RCLCPP_FATAL(LOGGER, "%d consecutive planning/execution failures - holding.",
                         machine_->consecutive_plan_failures());
        }
    }

    // Recovery from an interrupted insertion: straight-line pull-back along
    // the stroke. Returns to the recorded stroke start (exactly the standoff
    // pose) when known, else backs off by insertion_depth_m along tool -Z.
    // The machine turns SUCCESS into WAIT_FOR_VISION + latch clear.
    Outcome do_retract()
    {
        const tf2::Transform current = current_tcp_pose();
        const tf2::Transform target = insert_start_pose_
            ? *insert_start_pose_
            : mating_geometry::insertion_target(current, -params_.insertion_depth_m);
        const double pull_mm =
            (target.getOrigin() - current.getOrigin()).length() * 1000.0;
        RCLCPP_WARN(LOGGER, "Retracting %.1f mm along the tool axis (%s) at "
                            "insert speed.",
                    pull_mm, insert_start_pose_ ? "to recorded stroke start"
                                                : "by insertion_depth_m");
        // Same stroke planner as INSERT (straight path even without Pilz);
        // unguarded - pull-out friction would false-trigger the thresholds.
        const auto plan = plan_stroke(target, params_.insert_speed);
        if (plan &&
            move_group_.execute(*plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(LOGGER, "Retract complete - restarting at WAIT_FOR_VISION.");
            insert_start_pose_.reset();
            return Outcome::SUCCESS;
        }
        RCLCPP_ERROR(LOGGER, "Retract planning/execution failed - still "
                             "latched in FAULT. Clear the obstruction and "
                             "call ~/retract again, or recover manually.");
        return Outcome::FAILURE;
    }

    // One control cycle: gather world state into machine Inputs, let the
    // (unit-tested) machine decide the Action, execute it, and report the
    // Outcome back. All transition logic lives in mating_phase_machine.hpp.
    void step()
    {
        mating_phase_machine::Inputs in;
        in.stop_requested = stop_requested_.exchange(false);
        in.retract_requested = retract_requested_.exchange(false);
        in.reset_requested = reset_requested_.exchange(false);
        in.paused = paused_;
        in.insertion_enabled = insertion_enabled();

        if (in.stop_requested) {
            RCLCPP_ERROR(LOGGER, "Software STOP - latching FAULT (was %s).",
                         phase_name(phase_));
            if (phase_ == Phase::INSERT) {
                RCLCPP_ERROR(LOGGER, "STOP interrupted the insertion stroke - "
                                     "call ~/retract before reset.");
            }
        }
        if (in.reset_requested && machine_->reset_allowed()) {
            RCLCPP_WARN(LOGGER, "Resetting controller state (was %s%s).",
                        phase_name(phase_), paused_ ? ", paused" : "");
            paused_ = false;
            in.paused = false;
            insert_start_pose_.reset();
        }

        // Vision and goal - skipped when no motion could happen anyway.
        std::optional<geometry_msgs::msg::PoseStamped> marker;
        std::optional<tf2::Transform> goal;
        if (!in.stop_requested && !in.paused && !in.retract_requested) {
            marker = fresh_marker_pose();
            if (marker) {
                goal = compute_standoff_goal(*marker);
            }
        }
        in.vision_fresh = marker.has_value();
        in.raw_fresh = fresh_raw_detection();
        in.goal_valid = goal.has_value();

        tf2::Transform current;
        tf2::Vector3 pos_err;
        tf2::Quaternion rot_err;
        double pos_dist = NAN, rot_deg = NAN;
        if (goal) {
            current = current_tcp_pose();
            double rot_angle = 0.0;
            mating_geometry::pose_error(current, *goal, pos_err, rot_err, rot_angle);
            pos_dist = pos_err.length();
            rot_deg = rot_angle * 180.0 / M_PI;
            in.pos_err_m = pos_dist;
            in.rot_err_deg = rot_deg;
        }

        const auto action = machine_->step(in);
        sync_phase_from_machine();  // decide() may already have moved the phase

        // Situation logging, matching what the machine saw this cycle.
        if (in.paused) {
            RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 5000,
                                 "Paused - holding position (resume to continue).");
        } else if (phase_ == Phase::MATED || phase_ == Phase::FAULT) {
            RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 10000,
                                 "Phase %s - no motion will be commanded.",
                                 phase_name(phase_));
        } else if (!in.vision_fresh && phase_ != Phase::WAIT_FOR_VISION) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
                                 "Marker not visible / stale - holding position.");
        }
        if (goal) {
            RCLCPP_INFO(LOGGER, "[%s] err: %.1f mm, %.2f deg",
                        phase_name(phase_), pos_dist * 1000.0, rot_deg);
        }
        publish_status(in.vision_fresh, goal ? pos_dist * 1000.0 : NAN,
                       goal ? rot_deg : NAN);

        switch (action) {
        case Action::NONE:
            if (in.retract_requested) {
                RCLCPP_WARN(LOGGER, "Retract ignored: phase is %s, not FAULT.",
                            phase_name(phase_));
            }
            break;

        case Action::HOLD:
            publish_zero_twist();
            if (phase_ == Phase::ALIGN_FINE && goal && !in.raw_fresh &&
                pos_dist < params_.fine_pos_tol_m) {
                // In tolerance, but the filtered pose may be riding on
                // Kalman predictions - they steer, but never arm the stroke.
                RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 2000,
                                     "In tolerance but no fresh raw detection - "
                                     "holding arming count at %d/%d.",
                                     machine_->aligned_cycles(),
                                     params_.align_hold_cycles);
            } else if (phase_ == Phase::ALIGN_FINE && goal &&
                       !in.insertion_enabled &&
                       machine_->aligned_cycles() >= params_.align_hold_cycles) {
                RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 5000,
                                     "Aligned (%.1f mm, %.2f deg) - insertion "
                                     "disabled (enable_insertion=false), holding "
                                     "at standoff.", pos_dist * 1000.0, rot_deg);
            }
            break;

        case Action::ALIGN_COARSE_STEP:
            if (params_.align_mode == "servo") {
                publish_align_twist(pos_err, rot_err, params_.coarse_max_step_m,
                                    params_.coarse_max_step_deg);
                break;
            }
            note_align_result(move_lin(
                mating_geometry::clamped_target(
                    current, pos_err, rot_err, params_.coarse_max_step_m,
                    params_.coarse_max_step_deg * M_PI / 180.0),
                params_.coarse_speed));
            break;

        case Action::ALIGN_FINE_STEP:
            if (params_.align_mode == "servo") {
                publish_align_twist(pos_err, rot_err, params_.fine_max_step_m,
                                    params_.fine_max_step_deg);
                break;
            }
            note_align_result(move_lin(
                mating_geometry::clamped_target(
                    current, pos_err, rot_err, params_.fine_max_step_m,
                    params_.fine_max_step_deg * M_PI / 180.0),
                params_.fine_speed));
            break;

        case Action::COMMIT_INSERT: {
            const Outcome outcome = do_insert_stroke(current);
            machine_->note_result(outcome);
            if (outcome == Outcome::FAILURE &&
                machine_->phase() == Phase::ALIGN_FINE) {
                insert_start_pose_.reset();  // re-align from standoff
            }
            if (machine_->phase() == Phase::FAULT &&
                machine_->insert_interrupted()) {
                RCLCPP_ERROR(LOGGER, "FAULT during insertion stroke - "
                                     "call ~/retract before reset.");
            }
            break;
        }

        case Action::RETRACT:
            machine_->note_result(do_retract());
            break;
        }

        sync_phase_from_machine();
    }

    // ── Impedance insert backend ────────────────────────────────────────
    // Compliant, self-aligning stroke: hand the arm to the Cartesian-
    // impedance controller (soft lateral, firm axial), ramp its
    // equilibrium along the tool axis, and judge the outcome purely by
    // the external wrench. The trajectory controller gets the arm back
    // afterwards, whatever happened.

    bool switch_controllers(const std::string &activate, const std::string &deactivate)
    {
        using SwitchController = controller_manager_msgs::srv::SwitchController;
        if (!switch_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(LOGGER, "Controller switch service unavailable (%s).",
                         params_.controller_switch_service.c_str());
            return false;
        }
        auto req = std::make_shared<SwitchController::Request>();
        req->activate_controllers = {activate};
        req->deactivate_controllers = {deactivate};
        req->strictness = SwitchController::Request::STRICT;
        auto future = switch_client_->async_send_request(req);
        // Safe to block: this is the control thread, the executor spins
        // on its own thread.
        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
            !future.get()->ok) {
            RCLCPP_ERROR(LOGGER, "Controller switch failed (%s on, %s off).",
                         activate.c_str(), deactivate.c_str());
            return false;
        }
        RCLCPP_INFO(LOGGER, "Controllers switched: %s active, %s inactive.",
                    activate.c_str(), deactivate.c_str());
        return true;
    }

    void publish_equilibrium(const tf2::Transform &pose)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = node_->get_clock()->now();
        msg.header.frame_id = planning_frame_;
        tf2::toMsg(pose, msg.pose);
        equilibrium_pub_->publish(msg);
    }

    // Decisive force event during the compliant stroke? Same thresholds
    // and classification as the guarded MoveIt stroke.
    bool stroke_force_event(const tf2::Vector3 &axis, Outcome &out)
    {
        const auto f = fresh_force_in_planning();
        if (!f) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 1000,
                                 "Wrench stale during impedance stroke.");
            return false;
        }
        double axial = 0.0, lateral = 0.0;
        mating_geometry::wrench_axial_lateral(*f, axis, axial, lateral);
        if (lateral >= params_.max_lateral_force_n) {
            RCLCPP_ERROR(LOGGER, "Sideways load %.1f N during impedance stroke - "
                                 "FAULT; call ~/retract.", lateral);
            out = Outcome::LATERAL_ABORT;
            return true;
        }
        if (axial >= params_.contact_force_n) {
            const double travelled =
                (current_tcp_pose().getOrigin() -
                 insert_start_pose_->getOrigin()).dot(axis);
            if (travelled >= params_.min_contact_depth_m) {
                RCLCPP_INFO(LOGGER, "Seated by contact force (%.1f N) at %.1f mm "
                                    "depth - connector mated.", axial,
                            travelled * 1000.0);
                out = Outcome::CONTACT_SEATED;
            } else {
                RCLCPP_ERROR(LOGGER, "Contact at %.1f mm - before "
                                     "min_contact_depth_m (%.1f mm): obstruction. "
                                     "FAULT; call ~/retract.", travelled * 1000.0,
                             params_.min_contact_depth_m * 1000.0);
                out = Outcome::CONTACT_JAM;
            }
            return true;
        }
        return false;
    }

    // Stream the equilibrium along the tool axis at impedance_stroke_mps,
    // overdriving past full depth so the axial spring builds the seating
    // preload. On any non-seated exit the equilibrium is re-published at
    // the MEASURED pose, relaxing the spring where the arm actually is.
    Outcome run_impedance_ramp(const tf2::Transform &start,
                               const tf2::Vector3 &axis, double remaining)
    {
        const double dt = 0.02;  // 50 Hz setpoint stream
        const double total = remaining + params_.impedance_overdrive_m;
        double advanced = 0.0;
        double settle_left = params_.impedance_settle_s;
        Outcome out = Outcome::FAILURE;

        while (rclcpp::ok()) {
            if (stop_requested_ || paused_) {
                publish_equilibrium(current_tcp_pose());  // relax the preload here
                return Outcome::INTERRUPTED;
            }
            if (stroke_force_event(axis, out)) {
                if (out == Outcome::CONTACT_SEATED) {
                    insert_start_pose_.reset();
                } else {
                    publish_equilibrium(current_tcp_pose());
                }
                return out;
            }
            if (advanced < total) {
                advanced = std::min(total,
                                    advanced + params_.impedance_stroke_mps * dt);
            } else if ((settle_left -= dt) <= 0.0) {
                // Full ramp + settle without a decisive force: judge by depth.
                const double travelled =
                    (current_tcp_pose().getOrigin() -
                     insert_start_pose_->getOrigin()).dot(axis);
                if (travelled >= remaining - 0.002) {
                    RCLCPP_WARN(LOGGER, "Impedance stroke reached depth without "
                                        "contact_force_n - verify the connector "
                                        "actually seated.");
                    insert_start_pose_.reset();
                    return Outcome::SUCCESS;
                }
                RCLCPP_WARN(LOGGER, "Impedance stroke stalled at %.1f of %.1f mm "
                                    "without a decisive force - failed attempt.",
                            travelled * 1000.0, remaining * 1000.0);
                publish_equilibrium(current_tcp_pose());
                return Outcome::FAILURE;
            }
            tf2::Transform eq = start;  // orientation locked to the stroke start
            eq.setOrigin(start.getOrigin() + axis * advanced);
            publish_equilibrium(eq);
            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }
        return Outcome::INTERRUPTED;
    }

    Outcome do_impedance_stroke(const tf2::Transform &current,
                                const tf2::Vector3 &stroke_axis, double remaining)
    {
        RCLCPP_INFO(LOGGER, "Impedance stroke: %.1f mm (+%.1f mm preload lead) "
                            "at %.1f mm/s.",
                    remaining * 1000.0, params_.impedance_overdrive_m * 1000.0,
                    params_.impedance_stroke_mps * 1000.0);
        if (!switch_controllers(params_.impedance_controller,
                                params_.trajectory_controller)) {
            return Outcome::FAILURE;
        }
        // Let the controller seed its equilibrium at the current pose
        // (zero spring force) before the first setpoint arrives.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const Outcome out = run_impedance_ramp(current, stroke_axis, remaining);
        if (!switch_controllers(params_.trajectory_controller,
                                params_.impedance_controller)) {
            RCLCPP_FATAL(LOGGER, "Could not hand the arm back to '%s' - the "
                                 "impedance controller keeps holding (compliant, "
                                 "safe), but the sequence cannot continue. Switch "
                                 "controllers manually.",
                         params_.trajectory_controller.c_str());
            return (out == Outcome::CONTACT_SEATED || out == Outcome::SUCCESS)
                       ? out
                       : Outcome::FAILURE;
        }
        return out;
    }

    // Committed stroke: alignment was verified with fresh raw vision for
    // align_hold_cycles. Descend along the tool Z axis with the orientation
    // locked; the gripper occluding the marker from here on is expected and
    // does not abort the stroke.
    //
    // The stroke start is recorded so that (a) a stroke resumed after a
    // pause covers only the REMAINING depth instead of a full
    // insertion_depth_m from mid-stroke, and (b) ~/retract can return
    // exactly to the standoff pose. It is deliberately KEPT on jam/abort
    // outcomes - retract needs to know where the stroke began.
    Outcome do_insert_stroke(const tf2::Transform &current)
    {
        if (!insert_start_pose_) {
            insert_start_pose_ = current;
        }
        const tf2::Vector3 stroke_axis = tf2::quatRotate(
            insert_start_pose_->getRotation(), tf2::Vector3(0, 0, 1));
        const double progress =
            (current.getOrigin() - insert_start_pose_->getOrigin()).dot(stroke_axis);
        const double remaining = params_.insertion_depth_m - progress;
        if (remaining <= 0.001) {
            RCLCPP_INFO(LOGGER, "Insertion depth already reached - connector mated.");
            insert_start_pose_.reset();
            return Outcome::SUCCESS;
        }
        RCLCPP_INFO(LOGGER, "Committing insertion: %.1f mm along tool Z at %.0f%% speed.",
                    remaining * 1000.0, params_.insert_speed * 100.0);
        publish_zero_twist();  // servo mode: stop tracking, the stroke owns motion

        if (params_.insert_backend == "impedance") {
            return do_impedance_stroke(current, stroke_axis, remaining);
        }

        const auto target = mating_geometry::insertion_target(current, remaining);

        const auto plan = plan_stroke(target, params_.insert_speed);
        if (!plan) {
            return (paused_ || stop_requested_) ? Outcome::INTERRUPTED
                                                : Outcome::FAILURE;
        }
        switch (execute_stroke(*plan, *insert_start_pose_, target)) {
        case StrokeResult::DONE:
            // DONE from the guarded executor can also mean "ran out of time"
            // - only the TCP actually being at the stroke end counts.
            if ((current_tcp_pose().getOrigin() - target.getOrigin()).length() > 0.003) {
                RCLCPP_WARN(LOGGER, "Stroke ended short of the target - treating "
                                    "as a failed attempt.");
                return (paused_ || stop_requested_) ? Outcome::INTERRUPTED
                                                    : Outcome::FAILURE;
            }
            if (!params_.wrench_topic.empty()) {
                RCLCPP_WARN(LOGGER, "Stroke ran to full depth without reaching "
                                    "contact_force_n - verify the connector "
                                    "actually seated.");
            }
            RCLCPP_INFO(LOGGER, "Insertion stroke complete - connector mated.");
            insert_start_pose_.reset();
            return Outcome::SUCCESS;

        case StrokeResult::CONTACT: {
            const double travelled =
                (current_tcp_pose().getOrigin() -
                 insert_start_pose_->getOrigin()).dot(stroke_axis);
            if (travelled >= params_.min_contact_depth_m) {
                RCLCPP_INFO(LOGGER, "Seated by contact force at %.1f mm depth - "
                                    "connector mated.", travelled * 1000.0);
                insert_start_pose_.reset();
                return Outcome::CONTACT_SEATED;
            }
            RCLCPP_ERROR(LOGGER, "Contact at %.1f mm - before min_contact_depth_m "
                                 "(%.1f mm): obstruction/misalignment. FAULT; "
                                 "call ~/retract.", travelled * 1000.0,
                         params_.min_contact_depth_m * 1000.0);
            return Outcome::CONTACT_JAM;
        }

        case StrokeResult::LATERAL_ABORT:
            RCLCPP_ERROR(LOGGER, "Sideways load during insertion - FAULT; "
                                 "call ~/retract.");
            return Outcome::LATERAL_ABORT;

        case StrokeResult::FAILED:
        default:
            if (paused_ || stop_requested_) {
                // Operator halted mid-stroke: stay in INSERT, keep the
                // recorded start so a resume covers only what's left.
                return Outcome::INTERRUPTED;
            }
            RCLCPP_WARN(LOGGER, "Insertion attempt failed - re-verifying "
                                "alignment before retrying.");
            return Outcome::FAILURE;
        }
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface &move_group_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr equilibrium_pub_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retract_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr paused_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_rot_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    std::atomic<bool> reset_requested_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> retract_requested_{false};
    // Set when FAULT latches with an insertion stroke under way; read by the
    // reset service (executor thread) to refuse until ~/retract has run.
    std::atomic<bool> insert_interrupted_{false};

    Params params_;
    std::string planning_frame_;
    std::string eef_link_;

    std::mutex pose_mutex_;
    std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
    std::chrono::steady_clock::time_point latest_pose_arrival_;
    rclcpp::Time latest_raw_stamp_;
    std::chrono::steady_clock::time_point latest_raw_arrival_;
    bool has_raw_{false};

    std::mutex wrench_mutex_;
    std::optional<geometry_msgs::msg::WrenchStamped> latest_wrench_;
    std::chrono::steady_clock::time_point latest_wrench_arrival_;

    // The sequencing brain (pure, unit-tested). Owned and touched only by
    // the control thread; service callbacks read the atomic mirrors below.
    std::optional<PhaseMachine> machine_;
    std::atomic<Phase> phase_{Phase::WAIT_FOR_VISION};
    // TCP pose when the current insertion stroke was committed (control
    // thread only). Survives a pause so the resumed stroke covers only the
    // remaining depth, and lets ~/retract return exactly to standoff.
    std::optional<tf2::Transform> insert_start_pose_;
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
