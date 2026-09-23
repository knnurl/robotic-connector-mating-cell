// Continuous marker tracking on the Cartesian-impedance backend
// (TRACKING_SPEC.md section 5).
//
// Streams an equilibrium pose at 50 Hz so the arm FOLLOWS the marker instead
// of stepping to it. The goal is camera-centred: it holds the camera where
// cell_panel's ALIGN leaves it (the marker tracking_standoff_m straight
// ahead, at tracking_inplane_deg in the image). A bounded integrator pushes
// the equilibrium past that goal until the MEASURED error closes, which is
// how the friction residual F/k gets taken out - the controller cannot know
// it is stuck, only vision can. All of that law is pure and lives in
// tracking_law.hpp; this file is only its plumbing. ~/status (latched
// DiagnosticStatus) says what the node is doing and why.
//
// Three properties are load-bearing and easy to break:
//
//   * FRAMES. The goal is a TCP (EEF_FRAME_ID, fr3_hand_tcp) pose while the
//     controller seeds and measures at franka::Frame::kEndEffector. Nothing
//     in this cell asserts the two coincide - it carries a D405 wrist mount.
//     The offset is MEASURED at ~/start_tracking from TF and o_t_ee sampled
//     together, the hand-eye transform (TCP -> tracking_camera_frame) is
//     read from TF alongside it, and start is refused if either cannot be
//     established.
//   * THREADING. ~/start_tracking blocks on the controller manager and the
//     impedance controller's parameter service, so this node MUST be spun by
//     a MultiThreadedExecutor and the services, the service clients and the
//     timer MUST sit in different callback groups. Under rclcpp::spin the
//     reply can never be delivered: the one executor thread is inside the
//     callback waiting for it. See mating_node.cpp L1134-1136 for the same
//     reasoning.
//   * THE OPERATOR. The node comes up idle and publishes NOTHING until
//     ~/start_tracking is called from the panel, and vision loss holds where
//     the arm is rather than coasting or extrapolating.
//
// The subscriptions only store the latest message and its arrival time; all
// work happens in the wall timer, never in a callback.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mating_controller/mating_geometry.hpp"
#include "mating_controller/tracking_law.hpp"

using namespace std::chrono_literals;
using controller_manager_msgs::srv::ListControllers;
using diagnostic_msgs::msg::DiagnosticStatus;
using std_srvs::srv::Trigger;

struct Params
{
    std::string pose_topic{"/aruco/pose"};
    std::string raw_pose_topic{"/aruco/pose_raw"};
    std::string base_frame{"fr3_link0"};
    std::string eef_frame{"fr3_hand_tcp"};
    double vision_timeout_s{0.6};
    double raw_timeout_s{0.25};

    // Where ALIGN leaves the camera. cell_panel writes these before every
    // START, so ~/start_tracking re-reads them; these are the yaml values.
    std::string camera_frame{"camera_color_optical_frame"};
    double standoff_m{0.10};
    double inplane_deg{90.0};
    bool inplane_hold{false};

    std::string impedance_controller{"cartesian_impedance_stroke_controller"};
    std::string equilibrium_topic{
        "/cartesian_impedance_stroke_controller/equilibrium_pose"};
    std::string state_topic{"/franka_robot_state_broadcaster/robot_state"};

    double period_s{0.02};              // 50 Hz
    double ki{0.5};
    double lead_max_m{0.010};
    double lead_max_rad{0.017};
    double deadband_m{0.005};
    double deadband_rad{0.007};
    double max_lead_m{0.060};
    double max_lead_rad{0.26};
    std::string over_lead_policy{"hold"};
    double floor_below_start_m{0.030};
    double state_timeout_s{0.1};
    double profile_timeout_s{2.0};
    double settle_s{1.0};
    double max_force_n{30.0};
    double max_torque_nm{10.0};
    std::string log_dir;
    std::string gain_profile{"track"};
};

// What ~/status reports, besides the policy and the two vision ages, which
// are read when it is published.
struct Status
{
    std::string state{"idle"};   // idle | starting | tracking | holding | stopping
    std::string reason;
    uint8_t level{DiagnosticStatus::OK};
    double pos_err_mm{NAN};
    double rot_err_deg{NAN};
    double lead_mm{0.0};
    double lead_deg{0.0};
    double standoff_m{NAN};
    double inplane_deg{NAN};
};

class TrackingNode : public rclcpp::Node
{
public:
    explicit TrackingNode(const rclcpp::NodeOptions &options)
        : Node("tracking_node", options),
          tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        load_params();
        profile_ = profile_params();

        cfg_.ki = params_.ki;
        cfg_.lead_max_m = params_.lead_max_m;
        cfg_.lead_max_rad = params_.lead_max_rad;
        cfg_.deadband_m = params_.deadband_m;
        cfg_.deadband_rad = params_.deadband_rad;
        cfg_.max_lead_m = params_.max_lead_m;
        cfg_.max_lead_rad = params_.max_lead_rad;
        cfg_.over_lead_policy = params_.over_lead_policy;
        // A lead that could exceed 15 N must be a startup refusal, not a
        // surprise with the arm moving.
        const std::string why = tracking_law::validate_config(
            cfg_, max_gain(0), max_gain(1), params_.max_force_n,
            params_.max_torque_nm);
        if (!why.empty()) {
            throw std::runtime_error(why);
        }
        policy_ = *tracking_law::parse_over_lead(params_.over_lead_policy);
        status_.standoff_m = params_.standoff_m;
        status_.inplane_deg = params_.inplane_deg;

        srv_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // STOP gets its OWN group. Sharing srv_group_ made "STOP is always
        // live" false: a MutuallyExclusive group runs one callback at a time,
        // so a stop queued behind an in-flight start, and on_start blocks for
        // up to ~14 s across four service round-trips. The status timer sits
        // here too: all it may block on is finishing an over-lead stop.
        stop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        param_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        tick_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = tick_group_;
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            params_.pose_topic, rclcpp::QoS(1),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                latest_pose_ = *msg;
                latest_pose_arrival_ = std::chrono::steady_clock::now();
            },
            sub_options);
        // Arrival time only: the gate asks whether the detector saw the
        // marker just now, not what the filter predicts.
        raw_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            params_.raw_pose_topic, rclcpp::QoS(1),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                latest_raw_arrival_ = std::chrono::steady_clock::now();
            },
            sub_options);
        // o_t_ee: the same field cell_panel anchors on, and the pose the
        // controller holds its equilibrium against.
        state_sub_ = create_subscription<franka_msgs::msg::FrankaRobotState>(
            params_.state_topic, rclcpp::QoS(1),
            [this](franka_msgs::msg::FrankaRobotState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                tf2::fromMsg(msg->o_t_ee.pose, latest_ee_);
                has_state_ = true;
                latest_state_arrival_ = std::chrono::steady_clock::now();
            },
            sub_options);

        // The node must NOT depend on the operator panel remembering to stop
        // it. Every ros2_control controller is a lifecycle node, so it
        // announces its own transitions: if the impedance controller leaves
        // ACTIVE - RELEASE, a crash, a switch by mating_node - tracking halts
        // itself, a START in progress included. Otherwise an orphaned tracker
        // keeps streaming at 50 Hz and the next activation resumes autonomous
        // motion with nobody pressing anything.
        // Its own group: tick() holds control_mutex_ for its whole body, and
        // halt_tracking() takes the same lock, so this callback must never be
        // serialised behind the tick.
        rclcpp::SubscriptionOptions transition_options;
        transition_options.callback_group = stop_group_;
        transition_sub_ = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
            "/" + params_.impedance_controller + "/transition_event",
            rclcpp::QoS(10),
            [this](lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
                if ((!tracking_ && !starting_) ||
                    msg->goal_state.id ==
                        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    return;
                }
                RCLCPP_WARN(get_logger(),
                            "%s left ACTIVE (now '%s') - stopping tracking. The "
                            "arm is no longer on the impedance controller.",
                            params_.impedance_controller.c_str(),
                            msg->goal_state.label.c_str());
                halt_tracking(params_.impedance_controller + " left ACTIVE (now '" +
                                  msg->goal_state.label + "')",
                              true);
            },
            transition_options);

        eq_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            params_.equilibrium_topic, rclcpp::QoS(1));
        // Latched, so a panel started mid-run reads the state at once.
        status_pub_ = create_publisher<DiagnosticStatus>(
            "~/status", rclcpp::QoS(1).reliable().transient_local());
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            this, params_.impedance_controller, rmw_qos_profile_parameters,
            param_group_);
        list_client_ = create_client<ListControllers>(
            "/controller_manager/list_controllers", rmw_qos_profile_services_default,
            param_group_);

        // tracking_over_lead_policy is the one live parameter: cell_panel's
        // dropdown writes it mid-run. Anything but the three policies is
        // refused here, so it never reaches the tick. The rest are read at
        // construction or at START only.
        on_set_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &changed) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                std::optional<tracking_law::OverLead> policy;
                for (const auto &p : changed) {
                    if (p.get_name() != "tracking_over_lead_policy") {
                        continue;
                    }
                    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                        policy = tracking_law::parse_over_lead(p.as_string());
                    }
                    if (!policy) {
                        result.successful = false;
                        result.reason = "tracking_over_lead_policy must be hold, stop or clamp";
                        return result;
                    }
                }
                if (policy) {
                    policy_ = *policy;
                    RCLCPP_INFO(get_logger(), "Over-lead policy -> %s",
                                tracking_law::over_lead_name(*policy));
                }
                return result;
            });

        start_srv_ = create_service<Trigger>(
            "~/start_tracking",
            [this](const std::shared_ptr<Trigger::Request>,
                   std::shared_ptr<Trigger::Response> res) { on_start(res); },
            rmw_qos_profile_services_default, srv_group_);
        stop_srv_ = create_service<Trigger>(
            "~/stop_tracking",
            [this](const std::shared_ptr<Trigger::Request>,
                   std::shared_ptr<Trigger::Response> res) {
                const auto [ok, msg] = halt_tracking("stopped from ~/stop_tracking", false);
                res->success = ok;
                res->message = msg;
            },
            rmw_qos_profile_services_default, stop_group_);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(params_.period_s)),
            [this]() { tick(); }, tick_group_);
        // ~5 Hz, so a late subscriber and a dead node both show. It also
        // finishes an over-lead stop: tick() holds control_mutex_, which
        // halt_tracking() takes, so it cannot restore the gains itself.
        status_timer_ = create_wall_timer(
            200ms,
            [this]() {
                if (stop_pending_) {
                    halt_tracking("", true);
                }
                publish_status();
            },
            stop_group_);

        // Ctrl-C must not leave the arm stiff and leaning on a lead.
        // on_shutdown() runs AFTER rcl_shutdown, by which point the parameter
        // future can never complete and the restore silently fails - leaving
        // the arm on 10x the mating stiffness. A PRE-shutdown callback runs
        // while the executor still spins, so the reply can still arrive.
        get_node_base_interface()->get_context()->add_pre_shutdown_callback(
            [this]() {
                const auto [ok, msg] = halt_tracking("the node is shutting down", false);
                (void)ok;
                if (msg.find("NOT restored") != std::string::npos) {
                    RCLCPP_FATAL(get_logger(),
                                 "Shutting down with the '%s' profile STILL IN "
                                 "FORCE on %s (%s). Put the gains back before "
                                 "anyone touches the arm.",
                                 params_.gain_profile.c_str(),
                                 params_.impedance_controller.c_str(),
                                 msg.c_str());
                }
            });
        RCLCPP_INFO(get_logger(),
                    "Tracking node idle: nothing is published until "
                    "~/start_tracking. Profile '%s', %.0f Hz, over-lead policy %s.",
                    params_.gain_profile.c_str(), 1.0 / params_.period_s,
                    params_.over_lead_policy.c_str());
        publish_status();
    }

    ~TrackingNode() override
    {
        halt_tracking("the node is shutting down", false);
    }

private:
    // ── configuration ───────────────────────────────────────────────────

    void load_params()
    {
        auto &p = params_;
        // Every value comes from the yaml. A missing key is a refusal that
        // names it, never a silent fall back to the C++ default - which is
        // also why main() declares parameters from overrides.
        auto get = [this](const std::string &name, auto &field) {
            if (!get_parameter(name, field)) {
                throw std::runtime_error(
                    name + " is missing - it must be set in the params yaml");
            }
        };
        get("pose_topic", p.pose_topic);
        get("tracking_raw_pose_topic", p.raw_pose_topic);
        get("BASE_FRAME_ID", p.base_frame);
        get("EEF_FRAME_ID", p.eef_frame);
        get("vision_timeout_s", p.vision_timeout_s);
        get("tracking_raw_timeout_s", p.raw_timeout_s);
        get("tracking_camera_frame", p.camera_frame);
        get("tracking_standoff_m", p.standoff_m);
        get("tracking_inplane_deg", p.inplane_deg);
        get("tracking_inplane_hold", p.inplane_hold);
        get("impedance_controller", p.impedance_controller);
        get("equilibrium_topic", p.equilibrium_topic);
        get("tracking_state_topic", p.state_topic);
        get("tracking_period_s", p.period_s);
        get("tracking_ki", p.ki);
        get("tracking_lead_max_m", p.lead_max_m);
        get("tracking_lead_max_rad", p.lead_max_rad);
        get("tracking_deadband_m", p.deadband_m);
        get("tracking_deadband_rad", p.deadband_rad);
        get("tracking_max_lead_m", p.max_lead_m);
        get("tracking_max_lead_rad", p.max_lead_rad);
        get("tracking_over_lead_policy", p.over_lead_policy);
        get("tracking_floor_below_start_m", p.floor_below_start_m);
        get("tracking_state_timeout_s", p.state_timeout_s);
        get("tracking_profile_timeout_s", p.profile_timeout_s);
        get("tracking_settle_s", p.settle_s);
        get("tracking_max_force_n", p.max_force_n);
        get("tracking_max_torque_nm", p.max_torque_nm);
        get("tracking_log_dir", p.log_dir);
        get("tracking_gain_profile", p.gain_profile);
        if (!std::isfinite(p.period_s) || p.period_s <= 0.0) {
            throw std::runtime_error("tracking_period_s must be finite and positive");
        }
    }

    // The <profile>_* keys, renamed to the controller's own parameter names
    // so they can be sent as one atomic set. Validated by existence rather
    // than against a {track, mate} whitelist, so a new profile is a yaml
    // edit and not a rebuild.
    std::vector<rclcpp::Parameter> profile_params()
    {
        std::vector<rclcpp::Parameter> out;
        for (const char *suffix : {"k_pos_tool", "k_rot_tool", "damping_ratio",
                                   "setpoint_slew_mps", "setpoint_slew_rps"}) {
            const std::string key = params_.gain_profile + "_" + suffix;
            rclcpp::Parameter value;
            if (!get_parameter(key, value)) {
                throw std::runtime_error(
                    key + " is missing - tracking_gain_profile names it");
            }
            out.emplace_back(suffix, value.get_parameter_value());
        }
        return out;
    }

    // Largest entry of profile_[index] (0 = k_pos_tool, 1 = k_rot_tool): the
    // stiffness the deadband and the lead bound must be judged against.
    double max_gain(size_t index) const
    {
        double worst = 0.0;
        for (double k : profile_.at(index).as_double_array()) {
            worst = std::max(worst, k);
        }
        return worst;
    }

    // ── the impedance controller's parameters ───────────────────────────

    std::pair<bool, std::string> apply_params(const std::vector<rclcpp::Parameter> &values)
    {
        if (!param_client_->wait_for_service(1s)) {
            return {false, params_.impedance_controller +
                           " is not answering - is the controller spawned?"};
        }
        auto future = param_client_->set_parameters_atomically(values);
        if (future.wait_for(std::chrono::duration<double>(params_.profile_timeout_s)) !=
            std::future_status::ready) {
            return {false, "the impedance controller did not answer"};
        }
        const auto result = future.get();
        return {result.successful, result.successful ? std::string("applied")
                                                     : result.reason};
    }

    // The controller's current values, or nullopt if it did not answer or
    // does not have them. A NOT_SET reply is a failure: restoring it later
    // would be restoring nothing while reporting success.
    std::optional<std::vector<rclcpp::Parameter>> read_controller_params(
        const std::vector<std::string> &names)
    {
        if (!param_client_->wait_for_service(1s)) {
            return std::nullopt;
        }
        auto future = param_client_->get_parameters(names);
        if (future.wait_for(std::chrono::duration<double>(params_.profile_timeout_s)) !=
            std::future_status::ready) {
            return std::nullopt;
        }
        const auto values = future.get();
        if (values.size() != names.size()) {
            return std::nullopt;
        }
        for (const auto &value : values) {
            if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
                return std::nullopt;
            }
        }
        return values;
    }

    // ── freshness ───────────────────────────────────────────────────────

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
            stamp_age = (get_clock()->now() - stamp).seconds();
        } catch (const std::exception &) {
            return arrival_age;  // mismatched clock types (e.g. sim time)
        }
        if (stamp_age < -0.05) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "Pose header stamp is %.3f s in the future - using "
                                 "arrival time (unsynchronized camera clock?).",
                                 -stamp_age);
            return arrival_age;
        }
        return stamp_age;
    }

    // Age of the latest filtered marker pose, NAN before the first.
    double marker_age_s()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!latest_pose_) {
            return NAN;
        }
        return pose_age_s(rclcpp::Time(latest_pose_->header.stamp), latest_pose_arrival_);
    }

    // Arrival age of the latest RAW detection, NAN before the first.
    double raw_age_s()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!latest_raw_arrival_) {
            return NAN;
        }
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - *latest_raw_arrival_).count();
    }

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

    // A frozen robot state against a moving goal reads as a constant error
    // and would wind the lead to its clamp on stale data, so the measured
    // pose is freshness-checked exactly like vision is.
    std::optional<tf2::Transform> fresh_measured_ee()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!has_state_) {
            return std::nullopt;
        }
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - latest_state_arrival_).count();
        if (age > params_.state_timeout_s) {
            return std::nullopt;
        }
        return latest_ee_;
    }

    // ── frames ──────────────────────────────────────────────────────────

    // The fixed tool offset T_tcp_ee, measured rather than assumed: TF and
    // o_t_ee sampled together, and only accepted when three consecutive
    // samples agree, which proves it is static and that TF and FCI agree on
    // the kinematics. A wrong offset is a fixed error the 10 mm lead clamp
    // cannot absorb, so the lead would simply sit saturated.
    std::optional<tf2::Transform> capture_tool_offset(std::string &why)
    {
        auto agree = [](const tf2::Transform &a, const tf2::Transform &b) {
            const tf2::Vector3 turn = tracking_law::rotation_vector(
                b.getRotation() * a.getRotation().inverse());
            return (a.getOrigin() - b.getOrigin()).length() <= 0.001 &&
                   turn.length() <= 0.001;
        };
        std::vector<tf2::Transform> samples;
        why = "no robot state and no TF were sampled";
        for (int attempt = 0; attempt < 20 && rclcpp::ok(); ++attempt) {
            if (attempt > 0) {
                std::this_thread::sleep_for(50ms);
            }
            const auto measured = fresh_measured_ee();
            if (!measured) {
                why = "no fresh robot state on " + params_.state_topic;
                continue;
            }
            tf2::Transform t_base_tcp;
            try {
                tf2::fromMsg(tf_buffer_.lookupTransform(
                    params_.base_frame, params_.eef_frame,
                    tf2::TimePointZero).transform, t_base_tcp);
            } catch (const tf2::TransformException &e) {
                why = "TF " + params_.base_frame + " -> " + params_.eef_frame +
                      " unavailable: " + e.what();
                continue;
            }
            samples.push_back(t_base_tcp.inverse() * *measured);
            const size_t n = samples.size();
            if (n >= 3 && agree(samples[n - 3], samples[n - 2]) &&
                agree(samples[n - 2], samples[n - 1])) {
                return samples.back();
            }
            why = "the tool offset did not settle - is the arm at rest?";
        }
        return std::nullopt;
    }

    // The marker pose expressed in `frame` (the base frame for the goal, the
    // camera frame for the in-plane hold). Both lookups use a ZERO timeout:
    // mating_node can afford a 0.1 s blocking wait on its 0.4 s sequencing
    // thread, but inside a 20 ms timer one missing transform would stall five
    // ticks. A tick that cannot transform holds.
    std::optional<tf2::Transform> marker_in(const std::string &frame,
                                            const geometry_msgs::msg::PoseStamped &marker_msg)
    {
        geometry_msgs::msg::PoseStamped marker_out;
        try {
            // Transform at the image timestamp: the marker pose was captured
            // at that instant, so it must compose with the robot's TF from
            // the same instant. "Latest" is wrong by however far the arm
            // moved between capture and this control cycle (eye-in-hand).
            geometry_msgs::msg::TransformStamped tf;
            const rclcpp::Time stamp(marker_msg.header.stamp);
            if (stamp.nanoseconds() == 0) {
                tf = tf_buffer_.lookupTransform(
                    frame, marker_msg.header.frame_id, tf2::TimePointZero);
            } else {
                try {
                    tf = tf_buffer_.lookupTransform(
                        frame, marker_msg.header.frame_id, stamp);
                } catch (const tf2::TransformException &e) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "TF at image stamp unavailable (%s) - "
                                         "falling back to latest transform.", e.what());
                    tf = tf_buffer_.lookupTransform(
                        frame, marker_msg.header.frame_id, tf2::TimePointZero);
                }
            }
            tf2::doTransform(marker_msg, marker_out, tf);
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "TF %s -> %s unavailable: %s",
                                 marker_msg.header.frame_id.c_str(),
                                 frame.c_str(), e.what());
            return std::nullopt;
        }

        tf2::Transform t_frame_marker;
        tf2::fromMsg(marker_out.pose, t_frame_marker);
        return t_frame_marker;
    }

    // Empty if the impedance controller is ACTIVE, otherwise why START must
    // refuse. transition_event only catches it LEAVING active; one that never
    // got there - inactive after a RELEASE, or not spawned - would take the
    // gain step and a 50 Hz stream while the arm answers to something else.
    std::string controller_inactive()
    {
        if (!list_client_->wait_for_service(1s)) {
            return "the controller manager is not answering - is ros2_control up?";
        }
        auto future = list_client_->async_send_request(
            std::make_shared<ListControllers::Request>());
        if (future.wait_for(std::chrono::duration<double>(params_.profile_timeout_s)) !=
            std::future_status::ready) {
            return "the controller manager did not answer";
        }
        for (const auto &c : future.get()->controller) {
            if (c.name == params_.impedance_controller) {
                return c.state == "active"
                           ? std::string()
                           : params_.impedance_controller + " is " + c.state +
                                 ", not active - put the arm on it first";
            }
        }
        return params_.impedance_controller + " is not loaded";
    }

    // ── operator gates ──────────────────────────────────────────────────

    void on_start(const std::shared_ptr<Trigger::Response> &res)
    {
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            if (tracking_ || restore_) {
                res->success = false;
                res->message = tracking_ ? "already tracking"
                                         : "the last session is still stopping - try again";
                return;
            }
            starting_ = true;
            start_aborted_ = false;
            set_state("starting", "", DiagnosticStatus::OK);   // before any STOP can say idle
        }
        const std::string why = start_session();
        bool aborted = false;
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            starting_ = false;
            aborted = start_aborted_;
        }
        res->success = why.empty();
        res->message = why.empty() ? "tracking" : why;
        // A STOP during START has already said why; anything else is a
        // refusal the panel must show loudly.
        if (!why.empty() && !aborted) {
            set_state("idle", why, DiagnosticStatus::ERROR);
        }
    }

    // Everything ~/start_tracking does, in order. Empty on success, otherwise
    // why it refused.
    std::string start_session()
    {
        const std::string inactive = controller_inactive();
        if (!inactive.empty()) {
            return inactive;
        }
        // The ALIGN goal cell_panel wrote for this session. Read here and
        // only here: a mid-run write must not move the goal under the arm.
        std::string camera_frame = params_.camera_frame;
        double standoff_m = NAN;
        double inplane_deg = NAN;
        bool inplane_hold = false;
        get_parameter("tracking_camera_frame", camera_frame);
        get_parameter("tracking_standoff_m", standoff_m);
        get_parameter("tracking_inplane_deg", inplane_deg);
        get_parameter("tracking_inplane_hold", inplane_hold);
        if (!std::isfinite(standoff_m) || standoff_m <= 0.0) {
            return "tracking_standoff_m must be finite and positive";
        }
        if (!inplane_hold && !std::isfinite(inplane_deg)) {
            return "tracking_inplane_deg must be finite";
        }

        std::string why;
        const auto offset = capture_tool_offset(why);
        if (!offset) {
            return "could not measure the tool offset: " + why;
        }
        // Static, published by the launch. Without it the goal would hold
        // the TCP, not the camera, where ALIGN left the camera.
        tf2::Transform t_tcp_cam;
        try {
            tf2::fromMsg(tf_buffer_.lookupTransform(params_.eef_frame, camera_frame,
                                                    tf2::TimePointZero).transform,
                         t_tcp_cam);
        } catch (const tf2::TransformException &e) {
            return "TF " + params_.eef_frame + " -> " + camera_frame +
                   " unavailable - is the hand-eye transform published? " + e.what();
        }
        double inplane_rad = inplane_deg * M_PI / 180.0;
        if (inplane_hold) {
            // cell_panel's in-plane 'off': keep the angle the camera sees now.
            const auto pose = fresh_marker_pose();
            const auto t_cam_marker =
                pose ? marker_in(camera_frame, *pose) : std::optional<tf2::Transform>();
            if (!t_cam_marker) {
                return "tracking_inplane_hold needs the marker in view in " + camera_frame +
                       " at start";
            }
            inplane_rad = tracking_law::inplane_rad(t_cam_marker->getRotation());
        }

        const auto measured = fresh_measured_ee();
        if (!measured) {
            return "no fresh robot state on " + params_.state_topic;
        }
        const auto floating = read_controller_params({"float_mode"});
        if (!floating) {
            return "could not read float_mode from " + params_.impedance_controller;
        }
        if (floating->front().as_bool()) {
            return "the controller is in float_mode - press HOLD first; "
                   "a free-floating arm must not be gain-stepped";
        }
        // If the node cannot restore what it is about to change, it does not
        // change it. The panel retunes gains live, so the shipped defaults
        // are not necessarily what the operator had.
        const auto snapshot = read_controller_params(
            {"k_pos_tool", "k_rot_tool", "damping_ratio",
             "setpoint_slew_mps", "setpoint_slew_rps"});
        if (!snapshot) {
            return "could not read the current gains from " + params_.impedance_controller +
                   " - refusing to change what cannot be put back";
        }
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            if (start_aborted_) {
                return "stopped during start";
            }
            restore_ = snapshot;
        }

        // Re-seed the equilibrium at the arm and let it settle BEFORE the
        // gain step: k_pos 150 -> 1500 is a tenfold raise applied to whatever
        // equilibrium error already exists, and 20 mm of it saturates the
        // 30 N ceiling instantly.
        publish_equilibrium(*measured);
        std::this_thread::sleep_for(
            std::chrono::duration<double>(params_.settle_s));
        {
            // A STOP during the settle has already restored the snapshot and
            // said so; the gain step must not land after it.
            std::lock_guard<std::mutex> lock(control_mutex_);
            if (start_aborted_) {
                return "stopped during start";
            }
        }
        const auto [ok, reason] = apply_params(profile_);

        std::lock_guard<std::mutex> lock(control_mutex_);
        if (start_aborted_) {
            // The STOP restored the snapshot, but the profile may have landed
            // after it: put it back again, whichever order the controller saw.
            const auto [back, why_not] = apply_params(*snapshot);
            if (!back) {
                RCLCPP_ERROR(get_logger(),
                             "Stopped during start, but the gains were NOT restored "
                             "(%s) - the '%s' profile may be in force on %s.",
                             why_not.c_str(), params_.gain_profile.c_str(),
                             params_.impedance_controller.c_str());
                set_state("idle", "stopped during start - gains NOT restored: " + why_not,
                          DiagnosticStatus::WARN);
                return "stopped during start, but the gains were NOT restored: " + why_not;
            }
            return "stopped during start, gains restored";
        }
        if (!ok) {
            restore_.reset();
            return reason;
        }

        cfg_.z_floor_m = measured->getOrigin().z() - params_.floor_below_start_m;
        lead_ = {};
        held_ = false;
        t_tcp_ee_ = *offset;
        t_tcp_cam_ = t_tcp_cam;
        standoff_m_ = standoff_m;
        inplane_rad_ = inplane_rad;
        open_log();
        tracking_ = true;   // last: everything the tick reads is in place
        starting_ = false;
        {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            status_.pos_err_mm = NAN;
            status_.rot_err_deg = NAN;
            status_.lead_mm = 0.0;
            status_.lead_deg = 0.0;
            status_.standoff_m = standoff_m;
            status_.inplane_deg = inplane_rad * 180.0 / M_PI;
        }
        set_state("tracking", "", DiagnosticStatus::OK);
        const tf2::Vector3 &t = offset->getOrigin();
        RCLCPP_INFO(get_logger(),
                    "TRACKING on profile '%s': camera %.0f mm off the marker at "
                    "%.1f deg in-plane%s, over-lead policy %s, slew %.3f m/s, Z floor "
                    "%.0f mm, tool offset %.1f mm at %.2f deg (%s -> the controller's "
                    "EE frame).",
                    params_.gain_profile.c_str(), standoff_m * 1000.0,
                    inplane_rad * 180.0 / M_PI, inplane_hold ? " (held as found)" : "",
                    tracking_law::over_lead_name(policy_.load()),
                    profile_.at(3).as_double(), cfg_.z_floor_m * 1000.0,
                    t.length() * 1000.0,
                    tracking_law::rotation_vector(offset->getRotation()).length() *
                        180.0 / M_PI,
                    params_.eef_frame.c_str());
        return {};
    }

    // Stop publishing, relax the spring where the arm actually is, and put
    // back the gains we changed. Stopping always reports success: publishing
    // must halt whatever the controller says about the restore. `why` is what
    // ~/status then reports, at WARN for a fault.
    std::pair<bool, std::string> halt_tracking(std::string why, bool fault)
    {
        // Serialised against tick(): without this, STOP races an in-flight
        // tick and the LAST message on the wire can be the leading
        // equilibrium rather than the hold - the opposite of stopping.
        std::lock_guard<std::mutex> lock(control_mutex_);
        // A STOP that lands during START must win: START checks this before
        // it commits and backs out, restoring the gains itself if it got as
        // far as changing them.
        if (starting_) {
            start_aborted_ = true;
        }
        // tick() already ended an over-lead 'stop' but could not restore the
        // gains under this lock; its reason is the real one, whoever gets here.
        if (stop_pending_.exchange(false)) {
            why = pending_reason_;
            fault = true;
        }
        const uint8_t level = fault ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
        const bool was_tracking = tracking_.exchange(false);
        if (!was_tracking && !restore_) {
            if (!starting_) {
                return {true, "tracking was not running"};
            }
            set_state("idle", why, level);
            return {true, "start aborted"};
        }
        set_state("stopping", why, level);
        lead_ = {};
        const auto measured = fresh_measured_ee();
        if (measured) {
            publish_equilibrium(*measured);
        } else {
            RCLCPP_WARN(get_logger(), "No fresh robot state at stop - the "
                                      "equilibrium is left where it was.");
        }
        close_log();
        if (!restore_) {
            set_state("idle", why, level);
            return {true, "tracking stopped"};
        }
        const auto [ok, reason] = apply_params(*restore_);
        restore_.reset();
        if (!ok) {
            RCLCPP_ERROR(get_logger(),
                         "Tracking stopped, but the gains were NOT restored (%s) - "
                         "the '%s' profile is still in force on %s.", reason.c_str(),
                         params_.gain_profile.c_str(),
                         params_.impedance_controller.c_str());
            set_state("idle", why + " - gains NOT restored: " + reason,
                      DiagnosticStatus::WARN);
            return {true, "tracking stopped, but the gains were NOT restored: " +
                          reason};
        }
        set_state("idle", why, level);
        return {true, "tracking stopped, gains restored"};
    }

    // ── the loop ────────────────────────────────────────────────────────

    void tick()
    {
        if (!tracking_) {
            return;
        }
        // Held for the whole body so a STOP cannot interleave between the
        // integrator step and the publish. Re-checked under the lock: halt
        // may have run between the test above and acquiring it.
        std::lock_guard<std::mutex> lock(control_mutex_);
        if (!tracking_) {
            return;
        }
        const auto measured = fresh_measured_ee();
        if (!measured) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Robot state stale on %s - holding.",
                                 params_.state_topic.c_str());
            return hold(measured, NAN, "robot state stale");
        }
        const auto pose = fresh_marker_pose();
        if (!pose) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Marker pose stale on %s - holding where the arm "
                                 "is. Predictions never drive committed motion.",
                                 params_.pose_topic.c_str());
            return hold(measured, NAN, "marker stale");
        }
        const double stamp_s = rclcpp::Time(pose->header.stamp).seconds();
        // Predictions never drive committed motion (TRACKING_SPEC Decision
        // 5): the filtered pose coasts on its model through a dropout, so
        // motion also needs the detector itself to have seen the marker.
        if (!tracking_law::raw_fresh(raw_age_s(), params_.raw_timeout_s)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No raw detection on %s within %.2f s - holding.",
                                 params_.raw_pose_topic.c_str(), params_.raw_timeout_s);
            return hold(measured, stamp_s, "no fresh raw detection");
        }
        const auto t_base_marker = marker_in(params_.base_frame, *pose);
        if (!t_base_marker) {
            return hold(measured, stamp_s, "no transform");
        }
        const tf2::Transform goal_ee = tracking_law::goal_in_ee(
            tracking_law::camera_centred_goal(*t_base_marker, standoff_m_, inplane_rad_,
                                              t_tcp_cam_),
            t_tcp_ee_);

        tf2::Vector3 pos_err;
        tf2::Quaternion rot_err;
        double rot_angle = 0.0;
        mating_geometry::pose_error(*measured, goal_ee, pos_err, rot_err, rot_angle);
        lead_ = tracking_law::advance_lead(lead_, pos_err,
                                           tracking_law::rotation_vector(rot_err),
                                           cfg_, params_.period_s);

        const tf2::Transform eq = tracking_law::equilibrium_from(goal_ee, lead_);
        const tracking_law::OverLead policy = policy_;
        const tracking_law::Verdict verdict =
            tracking_law::decide(eq, *measured, cfg_, policy);
        const double pos_err_mm = pos_err.length() * 1000.0;
        const double rot_err_deg = rot_angle * 180.0 / M_PI;
        std::string state = "tracking";
        switch (verdict.act) {
        case tracking_law::Verdict::Act::kPublish:
            publish_equilibrium(verdict.eq);
            held_ = false;
            break;
        case tracking_law::Verdict::Act::kHold:
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Not publishing: %s", verdict.reason.c_str());
            hold_once(measured);
            state = "holding";
            break;
        case tracking_law::Verdict::Act::kStop:
            // Ends here, holding where the arm is. The gain restore is
            // halt_tracking()'s, which needs the lock this tick holds; the
            // status timer runs it, or a STOP does if it gets there first.
            RCLCPP_WARN(get_logger(), "Stopping tracking: %s", verdict.reason.c_str());
            publish_equilibrium(*measured);
            tracking_ = false;
            pending_reason_ = verdict.reason;
            stop_pending_ = true;
            state = "stopping";
            break;
        }
        const bool published = verdict.act == tracking_law::Verdict::Act::kPublish;
        log_record(stamp_s, &goal_ee, &*measured, pos_err_mm, rot_err_deg, published,
                   verdict.reason, policy);
        report_tick(state, published ? std::string() : verdict.reason, pos_err_mm,
                    rot_err_deg);
    }

    // A tick that cannot compute a goal: hold, and log and report why with
    // null geometry.
    void hold(const std::optional<tf2::Transform> &measured, double stamp_s,
              const std::string &reason)
    {
        hold_once(measured);
        log_record(stamp_s, nullptr, measured ? &*measured : nullptr, NAN, NAN, false,
                   reason, policy_);
        report_tick("holding", reason, NAN, NAN);
    }

    // Merely ceasing to publish does NOT stop the arm: the controller's
    // slew limiter keeps stepping toward the last target every cycle while
    // it has one, so a stale marker would let the arm travel the whole
    // outstanding lag. Publishing the measured pose once converges the slew
    // to a genuine hold with zero lead. lead_ is left alone, so a brief
    // occlusion does not lose the integrator and reacquire does not lurch.
    void hold_once(const std::optional<tf2::Transform> &measured)
    {
        if (held_ || !measured) {
            return;
        }
        publish_equilibrium(*measured);
        held_ = true;
    }

    void publish_equilibrium(const tf2::Transform &pose)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = params_.base_frame;
        // The controller drops any pose whose |q| is outside 1.0 +/- 0.1.
        tf2::Transform normalised = pose;
        normalised.setRotation(pose.getRotation().normalized());
        tf2::toMsg(normalised, msg.pose);
        eq_pub_->publish(msg);
    }

    // ── ~/status ────────────────────────────────────────────────────────

    // Published at once when the state or level changes, so the panel's
    // banner never lags a transition; a new reason alone rides the 5 Hz
    // timer, or a hold whose reason quotes a distance would publish at 50 Hz.
    void set_state(const std::string &state, const std::string &reason, uint8_t level)
    {
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            const bool changed = status_.state != state || status_.level != level;
            status_.state = state;
            status_.reason = reason;
            status_.level = level;
            if (!changed) {
                return;
            }
        }
        publish_status();
    }

    // Tick thread, under control_mutex_ - the only place lead_ may be read.
    void report_tick(const std::string &state, const std::string &reason,
                     double pos_err_mm, double rot_err_deg)
    {
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_.pos_err_mm = pos_err_mm;
            status_.rot_err_deg = rot_err_deg;
            status_.lead_mm = lead_.pos.length() * 1000.0;
            status_.lead_deg = lead_.rot.length() * 180.0 / M_PI;
        }
        set_state(state, reason,
                  state == "tracking" ? DiagnosticStatus::OK : DiagnosticStatus::WARN);
    }

    void publish_status()
    {
        // Copied and published under one lock: two threads that copy in one
        // order and publish in the other would latch the older state.
        std::lock_guard<std::mutex> lock(status_mutex_);
        const Status s = status_;
        DiagnosticStatus msg;
        msg.level = s.level;
        msg.name = get_name();
        msg.message = s.reason.empty() ? s.state : s.state + " - " + s.reason;
        auto add = [&msg](const std::string &key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = key;
            kv.value = value;
            msg.values.push_back(kv);
        };
        auto num = [](double v, int decimals) {   // NAN prints as "nan"
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
            return std::string(buf);
        };
        add("state", s.state);
        add("reason", s.reason);
        add("policy", tracking_law::over_lead_name(policy_));
        add("pos_err_mm", num(s.pos_err_mm, 1));
        add("rot_err_deg", num(s.rot_err_deg, 2));
        add("lead_mm", num(s.lead_mm, 1));
        add("lead_deg", num(s.lead_deg, 2));
        add("marker_age_s", num(marker_age_s(), 3));
        add("raw_age_s", num(raw_age_s(), 3));
        add("standoff_m", num(s.standoff_m, 3));
        add("inplane_deg", num(s.inplane_deg, 1));
        status_pub_->publish(msg);
    }

    // ── the V1-V6 log ───────────────────────────────────────────────────

    void open_log()
    {
        if (params_.log_dir.empty()) {
            return;
        }
        const std::time_t now = std::time(nullptr);
        char stamp[32] = {0};
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::gmtime(&now));
        const std::string path =
            params_.log_dir + "/tracking_" + stamp + ".jsonl";
        std::lock_guard<std::mutex> lock(log_mutex_);
        logf_.open(path);
        log_start_ = std::chrono::steady_clock::now();
        last_flush_ = log_start_;
        if (logf_.is_open()) {
            RCLCPP_INFO(get_logger(), "Tracking log -> %s", path.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Could not open %s - tracking without a "
                                       "log; V1-V6 cannot be read back.",
                         path.c_str());
        }
    }

    void close_log()
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        if (logf_.is_open()) {
            logf_.close();
        }
    }

    // One JSON object per tick, the way cell_panel's trace records a
    // session: this is what makes TRACKING_SPEC section 7 a log read rather
    // than a rerun. `reason` is why a tick held or stopped, or what a clamp
    // cut; a tick that held before computing a goal carries null geometry -
    // nothing was computed, and a zero there would read as a perfect track.
    void log_record(double stamp_s, const tf2::Transform *goal,
                    const tf2::Transform *meas, double pos_err_mm,
                    double rot_err_deg, bool published, const std::string &reason,
                    tracking_law::OverLead policy)
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        if (!logf_.is_open()) {
            return;
        }
        auto num = [](double v) {
            return std::isfinite(v) ? std::to_string(v) : std::string("null");
        };
        auto pose7 = [&num](const tf2::Transform *t) {
            if (t == nullptr) {
                return std::string("null");
            }
            const tf2::Vector3 &p = t->getOrigin();
            const tf2::Quaternion q = t->getRotation();
            return "[" + num(p.x()) + "," + num(p.y()) + "," + num(p.z()) + "," +
                   num(q.x()) + "," + num(q.y()) + "," + num(q.z()) + "," +
                   num(q.w()) + "]";
        };
        logf_ << "{\"t\":" << num(std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - log_start_).count())
              << ",\"stamp\":" << num(stamp_s)
              << ",\"goal\":" << pose7(goal)
              << ",\"meas\":" << pose7(meas)
              << ",\"pos_err_mm\":" << num(pos_err_mm)
              << ",\"rot_err_deg\":" << num(rot_err_deg)
              << ",\"lead_mm\":" << num(lead_.pos.length() * 1000.0)
              << ",\"lead_deg\":" << num(lead_.rot.length() * 180.0 / M_PI)
              << ",\"published\":" << (published ? "true" : "false")
              << ",\"policy\":\"" << tracking_law::over_lead_name(policy) << "\""
              << ",\"reason\":\"" << reason << "\"}\n";
        // At most once a second, and on close: a flush per tick is a write
        // syscall at 50 Hz under control_mutex_. A crash loses about 1 s.
        const auto now = std::chrono::steady_clock::now();
        if (now - last_flush_ >= 1s) {
            logf_.flush();
            last_flush_ = now;
        }
    }

    // ── state ───────────────────────────────────────────────────────────

    Params params_;
    tracking_law::Config cfg_;
    std::vector<rclcpp::Parameter> profile_;
    // Serialises halt_tracking() against the tick's read-modify-publish, and
    // guards restore_, which both of them touch.
    std::mutex control_mutex_;
    std::optional<std::vector<rclcpp::Parameter>> restore_;

    rclcpp::CallbackGroup::SharedPtr srv_group_;
    rclcpp::CallbackGroup::SharedPtr stop_group_;
    rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr
        transition_sub_;
    rclcpp::CallbackGroup::SharedPtr param_group_;
    rclcpp::CallbackGroup::SharedPtr tick_group_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr raw_sub_;
    rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr eq_pub_;
    rclcpp::Publisher<DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::Client<ListControllers>::SharedPtr list_client_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
    rclcpp::Service<Trigger>::SharedPtr start_srv_;
    rclcpp::Service<Trigger>::SharedPtr stop_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::mutex pose_mutex_;
    std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
    std::chrono::steady_clock::time_point latest_pose_arrival_;
    std::optional<std::chrono::steady_clock::time_point> latest_raw_arrival_;

    std::mutex state_mutex_;
    tf2::Transform latest_ee_;
    bool has_state_{false};
    std::chrono::steady_clock::time_point latest_state_arrival_;

    // The node comes up NOT tracking and publishes nothing until an operator
    // presses START.
    std::atomic<bool> tracking_{false};
    // START in progress, and whether a STOP landed during it (under
    // control_mutex_). starting_ is atomic for the transition callback's
    // lock-free test.
    std::atomic<bool> starting_{false};
    bool start_aborted_{false};
    // An over-lead 'stop' tick() could not finish; its reason under
    // control_mutex_.
    std::atomic<bool> stop_pending_{false};
    std::string pending_reason_;
    // Live: the on-set-parameters callback writes it, the tick reads it.
    std::atomic<tracking_law::OverLead> policy_{tracking_law::OverLead::kHold};
    // Timer thread only, except for the zeroing under tracking_ = false.
    // The session values are written by START before tracking_ goes true.
    tracking_law::Lead lead_;
    tf2::Transform t_tcp_ee_{tf2::Transform::getIdentity()};
    tf2::Transform t_tcp_cam_{tf2::Transform::getIdentity()};
    double standoff_m_{0.10};
    double inplane_rad_{0.0};
    bool held_{false};

    std::mutex status_mutex_;
    Status status_;

    std::mutex log_mutex_;
    std::ofstream logf_;
    std::chrono::steady_clock::time_point log_start_;
    std::chrono::steady_clock::time_point last_flush_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    // Without this the get_parameter idiom above finds nothing and every
    // tracking_* value in the params yaml is silently ignored.
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<TrackingNode>(node_options);

    // MUST be multi-threaded: ~/start_tracking blocks on the impedance
    // controller's parameter service, and only another executor thread can
    // deliver the reply.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
}
