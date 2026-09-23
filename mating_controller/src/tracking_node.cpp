// Continuous marker tracking on the Cartesian-impedance backend
// (TRACKING_SPEC.md section 5).
//
// Streams an equilibrium pose at 50 Hz so the arm FOLLOWS the marker instead
// of stepping to it. The goal comes from vision through the same
// mating_geometry::standoff_goal the stepped backend uses; a bounded
// integrator pushes the equilibrium past that goal until the MEASURED error
// closes, which is how the friction residual F/k gets taken out - the
// controller cannot know it is stuck, only vision can. All of that law is
// pure and lives in tracking_law.hpp; this file is only its plumbing.
//
// Three properties are load-bearing and easy to break:
//
//   * FRAMES. standoff_goal targets EEF_FRAME_ID (fr3_hand_tcp) while the
//     controller seeds and measures at franka::Frame::kEndEffector. Nothing
//     in this cell asserts the two coincide - it carries a D405 wrist mount.
//     The offset is MEASURED at ~/start_tracking from TF and o_t_ee sampled
//     together, and start is refused if it cannot be established.
//   * THREADING. ~/start_tracking blocks on the impedance controller's
//     parameter service, so this node MUST be spun by a MultiThreadedExecutor
//     and the services, the parameter client and the timer MUST sit in
//     different callback groups. Under rclcpp::spin the reply can never be
//     delivered: the one executor thread is inside the callback waiting for
//     it. See mating_node.cpp L1134-1136 for the same reasoning.
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
using std_srvs::srv::Trigger;

struct Params
{
    std::string pose_topic{"/aruco/pose"};
    std::string base_frame{"fr3_link0"};
    std::string eef_frame{"fr3_hand_tcp"};
    double vision_timeout_s{0.6};

    // Where the connector sits relative to the marker, in the marker frame -
    // the same keys mating_node reads, so both backends aim at one goal.
    double connector_offset_x{0.0};
    double connector_offset_y{0.0};
    double connector_offset_z{0.0};
    double tool_yaw_offset_deg{0.0};
    double standoff_height_m{0.10};

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
    double floor_below_start_m{0.030};
    double state_timeout_s{0.1};
    double profile_timeout_s{2.0};
    double settle_s{1.0};
    double max_force_n{30.0};
    double max_torque_nm{10.0};
    std::string log_dir;
    std::string gain_profile{"track"};
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
        // A lead that could exceed 15 N must be a startup refusal, not a
        // surprise with the arm moving.
        const std::string why = tracking_law::validate_config(
            cfg_, max_gain(0), max_gain(1), params_.max_force_n,
            params_.max_torque_nm);
        if (!why.empty()) {
            throw std::runtime_error(why);
        }

        srv_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // STOP gets its OWN group. Sharing srv_group_ made "STOP is always
        // live" false: a MutuallyExclusive group runs one callback at a time,
        // so a stop queued behind an in-flight start, and on_start blocks for
        // up to ~11 s across three parameter round-trips.
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
        // o_t_ee: the same field impedance_panel anchors on, and the pose the
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
        // ACTIVE - RELEASE, a crash, a switch by move_l - tracking halts
        // itself. Otherwise an orphaned tracker keeps streaming at 50 Hz and
        // the next activation resumes autonomous motion with nobody pressing
        // anything.
        // Its own group: tick() holds control_mutex_ for its whole body, and
        // halt_tracking() takes the same lock, so this callback must never be
        // serialised behind the tick.
        rclcpp::SubscriptionOptions transition_options;
        transition_options.callback_group = stop_group_;
        transition_sub_ = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
            "/" + params_.impedance_controller + "/transition_event",
            rclcpp::QoS(10),
            [this](lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
                if (!tracking_ ||
                    msg->goal_state.id ==
                        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    return;
                }
                RCLCPP_WARN(get_logger(),
                            "%s left ACTIVE (now '%s') - stopping tracking. The "
                            "arm is no longer on the impedance controller.",
                            params_.impedance_controller.c_str(),
                            msg->goal_state.label.c_str());
                halt_tracking();
            },
            transition_options);

        eq_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            params_.equilibrium_topic, rclcpp::QoS(1));
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            this, params_.impedance_controller, rmw_qos_profile_parameters,
            param_group_);

        start_srv_ = create_service<Trigger>(
            "~/start_tracking",
            [this](const std::shared_ptr<Trigger::Request>,
                   std::shared_ptr<Trigger::Response> res) { on_start(res); },
            rmw_qos_profile_services_default, srv_group_);
        stop_srv_ = create_service<Trigger>(
            "~/stop_tracking",
            [this](const std::shared_ptr<Trigger::Request>,
                   std::shared_ptr<Trigger::Response> res) {
                const auto [ok, msg] = halt_tracking();
                res->success = ok;
                res->message = msg;
            },
            rmw_qos_profile_services_default, stop_group_);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(params_.period_s)),
            [this]() { tick(); }, tick_group_);

        // Ctrl-C must not leave the arm stiff and leaning on a lead.
        // on_shutdown() runs AFTER rcl_shutdown, by which point the parameter
        // future can never complete and the restore silently fails - leaving
        // the arm on 10x the mating stiffness. A PRE-shutdown callback runs
        // while the executor still spins, so the reply can still arrive.
        get_node_base_interface()->get_context()->add_pre_shutdown_callback(
            [this]() {
                const auto [ok, msg] = halt_tracking();
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
                    "~/start_tracking. Profile '%s', %.0f Hz.",
                    params_.gain_profile.c_str(), 1.0 / params_.period_s);
    }

    ~TrackingNode() override
    {
        halt_tracking();
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
        get("BASE_FRAME_ID", p.base_frame);
        get("EEF_FRAME_ID", p.eef_frame);
        get("vision_timeout_s", p.vision_timeout_s);
        get("connector_offset_x", p.connector_offset_x);
        get("connector_offset_y", p.connector_offset_y);
        get("connector_offset_z", p.connector_offset_z);
        get("tool_yaw_offset_deg", p.tool_yaw_offset_deg);
        get("standoff_height_m", p.standoff_height_m);
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

    // Marker pose -> desired TCP standoff pose, in the base frame. Both
    // lookups use a ZERO timeout: mating_node can afford a 0.1 s blocking wait on
    // its 0.4 s sequencing thread, but inside a 20 ms timer one missing
    // transform would stall five ticks. A tick that cannot transform holds.
    std::optional<tf2::Transform> compute_standoff_goal(
        const geometry_msgs::msg::PoseStamped &marker_msg)
    {
        geometry_msgs::msg::PoseStamped marker_in_base;
        try {
            // Transform at the image timestamp: the marker pose was captured
            // at that instant, so it must compose with the robot's TF from
            // the same instant. "Latest" is wrong by however far the arm
            // moved between capture and this control cycle (eye-in-hand).
            geometry_msgs::msg::TransformStamped tf;
            const rclcpp::Time stamp(marker_msg.header.stamp);
            if (stamp.nanoseconds() == 0) {
                tf = tf_buffer_.lookupTransform(
                    params_.base_frame, marker_msg.header.frame_id, tf2::TimePointZero);
            } else {
                try {
                    tf = tf_buffer_.lookupTransform(
                        params_.base_frame, marker_msg.header.frame_id, stamp);
                } catch (const tf2::TransformException &e) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "TF at image stamp unavailable (%s) - "
                                         "falling back to latest transform.", e.what());
                    tf = tf_buffer_.lookupTransform(
                        params_.base_frame, marker_msg.header.frame_id,
                        tf2::TimePointZero);
                }
            }
            tf2::doTransform(marker_msg, marker_in_base, tf);
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "TF %s -> %s unavailable: %s",
                                 marker_msg.header.frame_id.c_str(),
                                 params_.base_frame.c_str(), e.what());
            return std::nullopt;
        }

        tf2::Transform t_base_marker;
        tf2::fromMsg(marker_in_base.pose, t_base_marker);

        return mating_geometry::standoff_goal(
            t_base_marker,
            params_.connector_offset_x, params_.connector_offset_y,
            params_.connector_offset_z, params_.standoff_height_m,
            params_.tool_yaw_offset_deg * M_PI / 180.0);
    }

    // ── operator gates ──────────────────────────────────────────────────

    void on_start(const std::shared_ptr<Trigger::Response> &res)
    {
        auto refuse = [&res](const std::string &why) {
            res->success = false;
            res->message = why;
        };
        if (tracking_) {
            return refuse("already tracking");
        }
        std::string why;
        const auto offset = capture_tool_offset(why);
        if (!offset) {
            return refuse("could not measure the tool offset: " + why);
        }
        const auto measured = fresh_measured_ee();
        if (!measured) {
            return refuse("no fresh robot state on " + params_.state_topic);
        }
        const auto floating = read_controller_params({"float_mode"});
        if (!floating) {
            return refuse("could not read float_mode from " +
                          params_.impedance_controller);
        }
        if (floating->front().as_bool()) {
            return refuse("the controller is in float_mode - press HOLD first; "
                          "a free-floating arm must not be gain-stepped");
        }
        // If the node cannot restore what it is about to change, it does not
        // change it. The panel retunes gains live, so the shipped defaults
        // are not necessarily what the operator had.
        restore_ = read_controller_params(
            {"k_pos_tool", "k_rot_tool", "damping_ratio",
             "setpoint_slew_mps", "setpoint_slew_rps"});
        if (!restore_) {
            return refuse("could not read the current gains from " +
                          params_.impedance_controller +
                          " - refusing to change what cannot be put back");
        }
        cfg_.z_floor_m = measured->getOrigin().z() - params_.floor_below_start_m;

        // Re-seed the equilibrium at the arm and let it settle BEFORE the
        // gain step: k_pos 150 -> 1500 is a tenfold raise applied to whatever
        // equilibrium error already exists, and 20 mm of it saturates the
        // 30 N ceiling instantly.
        publish_equilibrium(*measured);
        std::this_thread::sleep_for(
            std::chrono::duration<double>(params_.settle_s));
        const auto [ok, reason] = apply_params(profile_);
        if (!ok) {
            restore_.reset();
            return refuse(reason);
        }

        lead_ = {};
        held_ = false;
        t_tcp_ee_ = *offset;
        open_log();
        tracking_ = true;   // last: everything the tick reads is in place
        const tf2::Vector3 &t = offset->getOrigin();
        RCLCPP_INFO(get_logger(),
                    "TRACKING on profile '%s': slew %.3f m/s, Z floor %.0f mm, "
                    "tool offset %.1f mm at %.2f deg (%s -> the controller's EE "
                    "frame).",
                    params_.gain_profile.c_str(),
                    profile_.at(3).as_double(), cfg_.z_floor_m * 1000.0,
                    t.length() * 1000.0,
                    tracking_law::rotation_vector(offset->getRotation()).length() *
                        180.0 / M_PI,
                    params_.eef_frame.c_str());
        res->success = true;
        res->message = "tracking";
    }

    // Stop publishing, relax the spring where the arm actually is, and put
    // back the gains we changed. Stopping always reports success: publishing
    // must halt whatever the controller says about the restore.
    std::pair<bool, std::string> halt_tracking()
    {
        // Serialised against tick(): without this, STOP races an in-flight
        // tick and the LAST message on the wire can be the leading
        // equilibrium rather than the hold - the opposite of stopping.
        std::lock_guard<std::mutex> lock(control_mutex_);
        const bool was_tracking = tracking_.exchange(false);
        if (!was_tracking && !restore_) {
            return {true, "tracking was not running"};
        }
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
            return {true, "tracking stopped, but the gains were NOT restored: " +
                          reason};
        }
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
            hold_once(measured);
            return log_record(NAN, nullptr, nullptr, NAN, NAN, "robot state stale");
        }
        const auto pose = fresh_marker_pose();
        if (!pose) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Marker pose stale on %s - holding where the arm "
                                 "is. Predictions never drive committed motion.",
                                 params_.pose_topic.c_str());
            hold_once(measured);
            return log_record(NAN, nullptr, &*measured, NAN, NAN, "marker stale");
        }
        const double stamp_s = rclcpp::Time(pose->header.stamp).seconds();
        const auto goal_tcp = compute_standoff_goal(*pose);
        if (!goal_tcp) {
            hold_once(measured);
            return log_record(stamp_s, nullptr, &*measured, NAN, NAN, "no transform");
        }
        const tf2::Transform goal_ee = tracking_law::goal_in_ee(*goal_tcp, t_tcp_ee_);

        tf2::Vector3 pos_err;
        tf2::Quaternion rot_err;
        double rot_angle = 0.0;
        mating_geometry::pose_error(*measured, goal_ee, pos_err, rot_err, rot_angle);
        lead_ = tracking_law::advance_lead(lead_, pos_err,
                                           tracking_law::rotation_vector(rot_err),
                                           cfg_, params_.period_s);

        const tf2::Transform eq = tracking_law::equilibrium_from(goal_ee, lead_);
        const std::string veto = tracking_law::publish_veto(eq, *measured, cfg_);
        const double pos_err_mm = pos_err.length() * 1000.0;
        const double rot_err_deg = rot_angle * 180.0 / M_PI;
        if (!veto.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Not publishing: %s", veto.c_str());
            hold_once(measured);
            return log_record(stamp_s, &goal_ee, &*measured, pos_err_mm,
                              rot_err_deg, veto);
        }
        publish_equilibrium(eq);
        held_ = false;
        log_record(stamp_s, &goal_ee, &*measured, pos_err_mm, rot_err_deg, "");
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

    // One JSON object per tick, the way impedance_panel.trace records a
    // session: this is what makes TRACKING_SPEC section 7 a log read rather
    // than a rerun. A held tick carries its reason and null geometry -
    // nothing was computed, and a zero there would read as a perfect track.
    void log_record(double stamp_s, const tf2::Transform *goal,
                    const tf2::Transform *meas, double pos_err_mm,
                    double rot_err_deg, const std::string &hold_reason)
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
              << ",\"published\":" << (hold_reason.empty() ? "true" : "false")
              << ",\"hold_reason\":\"" << hold_reason << "\"}\n";
        logf_.flush();
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
    rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr eq_pub_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::Service<Trigger>::SharedPtr start_srv_;
    rclcpp::Service<Trigger>::SharedPtr stop_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex pose_mutex_;
    std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
    std::chrono::steady_clock::time_point latest_pose_arrival_;

    std::mutex state_mutex_;
    tf2::Transform latest_ee_;
    bool has_state_{false};
    std::chrono::steady_clock::time_point latest_state_arrival_;

    // The node comes up NOT tracking and publishes nothing until an operator
    // presses START.
    std::atomic<bool> tracking_{false};
    // Timer thread only, except for the zeroing under tracking_ = false.
    tracking_law::Lead lead_;
    tf2::Transform t_tcp_ee_{tf2::Transform::getIdentity()};
    bool held_{false};

    std::mutex log_mutex_;
    std::ofstream logf_;
    std::chrono::steady_clock::time_point log_start_;
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
