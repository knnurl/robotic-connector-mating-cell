// Pure control law for continuous marker tracking on the Cartesian-impedance
// backend (TRACKING_SPEC.md section 5): the camera-centred goal, the bounded
// integrator that closes the friction residual the spring cannot, the
// TCP-to-EE frame conversion, and the over-lead policy and veto that guard
// what may be published. No ROS node code here so the law is unit-testable
// (see test/test_tracking_law.cpp).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace tracking_law
{

// Worst extra force the bounded integrator may add is k * lead_max.
// TRACKING_SPEC.md section 6. tools/fr3/cell/test_cell_pins.py scrapes these
// three constants by name, so keep them bare decimal literals.
inline constexpr double kLeadForceMaxN = 15.0;
inline constexpr double kFrictionBreakawayN = 6.5;    // N,  spec section 2, worst case
inline constexpr double kFrictionBreakawayNm = 0.6;   // Nm, spec section 2

// Every entry is a parameter in tools/fr3/fr3_params.yaml, never a constant in
// the node. The deadbands belong to the stiffness actually in use: they are
// F_friction / k, so they move whenever the track profile moves.
struct Config
{
    double ki{0.5};                  // 1/s
    double lead_max_m{0.010};        // m
    double lead_max_rad{0.017};      // rad
    double deadband_m{0.005};        // m,   kFrictionBreakawayN / track k_pos 1500
    double deadband_rad{0.007};      // rad, kFrictionBreakawayNm / k_rot 90
    double max_lead_m{0.060};        // m, mirrors cell_panel MAX_LEAD_MM
    double max_lead_rad{0.26};       // rad, the angular twin of max_lead_m
    double z_floor_m{0.0};           // m, absolute in the base frame (tracking_z_floor_m)
    // The value validated at startup. It is live-settable, so the node keeps
    // the current one itself and hands it to decide() every tick.
    std::string over_lead_policy{"hold"};
    double buzz_stop_nm{0.5};        // Nm rms above 20 Hz on any joint ends tracking (BuzzMeter)
    // The workspace box, absolute in the base frame (cell_panel's drawer,
    // written before every START); z_floor_m is its bottom. Wide open until set.
    double box_x_min{-1e9}, box_x_max{1e9};
    double box_y_min{-1e9}, box_y_max{1e9};
    double box_z_max{1e9};
    // FR3 joint limits (franka_description joint_limits.yaml) and the margin
    // inside them in which a pull toward the stop holds.
    std::array<double, 7> joint_lower{-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};
    std::array<double, 7> joint_upper{2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};
    double joint_margin_rad{0.14};
};

// What to do while the equilibrium would lead the arm by more than
// max_lead_m or max_lead_rad (tracking_over_lead_policy): hold where the arm
// is and resume once it is back inside, end tracking, or keep following with
// the lead cut back to the caps.
enum class OverLead { kHold, kStop, kClamp };

inline std::optional<OverLead> parse_over_lead(const std::string &name)
{
    if (name == "hold") {
        return OverLead::kHold;
    }
    if (name == "stop") {
        return OverLead::kStop;
    }
    if (name == "clamp") {
        return OverLead::kClamp;
    }
    return std::nullopt;
}

inline const char *over_lead_name(OverLead policy)
{
    switch (policy) {
    case OverLead::kStop:
        return "stop";
    case OverLead::kClamp:
        return "clamp";
    case OverLead::kHold:
        break;
    }
    return "hold";
}

// The integrator state: how far past the goal the equilibrium is pushed.
struct Lead
{
    tf2::Vector3 pos{0.0, 0.0, 0.0};
    tf2::Vector3 rot{0.0, 0.0, 0.0};   // rotation vector: axis * angle, base frame
};

inline bool finite_vec(const tf2::Vector3 &v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline bool finite_pose(const tf2::Transform &t)
{
    const tf2::Quaternion q = t.getRotation();
    return finite_vec(t.getOrigin()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
           std::isfinite(q.z()) && std::isfinite(q.w());
}

// Whether a raw detection this old lets the tick move the arm (TRACKING_SPEC
// Decision 5). The age is NaN before the first detection: not fresh.
inline bool raw_fresh(double age_s, double timeout_s)
{
    return age_s <= timeout_s;   // false for NaN
}

// Energy above ~20 Hz in the measured joint torques: per joint, a 2nd-order
// Butterworth high-pass and a 0.1 s running rms. Tracking moves at a few Hz,
// so its torques barely reach this band; the 2026-09-23 40 Hz wrist buzz
// (rotational damping 2*zeta*sqrt(k_rot) against the wrist's small inertia)
// read 4-5 Nm rms on J1/J4 over a 0.02-0.07 Nm floor. Primed by the first
// sample, so a static gravity load is not a step.
class BuzzMeter
{
public:
    static constexpr int kJoints = 7;

    explicit BuzzMeter(double rate_hz = 1000.0, double cutoff_hz = 20.0,
                       double window_s = 0.1)
    {
        const double k = std::tan(M_PI * cutoff_hz / rate_hz);
        const double norm = 1.0 / (1.0 + std::sqrt(2.0) * k + k * k);
        b0_ = norm;
        b1_ = -2.0 * norm;
        a1_ = 2.0 * (k * k - 1.0) * norm;
        a2_ = (1.0 - std::sqrt(2.0) * k + k * k) * norm;
        alpha_ = 1.0 / (window_s * rate_hz);
    }

    void reset() { primed_ = false; }

    // One sample of the measured torques; the loudest joint's rms after it.
    // A non-finite sample is skipped rather than poisoning the filters.
    double update(const std::array<double, kJoints> &tau, int *loudest = nullptr)
    {
        for (double x : tau) {
            if (!std::isfinite(x)) {
                return level(loudest);
            }
        }
        if (!primed_) {
            for (int j = 0; j < kJoints; ++j) {
                x1_[j] = x2_[j] = tau[j];
                y1_[j] = y2_[j] = ms_[j] = 0.0;
            }
            primed_ = true;
        }
        for (int j = 0; j < kJoints; ++j) {
            const double y = b0_ * tau[j] + b1_ * x1_[j] + b0_ * x2_[j] - a1_ * y1_[j] -
                             a2_ * y2_[j];
            x2_[j] = x1_[j];
            x1_[j] = tau[j];
            y2_[j] = y1_[j];
            y1_[j] = y;
            ms_[j] += alpha_ * (y * y - ms_[j]);
        }
        return level(loudest);
    }

    double level(int *loudest = nullptr) const
    {
        int worst = 0;
        for (int j = 1; j < kJoints; ++j) {
            if (ms_[j] > ms_[worst]) {
                worst = j;
            }
        }
        if (loudest) {
            *loudest = worst;
        }
        return std::sqrt(ms_[worst]);
    }

private:
    double b0_, b1_, a1_, a2_, alpha_;
    bool primed_{false};
    std::array<double, kJoints> x1_{}, x2_{}, y1_{}, y2_{}, ms_{};
};

// Rescale to max_norm if longer, preserving direction. This is the whole of
// the anti-windup: the lead can never ask for more than k * max_norm.
inline tf2::Vector3 clamp_norm(const tf2::Vector3 &v, double max_norm)
{
    const double n = v.length();
    return (n > max_norm) ? v * (max_norm / n) : v;
}

inline std::string mm(double metres)
{
    return std::to_string(static_cast<int>(std::lround(metres * 1000.0)));
}

inline std::string deg(double radians)
{
    return std::to_string(static_cast<int>(std::lround(radians * 180.0 / M_PI)));
}

// Rotation as axis * shortest angle, so the lead is a vector and can be
// integrated and clamped like one.
inline tf2::Vector3 rotation_vector(const tf2::Quaternion &q)
{
    double angle = q.getAngle();  // in [0, 2*pi]
    tf2::Vector3 axis = q.getAxis();
    if (angle > M_PI) {  // take the short way
        angle = 2.0 * M_PI - angle;
        axis = -axis;
    }
    if (angle < 1e-12) {  // the guard the controller's slew_equilibrium uses
        return tf2::Vector3(0.0, 0.0, 0.0);
    }
    return axis * angle;
}

// One integrator step. Position and rotation freeze and clamp independently:
// their bounds have different units and different ceilings. Freezing inside
// half the deadband is what stops the lead hunting across the stiction band
// (TRACKING_SPEC.md Decision 4); a bad dt or a bad error holds the lead where
// it is rather than corrupting it.
inline Lead advance_lead(const Lead &lead, const tf2::Vector3 &pos_err,
                         const tf2::Vector3 &rot_err, const Config &cfg, double dt)
{
    if (!std::isfinite(dt) || dt <= 0.0 || !finite_vec(pos_err) || !finite_vec(rot_err)) {
        return lead;
    }
    Lead next = lead;
    if (pos_err.length() >= cfg.deadband_m * 0.5) {
        next.pos = clamp_norm(lead.pos + pos_err * (cfg.ki * dt), cfg.lead_max_m);
    }
    if (rot_err.length() >= cfg.deadband_rad * 0.5) {
        next.rot = clamp_norm(lead.rot + rot_err * (cfg.ki * dt), cfg.lead_max_rad);
    }
    return next;
}

// TRACK holds the CAMERA where ALIGN leaves it: optical axis anti-parallel
// to the marker normal, the marker standoff_m straight ahead, marker X at
// inplane_rad in the image (roscam.plane_normal.inplane_angle: 0 = right,
// +pi/2 = down). The camera goal in the marker frame is the flip about X
// followed by -inplane about the new Z, which puts marker X at +inplane in
// the image. Returned as the TCP pose in t_base_marker's frame; t_tcp_cam is
// the hand-eye transform (TF EEF_FRAME_ID -> camera optical frame).
inline tf2::Transform camera_centred_goal(const tf2::Transform &t_base_marker,
                                          double standoff_m, double inplane_rad,
                                          const tf2::Transform &t_tcp_cam)
{
    tf2::Quaternion q_flip, q_inplane;
    q_flip.setRPY(M_PI, 0.0, 0.0);
    q_inplane.setRPY(0.0, 0.0, -inplane_rad);
    const tf2::Transform t_marker_cam(q_flip * q_inplane, tf2::Vector3(0.0, 0.0, standoff_m));
    return t_base_marker * t_marker_cam * t_tcp_cam.inverse();
}

// Angle of the marker X axis in the camera image, radians, in (-pi, pi]: the
// same quantity as roscam.plane_normal.inplane_angle, from the marker's
// orientation in the camera optical frame.
inline double inplane_rad(const tf2::Quaternion &q_cam_marker)
{
    const tf2::Vector3 x = tf2::quatRotate(q_cam_marker, tf2::Vector3(1.0, 0.0, 0.0));
    return std::atan2(x.y(), x.x());
}

// The TCP goal is at EEF_FRAME_ID (fr3_hand_tcp), but the controller holds
// its equilibrium at franka::Frame::kEndEffector. t_tcp_ee is that fixed tool
// offset, measured at startup and never assumed: identity only means the two
// frames coincide.
inline tf2::Transform goal_in_ee(const tf2::Transform &goal_tcp,
                                 const tf2::Transform &t_tcp_ee)
{
    return goal_tcp * t_tcp_ee;
}

// The rotation a rotation vector (axis * angle) stands for.
inline tf2::Quaternion quaternion_from(const tf2::Vector3 &rot)
{
    tf2::Quaternion q = tf2::Quaternion::getIdentity();
    const double angle = rot.length();
    if (angle > 1e-12) {
        q.setRotation(rot / angle, angle);
    }
    return q;
}

// The published equilibrium: the vision goal pushed out by the lead. It takes
// no measured pose, so the orientation target can only come from vision
// (TRACKING_SPEC.md Decision 3). The lead composes on the LEFT, the same
// convention as the controller's err_q = equilibrium * orientation.inverse().
inline tf2::Transform equilibrium_from(const tf2::Transform &goal_ee, const Lead &lead)
{
    return tf2::Transform(quaternion_from(lead.rot) * goal_ee.getRotation(),
                          goal_ee.getOrigin() + lead.pos);
}

// The goal changes once per camera frame (~15 Hz) but is used every 20 ms
// tick. Published as steps, a fast slew covered each step and then stopped
// until the next frame: a 15 Hz stop-start that read 0.3-1.0 Nm on the buzz
// meter with no buzz (2026-09-24). GoalGlide spreads each step over the time
// between the frames that bound it, so the goal moves at the marker's own
// speed. It only ever moves between goals already seen, never past the newest
// (TRACKING_SPEC Decision 5), and costs up to one frame of delay.
class GoalGlide
{
public:
    // START: no frame seen yet.
    void clear()
    {
        primed_ = false;
        stamp_ = NAN;
    }

    // After a hold: the next glide starts from where the arm is.
    void release() { primed_ = false; }

    // The goal for a tick at now_s, given the newest detection's goal and its
    // image stamp. A new stamp starts a glide from wherever the last one had
    // got to (from the arm when released), lasting the frame interval clamped
    // to [min_s, max_s]; with no previous frame it lasts max_s.
    tf2::Transform step(const tf2::Transform &goal, double stamp_s, double now_s,
                        const tf2::Transform &arm, double min_s, double max_s)
    {
        if (!primed_ || stamp_s != stamp_) {
            double dur = stamp_s - stamp_;
            if (!std::isfinite(dur) || dur > max_s) {
                dur = max_s;
            }
            from_ = primed_ ? at(now_s) : arm;
            t0_ = now_s;
            dur_ = std::max(dur, min_s);
            stamp_ = stamp_s;
            primed_ = true;
        }
        to_ = goal;
        return at(now_s);
    }

private:
    tf2::Transform at(double now_s) const
    {
        const double s = dur_ > 0.0 ? std::clamp((now_s - t0_) / dur_, 0.0, 1.0) : 1.0;
        const tf2::Vector3 turn =
            rotation_vector(to_.getRotation() * from_.getRotation().inverse());
        return tf2::Transform(quaternion_from(turn * s) * from_.getRotation(),
                              from_.getOrigin() + (to_.getOrigin() - from_.getOrigin()) * s);
    }

    tf2::Transform from_, to_;
    double stamp_{NAN};
    double t0_{0.0};
    double dur_{0.0};
    bool primed_{false};
};

// Empty if the equilibrium sits within max_lead_m and max_lead_rad of the
// arm, otherwise how far out it is. The position cap is cell_panel's
// MAX_LEAD_MM; a 50 Hz stream must not be looser than the stepped path.
inline std::string lead_excess(const tf2::Transform &eq, const tf2::Transform &measured_ee,
                               const Config &cfg)
{
    const double lead = (eq.getOrigin() - measured_ee.getOrigin()).length();
    if (lead > cfg.max_lead_m) {
        return "the equilibrium would sit " + mm(lead) + " mm from the arm (cap " +
               mm(cfg.max_lead_m) + " mm)";
    }
    const double turn =
        rotation_vector(eq.getRotation() * measured_ee.getRotation().inverse()).length();
    if (turn > cfg.max_lead_rad) {
        return "the equilibrium would sit " + deg(turn) + " deg from the arm (cap " +
               deg(cfg.max_lead_rad) + " deg)";
    }
    return {};
}

// The clamp policy: the equilibrium cut back to the caps around the arm -
// position along the straight line to it, rotation along the shortest path
// from the measured orientation.
inline tf2::Transform clamp_lead(const tf2::Transform &eq, const tf2::Transform &measured_ee,
                                 const Config &cfg)
{
    const tf2::Vector3 turn = clamp_norm(
        rotation_vector(eq.getRotation() * measured_ee.getRotation().inverse()),
        cfg.max_lead_rad);
    return tf2::Transform(
        quaternion_from(turn) * measured_ee.getRotation(),
        measured_ee.getOrigin() +
            clamp_norm(eq.getOrigin() - measured_ee.getOrigin(), cfg.max_lead_m));
}

// Empty if the equilibrium may be published, otherwise why it may not. These
// hold whatever the over-lead policy says: a non-finite pose, and the cell's
// absolute Z floor in the base frame (cell_panel's FLOOR_Z_MM).
inline std::string publish_veto(const tf2::Transform &eq, const Config &cfg)
{
    if (!finite_pose(eq)) {
        return "the equilibrium pose is not finite";
    }
    if (eq.getOrigin().z() < cfg.z_floor_m) {
        return "the equilibrium would sit at z " + mm(eq.getOrigin().z()) +
               " mm, below the floor " + mm(cfg.z_floor_m) +
               " mm - the camera bracket hangs below the flange";
    }
    const tf2::Vector3 &p = eq.getOrigin();
    auto outside = [](const char *axis, double v, double lo, double hi) {
        return "the equilibrium would leave the workspace box: " + std::string(axis) + " " +
               mm(v) + " mm outside [" + mm(lo) + ", " + mm(hi) +
               "] - move the marker back inside";
    };
    if (p.x() < cfg.box_x_min || p.x() > cfg.box_x_max) {
        return outside("x", p.x(), cfg.box_x_min, cfg.box_x_max);
    }
    if (p.y() < cfg.box_y_min || p.y() > cfg.box_y_max) {
        return outside("y", p.y(), cfg.box_y_min, cfg.box_y_max);
    }
    if (p.z() > cfg.box_z_max) {
        return "the equilibrium would sit at z " + mm(p.z()) + " mm, above the box top " +
               mm(cfg.box_z_max) + " mm - move the marker back inside";
    }
    return {};
}

// FR3 kinematics (Craig's modified DH, Franka's published parameters): the
// origin and rotation axis of each joint in the base frame at q.
struct JointFrames
{
    std::array<tf2::Vector3, 7> origin;
    std::array<tf2::Vector3, 7> axis;
    tf2::Transform flange;
};

inline JointFrames joint_frames(const std::array<double, 7> &q)
{
    static constexpr std::array<double, 7> a{0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088};
    static constexpr std::array<double, 7> d{0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0};
    static constexpr std::array<double, 7> alpha{0.0,     -M_PI_2, M_PI_2, M_PI_2,
                                                 -M_PI_2, M_PI_2,  M_PI_2};
    JointFrames f;
    tf2::Transform t = tf2::Transform::getIdentity();
    for (size_t i = 0; i < 7; ++i) {
        const double ca = std::cos(alpha[i]), sa = std::sin(alpha[i]);
        const double ct = std::cos(q[i]), st = std::sin(q[i]);
        t = t * tf2::Transform(tf2::Matrix3x3(ct, -st, 0.0, st * ca, ct * ca, -sa, st * sa,
                                              ct * sa, ca),
                               tf2::Vector3(a[i], -d[i] * sa, d[i] * ca));
        f.origin[i] = t.getOrigin();
        f.axis[i] = t.getBasis().getColumn(2);
    }
    f.flange = t * tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0.0, 0.0, 0.107));
    return f;
}

// Empty unless a joint within joint_margin_rad of a limit would go further in.
// It would when the move to eq pulls it there - the pull is J^T [k_pos dp;
// k_rot dr], the joint torque a spring from the arm to eq applies - or when
// it is already moving there faster than 0.05 rad/s. Holding only then lets
// the operator back out by moving the marker; holding on proximity alone would
// never let go, because a held arm stays where it is. 2026-09-24: TRACK drove
// J2 past its stop twice, and the FR3's position-dependent velocity limit
// (zero at the stop) ended both runs with a joint_velocity_violation.
inline std::string joint_limit_veto(const std::array<double, 7> &q,
                                    const std::array<double, 7> &dq, const tf2::Transform &eq,
                                    const tf2::Transform &measured_ee, const Config &cfg,
                                    double k_pos, double k_rot)
{
    const JointFrames f = joint_frames(q);
    const tf2::Vector3 p = measured_ee.getOrigin();
    const tf2::Vector3 force = (eq.getOrigin() - p) * k_pos;
    const tf2::Vector3 moment =
        rotation_vector(eq.getRotation() * measured_ee.getRotation().inverse()) * k_rot;
    constexpr double kPullEps = 0.05;   // Nm: below this the goal is not pulling
    constexpr double kDriftEps = 0.05;  // rad/s: the drift speeds seen were 0.06-0.16
    for (size_t j = 0; j < 7; ++j) {
        const double pull = f.axis[j].dot((p - f.origin[j]).cross(force) + moment);
        const double to_lower = q[j] - cfg.joint_lower[j];
        const double to_upper = cfg.joint_upper[j] - q[j];
        const bool deeper_low = pull < -kPullEps || dq[j] < -kDriftEps;
        const bool deeper_high = pull > kPullEps || dq[j] > kDriftEps;
        if ((to_lower < cfg.joint_margin_rad && deeper_low) ||
            (to_upper < cfg.joint_margin_rad && deeper_high)) {
            const bool low = to_lower < to_upper;
            return "J" + std::to_string(j + 1) + " is " + deg(low ? to_lower : to_upper) +
                   " deg from its " + (low ? "lower" : "upper") +
                   " end stop - move the marker back toward the centre";
        }
    }
    return {};
}

// What one tick may do with an equilibrium. kHold publishes the measured
// pose (once); kStop ends tracking exactly like ~/stop_tracking.
struct Verdict
{
    enum class Act { kPublish, kHold, kStop };
    Act act{Act::kPublish};
    tf2::Transform eq;    // the pose to publish when act is kPublish
    std::string reason;   // why it holds or stops, or what the clamp cut; empty otherwise
};

// A non-finite pose holds, whatever the policy - NaN is not "far". So does
// one below the floor under hold and stop, before any cap is judged: the
// floor is what holds, 'stop' included, and the reason names it. Past a cap
// the policy decides; under clamp the floor then vetoes the clamped pose,
// which is what would actually be published.
inline Verdict decide(const tf2::Transform &eq, const tf2::Transform &measured_ee,
                      const Config &cfg, OverLead policy)
{
    if (!finite_pose(eq)) {
        return {Verdict::Act::kHold, eq, publish_veto(eq, cfg)};
    }
    if (policy != OverLead::kClamp) {
        const std::string floor = publish_veto(eq, cfg);
        if (!floor.empty()) {
            return {Verdict::Act::kHold, eq, floor};
        }
    }
    Verdict v{Verdict::Act::kPublish, eq, {}};
    std::string why = lead_excess(eq, measured_ee, cfg);
    if (!why.empty()) {
        switch (policy) {
        case OverLead::kHold:
            return {Verdict::Act::kHold, eq, why + " - wait for the arm to catch up"};
        case OverLead::kStop:
            return {Verdict::Act::kStop, eq, why + " - over-lead policy 'stop'"};
        case OverLead::kClamp:
            v.eq = clamp_lead(eq, measured_ee, cfg);
            v.reason = "clamped: " + why;
            break;
        }
    }
    why = publish_veto(v.eq, cfg);
    if (!why.empty()) {
        return {Verdict::Act::kHold, v.eq, why};
    }
    return v;
}

// Empty if the tracking configuration is acceptable, otherwise why it is not.
// The stiffnesses and the controller's two ceilings are arguments because they
// live in yaml: this is what makes a lead that could exceed 15 N a startup
// refusal instead of a surprise on the day.
inline std::string validate_config(const Config &cfg, double k_pos_max_n_per_m,
                                   double k_rot_max_nm_per_rad,
                                   double max_force_n, double max_torque_nm)
{
    if (!std::isfinite(cfg.ki) || cfg.ki <= 0.0) {
        return "tracking_ki must be finite and positive";
    }
    if (!std::isfinite(cfg.lead_max_m) || cfg.lead_max_m <= 0.0) {
        return "tracking_lead_max_m must be finite and positive";
    }
    if (!std::isfinite(cfg.lead_max_rad) || cfg.lead_max_rad <= 0.0) {
        return "tracking_lead_max_rad must be finite and positive";
    }
    if (!std::isfinite(cfg.max_lead_m) || cfg.max_lead_m <= 0.0) {
        return "tracking_max_lead_m must be finite and positive";
    }
    if (cfg.max_lead_m > 0.060) {
        return "tracking_max_lead_m must not exceed the panel's 60 mm equilibrium-lead cap";
    }
    if (!std::isfinite(cfg.max_lead_rad) || cfg.max_lead_rad <= 0.0) {
        return "tracking_max_lead_rad must be finite and positive";
    }
    if (cfg.max_lead_rad > 0.35) {
        return "tracking_max_lead_rad must not exceed 0.35 rad (20 deg)";
    }
    if (!std::isfinite(cfg.buzz_stop_nm) || cfg.buzz_stop_nm <= 0.0) {
        return "tracking_buzz_stop_nm must be finite and positive";
    }
    if (!parse_over_lead(cfg.over_lead_policy)) {
        return "tracking_over_lead_policy must be hold, stop or clamp, not '" +
               cfg.over_lead_policy + "'";
    }
    if (!std::isfinite(cfg.z_floor_m)) {
        return "the tracking Z floor must be finite";
    }
    const double band_m = kFrictionBreakawayN / k_pos_max_n_per_m;
    if (cfg.deadband_m < band_m || cfg.deadband_m > 3.0 * band_m) {
        return "tracking_deadband_m must be within [" + mm(band_m) + ", " + mm(3.0 * band_m) +
               "] mm at this k_pos_tool - the deadband is F_friction / the stiffness in use";
    }
    const double band_rad = kFrictionBreakawayNm / k_rot_max_nm_per_rad;
    if (cfg.deadband_rad < band_rad || cfg.deadband_rad > 3.0 * band_rad) {
        return "tracking_deadband_rad must be within [" + mm(band_rad) + ", " +
               mm(3.0 * band_rad) + "] mrad at this k_rot_tool";
    }
    if (k_pos_max_n_per_m * cfg.lead_max_m > kLeadForceMaxN) {
        return "k_pos_tool * tracking_lead_max_m must not exceed 15 N";
    }
    if (kLeadForceMaxN >= max_force_n) {
        return "the 15 N lead bound must stay below tracking_max_force_n";
    }
    if (k_rot_max_nm_per_rad * cfg.lead_max_rad >= max_torque_nm) {
        return "k_rot_tool * tracking_lead_max_rad must stay below tracking_max_torque_nm";
    }
    return {};
}

}  // namespace tracking_law
