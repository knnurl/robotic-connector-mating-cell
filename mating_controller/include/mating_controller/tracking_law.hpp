// Pure control law for continuous marker tracking on the Cartesian-impedance
// backend (TRACKING_SPEC.md section 5): the bounded integrator that closes the
// friction residual the spring cannot, the TCP-to-EE frame conversion, and the
// veto that guards what may be published. No ROS node code here so the law is
// unit-testable (see test/test_tracking_law.cpp).
#pragma once

#include <cmath>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace tracking_law
{

// Worst extra force the bounded integrator may add is k * lead_max.
// TRACKING_SPEC.md section 6. tools/fr3/test_impedance_panel.py scrapes
// these three constants by name, so keep them bare decimal literals.
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
    double max_lead_m{0.060};        // m, mirrors impedance_panel MAX_LEAD_MM
    double z_floor_m{0.0};           // m, seeded on ~/start_tracking
};

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

// standoff_goal targets EEF_FRAME_ID (fr3_hand_tcp), but the controller holds
// its equilibrium at franka::Frame::kEndEffector. t_tcp_ee is that fixed tool
// offset, measured at startup and never assumed: identity only means the two
// frames coincide.
inline tf2::Transform goal_in_ee(const tf2::Transform &goal_tcp,
                                 const tf2::Transform &t_tcp_ee)
{
    return goal_tcp * t_tcp_ee;
}

// The published equilibrium: the vision goal pushed out by the lead. It takes
// no measured pose, so the orientation target can only come from vision
// (TRACKING_SPEC.md Decision 3). The lead composes on the LEFT, the same
// convention as the controller's err_q = equilibrium * orientation.inverse().
inline tf2::Transform equilibrium_from(const tf2::Transform &goal_ee, const Lead &lead)
{
    tf2::Quaternion q_lead = tf2::Quaternion::getIdentity();
    const double angle = lead.rot.length();
    if (angle > 1e-12) {
        q_lead.setRotation(lead.rot / angle, angle);
    }
    return tf2::Transform(q_lead * goal_ee.getRotation(), goal_ee.getOrigin() + lead.pos);
}

// Empty if the equilibrium may be published, otherwise why it may not. These
// are the two bounds impedance_panel already enforces on this topic
// (MAX_LEAD_MM and FLOOR_BELOW_HOLD_MM); a 50 Hz stream must not bypass them.
inline std::string publish_veto(const tf2::Transform &eq,
                                const tf2::Transform &measured_ee, const Config &cfg)
{
    const tf2::Quaternion q = eq.getRotation();
    if (!finite_vec(eq.getOrigin()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) ||
        !std::isfinite(q.z()) || !std::isfinite(q.w())) {
        return "the equilibrium pose is not finite";
    }
    const double lead = (eq.getOrigin() - measured_ee.getOrigin()).length();
    if (lead > cfg.max_lead_m) {
        return "the equilibrium would sit " + mm(lead) + " mm from the arm (cap " +
               mm(cfg.max_lead_m) + " mm) - wait for the arm to catch up";
    }
    if (eq.getOrigin().z() < cfg.z_floor_m) {
        return "the equilibrium would sit at z " + mm(eq.getOrigin().z()) +
               " mm, below the floor " + mm(cfg.z_floor_m) +
               " mm - the camera bracket hangs below the flange";
    }
    return {};
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
