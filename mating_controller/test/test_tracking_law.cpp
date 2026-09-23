// Unit tests for the continuous-tracking control law: the camera-centred
// goal, the bounded integrator, the TCP-to-EE frame conversion, the
// over-lead policy and the publish veto - and the whole chain closed around
// a simulated arm.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "mating_controller/mating_geometry.hpp"
#include "mating_controller/tracking_law.hpp"

using mating_geometry::pose_error;
using tracking_law::advance_lead;
using tracking_law::camera_centred_goal;
using tracking_law::decide;
using tracking_law::equilibrium_from;
using tracking_law::goal_in_ee;
using tracking_law::inplane_rad;
using tracking_law::over_lead_name;
using tracking_law::OverLead;
using tracking_law::parse_over_lead;
using tracking_law::publish_veto;
using tracking_law::quaternion_from;
using tracking_law::raw_fresh;
using tracking_law::rotation_vector;
using tracking_law::validate_config;
using tracking_law::Verdict;

namespace
{

const double kNan = std::numeric_limits<double>::quiet_NaN();

tf2::Transform make_pose(double x, double y, double z,
                         double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::Transform(q, tf2::Vector3(x, y, z));
}

// The shipped track profile: fr3_params.yaml track_* and tracking_*.
tracking_law::Config shipped()
{
    tracking_law::Config cfg;
    cfg.ki = 0.5;
    cfg.lead_max_m = 0.010;
    cfg.lead_max_rad = 0.017;
    cfg.deadband_m = 0.005;
    cfg.deadband_rad = 0.007;
    cfg.max_lead_m = 0.060;
    cfg.max_lead_rad = 0.26;
    cfg.z_floor_m = 0.0;
    cfg.over_lead_policy = "hold";
    return cfg;
}

// validate_config against the shipped track stiffnesses and the controller's
// two ceilings (cartesian_impedance_stroke.yaml).
std::string check(const tracking_law::Config &cfg)
{
    return validate_config(cfg, 1500.0, 90.0, 30.0, 10.0);
}

// The hand-eye transform measured 2026-09-15 (tools/fr3/calib/handeye.yaml
// xyz / quat_xyzw): TCP -> camera optical frame, turned ~90 deg
// about the optical axis and 62 mm off it. Anything but identity, so a
// composition in the wrong order cannot pass by accident.
tf2::Transform handeye()
{
    return tf2::Transform(tf2::Quaternion(0.000855, 0.003126, 0.706706, 0.707500).normalized(),
                          tf2::Vector3(0.061126, -0.011144, -0.046550));
}

// A non-identity TCP -> EE tool offset, as in GoalInEe below.
tf2::Transform tool_offset()
{
    return make_pose(0.0, 0.0, 0.1034, 0.0, 0.0, -M_PI / 4.0);
}

// A marker on a surface tilted ~15 deg, in the base frame.
tf2::Transform tilted_marker()
{
    return make_pose(0.50, 0.05, 0.02, 0.20, -0.15, 0.70);
}

// The EE pose ALIGN leaves the arm in, built from ALIGN's own definition and
// NOT from camera_centred_goal: translate() nulls marker_pos_in_cam -
// [0, 0, target], level() turns the marker normal to face the camera, and
// inplane() puts marker X at the target angle in the image (0 = right,
// +90 = down).
tf2::Transform align_end_ee(const tf2::Transform &t_base_marker, double standoff_m,
                            double inplane)
{
    const tf2::Vector3 x(std::cos(inplane), std::sin(inplane), 0.0);  // marker X, camera frame
    const tf2::Vector3 z(0.0, 0.0, -1.0);                              // marker normal
    const tf2::Vector3 y = z.cross(x);
    const tf2::Matrix3x3 r_cam_marker(x.x(), y.x(), z.x(),
                                      x.y(), y.y(), z.y(),
                                      x.z(), y.z(), z.z());
    const tf2::Transform t_cam_marker(r_cam_marker, tf2::Vector3(0.0, 0.0, standoff_m));
    return t_base_marker * t_cam_marker.inverse() * handeye().inverse() * tool_offset();
}

double angle_between(const tf2::Quaternion &a, const tf2::Quaternion &b)
{
    return rotation_vector(a * b.inverse()).length();
}

// A first-order, slew-limited arm on the impedance spring, with the track
// profile's numbers. It moves toward the published equilibrium but stalls
// F_breakaway / k short of it - the residual the lead exists to take out -
// so a loop that leaned on the spring alone would park 4.3 mm and 6.7 mrad
// off.
struct Arm
{
    tf2::Transform ee;
    double band_m{tracking_law::kFrictionBreakawayN / 1500.0};
    double band_rad{tracking_law::kFrictionBreakawayNm / 90.0};
    double tau_s{0.05};
    double slew_mps{0.10};
    double slew_rps{0.5};

    void step(const tf2::Transform &eq, double dt)
    {
        const tf2::Vector3 d = eq.getOrigin() - ee.getOrigin();
        const double n = d.length();
        if (n > band_m) {
            ee.setOrigin(ee.getOrigin() +
                         d / n * std::min((n - band_m) * dt / tau_s, slew_mps * dt));
        }
        const tf2::Vector3 r = rotation_vector(eq.getRotation() * ee.getRotation().inverse());
        const double a = r.length();
        if (a > band_rad) {
            const tf2::Vector3 turn = r / a * std::min((a - band_rad) * dt / tau_s, slew_rps * dt);
            ee.setRotation((quaternion_from(turn) * ee.getRotation()).normalized());
        }
    }
};

struct Loop
{
    OverLead policy{OverLead::kHold};
    double lead_sign{1.0};      // -1 integrates the error the wrong way round
    double inplane_sign{1.0};   // -1 aims the camera at the mirrored in-plane angle
    double drift_mps{0.0};      // marker translation along base X
    double spin_rps{0.0};       // marker turn about its own normal
    double seconds{20.0};
};

struct Outcome
{
    double pos_err_m{0.0};      // at the end, from where ALIGN would leave the arm
    double rot_err_rad{0.0};
    double worst_pos_m{0.0};    // over the second half of the run
    double worst_rot_rad{0.0};
    int holds{0};
    bool stopped{false};
};

// tick() closed around the Arm, in the spirit of the archived servo sign tests
// (test_servo_signs.py, tag pre-cleanup-2026-09-23):
// vision -> camera-centred goal -> lead -> equilibrium -> over-lead policy ->
// publish (or hold the measured pose once) -> the arm moves. It is judged
// against align_end_ee, never against the goal it computed, so a goal in
// the wrong place cannot grade itself.
Outcome run_loop(const Loop &loop)
{
    const double dt = 0.02;
    const double standoff = 0.10;
    const double ip = M_PI / 2.0;   // cell_panel's default in-plane target
    const tf2::Transform marker0 = tilted_marker();
    auto cfg = shipped();

    // Start 30 mm and ~5 deg off the ALIGN pose: inside both caps.
    Arm arm;
    const tf2::Transform aligned = align_end_ee(marker0, standoff, ip);
    tf2::Quaternion off;
    off.setRPY(0.05, -0.04, 0.06);
    arm.ee = tf2::Transform(off * aligned.getRotation(),
                            aligned.getOrigin() + tf2::Vector3(0.020, -0.015, 0.015));
    cfg.z_floor_m = arm.ee.getOrigin().z() - 0.030;   // close under the arm, so the loop runs near it

    tracking_law::Lead lead;
    tf2::Transform published = arm.ee;
    bool held = false;
    Outcome run;
    const int steps = static_cast<int>(loop.seconds / dt);
    for (int i = 0; i < steps && !run.stopped; ++i) {
        const double t = i * dt;
        tf2::Quaternion spin;
        spin.setRPY(0.0, 0.0, loop.spin_rps * t);
        const tf2::Transform marker(marker0.getRotation() * spin,
                                    marker0.getOrigin() + tf2::Vector3(loop.drift_mps * t, 0, 0));

        const auto goal_ee = goal_in_ee(
            camera_centred_goal(marker, standoff, loop.inplane_sign * ip, handeye()),
            tool_offset());
        tf2::Vector3 pos_err;
        tf2::Quaternion rot_err;
        double angle;
        pose_error(arm.ee, goal_ee, pos_err, rot_err, angle);
        lead = advance_lead(lead, pos_err * loop.lead_sign,
                            rotation_vector(rot_err) * loop.lead_sign, cfg, dt);
        const Verdict v = decide(equilibrium_from(goal_ee, lead), arm.ee, cfg, loop.policy);
        if (v.act == Verdict::Act::kPublish) {
            published = v.eq;
            held = false;
        } else {
            ++run.holds;
            if (!held) {   // hold_once
                published = arm.ee;
                held = true;
            }
            run.stopped = v.act == Verdict::Act::kStop;
        }
        arm.step(published, dt);

        tf2::Vector3 e;
        tf2::Quaternion q;
        double a;
        pose_error(arm.ee, align_end_ee(marker, standoff, ip), e, q, a);
        run.pos_err_m = e.length();
        run.rot_err_rad = a;
        if (i >= steps / 2) {
            run.worst_pos_m = std::max(run.worst_pos_m, run.pos_err_m);
            run.worst_rot_rad = std::max(run.worst_rot_rad, run.rot_err_rad);
        }
    }
    return run;
}

}  // namespace

TEST(ValidateConfig, AcceptsTheShippedTrackConfig)
{
    // 1500 N/m * 10 mm = exactly the 15 N bound: the shipped pair is the
    // tightest one that may ship, and it must not be rejected.
    EXPECT_EQ(check(shipped()), "");
}

TEST(ValidateConfig, RejectsALeadThatCouldExceedFifteenNewtons)
{
    tracking_law::Config cfg = shipped();
    cfg.lead_max_m = 0.011;  // 16.5 N at k_pos 1500
    EXPECT_NE(check(cfg), "");
}

TEST(ValidateConfig, RejectsALeadThatCouldExceedTheTorqueCeiling)
{
    tracking_law::Config cfg = shipped();
    cfg.lead_max_rad = 0.12;  // 10.8 Nm at k_rot 90, above max_torque_nm 10
    EXPECT_NE(check(cfg), "");
}

TEST(ValidateConfig, RejectsADeadbandThatDoesNotMatchTheStiffness)
{
    // 1.8 mm is the deadband MEASURED at k = 3000; the track profile runs at
    // 1500, where the band is 4.3 mm. Shipping the 3000 number would put the
    // freeze band inside the stiction band and the lead would wind while the
    // arm is stuck.
    tracking_law::Config cfg = shipped();
    cfg.deadband_m = 0.002;
    EXPECT_NE(check(cfg), "");

    cfg = shipped();
    cfg.deadband_rad = 0.05;  // far above 3 * 0.6 Nm / 90
    EXPECT_NE(check(cfg), "");
}

TEST(ValidateConfig, RejectsANonPositiveOrNonFiniteKi)
{
    tracking_law::Config cfg = shipped();
    cfg.ki = 0.0;
    EXPECT_NE(check(cfg), "");
    cfg.ki = -0.5;
    EXPECT_NE(check(cfg), "");
    cfg.ki = kNan;
    EXPECT_NE(check(cfg), "");
}

TEST(ValidateConfig, RejectsAnEquilibriumLeadCapAboveThePanels)
{
    tracking_law::Config cfg = shipped();
    cfg.max_lead_m = 0.07;  // the panel refuses above 60 mm
    EXPECT_NE(check(cfg), "");
}

TEST(ValidateConfig, RejectsABadAngularLeadCap)
{
    tracking_law::Config cfg = shipped();
    for (double bad : {0.0, -0.1, kNan, std::numeric_limits<double>::infinity(), 0.36}) {
        cfg.max_lead_rad = bad;
        EXPECT_NE(check(cfg), "") << bad;
    }
    cfg.max_lead_rad = 0.35;   // the ceiling itself is allowed
    EXPECT_EQ(check(cfg), "");
}

TEST(ValidateConfig, RejectsAnUnknownOverLeadPolicy)
{
    tracking_law::Config cfg = shipped();
    for (const char *bad : {"", "Hold", "freeze", "hold "}) {
        cfg.over_lead_policy = bad;
        EXPECT_NE(check(cfg), "") << bad;
    }
    for (const char *good : {"hold", "stop", "clamp"}) {
        cfg.over_lead_policy = good;
        EXPECT_EQ(check(cfg), "") << good;
    }
}

TEST(ParseOverLead, KnowsExactlyThePanelsThreePolicies)
{
    EXPECT_EQ(parse_over_lead("hold"), OverLead::kHold);
    EXPECT_EQ(parse_over_lead("stop"), OverLead::kStop);
    EXPECT_EQ(parse_over_lead("clamp"), OverLead::kClamp);
    EXPECT_FALSE(parse_over_lead("STOP").has_value());
    EXPECT_FALSE(parse_over_lead("").has_value());
}

TEST(ParseOverLead, RoundTripsThroughOverLeadName)
{
    // ~/status reports the policy by this name and the panel adopts it into
    // its dropdown: a swapped name would show 'clamp' while the node stops.
    for (OverLead policy : {OverLead::kHold, OverLead::kStop, OverLead::kClamp}) {
        EXPECT_EQ(parse_over_lead(over_lead_name(policy)), policy);
    }
    EXPECT_STREQ(over_lead_name(OverLead::kHold), "hold");
    EXPECT_STREQ(over_lead_name(OverLead::kStop), "stop");
    EXPECT_STREQ(over_lead_name(OverLead::kClamp), "clamp");
}

TEST(RawFresh, OnlyARecentDetectionMayDriveMotion)
{
    // Before the first raw detection the age is NaN, and that is not fresh:
    // the filter's prediction alone must never move the arm.
    EXPECT_FALSE(raw_fresh(kNan, 0.25));
    EXPECT_TRUE(raw_fresh(0.0, 0.25));
    EXPECT_TRUE(raw_fresh(0.25, 0.25));   // the timeout itself is still fresh
    EXPECT_FALSE(raw_fresh(0.26, 0.25));
}

TEST(AdvanceLead, FreezesInsideHalfTheDeadband)
{
    // 2 mm error, 5 mm deadband: inside the stiction band, so integrating
    // would only hunt.
    const auto cfg = shipped();
    const tracking_law::Lead lead{tf2::Vector3(0.001, 0.0, 0.0), tf2::Vector3(0, 0, 0)};
    const auto next = advance_lead(lead, tf2::Vector3(0.002, 0.0, 0.0),
                                   tf2::Vector3(0, 0, 0), cfg, 0.02);
    EXPECT_NEAR(next.pos.x(), 0.001, 1e-12);
}

TEST(AdvanceLead, IntegratesOutsideTheDeadband)
{
    // 10 mm error, ki 0.5, dt 0.02 -> 0.1 mm of lead this cycle.
    const auto cfg = shipped();
    const auto next = advance_lead(tracking_law::Lead{}, tf2::Vector3(0.010, 0.0, 0.0),
                                   tf2::Vector3(0, 0, 0), cfg, 0.02);
    EXPECT_NEAR(next.pos.x(), 0.0001, 1e-12);
}

TEST(AdvanceLead, ClampsToLeadMax)
{
    // A marker that jumps 50 mm and stays there: the lead must stop at
    // lead_max (15 N), not wind on.
    const auto cfg = shipped();
    tracking_law::Lead lead;
    for (int i = 0; i < 1000; ++i) {
        lead = advance_lead(lead, tf2::Vector3(0.05, 0.0, 0.0),
                            tf2::Vector3(0.0, 0.0, 0.2), cfg, 0.02);
    }
    EXPECT_NEAR(lead.pos.length(), cfg.lead_max_m, 1e-12);
    EXPECT_NEAR(lead.rot.length(), cfg.lead_max_rad, 1e-12);
}

TEST(AdvanceLead, UnwindsWhenTheErrorReverses)
{
    const auto cfg = shipped();
    tracking_law::Lead lead;
    for (int i = 0; i < 100; ++i) {
        lead = advance_lead(lead, tf2::Vector3(0.05, 0.0, 0.0), tf2::Vector3(0, 0, 0),
                            cfg, 0.02);
    }
    ASSERT_GT(lead.pos.x(), 0.0);
    const auto unwound = advance_lead(lead, tf2::Vector3(-0.05, 0.0, 0.0),
                                      tf2::Vector3(0, 0, 0), cfg, 0.02);
    EXPECT_LT(unwound.pos.x(), lead.pos.x());
}

TEST(AdvanceLead, ZeroDtLeavesTheLeadAlone)
{
    // A stalled or non-finite clock must hold the lead, not corrupt it.
    const auto cfg = shipped();
    const tracking_law::Lead lead{tf2::Vector3(0.002, 0.0, 0.0), tf2::Vector3(0, 0, 0)};
    EXPECT_NEAR(advance_lead(lead, tf2::Vector3(0.05, 0, 0), tf2::Vector3(0, 0, 0),
                             cfg, 0.0).pos.x(), 0.002, 1e-12);
    EXPECT_NEAR(advance_lead(lead, tf2::Vector3(0.05, 0, 0), tf2::Vector3(0, 0, 0),
                             cfg, kNan).pos.x(), 0.002, 1e-12);
    EXPECT_NEAR(advance_lead(lead, tf2::Vector3(kNan, 0, 0), tf2::Vector3(0, 0, 0),
                             cfg, 0.02).pos.x(), 0.002, 1e-12);
}

TEST(AdvanceLead, PositionAndRotationFreezeIndependently)
{
    // Aligned in position but not in angle: the angular lead must keep
    // closing while the positional one holds.
    const auto cfg = shipped();
    const auto next = advance_lead(tracking_law::Lead{}, tf2::Vector3(0.002, 0.0, 0.0),
                                   tf2::Vector3(0.0, 0.0, 0.01), cfg, 0.02);
    EXPECT_NEAR(next.pos.length(), 0.0, 1e-12);
    EXPECT_NEAR(next.rot.z(), 0.0001, 1e-12);
}

TEST(CameraCentredGoal, WhereAlignLeftTheArmIsAlreadyOnTheGoal)
{
    // START right after ALIGN must not move the arm: zero goal error for
    // every in-plane target the panel offers, on a tilted marker. A sign
    // error in the in-plane angle shows at +/-90 deg; the hand-eye composed
    // the wrong way round shows everywhere.
    const auto marker = tilted_marker();
    for (double ip_deg : {0.0, 90.0, -90.0, 180.0}) {
        const double ip = ip_deg * M_PI / 180.0;
        const auto goal_ee =
            goal_in_ee(camera_centred_goal(marker, 0.10, ip, handeye()), tool_offset());
        tf2::Vector3 pos_err;
        tf2::Quaternion rot_err;
        double angle;
        pose_error(align_end_ee(marker, 0.10, ip), goal_ee, pos_err, rot_err, angle);
        EXPECT_NEAR(pos_err.length(), 0.0, 1e-9) << ip_deg;
        EXPECT_NEAR(angle, 0.0, 1e-6) << ip_deg;
    }
}

TEST(CameraCentredGoal, TheCameraSeesTheMarkerAsAlignLeftIt)
{
    // Read back through the camera, the numbers ALIGN converges on: the
    // marker standoff straight ahead and its X axis at the requested angle.
    const auto marker = tilted_marker();
    for (double ip_deg : {0.0, 90.0, -90.0, 180.0, 37.0}) {
        const double ip = ip_deg * M_PI / 180.0;
        const auto t_base_cam = camera_centred_goal(marker, 0.15, ip, handeye()) * handeye();
        const auto t_cam_marker = t_base_cam.inverse() * marker;
        EXPECT_NEAR((t_cam_marker.getOrigin() - tf2::Vector3(0.0, 0.0, 0.15)).length(), 0.0,
                    1e-9) << ip_deg;
        EXPECT_NEAR(std::remainder(inplane_rad(t_cam_marker.getRotation()) - ip, 2.0 * M_PI),
                    0.0, 1e-9) << ip_deg;
    }
}

TEST(InplaneRad, FollowsTheImageConvention)
{
    // roscam.plane_normal.inplane_angle: marker X to the right is 0, straight
    // down is +90 (optical +Y is down). A marker facing the camera, turned
    // by ip about the optical axis.
    tf2::Quaternion flip;
    flip.setRPY(M_PI, 0.0, 0.0);
    for (double ip_deg : {0.0, 90.0, -90.0, 37.0}) {
        const double ip = ip_deg * M_PI / 180.0;
        tf2::Quaternion turn;
        turn.setRPY(0.0, 0.0, ip);
        EXPECT_NEAR(inplane_rad(turn * flip), ip, 1e-9) << ip_deg;
    }
}

TEST(GoalInEe, IdentityOffsetLeavesTheGoalAlone)
{
    const auto goal = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    const auto out = goal_in_ee(goal, tf2::Transform::getIdentity());
    EXPECT_NEAR((out.getOrigin() - goal.getOrigin()).length(), 0.0, 1e-12);
    EXPECT_NEAR(out.getRotation().angleShortestPath(goal.getRotation()), 0.0, 1e-12);
}

TEST(GoalInEe, ATcpOffsetShiftsTheTargetButNotTheError)
{
    // The controller holds its equilibrium at kEndEffector while
    // camera_centred_goal produces a TCP goal. The conversion must ride ON the
    // goal's own rotation (goal * offset), which is what makes the error
    // between two EE poses equal the error between the two TCP poses.
    tf2::Quaternion q_off;
    q_off.setRPY(0.0, 0.0, -M_PI / 4.0);
    const tf2::Transform t_tcp_ee(q_off, tf2::Vector3(0.0, 0.0, 0.1034));

    const auto goal_tcp = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    const auto goal_ee = goal_in_ee(goal_tcp, t_tcp_ee);

    // Exactly the tool offset, expressed in the goal's own frame. With a
    // tool-down goal this differs from t_tcp_ee * goal_tcp, so the test fails
    // if the multiplication is reversed.
    const tf2::Vector3 expected =
        goal_tcp.getOrigin() + tf2::quatRotate(goal_tcp.getRotation(), t_tcp_ee.getOrigin());
    EXPECT_NEAR((goal_ee.getOrigin() - expected).length(), 0.0, 1e-12);
    EXPECT_NEAR(goal_ee.getRotation().angleShortestPath(
                    goal_tcp.getRotation() * t_tcp_ee.getRotation()), 0.0, 1e-12);

    // The 6-DOF error is unchanged by the conversion, so the integrator sees
    // the same thing it would have seen in the TCP frame.
    const auto measured_tcp = make_pose(0.38, 0.11, 0.30, M_PI, 0.0, 0.0);
    tf2::Vector3 pos_tcp, pos_ee;
    tf2::Quaternion rot_tcp, rot_ee;
    double ang_tcp, ang_ee;
    pose_error(measured_tcp, goal_tcp, pos_tcp, rot_tcp, ang_tcp);
    pose_error(measured_tcp * t_tcp_ee, goal_ee, pos_ee, rot_ee, ang_ee);
    EXPECT_NEAR(pos_ee.length(), pos_tcp.length(), 1e-9);
    EXPECT_NEAR(ang_ee, ang_tcp, 1e-9);
}

TEST(EquilibriumFrom, PositionIsTheGoalPlusTheLead)
{
    const auto goal = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    const tracking_law::Lead lead{tf2::Vector3(0.003, -0.002, 0.001), tf2::Vector3(0, 0, 0)};
    const auto eq = equilibrium_from(goal, lead);
    EXPECT_NEAR((eq.getOrigin() - (goal.getOrigin() + lead.pos)).length(), 0.0, 1e-12);
}

TEST(EquilibriumFrom, OrientationComesFromTheGoalNotTheMeasuredPose)
{
    // The signature takes no measured pose at all, so the angular deadband
    // cannot ratchet into the target (TRACKING_SPEC.md Decision 3): with no
    // angular lead the published orientation IS the vision goal's.
    const auto goal = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.3);
    const auto eq = equilibrium_from(goal, tracking_law::Lead{});
    EXPECT_NEAR(eq.getRotation().angleShortestPath(goal.getRotation()), 0.0, 1e-12);

    // With a lead, the published orientation is the goal turned by exactly
    // the lead angle - no more.
    const tracking_law::Lead lead{tf2::Vector3(0, 0, 0), tf2::Vector3(0.0, 0.0, 0.017)};
    const auto led = equilibrium_from(goal, lead);
    EXPECT_NEAR(led.getRotation().angleShortestPath(goal.getRotation()), 0.017, 1e-9);
}

TEST(PublishVeto, AcceptsAnEquilibriumAboveTheFloor)
{
    auto cfg = shipped();
    cfg.z_floor_m = 0.25;
    EXPECT_EQ(publish_veto(make_pose(0.41, 0.1, 0.3, M_PI, 0.0, 0.0), cfg), "");
}

TEST(PublishVeto, RefusesAnEquilibriumBelowTheZFloor)
{
    auto cfg = shipped();
    cfg.z_floor_m = 0.25;
    EXPECT_NE(publish_veto(make_pose(0.4, 0.1, 0.24, M_PI, 0.0, 0.0), cfg), "");
}

TEST(PublishVeto, RefusesANonFiniteEquilibrium)
{
    const auto cfg = shipped();
    const tf2::Transform bad_origin(tf2::Quaternion::getIdentity(),
                                    tf2::Vector3(kNan, 0.1, 0.3));
    EXPECT_NE(publish_veto(bad_origin, cfg), "");
    const tf2::Transform bad_rotation(tf2::Quaternion(kNan, 0.0, 0.0, 1.0),
                                      tf2::Vector3(0.4, 0.1, 0.3));
    EXPECT_NE(publish_veto(bad_rotation, cfg), "");
}

namespace
{
const OverLead kPolicies[] = {OverLead::kHold, OverLead::kStop, OverLead::kClamp};
}

TEST(Decide, PublishesAnEquilibriumInsideBothCapsUntouched)
{
    const auto cfg = shipped();
    const auto measured = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    const auto eq = make_pose(0.45, 0.1, 0.3, M_PI, 0.0, 0.2);   // 50 mm, 11 deg
    for (OverLead policy : kPolicies) {
        const auto v = decide(eq, measured, cfg, policy);
        EXPECT_EQ(v.act, Verdict::Act::kPublish);
        EXPECT_EQ(v.reason, "");
        EXPECT_NEAR((v.eq.getOrigin() - eq.getOrigin()).length(), 0.0, 1e-12);
        EXPECT_NEAR(angle_between(v.eq.getRotation(), eq.getRotation()), 0.0, 1e-9);
    }
}

TEST(Decide, HoldHoldsPastEitherCap)
{
    // 70 mm of lead against a 60 mm cap: at 1500 N/m that is 105 N commanded
    // into a 30 N ceiling. 20 deg against the 15 deg angular cap.
    const auto cfg = shipped();
    const auto measured = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    for (const auto &eq : {make_pose(0.47, 0.1, 0.3, M_PI, 0.0, 0.0),
                           make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 20.0 * M_PI / 180.0)}) {
        const auto v = decide(eq, measured, cfg, OverLead::kHold);
        EXPECT_EQ(v.act, Verdict::Act::kHold);
        EXPECT_NE(v.reason, "");
    }
}

TEST(Decide, StopStopsPastEitherCap)
{
    const auto cfg = shipped();
    const auto measured = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    for (const auto &eq : {make_pose(0.47, 0.1, 0.3, M_PI, 0.0, 0.0),
                           make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 20.0 * M_PI / 180.0)}) {
        const auto v = decide(eq, measured, cfg, OverLead::kStop);
        EXPECT_EQ(v.act, Verdict::Act::kStop);
        EXPECT_NE(v.reason, "");
    }
}

TEST(Decide, ClampCutsTheLeadBackToBothCaps)
{
    // 100 mm and 30 deg out: published 60 mm and 0.26 rad from the arm, on
    // the straight line and the shortest rotation toward the equilibrium.
    const auto cfg = shipped();
    const auto measured = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    tf2::Quaternion turn;
    turn.setRotation(tf2::Vector3(1.0, 2.0, 2.0).normalized(), 30.0 * M_PI / 180.0);
    const tf2::Transform eq(turn * measured.getRotation(),
                            measured.getOrigin() + tf2::Vector3(0.06, 0.08, 0.0));
    const auto v = decide(eq, measured, cfg, OverLead::kClamp);
    ASSERT_EQ(v.act, Verdict::Act::kPublish);
    EXPECT_NE(v.reason, "");   // the log says what the clamp cut

    const tf2::Vector3 lead = v.eq.getOrigin() - measured.getOrigin();
    EXPECT_NEAR(lead.length(), cfg.max_lead_m, 1e-12);
    EXPECT_NEAR(lead.normalized().dot(tf2::Vector3(0.6, 0.8, 0.0)), 1.0, 1e-12);

    const double to_arm = angle_between(v.eq.getRotation(), measured.getRotation());
    const double to_eq = angle_between(eq.getRotation(), v.eq.getRotation());
    EXPECT_NEAR(to_arm, cfg.max_lead_rad, 1e-9);
    EXPECT_NEAR(to_arm + to_eq, 30.0 * M_PI / 180.0, 1e-9);   // on the shortest path

    // Only the axis that is over its cap is cut.
    const auto far = make_pose(0.5, 0.1, 0.3, M_PI, 0.0, 0.05);
    const auto part = decide(far, measured, cfg, OverLead::kClamp);
    ASSERT_EQ(part.act, Verdict::Act::kPublish);
    EXPECT_NEAR((part.eq.getOrigin() - measured.getOrigin()).length(), cfg.max_lead_m, 1e-12);
    EXPECT_NEAR(angle_between(part.eq.getRotation(), far.getRotation()), 0.0, 1e-6);
}

TEST(Decide, TheZFloorAlwaysHolds)
{
    auto cfg = shipped();
    cfg.z_floor_m = 0.25;
    const auto measured = make_pose(0.4, 0.1, 0.27, M_PI, 0.0, 0.0);
    const auto below = make_pose(0.4, 0.1, 0.24, M_PI, 0.0, 0.0);   // inside both caps
    for (OverLead policy : kPolicies) {
        EXPECT_EQ(decide(below, measured, cfg, policy).act, Verdict::Act::kHold);
    }
    // A dive past the 60 mm cap as well: the floor still holds, under 'stop'
    // too, and the reason names the floor, not the cap. Clamping it still
    // ends below the floor.
    const auto dive = make_pose(0.4, 0.1, 0.17, M_PI, 0.0, 0.0);
    for (OverLead policy : kPolicies) {
        const auto v = decide(dive, measured, cfg, policy);
        EXPECT_EQ(v.act, Verdict::Act::kHold) << over_lead_name(policy);
        EXPECT_NE(v.reason.find("below the floor"), std::string::npos)
            << over_lead_name(policy) << ": " << v.reason;
    }
}

TEST(Decide, ClampIsFlooredOnThePoseItWouldPublish)
{
    // The same dive from 50 mm higher clamps to 260 mm, above the 250 mm
    // floor: clamp follows, it does not hold on the pose it cut back.
    auto cfg = shipped();
    cfg.z_floor_m = 0.25;
    const auto measured = make_pose(0.4, 0.1, 0.32, M_PI, 0.0, 0.0);
    const auto dive = make_pose(0.4, 0.1, 0.17, M_PI, 0.0, 0.0);
    const auto v = decide(dive, measured, cfg, OverLead::kClamp);
    ASSERT_EQ(v.act, Verdict::Act::kPublish) << v.reason;
    EXPECT_NEAR(v.eq.getOrigin().z(), 0.26, 1e-12);
    EXPECT_GE(v.eq.getOrigin().z(), cfg.z_floor_m);
}

TEST(Decide, ANonFiniteEquilibriumAlwaysHolds)
{
    // Never a stop, never a clamp: NaN is not "far".
    const auto cfg = shipped();
    const auto measured = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    const tf2::Transform bad_origin(tf2::Quaternion::getIdentity(),
                                    tf2::Vector3(kNan, 0.1, 0.3));
    const tf2::Transform bad_rotation(tf2::Quaternion(kNan, 0.0, 0.0, 1.0),
                                      tf2::Vector3(0.4, 0.1, 0.3));
    // Past the position cap as well: clamp_lead would rebuild this as a
    // FINITE pose with the measured orientation, so only the finite check
    // keeps clamp from publishing it.
    const tf2::Transform bad_rotation_far(tf2::Quaternion(kNan, 0.0, 0.0, 1.0),
                                          tf2::Vector3(0.5, 0.1, 0.3));
    for (OverLead policy : kPolicies) {
        EXPECT_EQ(decide(bad_origin, measured, cfg, policy).act, Verdict::Act::kHold);
        EXPECT_EQ(decide(bad_rotation, measured, cfg, policy).act, Verdict::Act::kHold);
        EXPECT_EQ(decide(bad_rotation_far, measured, cfg, policy).act, Verdict::Act::kHold);
    }
}

TEST(ClosedLoop, ConvergesOnAStaticMarker)
{
    // Closes inside half the deadband - where the lead freezes - which the
    // spring alone cannot: it parks a whole friction band (4.3 mm) off.
    const auto cfg = shipped();
    const Outcome run = run_loop(Loop{});
    EXPECT_LT(run.pos_err_m, cfg.deadband_m * 0.5 + 1e-4);
    EXPECT_LT(run.rot_err_rad, cfg.deadband_rad * 0.5 + 1e-4);
    EXPECT_EQ(run.holds, 0);
}

TEST(ClosedLoop, FollowsASlowlyMovingMarker)
{
    // 5 mm/s and 1 deg/s: the second half of the run stays inside the
    // deadband, without a single hold.
    const auto cfg = shipped();
    Loop loop;
    loop.drift_mps = 0.005;
    loop.spin_rps = M_PI / 180.0;
    const Outcome run = run_loop(loop);
    EXPECT_LT(run.worst_pos_m, cfg.deadband_m);
    EXPECT_LT(run.worst_rot_rad, cfg.deadband_rad);
    EXPECT_EQ(run.holds, 0);
}

TEST(ClosedLoop, ALeadOfTheWrongSignDoesNotConverge)
{
    // Proves the loop above can see a sign bug: integrating the error the
    // wrong way parks the arm further off than no integrator at all.
    Loop loop;
    loop.lead_sign = -1.0;
    const Outcome run = run_loop(loop);
    EXPECT_GT(run.pos_err_m, Arm{}.band_m);
    EXPECT_GT(run.rot_err_rad, Arm{}.band_rad);
}

TEST(ClosedLoop, AGoalOfTheWrongInplaneSignDoesNotConverge)
{
    // Same for the goal: the mirrored in-plane angle turns the camera 180 deg
    // about its axis. With clamp the arm sets off toward it (hold would never
    // move) until the Z floor holds it - nowhere near the ALIGN pose.
    const auto cfg = shipped();
    Loop loop;
    loop.inplane_sign = -1.0;
    loop.policy = OverLead::kClamp;
    const Outcome run = run_loop(loop);
    EXPECT_GT(run.pos_err_m, cfg.deadband_m);
    EXPECT_GT(run.rot_err_rad, cfg.deadband_rad);
}

TEST(RotationVector, AgreesWithPoseErrorOnASmallRotation)
{
    const auto current = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    const auto goal = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.02);
    tf2::Vector3 pos_err;
    tf2::Quaternion rot_err;
    double rot_angle;
    pose_error(current, goal, pos_err, rot_err, rot_angle);
    EXPECT_NEAR(rotation_vector(rot_err).length(), rot_angle, 1e-9);
}

TEST(RotationVector, FoldsAtPi)
{
    tf2::Quaternion q;
    q.setRotation(tf2::Vector3(0, 0, 1), M_PI);
    EXPECT_NEAR(rotation_vector(q).length(), M_PI, 1e-9);

    // Just past pi the short way is the other direction: the vector must fold
    // back, not run on toward 2*pi, or the lead would integrate the long way
    // round.
    q.setRotation(tf2::Vector3(0, 0, 1), M_PI + 0.1);
    const auto v = rotation_vector(q);
    EXPECT_NEAR(v.length(), M_PI - 0.1, 1e-9);
    EXPECT_LT(v.z(), 0.0);

    // An identity rotation has no axis to speak of: it must give exactly zero.
    EXPECT_NEAR(rotation_vector(tf2::Quaternion::getIdentity()).length(), 0.0, 1e-15);
}

// ---- buzz meter -------------------------------------------------------------

namespace
{

// A gravity-like static load, a slow 1 Hz tracking motion on every joint,
// and an optional 40 Hz buzz on J1 - the 2026-09-23 wrist mode, which read
// ~4 Nm on J1.
std::array<double, 7> torques(double t, double slow_nm, double buzz_nm)
{
    std::array<double, 7> tau{0.2, -21.0, 0.4, 14.0, 0.6, 2.1, 0.1};
    for (auto &x : tau) {
        x += slow_nm * std::sin(2.0 * M_PI * 1.0 * t);
    }
    tau[0] += buzz_nm * std::sin(2.0 * M_PI * 40.0 * t);
    return tau;
}

}  // namespace

TEST(BuzzMeter, AStaticLoadIsQuietFromTheFirstSample)
{
    tracking_law::BuzzMeter meter;
    double worst = 0.0;
    for (int i = 0; i < 1000; ++i) {
        worst = std::max(worst, meter.update(torques(i * 1e-3, 0.0, 0.0)));
    }
    EXPECT_LT(worst, 1e-9);   // primed: 21 Nm of gravity is not a step
}

TEST(BuzzMeter, TrackingMotionStaysQuiet)
{
    tracking_law::BuzzMeter meter;
    double worst = 0.0;
    for (int i = 0; i < 5000; ++i) {
        worst = std::max(worst, meter.update(torques(i * 1e-3, 3.0, 0.0)));
    }
    EXPECT_LT(worst, 0.05);   // 3 Nm at 1 Hz, against the 1 Nm stop
}

TEST(BuzzMeter, AFortyHertzBuzzTripsWithinFiftyMillisecondsOnItsJoint)
{
    tracking_law::BuzzMeter meter;
    for (int i = 0; i < 1000; ++i) {
        meter.update(torques(i * 1e-3, 3.0, 0.0));
    }
    int joint = -1;
    int tripped_at = -1;
    for (int i = 0; i < 300 && tripped_at < 0; ++i) {
        if (meter.update(torques(1.0 + i * 1e-3, 3.0, 4.0), &joint) > 1.0) {
            tripped_at = i;
        }
    }
    ASSERT_GE(tripped_at, 0) << "a 4 Nm buzz never reached the 1 Nm stop";
    EXPECT_LE(tripped_at, 50);
    EXPECT_EQ(joint, 0);
    for (int i = 0; i < 1000; ++i) {
        meter.update(torques(1.3 + i * 1e-3, 3.0, 4.0), &joint);
    }
    EXPECT_NEAR(meter.level(), 4.0 / std::sqrt(2.0), 0.25);   // rms of the buzz
}

TEST(BuzzMeter, ResetPrimesAgainWithoutATransient)
{
    tracking_law::BuzzMeter meter;
    for (int i = 0; i < 200; ++i) {
        meter.update(torques(i * 1e-3, 0.0, 0.0));
    }
    meter.reset();
    std::array<double, 7> other{5.0, -30.0, 1.0, 20.0, -1.0, 3.0, 0.5};
    double worst = 0.0;
    for (int i = 0; i < 200; ++i) {
        worst = std::max(worst, meter.update(other));
    }
    EXPECT_LT(worst, 1e-9);
}

TEST(ValidateConfig, RejectsABadBuzzStop)
{
    auto cfg = shipped();
    cfg.buzz_stop_nm = 0.0;
    EXPECT_NE(check(cfg), "");
    cfg.buzz_stop_nm = kNan;
    EXPECT_NE(check(cfg), "");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
