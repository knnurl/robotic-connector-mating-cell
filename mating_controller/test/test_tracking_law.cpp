// Unit tests for the continuous-tracking control law: the bounded integrator,
// the TCP-to-EE frame conversion and the publish veto.
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mating_controller/mating_geometry.hpp"
#include "mating_controller/tracking_law.hpp"

using mating_geometry::pose_error;
using tracking_law::advance_lead;
using tracking_law::equilibrium_from;
using tracking_law::goal_in_ee;
using tracking_law::publish_veto;
using tracking_law::rotation_vector;
using tracking_law::validate_config;

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
    cfg.z_floor_m = 0.0;
    return cfg;
}

// validate_config against the shipped track stiffnesses and the controller's
// two ceilings (cartesian_impedance_stroke.yaml).
std::string check(const tracking_law::Config &cfg)
{
    return validate_config(cfg, 1500.0, 90.0, 30.0, 10.0);
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
    // standoff_goal produces a TCP goal. The conversion must ride ON the
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

TEST(PublishVeto, AcceptsAnEquilibriumNearTheArmAndAboveTheFloor)
{
    auto cfg = shipped();
    cfg.z_floor_m = 0.25;
    const auto measured = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    const auto eq = make_pose(0.41, 0.1, 0.3, M_PI, 0.0, 0.0);
    EXPECT_EQ(publish_veto(eq, measured, cfg), "");
}

TEST(PublishVeto, RefusesAnEquilibriumFurtherThanTheLeadCap)
{
    // 70 mm of lead against a 60 mm cap: at 1500 N/m that is 105 N commanded
    // into a 30 N ceiling, which is the panel's own refusal.
    const auto cfg = shipped();
    const auto measured = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    const auto eq = make_pose(0.47, 0.1, 0.3, M_PI, 0.0, 0.0);
    EXPECT_NE(publish_veto(eq, measured, cfg), "");
}

TEST(PublishVeto, RefusesAnEquilibriumBelowTheZFloor)
{
    auto cfg = shipped();
    cfg.z_floor_m = 0.25;
    const auto measured = make_pose(0.4, 0.1, 0.27, M_PI, 0.0, 0.0);
    const auto eq = make_pose(0.4, 0.1, 0.24, M_PI, 0.0, 0.0);
    EXPECT_NE(publish_veto(eq, measured, cfg), "");
}

TEST(PublishVeto, RefusesANonFiniteEquilibrium)
{
    const auto cfg = shipped();
    const auto measured = make_pose(0.4, 0.1, 0.3, M_PI, 0.0, 0.0);
    const tf2::Transform bad_origin(tf2::Quaternion::getIdentity(),
                                    tf2::Vector3(kNan, 0.1, 0.3));
    EXPECT_NE(publish_veto(bad_origin, measured, cfg), "");
    const tf2::Transform bad_rotation(tf2::Quaternion(kNan, 0.0, 0.0, 1.0),
                                      tf2::Vector3(0.4, 0.1, 0.3));
    EXPECT_NE(publish_veto(bad_rotation, measured, cfg), "");
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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
