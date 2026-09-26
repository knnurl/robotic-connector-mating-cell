// Unit tests for the connector-mating pose math.
#include <cmath>

#include <gtest/gtest.h>

#include "mating_controller/mating_geometry.hpp"

using mating_geometry::clamped_target;
using mating_geometry::insertion_target;
using mating_geometry::pose_error;
using mating_geometry::servo_twist;
using mating_geometry::standoff_goal;
using mating_geometry::wrench_axial_lateral;

namespace
{

tf2::Transform make_pose(double x, double y, double z,
                         double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::Transform(q, tf2::Vector3(x, y, z));
}

}  // namespace

TEST(StandoffGoal, MarkerAtIdentityGoalHoversAboveWithToolDown)
{
    const tf2::Transform marker = tf2::Transform::getIdentity();
    const auto goal = standoff_goal(marker, 0.0, 0.0, 0.0, 0.10, 0.0);

    EXPECT_NEAR(goal.getOrigin().x(), 0.0, 1e-9);
    EXPECT_NEAR(goal.getOrigin().y(), 0.0, 1e-9);
    EXPECT_NEAR(goal.getOrigin().z(), 0.10, 1e-9);

    // Tool Z must point INTO the surface (anti-parallel to marker Z).
    const tf2::Vector3 tool_z =
        tf2::quatRotate(goal.getRotation(), tf2::Vector3(0, 0, 1));
    EXPECT_NEAR(tool_z.z(), -1.0, 1e-9);
}

TEST(StandoffGoal, TracksMarkerTiltRollPitch)
{
    // Marker tilted 10 deg about X: the goal must tilt with it (this is the
    // roll/pitch compensation feature).
    const double tilt = 10.0 * M_PI / 180.0;
    const tf2::Transform marker = make_pose(0.4, 0.1, 0.05, tilt, 0.0, 0.0);
    const auto goal = standoff_goal(marker, 0.0, 0.0, 0.0, 0.10, 0.0);

    // Goal position: 0.10 along the *tilted* marker Z.
    const tf2::Vector3 marker_z =
        tf2::quatRotate(marker.getRotation(), tf2::Vector3(0, 0, 1));
    const tf2::Vector3 expected = marker.getOrigin() + marker_z * 0.10;
    EXPECT_NEAR(goal.getOrigin().x(), expected.x(), 1e-9);
    EXPECT_NEAR(goal.getOrigin().y(), expected.y(), 1e-9);
    EXPECT_NEAR(goal.getOrigin().z(), expected.z(), 1e-9);

    // Tool Z anti-parallel to the tilted marker normal.
    const tf2::Vector3 tool_z =
        tf2::quatRotate(goal.getRotation(), tf2::Vector3(0, 0, 1));
    EXPECT_NEAR(tool_z.dot(marker_z), -1.0, 1e-9);
}

TEST(StandoffGoal, ConnectorOffsetExpressedInMarkerFrame)
{
    // Marker yawed 90 deg: an offset of +X in the marker frame must appear
    // as +Y in the reference frame.
    const tf2::Transform marker = make_pose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 2.0);
    const auto goal = standoff_goal(marker, 0.05, 0.0, 0.0, 0.10, 0.0);

    EXPECT_NEAR(goal.getOrigin().x(), 0.0, 1e-9);
    EXPECT_NEAR(goal.getOrigin().y(), 0.05, 1e-9);
    EXPECT_NEAR(goal.getOrigin().z(), 0.10, 1e-9);
}

TEST(StandoffGoal, YawOffsetRotatesToolAboutMarkerNormal)
{
    const tf2::Transform marker = tf2::Transform::getIdentity();
    const double yaw = 30.0 * M_PI / 180.0;
    const auto goal = standoff_goal(marker, 0.0, 0.0, 0.0, 0.10, yaw);

    // Tool Z unchanged by yaw offset...
    const tf2::Vector3 tool_z =
        tf2::quatRotate(goal.getRotation(), tf2::Vector3(0, 0, 1));
    EXPECT_NEAR(tool_z.z(), -1.0, 1e-9);
    // ...tool X rotated by the yaw offset within the marker plane.
    const tf2::Vector3 tool_x =
        tf2::quatRotate(goal.getRotation(), tf2::Vector3(1, 0, 0));
    EXPECT_NEAR(tool_x.x(), std::cos(yaw), 1e-9);
}

TEST(PoseError, ZeroForIdenticalPoses)
{
    const tf2::Transform p = make_pose(0.3, -0.2, 0.5, 0.1, 0.2, 0.3);
    tf2::Vector3 pos_err;
    tf2::Quaternion rot_err;
    double angle;
    pose_error(p, p, pos_err, rot_err, angle);
    EXPECT_NEAR(pos_err.length(), 0.0, 1e-9);
    EXPECT_NEAR(angle, 0.0, 1e-6);
}

TEST(PoseError, ReportsShortestAngle)
{
    const tf2::Transform current = tf2::Transform::getIdentity();
    // 350 deg yaw goal == -10 deg shortest rotation
    const tf2::Transform goal = make_pose(0, 0, 0, 0, 0, 350.0 * M_PI / 180.0);
    tf2::Vector3 pos_err;
    tf2::Quaternion rot_err;
    double angle;
    pose_error(current, goal, pos_err, rot_err, angle);
    EXPECT_NEAR(angle, 10.0 * M_PI / 180.0, 1e-6);
}

TEST(ClampedTarget, SmallErrorReachesGoalExactly)
{
    const tf2::Transform current = make_pose(0, 0, 0.5, 0, 0, 0);
    const tf2::Transform goal = make_pose(0.005, -0.003, 0.502, 0.01, -0.005, 0.02);
    tf2::Vector3 pos_err;
    tf2::Quaternion rot_err;
    double angle;
    pose_error(current, goal, pos_err, rot_err, angle);

    const auto target = clamped_target(current, pos_err, rot_err, 0.05, 0.1);

    tf2::Vector3 residual_pos;
    tf2::Quaternion residual_rot;
    double residual_angle;
    pose_error(target, goal, residual_pos, residual_rot, residual_angle);
    EXPECT_NEAR(residual_pos.length(), 0.0, 1e-9);
    EXPECT_NEAR(residual_angle, 0.0, 1e-6);
}

TEST(ClampedTarget, LargeErrorClampedToLimits)
{
    const tf2::Transform current = tf2::Transform::getIdentity();
    const tf2::Transform goal = make_pose(0.5, -0.5, 0.3, 0.0, 0.0, M_PI / 2.0);
    tf2::Vector3 pos_err;
    tf2::Quaternion rot_err;
    double angle;
    pose_error(current, goal, pos_err, rot_err, angle);

    const double max_step_m = 0.05;
    const double max_step_rad = 5.0 * M_PI / 180.0;
    const auto target = clamped_target(current, pos_err, rot_err, max_step_m, max_step_rad);

    // Translation step exactly at the clamp, along the error direction.
    const tf2::Vector3 step = target.getOrigin() - current.getOrigin();
    EXPECT_NEAR(step.length(), max_step_m, 1e-9);
    EXPECT_NEAR(step.normalized().dot(pos_err.normalized()), 1.0, 1e-9);

    // Rotation step exactly at the clamp.
    tf2::Quaternion q_step =
        target.getRotation() * current.getRotation().inverse();
    double step_angle = q_step.getAngle();
    if (step_angle > M_PI) step_angle = 2.0 * M_PI - step_angle;
    EXPECT_NEAR(step_angle, max_step_rad, 1e-6);
}

TEST(ClampedTarget, TakesShortWayAroundForReflexRotations)
{
    const tf2::Transform current = tf2::Transform::getIdentity();
    const tf2::Transform goal = make_pose(0, 0, 0, 0, 0, -20.0 * M_PI / 180.0);
    tf2::Vector3 pos_err;
    tf2::Quaternion rot_err;
    double angle;
    pose_error(current, goal, pos_err, rot_err, angle);

    const auto target = clamped_target(current, pos_err, rot_err, 0.05,
                                       5.0 * M_PI / 180.0);
    // Must move toward -yaw, not +yaw the long way.
    double yaw, pitch, roll;
    target.getBasis().getEulerYPR(yaw, pitch, roll);
    EXPECT_NEAR(yaw, -5.0 * M_PI / 180.0, 1e-6);
}

TEST(InsertionTarget, DescendsAlongToolAxisWithOrientationLocked)
{
    // Tool pointing straight down (180 deg roll): tool Z = world -Z, so the
    // stroke must DECREASE world z.
    const tf2::Transform current = make_pose(0.4, 0.1, 0.15, M_PI, 0.0, 0.0);
    const auto target = insertion_target(current, 0.05);

    EXPECT_NEAR(target.getOrigin().x(), 0.4, 1e-9);
    EXPECT_NEAR(target.getOrigin().y(), 0.1, 1e-9);
    EXPECT_NEAR(target.getOrigin().z(), 0.10, 1e-9);
    EXPECT_NEAR(target.getRotation().angleShortestPath(current.getRotation()),
                0.0, 1e-9);
}

TEST(InsertionTarget, NegativeDepthRetractsAlongToolAxis)
{
    // Retract = insertion with negative depth: straight back UP the tool
    // axis, orientation locked. With the tool pointing down (tool Z =
    // world -Z), a -0.04 stroke must INCREASE world z by 0.04.
    const tf2::Transform current = make_pose(0.4, 0.1, 0.11, M_PI, 0.0, 0.0);
    const auto target = insertion_target(current, -0.04);

    EXPECT_NEAR(target.getOrigin().x(), 0.4, 1e-9);
    EXPECT_NEAR(target.getOrigin().y(), 0.1, 1e-9);
    EXPECT_NEAR(target.getOrigin().z(), 0.15, 1e-9);
    EXPECT_NEAR(target.getRotation().angleShortestPath(current.getRotation()),
                0.0, 1e-9);

    // Retract must exactly invert a partial insertion from the same pose.
    const auto inserted = insertion_target(current, 0.04);
    const auto back = insertion_target(inserted, -0.04);
    EXPECT_NEAR((back.getOrigin() - current.getOrigin()).length(), 0.0, 1e-9);
}

TEST(InsertionTarget, FollowsTiltedTool)
{
    // Tool tilted 10 deg: the stroke must follow the tilted axis, not world Z.
    const double tilt = 10.0 * M_PI / 180.0;
    const tf2::Transform current = make_pose(0.0, 0.0, 0.2, M_PI - tilt, 0.0, 0.0);
    const auto target = insertion_target(current, 0.1);

    const tf2::Vector3 tool_z =
        tf2::quatRotate(current.getRotation(), tf2::Vector3(0, 0, 1));
    const tf2::Vector3 expected = current.getOrigin() + tool_z * 0.1;
    EXPECT_NEAR((target.getOrigin() - expected).length(), 0.0, 1e-9);
}

TEST(ServoTwist, ProportionalInsideClampsZeroAtGoal)
{
    // Small error: exact P-law, no clamping.
    tf2::Vector3 lin, ang;
    tf2::Quaternion rot_err;
    rot_err.setRPY(0.0, 0.0, 0.1);  // 0.1 rad yaw error
    servo_twist(tf2::Vector3(0.01, 0.0, 0.0), rot_err,
                2.0, 1.5, 0.05, 0.5, lin, ang);
    EXPECT_NEAR(lin.x(), 0.02, 1e-9);
    EXPECT_NEAR(lin.y(), 0.0, 1e-9);
    EXPECT_NEAR(ang.z(), 0.15, 1e-6);

    // Zero error: exactly zero twist (the hold/deadman command).
    servo_twist(tf2::Vector3(0, 0, 0), tf2::Quaternion::getIdentity(),
                2.0, 1.5, 0.05, 0.5, lin, ang);
    EXPECT_EQ(lin.length(), 0.0);
    EXPECT_EQ(ang.length(), 0.0);
}

TEST(ServoTwist, ClampsMagnitudePreservesDirection)
{
    tf2::Vector3 lin, ang;
    tf2::Quaternion rot_err;
    rot_err.setRPY(0.0, 0.0, 2.0);  // huge yaw error
    const tf2::Vector3 err(0.3, 0.4, 0.0);  // 0.5 m error
    servo_twist(err, rot_err, 2.0, 1.5, 0.05, 0.5, lin, ang);
    EXPECT_NEAR(lin.length(), 0.05, 1e-9);
    EXPECT_NEAR(lin.normalized().dot(err.normalized()), 1.0, 1e-9);
    EXPECT_NEAR(ang.length(), 0.5, 1e-9);
    EXPECT_GT(ang.z(), 0.0);
}

TEST(ServoTwist, TakesShortWayAroundForReflexRotations)
{
    tf2::Vector3 lin, ang;
    tf2::Quaternion rot_err;
    rot_err.setRPY(0.0, 0.0, -0.2);  // -0.2 rad = 2*pi - 0.2 the long way
    servo_twist(tf2::Vector3(0, 0, 0), rot_err, 1.0, 1.0, 0.05, 0.5, lin, ang);
    EXPECT_NEAR(ang.z(), -0.2, 1e-6);  // must go the short (negative) way
}

TEST(WrenchAxialLateral, DecomposesAlongTiltedToolAxis)
{
    // Tool pointing straight down (stroke direction = world -Z). A surface
    // pushing BACK on the tool pushes along world +Z -> positive axial.
    double axial, lateral;
    wrench_axial_lateral(tf2::Vector3(0, 0, 10.0), tf2::Vector3(0, 0, -1),
                         axial, lateral);
    EXPECT_NEAR(axial, 10.0, 1e-9);
    EXPECT_NEAR(lateral, 0.0, 1e-9);

    // Pure sideways snag: no axial component, full lateral.
    wrench_axial_lateral(tf2::Vector3(3.0, 4.0, 0.0), tf2::Vector3(0, 0, -1),
                         axial, lateral);
    EXPECT_NEAR(axial, 0.0, 1e-9);
    EXPECT_NEAR(lateral, 5.0, 1e-9);

    // Mixed load on a tilted axis decomposes exactly.
    const tf2::Vector3 z = tf2::Vector3(1, 0, -1).normalized();
    const tf2::Vector3 f = z * -7.0 + tf2::Vector3(0, 2.0, 0);  // 7 N reaction + 2 N lateral
    wrench_axial_lateral(f, z, axial, lateral);
    EXPECT_NEAR(axial, 7.0, 1e-9);
    EXPECT_NEAR(lateral, 2.0, 1e-9);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
