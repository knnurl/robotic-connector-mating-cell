// Pure pose math for the connector-mating controller. No ROS node code here
// so the geometry is unit-testable (see test/test_mating_geometry.cpp).
#pragma once

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace mating_geometry
{

// Desired TCP pose hovering above the connector, given the marker pose in
// some reference frame. The connector mate point sits at (offset_x, offset_y,
// offset_z) in the marker frame (X/Y in the marker plane, Z out of the work
// surface); the goal hovers standoff_height above it along marker +Z with the
// tool Z pointing into the surface (marker frame rotated 180 deg about X),
// optionally yawed about the marker normal.
inline tf2::Transform standoff_goal(const tf2::Transform &t_ref_marker,
                                    double offset_x, double offset_y, double offset_z,
                                    double standoff_height, double tool_yaw_rad)
{
    const tf2::Vector3 goal_position(offset_x, offset_y, offset_z + standoff_height);
    tf2::Quaternion q_flip, q_yaw;
    q_flip.setRPY(M_PI, 0.0, 0.0);
    q_yaw.setRPY(0.0, 0.0, tool_yaw_rad);
    const tf2::Transform t_marker_goal(q_yaw * q_flip, goal_position);
    return t_ref_marker * t_marker_goal;
}

// 6-DOF error between two poses. rot_angle is the magnitude of the shortest
// rotation, in [0, pi].
inline void pose_error(const tf2::Transform &current, const tf2::Transform &goal,
                       tf2::Vector3 &pos_err, tf2::Quaternion &rot_err, double &rot_angle)
{
    pos_err = goal.getOrigin() - current.getOrigin();
    rot_err = goal.getRotation() * current.getRotation().inverse();
    rot_err.normalize();
    rot_angle = rot_err.getAngle();  // in [0, 2*pi]
    if (rot_angle > M_PI) {
        rot_angle = 2.0 * M_PI - rot_angle;
    }
}

// Clamp the pose error to per-cycle limits and return the commanded pose.
inline tf2::Transform clamped_target(const tf2::Transform &current,
                                     const tf2::Vector3 &pos_err,
                                     const tf2::Quaternion &rot_err,
                                     double max_step_m, double max_step_rad)
{
    tf2::Vector3 step = pos_err;
    const double dist = step.length();
    if (dist > max_step_m) {
        step *= max_step_m / dist;
    }

    double angle = rot_err.getAngle();
    tf2::Vector3 axis = rot_err.getAxis();
    if (angle > M_PI) {  // take the short way
        angle = 2.0 * M_PI - angle;
        axis = -axis;
    }
    tf2::Quaternion q_step;
    if (angle > 1e-6) {
        q_step.setRotation(axis, std::min(angle, max_step_rad));
    } else {
        q_step = tf2::Quaternion::getIdentity();
    }

    tf2::Transform target;
    target.setOrigin(current.getOrigin() + step);
    target.setRotation((q_step * current.getRotation()).normalized());
    return target;
}

// Committed insertion stroke: straight along the current tool Z axis with the
// orientation locked.
inline tf2::Transform insertion_target(const tf2::Transform &current, double depth)
{
    const tf2::Vector3 tool_z =
        tf2::quatRotate(current.getRotation(), tf2::Vector3(0, 0, 1));
    tf2::Transform target = current;
    target.setOrigin(current.getOrigin() + tool_z * depth);
    return target;
}

// Proportional Cartesian velocity command toward the goal, for servo-mode
// alignment: v = clamp(gain * err). The angular command follows the
// shortest rotation (same convention as clamped_target). Zero error yields
// exactly zero twist - the deadman/hold command.
inline void servo_twist(const tf2::Vector3 &pos_err, const tf2::Quaternion &rot_err,
                        double pos_gain, double rot_gain,
                        double max_lin, double max_rot,
                        tf2::Vector3 &linear, tf2::Vector3 &angular)
{
    linear = pos_err * pos_gain;
    const double lin_mag = linear.length();
    if (lin_mag > max_lin) {
        linear *= max_lin / lin_mag;
    }

    double angle = rot_err.getAngle();
    tf2::Vector3 axis = rot_err.getAxis();
    if (angle > M_PI) {  // take the short way
        angle = 2.0 * M_PI - angle;
        axis = -axis;
    }
    if (angle < 1e-9) {
        angular = tf2::Vector3(0, 0, 0);
        return;
    }
    angular = axis * std::min(angle * rot_gain, max_rot);
}

// Split an external force (expressed in the same frame as tool_z) into the
// component along the tool axis (signed: positive = pushing back against
// the insertion direction) and the lateral remainder magnitude. Used to
// tell "seated" (axial reaction) from "snagged" (lateral load) during the
// insertion stroke.
inline void wrench_axial_lateral(const tf2::Vector3 &force, const tf2::Vector3 &tool_z,
                                 double &axial, double &lateral)
{
    const tf2::Vector3 z = tool_z.normalized();
    // The stroke pushes along +tool_z; the surface reacts along -tool_z.
    axial = -force.dot(z);
    lateral = (force - z * force.dot(z)).length();
}

}  // namespace mating_geometry
