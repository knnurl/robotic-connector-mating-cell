// The two pieces of the impedance controller that decide safety without any
// robot state: which live gains and slew caps are acceptable, and how a
// setpoint reaches the 1 kHz loop. Runs without franka, ROS or a controller
// manager.
//
//   colcon test --packages-select fr3_mating_controllers
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <mutex>
#include <thread>

#include "fr3_mating_controllers/impedance_detail.hpp"

using fr3_mating_controllers::detail::ConfigLimits;
using fr3_mating_controllers::detail::GainLimits;
using fr3_mating_controllers::detail::kTauSpecNm;
using fr3_mating_controllers::detail::TargetHandoff;
using fr3_mating_controllers::detail::validate_gains;
using fr3_mating_controllers::detail::validate_limits;
using fr3_mating_controllers::detail::validate_slew;

namespace
{
const Eigen::Vector3d kPos{150.0, 150.0, 800.0};
const Eigen::Vector3d kRot{10.0, 10.0, 20.0};
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

// Exposes the lock, so a test can hold it the way a publisher mid-write does.
struct ExposedHandoff : TargetHandoff
{
    using TargetHandoff::mutex_;
};
}  // namespace

TEST(ValidateGains, AcceptsTheShippedDefaults)
{
    EXPECT_EQ(validate_gains(kPos, kRot, 1.0, 5.0), "");
}

TEST(ValidateGains, RejectsDampingThatCannotStabilise)
{
    // 0 = undamped spring, negative = energy injected every cycle. The force
    // ceiling bounds how hard the arm pushes, not whether it oscillates.
    EXPECT_NE(validate_gains(kPos, kRot, 0.0, 5.0), "");
    EXPECT_NE(validate_gains(kPos, kRot, -1.0, 5.0), "");
    EXPECT_NE(validate_gains(kPos, kRot, 0.05, 5.0), "");
    EXPECT_NE(validate_gains(kPos, kRot, 2.5, 5.0), "");
}

TEST(ValidateGains, RejectsNonFiniteValues)
{
    EXPECT_NE(validate_gains({kNan, 150.0, 800.0}, kRot, 1.0, 5.0), "");
    EXPECT_NE(validate_gains(kPos, {10.0, kInf, 20.0}, 1.0, 5.0), "");
    EXPECT_NE(validate_gains(kPos, kRot, kNan, 5.0), "");
    EXPECT_NE(validate_gains(kPos, kRot, 1.0, kNan), "");
}

TEST(ValidateGains, RejectsOutOfRangeStiffness)
{
    EXPECT_NE(validate_gains({150.0, 150.0, 80000.0}, kRot, 1.0, 5.0), "");  // typo
    EXPECT_NE(validate_gains({-150.0, 150.0, 800.0}, kRot, 1.0, 5.0), "");
    EXPECT_NE(validate_gains(kPos, {10.0, 10.0, 301.0}, 1.0, 5.0), "");
    EXPECT_NE(validate_gains(kPos, kRot, 1.0, 51.0), "");
}

TEST(ValidateGains, BoundariesAreInclusive)
{
    EXPECT_EQ(validate_gains({0.0, 0.0, GainLimits::kPosMax}, {0.0, 0.0, GainLimits::kRotMax},
                             GainLimits::zetaMin, 0.0), "");
    EXPECT_EQ(validate_gains(kPos, kRot, GainLimits::zetaMax, GainLimits::nullspaceMax), "");
}

TEST(TargetHandoff, NothingToTakeBeforeAPublish)
{
    TargetHandoff h;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    EXPECT_FALSE(h.try_take(p, q));
}

TEST(TargetHandoff, TakesTheLatestSetpointExactlyOnce)
{
    TargetHandoff h;
    h.publish({1.0, 0.0, 0.0}, Eigen::Quaterniond::Identity());
    h.publish({2.0, 0.0, 0.0}, Eigen::Quaterniond::Identity());
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    ASSERT_TRUE(h.try_take(p, q));
    EXPECT_DOUBLE_EQ(p.x(), 2.0);
    EXPECT_FALSE(h.try_take(p, q));
}

TEST(TargetHandoff, InvalidateDiscardsEarlierSetpoints)
{
    // The float -> hold re-seed: a setpoint published while floating must not
    // drag the arm away from where the operator left it.
    TargetHandoff h;
    h.publish({5.0, 0.0, 0.0}, Eigen::Quaterniond::Identity());
    h.invalidate();
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    EXPECT_FALSE(h.try_take(p, q));
}

TEST(TargetHandoff, SetpointAfterInvalidateStillCounts)
{
    TargetHandoff h;
    h.publish({5.0, 0.0, 0.0}, Eigen::Quaterniond::Identity());
    h.invalidate();
    h.publish({6.0, 0.0, 0.0}, Eigen::Quaterniond::Identity());
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    ASSERT_TRUE(h.try_take(p, q));
    EXPECT_DOUBLE_EQ(p.x(), 6.0);
}

TEST(TargetHandoff, TryTakeNeverWaitsOnAHeldLock)
{
    // The 1 kHz loop must not block on the subscription thread's lock.
    ExposedHandoff h;
    h.publish({1.0, 2.0, 3.0}, Eigen::Quaterniond::Identity());
    std::unique_lock<std::mutex> held(h.mutex_);
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    auto taken = std::async(std::launch::async, [&] { return h.try_take(p, q); });
    if (taken.wait_for(std::chrono::milliseconds(200)) != std::future_status::ready) {
        held.unlock();  // let the blocked call finish so the test fails, not hangs
        taken.wait();
        FAIL() << "try_take blocked on a held lock";
    }
    EXPECT_FALSE(taken.get());
    held.unlock();
    ASSERT_TRUE(h.try_take(p, q));  // nothing was lost: the next cycle gets it
    EXPECT_DOUBLE_EQ(p.y(), 2.0);
}

TEST(TargetHandoff, ConcurrentPublishNeverYieldsATornSetpoint)
{
    // Publisher writes (i, i, i); a torn read would mix two publishes.
    TargetHandoff h;
    std::atomic<bool> done{false};
    std::thread publisher([&] {
        for (int i = 1; i <= 200000; ++i) {
            const double v = static_cast<double>(i);
            h.publish({v, v, v}, Eigen::Quaterniond::Identity());
        }
        done = true;
    });
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    int taken = 0;
    int torn = 0;
    int backwards = 0;
    double last = 0.0;
    while (true) {
        const bool finished = done.load();
        if (h.try_take(p, q)) {
            ++taken;
            if (p.x() != p.y() || p.y() != p.z()) {
                ++torn;
            }
            if (p.x() < last) {
                ++backwards;
            }
            last = p.x();
        } else if (finished) {
            break;
        }
    }
    publisher.join();  // before any assertion, so a failure reports instead of aborting
    EXPECT_GT(taken, 0);
    EXPECT_EQ(torn, 0);
    EXPECT_EQ(backwards, 0);
    EXPECT_DOUBLE_EQ(last, 200000.0);  // the final setpoint is always taken
}

TEST(ValidateLimits, AcceptsTheShippedValues)
{
    EXPECT_EQ(validate_limits(30.0, 10.0, 1.0, 0.05, 0.5, kTauSpecNm), "");
}

TEST(ValidateLimits, RejectsCeilingsThatDisableTheLawOrExceedFci)
{
    EXPECT_NE(validate_limits(0.0, 10.0, 1.0, 0.05, 0.5, kTauSpecNm), "");      // no force
    EXPECT_NE(validate_limits(30.0, kNan, 1.0, 0.05, 0.5, kTauSpecNm), "");
    EXPECT_NE(validate_limits(30.0, 10.0, 1.5, 0.05, 0.5, kTauSpecNm), "");     // > FCI
    EXPECT_NE(validate_limits(30.0, 10.0, 1.0, 0.0, 0.5, kTauSpecNm), "");      // no slew
    EXPECT_NE(validate_limits(30.0, 10.0, 1.0, 0.05, 2.0, kTauSpecNm), "");
    auto over = kTauSpecNm;
    over[5] = 20.0;                                                              // wrist > 12
    EXPECT_NE(validate_limits(30.0, 10.0, 1.0, 0.05, 0.5, over), "");
}

TEST(ValidateSlew, AcceptsTheStrokeAndTrackingRates)
{
    EXPECT_EQ(validate_slew(0.005, 0.5), "");   // the insertion stroke
    EXPECT_EQ(validate_slew(0.10, 0.5), "");    // the tracking range, both ends
    EXPECT_EQ(validate_slew(0.25, 0.5), "");
}

TEST(ValidateSlew, RejectsSlewAboveTheHardBound)
{
    EXPECT_NE(validate_slew(ConfigLimits::slewMpsMax + 0.01, 0.5), "");
    EXPECT_NE(validate_slew(0.10, ConfigLimits::slewRpsMax + 0.1), "");
}

TEST(ValidateSlew, RejectsZeroOrNonFiniteSlew)
{
    // A zero or NaN cap freezes the equilibrium where it stands: the arm would
    // never reach a target it was told to follow.
    EXPECT_NE(validate_slew(0.0, 0.5), "");
    EXPECT_NE(validate_slew(0.10, 0.0), "");
    EXPECT_NE(validate_slew(kNan, 0.5), "");
    EXPECT_NE(validate_slew(0.10, kInf), "");
}

TEST(ValidateSlew, LiveSlewUsesTheSameBoundAsConfigure)
{
    // The live path must not admit anything configure refuses, or the panel
    // could raise the cap past the bound a reconfigure would have rejected.
    EXPECT_EQ(validate_slew(ConfigLimits::slewMpsMax, ConfigLimits::slewRpsMax), "");
    EXPECT_EQ(validate_limits(30.0, 10.0, 1.0, ConfigLimits::slewMpsMax,
                              ConfigLimits::slewRpsMax, kTauSpecNm), "");
}

// ---------------------------------------------------------------- joint wall

namespace
{
using fr3_mating_controllers::detail::joint_wall_torque;
using fr3_mating_controllers::detail::JointWall;
using fr3_mating_controllers::detail::validate_wall;
using Vec7 = Eigen::Matrix<double, 7, 1>;

JointWall fr3_wall()
{
    JointWall w;
    w.lower = {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};
    w.upper = {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};
    w.margin_rad = 0.0873;
    w.k = {600, 600, 600, 600, 150, 150, 150};
    w.d = {20, 20, 20, 20, 3, 3, 3};
    return w;
}

Vec7 mid(const JointWall &w)
{
    Vec7 q;
    for (int i = 0; i < 7; ++i) q(i) = 0.5 * (w.lower[i] + w.upper[i]);
    return q;
}
}  // namespace

TEST(JointWall, NothingOutsideTheMargin)
{
    const JointWall w = fr3_wall();
    Vec7 q = mid(w);
    q(0) = w.upper[0] - w.margin_rad - 1e-6;                // just outside
    EXPECT_TRUE(joint_wall_torque(q, Vec7::Constant(1.0), w).isZero());
}

TEST(JointWall, TheSpringPushesBackOutOfEitherMargin)
{
    const JointWall w = fr3_wall();
    Vec7 q = mid(w);
    q(1) = w.upper[1] - w.margin_rad + 0.035;               // 2 deg into the high margin
    q(5) = w.lower[5] + w.margin_rad - 0.02;                // into the low margin
    const Vec7 tau = joint_wall_torque(q, Vec7::Zero(), w);
    EXPECT_NEAR(tau(1), -600.0 * 0.035, 1e-9);              // -21 Nm: back down
    EXPECT_NEAR(tau(5), 150.0 * 0.02, 1e-9);                // +3 Nm: back up
    for (int i : {0, 2, 3, 4, 6}) EXPECT_EQ(tau(i), 0.0);
}

TEST(JointWall, DampsBothWaysInsideTheMarginRampedIn)
{
    // Both ways: one-way damping threw a released joint back out at ~80 % of
    // its entry speed (review 2026-10-02). Damping only ever removes energy.
    const JointWall w = fr3_wall();
    const double deep = 0.5 * w.margin_rad;
    Vec7 q = mid(w), dq = Vec7::Zero();
    q(0) = w.upper[0] - w.margin_rad + deep;
    dq(0) = 0.5;                                            // toward the limit
    EXPECT_NEAR(joint_wall_torque(q, dq, w)(0), -600.0 * deep - 20.0 * 0.5, 1e-9);
    dq(0) = -0.5;                                           // away: slowed too
    EXPECT_NEAR(joint_wall_torque(q, dq, w)(0), -600.0 * deep + 20.0 * 0.5, 1e-9);
    q(0) = w.upper[0] - w.margin_rad + 0.004;               // 0.004 of the 0.01 rad ramp
    dq(0) = 0.5;
    EXPECT_NEAR(joint_wall_torque(q, dq, w)(0), -600.0 * 0.004 - 20.0 * 0.5 * 0.4, 1e-9);
}

TEST(JointWall, TheEnvelopeIsTheFr3s)
{
    using fr3_mating_controllers::detail::fr3_allowed_speed;
    // J2, 5 deg (0.0873 rad) short of its 1.7837 limit: 0.50 rad/s
    // (-0.20 + sqrt(5.17 * (1.7918 - 1.6964))); the review's number too.
    EXPECT_NEAR(fr3_allowed_speed(1, 1.7837 - 0.0873, true), 0.502, 0.002);
    EXPECT_NEAR(fr3_allowed_speed(0, 0.0, true), 2.62, 1e-9);          // capped mid-range
    EXPECT_EQ(fr3_allowed_speed(3, -0.1458, true), 0.0);                // at J4's stop
    EXPECT_NEAR(fr3_allowed_speed(5, 0.54092 + 0.1, false),
                -0.35 + std::sqrt(11.0 * 0.1), 1e-9);                   // J6 toward its low stop
}

TEST(JointWall, BrakesOnlyPastTheSpeedEnvelopeAndOnlyTowardTheLimit)
{
    using fr3_mating_controllers::detail::fr3_allowed_speed;
    JointWall w = fr3_wall();
    w.margin_rad = 0.0;                                     // the brake alone
    w.speed_frac = 0.8;
    w.brake_d = {60, 60, 60, 60, 6, 6, 6};
    Vec7 q = mid(w), dq = Vec7::Zero();
    q(1) = 1.5;                                             // J2, 0.28 rad from its stop
    const double allow = 0.8 * fr3_allowed_speed(1, 1.5, true);
    dq(1) = allow - 0.01;                                   // under the envelope: nothing
    EXPECT_EQ(joint_wall_torque(q, dq, w)(1), 0.0);
    dq(1) = allow + 0.3;                                    // over it: braked by the excess
    EXPECT_NEAR(joint_wall_torque(q, dq, w)(1), -60.0 * 0.3, 1e-9);
    dq(1) = -(allow + 0.3);                                 // as fast, but away from the
    EXPECT_EQ(joint_wall_torque(q, dq, w)(1), 0.0);         // near stop: far from the other
    q = mid(w);
    dq = Vec7::Constant(1.0);                               // mid-range at 1 rad/s: nothing
    EXPECT_TRUE(joint_wall_torque(q, dq, w).isZero());
}

TEST(JointWall, MarginZeroAndFracZeroAreOff)
{
    JointWall w = fr3_wall();
    w.margin_rad = 0.0;
    w.speed_frac = 0.0;
    Vec7 q = mid(w);
    q(3) = w.upper[3] - 0.001;
    EXPECT_TRUE(joint_wall_torque(q, Vec7::Constant(1.0), w).isZero());
}

TEST(JointWall, ValidationRefusesWhatCouldNotBeAWall)
{
    EXPECT_EQ(validate_wall(fr3_wall()), "");
    JointWall w = fr3_wall();
    w.margin_rad = 0.5;
    EXPECT_NE(validate_wall(w), "");
    w = fr3_wall();
    w.k[2] = -1.0;
    EXPECT_NE(validate_wall(w), "");
    w = fr3_wall();
    w.d[6] = kNan;
    EXPECT_NE(validate_wall(w), "");
    w = fr3_wall();
    w.upper[4] = w.lower[4] + 0.1;                          // less than two margins apart
    EXPECT_NE(validate_wall(w), "");
    w = fr3_wall();
    w.speed_frac = 1.5;
    EXPECT_NE(validate_wall(w), "");
    w = fr3_wall();
    w.brake_d[0] = -1.0;
    EXPECT_NE(validate_wall(w), "");
}
