// The two pieces of the impedance controller that decide safety without any
// robot state: which live gains are acceptable, and how a setpoint reaches the
// 1 kHz loop. Runs without franka, ROS or a controller manager.
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

using fr3_mating_controllers::detail::GainLimits;
using fr3_mating_controllers::detail::kTauSpecNm;
using fr3_mating_controllers::detail::TargetHandoff;
using fr3_mating_controllers::detail::validate_gains;
using fr3_mating_controllers::detail::validate_limits;

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
