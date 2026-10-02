// Pure, ROS-free pieces of the impedance controller that decide safety: the
// limits on the gains and the slew caps, and the setpoint handoff between the
// subscription thread and the 1 kHz loop. Header-only and dependent on
// nothing but Eigen, so they are unit-tested without a robot
// (test/test_impedance_detail.cpp).
#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

#include <Eigen/Dense>

namespace fr3_mating_controllers
{
namespace detail
{

// Ranges for the gains that may change while the controller holds the arm.
// The force ceiling bounds HOW HARD the arm can push, not whether the loop is
// stable: damping_ratio 0 is an undamped spring and a negative value injects
// energy, so an out-of-range value is rejected outright, never clamped.
// tools/fr3/cell/core.py GAIN_LIMITS mirrors these; a test in
// tools/fr3/cell/test_cell_pins.py fails if the two drift apart.
struct GainLimits
{
    static constexpr double kPosMax = 3000.0;       // N/m
    static constexpr double kRotMax = 300.0;        // Nm/rad
    static constexpr double zetaMin = 0.1;
    static constexpr double zetaMax = 2.0;
    static constexpr double nullspaceMax = 50.0;    // Nm/rad
};

// FR3 joint torque limits (datasheet). tau_max_nm may be lower, never higher.
inline constexpr std::array<double, 7> kTauSpecNm{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

// Bounds on the safety parameters, and the hard bound on the slew caps
// whether they are set at configure time or live (validate_slew is the live
// path). A ceiling of 0 or NaN would silently disable the task law, and a
// torque rate above 1 Nm per cycle exceeds what FCI tolerates.
// tools/fr3/cell/test_cell_pins.py checks the shipped yaml against these
// numbers, so an edit there cannot quietly make configure fail on the day.
struct ConfigLimits
{
    static constexpr double maxForceMin = 1.0;      // N
    static constexpr double maxForceMax = 100.0;
    static constexpr double maxTorqueMin = 0.5;     // Nm
    static constexpr double maxTorqueMax = 30.0;
    static constexpr double tauRateMin = 0.01;      // Nm per 1 ms cycle
    static constexpr double tauRateMax = 1.0;
    static constexpr double slewMpsMin = 0.001;     // m/s
    static constexpr double slewMpsMax = 0.30;
    static constexpr double slewRpsMin = 0.001;     // rad/s
    static constexpr double slewRpsMax = 1.0;
    static constexpr double tauMaxMin = 0.1;        // Nm; the max is kTauSpecNm
};

inline bool finite_in(double v, double lo, double hi)
{
    return std::isfinite(v) && v >= lo && v <= hi;
}

// Empty if the equilibrium slew caps are acceptable, otherwise why they are
// not. Split out of validate_limits so the live parameter callback can reach
// the same bound without re-checking the four ceilings it cannot change.
inline std::string validate_slew(double slew_mps, double slew_rps)
{
    if (!finite_in(slew_mps, ConfigLimits::slewMpsMin, ConfigLimits::slewMpsMax)) {
        return "setpoint_slew_mps must be within [0.001, 0.30] m/s";
    }
    if (!finite_in(slew_rps, ConfigLimits::slewRpsMin, ConfigLimits::slewRpsMax)) {
        return "setpoint_slew_rps must be within [0.001, 1.0] rad/s";
    }
    return {};
}

// Empty if the configure-time safety parameters are acceptable, otherwise why
// they are not.
inline std::string validate_limits(double max_force_n, double max_torque_nm,
                                   double tau_rate_limit, double slew_mps, double slew_rps,
                                   const std::array<double, 7> &tau_max_nm)
{
    if (!finite_in(max_force_n, ConfigLimits::maxForceMin, ConfigLimits::maxForceMax)) {
        return "max_force_n must be within [1, 100] N";
    }
    if (!finite_in(max_torque_nm, ConfigLimits::maxTorqueMin, ConfigLimits::maxTorqueMax)) {
        return "max_torque_nm must be within [0.5, 30] Nm";
    }
    if (!finite_in(tau_rate_limit, ConfigLimits::tauRateMin, ConfigLimits::tauRateMax)) {
        return "tau_rate_limit must be within [0.01, 1.0] Nm per cycle (1.0 is the FCI limit)";
    }
    const std::string slew_error = validate_slew(slew_mps, slew_rps);
    if (!slew_error.empty()) {
        return slew_error;
    }
    for (std::size_t i = 0; i < tau_max_nm.size(); ++i) {
        if (!finite_in(tau_max_nm[i], ConfigLimits::tauMaxMin, kTauSpecNm[i])) {
            return "tau_max_nm[" + std::to_string(i) + "] must be within [0.1, " +
                   std::to_string(static_cast<int>(kTauSpecNm[i])) + "] Nm (the FR3 limit)";
        }
    }
    return {};
}

// Empty if the set is acceptable, otherwise why it is not.
inline std::string validate_gains(const Eigen::Vector3d &k_pos, const Eigen::Vector3d &k_rot,
                                  double damping_ratio, double nullspace_stiffness)
{
    for (int i = 0; i < 3; ++i) {
        if (!finite_in(k_pos(i), 0.0, GainLimits::kPosMax)) {
            return "k_pos_tool entries must be within [0, 3000] N/m";
        }
        if (!finite_in(k_rot(i), 0.0, GainLimits::kRotMax)) {
            return "k_rot_tool entries must be within [0, 300] Nm/rad";
        }
    }
    if (!finite_in(damping_ratio, GainLimits::zetaMin, GainLimits::zetaMax)) {
        return "damping_ratio must be within [0.1, 2.0]";
    }
    if (!finite_in(nullspace_stiffness, 0.0, GainLimits::nullspaceMax)) {
        return "nullspace_stiffness must be within [0, 50] Nm/rad";
    }
    return {};
}

// Setpoint handoff. The subscription thread publishes; the RT loop takes,
// and never waits on a lock the non-RT thread holds.
//
//   publish()     non-RT. Blocking lock; the sequence is bumped BEFORE the
//                 write, inside the lock.
//   try_take()    RT. try_lock only; on contention it reports nothing new and
//                 the loop keeps its previous target for one more cycle.
//   invalidate()  RT (or on_activate). Discards everything published so far -
//                 used when the equilibrium is re-seeded, so a setpoint from
//                 before the re-seed can never drag the arm back. Because the
//                 sequence moves before the write, a publish that straddles
//                 invalidate() is discarded too: losing a racing setpoint
//                 leaves the arm holding, adopting it could move the arm.
//                 Anything published after invalidate() returns still counts.
class TargetHandoff
{
public:
    void publish(const Eigen::Vector3d &p, const Eigen::Quaterniond &q)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        seq_.store(seq_.load() + 1);
        position_ = p;
        orientation_ = q;
    }

    bool try_take(Eigen::Vector3d &p, Eigen::Quaterniond &q)
    {
        if (seq_.load() == taken_) {
            return false;
        }
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            return false;
        }
        p = position_;
        q = orientation_;
        taken_ = seq_.load();
        return true;
    }

    void invalidate() { taken_ = seq_.load(); }

protected:  // not private: the gtest holds mutex_ to prove try_take never waits
    std::mutex mutex_;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    std::atomic<std::uint64_t> seq_{0};
    std::uint64_t taken_{0};  // owned by the taking (RT) thread
};

// The joint-limit wall, in every mode (FLOAT included): without it the only
// guard was the robot's own reflex, which stops the arm and takes the driver
// down. Two parts, added to the commanded torque before the joint torque
// clamp and the rate limiter (so the 12 Nm wrist can still be overpowered by
// hand; the reflex stays the backstop):
//  - inside the last margin_rad before a limit, a spring pushing the joint
//    back out, and damping on its motion BOTH ways (one-way damping threw a
//    released joint back out at ~80 % of its entry speed: review 2026-10-02),
//    ramped in over the first kDampRampRad of the margin (a fixed 0.01 rad:
//    joints stop 0.4-2 deg in, so a quarter of an 8 deg margin left the
//    damping mostly off); the 1 Nm/ms rate limiter smooths its onset;
//  - a speed brake: the FR3 limits joint speed by position (the envelope
//    below), and a joint arriving faster than it trips a velocity reflex
//    before any position wall acts. Motion toward a limit beyond speed_frac
//    of that envelope is braked by brake_d times the excess. Far from the
//    limits the envelope is 2.6-5.3 rad/s, so it never acts there.
struct JointWall
{
    std::array<double, 7> lower{};      // rad, the FR3 limits
    std::array<double, 7> upper{};
    double margin_rad = 0.0;            // 0 = off
    std::array<double, 7> k{};          // Nm/rad, spring per joint
    std::array<double, 7> d{};          // Nm s/rad, damping inside the margin
    double speed_frac = 0.0;            // brake beyond this share of the envelope; 0 = off
    std::array<double, 7> brake_d{};    // Nm s/rad per rad/s over it
};

// The FR3's position-dependent joint speed limits: libfranka rate_limiting.h
// computeUpper/LowerLimitsJointVelocity, from
// frankarobotics.github.io/docs/control_parameters.html (FR3). The allowed
// speed toward the upper end stop is min(vmax, -a + sqrt(c (q_hi - q))),
// toward the lower one the same with (q - q_lo).
struct Fr3SpeedEnvelope
{
    static constexpr std::array<double, 7> a{0.30, 0.20, 0.20, 0.30, 0.35, 0.35, 0.35};
    static constexpr std::array<double, 7> c{12.0, 5.17, 7.00, 8.00, 34.0, 11.0, 34.0};
    static constexpr std::array<double, 7> q_hi{2.75010, 1.79180, 2.90650, -0.1458,
                                                2.81010, 4.52050, 3.01960};
    static constexpr std::array<double, 7> q_lo{-2.75010, -1.79180, -2.90650, -3.04810,
                                                -2.81010, 0.54092, -3.01960};
    static constexpr std::array<double, 7> v_max{2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
};

inline constexpr double kDampRampRad = 0.01;   // the wall's damping is full this far in

// The speed (>= 0, rad/s) the FR3 allows joint i toward its upper (up) or
// lower end stop at position q.
inline double fr3_allowed_speed(int i, double q, bool up)
{
    using E = Fr3SpeedEnvelope;
    const double room = up ? E::q_hi[i] - q : q - E::q_lo[i];
    return std::min(E::v_max[i], std::max(0.0, -E::a[i] + std::sqrt(std::max(0.0, E::c[i] * room))));
}

struct WallLimits
{
    static constexpr double marginMax = 0.35;     // rad (20 deg)
    static constexpr double kMax = 2000.0;        // Nm/rad
    static constexpr double dMax = 50.0;          // Nm s/rad
    static constexpr double brakeDMax = 200.0;    // Nm s/rad
};

// Empty if the wall's parameters are acceptable, otherwise why not.
inline std::string validate_wall(const JointWall &w)
{
    if (!finite_in(w.margin_rad, 0.0, WallLimits::marginMax)) {
        return "joint_wall_margin_rad must be within [0, 0.35] rad";
    }
    for (std::size_t i = 0; i < 7; ++i) {
        if (!std::isfinite(w.lower[i]) || !std::isfinite(w.upper[i]) ||
            !(w.upper[i] - w.lower[i] > 2.0 * w.margin_rad)) {
            return "joint_lower/joint_upper[" + std::to_string(i) +
                   "] must be finite, with more than twice the margin between them";
        }
        if (!finite_in(w.k[i], 0.0, WallLimits::kMax) ||
            !finite_in(w.d[i], 0.0, WallLimits::dMax)) {
            return "joint_wall_k[" + std::to_string(i) + "] must be within [0, 2000] and "
                   "joint_wall_d within [0, 50]";
        }
        if (!finite_in(w.brake_d[i], 0.0, WallLimits::brakeDMax)) {
            return "joint_brake_d[" + std::to_string(i) + "] must be within [0, 200]";
        }
    }
    if (!finite_in(w.speed_frac, 0.0, 1.0)) {
        return "joint_speed_frac must be within [0, 1]";
    }
    return {};
}

// The wall's torque at joint positions q and velocities dq (rad, rad/s).
inline Eigen::Matrix<double, 7, 1> joint_wall_torque(const Eigen::Matrix<double, 7, 1> &q,
                                                     const Eigen::Matrix<double, 7, 1> &dq,
                                                     const JointWall &w)
{
    Eigen::Matrix<double, 7, 1> tau = Eigen::Matrix<double, 7, 1>::Zero();
    const double ramp = std::min(kDampRampRad, w.margin_rad);
    for (int i = 0; i < 7; ++i) {
        if (w.margin_rad > 0.0) {
            const double in_lo = (w.lower[i] + w.margin_rad) - q(i);   // > 0: in the low margin
            const double in_hi = q(i) - (w.upper[i] - w.margin_rad);   // > 0: in the high margin
            const double in = std::max(in_lo, in_hi);
            if (in > 0.0) {
                tau(i) += (in_lo > 0.0 ? 1.0 : -1.0) * w.k[i] * in;
                tau(i) -= w.d[i] * dq(i) * std::min(1.0, in / ramp);  // both ways: dissipative
            }
        }
        if (w.speed_frac > 0.0) {
            const bool up = dq(i) > 0.0;
            const double excess = std::abs(dq(i)) - w.speed_frac * fr3_allowed_speed(i, q(i), up);
            if (excess > 0.0) {
                tau(i) -= (up ? 1.0 : -1.0) * w.brake_d[i] * excess;
            }
        }
    }
    return tau;
}

}  // namespace detail
}  // namespace fr3_mating_controllers
