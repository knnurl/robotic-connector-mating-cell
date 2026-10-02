// Pure mating-sequence state machine. No ROS, no MoveIt - just the
// decision logic, extracted from mating_node.cpp so every transition is unit-
// testable (see test/test_phase_machine.cpp). The node owns time, poses,
// TF, planning and execution; this class owns WHAT to do next and WHICH
// phase follows from each outcome.
//
// Contract per control cycle:
//   1. build Inputs from the world (freshness flags, errors, requests)
//   2. act = machine.step(inputs)      -> one Action to perform
//   3. execute the action (plan/move/publish twist/...)
//   4. machine.note_result(outcome)    -> phase bookkeeping for that action
// Service handlers use reset_allowed() / phase() to refuse or accept.
#pragma once

#include <algorithm>

namespace mating_phase_machine
{

enum class Phase { WAIT_FOR_VISION, ALIGN_COARSE, ALIGN_FINE, INSERT, MATED, FAULT };

inline const char *phase_name(Phase p)
{
    switch (p) {
    case Phase::WAIT_FOR_VISION: return "WAIT_FOR_VISION";
    case Phase::ALIGN_COARSE:    return "ALIGN_COARSE";
    case Phase::ALIGN_FINE:      return "ALIGN_FINE";
    case Phase::INSERT:          return "INSERT";
    case Phase::MATED:           return "MATED";
    case Phase::FAULT:           return "FAULT";
    }
    return "?";
}

// What the node must do this cycle.
enum class Action {
    NONE,          // phase changed / nothing to execute this cycle
    HOLD,          // keep position (zero twist in servo mode)
    ALIGN_COARSE_STEP,
    ALIGN_FINE_STEP,
    COMMIT_INSERT,  // execute the committed stroke (remaining depth)
    RETRACT,        // pull back along the stroke (operator recovery)
};

// How an executed Action ended. HOLD/NONE need no report.
enum class Outcome {
    SUCCESS,
    FAILURE,         // planning or execution failed
    CONTACT_SEATED,  // stroke: contact force at/after min depth
    CONTACT_JAM,     // stroke: contact force before min depth
    LATERAL_ABORT,   // stroke: sideways load tripped the guard
    INTERRUPTED,     // stroke halted by operator pause/stop
};

struct Config
{
    double coarse_pos_tol_m{0.02};
    double coarse_rot_tol_deg{5.0};
    double fine_pos_tol_m{0.0025};
    double fine_rot_tol_deg{1.0};
    int align_hold_cycles{3};
    int max_consecutive_plan_failures{5};
};

struct Inputs
{
    // Operator requests (already consumed/exchanged by the node).
    bool stop_requested{false};
    bool reset_requested{false};
    bool retract_requested{false};
    bool paused{false};
    // World state.
    bool vision_fresh{false};   // filtered pose usable
    bool raw_fresh{false};      // an actual detection (not a prediction)
    bool goal_valid{false};     // TF resolved, errors below are meaningful
    double pos_err_m{0.0};
    double rot_err_deg{0.0};
    bool insertion_enabled{true};
};

class PhaseMachine
{
public:
    explicit PhaseMachine(const Config &config) : cfg_(config) {}

    Phase phase() const { return phase_; }
    int aligned_cycles() const { return aligned_cycles_; }
    int consecutive_plan_failures() const { return consecutive_plan_failures_; }
    // A FAULT latched while the stroke was under way: the tool may be
    // partially engaged, so reset is refused until a RETRACT succeeded.
    bool insert_interrupted() const { return insert_interrupted_; }
    bool reset_allowed() const { return !insert_interrupted_; }

    Action step(const Inputs &in)
    {
        last_action_ = decide(in);
        return last_action_;
    }

    // Report how the last COMMIT_INSERT / RETRACT / ALIGN_*_STEP went.
    void note_result(Outcome outcome)
    {
        switch (last_action_) {
        case Action::ALIGN_COARSE_STEP:
        case Action::ALIGN_FINE_STEP:
            note_plan_result(outcome == Outcome::SUCCESS);
            return;
        case Action::COMMIT_INSERT:
            note_insert_result(outcome);
            return;
        case Action::RETRACT:
            if (outcome == Outcome::SUCCESS) {
                insert_interrupted_ = false;
                aligned_cycles_ = 0;
                consecutive_plan_failures_ = 0;
                phase_ = Phase::WAIT_FOR_VISION;
            }
            return;  // failure: stay latched in FAULT
        default:
            return;
        }
    }

private:
    Action decide(const Inputs &in)
    {
        if (in.stop_requested) {
            if (phase_ == Phase::INSERT) {
                insert_interrupted_ = true;
            }
            phase_ = Phase::FAULT;
            return Action::HOLD;
        }

        if (in.retract_requested) {
            // Only valid in FAULT: it is the recovery path for an
            // interrupted stroke, not a general jog command.
            return phase_ == Phase::FAULT ? Action::RETRACT : Action::NONE;
        }

        if (in.reset_requested && reset_allowed()) {
            aligned_cycles_ = 0;
            consecutive_plan_failures_ = 0;
            phase_ = Phase::WAIT_FOR_VISION;
            // fall through: the fresh sequence may act this same cycle
        }

        if (in.paused) {
            aligned_cycles_ = 0;
            return Action::HOLD;
        }

        if (phase_ == Phase::MATED || phase_ == Phase::FAULT) {
            return Action::HOLD;
        }

        if (!in.vision_fresh) {
            aligned_cycles_ = 0;
            return Action::HOLD;
        }

        if (!in.goal_valid) {
            return Action::HOLD;
        }

        if (phase_ == Phase::WAIT_FOR_VISION) {
            phase_ = Phase::ALIGN_COARSE;  // and act on it this same cycle
        }

        switch (phase_) {
        case Phase::ALIGN_COARSE:
            if (in.pos_err_m < cfg_.coarse_pos_tol_m &&
                in.rot_err_deg < cfg_.coarse_rot_tol_deg) {
                phase_ = Phase::ALIGN_FINE;
                return Action::NONE;  // fine logic starts next cycle
            }
            return Action::ALIGN_COARSE_STEP;

        case Phase::ALIGN_FINE:
            if (in.pos_err_m < cfg_.fine_pos_tol_m &&
                in.rot_err_deg < cfg_.fine_rot_tol_deg) {
                if (!in.raw_fresh) {
                    // Predictions may steer, never arm the stroke.
                    return Action::HOLD;
                }
                if (++aligned_cycles_ >= cfg_.align_hold_cycles) {
                    if (in.insertion_enabled) {
                        phase_ = Phase::INSERT;  // commits next cycle
                        return Action::NONE;
                    }
                    // Teach mode: aligned, hovering, reporting.
                }
                return Action::HOLD;
            }
            aligned_cycles_ = 0;
            if (in.pos_err_m > cfg_.coarse_pos_tol_m * 2.0) {
                phase_ = Phase::ALIGN_COARSE;  // target drifted away
                return Action::NONE;
            }
            return Action::ALIGN_FINE_STEP;

        case Phase::INSERT:
            return Action::COMMIT_INSERT;

        default:
            return Action::HOLD;
        }
    }

    void note_plan_result(bool ok)
    {
        if (ok) {
            consecutive_plan_failures_ = 0;
            return;
        }
        if (++consecutive_plan_failures_ >= cfg_.max_consecutive_plan_failures) {
            phase_ = Phase::FAULT;
        }
    }

    void note_insert_result(Outcome outcome)
    {
        switch (outcome) {
        case Outcome::SUCCESS:
        case Outcome::CONTACT_SEATED:
            phase_ = Phase::MATED;
            return;
        case Outcome::CONTACT_JAM:
        case Outcome::LATERAL_ABORT:
            insert_interrupted_ = true;
            phase_ = Phase::FAULT;
            return;
        case Outcome::INTERRUPTED:
            // Operator halted the stroke: stay in INSERT so a resume
            // covers only the remaining depth; a stop latches FAULT via
            // stop_requested next cycle.
            return;
        case Outcome::FAILURE:
            aligned_cycles_ = 0;
            note_plan_result(false);
            if (phase_ == Phase::FAULT) {
                // Plan-failure limit hit mid-stroke.
                insert_interrupted_ = true;
            } else {
                phase_ = Phase::ALIGN_FINE;  // re-verify alignment first
            }
            return;
        }
    }

    Config cfg_;
    Phase phase_{Phase::WAIT_FOR_VISION};
    Action last_action_{Action::NONE};
    int aligned_cycles_{0};
    int consecutive_plan_failures_{0};
    bool insert_interrupted_{false};
};

}  // namespace mating_phase_machine
