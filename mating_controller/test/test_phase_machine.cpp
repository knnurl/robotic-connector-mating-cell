// Unit tests for the pure mating phase machine - the full transition
// matrix, including the guards that previously lived untested inside the
// ROS node: raw-detection arming, interrupted-insert latching, retract
// recovery, drift-back, plan-failure accounting, and force outcomes.
#include <gtest/gtest.h>

#include "mating_controller/mating_phase_machine.hpp"

using mating_phase_machine::Action;
using mating_phase_machine::Config;
using mating_phase_machine::Inputs;
using mating_phase_machine::Outcome;
using mating_phase_machine::Phase;
using mating_phase_machine::PhaseMachine;

namespace
{

Config small_cfg()
{
    Config c;
    c.align_hold_cycles = 3;
    c.max_consecutive_plan_failures = 5;
    return c;
}

// Fresh vision + valid goal at a given error.
Inputs seeing(double pos_err_m, double rot_err_deg, bool raw = true)
{
    Inputs in;
    in.vision_fresh = true;
    in.raw_fresh = raw;
    in.goal_valid = true;
    in.pos_err_m = pos_err_m;
    in.rot_err_deg = rot_err_deg;
    return in;
}

// Drive a fresh machine to the brink of insertion and commit it.
void drive_to_insert(PhaseMachine &m)
{
    EXPECT_EQ(m.step(seeing(0.10, 10.0)), Action::ALIGN_COARSE_STEP);
    m.note_result(Outcome::SUCCESS);
    EXPECT_EQ(m.step(seeing(0.01, 2.0)), Action::NONE);  // -> ALIGN_FINE
    EXPECT_EQ(m.step(seeing(0.005, 1.2)), Action::ALIGN_FINE_STEP);
    m.note_result(Outcome::SUCCESS);
    for (int i = 0; i < 2; ++i) {
        EXPECT_EQ(m.step(seeing(0.001, 0.5)), Action::HOLD);  // arming counts
    }
    EXPECT_EQ(m.step(seeing(0.001, 0.5)), Action::NONE);  // armed -> INSERT
    ASSERT_EQ(m.phase(), Phase::INSERT);
}

}  // namespace

TEST(PhaseMachine, HappyPathToMated)
{
    PhaseMachine m(small_cfg());
    EXPECT_EQ(m.phase(), Phase::WAIT_FOR_VISION);
    drive_to_insert(m);
    EXPECT_EQ(m.step(seeing(0.001, 0.5)), Action::COMMIT_INSERT);
    m.note_result(Outcome::SUCCESS);
    EXPECT_EQ(m.phase(), Phase::MATED);
    // Latched: nothing further is commanded.
    EXPECT_EQ(m.step(seeing(0.10, 10.0)), Action::HOLD);
    EXPECT_EQ(m.phase(), Phase::MATED);
}

TEST(PhaseMachine, StaleVisionHoldsAndResetsArming)
{
    PhaseMachine m(small_cfg());
    drive_to_insert(m);
    // (arming counter check needs a fresh machine mid-way; rebuild)
    PhaseMachine m2(small_cfg());
    EXPECT_EQ(m2.step(seeing(0.10, 10.0)), Action::ALIGN_COARSE_STEP);
    m2.note_result(Outcome::SUCCESS);
    EXPECT_EQ(m2.step(seeing(0.01, 2.0)), Action::NONE);
    EXPECT_EQ(m2.step(seeing(0.001, 0.5)), Action::HOLD);  // count = 1
    EXPECT_EQ(m2.aligned_cycles(), 1);

    Inputs blind;  // no vision at all
    EXPECT_EQ(m2.step(blind), Action::HOLD);
    EXPECT_EQ(m2.aligned_cycles(), 0);        // arming restarts
    EXPECT_EQ(m2.phase(), Phase::ALIGN_FINE);  // phase is kept
}

TEST(PhaseMachine, PredictionsSteerButNeverArm)
{
    PhaseMachine m(small_cfg());
    EXPECT_EQ(m.step(seeing(0.10, 10.0)), Action::ALIGN_COARSE_STEP);
    m.note_result(Outcome::SUCCESS);
    EXPECT_EQ(m.step(seeing(0.01, 2.0)), Action::NONE);
    // In tolerance forever on prediction-only poses: count must never move.
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(m.step(seeing(0.001, 0.5, /*raw=*/false)), Action::HOLD);
        EXPECT_EQ(m.aligned_cycles(), 0);
    }
    EXPECT_EQ(m.phase(), Phase::ALIGN_FINE);
    // Predictions may still steer a fine step when out of tolerance.
    EXPECT_EQ(m.step(seeing(0.005, 1.2, /*raw=*/false)), Action::ALIGN_FINE_STEP);
}

TEST(PhaseMachine, TeachModeHoversWithoutInserting)
{
    PhaseMachine m(small_cfg());
    EXPECT_EQ(m.step(seeing(0.01, 2.0)), Action::NONE);  // straight to FINE
    Inputs in = seeing(0.001, 0.5);
    in.insertion_enabled = false;
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(m.step(in), Action::HOLD);
    }
    EXPECT_EQ(m.phase(), Phase::ALIGN_FINE);  // never INSERT
}

TEST(PhaseMachine, DriftBackToCoarse)
{
    PhaseMachine m(small_cfg());
    EXPECT_EQ(m.step(seeing(0.01, 2.0)), Action::NONE);  // -> FINE
    EXPECT_EQ(m.step(seeing(0.001, 0.5)), Action::HOLD);
    EXPECT_EQ(m.aligned_cycles(), 1);
    // Target jumps far away (operator moved the jig).
    EXPECT_EQ(m.step(seeing(0.05, 1.0)), Action::NONE);
    EXPECT_EQ(m.phase(), Phase::ALIGN_COARSE);
    EXPECT_EQ(m.aligned_cycles(), 0);
}

TEST(PhaseMachine, RepeatedPlanFailuresLatchFault)
{
    PhaseMachine m(small_cfg());
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(m.step(seeing(0.10, 10.0)), Action::ALIGN_COARSE_STEP);
        m.note_result(Outcome::FAILURE);
    }
    EXPECT_EQ(m.phase(), Phase::FAULT);
    EXPECT_TRUE(m.reset_allowed());  // no stroke was under way
    EXPECT_EQ(m.step(seeing(0.10, 10.0)), Action::HOLD);
}

TEST(PhaseMachine, PlanFailureCounterClearsOnSuccess)
{
    PhaseMachine m(small_cfg());
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(m.step(seeing(0.10, 10.0)), Action::ALIGN_COARSE_STEP);
        m.note_result(Outcome::FAILURE);
    }
    m.step(seeing(0.10, 10.0));
    m.note_result(Outcome::SUCCESS);
    EXPECT_EQ(m.consecutive_plan_failures(), 0);
    EXPECT_NE(m.phase(), Phase::FAULT);
}

TEST(PhaseMachine, StopDuringInsertRefusesResetUntilRetract)
{
    PhaseMachine m(small_cfg());
    drive_to_insert(m);

    Inputs stop = seeing(0.001, 0.5);
    stop.stop_requested = true;
    EXPECT_EQ(m.step(stop), Action::HOLD);
    EXPECT_EQ(m.phase(), Phase::FAULT);
    EXPECT_TRUE(m.insert_interrupted());
    EXPECT_FALSE(m.reset_allowed());

    // Reset must not clear the latch while interrupted.
    Inputs reset = seeing(0.001, 0.5);
    reset.reset_requested = true;
    m.step(reset);
    EXPECT_EQ(m.phase(), Phase::FAULT);

    // Retract failure keeps the latch...
    Inputs retract;
    retract.retract_requested = true;
    EXPECT_EQ(m.step(retract), Action::RETRACT);
    m.note_result(Outcome::FAILURE);
    EXPECT_EQ(m.phase(), Phase::FAULT);
    EXPECT_FALSE(m.reset_allowed());

    // ...retract success clears it and restarts the sequence.
    EXPECT_EQ(m.step(retract), Action::RETRACT);
    m.note_result(Outcome::SUCCESS);
    EXPECT_EQ(m.phase(), Phase::WAIT_FOR_VISION);
    EXPECT_TRUE(m.reset_allowed());
}

TEST(PhaseMachine, RetractOnlyAvailableInFault)
{
    PhaseMachine m(small_cfg());
    Inputs retract;
    retract.retract_requested = true;
    EXPECT_EQ(m.step(retract), Action::NONE);
    EXPECT_EQ(m.phase(), Phase::WAIT_FOR_VISION);
}

TEST(PhaseMachine, ContactOutcomes)
{
    // Seated by force -> MATED.
    PhaseMachine seated(small_cfg());
    drive_to_insert(seated);
    EXPECT_EQ(seated.step(seeing(0.001, 0.5)), Action::COMMIT_INSERT);
    seated.note_result(Outcome::CONTACT_SEATED);
    EXPECT_EQ(seated.phase(), Phase::MATED);

    // Early contact = jam -> FAULT with the interrupted latch.
    PhaseMachine jam(small_cfg());
    drive_to_insert(jam);
    jam.step(seeing(0.001, 0.5));
    jam.note_result(Outcome::CONTACT_JAM);
    EXPECT_EQ(jam.phase(), Phase::FAULT);
    EXPECT_FALSE(jam.reset_allowed());

    // Sideways snag -> same latch.
    PhaseMachine snag(small_cfg());
    drive_to_insert(snag);
    snag.step(seeing(0.001, 0.5));
    snag.note_result(Outcome::LATERAL_ABORT);
    EXPECT_EQ(snag.phase(), Phase::FAULT);
    EXPECT_FALSE(snag.reset_allowed());
}

TEST(PhaseMachine, InterruptedStrokeStaysInInsertForResume)
{
    PhaseMachine m(small_cfg());
    drive_to_insert(m);
    EXPECT_EQ(m.step(seeing(0.001, 0.5)), Action::COMMIT_INSERT);
    m.note_result(Outcome::INTERRUPTED);  // pause mid-stroke
    EXPECT_EQ(m.phase(), Phase::INSERT);

    Inputs paused = seeing(0.001, 0.5);
    paused.paused = true;
    EXPECT_EQ(m.step(paused), Action::HOLD);
    // Resume: the stroke recommits (node handles remaining depth).
    EXPECT_EQ(m.step(seeing(0.001, 0.5)), Action::COMMIT_INSERT);
}

TEST(PhaseMachine, InsertPlanFailureDropsToFineRealign)
{
    PhaseMachine m(small_cfg());
    drive_to_insert(m);
    m.step(seeing(0.001, 0.5));
    m.note_result(Outcome::FAILURE);
    EXPECT_EQ(m.phase(), Phase::ALIGN_FINE);
    EXPECT_EQ(m.aligned_cycles(), 0);
    EXPECT_TRUE(m.reset_allowed());  // no engagement implied by a plan failure
}

TEST(PhaseMachine, PauseHoldsAndResetsArmingButKeepsPhase)
{
    PhaseMachine m(small_cfg());
    EXPECT_EQ(m.step(seeing(0.01, 2.0)), Action::NONE);  // -> FINE
    m.step(seeing(0.001, 0.5));
    EXPECT_EQ(m.aligned_cycles(), 1);
    Inputs paused = seeing(0.001, 0.5);
    paused.paused = true;
    EXPECT_EQ(m.step(paused), Action::HOLD);
    EXPECT_EQ(m.aligned_cycles(), 0);
    EXPECT_EQ(m.phase(), Phase::ALIGN_FINE);
}

TEST(PhaseMachine, ResetRestartsSequenceWhenAllowed)
{
    PhaseMachine m(small_cfg());
    drive_to_insert(m);
    m.step(seeing(0.001, 0.5));
    m.note_result(Outcome::SUCCESS);
    ASSERT_EQ(m.phase(), Phase::MATED);

    Inputs reset = seeing(0.10, 10.0);
    reset.reset_requested = true;
    // Same cycle: sequence restarts AND acts on the fresh vision.
    EXPECT_EQ(m.step(reset), Action::ALIGN_COARSE_STEP);
    EXPECT_EQ(m.phase(), Phase::ALIGN_COARSE);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
