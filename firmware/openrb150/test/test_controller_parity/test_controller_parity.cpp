// Native controller trace parity tests.
// Run with: pio test -e native -f test_controller_parity

#include <unity.h>

#include "../fixtures/controller_arm_walk_estop_replay.h"
#include "../support/controller_parity.h"

namespace {

controller::parity::Trace makeSyntheticTrace() {
  const controller::replay::Fixture& replay =
      controller::replay::fixtures::armWalkEstop();
  controller::parity::Trace trace;
  trace.header.origin = controller::parity::CaptureOrigin::Synthetic;
  trace.header.frame_count = replay.header.frame_count;
  trace.header.nominal_period_ms = 10;

  controller::ControllerCore target_core;
  controller::RobotCommand target_command;
  for (uint8_t index = 0; index < trace.header.frame_count; ++index) {
    const controller::replay::ReplayFrame& source = replay.frames[index];
    controller::parity::TraceFrame& destination = trace.frames[index];
    destination.input.state = source.state;
    destination.input.intent = source.intent;
    destination.input.config = source.config;
    destination.input.time = source.time;
    target_core.step(destination.input.state, destination.input.intent,
                     destination.input.config, destination.input.time,
                     target_command);
    destination.observed = controller::parity::capture(target_command);
  }
  return trace;
}

}  // namespace

void test_synthetic_trace_replays_with_exact_controller_parity() {
  const controller::parity::Result result =
      controller::parity::replayAndCompare(makeSyntheticTrace());

  TEST_ASSERT_TRUE(result.passed);
  TEST_ASSERT_EQUAL_UINT8(8, result.frame_index);
  TEST_ASSERT_EQUAL_UINT8(0xFF, result.goal_index);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(controller::parity::Failure::None),
      static_cast<uint8_t>(result.failure));
}

void test_goal_tick_mismatch_reports_frame_joint_and_field() {
  controller::parity::Trace trace = makeSyntheticTrace();
  TEST_ASSERT_GREATER_THAN_UINT8(0, trace.frames[6].observed.goal_count);
  ++trace.frames[6].observed.goals[0].tick;

  const controller::parity::Result result =
      controller::parity::replayAndCompare(trace);

  TEST_ASSERT_FALSE(result.passed);
  TEST_ASSERT_EQUAL_UINT8(6, result.frame_index);
  TEST_ASSERT_EQUAL_UINT8(0, result.goal_index);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(controller::parity::Failure::GoalTick),
      static_cast<uint8_t>(result.failure));
}

    void test_invalid_trace_metadata_and_timing_are_rejected() {
      controller::parity::Trace bad_origin = makeSyntheticTrace();
      bad_origin.header.origin =
        static_cast<controller::parity::CaptureOrigin>(0xFF);
      controller::parity::Result result =
        controller::parity::replayAndCompare(bad_origin);

      TEST_ASSERT_FALSE(result.passed);
      TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(controller::parity::Failure::HeaderOrigin),
        static_cast<uint8_t>(result.failure));

      controller::parity::Trace bad_timing = makeSyntheticTrace();
      bad_timing.frames[0].input.time.valid = true;
      bad_timing.frames[0].input.time.dt_ms =
        controller::kDefaultMaxControllerElapsedMs + 1;
      result = controller::parity::replayAndCompare(bad_timing);

      TEST_ASSERT_FALSE(result.passed);
      TEST_ASSERT_EQUAL_UINT8(0, result.frame_index);
      TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(controller::parity::Failure::InputTiming),
        static_cast<uint8_t>(result.failure));
    }

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_synthetic_trace_replays_with_exact_controller_parity);
  RUN_TEST(test_goal_tick_mismatch_reports_frame_joint_and_field);
      RUN_TEST(test_invalid_trace_metadata_and_timing_are_rejected);
  return UNITY_END();
}