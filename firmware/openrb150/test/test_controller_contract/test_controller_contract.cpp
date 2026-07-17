// Native contract tests for the portable ControllerCore boundary.
// Run with: pio test -e native -f test_controller_contract

#include <type_traits>

#include <unity.h>

#include "../../src/controller/controller_contract.h"

namespace {

using controller::ControllerConfigSnapshot;
using controller::ControllerIntent;
using controller::ControllerStepInput;
using controller::ControllerTime;
using controller::RobotCommand;
using controller::RobotState;
using controller::SnapshotValidity;

static_assert(std::is_copy_constructible<RobotState>::value,
              "RobotState must be a replayable value snapshot");
static_assert(std::is_copy_constructible<ControllerIntent>::value,
              "ControllerIntent must be a replayable value snapshot");
static_assert(std::is_copy_constructible<ControllerConfigSnapshot>::value,
              "ControllerConfigSnapshot must be a replayable value snapshot");
static_assert(std::is_copy_constructible<RobotCommand>::value,
              "RobotCommand must be a replayable value result");

void acceptsPlannedStepSignature(const RobotState& state,
                                 const ControllerIntent& intent,
                                 const ControllerConfigSnapshot& config,
                                 const ControllerTime& time,
                                 RobotCommand& command) {
  (void)state;
  (void)intent;
  (void)config;
  (void)time;
  command.goal_valid = false;
}

}  // namespace

void test_contract_defaults_are_fail_closed() {
  ControllerStepInput input;
  RobotCommand command;

  TEST_ASSERT_FALSE(input.time.valid);
  TEST_ASSERT_FALSE(input.state.config_ready);
  TEST_ASSERT_TRUE(input.intent.rc.kill);
  TEST_ASSERT_TRUE(input.intent.rc.failsafe);
  TEST_ASSERT_FALSE(input.intent.maintenance.lock_held);
  TEST_ASSERT_FALSE(input.config.valid);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(SnapshotValidity::Unknown),
      static_cast<uint8_t>(input.state.dxl.validity));
  TEST_ASSERT_EQUAL_UINT8(0, input.state.dxl.servo_count);
  TEST_ASSERT_FALSE(command.goal_valid);
  TEST_ASSERT_FALSE(command.allow_dxl_power);
  TEST_ASSERT_FALSE(command.allow_torque);
}

void test_contract_arrays_are_fixed_to_robot_capacities() {
  RobotState state;
  RobotCommand command;

  TEST_ASSERT_EQUAL_UINT32(
      sizeof(dxl::ServoStatus) * config::kNumServos,
      sizeof(state.dxl.servos));
  TEST_ASSERT_EQUAL_UINT32(
      sizeof(sensors::LegContactState) * sensors::kNumFeet,
      sizeof(state.contact.feet));
  TEST_ASSERT_EQUAL_UINT32(
      sizeof(gait::PipelineJoint) * config::kNumServos,
      sizeof(command.goals.joints));
}

void test_contract_supports_const_deterministic_step_inputs() {
  ControllerStepInput input;
  input.time.now_ms = 100;
  input.time.dt_ms = 10;
  input.time.valid = true;
  input.config.valid = true;
  input.config.revision = 7;
  input.intent.motion.seq = 9;

  RobotCommand command;
  acceptsPlannedStepSignature(input.state, input.intent, input.config,
                              input.time, command);

  TEST_ASSERT_TRUE(input.time.valid);
  TEST_ASSERT_EQUAL_UINT32(10, input.time.dt_ms);
  TEST_ASSERT_EQUAL_UINT32(7, input.config.revision);
  TEST_ASSERT_EQUAL_UINT32(9, input.intent.motion.seq);
  TEST_ASSERT_FALSE(command.goal_valid);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_contract_defaults_are_fail_closed);
  RUN_TEST(test_contract_arrays_are_fixed_to_robot_capacities);
  RUN_TEST(test_contract_supports_const_deterministic_step_inputs);
  return UNITY_END();
}