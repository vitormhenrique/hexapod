// Native replay execution tests for portable ControllerCore.
// Run with: pio test -e native -f test_controller_replay

#include <unity.h>

#include "../fixtures/controller_arm_walk_estop_replay.h"

void test_arm_walk_estop_fixture_passes() {
  const controller::replay::Result result =
      controller::replay::run(
          controller::replay::fixtures::armWalkEstop());

  TEST_ASSERT_TRUE(result.passed);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(controller::replay::Failure::None),
      static_cast<uint8_t>(result.failure));
  TEST_ASSERT_EQUAL_UINT8(8, result.frame_index);
}

void test_semantic_fixture_mismatch_identifies_frame_and_field() {
  controller::replay::Fixture corrupted =
      controller::replay::fixtures::armWalkEstop();
  corrupted.frames[6].expected.goal_valid = false;

  const controller::replay::Result result = controller::replay::run(corrupted);

  TEST_ASSERT_FALSE(result.passed);
  TEST_ASSERT_EQUAL_UINT8(6, result.frame_index);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(controller::replay::Failure::GoalValidity),
      static_cast<uint8_t>(result.failure));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_arm_walk_estop_fixture_passes);
  RUN_TEST(test_semantic_fixture_mismatch_identifies_frame_and_field);
  return UNITY_END();
}