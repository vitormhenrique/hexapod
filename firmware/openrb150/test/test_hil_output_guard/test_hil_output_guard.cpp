#include <unity.h>

#include "hil/output_guard.h"

namespace {

void test_output_disabled_blocks_all_actuating_operations() {
  hil::OutputGuard guard(true);

  TEST_ASSERT_FALSE(guard.allowPowerEnable());
  TEST_ASSERT_FALSE(guard.allowTorque(true));
  TEST_ASSERT_TRUE(guard.allowTorque(false));
  TEST_ASSERT_FALSE(guard.allowGoalWrite(2));
  guard.recordBlockedGoal(0, 1, 2048);
  guard.recordBlockedGoal(1, 2, 3072);
  guard.finishBlockedGoalWrite();
  TEST_ASSERT_FALSE(guard.allowDxlWrite());

  const hil::OutputGuardStatus status = guard.status();
  TEST_ASSERT_TRUE(status.output_disabled);
  TEST_ASSERT_TRUE(status.power_guard_active);
  TEST_ASSERT_TRUE(status.torque_guard_active);
  TEST_ASSERT_TRUE(status.goal_guard_active);
  TEST_ASSERT_TRUE(status.write_guard_active);
  TEST_ASSERT_EQUAL_UINT32(1, status.blocked_power_enable);
  TEST_ASSERT_EQUAL_UINT32(1, status.blocked_torque_enable);
  TEST_ASSERT_EQUAL_UINT32(1, status.blocked_goal_write);
  TEST_ASSERT_EQUAL_UINT32(1, status.blocked_dxl_write);
  TEST_ASSERT_EQUAL_UINT32(1, status.last_goal_sequence);
  TEST_ASSERT_EQUAL_UINT8(2, status.last_goal_count);

  hil::GoalTargetRecord goals[2];
  TEST_ASSERT_EQUAL_UINT8(2, guard.copyLastBlockedGoals(goals, 2));
  TEST_ASSERT_EQUAL_UINT8(1, goals[0].id);
  TEST_ASSERT_EQUAL_INT32(2048, goals[0].tick);
  TEST_ASSERT_EQUAL_UINT8(2, goals[1].id);
  TEST_ASSERT_EQUAL_INT32(3072, goals[1].tick);
}

void test_normal_build_guard_leaves_output_paths_available() {
  hil::OutputGuard guard(false);

  TEST_ASSERT_TRUE(guard.allowPowerEnable());
  TEST_ASSERT_TRUE(guard.allowTorque(true));
  TEST_ASSERT_TRUE(guard.allowTorque(false));
  TEST_ASSERT_TRUE(guard.allowGoalWrite(1));
  TEST_ASSERT_TRUE(guard.allowDxlWrite());

  const hil::OutputGuardStatus status = guard.status();
  TEST_ASSERT_FALSE(status.output_disabled);
  TEST_ASSERT_EQUAL_UINT32(0, status.blocked_power_enable);
  TEST_ASSERT_EQUAL_UINT32(0, status.blocked_torque_enable);
  TEST_ASSERT_EQUAL_UINT32(0, status.blocked_goal_write);
  TEST_ASSERT_EQUAL_UINT32(0, status.blocked_dxl_write);
  TEST_ASSERT_EQUAL_UINT32(0, status.last_goal_sequence);
  TEST_ASSERT_EQUAL_UINT8(0, status.last_goal_count);
}

void test_reset_preserves_mode_and_clears_counters() {
  hil::OutputGuard guard(true);
  guard.allowPowerEnable();
  guard.allowTorque(true);
  guard.allowGoalWrite(1);
  guard.recordBlockedGoal(0, 9, 1024);
  guard.finishBlockedGoalWrite();
  guard.allowDxlWrite();

  guard.reset();

  const hil::OutputGuardStatus status = guard.status();
  TEST_ASSERT_TRUE(status.output_disabled);
  TEST_ASSERT_EQUAL_UINT32(0, status.blocked_power_enable);
  TEST_ASSERT_EQUAL_UINT32(0, status.blocked_torque_enable);
  TEST_ASSERT_EQUAL_UINT32(0, status.blocked_goal_write);
  TEST_ASSERT_EQUAL_UINT32(0, status.blocked_dxl_write);
  TEST_ASSERT_EQUAL_UINT32(0, status.last_goal_sequence);
  TEST_ASSERT_EQUAL_UINT8(0, status.last_goal_count);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_output_disabled_blocks_all_actuating_operations);
  RUN_TEST(test_normal_build_guard_leaves_output_paths_available);
  RUN_TEST(test_reset_preserves_mode_and_clears_counters);
  return UNITY_END();
}