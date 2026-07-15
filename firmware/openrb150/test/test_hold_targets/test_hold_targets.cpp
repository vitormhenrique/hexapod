#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/dxl/hold_targets.h"

using namespace dxl;

namespace {

ServoStatus status(uint8_t id, int32_t tick, bool ok = true) {
  ServoStatus out;
  out.id = id;
  out.present_position = tick;
  out.ok = ok;
  return out;
}

}  // namespace

void test_complete_reordered_snapshot_builds_hold_targets() {
  const uint8_t ids[] = {1, 2, 3};
  const ServoStatus statuses[] = {
      status(3, 3000), status(1, 1000), status(2, 2000)};
  GoalTarget out[3];
  TEST_ASSERT_TRUE(buildHoldTargets(ids, 3, statuses, 3, out, 3));
  TEST_ASSERT_EQUAL_UINT8(1, out[0].id);
  TEST_ASSERT_EQUAL_UINT16(1000, out[0].tick);
  TEST_ASSERT_EQUAL_UINT8(2, out[1].id);
  TEST_ASSERT_EQUAL_UINT16(2000, out[1].tick);
  TEST_ASSERT_EQUAL_UINT8(3, out[2].id);
  TEST_ASSERT_EQUAL_UINT16(3000, out[2].tick);
}

void test_missing_or_stale_servo_rejects_entire_set() {
  const uint8_t ids[] = {1, 2};
  GoalTarget out[2];
  ServoStatus statuses[] = {status(1, 1000), status(2, 2000, false)};
  TEST_ASSERT_FALSE(buildHoldTargets(ids, 2, statuses, 2, out, 2));
  TEST_ASSERT_FALSE(buildHoldTargets(ids, 2, statuses, 1, out, 2));
}

void test_out_of_range_present_position_rejects_entire_set() {
  const uint8_t ids[] = {1};
  GoalTarget out[1];
  ServoStatus statuses[] = {status(1, -1)};
  TEST_ASSERT_FALSE(buildHoldTargets(ids, 1, statuses, 1, out, 1));
  statuses[0] = status(1, config::kServoMaxTick + 1);
  TEST_ASSERT_FALSE(buildHoldTargets(ids, 1, statuses, 1, out, 1));
  statuses[0] = status(1, config::kServoMaxTick);
  TEST_ASSERT_TRUE(buildHoldTargets(ids, 1, statuses, 1, out, 1));
}

void test_partial_output_capacity_rejects_entire_set() {
  const uint8_t ids[] = {1, 2};
  const ServoStatus statuses[] = {status(1, 1000), status(2, 2000)};
  GoalTarget out[1];
  TEST_ASSERT_FALSE(buildHoldTargets(ids, 2, statuses, 2, out, 1));
}

void test_torque_off_confirmation_requires_coverage_only_while_powered() {
  TEST_ASSERT_TRUE(torqueOffConfirmed(false, false, false));
  TEST_ASSERT_FALSE(torqueOffConfirmed(true, false, true));
  TEST_ASSERT_FALSE(torqueOffConfirmed(true, true, false));
  TEST_ASSERT_TRUE(torqueOffConfirmed(true, true, true));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_complete_reordered_snapshot_builds_hold_targets);
  RUN_TEST(test_missing_or_stale_servo_rejects_entire_set);
  RUN_TEST(test_out_of_range_present_position_rejects_entire_set);
  RUN_TEST(test_partial_output_capacity_rejects_entire_set);
  RUN_TEST(test_torque_off_confirmation_requires_coverage_only_while_powered);
  return UNITY_END();
}