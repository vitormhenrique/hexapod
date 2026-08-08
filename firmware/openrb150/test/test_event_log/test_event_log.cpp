#include <string.h>
#include <unity.h>

#include "../../src/safety/event_log.h"

using namespace safety;

void test_event_line_is_human_readable() {
  PersistentEvent event;
  event.timestamp_ms = 1234;
  event.code = ErrorCode::BatteryLow;
  event.detail = 101;
  event.severity = ErrorSeverity::Warning;
  char line[96];
  const size_t len = formatEventLog(event, line, sizeof(line));
  TEST_ASSERT_GREATER_THAN(0, len);
  TEST_ASSERT_EQUAL_STRING(
      "T=1234 SEV=WARN CODE=BATTERY_LOW DETAIL=101\n", line);
}

void test_crash_line_contains_registers() {
  fault_capture::Snapshot crash;
  crash.reason = fault_capture::FatalReason::HardFault;
  crash.stage = fault_capture::StartupStage::TasksRunning;
  memcpy(crash.task_name, "control", 8);
  crash.pc = 0x1234ABCD;
  crash.lr = 0xDEADBEEF;
  char line[128];
  const size_t len = formatCrashLog(crash, line, sizeof(line));
  TEST_ASSERT_GREATER_THAN(0, len);
  TEST_ASSERT_NOT_NULL(strstr(line, "CRASH=1"));
  TEST_ASSERT_NOT_NULL(strstr(line, "TASK=control"));
  TEST_ASSERT_NOT_NULL(strstr(line, "PC=0x1234ABCD"));
  TEST_ASSERT_NOT_NULL(strstr(line, "LR=0xDEADBEEF"));
}

void test_queue_is_bounded_fifo() {
  PersistentEventQueue queue;
  for (uint8_t i = 0; i < PersistentEventQueue::kCapacity; ++i) {
    PersistentEvent event;
    event.detail = i;
    TEST_ASSERT_TRUE(queue.push(event));
  }
  TEST_ASSERT_FALSE(queue.push(PersistentEvent{}));
  TEST_ASSERT_EQUAL_UINT16(1, queue.dropped());
  for (uint8_t i = 0; i < PersistentEventQueue::kCapacity; ++i) {
    PersistentEvent event;
    TEST_ASSERT_TRUE(queue.peek(event));
    TEST_ASSERT_EQUAL_UINT8(i, event.detail);
    TEST_ASSERT_TRUE(queue.pop());
  }
  TEST_ASSERT_EQUAL_UINT8(0, queue.size());
}

void test_watchdog_line_contains_retained_progress() {
  char line[160];
  const size_t len = formatWatchdogResetLog(
      0x12, 71, 81, 3, line, sizeof(line));
  TEST_ASSERT_GREATER_THAN(0, len);
  TEST_ASSERT_NOT_NULL(strstr(line, "WATCHDOG=1"));
  TEST_ASSERT_NOT_NULL(strstr(line, "MISSED=0x00000012"));
  TEST_ASSERT_NOT_NULL(strstr(line, "DXL_PROGRESS=71"));
  TEST_ASSERT_NOT_NULL(strstr(line, "CONTROL_PROGRESS=81"));
  TEST_ASSERT_NOT_NULL(strstr(line, "STATE=3"));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_event_line_is_human_readable);
  RUN_TEST(test_crash_line_contains_registers);
  RUN_TEST(test_queue_is_bounded_fifo);
  RUN_TEST(test_watchdog_line_contains_retained_progress);
  return UNITY_END();
}