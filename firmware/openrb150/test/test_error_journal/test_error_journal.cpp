// Native (host) Unity tests for the deduplicated error journal.
//
// Run with:  pio test -e native -f test_error_journal

#include <unity.h>

#include "safety/error_journal.h"

using namespace safety;

void test_first_occurrence_is_announced() {
  ErrorJournal j;
  TEST_ASSERT_TRUE(j.note(ErrorCode::RcFailsafe, 0, ErrorSeverity::Error, 100));
  TEST_ASSERT_TRUE(j.hasPending());
  ErrorEntry e;
  TEST_ASSERT_TRUE(j.takePending(&e));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ErrorCode::RcFailsafe),
                          static_cast<uint8_t>(e.code));
  TEST_ASSERT_EQUAL_UINT16(1, e.count);
  TEST_ASSERT_NOT_EQUAL(0, e.sequence);
  TEST_ASSERT_FALSE(j.hasPending());
}

// The whole point of the journal: a fault repeating at the control-loop rate
// must produce ONE announcement, not one per cycle.
void test_repeats_are_suppressed_and_counted() {
  ErrorJournal j;
  j.note(ErrorCode::GoalClamped, 0, ErrorSeverity::Info, 0);
  ErrorEntry e;
  TEST_ASSERT_TRUE(j.takePending(&e));
  for (uint32_t t = 10; t < kRepeatIntervalMs; t += 10) {
    TEST_ASSERT_FALSE(j.note(ErrorCode::GoalClamped, 0, ErrorSeverity::Info, t));
  }
  TEST_ASSERT_FALSE(j.hasPending());
  TEST_ASSERT_EQUAL_UINT32(kRepeatIntervalMs / 10 - 1, j.suppressed());
  TEST_ASSERT_EQUAL_UINT16(kRepeatIntervalMs / 10, j.at(0).count);
}

void test_still_failing_is_reannounced_once_per_interval() {
  ErrorJournal j;
  j.note(ErrorCode::DxlWriteFailed, 3, ErrorSeverity::Error, 0);
  ErrorEntry first;
  TEST_ASSERT_TRUE(j.takePending(&first));

  TEST_ASSERT_FALSE(
      j.note(ErrorCode::DxlWriteFailed, 3, ErrorSeverity::Error, 4000));
  TEST_ASSERT_TRUE(
      j.note(ErrorCode::DxlWriteFailed, 3, ErrorSeverity::Error, 5000));
  ErrorEntry repeat;
  TEST_ASSERT_TRUE(j.takePending(&repeat));
  // Same incident: the running count carries over and the sequence advances.
  TEST_ASSERT_EQUAL_UINT16(3, repeat.count);
  TEST_ASSERT_NOT_EQUAL(first.sequence, repeat.sequence);
}

void test_quiet_key_starts_a_new_incident() {
  ErrorJournal j;
  j.note(ErrorCode::I2cFootSensorFault, 2, ErrorSeverity::Warning, 0);
  ErrorEntry e;
  j.takePending(&e);
  TEST_ASSERT_TRUE(j.note(ErrorCode::I2cFootSensorFault, 2,
                          ErrorSeverity::Warning, kClearAfterMs + 1));
  TEST_ASSERT_TRUE(j.takePending(&e));
  TEST_ASSERT_EQUAL_UINT16(1, e.count);  // counter restarted
}

void test_distinct_details_are_distinct_entries() {
  ErrorJournal j;
  TEST_ASSERT_TRUE(
      j.note(ErrorCode::DxlHardwareError, 1, ErrorSeverity::Error, 0));
  TEST_ASSERT_TRUE(
      j.note(ErrorCode::DxlHardwareError, 2, ErrorSeverity::Error, 0));
  TEST_ASSERT_EQUAL_UINT8(2, j.size());
  TEST_ASSERT_EQUAL_UINT8(2, j.pendingCount());
}

void test_pending_drains_highest_severity_first() {
  ErrorJournal j;
  j.note(ErrorCode::GoalClamped, 0, ErrorSeverity::Info, 0);
  j.note(ErrorCode::BatteryLow, 99, ErrorSeverity::Warning, 1);
  j.note(ErrorCode::WatchdogStall, 0, ErrorSeverity::Critical, 2);
  ErrorEntry e;
  TEST_ASSERT_TRUE(j.takePending(&e));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ErrorCode::WatchdogStall),
                          static_cast<uint8_t>(e.code));
  TEST_ASSERT_TRUE(j.takePending(&e));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ErrorCode::BatteryLow),
                          static_cast<uint8_t>(e.code));
  TEST_ASSERT_TRUE(j.takePending(&e));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ErrorCode::GoalClamped),
                          static_cast<uint8_t>(e.code));
  TEST_ASSERT_FALSE(j.takePending(&e));
}

// A storm of low-severity noise must never be able to evict an unsent critical
// fault from the fixed table.
void test_full_table_never_evicts_a_pending_critical() {
  ErrorJournal j;
  j.note(ErrorCode::WatchdogStall, 0, ErrorSeverity::Critical, 0);
  for (uint8_t i = 0; i < kMaxErrorEntries + 8; ++i) {
    j.note(ErrorCode::GoalClamped, i, ErrorSeverity::Info,
           static_cast<uint32_t>(10 + i));
  }
  bool found_critical = false;
  for (uint8_t i = 0; i < j.size(); ++i) {
    if (j.at(i).code == ErrorCode::WatchdogStall) found_critical = true;
  }
  TEST_ASSERT_TRUE(found_critical);
  TEST_ASSERT_LESS_OR_EQUAL_UINT8(kMaxErrorEntries, j.size());
}

void test_none_code_is_ignored() {
  ErrorJournal j;
  TEST_ASSERT_FALSE(j.note(ErrorCode::None, 0, ErrorSeverity::Critical, 0));
  TEST_ASSERT_EQUAL_UINT8(0, j.size());
  TEST_ASSERT_FALSE(j.hasLatest());
}

void test_latest_tracks_the_last_announced_entry() {
  ErrorJournal j;
  TEST_ASSERT_FALSE(j.hasLatest());
  j.note(ErrorCode::ConfigVolatile, 0, ErrorSeverity::Warning, 0);
  j.takePending(nullptr);
  TEST_ASSERT_TRUE(j.hasLatest());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ErrorCode::ConfigVolatile),
                          static_cast<uint8_t>(j.latest().code));
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_first_occurrence_is_announced);
  RUN_TEST(test_repeats_are_suppressed_and_counted);
  RUN_TEST(test_still_failing_is_reannounced_once_per_interval);
  RUN_TEST(test_quiet_key_starts_a_new_incident);
  RUN_TEST(test_distinct_details_are_distinct_entries);
  RUN_TEST(test_pending_drains_highest_severity_first);
  RUN_TEST(test_full_table_never_evicts_a_pending_critical);
  RUN_TEST(test_none_code_is_ignored);
  RUN_TEST(test_latest_tracks_the_last_announced_entry);
  return UNITY_END();
}
