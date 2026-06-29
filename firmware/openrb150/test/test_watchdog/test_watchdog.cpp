// Native (host) unit tests for the cooperative watchdog liveness + critical-
// stall logic. The SAMD21 hardware WDT register access is guarded by
// ARDUINO_ARCH_SAMD, so on the native target hwEnable()/hwPet() are no-ops and
// only the portable bookkeeping under test here is exercised.
// Run with: pio test -e native -f test_watchdog

#include <unity.h>

#include "../../src/safety/watchdog.h"

using namespace watchdog;

namespace {

constexpr uint32_t bit(TaskId id) {
  return 1u << static_cast<uint8_t>(id);
}

// Heartbeat every task once (one full healthy window).
void checkInAll() {
  checkIn(TaskId::Control);
  checkIn(TaskId::Dxl);
  checkIn(TaskId::Rc);
  checkIn(TaskId::Api);
  checkIn(TaskId::I2c);
  checkIn(TaskId::Health);
}

// Establish a clean baseline: init, one full window, evaluate. After this every
// task is "live" and the missed mask is clear.
void primeHealthy() {
  init();
  checkInAll();
  evaluate();
}

}  // namespace

void setUp() { init(); }
void tearDown() {}

// After init() nothing has missed yet and no critical stall is flagged.
void test_init_clears_state() {
  TEST_ASSERT_EQUAL_UINT32(0u, missedMask());
  TEST_ASSERT_FALSE(criticalStalled());
}

// First evaluate() with no heartbeats at all marks every task missed (the
// counters never moved off their zero baseline).
void test_no_heartbeats_marks_all_missed() {
  init();
  evaluate();
  const uint32_t all = bit(TaskId::Control) | bit(TaskId::Dxl) |
                       bit(TaskId::Rc) | bit(TaskId::Api) | bit(TaskId::I2c) |
                       bit(TaskId::Health);
  TEST_ASSERT_EQUAL_UINT32(all, missedMask());
  TEST_ASSERT_TRUE(criticalStalled());
}

// A full window of heartbeats clears the missed mask.
void test_full_window_is_live() {
  init();
  checkInAll();
  evaluate();
  TEST_ASSERT_EQUAL_UINT32(0u, missedMask());
  TEST_ASSERT_FALSE(criticalStalled());
}

// A motion-critical task that stops checking in is both reported in the mask
// and flagged as a critical stall (-> WDT pet withheld in firmware).
void test_control_stall_is_critical() {
  primeHealthy();
  // Next window: everyone but Control checks in.
  checkIn(TaskId::Dxl);
  checkIn(TaskId::Rc);
  checkIn(TaskId::Api);
  checkIn(TaskId::I2c);
  checkIn(TaskId::Health);
  evaluate();
  TEST_ASSERT_EQUAL_UINT32(bit(TaskId::Control), missedMask());
  TEST_ASSERT_TRUE(criticalStalled());
}

void test_dxl_stall_is_critical() {
  primeHealthy();
  checkIn(TaskId::Control);
  checkIn(TaskId::Rc);
  checkIn(TaskId::Api);
  checkIn(TaskId::I2c);
  checkIn(TaskId::Health);
  evaluate();
  TEST_ASSERT_EQUAL_UINT32(bit(TaskId::Dxl), missedMask());
  TEST_ASSERT_TRUE(criticalStalled());
}

// Non-critical stalls (Rc/Api/I2c/Health) are reported but must NOT force a
// reset: the motion loop is healthy so criticalStalled() stays false and the
// firmware keeps petting the hardware WDT.
void test_noncritical_stall_does_not_force_reset() {
  primeHealthy();
  // Only the motion-critical pair checks in this window.
  checkIn(TaskId::Control);
  checkIn(TaskId::Dxl);
  evaluate();
  const uint32_t expected = bit(TaskId::Rc) | bit(TaskId::Api) |
                            bit(TaskId::I2c) | bit(TaskId::Health);
  TEST_ASSERT_EQUAL_UINT32(expected, missedMask());
  TEST_ASSERT_FALSE(criticalStalled());
}

// A transient single-window critical miss recovers on the next window: once the
// task resumes checking in, criticalStalled() clears (the WDT would be petted
// again before the ~2 s timeout).
void test_critical_stall_recovers() {
  primeHealthy();
  // Window with Control missing.
  checkIn(TaskId::Dxl);
  checkIn(TaskId::Rc);
  checkIn(TaskId::Api);
  checkIn(TaskId::I2c);
  checkIn(TaskId::Health);
  evaluate();
  TEST_ASSERT_TRUE(criticalStalled());
  // Next window: Control resumes -> recovered.
  checkInAll();
  evaluate();
  TEST_ASSERT_EQUAL_UINT32(0u, missedMask());
  TEST_ASSERT_FALSE(criticalStalled());
}

// checkIn() must ignore an out-of-range id without corrupting state.
void test_checkin_out_of_range_ignored() {
  init();
  checkIn(TaskId::Count);  // == kTaskCount, must be a no-op
  checkInAll();
  evaluate();
  TEST_ASSERT_EQUAL_UINT32(0u, missedMask());
  TEST_ASSERT_FALSE(criticalStalled());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_init_clears_state);
  RUN_TEST(test_no_heartbeats_marks_all_missed);
  RUN_TEST(test_full_window_is_live);
  RUN_TEST(test_control_stall_is_critical);
  RUN_TEST(test_dxl_stall_is_critical);
  RUN_TEST(test_noncritical_stall_does_not_force_reset);
  RUN_TEST(test_critical_stall_recovers);
  RUN_TEST(test_checkin_out_of_range_ignored);
  return UNITY_END();
}
