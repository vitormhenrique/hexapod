// Native tests for the adapter-fed ControllerCore clock.
// Run with: pio test -e native -f test_controller_time

#include <stdint.h>

#include <unity.h>

#include "../../src/controller/controller_time.h"

using controller::ControllerClock;
using controller::ControllerTime;
using controller::controllerTimeFromTickCount;

void test_first_and_zero_elapsed_samples_are_valid() {
  ControllerClock clock;

  const ControllerTime first = clock.sampleMilliseconds(100);
  const ControllerTime zero = clock.sampleMilliseconds(100);

  TEST_ASSERT_TRUE(first.valid);
  TEST_ASSERT_EQUAL_UINT32(100, first.now_ms);
  TEST_ASSERT_EQUAL_UINT32(0, first.dt_ms);
  TEST_ASSERT_TRUE(zero.valid);
  TEST_ASSERT_EQUAL_UINT32(0, zero.dt_ms);
}

void test_nominal_ten_ms_tick_matches_native_injected_time() {
  ControllerClock rtos_clock;
  ControllerClock native_clock;

  const ControllerTime rtos_first =
      controllerTimeFromTickCount(rtos_clock, 100, 1);
  const ControllerTime native_first = native_clock.sampleMilliseconds(100);
  const ControllerTime rtos_next =
      controllerTimeFromTickCount(rtos_clock, 110, 1);
  const ControllerTime native_next = native_clock.sampleMilliseconds(110);

  TEST_ASSERT_TRUE(rtos_first.valid);
  TEST_ASSERT_EQUAL_UINT32(native_first.now_ms, rtos_first.now_ms);
  TEST_ASSERT_EQUAL_UINT32(native_first.dt_ms, rtos_first.dt_ms);
  TEST_ASSERT_TRUE(rtos_next.valid);
  TEST_ASSERT_EQUAL_UINT32(native_next.now_ms, rtos_next.now_ms);
  TEST_ASSERT_EQUAL_UINT32(native_next.dt_ms, rtos_next.dt_ms);
  TEST_ASSERT_EQUAL_UINT32(10, rtos_next.dt_ms);
}

void test_overrun_preserves_real_elapsed_time() {
  ControllerClock clock;
  clock.sampleMilliseconds(10);

  const ControllerTime overrun = clock.sampleMilliseconds(85);

  TEST_ASSERT_TRUE(overrun.valid);
  TEST_ASSERT_EQUAL_UINT32(75, overrun.dt_ms);
}

void test_unsigned_wraparound_preserves_elapsed_time() {
  ControllerClock clock;
  clock.sampleMilliseconds(UINT32_MAX - 7u);

  const ControllerTime wrapped = clock.sampleMilliseconds(2);

  TEST_ASSERT_TRUE(wrapped.valid);
  TEST_ASSERT_EQUAL_UINT32(10, wrapped.dt_ms);
}

void test_invalid_tick_period_and_large_jump_fail_closed() {
  ControllerClock clock;
  const ControllerTime invalid_period = controllerTimeFromTickCount(clock, 10, 0);
  TEST_ASSERT_FALSE(invalid_period.valid);

  clock.sampleMilliseconds(10);
  const ControllerTime invalid_jump = clock.sampleMilliseconds(1011);
  TEST_ASSERT_FALSE(invalid_jump.valid);
  TEST_ASSERT_EQUAL_UINT32(0, invalid_jump.dt_ms);

  const ControllerTime rebased = clock.sampleMilliseconds(1021);
  TEST_ASSERT_TRUE(rebased.valid);
  TEST_ASSERT_EQUAL_UINT32(10, rebased.dt_ms);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_first_and_zero_elapsed_samples_are_valid);
  RUN_TEST(test_nominal_ten_ms_tick_matches_native_injected_time);
  RUN_TEST(test_overrun_preserves_real_elapsed_time);
  RUN_TEST(test_unsigned_wraparound_preserves_elapsed_time);
  RUN_TEST(test_invalid_tick_period_and_large_jump_fail_closed);
  return UNITY_END();
}