// Native coverage for build-gated benchmark metric arithmetic.

#include <unity.h>

#include "../../src/app/benchmark.h"

using app::benchmark::Task;
using app::benchmark::g_collector;

#if !defined(HEXAPOD_BENCHMARK)

void test_benchmark_requires_enabled_build_flag() {
  TEST_IGNORE_MESSAGE("run under native_benchmark");
}

#else

void test_task_metric_tracks_runtime_jitter_and_overrun() {
  g_collector.reset();
  g_collector.recordTask(Task::Control, 1000, 1300, 1000);
  g_collector.recordTask(Task::Control, 2200, 3700, 1000);

  const app::benchmark::Snapshot snapshot = g_collector.snapshot();
  const app::benchmark::Metric& control =
      snapshot.tasks[static_cast<uint8_t>(Task::Control)];
  TEST_ASSERT_EQUAL_UINT32(2, control.count);
  TEST_ASSERT_EQUAL_UINT32(1800, control.total_us);
  TEST_ASSERT_EQUAL_UINT32(1500, control.max_us);
  TEST_ASSERT_EQUAL_UINT32(200, control.max_release_jitter_us);
  TEST_ASSERT_EQUAL_UINT32(1, control.overruns);
}

void test_motion_metrics_only_include_motion_cycles() {
  g_collector.reset();
  g_collector.recordControlStep(25, false);
  g_collector.recordControlStep(50, true);
  g_collector.recordDxlCycle(75, false);
  g_collector.recordDxlCycle(100, true);
  g_collector.recordI2cOled(125);
  g_collector.recordI2cImu(150);
  g_collector.recordI2cStorage(175);

  const app::benchmark::Snapshot snapshot = g_collector.snapshot();
  TEST_ASSERT_EQUAL_UINT32(2, snapshot.control_step.count);
  TEST_ASSERT_EQUAL_UINT32(1, snapshot.motion_control_step.count);
  TEST_ASSERT_EQUAL_UINT32(50, snapshot.motion_control_step.max_us);
  TEST_ASSERT_EQUAL_UINT32(1, snapshot.motion_dxl_cycle.count);
  TEST_ASSERT_EQUAL_UINT32(100, snapshot.motion_dxl_cycle.max_us);
  TEST_ASSERT_EQUAL_UINT32(125, snapshot.i2c_oled.max_us);
  TEST_ASSERT_EQUAL_UINT32(150, snapshot.i2c_imu.max_us);
  TEST_ASSERT_EQUAL_UINT32(175, snapshot.i2c_storage.max_us);
}

#endif

int main(int, char**) {
  UNITY_BEGIN();
#if !defined(HEXAPOD_BENCHMARK)
  RUN_TEST(test_benchmark_requires_enabled_build_flag);
#else
  RUN_TEST(test_task_metric_tracks_runtime_jitter_and_overrun);
  RUN_TEST(test_motion_metrics_only_include_motion_cycles);
#endif
  return UNITY_END();
}