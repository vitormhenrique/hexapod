#include <unity.h>

#include "dxl/dxl_fault_monitor.h"

void test_protocol1_overload_must_persist() {
  dxl::FaultMonitor monitor;
  monitor.observe(5, dxl::TableKind::Mx28Legacy, 0x20);
  TEST_ASSERT_FALSE(monitor.faulted());
  monitor.observe(5, dxl::TableKind::Mx28Legacy, 0x20);
  TEST_ASSERT_FALSE(monitor.faulted());
  monitor.observe(5, dxl::TableKind::Mx28Legacy, 0x20);
  TEST_ASSERT_TRUE(monitor.faulted());
}

void test_clear_sample_resets_persistent_streak() {
  dxl::FaultMonitor monitor;
  monitor.observe(5, dxl::TableKind::Mx28Legacy, 0x20);
  monitor.observe(5, dxl::TableKind::Mx28Legacy, 0x00);
  monitor.observe(5, dxl::TableKind::Mx28Legacy, 0x20);
  monitor.observe(5, dxl::TableKind::Mx28Legacy, 0x20);
  TEST_ASSERT_FALSE(monitor.faulted());
}

void test_protocol1_packet_errors_do_not_hard_fault() {
  dxl::FaultMonitor monitor;
  for (uint8_t sample = 0; sample < 10; ++sample) {
    monitor.observe(2, dxl::TableKind::Mx28Legacy, 0x58);
  }
  TEST_ASSERT_FALSE(monitor.faulted());
}

void test_critical_temperature_and_mx2_electrical_fault_immediately() {
  dxl::FaultMonitor legacy;
  legacy.observe(0, dxl::TableKind::Mx28Legacy, 0x04);
  TEST_ASSERT_TRUE(legacy.faulted());

  dxl::FaultMonitor mx2;
  mx2.observe(0, dxl::TableKind::Mx28V2, 0x10);
  TEST_ASSERT_TRUE(mx2.faulted());
}

void test_reset_releases_latched_monitor() {
  dxl::FaultMonitor monitor;
  monitor.observe(0, dxl::TableKind::Mx28Legacy, 0x04);
  TEST_ASSERT_TRUE(monitor.faulted());
  monitor.reset();
  TEST_ASSERT_FALSE(monitor.faulted());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_protocol1_overload_must_persist);
  RUN_TEST(test_clear_sample_resets_persistent_streak);
  RUN_TEST(test_protocol1_packet_errors_do_not_hard_fault);
  RUN_TEST(test_critical_temperature_and_mx2_electrical_fault_immediately);
  RUN_TEST(test_reset_releases_latched_monitor);
  return UNITY_END();
}
