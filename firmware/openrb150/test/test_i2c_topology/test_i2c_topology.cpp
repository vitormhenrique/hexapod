// Native (host) unit tests for the portable I2C topology model + classifier.
// No Arduino/Wire dependencies; exercises i2c_topology.{h,cpp}.
//
// Run with: pio test -e native

#include <unity.h>

#include "../../src/sensors/i2c_topology.h"

using namespace i2c;

void test_init_topology_is_empty() {
  I2cTopology topo;
  // Dirty it first, then init.
  topo.mux_present = true;
  topo.eeprom_present = true;
  topo.channels[0].vcnl_present = true;
  initTopology(topo);
  TEST_ASSERT_FALSE(topo.mux_present);
  TEST_ASSERT_FALSE(topo.eeprom_present);
  for (uint8_t i = 0; i < kNumMuxChannels; ++i) {
    TEST_ASSERT_FALSE(topo.channels[i].scanned);
    TEST_ASSERT_FALSE(topo.channels[i].vcnl_present);
    TEST_ASSERT_FALSE(topo.channels[i].lps_present);
    TEST_ASSERT_EQUAL_UINT8(0, topo.channels[i].device_count);
    TEST_ASSERT_EQUAL(static_cast<int>(FootSensorState::Missing),
                      static_cast<int>(topo.channels[i].state));
  }
}

void test_classify_foot_sensor() {
  TEST_ASSERT_EQUAL(static_cast<int>(FootSensorState::Present),
                    static_cast<int>(classifyFootSensor(true, true)));
  TEST_ASSERT_EQUAL(static_cast<int>(FootSensorState::Fault),
                    static_cast<int>(classifyFootSensor(true, false)));
  TEST_ASSERT_EQUAL(static_cast<int>(FootSensorState::Fault),
                    static_cast<int>(classifyFootSensor(false, true)));
  TEST_ASSERT_EQUAL(static_cast<int>(FootSensorState::Missing),
                    static_cast<int>(classifyFootSensor(false, false)));
}

void test_expected_root_address() {
  TEST_ASSERT_TRUE(isExpectedRootAddress(kAddrTcaMux));
  TEST_ASSERT_TRUE(isExpectedRootAddress(kAddrEeprom));
  TEST_ASSERT_FALSE(isExpectedRootAddress(kAddrVcnl4040));
  TEST_ASSERT_FALSE(isExpectedRootAddress(0x00));
}

void test_update_channel_state() {
  ChannelInfo ch;
  ch.vcnl_present = true;
  ch.lps_present = true;
  updateChannelState(ch);
  TEST_ASSERT_EQUAL(static_cast<int>(FootSensorState::Present),
                    static_cast<int>(ch.state));

  ch.lps_present = false;
  updateChannelState(ch);
  TEST_ASSERT_EQUAL(static_cast<int>(FootSensorState::Fault),
                    static_cast<int>(ch.state));
}

void test_foot_sensor_present_mask_and_count() {
  I2cTopology topo;
  initTopology(topo);
  // Present on channels 0, 2, 5; partial (fault) on 1; missing on 3,4.
  topo.channels[0].state = FootSensorState::Present;
  topo.channels[1].state = FootSensorState::Fault;
  topo.channels[2].state = FootSensorState::Present;
  topo.channels[5].state = FootSensorState::Present;

  const uint8_t mask = footSensorPresentMask(topo);
  TEST_ASSERT_EQUAL_UINT8((1 << 0) | (1 << 2) | (1 << 5), mask);
  TEST_ASSERT_EQUAL_UINT8(3, footSensorPresentCount(topo));
}

void test_mask_ignores_reserved_channels() {
  I2cTopology topo;
  initTopology(topo);
  // Channels 6,7 are reserved and must never contribute to the foot mask/count.
  topo.channels[6].state = FootSensorState::Present;
  topo.channels[7].state = FootSensorState::Present;
  TEST_ASSERT_EQUAL_UINT8(0, footSensorPresentMask(topo));
  TEST_ASSERT_EQUAL_UINT8(0, footSensorPresentCount(topo));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_init_topology_is_empty);
  RUN_TEST(test_classify_foot_sensor);
  RUN_TEST(test_expected_root_address);
  RUN_TEST(test_update_channel_state);
  RUN_TEST(test_foot_sensor_present_mask_and_count);
  RUN_TEST(test_mask_ignores_reserved_channels);
  return UNITY_END();
}
