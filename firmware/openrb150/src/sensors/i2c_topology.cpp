#include "i2c_topology.h"

namespace i2c {

void initTopology(I2cTopology& topo) {
  topo.mux_present = false;
  topo.eeprom_present = false;
  for (uint8_t i = 0; i < kNumMuxChannels; ++i) {
    topo.channels[i] = ChannelInfo{};
  }
}

FootSensorState classifyFootSensor(bool vcnl_present, bool lps_present) {
  if (vcnl_present && lps_present) {
    return FootSensorState::Present;
  }
  if (vcnl_present || lps_present) {
    return FootSensorState::Fault;  // partial board: one device missing
  }
  return FootSensorState::Missing;
}

bool isExpectedRootAddress(uint8_t addr) {
  return addr == kAddrTcaMux || addr == kAddrEeprom;
}

void updateChannelState(ChannelInfo& ch) {
  ch.state = classifyFootSensor(ch.vcnl_present, ch.lps_present);
}

uint8_t footSensorPresentMask(const I2cTopology& topo) {
  uint8_t mask = 0;
  for (uint8_t ch = 0; ch < kNumFootChannels; ++ch) {
    if (topo.channels[ch].state == FootSensorState::Present) {
      mask |= static_cast<uint8_t>(1u << ch);
    }
  }
  return mask;
}

uint8_t footSensorPresentCount(const I2cTopology& topo) {
  uint8_t count = 0;
  for (uint8_t ch = 0; ch < kNumFootChannels; ++ch) {
    if (topo.channels[ch].state == FootSensorState::Present) {
      ++count;
    }
  }
  return count;
}

}  // namespace i2c
