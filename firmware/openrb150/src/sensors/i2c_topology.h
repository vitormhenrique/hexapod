#pragma once

// ===========================================================================
// I2C topology model and device classification (portable, no Arduino deps).
//
// The robot's I2C tree (AGENTS.md 4.3):
//   * root bus: TCA9548A 8-channel mux @ 0x70, 24LC32 config EEPROM @ 0x50
//   * mux channels 0..5: one Robotic Finger Sensor v2 foot board each, which
//     carries a VCNL4040 proximity sensor (0x60) and an LPS25HB pressure
//     sensor (0x5C). Channels 6,7 are reserved.
//
// This translation unit holds only the data model + classification logic so it
// can be unit-tested on the host. The actual Wire transactions and exclusive
// mux channel selection live in i2c_bus.{h,cpp} (Arduino-only).
// ===========================================================================

#include <stdint.h>

namespace i2c {

// Root-bus device addresses.
constexpr uint8_t kAddrTcaMux = 0x70;   // TCA9548A 8-channel I2C mux
constexpr uint8_t kAddrEeprom = 0x50;   // 24LC32 / CAT24C32 config EEPROM

// Robotic Finger Sensor v2 on-board device addresses.
constexpr uint8_t kAddrVcnl4040 = 0x60;  // proximity / ambient (VCNL4040)
constexpr uint8_t kAddrLps25hb = 0x5C;   // barometric pressure (LPS25HB, SA0=0)

constexpr uint8_t kNumMuxChannels = 8;   // TCA9548A has 8 channels
constexpr uint8_t kNumFootChannels = 6;  // channels 0..5 carry foot sensors

// Classification of a foot sensor board on one mux channel.
enum class FootSensorState : uint8_t {
  Missing = 0,  // neither on-board device responded
  Present = 1,  // both VCNL4040 and LPS25HB responded
  Fault = 2,    // only one of the two devices responded (partial board)
};

// Per-channel scan result.
struct ChannelInfo {
  bool scanned = false;
  bool vcnl_present = false;
  bool lps_present = false;
  uint8_t device_count = 0;  // total addresses that ACKed on this channel
  FootSensorState state = FootSensorState::Missing;
};

// Full discovered I2C topology.
struct I2cTopology {
  bool mux_present = false;
  bool eeprom_present = false;
  ChannelInfo channels[kNumMuxChannels];
};

// Reset a topology to the all-absent state.
void initTopology(I2cTopology& topo);

// Classify a foot board from the presence of its two on-board devices.
FootSensorState classifyFootSensor(bool vcnl_present, bool lps_present);

// True if `addr` is an expected root-bus device (mux or EEPROM).
bool isExpectedRootAddress(uint8_t addr);

// Recompute the derived `state` of a single channel from its presence flags.
void updateChannelState(ChannelInfo& ch);

// Bitmask (bit N = channel N) of foot channels 0..5 classified Present.
uint8_t footSensorPresentMask(const I2cTopology& topo);

// Count of foot channels 0..5 classified Present.
uint8_t footSensorPresentCount(const I2cTopology& topo);

}  // namespace i2c
