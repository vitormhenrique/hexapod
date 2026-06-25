#pragma once

// ===========================================================================
// I2C bus manager (single owner of Wire / the root I2C bus).
//
// Per AGENTS.md 5.1 only the I2C task may touch Wire, the TCA9548A mux, the
// 24LC32 EEPROM, and the foot sensors. This class encapsulates that ownership
// and provides the Phase 1 discovery path (rbg.7):
//
//   * root-bus scan: detect the mux (0x70) and EEPROM (0x50)
//   * exclusive mux channel selection (exactly one channel active at a time)
//   * per-channel sensor scan for the Robotic Finger Sensor v2 devices
//
// Exclusive selection matters because every foot board uses the same fixed
// addresses (0x60 / 0x5C); only one mux channel may be enabled while probing.
//
// Arduino-only (pulls in Wire). The topology data model + classification logic
// lives in i2c_topology.{h,cpp}, which is unit-tested on the host.
// ===========================================================================

#include <Wire.h>
#include <stdint.h>

#include "i2c_topology.h"

namespace i2c {

// Aggregate I2C health counters (surfaced via telemetry/diagnostics later).
struct I2cStats {
  uint32_t root_scans = 0;
  uint32_t channel_scans = 0;
  uint32_t mux_select_errors = 0;
  uint8_t last_error = 0;  // last Wire endTransmission() status
};

class I2cBus {
 public:
  explicit I2cBus(arduino::TwoWire& wire) : wire_(wire) {}

  // Initialize the I2C peripheral. Does not scan. Default 100 kHz keeps long
  // foot-sensor harness runs reliable; raise later only if proven safe.
  void begin(uint32_t clock_hz = 100000);
  bool isReady() const { return ready_; }

  // Probe a 7-bit address on the currently selected bus/channel. Returns true
  // if the device ACKed (endTransmission() == 0).
  bool probe(uint8_t addr);

  // Select exactly one mux channel (0..7) by writing its one-hot bitmask to the
  // TCA9548A. Returns true on a clean write. Any previously selected channel is
  // implicitly deselected (one-hot, not additive).
  bool selectChannel(uint8_t channel);

  // Deselect all mux channels (write 0x00). Leaves the root bus addressable.
  bool selectNone();

  // Scan the root bus for the mux and EEPROM, filling topo.mux_present /
  // topo.eeprom_present. Leaves all mux channels deselected.
  void scanRoot(I2cTopology& topo);

  // Scan foot-sensor channels 0..5 (requires the mux to be present). For each
  // channel: select it exclusively, probe the VCNL4040/LPS25HB addresses,
  // classify the board, then deselect. Channels 6,7 are left untouched.
  void scanChannels(I2cTopology& topo);

  // Full discovery: scanRoot then, if the mux is present, scanChannels.
  void scanAll(I2cTopology& topo);

  const I2cStats& stats() const { return stats_; }

 private:
  arduino::TwoWire& wire_;
  I2cStats stats_;
  bool ready_ = false;
};

}  // namespace i2c
