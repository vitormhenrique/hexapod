#pragma once

// ===========================================================================
// I2C bus manager (single owner of SERCOM0 / the root I2C bus).
//
// Per AGENTS.md 5.1 only the I2C task may touch SERCOM0, the TCA9548A mux, the
// Qwiic OpenLog, debug OLED, and foot sensors. This class encapsulates ownership
// and provides the Phase 1 discovery path (rbg.7):
//
//   * root-bus scan: detect the mux, OpenLog, and optional debug OLED
//   * exclusive mux channel selection (exactly one channel active at a time)
//   * per-channel sensor scan for the Robotic Finger Sensor v2 devices
//
// Exclusive selection matters because every foot board uses the same fixed
// addresses (0x60 / 0x5C); only one mux channel may be enabled while probing.
//
// Arduino-only. The topology data model + classification logic
// lives in i2c_topology.{h,cpp}, which is unit-tested on the host.
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#include "i2c_topology.h"

namespace i2c {

// Aggregate I2C health counters (surfaced via telemetry/diagnostics later).
struct I2cStats {
  uint32_t root_scans = 0;
  uint32_t channel_scans = 0;
  uint32_t mux_select_errors = 0;
  uint32_t timeouts = 0;
  uint32_t recoveries = 0;
  uint8_t last_error = 0;
};

class I2cBus {
 public:
  I2cBus() = default;

  // Initialize the I2C peripheral. Does not scan. Default 100 kHz keeps long
  // foot-sensor harness runs reliable; raise later only if proven safe.
  bool begin(uint32_t clock_hz = 100000);
  bool isReady() const { return ready_; }
  TwoWire& wire() { return Wire; }
  uint32_t transactionTimeoutUs() const { return transaction_timeout_us_; }
  void setTransactionTimeoutUs(uint32_t timeout_us) {
    transaction_timeout_us_ = timeout_us == 0 ? 5000u : timeout_us;
  }

  bool write(uint8_t addr, const uint8_t* data, uint8_t len,
             bool send_stop = true);
  bool read(uint8_t addr, uint8_t* data, uint8_t len,
            bool send_stop = true);
  bool writeRead(uint8_t addr, const uint8_t* write_data, uint8_t write_len,
                 uint8_t* read_data, uint8_t read_len);

  // Probe a 7-bit address on the currently selected bus/channel. Returns true
  // if the device ACKed (endTransmission() == 0).
  bool probe(uint8_t addr);

  // Select exactly one mux channel (0..7) by writing its one-hot bitmask to the
  // TCA9548A. Returns true on a clean write. Any previously selected channel is
  // implicitly deselected (one-hot, not additive).
  bool selectChannel(uint8_t channel);

  // Deselect all mux channels (write 0x00). Leaves the root bus addressable.
  bool selectNone();

  // Scan root devices and record the detected alternate addresses.
  void scanRoot(I2cTopology& topo);

  // Scan foot-sensor channels 0..5 (requires the mux to be present). For each
  // channel: select it exclusively, probe the VCNL4040/LPS25HB addresses,
  // classify the board, then deselect. Channels 6,7 are left untouched.
  void scanChannels(I2cTopology& topo);

  // Full discovery: scanRoot then, if the mux is present, scanChannels.
  void scanAll(I2cTopology& topo);

  const I2cStats& stats() const { return stats_; }

 private:
  bool initializeHardware();
  bool start(uint8_t addr, bool read);
  bool sendStop();
  bool waitForFlag(uint8_t flags);
  bool waitForSync(uint32_t mask);
  bool recover();
  bool fail(uint8_t error, bool recover_bus);

  I2cStats stats_;
  bool ready_ = false;
  uint32_t clock_hz_ = 100000;
  uint32_t transaction_timeout_us_ = 5000;
};

}  // namespace i2c
