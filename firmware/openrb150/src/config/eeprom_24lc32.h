#pragma once

// ===========================================================================
// 24LC32 / CAT24C32 I2C EEPROM driver (Arduino-only).
//
// 4 KB EEPROM on the root I2C bus at 0x50 (AGENTS.md 4.3), used to persist the
// transactional robot config. Implements config::ConfigBackend so the portable
// ConfigStore logic runs unchanged on-target and against a RAM fake in tests.
//
// Device characteristics:
//   * 4096 bytes, 12-bit word address (2-byte, big-endian on the wire).
//   * 32-byte page write buffer: a write must not cross a 32-byte page
//     boundary, so multi-page writes are split here.
//   * After a page write the device is busy (~5 ms); completion is detected by
//     ACK polling rather than a fixed delay.
//
// Owned exclusively by the I2C task (shares Wire with i2c::I2cBus; all EEPROM
// access happens with every mux channel deselected, i.e. on the root bus).
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#include "config_store.h"

namespace config {

class Eeprom24LC32 : public ConfigBackend {
 public:
  static constexpr uint8_t kI2cAddr = 0x50;
  static constexpr uint16_t kCapacity = 4096;
  static constexpr uint8_t kPageSize = 32;
  static constexpr uint8_t kWritePollTries = 25;  // ~25 ms worst case

  Eeprom24LC32(arduino::TwoWire& wire, uint8_t addr = kI2cAddr)
      : wire_(wire), addr_(addr) {}

  // ConfigBackend: sequential read across the device (auto-chunked to the Wire
  // RX buffer). Returns false on any I2C error or out-of-range access.
  bool read(uint16_t addr, uint8_t* buf, uint16_t len) override;

  // ConfigBackend: write across the device, split on 32-byte page boundaries
  // with ACK-polling between pages. Returns false on error or out-of-range.
  bool write(uint16_t addr, const uint8_t* buf, uint16_t len) override;

  // True if the device ACKs its address (present and not mid-write).
  bool isReady();

 private:
  // Block until the current write completes (ACK poll), bounded by tries.
  bool waitWriteComplete();
  // Write up to one page (must not cross a page boundary).
  bool writePage(uint16_t addr, const uint8_t* buf, uint8_t len);

  arduino::TwoWire& wire_;
  uint8_t addr_;
};

}  // namespace config
