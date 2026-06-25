#pragma once

// ===========================================================================
// Robotic Finger Sensor v2 reader (Arduino-only, Wire transactions).
//
// Each foot board carries a VCNL4040 proximity sensor (0x60) and an LPS25HB
// barometric pressure sensor (0x5C). The caller (i2cTask, the sole Wire owner)
// selects the board's TCA9548A mux channel BEFORE calling read functions here;
// this class never touches the mux. All transactions are bounded and report
// failure (false) instead of blocking so the control loop is never stalled by
// a missing or faulty sensor (AGENTS.md 1.1 / 5.4).
//
// The portable contact state machine lives in contact_estimator.{h,cpp}; this
// reader only does the register I/O and hands raw counts upward.
//
// Datasheet registers used:
//   VCNL4040 (16-bit, low byte first):
//     PS_CONF1/2  command 0x03  -> power up PS, 16-bit output
//     PS_CONF3/MS command 0x04  -> LED current
//     PS_DATA     command 0x08  -> proximity counts
//   LPS25HB:
//     WHO_AM_I    0x0F (= 0xBD)
//     CTRL_REG1   0x20 -> PD=1 (active), ODR set
//     PRESS_OUT   0x28..0x2A (24-bit, auto-increment via MSB of sub-address)
// ===========================================================================

#include <Wire.h>
#include <stdint.h>

#include "contact_estimator.h"

namespace sensors {

class FingerSensorReader {
 public:
  explicit FingerSensorReader(arduino::TwoWire& wire) : wire_(wire) {}

  // Power up + configure the VCNL4040 and LPS25HB on the CURRENTLY SELECTED mux
  // channel. Returns true only if both devices were configured. Call once per
  // present channel before polling (and again after a recovered fault).
  bool configureChannel();

  // Read proximity counts (VCNL4040 PS_DATA) on the selected channel.
  bool readProximity(uint16_t& out);

  // Read raw pressure (LPS25HB 24-bit PRESS_OUT) on the selected channel.
  bool readPressure(int32_t& out);

  // Convenience: read both into a FootSample (sample.ok is true only when both
  // reads succeed). On any failure the partial values are left at 0.
  FootSample readFoot();

 private:
  bool write8(uint8_t addr, uint8_t reg, uint8_t value);
  bool writeCmd16(uint8_t addr, uint8_t cmd, uint8_t low, uint8_t high);
  bool readCmd16(uint8_t addr, uint8_t cmd, uint16_t& out);
  bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);

  arduino::TwoWire& wire_;
};

}  // namespace sensors
