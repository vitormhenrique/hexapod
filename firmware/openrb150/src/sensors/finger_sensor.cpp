#include "finger_sensor.h"

#include "i2c_topology.h"  // kAddrVcnl4040 / kAddrLps25hb

namespace sensors {

namespace {

// VCNL4040 command codes (each addresses a 16-bit register, low byte first).
constexpr uint8_t kVcnlPsConf12 = 0x03;  // PS_CONF1 (low) / PS_CONF2 (high)
constexpr uint8_t kVcnlPsConf3Ms = 0x04;  // PS_CONF3 (low) / PS_MS (high)
constexpr uint8_t kVcnlPsData = 0x08;     // proximity output

// PS_CONF1 low byte: PS_SD bit0 = 0 powers the proximity engine on; default
// duty/integration/persistence otherwise.
constexpr uint8_t kVcnlPsConf1 = 0x00;
// PS_CONF2 high byte: PS_HD bit3 = 1 selects 16-bit proximity output.
constexpr uint8_t kVcnlPsConf2 = 0x08;
// PS_CONF3 low byte: defaults. PS_MS high byte: LED current ~ default (75 mA).
constexpr uint8_t kVcnlPsConf3 = 0x00;
constexpr uint8_t kVcnlPsMs = 0x00;

// LPS25HB registers.
constexpr uint8_t kLpsWhoAmI = 0x0F;     // = 0xBD
constexpr uint8_t kLpsCtrlReg1 = 0x20;
constexpr uint8_t kLpsPressOutXl = 0x28;  // 0x28..0x2A, 24-bit
constexpr uint8_t kLpsWhoAmIValue = 0xBD;
// CTRL_REG1: PD bit7 = 1 (active), ODR bits[6:4] = 011 -> 12.5 Hz, BDU bit2.
constexpr uint8_t kLpsCtrlReg1Active = 0xB4;
// Auto-increment of the sub-address requires bit7 set on multi-byte reads.
constexpr uint8_t kLpsAutoIncrement = 0x80;

}  // namespace

bool FingerSensorReader::write8(uint8_t addr, uint8_t reg, uint8_t value) {
  wire_.beginTransmission(addr);
  wire_.write(reg);
  wire_.write(value);
  return wire_.endTransmission() == 0;
}

bool FingerSensorReader::writeCmd16(uint8_t addr, uint8_t cmd, uint8_t low,
                                    uint8_t high) {
  wire_.beginTransmission(addr);
  wire_.write(cmd);
  wire_.write(low);
  wire_.write(high);
  return wire_.endTransmission() == 0;
}

bool FingerSensorReader::readCmd16(uint8_t addr, uint8_t cmd, uint16_t& out) {
  wire_.beginTransmission(addr);
  wire_.write(cmd);
  // Repeated start (no stop) so the device keeps the command pointer.
  if (wire_.endTransmission(false) != 0) {
    return false;
  }
  if (wire_.requestFrom(addr, static_cast<uint8_t>(2)) != 2) {
    return false;
  }
  const uint8_t low = static_cast<uint8_t>(wire_.read());
  const uint8_t high = static_cast<uint8_t>(wire_.read());
  out = static_cast<uint16_t>(low | (static_cast<uint16_t>(high) << 8));
  return true;
}

bool FingerSensorReader::readRegs(uint8_t addr, uint8_t reg, uint8_t* buf,
                                  uint8_t len) {
  wire_.beginTransmission(addr);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) {
    return false;
  }
  if (wire_.requestFrom(addr, len) != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) {
    buf[i] = static_cast<uint8_t>(wire_.read());
  }
  return true;
}

bool FingerSensorReader::configureChannel() {
  // VCNL4040: power up the proximity engine in 16-bit mode and set LED current.
  bool ok = writeCmd16(i2c::kAddrVcnl4040, kVcnlPsConf12, kVcnlPsConf1,
                       kVcnlPsConf2);
  ok = writeCmd16(i2c::kAddrVcnl4040, kVcnlPsConf3Ms, kVcnlPsConf3, kVcnlPsMs) &&
       ok;

  // LPS25HB: verify identity then bring it out of power-down.
  uint8_t who = 0;
  const bool who_ok =
      readRegs(i2c::kAddrLps25hb, kLpsWhoAmI, &who, 1) && who == kLpsWhoAmIValue;
  ok = who_ok && ok;
  ok = write8(i2c::kAddrLps25hb, kLpsCtrlReg1, kLpsCtrlReg1Active) && ok;
  return ok;
}

bool FingerSensorReader::readProximity(uint16_t& out) {
  return readCmd16(i2c::kAddrVcnl4040, kVcnlPsData, out);
}

bool FingerSensorReader::readPressure(int32_t& out) {
  uint8_t b[3] = {0, 0, 0};
  if (!readRegs(i2c::kAddrLps25hb,
                static_cast<uint8_t>(kLpsPressOutXl | kLpsAutoIncrement), b,
                3)) {
    return false;
  }
  // 24-bit signed little-endian pressure count (XL, L, H).
  int32_t raw = static_cast<int32_t>(b[0]) |
                (static_cast<int32_t>(b[1]) << 8) |
                (static_cast<int32_t>(b[2]) << 16);
  if (raw & 0x00800000) {  // sign-extend from 24 bits
    raw |= ~0x00FFFFFF;
  }
  out = raw;
  return true;
}

FootSample FingerSensorReader::readFoot() {
  FootSample fs;
  uint16_t prox = 0;
  int32_t press = 0;
  const bool prox_ok = readProximity(prox);
  const bool press_ok = readPressure(press);
  fs.proximity_raw = prox_ok ? prox : 0;
  fs.pressure_raw = press_ok ? press : 0;
  fs.ok = prox_ok && press_ok;
  return fs;
}

}  // namespace sensors
