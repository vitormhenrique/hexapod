#pragma once

// ===========================================================================
// DYNAMIXEL logical parameter map (portable, no Arduino deps).
//
// The USB API exposes *logical* servo parameters (id, baud, return delay, PID,
// limits, torque limit, ...) rather than raw control-table addresses, because
// the two MX-28 control tables (legacy Protocol 1.0 vs MX-28(2.0), see
// dxl_model.h / AGENTS.md 4.2) place the same logical value at different
// addresses, lengths, and even with no equivalent at all. For example the joint
// travel bound is CW/CCW Angle Limit on legacy and Min/Max Position Limit on
// MX(2.0).
//
// This module maps a (TableKind, LogicalParam) pair to a ParamDescriptor giving
// the register address, byte length, EEPROM-vs-RAM region (EEPROM writes need
// torque off), signedness, and whether the value is writable. The Arduino bus
// executor reads/writes by raw address using these descriptors, so the address
// logic stays here and is unit-tested on the host.
//
// A parameter that has no equivalent on a given table returns false from
// paramDescriptor(), and the API reports it as Unsupported for that servo.
// ===========================================================================

#include <stdint.h>

#include "dxl_model.h"

namespace dxl {

// Logical servo parameters (AGENTS.md 5.6 "Logical parameters ... at minimum").
// Wire values are stable; do not renumber.
enum class LogicalParam : uint8_t {
  Id = 0,
  BaudRate = 1,
  ReturnDelayTime = 2,
  CwAngleLimit = 3,        // legacy only (min Goal Position bound)
  CcwAngleLimit = 4,       // legacy only (max Goal Position bound)
  MinPositionLimit = 5,    // MX(2.0) only
  MaxPositionLimit = 6,    // MX(2.0) only
  TemperatureLimit = 7,
  MinVoltageLimit = 8,
  MaxVoltageLimit = 9,
  MaxTorque = 10,          // legacy only
  StatusReturnLevel = 11,
  Shutdown = 12,
  PidP = 13,
  PidI = 14,
  PidD = 15,
  MovingSpeed = 16,        // legacy only (RAM); MX(2.0) uses ProfileVelocity
  TorqueLimit = 17,        // legacy only (RAM)
  GoalAcceleration = 18,   // legacy only (RAM); MX(2.0) uses ProfileAcceleration
  HomingOffset = 19,       // MX(2.0) only (signed)
  ProfileVelocity = 20,    // MX(2.0) only
  ProfileAcceleration = 21,  // MX(2.0) only
  BusWatchdog = 22,        // MX(2.0) only
  TorqueEnable = 23,       // RAM, both tables
  Count = 24,
};

inline bool isLogicalParam(uint8_t v) {
  return v < static_cast<uint8_t>(LogicalParam::Count);
}

// Where a parameter lives, which controls whether a write needs torque off.
enum class ParamRegion : uint8_t {
  Eeprom = 0,  // write requires torque off; persists in servo flash
  Ram = 1,     // write allowed with torque on
};

// Resolved addressing for one (table, param) pair. Kept an aggregate (no
// member initializers / constructors) so the descriptor tables can use brace
// initialization under the Arduino C++11 toolchain. Read only after a
// successful paramDescriptor() call.
struct ParamDescriptor {
  uint16_t address;
  uint8_t length;          // 1, 2, or 4 bytes
  ParamRegion region;
  bool is_signed;          // two's-complement (e.g. HomingOffset)
  bool writable;           // false for read-only telemetry-style fields
};

// Resolve the descriptor for `param` on a servo using `table`. Returns false
// when the parameter has no equivalent on that control table (or the table is
// Unknown), leaving `out` untouched.
bool paramDescriptor(TableKind table, LogicalParam param, ParamDescriptor& out);

// True when a write to `param` on `table` requires torque to be disabled first
// (EEPROM region). Convenience over paramDescriptor().region.
bool paramNeedsTorqueOff(TableKind table, LogicalParam param);

// The pair of logical parameters SET_SERVO_LIMITS maps to for a table: the
// joint travel min/max bound. Legacy -> {CwAngleLimit, CcwAngleLimit};
// MX(2.0) -> {MinPositionLimit, MaxPositionLimit}. Returns false for Unknown.
bool servoLimitParams(TableKind table, LogicalParam& min_param,
                      LogicalParam& max_param);

}  // namespace dxl
