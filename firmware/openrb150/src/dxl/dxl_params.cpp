// DYNAMIXEL logical parameter map (portable, host-tested).
// See dxl_params.h for the addressing contract.

#include "dxl_params.h"

namespace dxl {
namespace {

constexpr ParamRegion kEe = ParamRegion::Eeprom;
constexpr ParamRegion kRam = ParamRegion::Ram;

bool legacyDescriptor(LogicalParam p, ParamDescriptor& d) {
  // Legacy MX-28 (Protocol 1.0) control table (AGENTS.md 4.2).
  switch (p) {
    case LogicalParam::Id:               d = {3, 1, kEe, false, true};  return true;
    case LogicalParam::BaudRate:         d = {4, 1, kEe, false, true};  return true;
    case LogicalParam::ReturnDelayTime:  d = {5, 1, kEe, false, true};  return true;
    case LogicalParam::CwAngleLimit:     d = {6, 2, kEe, false, true};  return true;
    case LogicalParam::CcwAngleLimit:    d = {8, 2, kEe, false, true};  return true;
    case LogicalParam::TemperatureLimit: d = {11, 1, kEe, false, true}; return true;
    case LogicalParam::MinVoltageLimit:  d = {12, 1, kEe, false, true}; return true;
    case LogicalParam::MaxVoltageLimit:  d = {13, 1, kEe, false, true}; return true;
    case LogicalParam::MaxTorque:        d = {14, 2, kEe, false, true}; return true;
    case LogicalParam::StatusReturnLevel:d = {16, 1, kEe, false, true}; return true;
    case LogicalParam::Shutdown:         d = {18, 1, kEe, false, true}; return true;
    case LogicalParam::TorqueEnable:     d = {24, 1, kRam, false, true}; return true;
    // Legacy PID gains are single bytes: D@26, I@27, P@28.
    case LogicalParam::PidD:             d = {26, 1, kRam, false, true}; return true;
    case LogicalParam::PidI:             d = {27, 1, kRam, false, true}; return true;
    case LogicalParam::PidP:             d = {28, 1, kRam, false, true}; return true;
    case LogicalParam::MovingSpeed:      d = {32, 2, kRam, false, true}; return true;
    case LogicalParam::TorqueLimit:      d = {34, 2, kRam, false, true}; return true;
    case LogicalParam::GoalAcceleration: d = {73, 1, kRam, false, true}; return true;
    default:
      return false;  // no legacy equivalent (e.g. Min/MaxPositionLimit, V2-only)
  }
}

bool v2Descriptor(LogicalParam p, ParamDescriptor& d) {
  // MX-28(2.0) (Protocol 2.0) control table (AGENTS.md 4.2 + ROBOTIS std table).
  switch (p) {
    case LogicalParam::Id:               d = {7, 1, kEe, false, true};   return true;
    case LogicalParam::BaudRate:         d = {8, 1, kEe, false, true};   return true;
    case LogicalParam::ReturnDelayTime:  d = {9, 1, kEe, false, true};   return true;
    case LogicalParam::HomingOffset:     d = {20, 4, kEe, true, true};   return true;
    case LogicalParam::TemperatureLimit: d = {31, 1, kEe, false, true};  return true;
    case LogicalParam::MaxVoltageLimit:  d = {32, 2, kEe, false, true};  return true;
    case LogicalParam::MinVoltageLimit:  d = {34, 2, kEe, false, true};  return true;
    case LogicalParam::MaxPositionLimit: d = {48, 4, kEe, false, true};  return true;
    case LogicalParam::MinPositionLimit: d = {52, 4, kEe, false, true};  return true;
    case LogicalParam::Shutdown:         d = {63, 1, kEe, false, true};  return true;
    case LogicalParam::TorqueEnable:     d = {64, 1, kRam, false, true}; return true;
    case LogicalParam::StatusReturnLevel:d = {68, 1, kRam, false, true}; return true;
    // MX(2.0) position PID gains are 2 bytes: D@80, I@82, P@84.
    case LogicalParam::PidD:             d = {80, 2, kRam, false, true}; return true;
    case LogicalParam::PidI:             d = {82, 2, kRam, false, true}; return true;
    case LogicalParam::PidP:             d = {84, 2, kRam, false, true}; return true;
    case LogicalParam::BusWatchdog:      d = {98, 1, kRam, false, true}; return true;
    case LogicalParam::ProfileAcceleration: d = {108, 4, kRam, false, true}; return true;
    case LogicalParam::ProfileVelocity:  d = {112, 4, kRam, false, true}; return true;
    default:
      return false;  // no V2 equivalent (e.g. CW/CCW angle limit, MaxTorque)
  }
}

}  // namespace

bool paramDescriptor(TableKind table, LogicalParam param, ParamDescriptor& out) {
  switch (table) {
    case TableKind::Mx28Legacy:
      return legacyDescriptor(param, out);
    case TableKind::Mx28V2:
      return v2Descriptor(param, out);
    case TableKind::Unknown:
    default:
      return false;
  }
}

bool paramNeedsTorqueOff(TableKind table, LogicalParam param) {
  ParamDescriptor d;
  if (!paramDescriptor(table, param, d)) {
    return false;
  }
  return d.region == ParamRegion::Eeprom;
}

bool servoLimitParams(TableKind table, LogicalParam& min_param,
                      LogicalParam& max_param) {
  switch (table) {
    case TableKind::Mx28Legacy:
      min_param = LogicalParam::CwAngleLimit;
      max_param = LogicalParam::CcwAngleLimit;
      return true;
    case TableKind::Mx28V2:
      min_param = LogicalParam::MinPositionLimit;
      max_param = LogicalParam::MaxPositionLimit;
      return true;
    case TableKind::Unknown:
    default:
      return false;
  }
}

}  // namespace dxl
