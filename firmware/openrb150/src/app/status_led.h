#pragma once

#include <stdint.h>

#include "../safety/state_machine.h"

namespace app {
namespace status_led {

struct Inputs {
  safety::State state = safety::State::Boot;
  safety::FaultReason fault = safety::FaultReason::None;
  bool configured_servo_coverage = false;
  bool all_servo_poses_known = false;
  bool watchdog_stalled = false;
};

enum class Pattern : uint8_t {
  BootFast,
  DisarmedHeartbeat,
  ArmingDiscoverIds,
  ArmingReadPoses,
  ArmedSolid,
  MaintenanceDouble,
  PassiveSlow,
  FaultCode,
};

Pattern patternFor(const Inputs& inputs);
bool ledOn(const Inputs& inputs, uint32_t now_ms);
uint8_t faultPulseCount(const Inputs& inputs);

}  // namespace status_led
}  // namespace app
