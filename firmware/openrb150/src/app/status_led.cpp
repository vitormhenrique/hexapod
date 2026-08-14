#include "status_led.h"

namespace app {
namespace status_led {
namespace {

bool pulseTrain(uint32_t now_ms, uint8_t pulses, uint32_t cycle_ms) {
  const uint32_t phase = now_ms % cycle_ms;
  constexpr uint32_t kPulseSlotMs = 300;
  constexpr uint32_t kPulseOnMs = 100;
  return phase < static_cast<uint32_t>(pulses) * kPulseSlotMs &&
         (phase % kPulseSlotMs) < kPulseOnMs;
}

bool isArmedState(safety::State state) {
  return state == safety::State::StandReady ||
         state == safety::State::RcManual ||
         state == safety::State::ContactTerrain ||
         state == safety::State::JetsonAssisted;
}

}  // namespace

Pattern patternFor(const Inputs& inputs) {
  if (inputs.watchdog_stalled || inputs.state == safety::State::FaultSoft ||
      inputs.state == safety::State::FaultHard ||
      inputs.state == safety::State::Estop) {
    return Pattern::FaultCode;
  }
  if (inputs.state == safety::State::Boot ||
      inputs.state == safety::State::ConfigLoad) {
    return Pattern::BootFast;
  }
  if (inputs.state == safety::State::ArmingChecks) {
    if (!inputs.configured_servo_coverage) return Pattern::ArmingDiscoverIds;
    if (!inputs.all_servo_poses_known) return Pattern::ArmingReadPoses;
    return Pattern::BootFast;
  }
  if (isArmedState(inputs.state)) return Pattern::ArmedSolid;
  if (inputs.state == safety::State::MacMaintenance) {
    return Pattern::MaintenanceDouble;
  }
  if (inputs.state == safety::State::PassivePoseStream) {
    return Pattern::PassiveSlow;
  }
  return Pattern::DisarmedHeartbeat;
}

uint8_t faultPulseCount(const Inputs& inputs) {
  if (inputs.watchdog_stalled) {
    return static_cast<uint8_t>(safety::FaultReason::Watchdog);
  }
  const uint8_t reason = static_cast<uint8_t>(inputs.fault);
  if (reason >= static_cast<uint8_t>(safety::FaultReason::RcKill) &&
      reason <= static_cast<uint8_t>(safety::FaultReason::ArmingTimeout)) {
    return reason;
  }
  if (inputs.state == safety::State::FaultHard) return 3;
  if (inputs.state == safety::State::Estop) return 4;
  return 1;
}

bool ledOn(const Inputs& inputs, uint32_t now_ms) {
  switch (patternFor(inputs)) {
    case Pattern::BootFast:
      return (now_ms % 200u) < 100u;
    case Pattern::DisarmedHeartbeat:
      return (now_ms % 2000u) < 100u;
    case Pattern::ArmingDiscoverIds:
      return pulseTrain(now_ms, 2, 1600);
    case Pattern::ArmingReadPoses:
      return pulseTrain(now_ms, 3, 1800);
    case Pattern::ArmedSolid:
      return true;
    case Pattern::MaintenanceDouble:
      return pulseTrain(now_ms, 2, 2000);
    case Pattern::PassiveSlow:
      return (now_ms % 1000u) < 500u;
    case Pattern::FaultCode:
      return pulseTrain(now_ms, faultPulseCount(inputs), 3000);
  }
  return false;
}

Rgb colorFor(const Inputs& inputs) {
  switch (patternFor(inputs)) {
    case Pattern::FaultCode:
      return Rgb{255, 0, 0};  // red: fault / estop / watchdog stall
    case Pattern::BootFast:
      return Rgb{255, 255, 255};  // white: boot, config load, final checks
    case Pattern::DisarmedHeartbeat:
      return Rgb{0, 0, 255};  // blue: disarmed, waiting for arming
    case Pattern::ArmingDiscoverIds:
      return Rgb{255, 180, 0};  // yellow: discovering configured servo IDs
    case Pattern::ArmingReadPoses:
      return Rgb{255, 64, 0};  // amber: waiting for all present poses
    case Pattern::MaintenanceDouble:
      return Rgb{255, 0, 255};  // magenta: host maintenance lock
    case Pattern::PassiveSlow:
      return Rgb{0, 255, 255};  // cyan: torque-off passive pose stream
    case Pattern::ArmedSolid:
      break;  // armed sub-modes get distinct colors below
  }
  // Armed: differentiate the active control mode by hue.
  if (inputs.state == safety::State::ContactTerrain) {
    return Rgb{128, 255, 0};  // chartreuse: contact-aware terrain gait
  }
  if (inputs.state == safety::State::JetsonAssisted) {
    return Rgb{128, 0, 255};  // violet: Jetson autonomy authorized
  }
  return Rgb{0, 255, 0};  // green: stand-ready / RC manual
}

Rgb rgbFrame(const Inputs& inputs, uint32_t now_ms) {
  return ledOn(inputs, now_ms) ? colorFor(inputs) : Rgb{};
}

}  // namespace status_led
}  // namespace app
