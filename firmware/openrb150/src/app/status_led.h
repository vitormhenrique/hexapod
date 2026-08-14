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

// 8-bit color for the onboard RGB status pixel (MicroMod carrier NeoPixel).
// Values are full-scale; the board driver applies global dimming. Plain
// aggregate (no member initializers) so brace-init works on the C++11 SAMD
// toolchain; `Rgb{}` value-initializes to black.
struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

Pattern patternFor(const Inputs& inputs);
bool ledOn(const Inputs& inputs, uint32_t now_ms);
uint8_t faultPulseCount(const Inputs& inputs);

// Base color for the current state/mode (independent of blink phase):
//   fault/estop/watchdog  red        boot/config load      white
//   disarmed              blue       arming (IDs/poses)    yellow / amber
//   stand-ready/RC manual green      contact terrain       chartreuse
//   Jetson assisted       violet     maintenance           magenta
//   passive pose stream   cyan
Rgb colorFor(const Inputs& inputs);

// Color with the ledOn() blink gate applied: {0,0,0} during off phases, the
// colorFor() value otherwise. This is the frame to hand to the board driver.
Rgb rgbFrame(const Inputs& inputs, uint32_t now_ms);

}  // namespace status_led
}  // namespace app
