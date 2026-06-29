#pragma once

// ===========================================================================
// Arduino-free present-status snapshot POD for a single servo.
//
// Split out of dxl_bus.h (which pulls in Dynamixel2Arduino and is therefore
// target-only) so the portable telemetry encoders (protocol/telemetry_encode)
// and their host/native tests can consume ServoStatus without dragging the
// Arduino/DYNAMIXEL stack into the native build.
// ===========================================================================

#include <stdint.h>

namespace dxl {

// Present-status snapshot from a single servo (torque-off read).
struct ServoStatus {
  uint8_t id = 0;
  int32_t present_position = 0;   // raw ticks
  int32_t present_velocity = 0;   // raw (legacy Present Speed / 2.0 Present Velocity)
  int32_t present_load = 0;       // signed raw load (legacy) / 0.1% (MX 2.0)
  uint16_t present_voltage_mv = 0;  // millivolts (raw 0.1 V units * 100)
  int8_t present_temperature_c = 0;
  uint8_t hardware_error = 0;  // MX(2.0) Hardware Error Status; 0 on legacy
  bool torque_enabled = false;
  bool ok = false;  // true if at least the present position read succeeded
};

}  // namespace dxl
