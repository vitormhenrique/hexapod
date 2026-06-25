#pragma once

// ===========================================================================
// Servo map + joint-limit enforcement (portable, no Arduino deps).
//
// Converts a logical per-leg joint target (a URDF-zero-relative angle, in
// radians, as produced by leg IK) into a DYNAMIXEL goal position in ticks,
// applying the validated robot config (config_schema.h) for that joint:
//
//   tick = center(2048) + trim_ticks + sign * round(deg(angle) * ticks/deg)
//
// then clamping to the configured [min_tick, max_tick] travel (and, defensively,
// to the device range [0, 4095]). Clamp flags are reported so callers (and
// telemetry, issue 22l.6) can see when a target was saturated.
//
// The inverse (tick -> angle) is provided for feedback / passive pose streaming
// (issue 22l: passive_joint_state): it applies the same sign/zero so a present
// position reads back as the logical joint angle.
//
// This is the final safety gate between IK output and the servo bus
// (AGENTS.md 5.2): every commanded tick passes through here. No heap; the map
// holds a const reference to the active RobotConfig.
//
// Mapping uses the MX-28 convention (HexNav IK reference section 7):
//   center 180deg = tick 2048, 4096 ticks/rev (0.088 deg/tick).
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"

namespace dxl {

// MX-28 position scale: 4096 ticks per full revolution.
constexpr float kTicksPerRev = 4096.0f;
constexpr float kTicksPerDeg = kTicksPerRev / 360.0f;  // ~11.3778
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kDegToRad = kPi / 180.0f;

// Result of converting one joint angle to a goal tick.
struct JointCommand {
  uint16_t tick = config::kServoCenterTick;  // clamped goal position
  bool clamped_low = false;   // hit min_tick (or device floor)
  bool clamped_high = false;  // hit max_tick (or device ceiling)
  bool unmapped = false;      // no servo configured for (leg, joint)
};

class ServoMap {
 public:
  explicit ServoMap(const config::RobotConfig& cfg) : cfg_(cfg) {}

  // Servo config for a (leg, joint) slot, or nullptr if none is mapped.
  const config::ServoConfig* servoFor(uint8_t leg, uint8_t joint) const;

  // Servo config for a DXL bus id, or nullptr if no servo uses that id.
  const config::ServoConfig* servoForId(uint8_t id) const;

  // Convert a URDF-zero-relative joint angle (radians) to a clamped goal tick
  // with clamp flags. If the slot is unmapped, returns unmapped=true and the
  // center tick.
  JointCommand angleToTick(uint8_t leg, uint8_t joint, float angle_rad) const;

  // Inverse: a present-position tick -> URDF-zero-relative joint angle (rad),
  // applying the same sign/zero. Returns 0 for an unmapped slot.
  float tickToAngle(uint8_t leg, uint8_t joint, uint16_t tick) const;

 private:
  const config::RobotConfig& cfg_;
};

}  // namespace dxl
