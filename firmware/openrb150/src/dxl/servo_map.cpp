// Servo map + joint-limit enforcement (portable, host-tested).
// See servo_map.h for the conversion model.

#include "servo_map.h"

#include <math.h>

namespace dxl {

const config::ServoConfig* ServoMap::servoFor(uint8_t leg, uint8_t joint) const {
  for (uint8_t i = 0; i < config::kNumServos; ++i) {
    const config::ServoConfig& s = cfg_.servos[i];
    if (s.leg == leg && s.joint == joint) return &s;
  }
  return nullptr;
}

const config::ServoConfig* ServoMap::servoForId(uint8_t id) const {
  for (uint8_t i = 0; i < config::kNumServos; ++i) {
    if (cfg_.servos[i].id == id) return &cfg_.servos[i];
  }
  return nullptr;
}

JointCommand ServoMap::angleToTick(uint8_t leg, uint8_t joint,
                                   float angle_rad) const {
  JointCommand out;
  const config::ServoConfig* s = servoFor(leg, joint);
  if (s == nullptr) {
    out.unmapped = true;
    out.tick = config::kServoCenterTick;
    return out;
  }

  // angle (rad) -> servo offset in ticks, with per-servo sign.
  const float deg = angle_rad * kRadToDeg;
  const long offset = lroundf(deg * kTicksPerDeg);
  long raw = static_cast<long>(config::kServoCenterTick) +
             static_cast<long>(s->trim_ticks) +
             static_cast<long>(s->sign) * offset;

  // Clamp to the configured travel window first (the authoritative joint limit).
  const long lo = static_cast<long>(s->min_tick);
  const long hi = static_cast<long>(s->max_tick);
  if (raw < lo) {
    raw = lo;
    out.clamped_low = true;
  }
  if (raw > hi) {
    raw = hi;
    out.clamped_high = true;
  }

  // Defensive: never exceed the physical device range even if config is odd.
  if (raw < 0) {
    raw = 0;
    out.clamped_low = true;
  }
  if (raw > static_cast<long>(config::kServoMaxTick)) {
    raw = config::kServoMaxTick;
    out.clamped_high = true;
  }

  out.tick = static_cast<uint16_t>(raw);
  return out;
}

float ServoMap::tickToAngle(uint8_t leg, uint8_t joint, uint16_t tick) const {
  const config::ServoConfig* s = servoFor(leg, joint);
  if (s == nullptr) return 0.0f;
  const float offset_ticks = static_cast<float>(static_cast<int>(tick) -
                                                static_cast<int>(config::kServoCenterTick) -
                                                static_cast<int>(s->trim_ticks));
  const float deg = offset_ticks / kTicksPerDeg;
  return static_cast<float>(s->sign) * deg * kDegToRad;
}

}  // namespace dxl
