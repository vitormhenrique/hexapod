// Portable telemetry payload encoders. See telemetry_encode.h.

#include "protocol/telemetry_encode.h"

#include <math.h>

#include "config/config_schema.h"

namespace protocol {
namespace {

inline uint16_t put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  return 2;
}

inline uint16_t put32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
  return 4;
}

// Convert a joint angle (radians) to the wire int16 centidegree, saturating to
// the int16 range so a wild reading cannot wrap.
inline uint16_t angleCentideg(float rad) {
  long centideg = lroundf(rad * dxl::kRadToDeg * 100.0f);
  if (centideg > 32767) centideg = 32767;
  if (centideg < -32768) centideg = -32768;
  return static_cast<uint16_t>(static_cast<int16_t>(centideg));
}

}  // namespace

uint16_t encodeServoStatus(const dxl::ServoStatus* servos, uint8_t count,
                           uint8_t* out) {
  uint16_t o = 0;
  out[o++] = count;
  for (uint8_t i = 0; i < count; ++i) {
    const dxl::ServoStatus& s = servos[i];
    out[o++] = s.id;
    o += put32(&out[o], static_cast<uint32_t>(s.present_position));
    o += put16(&out[o], static_cast<uint16_t>(s.present_velocity));
    o += put16(&out[o], static_cast<uint16_t>(s.present_load));
    o += put16(&out[o], s.present_voltage_mv);
    out[o++] = static_cast<uint8_t>(s.present_temperature_c);
    out[o++] = s.hardware_error;
    out[o++] = s.torque_enabled ? 1 : 0;
  }
  return o;
}

uint16_t encodeJointState(const dxl::ServoMap& map,
                          const dxl::ServoStatus* servos, uint8_t count,
                          uint8_t* out) {
  uint16_t o = 0;
  uint8_t* countp = &out[o++];
  uint8_t emitted = 0;
  for (uint8_t i = 0; i < count; ++i) {
    const dxl::ServoStatus& s = servos[i];
    const config::ServoConfig* sc = map.servoForId(s.id);
    if (sc == nullptr) continue;  // skip servos not in the map
    // Single-turn present position; clamp into the device range so a multi-turn
    // or stale read cannot wrap the mapped angle.
    int32_t raw = s.present_position;
    if (raw < 0) raw = 0;
    if (raw > config::kServoMaxTick) raw = config::kServoMaxTick;
    const float rad =
        map.tickToAngle(sc->leg, sc->joint, static_cast<uint16_t>(raw));
    out[o++] = sc->leg;
    out[o++] = sc->joint;
    o += put16(&out[o], angleCentideg(rad));
    ++emitted;
  }
  *countp = emitted;
  return o;
}

uint16_t encodeServoGoals(const dxl::ServoMap& map,
                          const gait::PipelineJoint* joints, uint8_t count,
                          uint8_t* out) {
  uint16_t o = 0;
  uint8_t* countp = &out[o++];
  uint8_t emitted = 0;
  for (uint8_t i = 0; i < count; ++i) {
    const gait::PipelineJoint& jt = joints[i];
    const float rad = map.tickToAngle(jt.leg, jt.joint, jt.tick);
    out[o++] = jt.leg;
    out[o++] = jt.joint;
    o += put16(&out[o], angleCentideg(rad));
    out[o++] = jt.clamped ? 0x01 : 0x00;
    ++emitted;
  }
  *countp = emitted;
  return o;
}

uint16_t encodeServoGoals(const dxl::ServoMap& map,
                          const MaintTargetSet& targets, uint8_t* out) {
  uint16_t o = 0;
  uint8_t* countp = &out[o++];
  uint8_t emitted = 0;
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    for (uint8_t j = 0; j < config::kJointsPerLeg; ++j) {
      if (!targets.set[leg][j]) continue;
      if (map.servoFor(leg, j) == nullptr) continue;  // skip unmapped
      const float rad = map.tickToAngle(leg, j, targets.tick[leg][j]);
      out[o++] = leg;
      out[o++] = j;
      o += put16(&out[o], angleCentideg(rad));
      out[o++] = targets.clamped[leg][j] ? 0x01 : 0x00;
      ++emitted;
    }
  }
  *countp = emitted;
  return o;
}

uint16_t encodeLegState(const MaintTargetSet& targets, uint8_t* out) {
  uint16_t o = 0;
  uint8_t* countp = &out[o++];
  uint8_t emitted = 0;
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    if (!targets.leg_target_set[leg]) continue;
    out[o++] = leg;
    o += put16(&out[o], static_cast<uint16_t>(targets.foot_x_mm[leg]));
    o += put16(&out[o], static_cast<uint16_t>(targets.foot_y_mm[leg]));
    o += put16(&out[o], static_cast<uint16_t>(targets.foot_z_mm[leg]));
    uint8_t flags = 0;
    if (targets.leg_reachable[leg]) flags |= 0x01;
    if (targets.leg_clamped[leg]) flags |= 0x02;
    out[o++] = flags;
    ++emitted;
  }
  *countp = emitted;
  return o;
}

}  // namespace protocol
