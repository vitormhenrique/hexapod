// Persistent robot configuration: compiled defaults, serialization, validation.
// Portable (no Arduino deps); unit-tested on the host (pio test -e native).

#include "config_schema.h"

#include <string.h>

namespace config {
namespace {

// Compile-time guard: the serialized payload must fit the config store slot.
static_assert(kConfigPayloadSize <= 2032,
              "config payload exceeds config-store kMaxPayload");

// ---- little-endian byte writers (advance the offset) ----------------------
void putU8(uint8_t* b, uint16_t& o, uint8_t v) { b[o++] = v; }
void putI8(uint8_t* b, uint16_t& o, int8_t v) { b[o++] = static_cast<uint8_t>(v); }

void putU16(uint8_t* b, uint16_t& o, uint16_t v) {
  b[o++] = static_cast<uint8_t>(v & 0xFF);
  b[o++] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
void putI16(uint8_t* b, uint16_t& o, int16_t v) {
  putU16(b, o, static_cast<uint16_t>(v));
}
void putU32(uint8_t* b, uint16_t& o, uint32_t v) {
  b[o++] = static_cast<uint8_t>(v & 0xFF);
  b[o++] = static_cast<uint8_t>((v >> 8) & 0xFF);
  b[o++] = static_cast<uint8_t>((v >> 16) & 0xFF);
  b[o++] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
void putI32(uint8_t* b, uint16_t& o, int32_t v) {
  putU32(b, o, static_cast<uint32_t>(v));
}

// ---- little-endian byte readers (advance the offset) ----------------------
uint8_t getU8(const uint8_t* b, uint16_t& o) { return b[o++]; }
int8_t getI8(const uint8_t* b, uint16_t& o) { return static_cast<int8_t>(b[o++]); }

uint16_t getU16(const uint8_t* b, uint16_t& o) {
  uint16_t v = static_cast<uint16_t>(b[o]) |
               (static_cast<uint16_t>(b[o + 1]) << 8);
  o += 2;
  return v;
}
int16_t getI16(const uint8_t* b, uint16_t& o) {
  return static_cast<int16_t>(getU16(b, o));
}
uint32_t getU32(const uint8_t* b, uint16_t& o) {
  uint32_t v = static_cast<uint32_t>(b[o]) |
               (static_cast<uint32_t>(b[o + 1]) << 8) |
               (static_cast<uint32_t>(b[o + 2]) << 16) |
               (static_cast<uint32_t>(b[o + 3]) << 24);
  o += 4;
  return v;
}
int32_t getI32(const uint8_t* b, uint16_t& o) {
  return static_cast<int32_t>(getU32(b, o));
}

// Per-leg coxa mount placement (body-centered frame B), HexNav IK ref section 3.
// 0.1 mm / 0.01 deg. Order: x, y, z, home yaw.
struct LegSeed {
  int16_t x_dmm;
  int16_t y_dmm;
  int16_t z_dmm;
  int16_t yaw_cdeg;
};
constexpr LegSeed kLegSeeds[kNumLegs] = {
    {-656, -1156, -165, 13500},   // leg 1 rear-left
    {656, -1156, -165, -13500},   // leg 2 rear-right
    {698, 0, -165, -9000},        // leg 3 mid-right
    {656, 1156, -165, -4500},     // leg 4 front-right
    {-656, 1156, -165, 4500},     // leg 5 front-left
    {-698, 0, -165, 9000},        // leg 6 mid-left
};

// Left legs (1, 5, 6) -> index {0, 4, 5} use sign +1; right legs (2, 3, 4) ->
// index {1, 2, 3} use sign -1 (HexNav IK ref section 7).
bool isLeftLeg(uint8_t leg) { return leg == 0 || leg == 4 || leg == 5; }

}  // namespace

void defaultRobotConfig(RobotConfig& cfg) {
  cfg = RobotConfig{};  // zero-init via member defaults
  cfg.schema_version = kSchemaVersion;
  memset(cfg.robot_name, 0, sizeof(cfg.robot_name));
  strncpy(cfg.robot_name, "HexNav", sizeof(cfg.robot_name) - 1);

  // Shared link lengths (0.01 mm), HexNav IK ref section 4.
  cfg.links.coxa_cmm = 5608;   // L_COXA 56.08 mm
  cfg.links.femur_cmm = 6651;  // L_FEMUR 66.51 mm
  cfg.links.tibia_cmm = 2486;  // L_TIBIA 24.86 mm (calibrate toe on hw)

  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    cfg.legs[leg].mount_x_dmm = kLegSeeds[leg].x_dmm;
    cfg.legs[leg].mount_y_dmm = kLegSeeds[leg].y_dmm;
    cfg.legs[leg].mount_z_dmm = kLegSeeds[leg].z_dmm;
    cfg.legs[leg].mount_yaw_cdeg = kLegSeeds[leg].yaw_cdeg;
  }

  // Servo map. The array is indexed in (leg, joint) order, but the DXL bus ids
  // follow the robot wiring: coxa = 1..6, femur = 7..12, tibia = 13..18, i.e.
  // id = joint*kNumLegs + leg + 1. Conservative +/-90deg travel about 2048.
  for (uint8_t i = 0; i < kNumServos; ++i) {
    const uint8_t leg = i / kJointsPerLeg;
    const uint8_t joint = i % kJointsPerLeg;
    ServoConfig& s = cfg.servos[i];
    s.id = static_cast<uint8_t>(joint * kNumLegs + leg + 1);
    s.leg = leg;
    s.joint = joint;
    s.sign = isLeftLeg(leg) ? 1 : -1;
    s.trim_ticks = 0;
    s.min_tick = static_cast<uint16_t>(kServoCenterTick - 1024);  // 1024
    s.max_tick = static_cast<uint16_t>(kServoCenterTick + 1024);  // 3072
  }

  // Gait defaults (HexNav IK ref section 5: 40 mm ride height).
  cfg.gait.body_height_mm = 40;
  cfg.gait.stride_len_mm = 60;
  cfg.gait.step_height_mm = 30;
  cfg.gait.duty_x255 = 128;   // 0.5 duty (tripod)
  cfg.gait.speed_x255 = 128;  // half speed
  cfg.gait.gait = static_cast<uint8_t>(GaitId::Stand);

  // Foot sensors: disabled until calibrated.
  for (uint8_t f = 0; f < kNumFootSensors; ++f) {
    cfg.feet[f] = FootSensorCal{};
  }

  // Conservative baseline: only sensor polling defaults on (so present boards
  // stream raw data); all richer/safety features stay off until hardware,
  // calibration, and an explicit request enable them (mirrors the protocol
  // kFeatureDefaultEnabled set so adopting this config preserves polling).
  cfg.feature_defaults = kFeatureDefaultMask;
}

uint16_t serializeRobotConfig(const RobotConfig& cfg, uint8_t* out,
                              uint16_t max_len) {
  if (max_len < kConfigPayloadSize) return 0;
  uint16_t o = 0;

  putU16(out, o, cfg.schema_version);
  memcpy(&out[o], cfg.robot_name, kRobotNameLen);
  o += kRobotNameLen;

  putU16(out, o, cfg.links.coxa_cmm);
  putU16(out, o, cfg.links.femur_cmm);
  putU16(out, o, cfg.links.tibia_cmm);

  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    putI16(out, o, cfg.legs[leg].mount_x_dmm);
    putI16(out, o, cfg.legs[leg].mount_y_dmm);
    putI16(out, o, cfg.legs[leg].mount_z_dmm);
    putI16(out, o, cfg.legs[leg].mount_yaw_cdeg);
  }

  for (uint8_t i = 0; i < kNumServos; ++i) {
    const ServoConfig& s = cfg.servos[i];
    putU8(out, o, s.id);
    putU8(out, o, s.leg);
    putU8(out, o, s.joint);
    putI8(out, o, s.sign);
    putI16(out, o, s.trim_ticks);
    putU16(out, o, s.min_tick);
    putU16(out, o, s.max_tick);
  }

  putU16(out, o, cfg.gait.body_height_mm);
  putU16(out, o, cfg.gait.stride_len_mm);
  putU16(out, o, cfg.gait.step_height_mm);
  putU8(out, o, cfg.gait.duty_x255);
  putU8(out, o, cfg.gait.speed_x255);
  putU8(out, o, cfg.gait.gait);

  for (uint8_t f = 0; f < kNumFootSensors; ++f) {
    putI32(out, o, cfg.feet[f].pressure_baseline);
    putU16(out, o, cfg.feet[f].near_thresh);
    putU16(out, o, cfg.feet[f].touch_thresh);
    putU16(out, o, cfg.feet[f].load_thresh);
    putU8(out, o, cfg.feet[f].enabled);
  }

  putU32(out, o, cfg.feature_defaults);

  return o;  // == kConfigPayloadSize
}

bool deserializeRobotConfig(const uint8_t* in, uint16_t len, RobotConfig& out) {
  if (len != kConfigPayloadSize) return false;
  uint16_t o = 0;

  out = RobotConfig{};
  out.schema_version = getU16(in, o);
  if (out.schema_version != kSchemaVersion) return false;

  memcpy(out.robot_name, &in[o], kRobotNameLen);
  out.robot_name[kRobotNameLen - 1] = '\0';  // ensure NUL termination
  o += kRobotNameLen;

  out.links.coxa_cmm = getU16(in, o);
  out.links.femur_cmm = getU16(in, o);
  out.links.tibia_cmm = getU16(in, o);

  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    out.legs[leg].mount_x_dmm = getI16(in, o);
    out.legs[leg].mount_y_dmm = getI16(in, o);
    out.legs[leg].mount_z_dmm = getI16(in, o);
    out.legs[leg].mount_yaw_cdeg = getI16(in, o);
  }

  for (uint8_t i = 0; i < kNumServos; ++i) {
    ServoConfig& s = out.servos[i];
    s.id = getU8(in, o);
    s.leg = getU8(in, o);
    s.joint = getU8(in, o);
    s.sign = getI8(in, o);
    s.trim_ticks = getI16(in, o);
    s.min_tick = getU16(in, o);
    s.max_tick = getU16(in, o);
  }

  out.gait.body_height_mm = getU16(in, o);
  out.gait.stride_len_mm = getU16(in, o);
  out.gait.step_height_mm = getU16(in, o);
  out.gait.duty_x255 = getU8(in, o);
  out.gait.speed_x255 = getU8(in, o);
  out.gait.gait = getU8(in, o);

  for (uint8_t f = 0; f < kNumFootSensors; ++f) {
    out.feet[f].pressure_baseline = getI32(in, o);
    out.feet[f].near_thresh = getU16(in, o);
    out.feet[f].touch_thresh = getU16(in, o);
    out.feet[f].load_thresh = getU16(in, o);
    out.feet[f].enabled = getU8(in, o);
  }

  out.feature_defaults = getU32(in, o);

  return o == len;
}

bool validateRobotConfig(const RobotConfig& cfg) {
  if (cfg.schema_version != kSchemaVersion) return false;

  // Link lengths must be non-zero (IK divides by/uses them).
  if (cfg.links.coxa_cmm == 0 || cfg.links.femur_cmm == 0 ||
      cfg.links.tibia_cmm == 0) {
    return false;
  }

  // Gait selection must be a known gait, and the persisted gait defaults must
  // sit inside the gait engine's safe envelope -- not merely be clampable to it.
  // A persisted default the engine would have to clamp back is a configuration
  // error, not a safe baseline (lmt.8 / audit 22l.1).
  if (cfg.gait.gait > static_cast<uint8_t>(GaitId::Crawl)) return false;
  if (cfg.gait.body_height_mm < kMinGaitBodyHeightMm ||
      cfg.gait.body_height_mm > kMaxGaitBodyHeightMm) {
    return false;
  }
  if (cfg.gait.stride_len_mm > kMaxGaitStrideMm) return false;
  if (cfg.gait.step_height_mm > kMaxGaitStepMm) return false;

  // feature_defaults may only request known features; an unknown bit means the
  // payload was produced by a newer/garbage schema and must not be trusted.
  if ((cfg.feature_defaults & ~kKnownFeatureBits) != 0u) return false;

  // Servo map: ids unique + in 1..253, leg/joint in range, sign +/-1,
  // min < max, ticks within device range. Also require each (leg, joint) slot
  // to be covered exactly once.
  bool joint_seen[kNumLegs][kJointsPerLeg] = {{false}};
  for (uint8_t i = 0; i < kNumServos; ++i) {
    const ServoConfig& s = cfg.servos[i];

    if (s.id == 0 || s.id > 253) return false;
    if (s.leg >= kNumLegs) return false;
    if (s.joint >= kJointsPerLeg) return false;
    if (s.sign != 1 && s.sign != -1) return false;
    if (s.min_tick >= s.max_tick) return false;
    if (s.max_tick > kServoMaxTick) return false;

    if (joint_seen[s.leg][s.joint]) return false;  // duplicate slot
    joint_seen[s.leg][s.joint] = true;

    for (uint8_t j = 0; j < i; ++j) {
      if (cfg.servos[j].id == s.id) return false;  // duplicate id
    }
  }
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    for (uint8_t j = 0; j < kJointsPerLeg; ++j) {
      if (!joint_seen[leg][j]) return false;  // missing slot
    }
  }

  // Enabled foot sensors must carry a complete, ordered calibration: all three
  // thresholds set and LOADED at or above TOUCH. The contact estimator escalates
  // NEAR (proximity) -> TOUCH -> LOADED (both pressure-delta), so a zero or
  // inverted pressure threshold would never fire or would misclassify load.
  // near_thresh is a proximity reading (different sensor/units), so it is only
  // required to be non-zero, not ordered against the pressure thresholds.
  // Disabled feet carry no calibration and are not constrained.
  for (uint8_t f = 0; f < kNumFootSensors; ++f) {
    const FootSensorCal& c = cfg.feet[f];
    if (!c.enabled) continue;
    if (c.near_thresh == 0 || c.touch_thresh == 0 || c.load_thresh == 0) {
      return false;
    }
    if (c.load_thresh < c.touch_thresh) return false;
  }

  return true;
}

}  // namespace config
