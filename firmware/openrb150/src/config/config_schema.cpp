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

// Per-leg coxa mount placement (body-centered frame B, x right / y forward /
// z up). Measured on the CAD (dimensions.md): coxa rotation centres sit on the
// body mid-plane (z = 0). 0.1 mm / 0.01 deg. Order: x, y, z, home yaw.
struct LegSeed {
  int16_t x_dmm;
  int16_t y_dmm;
  int16_t z_dmm;
  int16_t yaw_cdeg;
};
constexpr LegSeed kLegSeeds[kNumLegs] = {
    {-656, -1156, 0, 13500},   // leg 1 rear-left
    {656, -1156, 0, -13500},   // leg 2 rear-right
    {698, 0, 0, -9000},        // leg 3 mid-right
    {656, 1156, 0, -4500},     // leg 4 front-right
    {-656, 1156, 0, 4500},     // leg 5 front-left
    {-698, 0, 0, 9000},        // leg 6 mid-left
};

// Kinematic model + gait defaults measured on the CAD (dimensions.md).
//   L1 coxa   = 52.00 mm   hip-yaw axis -> femur axis (radial; same height)
//   L2 femur  = 66.51 mm   femur axis -> tibia axis
//   L3 tibia  = 117.16 mm  tibia (knee) axis -> foot tip
//   home foot = 126.75 mm radial, 131.73 mm below the coxa axis (centered
//               servos); body centre stands 131.73 mm above ground.
// Applied to fresh configs and to legacy schema migrations that predate the
// measured model. Servo id/sign/trim/limit calibration is set separately so
// migrations that must preserve measured servo calibration can skip it.
void applyRobotKinematicsProfile(RobotConfig& cfg) {
  cfg.links.coxa_cmm = 5200;
  cfg.links.femur_cmm = 6651;
  cfg.links.tibia_cmm = 11716;

  cfg.geometry.home_radius_cmm = 12675;
  cfg.geometry.home_foot_z_cmm = -13173;
  cfg.geometry.coxa_lift_cmm = 0;

  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    cfg.legs[leg].mount_x_dmm = kLegSeeds[leg].x_dmm;
    cfg.legs[leg].mount_y_dmm = kLegSeeds[leg].y_dmm;
    cfg.legs[leg].mount_z_dmm = kLegSeeds[leg].z_dmm;
    cfg.legs[leg].mount_yaw_cdeg = kLegSeeds[leg].yaw_cdeg;
  }

  cfg.gait.body_height_mm = 132;  // centered-servo standing height (131.73)
  cfg.gait.stride_len_mm = 60;
  cfg.gait.step_height_mm = 30;
  cfg.gait.duty_x255 = 159;   // Mark III 8-step tripod: 5/8 grounded
  cfg.gait.speed_x255 = 128;  // nominal keyframe cadence
  cfg.gait.gait = static_cast<uint8_t>(GaitId::Stand);
}

void applyRobotMotionProfile(RobotConfig& cfg) {
  applyRobotKinematicsProfile(cfg);

  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    for (uint8_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const uint8_t slot = static_cast<uint8_t>(leg * kJointsPerLeg + joint);
      ServoConfig& servo = cfg.servos[slot];
      servo.id = static_cast<uint8_t>(slot + 1);
      servo.leg = leg;
      servo.joint = joint;
      servo.sign = 1;
      servo.trim_ticks = 0;
      servo.min_tick = 1024;
      servo.max_tick = 3072;
    }
  }
}

// Flash-resident zero image. `cfg = RobotConfig{}` materializes a ~508-byte
// temporary on the caller's stack (this sits under the api/control task call
// chains); copy-assigning from this constant avoids the stack transient.
constexpr RobotConfig kZeroRobotConfig{};

}  // namespace

void defaultRobotConfig(RobotConfig& cfg) {
  cfg = kZeroRobotConfig;  // zero-init via member defaults
  cfg.schema_version = kSchemaVersion;
  memset(cfg.robot_name, 0, sizeof(cfg.robot_name));
  strncpy(cfg.robot_name, "HexNav", sizeof(cfg.robot_name) - 1);

  applyRobotMotionProfile(cfg);

  defaultRcInputCalibration(cfg.rc_input);
  defaultBodyCommandLimits(cfg.body_command);

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

  putU16(out, o, cfg.geometry.home_radius_cmm);
  putI16(out, o, cfg.geometry.home_foot_z_cmm);
  putU16(out, o, cfg.geometry.coxa_lift_cmm);

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

  for (uint8_t index = 0; index < kNumRcAnalogInputs; ++index) {
    const RcChannelCalibration& c = cfg.rc_input.channels[index];
    putU8(out, o, c.source);
    putU8(out, o, c.type);
    putI16(out, o, c.min_raw);
    putI16(out, o, c.center_raw);
    putI16(out, o, c.max_raw);
    putU8(out, o, c.reversed);
    putU8(out, o, c.deadband_x255);
    putU8(out, o, c.expo_x255);
    putU16(out, o, c.filter_tau_ms);
    putU16(out, o, c.switch_debounce_ms);
  }

  const BodyCommandLimits& body = cfg.body_command;
  putU16(out, o, body.max_forward_milli);
  putU16(out, o, body.max_reverse_milli);
  putU16(out, o, body.max_lateral_milli);
  putU16(out, o, body.max_yaw_milli);
  putU16(out, o, body.forward_accel_milli_per_s);
  putU16(out, o, body.forward_decel_milli_per_s);
  putU16(out, o, body.lateral_accel_milli_per_s);
  putU16(out, o, body.lateral_decel_milli_per_s);
  putU16(out, o, body.yaw_accel_milli_per_s);
  putU16(out, o, body.yaw_decel_milli_per_s);
  putU16(out, o, body.height_rise_mm_per_s);
  putU16(out, o, body.height_lower_mm_per_s);
  putU16(out, o, body.pose_translation_rate_mm_per_s);
  putU16(out, o, body.pose_rotation_rate_millirad_per_s);

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
  if (in == nullptr || len < 2) return false;
  uint16_t o = 0;

  out = kZeroRobotConfig;
  out.schema_version = getU16(in, o);
  const bool legacy_v3 = out.schema_version == kLegacySchemaVersionV3 &&
                         len == kLegacyConfigPayloadSizeV3;
  const bool legacy_v4 = out.schema_version == kLegacySchemaVersionV4 &&
                         len == kLegacyConfigPayloadSizeV4;
  const bool legacy_v5 = out.schema_version == kLegacySchemaVersionV5 &&
                         len == kConfigPayloadSize;
  const bool legacy_v6 = out.schema_version == kLegacySchemaVersionV6 &&
                         len == kConfigPayloadSize;
  const bool legacy_v7 = out.schema_version == kLegacySchemaVersionV7 &&
                         len == kConfigPayloadSize;
  const bool legacy_v8 = out.schema_version == kLegacySchemaVersionV8 &&
                         len == kConfigPayloadSize;
  if (!legacy_v3 && !legacy_v4 && !legacy_v5 && !legacy_v6 && !legacy_v7 &&
      !legacy_v8 &&
      (out.schema_version != kSchemaVersion || len != kConfigPayloadSize)) {
    return false;
  }

  memcpy(out.robot_name, &in[o], kRobotNameLen);
  out.robot_name[kRobotNameLen - 1] = '\0';  // ensure NUL termination
  o += kRobotNameLen;

  out.links.coxa_cmm = getU16(in, o);
  out.links.femur_cmm = getU16(in, o);
  out.links.tibia_cmm = getU16(in, o);

  out.geometry.home_radius_cmm = getU16(in, o);
  out.geometry.home_foot_z_cmm = getI16(in, o);
  out.geometry.coxa_lift_cmm = getU16(in, o);

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

  if (legacy_v3) {
    defaultRcInputCalibration(out.rc_input);
  } else {
    for (uint8_t index = 0; index < kNumRcAnalogInputs; ++index) {
      RcChannelCalibration& c = out.rc_input.channels[index];
      c.source = getU8(in, o);
      c.type = getU8(in, o);
      c.min_raw = getI16(in, o);
      c.center_raw = getI16(in, o);
      c.max_raw = getI16(in, o);
      c.reversed = getU8(in, o);
      c.deadband_x255 = getU8(in, o);
      c.expo_x255 = getU8(in, o);
      c.filter_tau_ms = getU16(in, o);
      c.switch_debounce_ms = getU16(in, o);
    }
  }

  if (legacy_v3 || legacy_v4) {
    defaultBodyCommandLimits(out.body_command);
  } else {
    BodyCommandLimits& body = out.body_command;
    body.max_forward_milli = getU16(in, o);
    body.max_reverse_milli = getU16(in, o);
    body.max_lateral_milli = getU16(in, o);
    body.max_yaw_milli = getU16(in, o);
    body.forward_accel_milli_per_s = getU16(in, o);
    body.forward_decel_milli_per_s = getU16(in, o);
    body.lateral_accel_milli_per_s = getU16(in, o);
    body.lateral_decel_milli_per_s = getU16(in, o);
    body.yaw_accel_milli_per_s = getU16(in, o);
    body.yaw_decel_milli_per_s = getU16(in, o);
    body.height_rise_mm_per_s = getU16(in, o);
    body.height_lower_mm_per_s = getU16(in, o);
    body.pose_translation_rate_mm_per_s = getU16(in, o);
    body.pose_rotation_rate_millirad_per_s = getU16(in, o);
  }

  for (uint8_t f = 0; f < kNumFootSensors; ++f) {
    out.feet[f].pressure_baseline = getI32(in, o);
    out.feet[f].near_thresh = getU16(in, o);
    out.feet[f].touch_thresh = getU16(in, o);
    out.feet[f].load_thresh = getU16(in, o);
    out.feet[f].enabled = getU8(in, o);
  }

  out.feature_defaults = getU32(in, o);

  if (o != len) return false;
  if (legacy_v6 || legacy_v7) {
    // Pre-verified-hardware profiles: replace the complete motion profile,
    // including the servo map.
    applyRobotMotionProfile(out);
  } else if (legacy_v3 || legacy_v4 || legacy_v5 || legacy_v8) {
    // These schemas carried verified servo maps (ids/signs/trims/limits) but
    // their kinematic model (links/home/mounts/gait) predates the measured
    // CAD (dimensions.md): they used the URDF tibia-frame reduction whose
    // 24.86 mm "tibia" is a mesh-frame offset, not the physical leg. Replace
    // only the kinematics; preserve the per-servo calibration (safety:
    // migration must never silently erase measured calibration).
    applyRobotKinematicsProfile(out);
  }
  // The in-memory config always carries the active schema after migration.
  out.schema_version = kSchemaVersion;
  return true;
}

void defaultRcInputCalibration(RcInputCalibration& calibration) {
  calibration = RcInputCalibration{};
  for (uint8_t index = 0; index < kNumRcAnalogInputs; ++index) {
    RcChannelCalibration& c = calibration.channels[index];
    c.source = static_cast<uint8_t>(index + 1u);
    if (index < 4) {
      c.type = static_cast<uint8_t>(RcChannelType::CenteredAnalog);
      c.min_raw = -1000;
      c.center_raw = 0;
      c.max_raw = 1000;
      c.deadband_x255 = 13;  // ~5%, matching the prior bridge default.
      c.filter_tau_ms = 60;
    } else if (index < 6) {
      c.type = static_cast<uint8_t>(RcChannelType::UnipolarAnalog);
      c.min_raw = 0;
      c.center_raw = 0;
      c.max_raw = 1000;
      c.filter_tau_ms = 120;
    } else {
      c.type = static_cast<uint8_t>(RcChannelType::RelativeEncoder);
      c.min_raw = 0;
      c.center_raw = 0;
      c.max_raw = 2047;
    }
  }
}

bool validateRcInputCalibration(const RcInputCalibration& calibration) {
  bool source_seen[kNumRcAnalogInputs] = {false};
  for (uint8_t index = 0; index < kNumRcAnalogInputs; ++index) {
    const RcChannelCalibration& c = calibration.channels[index];
    if (c.source == 0 || c.source > kNumRcAnalogInputs) return false;
    const uint8_t source_index = static_cast<uint8_t>(c.source - 1u);
    if (source_seen[source_index]) return false;
    source_seen[source_index] = true;

    if (c.type > static_cast<uint8_t>(RcChannelType::RelativeEncoder) ||
        c.reversed > 1 || c.deadband_x255 > 128 ||
        c.filter_tau_ms > kRcFilterTauMaxMs ||
        c.switch_debounce_ms > kRcSwitchDebounceMaxMs ||
        c.min_raw >= c.max_raw || c.center_raw < c.min_raw ||
        c.center_raw > c.max_raw) {
      return false;
    }

    const RcChannelType type = static_cast<RcChannelType>(c.type);
    if (source_index < 4 && type != RcChannelType::CenteredAnalog) return false;
    if (source_index >= 4 && source_index < 6 &&
        type != RcChannelType::UnipolarAnalog) {
      return false;
    }
    if (source_index >= 6 && type != RcChannelType::RelativeEncoder) {
      return false;
    }
    if (type == RcChannelType::CenteredAnalog &&
        (c.center_raw == c.min_raw || c.center_raw == c.max_raw)) {
      return false;
    }
  }
  return true;
}

void defaultBodyCommandLimits(BodyCommandLimits& limits) {
  limits = BodyCommandLimits{};
}

bool validateBodyCommandLimits(const BodyCommandLimits& limits) {
  if (limits.max_forward_milli == 0 ||
      limits.max_forward_milli > kBodyCommandMaxScaleMilli ||
      limits.max_reverse_milli == 0 ||
      limits.max_reverse_milli > kBodyCommandMaxScaleMilli ||
      limits.max_lateral_milli == 0 ||
      limits.max_lateral_milli > kBodyCommandMaxScaleMilli ||
      limits.max_yaw_milli == 0 ||
      limits.max_yaw_milli > kBodyCommandMaxScaleMilli) {
    return false;
  }
  const uint16_t accel_limits[] = {
      limits.forward_accel_milli_per_s, limits.forward_decel_milli_per_s,
      limits.lateral_accel_milli_per_s, limits.lateral_decel_milli_per_s,
      limits.yaw_accel_milli_per_s, limits.yaw_decel_milli_per_s};
  for (uint8_t index = 0; index < sizeof(accel_limits) / sizeof(accel_limits[0]);
       ++index) {
    if (accel_limits[index] == 0 ||
        accel_limits[index] > kBodyCommandMaxAccelMilliPerS) {
      return false;
    }
  }
  return limits.height_rise_mm_per_s > 0 &&
         limits.height_rise_mm_per_s <= kBodyHeightRateMaxMmPerS &&
         limits.height_lower_mm_per_s > 0 &&
         limits.height_lower_mm_per_s <= kBodyHeightRateMaxMmPerS &&
         limits.pose_translation_rate_mm_per_s > 0 &&
         limits.pose_translation_rate_mm_per_s <=
             kBodyPoseTranslationRateMaxMmPerS &&
         limits.pose_rotation_rate_millirad_per_s > 0 &&
         limits.pose_rotation_rate_millirad_per_s <=
             kBodyPoseRotationRateMaxMilliRadPerS;
}

bool validateRobotConfig(const RobotConfig& cfg) {
  if (cfg.schema_version != kSchemaVersion) return false;

  // Link lengths must be non-zero (IK divides by/uses them).
  if (cfg.links.coxa_cmm == 0 || cfg.links.femur_cmm == 0 ||
      cfg.links.tibia_cmm == 0) {
    return false;
  }

  // The home stance radius must be non-zero -- it places the neutral foot and
  // seeds the leg IK rest offset; a zero radius is a degenerate stance. The
  // foot-Z and coxa lift may legitimately be zero, so they are unconstrained.
  if (cfg.geometry.home_radius_cmm == 0) return false;

  // The all-zero joint pose is seeded from this neutral coxa-frame foot
  // target. Reject a configuration that places it outside the femur/tibia
  // annulus instead of accepting a calibration that the IK can only clamp.
  // All values remain in centi-mm here; use signed 64-bit intermediates so
  // even a syntactically valid max-width config cannot overflow the squares.
  const int64_t home_planar_cmm =
      static_cast<int64_t>(cfg.geometry.home_radius_cmm) -
      static_cast<int64_t>(cfg.links.coxa_cmm);
  const int64_t home_z_cmm = cfg.geometry.home_foot_z_cmm;
  const int64_t home_distance_sq =
      home_planar_cmm * home_planar_cmm + home_z_cmm * home_z_cmm;
  int64_t reach_min_cmm = static_cast<int64_t>(cfg.links.femur_cmm) -
                          static_cast<int64_t>(cfg.links.tibia_cmm);
  if (reach_min_cmm < 0) reach_min_cmm = -reach_min_cmm;
  const int64_t reach_max_cmm = static_cast<int64_t>(cfg.links.femur_cmm) +
                                static_cast<int64_t>(cfg.links.tibia_cmm);
  if (home_distance_sq < reach_min_cmm * reach_min_cmm ||
      home_distance_sq > reach_max_cmm * reach_max_cmm) {
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
  if (!validateRcInputCalibration(cfg.rc_input)) return false;
  if (!validateBodyCommandLimits(cfg.body_command)) return false;

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
