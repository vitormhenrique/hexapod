#pragma once

// ===========================================================================
// Persistent robot configuration schema (portable, no Arduino deps).
//
// This is the single source of truth for everything the firmware must remember
// across power cycles (AGENTS.md 4.3, Phase 2 issue 22l.1):
//
//   * robot identity + schema version
//   * per-leg body geometry (coxa mount placement + home yaw)
//   * shared leg link lengths (coxa / femur / tibia)
//   * the 18-servo map: DXL id -> (leg, joint role), sign, trim, min/max ticks
//   * gait defaults (body height, stride, step height, duty, speed, gait id)
//   * per-foot contact-sensor calibration (baseline + thresholds)
//   * feature default bitmask
//
// The struct is held as a RAM shadow (RobotConfig) and (de)serialized to a
// compact, little-endian, version-tagged byte payload that the transactional
// config store (config_store.h) persists in the 24LC32 EEPROM. Serialization is
// explicit (field by field) so the on-wire layout is endian-stable and
// independent of compiler struct padding.
//
// Default values are the compiled SAFE fallback used when the EEPROM is missing
// or blank (AGENTS.md 1.3 / 4.3): geometry comes from the HexNav URDF kinematics
// reference, contact sensing is disabled until calibrated, and servo travel is
// clamped to a conservative +/-90deg about the 180deg center.
//
// This translation unit is deliberately free of Arduino includes so it can be
// unit-tested on the host (pio test -e native).
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace config {

// ---------------------------------------------------------------------------
// Fixed dimensions (AGENTS.md: 6 legs, 3 DOF each, 18 MX-28, 6 foot sensors).
// ---------------------------------------------------------------------------
constexpr uint8_t kNumLegs = 6;
constexpr uint8_t kJointsPerLeg = 3;
constexpr uint8_t kNumServos = kNumLegs * kJointsPerLeg;  // 18
constexpr uint8_t kNumFootSensors = 6;                    // mux channels 0..5
constexpr uint8_t kRobotNameLen = 16;                     // incl. NUL terminator

// Schema version for the serialized payload. Bump on any layout change so the
// loader can reject configs it does not understand and fall back to defaults.
// v2 (lmt.11): added the BodyGeometry block (home stance + coxa lift).
constexpr uint16_t kSchemaVersion = 2;

// MX-28: 4096 ticks/rev, center 180deg = tick 2048.
constexpr uint16_t kServoCenterTick = 2048;
constexpr uint16_t kServoMaxTick = 4095;

// ---------------------------------------------------------------------------
// Enums.
// ---------------------------------------------------------------------------
enum class JointRole : uint8_t {
  Coxa = 0,
  Femur = 1,
  Tibia = 2,
};

// Default gait selection persisted in config (matches gait engine, issue 22l.5).
enum class GaitId : uint8_t {
  Stand = 0,
  Sit = 1,
  Tripod = 2,
  Ripple = 3,
  Wave = 4,
  Crawl = 5,
};

// Feature default bits (mirrors AGENTS.md 1.3 feature flags; defaults conservative).
enum FeatureBit : uint32_t {
  kFeatFootContact = 1u << 0,
  kFeatTerrainLeveling = 1u << 1,
  kFeatSensorPolling = 1u << 2,
  kFeatPassivePoseStream = 1u << 3,
  kFeatJetsonControl = 1u << 4,
};

// Mask of every defined feature_defaults bit. validateRobotConfig rejects any
// config that sets a bit outside this mask, so a forward/garbage payload cannot
// silently request an undefined feature.
constexpr uint32_t kKnownFeatureBits = kFeatFootContact | kFeatTerrainLeveling |
                                       kFeatSensorPolling |
                                       kFeatPassivePoseStream | kFeatJetsonControl;

// Safe envelope for persisted gait defaults (validateRobotConfig). Mirrors the
// gait engine's runtime clamps (gait/gait_engine.h: kMaxStrideMm 80, kMaxStepMm
// 50, foot-Z floor -120 mm) without a layering dependency. A persisted default
// outside this envelope is a configuration error -- the engine would otherwise
// silently clamp it -- so it is rejected rather than stored as a "safe" default.
constexpr uint16_t kMaxGaitStrideMm = 80;
constexpr uint16_t kMaxGaitStepMm = 50;
constexpr uint16_t kMinGaitBodyHeightMm = 5;
constexpr uint16_t kMaxGaitBodyHeightMm = 120;

// ---------------------------------------------------------------------------
// Sub-structures.
// ---------------------------------------------------------------------------

// Per-leg body placement. Coxa mount position is in the body-centered frame B
// (HexNav IK ref section 3), stored in tenths of a millimetre to stay integer
// while keeping sub-mm precision. mount_yaw is the home hip-yaw azimuth in
// centidegrees (servo-180 pointing direction).
struct LegGeometry {
  int16_t mount_x_dmm = 0;    // 0.1 mm
  int16_t mount_y_dmm = 0;    // 0.1 mm
  int16_t mount_z_dmm = 0;    // 0.1 mm
  int16_t mount_yaw_cdeg = 0;  // 0.01 deg
};

// Shared leg link lengths (identical for all 6 legs, HexNav IK ref section 4),
// stored in hundredths of a millimetre.
struct LinkLengths {
  uint16_t coxa_cmm = 0;   // 0.01 mm  (L_COXA)
  uint16_t femur_cmm = 0;  // 0.01 mm  (L_FEMUR)
  uint16_t tibia_cmm = 0;  // 0.01 mm  (L_TIBIA, calibrate toe offset on hw)
};

// Stance/coxa geometry calibration shared by all six legs (HexNav IK ref
// sections 4/5/12). Persisted so the values measured on the assembled robot can
// override the compiled reference model -- the default neutral stance sits near
// the two-link reach boundary, so small measured offsets matter for HIL.
//   * home_radius / home_foot_z place the neutral foot in the coxa frame; the
//     leg IK bakes them into its rest offset (home pose -> all-zero angles).
//   * coxa_lift is the coxa joint axis height above the leg's body mount.
// Stored in hundredths of a millimetre; home_foot_z is signed (foot below hip).
struct BodyGeometry {
  uint16_t home_radius_cmm = 0;  // 0.01 mm, neutral foot radial distance
  int16_t home_foot_z_cmm = 0;   // 0.01 mm, neutral foot height (negative = down)
  uint16_t coxa_lift_cmm = 0;    // 0.01 mm, coxa axis lift above the body mount
};

// One servo's logical mapping + travel limits. `sign` is +1 or -1 (HexNav IK
// ref section 7). `trim_ticks` is a signed mechanical-zero offset added to the
// 180deg center. min/max ticks clamp the goal position after IK.
struct ServoConfig {
  uint8_t id = 0;                 // DXL bus id (1..253)
  uint8_t leg = 0;                // 0..kNumLegs-1
  uint8_t joint = 0;              // JointRole value
  int8_t sign = 1;                // +1 or -1
  int16_t trim_ticks = 0;         // signed offset added to center
  uint16_t min_tick = 0;          // lower goal clamp (0..4095)
  uint16_t max_tick = kServoMaxTick;  // upper goal clamp (0..4095)
};

// Default gait/body parameters used at boot and as command baselines.
struct GaitDefaults {
  uint16_t body_height_mm = 0;   // ride height (feet below hip mounts)
  uint16_t stride_len_mm = 0;    // nominal stride
  uint16_t step_height_mm = 0;   // swing clearance
  uint8_t duty_x255 = 0;         // duty factor * 255 (e.g. 0.5 -> 128)
  uint8_t speed_x255 = 0;        // speed scalar * 255
  uint8_t gait = 0;              // GaitId value
};

// Per-foot contact sensor calibration (Robotic Finger Sensor v2). Disabled by
// default until a calibration capture establishes a baseline (AGENTS.md 4.4).
struct FootSensorCal {
  int32_t pressure_baseline = 0;  // raw LPS25HB baseline
  uint16_t near_thresh = 0;       // proximity NEAR crossing
  uint16_t touch_thresh = 0;      // pressure-delta TOUCH crossing
  uint16_t load_thresh = 0;       // pressure-delta LOADED crossing
  uint8_t enabled = 0;            // 0 until calibrated
};

// ---------------------------------------------------------------------------
// Top-level persistent config (RAM shadow).
// ---------------------------------------------------------------------------
struct RobotConfig {
  uint16_t schema_version = kSchemaVersion;
  char robot_name[kRobotNameLen] = {0};
  LinkLengths links;
  BodyGeometry geometry;
  LegGeometry legs[kNumLegs];
  ServoConfig servos[kNumServos];
  GaitDefaults gait;
  FootSensorCal feet[kNumFootSensors];
  uint32_t feature_defaults = 0;
};

// Default feature-enable bitmask stored in RobotConfig.feature_defaults. Bit
// index == protocol Feature enum order (FootContact=0, TerrainLeveling=1,
// SensorPolling=2, JetsonControl=3, PassivePose=4). Only SensorPolling defaults
// on so present sensor boards stream raw data on boot; richer/safety features
// stay off until armed and explicitly requested. Mirrors the protocol layer's
// kFeatureDefaultEnabled baseline (feature_api.h) without a layering dependency,
// so adopting a freshly-defaulted config does not silently disable polling.
constexpr uint32_t kFeatureDefaultMask = (1u << 2);  // SensorPolling

// Serialized payload size (bytes) for the current schema. Static so callers can
// size buffers and so a compile-time check guards against payload overflow.
constexpr uint16_t kConfigPayloadSize =
    2                                  // schema_version
    + kRobotNameLen                    // robot_name
    + 3 * 2                            // links (3 x uint16)
    + (2 + 2 + 2)                      // body geometry (u16 radius, i16 z, u16 lift)
    + kNumLegs * (4 * 2)               // legs (4 x int16 each)
    + kNumServos * (1 + 1 + 1 + 1 + 2 + 2 + 2)  // servos (10 bytes each)
    + (2 + 2 + 2 + 1 + 1 + 1)          // gait (9 bytes)
    + kNumFootSensors * (4 + 2 + 2 + 2 + 1)     // feet (11 bytes each)
    + 4;                               // feature_defaults

// ---------------------------------------------------------------------------
// API.
// ---------------------------------------------------------------------------

// Populate `cfg` with the compiled SAFE defaults (HexNav geometry, conservative
// servo limits, contact sensing disabled).
void defaultRobotConfig(RobotConfig& cfg);

// Serialize `cfg` into `out` (capacity `max_len`) as a little-endian,
// version-tagged payload. Returns the number of bytes written, or 0 if the
// buffer is too small.
uint16_t serializeRobotConfig(const RobotConfig& cfg, uint8_t* out,
                              uint16_t max_len);

// Deserialize a payload produced by serializeRobotConfig. Returns false if the
// length is wrong or the schema version is unsupported.
bool deserializeRobotConfig(const uint8_t* in, uint16_t len, RobotConfig& out);

// Validate a config's invariants (unique servo ids, in-range legs/joints/ticks,
// non-zero link lengths and body height, non-zero home stance radius, sane
// signs). Returns true if safe to use. Does NOT mutate `cfg`.
bool validateRobotConfig(const RobotConfig& cfg);

}  // namespace config
