// Native (host) unit tests for the portable persistent robot config schema.
// No Arduino/Wire dependencies.
//
// Run with: pio test -e native

#include <string.h>
#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/protocol/crc16.h"

using namespace config;

void test_defaults_are_valid() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  TEST_ASSERT_TRUE(validateRobotConfig(cfg));
  TEST_ASSERT_EQUAL_UINT16(kSchemaVersion, cfg.schema_version);
  TEST_ASSERT_EQUAL_STRING("HexNav", cfg.robot_name);
}

void test_default_servo_map() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);

  // 18 servos in (leg, joint) array order. DXL ids follow the wiring
  // leg-major: leg1 = coxa 1 / femur 2 / tibia 3, leg2 = 4/5/6, ...
  // leg6 = 16/17/18 (id = leg*3 + joint + 1).
  for (uint8_t i = 0; i < kNumServos; ++i) {
    const uint8_t leg = i / kJointsPerLeg;
    const uint8_t joint = i % kJointsPerLeg;
    TEST_ASSERT_EQUAL_UINT8(leg * kJointsPerLeg + joint + 1, cfg.servos[i].id);
    TEST_ASSERT_EQUAL_UINT8(leg, cfg.servos[i].leg);
    TEST_ASSERT_EQUAL_UINT8(joint, cfg.servos[i].joint);
    const int16_t expected_trim =
      joint == static_cast<uint8_t>(JointRole::Femur)
        ? kDefaultFemurTrimTicks
        : (joint == static_cast<uint8_t>(JointRole::Tibia)
             ? kDefaultTibiaTrimTicks
             : kDefaultCoxaTrimTicks);
    TEST_ASSERT_EQUAL_INT16(expected_trim, cfg.servos[i].trim_ticks);
    TEST_ASSERT_TRUE(cfg.servos[i].min_tick < cfg.servos[i].max_tick);
  }

  // Spot-check the published map (leg 1 -> index 0..2, leg 6 -> index 15..17).
  TEST_ASSERT_EQUAL_UINT8(1, cfg.servos[0].id);    // leg1 coxa
  TEST_ASSERT_EQUAL_UINT8(2, cfg.servos[1].id);    // leg1 femur
  TEST_ASSERT_EQUAL_UINT8(3, cfg.servos[2].id);    // leg1 tibia
  TEST_ASSERT_EQUAL_UINT8(16, cfg.servos[15].id);  // leg6 coxa
  TEST_ASSERT_EQUAL_UINT8(17, cfg.servos[16].id);  // leg6 femur
  TEST_ASSERT_EQUAL_UINT8(18, cfg.servos[17].id);  // leg6 tibia

  // All legs use sign +1 (hardware validation: right legs were mirrored).
  TEST_ASSERT_EQUAL_INT8(1, cfg.servos[0].sign);    // leg 0 (left)
  TEST_ASSERT_EQUAL_INT8(1, cfg.servos[3].sign);    // leg 1 (right)
  TEST_ASSERT_EQUAL_INT8(1, cfg.servos[6].sign);    // leg 2 (right)
  TEST_ASSERT_EQUAL_INT8(1, cfg.servos[9].sign);    // leg 3 (right)
  TEST_ASSERT_EQUAL_INT8(1, cfg.servos[12].sign);   // leg 4 (left)
  TEST_ASSERT_EQUAL_INT8(1, cfg.servos[15].sign);   // leg 5 (left)
}

void test_default_geometry() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  TEST_ASSERT_EQUAL_UINT16(5608, cfg.links.coxa_cmm);
  TEST_ASSERT_EQUAL_UINT16(6651, cfg.links.femur_cmm);
  TEST_ASSERT_EQUAL_UINT16(2486, cfg.links.tibia_cmm);
  // Stance/coxa geometry mirrors the gait-layer reference nominals (lmt.11).
  TEST_ASSERT_EQUAL_UINT16(12700, cfg.geometry.home_radius_cmm);
  TEST_ASSERT_EQUAL_INT16(-4455, cfg.geometry.home_foot_z_cmm);
  TEST_ASSERT_EQUAL_UINT16(2100, cfg.geometry.coxa_lift_cmm);
  // Leg 1 rear-left mount.
  TEST_ASSERT_EQUAL_INT16(-656, cfg.legs[0].mount_x_dmm);
  TEST_ASSERT_EQUAL_INT16(-1156, cfg.legs[0].mount_y_dmm);
  TEST_ASSERT_EQUAL_INT16(13500, cfg.legs[0].mount_yaw_cdeg);
  // All legs sit 16.5 mm below base_link.
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_EQUAL_INT16(-165, cfg.legs[leg].mount_z_dmm);
  }
}

void test_default_gait_and_features() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  TEST_ASSERT_EQUAL_UINT16(40, cfg.gait.body_height_mm);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GaitId::Stand), cfg.gait.gait);
  // Conservative defaults: only sensor polling on (so present boards stream raw
  // data); no other optional feature and no foot sensor enabled (lmt.7).
  TEST_ASSERT_EQUAL_UINT32(kFeatureDefaultMask, cfg.feature_defaults);
  for (uint8_t f = 0; f < kNumFootSensors; ++f) {
    TEST_ASSERT_EQUAL_UINT8(0, cfg.feet[f].enabled);
  }
  TEST_ASSERT_TRUE(validateRcInputCalibration(cfg.rc_input));
  TEST_ASSERT_EQUAL_UINT8(1, cfg.rc_input.channels[0].source);
  TEST_ASSERT_EQUAL_INT16(-1000, cfg.rc_input.channels[0].min_raw);
  TEST_ASSERT_EQUAL_INT16(0, cfg.rc_input.channels[0].center_raw);
  TEST_ASSERT_EQUAL_INT16(1000, cfg.rc_input.channels[0].max_raw);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(RcChannelType::CenteredAnalog),
      cfg.rc_input.channels[0].type);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(RcChannelType::UnipolarAnalog),
      cfg.rc_input.channels[4].type);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(RcChannelType::RelativeEncoder),
      cfg.rc_input.channels[6].type);
  TEST_ASSERT_EQUAL_UINT8(13, cfg.rc_input.channels[0].deadband_x255);
  TEST_ASSERT_EQUAL_UINT16(60, cfg.rc_input.channels[0].filter_tau_ms);
  TEST_ASSERT_EQUAL_UINT16(120, cfg.rc_input.channels[4].filter_tau_ms);
  TEST_ASSERT_EQUAL_UINT8(0, cfg.rc_input.channels[0].expo_x255);
  TEST_ASSERT_TRUE(validateBodyCommandLimits(cfg.body_command));
  TEST_ASSERT_EQUAL_UINT16(1200,
                           cfg.body_command.forward_accel_milli_per_s);
  TEST_ASSERT_EQUAL_UINT16(1800,
                           cfg.body_command.forward_decel_milli_per_s);
}

void test_serialize_size_matches_constant() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  uint8_t buf[kConfigPayloadSize];
  uint16_t n = serializeRobotConfig(cfg, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(kConfigPayloadSize, n);
}

void test_serialize_buffer_too_small() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  uint8_t buf[8];
  TEST_ASSERT_EQUAL_UINT16(0, serializeRobotConfig(cfg, buf, sizeof(buf)));
}

void test_round_trip_preserves_fields() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  // Mutate a spread of fields to make the round-trip meaningful.
  strncpy(cfg.robot_name, "Bench-1", sizeof(cfg.robot_name) - 1);
  cfg.servos[7].trim_ticks = -37;
  cfg.servos[7].min_tick = 900;
  cfg.servos[7].max_tick = 3100;
  cfg.gait.stride_len_mm = 75;
  cfg.geometry.home_radius_cmm = 13050;
  cfg.geometry.home_foot_z_cmm = -5012;
  cfg.geometry.coxa_lift_cmm = 2375;
  cfg.feet[2].pressure_baseline = -123456;
  cfg.feet[2].near_thresh = 300;
  cfg.feet[2].touch_thresh = 4200;
  cfg.feet[2].load_thresh = 5000;  // LOADED >= TOUCH
  cfg.feet[2].enabled = 1;
  cfg.rc_input.channels[1].min_raw = -900;
  cfg.rc_input.channels[1].center_raw = 25;
  cfg.rc_input.channels[1].max_raw = 975;
  cfg.rc_input.channels[1].reversed = 1;
  cfg.rc_input.channels[1].filter_tau_ms = 60;
  cfg.body_command.forward_accel_milli_per_s = 900;
  cfg.body_command.pose_rotation_rate_millirad_per_s = 600;
  cfg.feature_defaults = kFeatSensorPolling | kFeatPassivePoseStream;

  uint8_t buf[kConfigPayloadSize];
  uint16_t n = serializeRobotConfig(cfg, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(kConfigPayloadSize, n);

  RobotConfig got;
  TEST_ASSERT_TRUE(deserializeRobotConfig(buf, n, got));

  TEST_ASSERT_EQUAL_STRING("Bench-1", got.robot_name);
  TEST_ASSERT_EQUAL_INT16(-37, got.servos[7].trim_ticks);
  TEST_ASSERT_EQUAL_UINT16(900, got.servos[7].min_tick);
  TEST_ASSERT_EQUAL_UINT16(3100, got.servos[7].max_tick);
  TEST_ASSERT_EQUAL_UINT16(75, got.gait.stride_len_mm);
  TEST_ASSERT_EQUAL_UINT16(13050, got.geometry.home_radius_cmm);
  TEST_ASSERT_EQUAL_INT16(-5012, got.geometry.home_foot_z_cmm);
  TEST_ASSERT_EQUAL_UINT16(2375, got.geometry.coxa_lift_cmm);
  TEST_ASSERT_EQUAL_INT32(-123456, got.feet[2].pressure_baseline);
  TEST_ASSERT_EQUAL_UINT16(4200, got.feet[2].touch_thresh);
  TEST_ASSERT_EQUAL_UINT8(1, got.feet[2].enabled);
  TEST_ASSERT_EQUAL_INT16(-900, got.rc_input.channels[1].min_raw);
  TEST_ASSERT_EQUAL_INT16(25, got.rc_input.channels[1].center_raw);
  TEST_ASSERT_EQUAL_INT16(975, got.rc_input.channels[1].max_raw);
  TEST_ASSERT_EQUAL_UINT8(1, got.rc_input.channels[1].reversed);
  TEST_ASSERT_EQUAL_UINT16(60, got.rc_input.channels[1].filter_tau_ms);
  TEST_ASSERT_EQUAL_UINT16(900, got.body_command.forward_accel_milli_per_s);
  TEST_ASSERT_EQUAL_UINT16(600,
                           got.body_command.pose_rotation_rate_millirad_per_s);
  TEST_ASSERT_EQUAL_UINT32(kFeatSensorPolling | kFeatPassivePoseStream,
                           got.feature_defaults);
  TEST_ASSERT_TRUE(validateRobotConfig(got));
}

void test_deserialize_rejects_bad_length() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  uint8_t buf[kConfigPayloadSize];
  serializeRobotConfig(cfg, buf, sizeof(buf));
  RobotConfig got;
  TEST_ASSERT_FALSE(deserializeRobotConfig(buf, kConfigPayloadSize - 1, got));
}

void test_deserialize_rejects_bad_version() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  uint8_t buf[kConfigPayloadSize];
  serializeRobotConfig(cfg, buf, sizeof(buf));
  buf[0] = 0xEE;  // corrupt schema_version low byte
  buf[1] = 0xEE;
  RobotConfig got;
  TEST_ASSERT_FALSE(deserializeRobotConfig(buf, kConfigPayloadSize, got));
}

void test_deserialize_migrates_v3_payload_with_default_rc_and_body_limits() {
  RobotConfig current;
  defaultRobotConfig(current);
  current.servos[4].trim_ticks = -27;
  current.gait.stride_len_mm = 71;

    uint8_t v5[kConfigPayloadSize];
  TEST_ASSERT_EQUAL_UINT16(kConfigPayloadSize,
                 serializeRobotConfig(current, v5, sizeof(v5)));

  constexpr uint16_t kRcOffset =
      kLegacyConfigPayloadSizeV3 -
      kNumFootSensors * (4 + 2 + 2 + 2 + 1) - 4;
    constexpr uint16_t kRcBytes =
      kNumRcAnalogInputs * (1 + 1 + 2 + 2 + 2 + 1 + 1 + 1 + 2 + 2);
    constexpr uint16_t kBodyLimitBytes = 14 * 2;
  uint8_t v3[kLegacyConfigPayloadSizeV3];
    memcpy(v3, v5, kRcOffset);
  memcpy(&v3[kRcOffset],
       &v5[kRcOffset + kRcBytes + kBodyLimitBytes],
         kLegacyConfigPayloadSizeV3 - kRcOffset);
    v3[0] = static_cast<uint8_t>(kLegacySchemaVersionV3 & 0xFF);
    v3[1] = static_cast<uint8_t>(kLegacySchemaVersionV3 >> 8);

  RobotConfig migrated;
  TEST_ASSERT_TRUE(deserializeRobotConfig(v3, sizeof(v3), migrated));
  TEST_ASSERT_EQUAL_UINT16(kSchemaVersion, migrated.schema_version);
  TEST_ASSERT_EQUAL_INT16(-27, migrated.servos[4].trim_ticks);
  TEST_ASSERT_EQUAL_UINT16(71, migrated.gait.stride_len_mm);
  TEST_ASSERT_TRUE(validateRcInputCalibration(migrated.rc_input));
  TEST_ASSERT_TRUE(validateBodyCommandLimits(migrated.body_command));
  TEST_ASSERT_TRUE(validateRobotConfig(migrated));
}

void test_deserialize_migrates_v4_payload_with_default_body_limits() {
  RobotConfig current;
  defaultRobotConfig(current);
  current.rc_input.channels[0].filter_tau_ms = 80;
  current.servos[3].trim_ticks = -19;
  uint8_t v5[kConfigPayloadSize];
  TEST_ASSERT_EQUAL_UINT16(kConfigPayloadSize,
                           serializeRobotConfig(current, v5, sizeof(v5)));

  constexpr uint16_t kRcOffset =
      kLegacyConfigPayloadSizeV3 -
      kNumFootSensors * (4 + 2 + 2 + 2 + 1) - 4;
  constexpr uint16_t kRcBytes =
      kNumRcAnalogInputs * (1 + 1 + 2 + 2 + 2 + 1 + 1 + 1 + 2 + 2);
  constexpr uint16_t kBodyLimitBytes = 14 * 2;
  uint8_t v4[kLegacyConfigPayloadSizeV4];
  memcpy(v4, v5, kRcOffset + kRcBytes);
  memcpy(&v4[kRcOffset + kRcBytes],
         &v5[kRcOffset + kRcBytes + kBodyLimitBytes],
         kLegacyConfigPayloadSizeV4 - (kRcOffset + kRcBytes));
  v4[0] = static_cast<uint8_t>(kLegacySchemaVersionV4 & 0xFF);
  v4[1] = static_cast<uint8_t>(kLegacySchemaVersionV4 >> 8);

  RobotConfig migrated;
  TEST_ASSERT_TRUE(deserializeRobotConfig(v4, sizeof(v4), migrated));
  TEST_ASSERT_EQUAL_UINT16(kSchemaVersion, migrated.schema_version);
  TEST_ASSERT_EQUAL_INT16(-19, migrated.servos[3].trim_ticks);
  TEST_ASSERT_EQUAL_UINT16(80, migrated.rc_input.channels[0].filter_tau_ms);
  TEST_ASSERT_TRUE(validateBodyCommandLimits(migrated.body_command));
}

void test_validate_rejects_duplicate_id() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  cfg.servos[5].id = cfg.servos[4].id;  // duplicate
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));
}

void test_validate_rejects_bad_ranges() {
  RobotConfig cfg;

  defaultRobotConfig(cfg);
  cfg.servos[0].sign = 0;  // must be +/-1
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.servos[0].min_tick = cfg.servos[0].max_tick;  // min < max required
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.servos[0].max_tick = kServoMaxTick + 1;  // out of device range
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.links.femur_cmm = 0;  // zero link length
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.gait.body_height_mm = 0;  // zero ride height
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));
}

// lmt.11: a zero home stance radius is a degenerate stance (the IK rest offset
// and neutral foot collapse onto the coxa axis) and must be rejected.
void test_validate_rejects_zero_home_radius() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  cfg.geometry.home_radius_cmm = 0;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));
}

void test_validate_rejects_unreachable_home_stance() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  cfg.geometry.home_radius_cmm = 30000;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  // Exact annulus boundaries remain usable neutral poses.
  defaultRobotConfig(cfg);
  cfg.geometry.home_radius_cmm = static_cast<uint16_t>(
      cfg.links.coxa_cmm + cfg.links.femur_cmm + cfg.links.tibia_cmm);
  cfg.geometry.home_foot_z_cmm = 0;
  TEST_ASSERT_TRUE(validateRobotConfig(cfg));
}

void test_validate_rejects_missing_joint_slot() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  // Point two servos at the same (leg, joint) slot, leaving another uncovered.
  cfg.servos[1].leg = cfg.servos[0].leg;
  cfg.servos[1].joint = cfg.servos[0].joint;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));
}

// lmt.8: gait defaults outside the engine's safe envelope are rejected (the
// engine would otherwise silently clamp them), and feature_defaults may only
// set known bits.
void test_validate_rejects_unsafe_gait_and_features() {
  RobotConfig cfg;

  defaultRobotConfig(cfg);
  cfg.gait.body_height_mm = kMaxGaitBodyHeightMm + 1;  // above foot-Z envelope
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.gait.body_height_mm = kMinGaitBodyHeightMm - 1;  // below safe floor
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.gait.stride_len_mm = kMaxGaitStrideMm + 1;  // beyond max stroke
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.gait.step_height_mm = kMaxGaitStepMm + 1;  // beyond max lift
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.feature_defaults = 1u << 5;  // undefined feature bit
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  // Boundary values stay valid.
  defaultRobotConfig(cfg);
  cfg.gait.body_height_mm = kMaxGaitBodyHeightMm;
  cfg.gait.stride_len_mm = kMaxGaitStrideMm;
  cfg.gait.step_height_mm = kMaxGaitStepMm;
  cfg.feature_defaults = kKnownFeatureBits;
  TEST_ASSERT_TRUE(validateRobotConfig(cfg));
}

// lmt.8: an enabled foot sensor must carry a complete, ordered pressure
// calibration; disabled feet are unconstrained.
void test_validate_rejects_bad_foot_calibration() {
  RobotConfig cfg;

  // Enabled but no thresholds set -> reject.
  defaultRobotConfig(cfg);
  cfg.feet[0].enabled = 1;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  // Enabled, near/touch set but load missing -> reject.
  defaultRobotConfig(cfg);
  cfg.feet[0].enabled = 1;
  cfg.feet[0].near_thresh = 300;
  cfg.feet[0].touch_thresh = 4000;
  cfg.feet[0].load_thresh = 0;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  // Enabled with LOADED below TOUCH -> reject (inverted ordering).
  defaultRobotConfig(cfg);
  cfg.feet[0].enabled = 1;
  cfg.feet[0].near_thresh = 300;
  cfg.feet[0].touch_thresh = 4000;
  cfg.feet[0].load_thresh = 3000;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  // Complete, ordered calibration -> valid.
  defaultRobotConfig(cfg);
  cfg.feet[0].enabled = 1;
  cfg.feet[0].near_thresh = 300;
  cfg.feet[0].touch_thresh = 4000;
  cfg.feet[0].load_thresh = 5000;
  TEST_ASSERT_TRUE(validateRobotConfig(cfg));

  // Disabled foot with zero thresholds stays valid (default case).
  defaultRobotConfig(cfg);
  cfg.feet[0].enabled = 0;
  TEST_ASSERT_TRUE(validateRobotConfig(cfg));
}

void test_validate_rejects_bad_rc_input_calibration() {
  RobotConfig cfg;

  defaultRobotConfig(cfg);
  cfg.rc_input.channels[0].center_raw = cfg.rc_input.channels[0].min_raw;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.rc_input.channels[1].source = cfg.rc_input.channels[0].source;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.rc_input.channels[4].type =
      static_cast<uint8_t>(RcChannelType::CenteredAnalog);
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.rc_input.channels[0].reversed = 2;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.rc_input.channels[0].filter_tau_ms = kRcFilterTauMaxMs + 1;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));
}

void test_validate_rejects_bad_body_command_limits() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  cfg.body_command.max_forward_milli = 0;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.body_command.yaw_decel_milli_per_s =
      kBodyCommandMaxAccelMilliPerS + 1;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));

  defaultRobotConfig(cfg);
  cfg.body_command.pose_translation_rate_mm_per_s = 0;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));
}

// Cross-check: the serialized default-config bytes must match the host
// reference byte-for-byte. The host generator (protocol/tests/gen_vectors.py)
// emits frames.json["config"]["default_payload_crc"] over the same payload, and
// the Python tests assert their encoder reproduces it; pinning the CRC here
// guarantees the C++ serializer and the Python config decoder agree on the
// exact wire layout the eax.4 host decoder consumes.
void test_default_payload_crc_matches_host_vector() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  uint8_t buf[kConfigPayloadSize];
  uint16_t n = serializeRobotConfig(cfg, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(kConfigPayloadSize, n);
  // frames.json config.default_payload_crc (CRC-16/CCITT-FALSE).
  TEST_ASSERT_EQUAL_UINT16(43665, protocol::crc16(buf, n));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_defaults_are_valid);
  RUN_TEST(test_default_servo_map);
  RUN_TEST(test_default_geometry);
  RUN_TEST(test_default_gait_and_features);
  RUN_TEST(test_serialize_size_matches_constant);
  RUN_TEST(test_serialize_buffer_too_small);
  RUN_TEST(test_round_trip_preserves_fields);
  RUN_TEST(test_deserialize_rejects_bad_length);
  RUN_TEST(test_deserialize_rejects_bad_version);
  RUN_TEST(test_deserialize_migrates_v3_payload_with_default_rc_and_body_limits);
  RUN_TEST(test_deserialize_migrates_v4_payload_with_default_body_limits);
  RUN_TEST(test_validate_rejects_duplicate_id);
  RUN_TEST(test_validate_rejects_bad_ranges);
  RUN_TEST(test_validate_rejects_zero_home_radius);
  RUN_TEST(test_validate_rejects_unreachable_home_stance);
  RUN_TEST(test_validate_rejects_missing_joint_slot);
  RUN_TEST(test_validate_rejects_unsafe_gait_and_features);
  RUN_TEST(test_validate_rejects_bad_foot_calibration);
  RUN_TEST(test_validate_rejects_bad_rc_input_calibration);
  RUN_TEST(test_validate_rejects_bad_body_command_limits);
  RUN_TEST(test_default_payload_crc_matches_host_vector);
  return UNITY_END();
}
