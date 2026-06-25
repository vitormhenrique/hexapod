// Native (host) unit tests for the portable persistent robot config schema.
// No Arduino/Wire dependencies.
//
// Run with: pio test -e native

#include <string.h>
#include <unity.h>

#include "../../src/config/config_schema.h"

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

  // 18 servos in (leg, joint) array order. DXL ids follow the wiring:
  // coxa = 1..6, femur = 7..12, tibia = 13..18 (id = joint*6 + leg + 1).
  for (uint8_t i = 0; i < kNumServos; ++i) {
    const uint8_t leg = i / kJointsPerLeg;
    const uint8_t joint = i % kJointsPerLeg;
    TEST_ASSERT_EQUAL_UINT8(joint * kNumLegs + leg + 1, cfg.servos[i].id);
    TEST_ASSERT_EQUAL_UINT8(leg, cfg.servos[i].leg);
    TEST_ASSERT_EQUAL_UINT8(joint, cfg.servos[i].joint);
    TEST_ASSERT_TRUE(cfg.servos[i].min_tick < cfg.servos[i].max_tick);
  }

  // Spot-check the published map (leg 1 -> index 0..2, leg 6 -> index 15..17).
  TEST_ASSERT_EQUAL_UINT8(1, cfg.servos[0].id);    // leg1 coxa
  TEST_ASSERT_EQUAL_UINT8(7, cfg.servos[1].id);    // leg1 femur
  TEST_ASSERT_EQUAL_UINT8(13, cfg.servos[2].id);   // leg1 tibia
  TEST_ASSERT_EQUAL_UINT8(6, cfg.servos[15].id);   // leg6 coxa
  TEST_ASSERT_EQUAL_UINT8(12, cfg.servos[16].id);  // leg6 femur
  TEST_ASSERT_EQUAL_UINT8(18, cfg.servos[17].id);  // leg6 tibia

  // Left legs (0,4,5) sign +1; right legs (1,2,3) sign -1.
  TEST_ASSERT_EQUAL_INT8(1, cfg.servos[0].sign);    // leg 0 (left)
  TEST_ASSERT_EQUAL_INT8(-1, cfg.servos[3].sign);   // leg 1 (right)
  TEST_ASSERT_EQUAL_INT8(-1, cfg.servos[6].sign);   // leg 2 (right)
  TEST_ASSERT_EQUAL_INT8(-1, cfg.servos[9].sign);   // leg 3 (right)
  TEST_ASSERT_EQUAL_INT8(1, cfg.servos[12].sign);   // leg 4 (left)
  TEST_ASSERT_EQUAL_INT8(1, cfg.servos[15].sign);   // leg 5 (left)
}

void test_default_geometry() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  TEST_ASSERT_EQUAL_UINT16(5608, cfg.links.coxa_cmm);
  TEST_ASSERT_EQUAL_UINT16(6651, cfg.links.femur_cmm);
  TEST_ASSERT_EQUAL_UINT16(2486, cfg.links.tibia_cmm);
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
  // Conservative defaults: no optional features on, no foot sensor enabled.
  TEST_ASSERT_EQUAL_UINT32(0u, cfg.feature_defaults);
  for (uint8_t f = 0; f < kNumFootSensors; ++f) {
    TEST_ASSERT_EQUAL_UINT8(0, cfg.feet[f].enabled);
  }
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
  cfg.feet[2].pressure_baseline = -123456;
  cfg.feet[2].touch_thresh = 4200;
  cfg.feet[2].enabled = 1;
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
  TEST_ASSERT_EQUAL_INT32(-123456, got.feet[2].pressure_baseline);
  TEST_ASSERT_EQUAL_UINT16(4200, got.feet[2].touch_thresh);
  TEST_ASSERT_EQUAL_UINT8(1, got.feet[2].enabled);
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

void test_validate_rejects_missing_joint_slot() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  // Point two servos at the same (leg, joint) slot, leaving another uncovered.
  cfg.servos[1].leg = cfg.servos[0].leg;
  cfg.servos[1].joint = cfg.servos[0].joint;
  TEST_ASSERT_FALSE(validateRobotConfig(cfg));
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
  RUN_TEST(test_validate_rejects_duplicate_id);
  RUN_TEST(test_validate_rejects_bad_ranges);
  RUN_TEST(test_validate_rejects_missing_joint_slot);
  return UNITY_END();
}
