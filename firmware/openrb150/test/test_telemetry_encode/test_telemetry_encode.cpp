// Native (host) unit tests for the portable telemetry payload encoders
// (protocol/telemetry_encode). Verifies the exact little-endian wire layouts
// for servo_status / joint_state / servo_goals / leg_state against the schema
// the Python decoders (protocol/python/.../telemetry.py) consume. No Arduino
// deps (audit 22l.9 / lmt.12).
//
// Run with: pio test -e native -f test_telemetry_encode

#include <math.h>
#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/dxl/dxl_status.h"
#include "../../src/dxl/servo_map.h"
#include "../../src/gait/gait_pipeline.h"
#include "../../src/protocol/maintenance_target_api.h"
#include "../../src/protocol/telemetry_encode.h"

using namespace config;

namespace {

uint16_t rdU16(const uint8_t* p, int o) {
  return static_cast<uint16_t>(p[o] | (p[o + 1] << 8));
}
int16_t rdI16(const uint8_t* p, int o) {
  return static_cast<int16_t>(rdU16(p, o));
}
uint32_t rdU32(const uint8_t* p, int o) {
  return static_cast<uint32_t>(p[o]) | (static_cast<uint32_t>(p[o + 1]) << 8) |
         (static_cast<uint32_t>(p[o + 2]) << 16) |
         (static_cast<uint32_t>(p[o + 3]) << 24);
}

// Independent copy of the encoder's angle->wire conversion, used to compute the
// expected centidegree bytes for non-center ticks.
int16_t expectCentideg(float rad) {
  long c = lroundf(rad * dxl::kRadToDeg * 100.0f);
  if (c > 32767) c = 32767;
  if (c < -32768) c = -32768;
  return static_cast<int16_t>(c);
}

}  // namespace

// ---------------------------------------------------------------------------
// servo_status: count(1) + 14 bytes/servo.
// ---------------------------------------------------------------------------
void test_servo_status_layout() {
  dxl::ServoStatus servos[2];
  servos[0].id = 5;
  servos[0].present_position = 0x01020304;
  servos[0].present_velocity = -3;
  servos[0].present_load = 257;
  servos[0].present_voltage_mv = 12000;
  servos[0].present_temperature_c = 37;
  servos[0].hardware_error = 0x02;
  servos[0].torque_enabled = true;
  servos[1].id = 9;
  servos[1].present_position = 2048;
  servos[1].present_voltage_mv = 11000;
  servos[1].present_temperature_c = 30;

  uint8_t out[64] = {0};
  const uint16_t n = protocol::encodeServoStatus(servos, 2, out);

  TEST_ASSERT_EQUAL_UINT16(1 + 2 * 14, n);
  TEST_ASSERT_EQUAL_UINT8(2, out[0]);
  // servo 0
  TEST_ASSERT_EQUAL_UINT8(5, out[1]);
  TEST_ASSERT_EQUAL_UINT32(0x01020304u, rdU32(out, 2));
  TEST_ASSERT_EQUAL_INT16(-3, rdI16(out, 6));
  TEST_ASSERT_EQUAL_INT16(257, rdI16(out, 8));
  TEST_ASSERT_EQUAL_UINT16(12000, rdU16(out, 10));
  TEST_ASSERT_EQUAL_INT8(37, static_cast<int8_t>(out[12]));
  TEST_ASSERT_EQUAL_UINT8(0x02, out[13]);
  TEST_ASSERT_EQUAL_UINT8(1, out[14]);
  // servo 1 starts at 1 + 14 = 15
  TEST_ASSERT_EQUAL_UINT8(9, out[15]);
  TEST_ASSERT_EQUAL_UINT32(2048u, rdU32(out, 16));
  TEST_ASSERT_EQUAL_UINT16(11000, rdU16(out, 24));
  TEST_ASSERT_EQUAL_UINT8(0, out[29]);  // torque off
}

void test_servo_status_empty() {
  uint8_t out[8] = {0xAA};
  const uint16_t n = protocol::encodeServoStatus(nullptr, 0, out);
  TEST_ASSERT_EQUAL_UINT16(1, n);
  TEST_ASSERT_EQUAL_UINT8(0, out[0]);
}

// ---------------------------------------------------------------------------
// joint_state: count(1) + 4 bytes/joint, mapped + unmapped skipped.
// ---------------------------------------------------------------------------
void test_joint_state_maps_and_skips_unmapped() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  dxl::ServoMap map(cfg);

  dxl::ServoStatus servos[3];
  servos[0].id = 1;  // leg 0, coxa
  servos[0].present_position = kServoCenterTick;
  servos[1].id = 200;  // not in the map -> skipped
  servos[1].present_position = kServoCenterTick;
  servos[2].id = 18;  // leg 5, tibia
  servos[2].present_position = kServoCenterTick;

  uint8_t out[64] = {0};
  const uint16_t n = protocol::encodeJointState(map, servos, 3, out);

  TEST_ASSERT_EQUAL_UINT16(1 + 2 * 4, n);  // 200 skipped
  TEST_ASSERT_EQUAL_UINT8(2, out[0]);
  // joint 0 = leg 0 / coxa, center tick -> 0 centideg
  TEST_ASSERT_EQUAL_UINT8(0, out[1]);
  TEST_ASSERT_EQUAL_UINT8(0, out[2]);
  TEST_ASSERT_EQUAL_INT16(0, rdI16(out, 3));
  // joint 1 = leg 5 / tibia
  TEST_ASSERT_EQUAL_UINT8(5, out[5]);
  TEST_ASSERT_EQUAL_UINT8(2, out[6]);
  TEST_ASSERT_EQUAL_INT16(0, rdI16(out, 7));
}

void test_joint_state_clamps_tick_range() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  dxl::ServoMap map(cfg);

  // A negative present position must clamp to tick 0 (not wrap), and a value
  // above the device max must clamp to kServoMaxTick.
  dxl::ServoStatus lo;
  lo.id = 1;
  lo.present_position = -100;
  dxl::ServoStatus lo_ref;
  lo_ref.id = 1;
  lo_ref.present_position = 0;
  uint8_t a[16] = {0};
  uint8_t b[16] = {0};
  protocol::encodeJointState(map, &lo, 1, a);
  protocol::encodeJointState(map, &lo_ref, 1, b);
  TEST_ASSERT_EQUAL_INT16(rdI16(b, 3), rdI16(a, 3));

  dxl::ServoStatus hi;
  hi.id = 1;
  hi.present_position = 5000;
  dxl::ServoStatus hi_ref;
  hi_ref.id = 1;
  hi_ref.present_position = kServoMaxTick;
  protocol::encodeJointState(map, &hi, 1, a);
  protocol::encodeJointState(map, &hi_ref, 1, b);
  TEST_ASSERT_EQUAL_INT16(rdI16(b, 3), rdI16(a, 3));
}

// ---------------------------------------------------------------------------
// servo_goals (gait source): count(1) + 5 bytes/joint with clamp flag.
// ---------------------------------------------------------------------------
void test_servo_goals_gait_layout() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  dxl::ServoMap map(cfg);

  gait::PipelineJoint joints[2];
  joints[0].id = 1;
  joints[0].leg = 0;
  joints[0].joint = 0;
  joints[0].tick = kServoCenterTick;
  joints[0].clamped = false;
  joints[1].id = 7;
  joints[1].leg = 0;
  joints[1].joint = 1;
  joints[1].tick = 2560;
  joints[1].clamped = true;

  uint8_t out[64] = {0};
  const uint16_t n = protocol::encodeServoGoals(map, joints, 2, out);

  TEST_ASSERT_EQUAL_UINT16(1 + 2 * 5, n);
  TEST_ASSERT_EQUAL_UINT8(2, out[0]);
  // joint 0: leg 0 / coxa, center -> 0, not clamped
  TEST_ASSERT_EQUAL_UINT8(0, out[1]);
  TEST_ASSERT_EQUAL_UINT8(0, out[2]);
  TEST_ASSERT_EQUAL_INT16(0, rdI16(out, 3));
  TEST_ASSERT_EQUAL_UINT8(0x00, out[5]);
  // joint 1: leg 0 / femur, tick 2560, clamped
  TEST_ASSERT_EQUAL_UINT8(0, out[6]);
  TEST_ASSERT_EQUAL_UINT8(1, out[7]);
  TEST_ASSERT_EQUAL_INT16(expectCentideg(map.tickToAngle(0, 1, 2560)),
                          rdI16(out, 8));
  TEST_ASSERT_EQUAL_UINT8(0x01, out[10]);
}

// ---------------------------------------------------------------------------
// servo_goals (maintenance source): same 5-byte layout from the target set.
// ---------------------------------------------------------------------------
void test_servo_goals_maintenance_layout() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  dxl::ServoMap map(cfg);

  protocol::MaintTargetSet tgt{};
  tgt.set[0][0] = true;
  tgt.tick[0][0] = kServoCenterTick;
  tgt.clamped[0][0] = false;
  tgt.set[5][2] = true;
  tgt.tick[5][2] = 2300;
  tgt.clamped[5][2] = true;

  uint8_t out[64] = {0};
  const uint16_t n = protocol::encodeServoGoals(map, tgt, out);

  TEST_ASSERT_EQUAL_UINT16(1 + 2 * 5, n);
  TEST_ASSERT_EQUAL_UINT8(2, out[0]);
  // iteration order is leg-major then joint, so (0,0) first
  TEST_ASSERT_EQUAL_UINT8(0, out[1]);
  TEST_ASSERT_EQUAL_UINT8(0, out[2]);
  TEST_ASSERT_EQUAL_UINT8(0x00, out[5]);
  // (5,2) last
  TEST_ASSERT_EQUAL_UINT8(5, out[6]);
  TEST_ASSERT_EQUAL_UINT8(2, out[7]);
  TEST_ASSERT_EQUAL_INT16(expectCentideg(map.tickToAngle(5, 2, 2300)),
                          rdI16(out, 8));
  TEST_ASSERT_EQUAL_UINT8(0x01, out[10]);
}

// ---------------------------------------------------------------------------
// leg_state: count(1) + 8 bytes/leg with reachable/clamped flags.
// ---------------------------------------------------------------------------
void test_leg_state_layout() {
  protocol::MaintTargetSet tgt{};
  tgt.leg_target_set[2] = true;
  tgt.foot_x_mm[2] = 100;
  tgt.foot_y_mm[2] = -50;
  tgt.foot_z_mm[2] = -44;
  tgt.leg_reachable[2] = true;
  tgt.leg_clamped[2] = false;
  tgt.leg_target_set[4] = true;
  tgt.foot_x_mm[4] = -200;
  tgt.foot_y_mm[4] = 0;
  tgt.foot_z_mm[4] = 10;
  tgt.leg_reachable[4] = false;
  tgt.leg_clamped[4] = true;

  uint8_t out[64] = {0};
  const uint16_t n = protocol::encodeLegState(tgt, out);

  TEST_ASSERT_EQUAL_UINT16(1 + 2 * 8, n);
  TEST_ASSERT_EQUAL_UINT8(2, out[0]);
  // leg 2 (reachable)
  TEST_ASSERT_EQUAL_UINT8(2, out[1]);
  TEST_ASSERT_EQUAL_INT16(100, rdI16(out, 2));
  TEST_ASSERT_EQUAL_INT16(-50, rdI16(out, 4));
  TEST_ASSERT_EQUAL_INT16(-44, rdI16(out, 6));
  TEST_ASSERT_EQUAL_UINT8(0x01, out[8]);
  // leg 4 (clamped, not reachable)
  TEST_ASSERT_EQUAL_UINT8(4, out[9]);
  TEST_ASSERT_EQUAL_INT16(-200, rdI16(out, 10));
  TEST_ASSERT_EQUAL_INT16(0, rdI16(out, 12));
  TEST_ASSERT_EQUAL_INT16(10, rdI16(out, 14));
  TEST_ASSERT_EQUAL_UINT8(0x02, out[16]);
}

void test_leg_state_empty() {
  protocol::MaintTargetSet tgt{};
  uint8_t out[8] = {0xAA};
  const uint16_t n = protocol::encodeLegState(tgt, out);
  TEST_ASSERT_EQUAL_UINT16(1, n);
  TEST_ASSERT_EQUAL_UINT8(0, out[0]);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_servo_status_layout);
  RUN_TEST(test_servo_status_empty);
  RUN_TEST(test_joint_state_maps_and_skips_unmapped);
  RUN_TEST(test_joint_state_clamps_tick_range);
  RUN_TEST(test_servo_goals_gait_layout);
  RUN_TEST(test_servo_goals_maintenance_layout);
  RUN_TEST(test_leg_state_layout);
  RUN_TEST(test_leg_state_empty);
  return UNITY_END();
}
