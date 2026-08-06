#include <unity.h>

#include "../../src/input/crsf_parser.h"
#include "../../src/input/crsf_telemetry.h"

using namespace crsf;

void test_hexapod_status_matches_controller_wire_layout() {
  telemetry::HexapodStatus status;
  status.flags = telemetry::flag::kArmed | telemetry::flag::kMotionGate |
                 telemetry::flag::kImuPresent | telemetry::flag::kImuFresh;
  status.safety_state = 5;
  status.command_source = 1;
  status.gait = 2;
  status.control_mode = 0;
  status.fault_reason = 0;
  status.speed_x255 = 191;
  status.duty_x255 = 153;
  status.imu_calibration = 0xDB;
  status.battery_mv = 12000;
  status.body_height_mm = 40;
  status.stride_mm = 60;
  status.step_height_mm = 30;
  // v2: gait-tune editor open on Stride (param 1) with a preview running, and
  // a Warning-severity (2) deduplicated error.
  status.tune_flags = telemetry::tuneflag::kTuneActive |
                      telemetry::tuneflag::kPreviewActive |
                      (1u << telemetry::tuneflag::kParamShift) |
                      (1u << telemetry::tuneflag::kSeverityShift);
  status.error_code = 16;  // BatteryLow
  status.error_detail = 102;
  status.error_sequence = 7;
  status.error_count = 300;
  status.error_suppressed = 4096;

  uint8_t payload[telemetry::kHexapodPayloadBytes];
  TEST_ASSERT_EQUAL_UINT8(telemetry::kHexapodPayloadBytes,
                          telemetry::encodeHexapodStatus(status, payload));
  const uint8_t expected[telemetry::kHexapodPayloadBytes] = {
      0x48, 0x58, 0x02, 0x33, 0x05, 0x01, 0x02, 0x00, 0x00, 0xBF,
      0x99, 0xDB, 0x2E, 0xE0, 0x00, 0x28, 0x00, 0x3C, 0x00, 0x1E,
      0x53, 0x10, 0x66, 0x07, 0x01, 0x2C, 0x10, 0x00,
  };
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, payload, sizeof(expected));
}

void test_frame_builder_adds_length_and_crc() {
  const uint8_t payload[] = {0x48, 0x58, 0x01};
  uint8_t frame[kMaxFrameLen] = {};
  const uint8_t size = telemetry::buildFrame(
      telemetry::kFrameTypeHexapodStatus, payload, sizeof(payload), frame,
      sizeof(frame));

  TEST_ASSERT_EQUAL_UINT8(7, size);
  TEST_ASSERT_EQUAL_UINT8(kSyncByte, frame[0]);
  TEST_ASSERT_EQUAL_UINT8(5, frame[1]);
  TEST_ASSERT_EQUAL_UINT8(telemetry::kFrameTypeHexapodStatus, frame[2]);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, &frame[3], sizeof(payload));
  TEST_ASSERT_EQUAL_UINT8(crc8(&frame[2], 4), frame[6]);
  TEST_ASSERT_EQUAL_UINT8(0, telemetry::buildFrame(
      telemetry::kFrameTypeHexapodStatus, payload, sizeof(payload), frame, 6));
}

void test_standard_sensor_scaling_is_big_endian() {
  uint8_t battery[telemetry::kBatteryPayloadBytes];
  telemetry::encodeBattery(12000, 75, battery);
  TEST_ASSERT_EQUAL_UINT8(0x00, battery[0]);
  TEST_ASSERT_EQUAL_UINT8(0x78, battery[1]);  // 120 * 0.1 V
  TEST_ASSERT_EQUAL_UINT8(75, battery[7]);

  uint8_t attitude[telemetry::kAttitudePayloadBytes];
  telemetry::encodeAttitude(9000, -4500, 0, attitude);
  const int16_t pitch = static_cast<int16_t>(
      static_cast<uint16_t>(attitude[0] << 8) | attitude[1]);
  const int16_t roll = static_cast<int16_t>(
      static_cast<uint16_t>(attitude[2] << 8) | attitude[3]);
  TEST_ASSERT_INT16_WITHIN(2, 15708, pitch);   // pi/2 * 10000
  TEST_ASSERT_INT16_WITHIN(2, -7854, roll);   // -pi/4 * 10000
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_hexapod_status_matches_controller_wire_layout);
  RUN_TEST(test_frame_builder_adds_length_and_crc);
  RUN_TEST(test_standard_sensor_scaling_is_big_endian);
  return UNITY_END();
}
