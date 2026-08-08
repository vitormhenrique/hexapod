#include <string.h>
#include <unity.h>

#include "../../src/logging/capture_log.h"

using namespace logging;

void test_remote_row_contains_raw_and_decoded_values() {
  RemoteCapture remote;
  remote.timestamp_ms = 1234;
  remote.frame_sequence = 77;
  remote.flags = 5;
  remote.mode = 2;
  remote.gait = 1;
  remote.gimbal[0] = -1000;
  remote.gimbal[1] = 250;
  remote.pot[0] = 500;
  remote.encoder[1] = -12;
  remote.switch_mask = 0x81;
  remote.button_mask = 0x08;
  remote.twist_milli[0] = 625;
  remote.pose[3] = -120;
  remote.speed_x255 = 128;
  remote.duty_x255 = 159;

  char line[256];
  TEST_ASSERT_GREATER_THAN(
      0, formatRemoteCaptureRow(3, 9, remote, line, sizeof(line)));
  TEST_ASSERT_NOT_NULL(strstr(line, "R,3,9,1234,77,5,2,1,-1000,250"));
  TEST_ASSERT_NOT_NULL(strstr(line, ",500,0,0,-12,129,8,"));
  TEST_ASSERT_NOT_NULL(strstr(line, ",625,0,0,0,0,0,-120,0,0,128"));
}

void test_remote_row_extremes_fit_runtime_buffer() {
  RemoteCapture remote;
  remote.timestamp_ms = 0xFFFFFFFFu;
  remote.frame_sequence = 0xFFFFFFFFu;
  remote.flags = 0xFF;
  remote.mode = 255;
  remote.gait = 255;
  for (uint8_t i = 0; i < 4; ++i) remote.gimbal[i] = -32768;
  for (uint8_t i = 0; i < 2; ++i) {
    remote.pot[i] = 32767;
    remote.encoder[i] = INT32_MIN;
    remote.toggle[i] = 255;
    remote.nav_mask[i] = 255;
  }
  for (uint8_t i = 0; i < 3; ++i) remote.twist_milli[i] = -32768;
  for (uint8_t i = 0; i < 6; ++i) remote.pose[i] = -32768;
  remote.switch_mask = 255;
  remote.button_mask = 255;
  remote.speed_x255 = 255;
  remote.body_height_x255 = 255;
  remote.stride_x255 = 255;
  remote.step_height_x255 = 255;
  remote.duty_x255 = 255;

  char line[256];
  TEST_ASSERT_GREATER_THAN(
      0, formatRemoteCaptureRow(0xFFFFFFFFu, 0xFFFFFFFFu, remote, line,
                                sizeof(line)));
}

void test_servo_row_contains_dynamixel_health_fields() {
  dxl::ServoStatus servo;
  servo.id = 12;
  servo.ok = true;
  servo.present_position = 2048;
  servo.present_velocity = -31;
  servo.present_load = 427;
  servo.present_voltage_mv = 12100;
  servo.present_temperature_c = 46;
  servo.hardware_error = 4;
  servo.torque_enabled = true;

  char line[128];
  TEST_ASSERT_GREATER_THAN(
      0, formatServoCaptureRow(3, 9, 1250, servo, line, sizeof(line)));
  TEST_ASSERT_EQUAL_STRING("S,3,9,1250,12,1,2048,-31,427,12100,46,4,1\n",
                           line);
}

void test_capture_markers_delimit_sessions() {
  char line[64];
  TEST_ASSERT_GREATER_THAN(
      0, formatCaptureMarker(true, 4, 1000, 0, line, sizeof(line)));
  TEST_ASSERT_EQUAL_STRING("BEGIN,4,1000,0\n", line);
  TEST_ASSERT_GREATER_THAN(
      0, formatCaptureMarker(false, 4, 9000, 17, line, sizeof(line)));
  TEST_ASSERT_EQUAL_STRING("END,4,9000,17\n", line);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_remote_row_contains_raw_and_decoded_values);
  RUN_TEST(test_remote_row_extremes_fit_runtime_buffer);
  RUN_TEST(test_servo_row_contains_dynamixel_health_fields);
  RUN_TEST(test_capture_markers_delimit_sessions);
  return UNITY_END();
}