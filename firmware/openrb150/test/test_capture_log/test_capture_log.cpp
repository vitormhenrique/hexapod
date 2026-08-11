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
      0, formatServoCaptureRow(3, 9, 1250, servo, -315, line, sizeof(line)));
  TEST_ASSERT_EQUAL_STRING("S,3,9,1250,12,1,2048,-315,-31,427,12100,46,4,1\n",
                           line);
}

void test_motion_leg_and_goal_rows_preserve_diagnostic_targets() {
  AppliedMotionCapture motion;
  motion.goal_sequence = 77;
  motion.command_source = 1;
  motion.safety_state = 4;
  motion.gait = 2;
  motion.flags = kAppliedMotionFlagMotionGate |
                 kAppliedMotionFlagGoalReachLimited;
  motion.body_height_mm = 132;
  motion.stride_mm = 60;
  motion.step_height_mm = 30;
  motion.duty_x255 = 159;
  motion.speed_x255 = 128;
  char line[256];
  TEST_ASSERT_EQUAL_STRING(
      "C,3,9,1250,77,1,4,2,17,132,60,30,159,128\n",
      (formatAppliedMotionCaptureRow(3, 9, 1250, motion, line,
                                     sizeof(line)), line));

  gait::PipelineLegTarget legs[2];
  legs[0].foot_x_mm = -155;
  legs[0].foot_y_mm = 205;
  legs[0].foot_z_mm = -132;
  legs[0].flags = gait::kPipelineLegFlagReachable;
  legs[1].foot_x_mm = 155;
  legs[1].foot_y_mm = 205;
  legs[1].foot_z_mm = -120;
  legs[1].flags = gait::kPipelineLegFlagSwing |
                  gait::kPipelineLegFlagReachable;
  TEST_ASSERT_EQUAL_STRING(
      "L,3,9,1250,77,2,0,-155,205,-132,2,1,155,205,-120,3\n",
      (formatLegCaptureRow(3, 9, 1250, 77, legs, 2, line,
                           sizeof(line)), line));

  GoalCapture goals[2];
  goals[0].id = 10;
  goals[0].leg = 0;
  goals[0].joint = 0;
  goals[0].goal_tick = 2050;
  goals[0].goal_angle_centideg = 18;
  goals[1].id = 11;
  goals[1].leg = 0;
  goals[1].joint = 1;
  goals[1].goal_tick = 1700;
  goals[1].goal_angle_centideg = -3050;
  goals[1].flags = kGoalCaptureFlagClamped;
  TEST_ASSERT_EQUAL_STRING(
      "G,3,9,1250,77,2,10,0,0,2050,18,0,11,0,1,1700,-3050,1\n",
      (formatGoalCaptureRow(3, 9, 1250, 77, goals, 2, line,
                            sizeof(line)), line));
}

void test_capture_markers_delimit_sessions() {
  char line[64];
  TEST_ASSERT_GREATER_THAN(
      0, formatCaptureMarker(true, 4, 1000, 0, line, sizeof(line)));
  TEST_ASSERT_EQUAL_STRING("BEGIN,4,1000,0,2\n", line);
  TEST_ASSERT_GREATER_THAN(
      0, formatCaptureMarker(false, 4, 9000, 17, line, sizeof(line)));
  TEST_ASSERT_EQUAL_STRING("END,4,9000,17\n", line);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_remote_row_contains_raw_and_decoded_values);
  RUN_TEST(test_remote_row_extremes_fit_runtime_buffer);
  RUN_TEST(test_servo_row_contains_dynamixel_health_fields);
  RUN_TEST(test_motion_leg_and_goal_rows_preserve_diagnostic_targets);
  RUN_TEST(test_capture_markers_delimit_sessions);
  return UNITY_END();
}