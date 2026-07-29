#include <unity.h>

#include "sensors/bno085_protocol.h"

namespace {

void writeLe16(uint8_t* destination, int16_t value) {
  const uint16_t bits = static_cast<uint16_t>(value);
  destination[0] = static_cast<uint8_t>(bits);
  destination[1] = static_cast<uint8_t>(bits >> 8);
}

void makeRotationPayload(uint8_t payload[19], int16_t x, int16_t y,
                         int16_t z, int16_t w, uint8_t accuracy) {
  for (uint8_t index = 0; index < 19; ++index) payload[index] = 0;
  payload[0] = sensors::bno085::kReportBaseTimestamp;
  payload[5] = sensors::bno085::kReportRotationVector;
  payload[7] = accuracy;
  writeLe16(&payload[9], x);
  writeLe16(&payload[11], y);
  writeLe16(&payload[13], z);
  writeLe16(&payload[15], w);
}

}  // namespace

void test_feature_command_enables_rotation_vector_at_requested_rate() {
  uint8_t command[sensors::bno085::kSetFeatureBytes];
  TEST_ASSERT_EQUAL_UINT8(
      sensors::bno085::kSetFeatureBytes,
      sensors::bno085::buildRotationVectorFeature(50000, command));
  TEST_ASSERT_EQUAL_HEX8(sensors::bno085::kReportSetFeature, command[0]);
  TEST_ASSERT_EQUAL_HEX8(sensors::bno085::kReportRotationVector, command[1]);
  TEST_ASSERT_EQUAL_HEX8(0x50, command[5]);
  TEST_ASSERT_EQUAL_HEX8(0xC3, command[6]);
  TEST_ASSERT_EQUAL_HEX8(0x00, command[7]);
  TEST_ASSERT_EQUAL_HEX8(0x00, command[8]);
}

void test_identity_quaternion_decodes_to_zero_euler() {
  uint8_t payload[19];
  makeRotationPayload(payload, 0, 0, 0, 16384, 3);
  sensors::bno085::OrientationSample sample;
  TEST_ASSERT_TRUE(
      sensors::bno085::decodeRotationVector(payload, sizeof(payload), sample));
  TEST_ASSERT_TRUE(sample.ok);
  TEST_ASSERT_EQUAL_INT16(0, sample.roll_cdeg);
  TEST_ASSERT_EQUAL_INT16(0, sample.pitch_cdeg);
  TEST_ASSERT_EQUAL_INT16(0, sample.yaw_cdeg);
  TEST_ASSERT_EQUAL_UINT8(3, sample.quality);
}

void test_quaternion_decodes_positive_ninety_degree_yaw() {
  uint8_t payload[19];
  makeRotationPayload(payload, 0, 0, 11585, 11585, 2);
  sensors::bno085::OrientationSample sample;
  TEST_ASSERT_TRUE(
      sensors::bno085::decodeRotationVector(payload, sizeof(payload), sample));
  TEST_ASSERT_INT16_WITHIN(2, 9000, sample.yaw_cdeg);
  TEST_ASSERT_INT16_WITHIN(2, 0, sample.roll_cdeg);
  TEST_ASSERT_INT16_WITHIN(2, 0, sample.pitch_cdeg);
}

void test_malformed_or_zero_quaternion_is_rejected() {
  uint8_t payload[19];
  makeRotationPayload(payload, 0, 0, 0, 0, 0);
  sensors::bno085::OrientationSample sample;
  TEST_ASSERT_FALSE(
      sensors::bno085::decodeRotationVector(payload, sizeof(payload), sample));
  TEST_ASSERT_FALSE(
      sensors::bno085::decodeRotationVector(payload, 18, sample));
  payload[5] = sensors::bno085::kReportRotationVector + 1;
  TEST_ASSERT_FALSE(
      sensors::bno085::decodeRotationVector(payload, sizeof(payload), sample));
}

void test_quality_is_repeated_in_legacy_status_fields() {
  TEST_ASSERT_EQUAL_HEX8(0x00, sensors::bno085::packQuality(0));
  TEST_ASSERT_EQUAL_HEX8(0x55, sensors::bno085::packQuality(1));
  TEST_ASSERT_EQUAL_HEX8(0xAA, sensors::bno085::packQuality(2));
  TEST_ASSERT_EQUAL_HEX8(0xFF, sensors::bno085::packQuality(3));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_feature_command_enables_rotation_vector_at_requested_rate);
  RUN_TEST(test_identity_quaternion_decodes_to_zero_euler);
  RUN_TEST(test_quaternion_decodes_positive_ninety_degree_yaw);
  RUN_TEST(test_malformed_or_zero_quaternion_is_rejected);
  RUN_TEST(test_quality_is_repeated_in_legacy_status_fields);
  return UNITY_END();
}