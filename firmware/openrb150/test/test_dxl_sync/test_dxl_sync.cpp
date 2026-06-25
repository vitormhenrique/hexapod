// Native (host) unit tests for dxl_sync addressing + (de)serialization.
// No Arduino deps. Run with: pio test -e native

#include <unity.h>

#include "../../src/dxl/dxl_sync.h"

using namespace dxl;

void test_goal_addr_len_by_table() {
  TEST_ASSERT_EQUAL_UINT16(30, goalAddr(TableKind::Mx28Legacy));
  TEST_ASSERT_EQUAL_UINT8(2, goalLen(TableKind::Mx28Legacy));
  TEST_ASSERT_EQUAL_UINT16(116, goalAddr(TableKind::Mx28V2));
  TEST_ASSERT_EQUAL_UINT8(4, goalLen(TableKind::Mx28V2));
  // Unknown falls back to the conservative legacy layout.
  TEST_ASSERT_EQUAL_UINT16(30, goalAddr(TableKind::Unknown));
  TEST_ASSERT_EQUAL_UINT8(2, goalLen(TableKind::Unknown));
}

void test_status_addr_len_by_table() {
  TEST_ASSERT_EQUAL_UINT16(36, statusAddr(TableKind::Mx28Legacy));
  TEST_ASSERT_EQUAL_UINT8(8, statusLen(TableKind::Mx28Legacy));
  TEST_ASSERT_EQUAL_UINT16(124, statusAddr(TableKind::Mx28V2));
  TEST_ASSERT_EQUAL_UINT8(23, statusLen(TableKind::Mx28V2));
}

void test_encode_goal_legacy_le() {
  uint8_t buf[4] = {0xAA, 0xAA, 0xAA, 0xAA};
  const uint8_t n = encodeGoal(TableKind::Mx28Legacy, 0x0123, buf);
  TEST_ASSERT_EQUAL_UINT8(2, n);
  TEST_ASSERT_EQUAL_HEX8(0x23, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x01, buf[1]);
  // High bytes untouched for the 2-byte legacy write.
  TEST_ASSERT_EQUAL_HEX8(0xAA, buf[2]);
}

void test_encode_goal_v2_le() {
  uint8_t buf[4] = {0xAA, 0xAA, 0xAA, 0xAA};
  const uint8_t n = encodeGoal(TableKind::Mx28V2, 2048, buf);
  TEST_ASSERT_EQUAL_UINT8(4, n);
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x08, buf[1]);  // 2048 = 0x0800
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[3]);
}

void test_encode_goal_max_tick() {
  uint8_t buf[4] = {0};
  encodeGoal(TableKind::Mx28V2, 4095, buf);
  TEST_ASSERT_EQUAL_HEX8(0xFF, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x0F, buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[3]);
}

void test_legacy_sign_magnitude() {
  TEST_ASSERT_EQUAL_INT32(0, decodeLegacySignMag(0));
  TEST_ASSERT_EQUAL_INT32(300, decodeLegacySignMag(300));
  TEST_ASSERT_EQUAL_INT32(-300, decodeLegacySignMag(0x400 | 300));
  TEST_ASSERT_EQUAL_INT32(1023, decodeLegacySignMag(0x3FF));
  TEST_ASSERT_EQUAL_INT32(-1023, decodeLegacySignMag(0x7FF));
}

void test_decode_status_legacy_golden() {
  // position=0x0800 (2048), speed=-100, load=+50, voltage=120(12.0V), temp=35C
  uint8_t blk[8];
  blk[0] = 0x00;
  blk[1] = 0x08;                // position 2048
  uint16_t spd = 0x400 | 100;   // -100 sign-magnitude
  blk[2] = spd & 0xFF;
  blk[3] = spd >> 8;
  blk[4] = 50;
  blk[5] = 0;                   // load +50
  blk[6] = 120;                 // 12.0 V
  blk[7] = 35;                  // 35 C

  StatusFields f;
  TEST_ASSERT_TRUE(decodeStatus(TableKind::Mx28Legacy, blk, f));
  TEST_ASSERT_EQUAL_INT32(2048, f.position);
  TEST_ASSERT_EQUAL_INT32(-100, f.velocity);
  TEST_ASSERT_EQUAL_INT32(50, f.load);
  TEST_ASSERT_EQUAL_UINT16(12000, f.voltage_mv);
  TEST_ASSERT_EQUAL_INT8(35, f.temperature_c);
}

void test_decode_status_v2_golden() {
  uint8_t blk[23] = {0};
  // pwm @0 (ignored)
  blk[0] = 0x10;
  blk[1] = 0x00;
  // load @2 = -200 (0.1%)
  int16_t load = -200;
  blk[2] = static_cast<uint8_t>(load & 0xFF);
  blk[3] = static_cast<uint8_t>((load >> 8) & 0xFF);
  // velocity @4 (4B) = -1500
  int32_t vel = -1500;
  blk[4] = vel & 0xFF;
  blk[5] = (vel >> 8) & 0xFF;
  blk[6] = (vel >> 16) & 0xFF;
  blk[7] = (vel >> 24) & 0xFF;
  // position @8 (4B) = 2049
  int32_t pos = 2049;
  blk[8] = pos & 0xFF;
  blk[9] = (pos >> 8) & 0xFF;
  blk[10] = (pos >> 16) & 0xFF;
  blk[11] = (pos >> 24) & 0xFF;
  // voltage @20 (2B) = 121 -> 12.1 V
  blk[20] = 121;
  blk[21] = 0;
  // temp @22 = 40 C
  blk[22] = 40;

  StatusFields f;
  TEST_ASSERT_TRUE(decodeStatus(TableKind::Mx28V2, blk, f));
  TEST_ASSERT_EQUAL_INT32(2049, f.position);
  TEST_ASSERT_EQUAL_INT32(-1500, f.velocity);
  TEST_ASSERT_EQUAL_INT32(-200, f.load);
  TEST_ASSERT_EQUAL_UINT16(12100, f.voltage_mv);
  TEST_ASSERT_EQUAL_INT8(40, f.temperature_c);
}

void test_decode_status_unknown_fails() {
  uint8_t blk[23] = {0};
  StatusFields f;
  TEST_ASSERT_FALSE(decodeStatus(TableKind::Unknown, blk, f));
}

void test_pos_addr_len_by_table() {
  TEST_ASSERT_EQUAL_UINT16(36, posAddr(TableKind::Mx28Legacy));
  TEST_ASSERT_EQUAL_UINT8(2, posLen(TableKind::Mx28Legacy));
  TEST_ASSERT_EQUAL_UINT16(132, posAddr(TableKind::Mx28V2));
  TEST_ASSERT_EQUAL_UINT8(4, posLen(TableKind::Mx28V2));
}

void test_decode_position() {
  uint8_t legacy[2] = {0x00, 0x08};  // 2048
  TEST_ASSERT_EQUAL_INT32(2048, decodePosition(TableKind::Mx28Legacy, legacy));
  uint8_t v2[4] = {0x01, 0x08, 0x00, 0x00};  // 2049
  TEST_ASSERT_EQUAL_INT32(2049, decodePosition(TableKind::Mx28V2, v2));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_goal_addr_len_by_table);
  RUN_TEST(test_status_addr_len_by_table);
  RUN_TEST(test_encode_goal_legacy_le);
  RUN_TEST(test_encode_goal_v2_le);
  RUN_TEST(test_encode_goal_max_tick);
  RUN_TEST(test_legacy_sign_magnitude);
  RUN_TEST(test_decode_status_legacy_golden);
  RUN_TEST(test_decode_status_v2_golden);
  RUN_TEST(test_decode_status_unknown_fails);
  RUN_TEST(test_pos_addr_len_by_table);
  RUN_TEST(test_decode_position);
  return UNITY_END();
}
