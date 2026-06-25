// Native (host) unit tests for the portable CRSF parser and RC status mapping.
// No Arduino dependencies; exercises crsf_parser.{h,cpp}.
//
// Run with: pio test -e native

#include <string.h>
#include <unity.h>

#include "../../src/input/crsf_parser.h"

using namespace crsf;

namespace {

// Pack 16 11-bit channel values into a 22-byte CRSF RC payload (little-endian).
void packChannels(const uint16_t ch[kNumChannels], uint8_t out[kRcPayloadBytes]) {
  memset(out, 0, kRcPayloadBytes);
  uint32_t bits = 0;
  uint8_t bits_avail = 0;
  uint8_t byte_idx = 0;
  uint8_t ci = 0;
  while (byte_idx < kRcPayloadBytes) {
    while (bits_avail < 8 && ci < kNumChannels) {
      bits |= static_cast<uint32_t>(ch[ci++] & 0x7FF) << bits_avail;
      bits_avail += 11;
    }
    out[byte_idx++] = static_cast<uint8_t>(bits & 0xFF);
    bits >>= 8;
    bits_avail = (bits_avail >= 8) ? (bits_avail - 8) : 0;
  }
}

// Build a full CRSF RC channels frame (sync, len, type, payload, crc8).
// Returns total length written into `frame`.
uint8_t buildRcFrame(const uint16_t ch[kNumChannels], uint8_t* frame) {
  frame[0] = kSyncByte;
  frame[1] = kRcPayloadBytes + 2;  // type + payload + crc
  frame[2] = kFrameTypeRcChannels;
  packChannels(ch, &frame[3]);
  frame[3 + kRcPayloadBytes] = crc8(&frame[2], kRcPayloadBytes + 1);
  return static_cast<uint8_t>(3 + kRcPayloadBytes + 1);
}

}  // namespace

void test_crc8_known_vector() {
  // CRSF DVB-S2 CRC8 (poly 0xD5, init 0) over {0x16} is 0xD3.
  const uint8_t one = kFrameTypeRcChannels;
  TEST_ASSERT_EQUAL_UINT8(0xD3, crc8(&one, 1));
}

void test_ticks_to_micros_bounds() {
  TEST_ASSERT_UINT16_WITHIN(2, 988, ticksToMicros(kTicksMin));
  TEST_ASSERT_UINT16_WITHIN(2, 1500, ticksToMicros(kTicksMid));
  TEST_ASSERT_UINT16_WITHIN(2, 2012, ticksToMicros(kTicksMax));
}

void test_parser_decodes_full_frame() {
  uint16_t ch[kNumChannels];
  for (uint8_t i = 0; i < kNumChannels; ++i) {
    ch[i] = static_cast<uint16_t>(200 + i * 100);  // distinct, in range
  }
  uint8_t frame[kMaxFrameLen];
  const uint8_t len = buildRcFrame(ch, frame);

  Parser p;
  ChannelData out;
  bool got = false;
  for (uint8_t i = 0; i < len; ++i) {
    if (p.push(frame[i], out)) {
      got = true;
    }
  }
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL_UINT32(1, p.framesDecoded());
  TEST_ASSERT_EQUAL_UINT32(0, p.crcErrors());
  for (uint8_t i = 0; i < kNumChannels; ++i) {
    TEST_ASSERT_EQUAL_UINT16(ch[i], out.channels[i]);
  }
}

void test_parser_resyncs_after_garbage() {
  uint16_t ch[kNumChannels];
  for (uint8_t i = 0; i < kNumChannels; ++i) ch[i] = kTicksMid;
  uint8_t frame[kMaxFrameLen];
  const uint8_t len = buildRcFrame(ch, frame);

  Parser p;
  ChannelData out;
  // Leading noise (no false sync) then a clean frame.
  const uint8_t junk[] = {0x00, 0xFF, 0x12, 0x34};
  for (uint8_t b : junk) p.push(b, out);
  bool got = false;
  for (uint8_t i = 0; i < len; ++i) {
    if (p.push(frame[i], out)) got = true;
  }
  TEST_ASSERT_TRUE(got);
}

void test_parser_rejects_bad_crc() {
  uint16_t ch[kNumChannels];
  for (uint8_t i = 0; i < kNumChannels; ++i) ch[i] = kTicksMid;
  uint8_t frame[kMaxFrameLen];
  const uint8_t len = buildRcFrame(ch, frame);
  frame[len - 1] ^= 0xFF;  // corrupt CRC

  Parser p;
  ChannelData out;
  bool got = false;
  for (uint8_t i = 0; i < len; ++i) {
    if (p.push(frame[i], out)) got = true;
  }
  TEST_ASSERT_FALSE(got);
  TEST_ASSERT_EQUAL_UINT32(1, p.crcErrors());
  TEST_ASSERT_EQUAL_UINT32(0, p.framesDecoded());
}

void test_rc_status_arm_kill_gait_mapping() {
  uint16_t ch[kNumChannels];
  for (uint8_t i = 0; i < kNumChannels; ++i) ch[i] = kTicksMid;
  ch[kChArm] = kTicksMax;   // arm high
  ch[kChKill] = kTicksMin;  // kill low
  ch[kChGait] = kTicksMin;  // gait position 0

  ChannelData frame;
  for (uint8_t i = 0; i < kNumChannels; ++i) frame.channels[i] = ch[i];

  RcStatus rc;
  initRcStatus(rc);
  applyFrame(rc, frame, /*now_ms=*/1000);
  TEST_ASSERT_FALSE(rc.failsafe);
  TEST_ASSERT_TRUE(rc.armed);
  TEST_ASSERT_FALSE(rc.kill);
  TEST_ASSERT_EQUAL_UINT8(0, rc.gait_index);

  // Kill switch high overrides arm.
  frame.channels[kChKill] = kTicksMax;
  frame.channels[kChGait] = kTicksMax;  // gait position 2
  applyFrame(rc, frame, /*now_ms=*/1100);
  TEST_ASSERT_TRUE(rc.kill);
  TEST_ASSERT_FALSE(rc.armed);
  TEST_ASSERT_EQUAL_UINT8(2, rc.gait_index);

  // Mid gait position 1.
  frame.channels[kChGait] = kTicksMid;
  applyFrame(rc, frame, /*now_ms=*/1200);
  TEST_ASSERT_EQUAL_UINT8(1, rc.gait_index);
}

void test_rc_status_failsafe_on_stale() {
  ChannelData frame;
  for (uint8_t i = 0; i < kNumChannels; ++i) frame.channels[i] = kTicksMid;
  frame.channels[kChArm] = kTicksMax;
  frame.channels[kChKill] = kTicksMin;

  RcStatus rc;
  initRcStatus(rc);
  applyFrame(rc, frame, /*now_ms=*/1000);
  TEST_ASSERT_TRUE(rc.armed);

  // Within timeout: still live.
  evaluateFailsafe(rc, /*now_ms=*/1100, /*timeout=*/250);
  TEST_ASSERT_FALSE(rc.failsafe);
  TEST_ASSERT_TRUE(rc.armed);

  // Past timeout: failsafe forces disarm + kill.
  evaluateFailsafe(rc, /*now_ms=*/1400, /*timeout=*/250);
  TEST_ASSERT_TRUE(rc.failsafe);
  TEST_ASSERT_FALSE(rc.armed);
  TEST_ASSERT_TRUE(rc.kill);
}

void test_rc_status_init_is_safe() {
  RcStatus rc;
  initRcStatus(rc);
  TEST_ASSERT_TRUE(rc.failsafe);
  TEST_ASSERT_FALSE(rc.armed);
  TEST_ASSERT_TRUE(rc.kill);
  TEST_ASSERT_FALSE(rc.ever_seen);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc8_known_vector);
  RUN_TEST(test_ticks_to_micros_bounds);
  RUN_TEST(test_parser_decodes_full_frame);
  RUN_TEST(test_parser_resyncs_after_garbage);
  RUN_TEST(test_parser_rejects_bad_crc);
  RUN_TEST(test_rc_status_arm_kill_gait_mapping);
  RUN_TEST(test_rc_status_failsafe_on_stale);
  RUN_TEST(test_rc_status_init_is_safe);
  return UNITY_END();
}
