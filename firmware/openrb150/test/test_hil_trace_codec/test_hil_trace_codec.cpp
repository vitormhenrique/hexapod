#include <unity.h>

#include "../../src/hil/trace_codec.h"
#include "../../src/protocol/crc16.h"

namespace {

constexpr uint16_t kPayloadCapacity = 80;
constexpr uint16_t kLogicalLength = 173;

void fillLogical(uint8_t* logical) {
  for (uint16_t index = 0; index < kLogicalLength; ++index) {
    logical[index] = static_cast<uint8_t>((index * 37u) & 0xFFu);
  }
}

hil::trace::FragmentHeader makeHeader(uint8_t index, uint8_t count,
                                      const uint8_t* logical) {
  hil::trace::FragmentHeader header;
  header.session_id = 41;
  header.capture_id = 7;
  header.record_seq = 3;
  header.record_type = hil::trace::RecordType::Step;
  header.fragment_index = index;
  header.fragment_count = count;
  header.logical_length = kLogicalLength;
  header.logical_crc16 = protocol::crc16(logical, kLogicalLength);
  return header;
}

}  // namespace

void test_fragment_codec_round_trips_an_ordered_record() {
  uint8_t logical[kLogicalLength];
  fillLogical(logical);
  const uint16_t slice_capacity = hil::trace::maxFragmentData(kPayloadCapacity);
  const uint8_t count = static_cast<uint8_t>(
      (kLogicalLength + slice_capacity - 1u) / slice_capacity);
  TEST_ASSERT_EQUAL_UINT8(3, count);

  uint8_t rebuilt[kLogicalLength] = {};
  hil::trace::ReassemblyState state;
  for (uint8_t index = 0; index < count; ++index) {
    uint8_t payload[kPayloadCapacity] = {};
    uint16_t payload_len = 0;
    TEST_ASSERT_TRUE(hil::trace::encodeFragment(makeHeader(index, count, logical),
                                                logical, kLogicalLength,
                                                payload, sizeof(payload),
                                                &payload_len));
    const hil::trace::ReassemblyResult result = hil::trace::acceptFragment(
        payload, payload_len, rebuilt, sizeof(rebuilt), &state);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(index + 1u == count
                                 ? hil::trace::ReassemblyResult::Complete
                                 : hil::trace::ReassemblyResult::Accepted),
        static_cast<uint8_t>(result));
  }
  TEST_ASSERT_EQUAL_HEX8_ARRAY(logical, rebuilt, kLogicalLength);
}

void test_fragment_codec_rejects_out_of_order_and_bad_crc_records() {
  uint8_t logical[kLogicalLength];
  fillLogical(logical);
  const uint16_t slice_capacity = hil::trace::maxFragmentData(kPayloadCapacity);
  const uint8_t count = static_cast<uint8_t>(
      (kLogicalLength + slice_capacity - 1u) / slice_capacity);
  uint8_t payload[kPayloadCapacity] = {};
  uint16_t payload_len = 0;
  uint8_t rebuilt[kLogicalLength] = {};
  hil::trace::ReassemblyState state;

  TEST_ASSERT_TRUE(hil::trace::encodeFragment(makeHeader(1, count, logical),
                                              logical, kLogicalLength, payload,
                                              sizeof(payload), &payload_len));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::trace::ReassemblyResult::OutOfOrder),
      static_cast<uint8_t>(hil::trace::acceptFragment(
          payload, payload_len, rebuilt, sizeof(rebuilt), &state)));

  TEST_ASSERT_TRUE(hil::trace::encodeFragment(makeHeader(0, count, logical),
                                              logical, kLogicalLength, payload,
                                              sizeof(payload), &payload_len));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::trace::ReassemblyResult::Accepted),
      static_cast<uint8_t>(hil::trace::acceptFragment(
          payload, payload_len, rebuilt, sizeof(rebuilt), &state)));

  TEST_ASSERT_TRUE(hil::trace::encodeFragment(makeHeader(1, count, logical),
                                              logical, kLogicalLength, payload,
                                              sizeof(payload), &payload_len));
  payload[hil::trace::kFragmentPrefixBytes] ^= 0x01;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::trace::ReassemblyResult::Accepted),
      static_cast<uint8_t>(hil::trace::acceptFragment(
          payload, payload_len, rebuilt, sizeof(rebuilt), &state)));

  TEST_ASSERT_TRUE(hil::trace::encodeFragment(makeHeader(2, count, logical),
                                              logical, kLogicalLength, payload,
                                              sizeof(payload), &payload_len));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::trace::ReassemblyResult::BadCrc),
      static_cast<uint8_t>(hil::trace::acceptFragment(
          payload, payload_len, rebuilt, sizeof(rebuilt), &state)));
}

    void test_fragment_slice_encoder_matches_full_encoder() {
      uint8_t logical[kLogicalLength];
      fillLogical(logical);
      const uint16_t slice_capacity = hil::trace::maxFragmentData(kPayloadCapacity);
      const uint8_t count = static_cast<uint8_t>(
        (kLogicalLength + slice_capacity - 1u) / slice_capacity);
      const auto header = makeHeader(1, count, logical);
      uint8_t full[kPayloadCapacity] = {};
      uint16_t full_len = 0;
      TEST_ASSERT_TRUE(hil::trace::encodeFragment(header, logical, kLogicalLength,
                            full, sizeof(full), &full_len));

      const uint16_t offset = slice_capacity;
      const uint16_t slice_length =
        kLogicalLength - offset < slice_capacity ? kLogicalLength - offset
                            : slice_capacity;
      uint8_t sliced[kPayloadCapacity] = {};
      uint16_t sliced_len = 0;
      TEST_ASSERT_TRUE(hil::trace::encodeFragmentSlice(
        header, &logical[offset], slice_length, sliced, sizeof(sliced),
        &sliced_len));
      TEST_ASSERT_EQUAL_UINT16(full_len, sliced_len);
      TEST_ASSERT_EQUAL_HEX8_ARRAY(full, sliced, full_len);
    }

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_fragment_codec_round_trips_an_ordered_record);
  RUN_TEST(test_fragment_codec_rejects_out_of_order_and_bad_crc_records);
  RUN_TEST(test_fragment_slice_encoder_matches_full_encoder);
  return UNITY_END();
}