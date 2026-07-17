#include "trace_codec.h"

#include "../protocol/crc16.h"

namespace hil {
namespace trace {
namespace {

uint16_t readU16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

void writeU16(uint8_t* p, uint16_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFF);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void writeU32(uint8_t* p, uint32_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFF);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

bool decodeHeader(const uint8_t* payload, uint16_t payload_len,
                  FragmentHeader* header, uint16_t* fragment_data_len) {
  if (payload == nullptr || header == nullptr || fragment_data_len == nullptr ||
      payload_len < kFragmentPrefixBytes) {
    return false;
  }

  FragmentHeader decoded;
  decoded.schema_version = readU16(&payload[0]);
  decoded.session_id = readU32(&payload[2]);
  decoded.capture_id = readU32(&payload[6]);
  decoded.record_seq = readU32(&payload[10]);
  decoded.record_type = static_cast<RecordType>(payload[14]);
  decoded.fragment_index = payload[15];
  decoded.fragment_count = payload[16];
  decoded.logical_length = readU16(&payload[17]);
  decoded.logical_crc16 = readU16(&payload[19]);

  if (decoded.schema_version != kSchemaVersion || decoded.session_id == 0 ||
      decoded.capture_id == 0 || decoded.fragment_count == 0 ||
      decoded.fragment_index >= decoded.fragment_count ||
      decoded.logical_length == 0 ||
      static_cast<uint8_t>(decoded.record_type) >
          static_cast<uint8_t>(RecordType::End)) {
    return false;
  }
  *header = decoded;
  *fragment_data_len =
      static_cast<uint16_t>(payload_len - kFragmentPrefixBytes);
  return true;
}

bool sameRecord(const FragmentHeader& a, const FragmentHeader& b) {
  return a.schema_version == b.schema_version &&
         a.session_id == b.session_id && a.capture_id == b.capture_id &&
         a.record_seq == b.record_seq && a.record_type == b.record_type &&
         a.fragment_count == b.fragment_count &&
         a.logical_length == b.logical_length &&
         a.logical_crc16 == b.logical_crc16;
}

}  // namespace

uint16_t maxFragmentData(uint16_t cap) {
  return cap > kFragmentPrefixBytes
             ? static_cast<uint16_t>(cap - kFragmentPrefixBytes)
             : 0;
}

bool encodeFragment(const FragmentHeader& header, const uint8_t* logical,
                    uint16_t logical_length, uint8_t* out, uint16_t out_cap,
                    uint16_t* out_length) {
  if (logical == nullptr || out == nullptr || out_length == nullptr ||
      header.schema_version != kSchemaVersion || header.session_id == 0 ||
      header.capture_id == 0 || header.fragment_count == 0 ||
      header.fragment_index >= header.fragment_count || logical_length == 0 ||
      logical_length != header.logical_length ||
      static_cast<uint8_t>(header.record_type) >
          static_cast<uint8_t>(RecordType::End)) {
    return false;
  }

  const uint16_t fragment_capacity = maxFragmentData(out_cap);
  if (fragment_capacity == 0) return false;
  const uint16_t expected_count = static_cast<uint16_t>(
      (logical_length + fragment_capacity - 1u) / fragment_capacity);
  if (expected_count == 0 || expected_count > 255u ||
      header.fragment_count != expected_count ||
      header.logical_crc16 != protocol::crc16(logical, logical_length)) {
    return false;
  }

  const uint32_t offset =
      static_cast<uint32_t>(header.fragment_index) * fragment_capacity;
  if (offset >= logical_length) return false;
  const uint16_t remaining =
      static_cast<uint16_t>(logical_length - static_cast<uint16_t>(offset));
  const uint16_t slice_length =
      remaining < fragment_capacity ? remaining : fragment_capacity;
  const uint16_t total_length =
      static_cast<uint16_t>(kFragmentPrefixBytes + slice_length);
    if (total_length > out_cap) return false;

    return encodeFragmentSlice(header, &logical[static_cast<uint16_t>(offset)],
                 slice_length, out, out_cap, out_length);
  }

  bool encodeFragmentSlice(const FragmentHeader& header,
               const uint8_t* fragment_bytes,
               uint16_t fragment_length, uint8_t* out,
               uint16_t out_cap, uint16_t* out_length) {
    if (fragment_bytes == nullptr || out == nullptr || out_length == nullptr ||
      header.schema_version != kSchemaVersion || header.session_id == 0 ||
      header.capture_id == 0 || header.fragment_count == 0 ||
      header.fragment_index >= header.fragment_count ||
      header.logical_length == 0 ||
      static_cast<uint8_t>(header.record_type) >
        static_cast<uint8_t>(RecordType::End)) {
    return false;
    }
    const uint16_t fragment_capacity = maxFragmentData(out_cap);
    if (fragment_capacity == 0) return false;
    const uint16_t expected_count = static_cast<uint16_t>(
      (header.logical_length + fragment_capacity - 1u) / fragment_capacity);
    if (expected_count == 0 || expected_count > 255u ||
      header.fragment_count != expected_count) {
    return false;
    }
    const uint32_t offset =
      static_cast<uint32_t>(header.fragment_index) * fragment_capacity;
    if (offset >= header.logical_length) return false;
    const uint16_t remaining = static_cast<uint16_t>(
      header.logical_length - static_cast<uint16_t>(offset));
    const uint16_t expected_length =
      remaining < fragment_capacity ? remaining : fragment_capacity;
    if (fragment_length != expected_length ||
      kFragmentPrefixBytes + fragment_length > out_cap) {
    return false;
    }

  writeU16(&out[0], header.schema_version);
  writeU32(&out[2], header.session_id);
  writeU32(&out[6], header.capture_id);
  writeU32(&out[10], header.record_seq);
  out[14] = static_cast<uint8_t>(header.record_type);
  out[15] = header.fragment_index;
  out[16] = header.fragment_count;
  writeU16(&out[17], header.logical_length);
  writeU16(&out[19], header.logical_crc16);
  for (uint16_t index = 0; index < fragment_length; ++index) {
    out[kFragmentPrefixBytes + index] = fragment_bytes[index];
  }
  *out_length = static_cast<uint16_t>(kFragmentPrefixBytes + fragment_length);
  return true;
}

ReassemblyResult acceptFragment(const uint8_t* payload, uint16_t payload_len,
                                uint8_t* out, uint16_t out_cap,
                                ReassemblyState* state) {
  if (out == nullptr || state == nullptr) return ReassemblyResult::BadRequest;

  FragmentHeader header;
  uint16_t fragment_data_len = 0;
  if (!decodeHeader(payload, payload_len, &header, &fragment_data_len) ||
      fragment_data_len == 0 || header.logical_length > out_cap) {
    return ReassemblyResult::BadRequest;
  }

  if (!state->active) {
    if (header.fragment_index != 0) return ReassemblyResult::OutOfOrder;
    state->header = header;
    state->received = 0;
    state->next_fragment = 0;
    state->active = true;
  } else if (!sameRecord(state->header, header)) {
    return ReassemblyResult::WrongRecord;
  }

  if (header.fragment_index != state->next_fragment ||
      static_cast<uint32_t>(state->received) + fragment_data_len >
          header.logical_length) {
    return ReassemblyResult::OutOfOrder;
  }
  for (uint16_t index = 0; index < fragment_data_len; ++index) {
    out[state->received + index] = payload[kFragmentPrefixBytes + index];
  }
  state->received = static_cast<uint16_t>(state->received + fragment_data_len);
  ++state->next_fragment;

  if (state->next_fragment < header.fragment_count) {
    return ReassemblyResult::Accepted;
  }
  if (state->received != header.logical_length ||
      protocol::crc16(out, state->received) != header.logical_crc16) {
    state->active = false;
    return ReassemblyResult::BadCrc;
  }
  state->active = false;
  return ReassemblyResult::Complete;
}

}  // namespace trace
}  // namespace hil