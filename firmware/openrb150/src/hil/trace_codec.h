#pragma once

#include <stddef.h>
#include <stdint.h>

namespace hil {
namespace trace {

constexpr uint16_t kSchemaVersion = 1;
constexpr uint8_t kTraceFragmentEvent = 0xA0;
constexpr uint8_t kOutputBlockedEvent = 0xA1;

// The common event payload prefix defined by hil_serial_authority_contract.md.
constexpr uint16_t kFragmentPrefixBytes = 21;

enum class RecordType : uint8_t {
  Begin = 0,
  Config = 1,
  Step = 2,
  Marker = 3,
  OutputBlocked = 4,
  End = 5,
};

enum class ReassemblyResult : uint8_t {
  Accepted = 0,
  Complete = 1,
  BadRequest = 2,
  WrongRecord = 3,
  OutOfOrder = 4,
  Overflow = 5,
  BadCrc = 6,
};

struct FragmentHeader {
  uint16_t schema_version = kSchemaVersion;
  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  uint32_t record_seq = 0;
  RecordType record_type = RecordType::Begin;
  uint8_t fragment_index = 0;
  uint8_t fragment_count = 0;
  uint16_t logical_length = 0;
  uint16_t logical_crc16 = 0;
};

// Caller-owned state and output storage keep the codec heap-free and allow the
// MCU sender to avoid a second full logical-record allocation.
struct ReassemblyState {
  FragmentHeader header{};
  uint16_t received = 0;
  uint8_t next_fragment = 0;
  bool active = false;
};

// Returns the largest logical slice that fits in an event payload of `cap`.
uint16_t maxFragmentData(uint16_t cap);

// Builds one full event payload, including the common fragment prefix. The
// logical record is supplied as a caller-owned byte sequence and is never
// retained by the codec. Returns false for malformed sizes/indexes or when
// `out` cannot hold the requested payload.
bool encodeFragment(const FragmentHeader& header, const uint8_t* logical,
                    uint16_t logical_length, uint8_t* out, uint16_t out_cap,
                    uint16_t* out_length);

// Builds one fragment from the slice selected by `header.fragment_index`.
// The caller has already produced `fragment_bytes` from the canonical logical
// record, which avoids a second full-record buffer on the ATSAMD21.
bool encodeFragmentSlice(const FragmentHeader& header,
                         const uint8_t* fragment_bytes,
                         uint16_t fragment_length, uint8_t* out,
                         uint16_t out_cap, uint16_t* out_length);

// Validates one ordered fragment and copies its logical slice into `out`.
// `out` must remain the same caller-owned buffer for the whole record. A
// completed record is CRC-checked before `Complete` is returned.
ReassemblyResult acceptFragment(const uint8_t* payload, uint16_t payload_len,
                                uint8_t* out, uint16_t out_cap,
                                ReassemblyState* state);

}  // namespace trace
}  // namespace hil