#pragma once

// ===========================================================================
// Transactional append-only config store (portable, no Arduino deps).
//
// Persists opaque config payloads as records in one file on the Qwiic OpenLog.
// Each record carries a 16-byte header followed by the payload and an 8-byte
// commit trailer:
//
//   magic(4) version(2) sequence(4) length(2) payload_crc(2) header_crc(2)
//   payload(length)
//   trailer_magic(4) sequence(4)
//
// The trailer is appended last. A reset or card failure during a commit leaves
// an incomplete tail which is ignored, so the previous record remains valid.
//   * No heap: the caller owns the payload buffer.
//
// File IO is abstracted so this logic is unit-tested with a RAM-backed fake.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace config {

constexpr uint32_t kConfigMagic = 0x48455843;  // 'HEXC' little-endian
constexpr uint16_t kConfigVersion = 1;
constexpr uint32_t kCommitMagic = 0x54494D43;  // 'CMIT' little-endian

constexpr uint16_t kHeaderSize = 16;
constexpr uint16_t kTrailerSize = 8;
constexpr uint16_t kMaxPayload = 2032;

// Decoded slot header (host representation).
struct SlotHeader {
  uint32_t magic = 0;
  uint16_t version = 0;
  uint32_t sequence = 0;
  uint16_t length = 0;
  uint16_t payload_crc = 0;
  uint16_t header_crc = 0;
};

// Minimal append-file surface implemented by the Qwiic OpenLog driver.
class ConfigFile {
 public:
  virtual bool size(uint32_t& out) = 0;
  virtual bool read(uint32_t offset, uint8_t* buf, uint16_t len) = 0;
  virtual bool append(const uint8_t* buf, uint16_t len) = 0;
  virtual bool sync() = 0;

 protected:
  ~ConfigFile() = default;
};

struct StoreStatus {
  bool has_valid_record = false;
  uint32_t sequence = 0;
  uint16_t length = 0;
  uint32_t file_size = 0;
  uint32_t valid_records = 0;
};

// Serialize/deserialize a header to/from its 16-byte on-file form. Exposed
// for testing; header_crc is computed/validated by the store, not here.
void serializeHeader(const SlotHeader& h, uint8_t out[kHeaderSize]);
void deserializeHeader(const uint8_t in[kHeaderSize], SlotHeader& h);

// CRC-16 over a header's first 14 bytes (everything before header_crc).
uint16_t headerCrc(const uint8_t serialized[kHeaderSize]);

class ConfigStore {
 public:
  explicit ConfigStore(ConfigFile& file) : file_(file) {}

  // Load the newest complete, CRC-valid record.
  bool load(uint8_t* out, uint16_t max_len, uint16_t& out_len);

  // Append header, payload, and commit trailer, then flush the OpenLog file.
  bool commit(const uint8_t* payload, uint16_t len);

  bool inspect(StoreStatus& status);

 private:
  struct RecordLocation {
    uint32_t payload_offset = 0;
    SlotHeader header{};
  };

  bool scan(StoreStatus& status, RecordLocation* newest);
  bool payloadValid(uint32_t payload_offset, const SlotHeader& h);

  ConfigFile& file_;
  uint32_t latest_sequence_ = 0;
  bool sequence_known_ = false;
};

}  // namespace config
