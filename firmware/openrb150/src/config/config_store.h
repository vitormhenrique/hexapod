#pragma once

// ===========================================================================
// Transactional A/B config store (portable, no Arduino deps).
//
// Persists a single opaque config payload in two redundant slots of a byte-
// addressable backend (the 24LC32 EEPROM on the robot, AGENTS.md 4.3). Each
// slot carries a 16-byte header:
//
//   magic(4) version(2) sequence(4) length(2) payload_crc(2) header_crc(2)
//
// Design goals (AGENTS.md 4.3):
//   * Load the newest VALID slot at boot (highest sequence wins).
//   * Reject corrupt slots via header_crc + payload_crc.
//   * Power-cycle safe: commit always writes the OTHER slot with sequence+1, so
//     the currently valid slot is never touched mid-write. A crash during
//     commit leaves the previous good slot intact.
//   * No heap: the caller owns the payload buffer.
//
// The byte IO is abstracted behind ConfigBackend so this logic is unit-tested
// on the host with a RAM-backed fake. The 24LC32 driver implements the same
// interface on-target (eeprom_24lc32.h).
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace config {

constexpr uint32_t kConfigMagic = 0x48455843;  // 'HEXC' little-endian
constexpr uint16_t kConfigVersion = 1;

constexpr uint8_t kSlotCount = 2;
constexpr uint16_t kSlotSize = 2048;      // half of the 4096-byte EEPROM
constexpr uint16_t kHeaderSize = 16;      // serialized slot header
constexpr uint16_t kMaxPayload = kSlotSize - kHeaderSize;  // 2032

// EEPROM byte offset of each slot.
constexpr uint16_t kSlotAddr[kSlotCount] = {0, kSlotSize};

// Decoded slot header (host representation).
struct SlotHeader {
  uint32_t magic = 0;
  uint16_t version = 0;
  uint32_t sequence = 0;
  uint16_t length = 0;
  uint16_t payload_crc = 0;
  uint16_t header_crc = 0;
};

// Abstract byte-addressable persistence backend. Implementations must perform
// any page splitting / write-completion polling internally so callers see a
// simple read/write of arbitrary [addr, addr+len) within the device.
class ConfigBackend {
 public:
  virtual bool read(uint16_t addr, uint8_t* buf, uint16_t len) = 0;
  virtual bool write(uint16_t addr, const uint8_t* buf, uint16_t len) = 0;

 protected:
  ~ConfigBackend() = default;
};

// Per-slot validity summary (diagnostics / tests).
struct SlotStatus {
  bool valid = false;
  uint32_t sequence = 0;
  uint16_t length = 0;
};

// Serialize/deserialize a header to/from its 16-byte on-EEPROM form. Exposed
// for testing; header_crc is computed/validated by the store, not here.
void serializeHeader(const SlotHeader& h, uint8_t out[kHeaderSize]);
void deserializeHeader(const uint8_t in[kHeaderSize], SlotHeader& h);

// CRC-16 over a header's first 14 bytes (everything before header_crc).
uint16_t headerCrc(const uint8_t serialized[kHeaderSize]);

class ConfigStore {
 public:
  explicit ConfigStore(ConfigBackend& backend) : backend_(backend) {}

  // Load the newest valid slot's payload into `out` (capacity `max_len`).
  // Returns true and sets `out_len` on success; false if neither slot is valid
  // (blank or fully corrupt EEPROM -> caller should fall back to defaults).
  bool load(uint8_t* out, uint16_t max_len, uint16_t& out_len);

  // Commit a new payload. Writes the inactive slot with sequence = newest+1,
  // payload first then header, so a crash cannot corrupt the active slot.
  // Returns false if len exceeds kMaxPayload or the backend write fails.
  bool commit(const uint8_t* payload, uint16_t len);

  // Validate both slots without copying payloads.
  void inspect(SlotStatus status[kSlotCount]);

 private:
  // Read + validate one slot's header. `valid` reflects magic/version/header_crc.
  bool readHeader(uint8_t slot, SlotHeader& h, bool& valid);
  // Verify the payload CRC of an already header-valid slot.
  bool payloadValid(uint8_t slot, const SlotHeader& h);
  // Index of the newest fully valid slot, or -1 if none.
  int newestValidSlot(SlotHeader* out_header);

  ConfigBackend& backend_;
};

}  // namespace config
