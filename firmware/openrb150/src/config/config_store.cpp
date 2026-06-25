#include "config_store.h"

#include "../protocol/crc16.h"

namespace config {

namespace {

void put_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void put_u32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

uint16_t get_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t get_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

void serializeHeader(const SlotHeader& h, uint8_t out[kHeaderSize]) {
  put_u32(&out[0], h.magic);
  put_u16(&out[4], h.version);
  put_u32(&out[6], h.sequence);
  put_u16(&out[10], h.length);
  put_u16(&out[12], h.payload_crc);
  put_u16(&out[14], h.header_crc);
}

void deserializeHeader(const uint8_t in[kHeaderSize], SlotHeader& h) {
  h.magic = get_u32(&in[0]);
  h.version = get_u16(&in[4]);
  h.sequence = get_u32(&in[6]);
  h.length = get_u16(&in[10]);
  h.payload_crc = get_u16(&in[12]);
  h.header_crc = get_u16(&in[14]);
}

uint16_t headerCrc(const uint8_t serialized[kHeaderSize]) {
  // CRC covers everything before the trailing header_crc field (bytes 0..13).
  return protocol::crc16(serialized, kHeaderSize - 2);
}

bool ConfigStore::readHeader(uint8_t slot, SlotHeader& h, bool& valid) {
  valid = false;
  uint8_t raw[kHeaderSize];
  if (!backend_.read(kSlotAddr[slot], raw, kHeaderSize)) {
    return false;
  }
  deserializeHeader(raw, h);
  if (h.magic != kConfigMagic || h.version != kConfigVersion ||
      h.length > kMaxPayload) {
    return true;  // read ok, but slot not valid
  }
  if (headerCrc(raw) != h.header_crc) {
    return true;  // corrupt header
  }
  valid = true;
  return true;
}

bool ConfigStore::payloadValid(uint8_t slot, const SlotHeader& h) {
  uint16_t crc = protocol::kCrc16Init;
  const uint16_t base = static_cast<uint16_t>(kSlotAddr[slot] + kHeaderSize);
  uint8_t chunk[64];
  uint16_t remaining = h.length;
  uint16_t off = 0;
  while (remaining > 0) {
    const uint16_t n = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
    if (!backend_.read(static_cast<uint16_t>(base + off), chunk, n)) {
      return false;
    }
    crc = protocol::crc16Update(crc, chunk, n);
    off = static_cast<uint16_t>(off + n);
    remaining = static_cast<uint16_t>(remaining - n);
  }
  return crc == h.payload_crc;
}

int ConfigStore::newestValidSlot(SlotHeader* out_header) {
  int best = -1;
  SlotHeader best_h;
  for (uint8_t s = 0; s < kSlotCount; ++s) {
    SlotHeader h;
    bool valid = false;
    if (!readHeader(s, h, valid) || !valid) {
      continue;
    }
    if (!payloadValid(s, h)) {
      continue;
    }
    if (best < 0 || h.sequence > best_h.sequence) {
      best = s;
      best_h = h;
    }
  }
  if (best >= 0 && out_header != nullptr) {
    *out_header = best_h;
  }
  return best;
}

bool ConfigStore::load(uint8_t* out, uint16_t max_len, uint16_t& out_len) {
  SlotHeader h;
  const int slot = newestValidSlot(&h);
  if (slot < 0) {
    out_len = 0;
    return false;
  }
  if (h.length > max_len) {
    out_len = 0;
    return false;
  }
  const uint16_t base = static_cast<uint16_t>(kSlotAddr[slot] + kHeaderSize);
  if (!backend_.read(base, out, h.length)) {
    return false;
  }
  out_len = h.length;
  return true;
}

bool ConfigStore::commit(const uint8_t* payload, uint16_t len) {
  if (len > kMaxPayload) {
    return false;
  }

  SlotHeader newest;
  const int active = newestValidSlot(&newest);
  // Target the inactive slot; if none valid, start at slot 0.
  const uint8_t target = (active < 0) ? 0 : static_cast<uint8_t>(active ^ 1);
  const uint32_t seq = (active < 0) ? 1 : newest.sequence + 1;

  const uint16_t payload_addr =
      static_cast<uint16_t>(kSlotAddr[target] + kHeaderSize);

  // Write payload first; the header (with CRCs) is committed last so a partial
  // write is never seen as valid.
  if (len > 0 && !backend_.write(payload_addr, payload, len)) {
    return false;
  }

  SlotHeader h;
  h.magic = kConfigMagic;
  h.version = kConfigVersion;
  h.sequence = seq;
  h.length = len;
  h.payload_crc = protocol::crc16(payload, len);

  uint8_t raw[kHeaderSize];
  serializeHeader(h, raw);
  h.header_crc = headerCrc(raw);
  put_u16(&raw[14], h.header_crc);

  return backend_.write(kSlotAddr[target], raw, kHeaderSize);
}

void ConfigStore::inspect(SlotStatus status[kSlotCount]) {
  for (uint8_t s = 0; s < kSlotCount; ++s) {
    SlotHeader h;
    bool valid = false;
    status[s] = SlotStatus{};
    if (!readHeader(s, h, valid) || !valid) {
      continue;
    }
    status[s].sequence = h.sequence;
    status[s].length = h.length;
    status[s].valid = payloadValid(s, h);
  }
}

}  // namespace config
