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

bool ConfigStore::payloadValid(uint32_t payload_offset, const SlotHeader& h) {
  uint16_t crc = protocol::kCrc16Init;
  uint8_t chunk[64];
  uint16_t remaining = h.length;
  uint32_t off = 0;
  while (remaining > 0) {
    const uint16_t n = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
    if (!file_.read(payload_offset + off, chunk, n)) {
      return false;
    }
    crc = protocol::crc16Update(crc, chunk, n);
    off = static_cast<uint16_t>(off + n);
    remaining = static_cast<uint16_t>(remaining - n);
  }
  return crc == h.payload_crc;
}

bool ConfigStore::scan(StoreStatus& status, RecordLocation* newest) {
  status = StoreStatus{};
  if (!file_.size(status.file_size)) return false;

  uint32_t offset = 0;
  while (status.file_size - offset >= kHeaderSize + kTrailerSize) {
    uint8_t raw[kHeaderSize];
    if (!file_.read(offset, raw, sizeof(raw))) return false;
    SlotHeader h;
    deserializeHeader(raw, h);
    const bool header_valid =
        h.magic == kConfigMagic && h.version == kConfigVersion &&
        h.length <= kMaxPayload && headerCrc(raw) == h.header_crc;
    if (!header_valid) {
      ++offset;
      continue;
    }

    const uint32_t payload_offset = offset + kHeaderSize;
    const uint32_t trailer_offset = payload_offset + h.length;
    const uint32_t record_end = trailer_offset + kTrailerSize;
    if (record_end > status.file_size) break;

    const bool payload_valid = payloadValid(payload_offset, h);
    uint8_t trailer[kTrailerSize];
    if (!file_.read(trailer_offset, trailer, sizeof(trailer))) return false;
    const bool committed = get_u32(&trailer[0]) == kCommitMagic &&
                 get_u32(&trailer[4]) == h.sequence;
    if (payload_valid && committed) {
      ++status.valid_records;
      if (!status.has_valid_record || h.sequence > status.sequence) {
        status.has_valid_record = true;
        status.sequence = h.sequence;
        status.length = h.length;
        if (newest != nullptr) {
          newest->payload_offset = payload_offset;
          newest->header = h;
        }
      }
    }
    offset = record_end;
  }
  return true;
}

bool ConfigStore::load(uint8_t* out, uint16_t max_len, uint16_t& out_len) {
  StoreStatus status;
  RecordLocation newest;
  if (!scan(status, &newest)) {
    sequence_known_ = false;
    out_len = 0;
    return false;
  }
  latest_sequence_ = status.sequence;
  sequence_known_ = true;
  if (!status.has_valid_record) {
    out_len = 0;
    return false;
  }
  if (newest.header.length > max_len) {
    out_len = 0;
    return false;
  }
  if (!file_.read(newest.payload_offset, out, newest.header.length)) {
    return false;
  }
  out_len = newest.header.length;
  return true;
}

bool ConfigStore::commit(const uint8_t* payload, uint16_t len) {
  if (len > kMaxPayload) {
    return false;
  }

  if (!sequence_known_) {
    StoreStatus status;
    if (!scan(status, nullptr)) return false;
    latest_sequence_ = status.sequence;
    sequence_known_ = true;
  }
  const uint32_t seq = latest_sequence_ + 1u;

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

  uint8_t trailer[kTrailerSize];
  put_u32(&trailer[0], kCommitMagic);
  put_u32(&trailer[4], seq);

  if (!file_.append(raw, sizeof(raw))) return false;
  if (len > 0 && !file_.append(payload, len)) return false;
  if (!file_.append(trailer, sizeof(trailer))) return false;
  if (!file_.sync()) return false;
  latest_sequence_ = seq;
  return true;
}

bool ConfigStore::inspect(StoreStatus& status) {
  if (!scan(status, nullptr)) {
    sequence_known_ = false;
    return false;
  }
  latest_sequence_ = status.sequence;
  sequence_known_ = true;
  return true;
}

}  // namespace config
