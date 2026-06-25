#include "framing.h"

#include "crc16.h"

namespace protocol {
namespace {

void writeHeader(const Header& h, uint8_t* b) {
  b[0] = h.magic;
  b[1] = h.version_major;
  b[2] = h.version_minor;
  b[3] = h.msg_type;
  b[4] = h.msg_id;
  b[5] = h.flags;
  b[6] = static_cast<uint8_t>(h.seq & 0xFF);
  b[7] = static_cast<uint8_t>((h.seq >> 8) & 0xFF);
  b[8] = static_cast<uint8_t>(h.timestamp_ms & 0xFF);
  b[9] = static_cast<uint8_t>((h.timestamp_ms >> 8) & 0xFF);
  b[10] = static_cast<uint8_t>((h.timestamp_ms >> 16) & 0xFF);
  b[11] = static_cast<uint8_t>((h.timestamp_ms >> 24) & 0xFF);
  b[12] = static_cast<uint8_t>(h.payload_len & 0xFF);
  b[13] = static_cast<uint8_t>((h.payload_len >> 8) & 0xFF);
}

void readHeader(const uint8_t* b, Header* h) {
  h->magic = b[0];
  h->version_major = b[1];
  h->version_minor = b[2];
  h->msg_type = b[3];
  h->msg_id = b[4];
  h->flags = b[5];
  h->seq = static_cast<uint16_t>(b[6] | (b[7] << 8));
  h->timestamp_ms = static_cast<uint32_t>(b[8]) |
                    (static_cast<uint32_t>(b[9]) << 8) |
                    (static_cast<uint32_t>(b[10]) << 16) |
                    (static_cast<uint32_t>(b[11]) << 24);
  h->payload_len = static_cast<uint16_t>(b[12] | (b[13] << 8));
}

}  // namespace

size_t encodeFrame(const Header& header, const uint8_t* payload,
                   uint8_t* out, size_t out_cap) {
  if (header.payload_len > kMaxPayload) {
    return 0;
  }

  // Assemble header || payload || crc16 into a contiguous body buffer.
  uint8_t body[kMaxFrameBody];
  writeHeader(header, body);
  for (size_t i = 0; i < header.payload_len; ++i) {
    body[kHeaderLen + i] = payload[i];
  }
  const size_t crc_len_offset = kHeaderLen + header.payload_len;
  const uint16_t crc = crc16(body, crc_len_offset);
  body[crc_len_offset] = static_cast<uint8_t>(crc & 0xFF);
  body[crc_len_offset + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  const size_t body_len = crc_len_offset + kCrcLen;

  // COBS-encode the body one byte past the leading 0x00 delimiter.
  if (out_cap < cobsMaxEncodedLen(body_len) + 2) {
    return 0;
  }
  out[0] = 0x00;
  const size_t enc = cobsEncode(body, body_len, out + 1, out_cap - 2);
  if (enc == 0) {
    return 0;
  }
  out[1 + enc] = 0x00;
  return enc + 2;
}

DecodeStatus decodeFrameBody(const uint8_t* body_cobs, size_t body_len,
                             Header* header, uint8_t* payload_out,
                             size_t payload_cap, size_t* payload_len) {
  uint8_t body[kMaxFrameBody];
  const size_t decoded = cobsDecode(body_cobs, body_len, body, sizeof(body));
  if (decoded == 0) {
    return DecodeStatus::BadCobs;
  }
  if (decoded < kHeaderLen + kCrcLen) {
    return DecodeStatus::TooShort;
  }
  if (body[0] != kMagic) {
    return DecodeStatus::BadMagic;
  }

  readHeader(body, header);

  // payload_len must be consistent with the decoded byte count.
  if (header->payload_len > kMaxPayload ||
      kHeaderLen + header->payload_len + kCrcLen != decoded) {
    return DecodeStatus::BadLength;
  }

  const size_t crc_offset = kHeaderLen + header->payload_len;
  const uint16_t want = crc16(body, crc_offset);
  const uint16_t got = static_cast<uint16_t>(body[crc_offset] |
                                             (body[crc_offset + 1] << 8));
  if (want != got) {
    return DecodeStatus::BadCrc;
  }

  if (header->payload_len > payload_cap) {
    return DecodeStatus::BufferTooSmall;
  }
  for (size_t i = 0; i < header->payload_len; ++i) {
    payload_out[i] = body[kHeaderLen + i];
  }
  if (payload_len != nullptr) {
    *payload_len = header->payload_len;
  }
  return DecodeStatus::Ok;
}

}  // namespace protocol
