#pragma once

// ===========================================================================
// Protocol frame format (AGENTS.md 6.1).
//
// On the wire:
//   0x00  COBS( header || payload || crc16 )  0x00
//
// The CRC-16/CCITT-FALSE is computed over header || payload (the unencoded
// bytes), appended little-endian, and the whole lot is COBS-encoded so it
// contains no 0x00 bytes. A single 0x00 delimiter brackets each frame.
//
// 14-byte header, all multi-byte integers little-endian:
//   magic(1) ver_major(1) ver_minor(1) msg_type(1) msg_id(1) flags(1)
//   seq(2) timestamp_ms(4) payload_len(2)
//
// Portable C++, no heap: the caller owns all buffers.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

#include "cobs.h"

namespace protocol {

constexpr uint8_t kMagic = 0xA5;
constexpr uint8_t kVersionMajor = 0;
constexpr uint8_t kVersionMinor = 1;

constexpr size_t kHeaderLen = 14;
constexpr size_t kCrcLen = 2;
constexpr size_t kMaxPayload = 256;

// Largest unencoded frame body (header + payload + crc).
constexpr size_t kMaxFrameBody = kHeaderLen + kMaxPayload + kCrcLen;
// Largest on-wire frame: COBS-encoded body plus two 0x00 delimiters.
constexpr size_t kMaxWireFrame = cobsMaxEncodedLen(kMaxFrameBody) + 2;

enum class MsgType : uint8_t {
  Command = 0,
  Response = 1,
  Telemetry = 2,
  Event = 3,
};

struct Header {
  uint8_t magic = kMagic;
  uint8_t version_major = kVersionMajor;
  uint8_t version_minor = kVersionMinor;
  uint8_t msg_type = 0;
  uint8_t msg_id = 0;
  uint8_t flags = 0;
  uint16_t seq = 0;
  uint32_t timestamp_ms = 0;
  uint16_t payload_len = 0;
};

enum class DecodeStatus : uint8_t {
  Ok = 0,
  TooShort,       // not enough bytes for header + crc
  BadCobs,        // COBS decode failed
  BadMagic,       // magic mismatch
  BadCrc,         // CRC check failed
  BadLength,      // payload_len inconsistent with frame / over kMaxPayload
  BufferTooSmall, // output payload buffer too small
};

// Encode a frame into `out` (must be >= kMaxWireFrame). `payload` may be null
// when payload_len == 0. Returns on-wire byte count (including both 0x00
// delimiters), or 0 on error (payload too large / output buffer too small).
size_t encodeFrame(const Header& header, const uint8_t* payload,
                   uint8_t* out, size_t out_cap);

// Decode one COBS-encoded frame body (the bytes BETWEEN the 0x00 delimiters,
// delimiters excluded). Fills `header` and copies the payload into
// `payload_out` (capacity `payload_cap`). On success returns DecodeStatus::Ok
// and sets *payload_len. Corrupt frames return a specific failure status.
DecodeStatus decodeFrameBody(const uint8_t* body, size_t body_len,
                             Header* header, uint8_t* payload_out,
                             size_t payload_cap, size_t* payload_len);

}  // namespace protocol
