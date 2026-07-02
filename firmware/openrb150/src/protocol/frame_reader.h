#pragma once

// ===========================================================================
// Streaming COBS frame reader.
//
// Accumulates incoming bytes and detects complete frames delimited by 0x00
// (see framing.h). A single 0x00 serves as both the trailing delimiter of one
// frame and the leading delimiter of the next, so back-to-back frames are
// handled. The emitted body is the COBS-encoded bytes BETWEEN delimiters; feed
// it to decodeFrameBody().
//
// Static, heap-free, single-consumer. Overlong runs (buffer overflow) are
// dropped until the next delimiter so a corrupt stream cannot wedge the reader.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

#include "framing.h"

namespace protocol {

// Largest COBS-encoded body the reader will buffer (no delimiters).
constexpr size_t kMaxFrameBodyCobs = cobsMaxEncodedLen(kMaxFrameBody);

class FrameReader {
 public:
  FrameReader() { reset(); }

  void reset();

  // Feed one received byte. Returns true when a complete, non-empty frame body
  // is available via body()/length(); the caller should consume it before the
  // next push(). Returns false otherwise (mid-frame, delimiter with no data,
  // or a dropped overflow frame).
  bool push(uint8_t byte);

  const uint8_t* body() const { return buf_; }
  size_t length() const { return len_; }

  // Link-health counters (hexapod_src-lv6): complete frame bodies emitted and
  // frames dropped because they exceeded the buffer. Monotonic since reset();
  // exposed so api_stats telemetry can report USB rx health.
  uint32_t framesOk() const { return frames_ok_; }
  uint32_t overflowsDropped() const { return overflows_dropped_; }

 private:
  uint8_t buf_[kMaxFrameBodyCobs];
  size_t len_;
  bool overflow_;
  bool ready_;
  uint32_t frames_ok_;
  uint32_t overflows_dropped_;
};

}  // namespace protocol
