#include "crsf_parser.h"

namespace crsf {

namespace {
// Microsecond thresholds for switch interpretation (mid = 1500 us).
constexpr uint16_t kSwitchHigh = 1500;  // > => switch active
constexpr uint16_t kGaitLow = 1300;     // < => gait 0
constexpr uint16_t kGaitHigh = 1700;    // >= => gait 2, else gait 1
}  // namespace

uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

uint16_t ticksToMicros(uint16_t ticks) {
  // Linear map matching the common CRSF convention: us = ticks * 5/8 + 880,
  // giving ~988 us at tick 172 and ~2012 us at tick 1811.
  return static_cast<uint16_t>(((static_cast<uint32_t>(ticks) * 5) >> 3) + 880);
}

float stickToUnit(uint16_t micros) {
  int v = static_cast<int>(micros) - static_cast<int>(kMicrosMid);
  const int dead = static_cast<int>(kStickDeadbandUs);
  if (v > -dead && v < dead) return 0.0f;
  // Bias out the deadband so the command ramps from zero at the deadband edge.
  v += (v > 0) ? -dead : dead;
  float span = static_cast<float>(kStickHalfSpanUs) - static_cast<float>(dead);
  if (span < 1.0f) span = 1.0f;  // guard against a degenerate deadband config
  float u = static_cast<float>(v) / span;
  if (u > 1.0f) u = 1.0f;
  if (u < -1.0f) u = -1.0f;
  return u;
}

void Parser::reset() {
  idx_ = 0;
  frame_len_ = 0;
  state_ = State::Sync;
  frames_decoded_ = 0;
  crc_errors_ = 0;
}

void Parser::unpackChannels(ChannelData& out) const {
  // Payload starts after sync(0), length(1), type(2): first payload byte is
  // buf_[3]. 16 channels are packed little-endian as contiguous 11-bit fields.
  const uint8_t* d = &buf_[3];
  uint32_t bits = 0;
  uint8_t bits_avail = 0;
  uint8_t byte_idx = 0;
  uint8_t ch = 0;
  while (ch < kNumChannels) {
    if (bits_avail < 11) {
      bits |= static_cast<uint32_t>(d[byte_idx++]) << bits_avail;
      bits_avail += 8;
    } else {
      out.channels[ch++] = static_cast<uint16_t>(bits & 0x7FF);
      bits >>= 11;
      bits_avail -= 11;
    }
  }
}

bool Parser::push(uint8_t byte, ChannelData& out) {
  switch (state_) {
    case State::Sync:
      if (byte == kSyncByte) {
        buf_[0] = byte;
        idx_ = 1;
        state_ = State::Length;
      }
      return false;

    case State::Length:
      // frame_len counts type + payload + crc. Reject implausible lengths.
      if (byte < 2 || byte > (kMaxFrameLen - 2)) {
        state_ = State::Sync;
        return false;
      }
      frame_len_ = byte;
      buf_[1] = byte;
      idx_ = 2;
      state_ = State::Data;
      return false;

    case State::Data: {
      buf_[idx_++] = byte;
      // Full frame received once we have sync + length + frame_len bytes.
      if (idx_ < static_cast<uint8_t>(2 + frame_len_)) {
        return false;
      }
      state_ = State::Sync;

      // CRC covers type + payload (frame_len - 1 bytes) starting at buf_[2];
      // the final byte is the transmitted CRC.
      const uint8_t crc_calc = crc8(&buf_[2], static_cast<uint8_t>(frame_len_ - 1));
      const uint8_t crc_rx = buf_[2 + frame_len_ - 1];
      if (crc_calc != crc_rx) {
        ++crc_errors_;
        return false;
      }

      const uint8_t type = buf_[2];
      if (type != kFrameTypeRcChannels ||
          frame_len_ != (kRcPayloadBytes + 2)) {
        // Valid CRSF frame, but not the RC channels frame we consume.
        return false;
      }

      unpackChannels(out);
      ++frames_decoded_;
      return true;
    }
  }
  return false;
}

void initRcStatus(RcStatus& rc) {
  for (uint8_t i = 0; i < kNumChannels; ++i) {
    rc.channels_us[i] = kMicrosMid;
  }
  rc.armed = false;
  rc.kill = true;
  rc.gait_index = 0;
  rc.autonomy = false;
  rc.failsafe = true;
  rc.last_frame_ms = 0;
  rc.ever_seen = false;
}

void applyFrame(RcStatus& rc, const ChannelData& frame, uint32_t now_ms) {
  for (uint8_t i = 0; i < kNumChannels; ++i) {
    rc.channels_us[i] = ticksToMicros(frame.channels[i]);
  }

  const uint16_t gait_us = rc.channels_us[kChGait];
  if (gait_us < kGaitLow) {
    rc.gait_index = 0;
  } else if (gait_us >= kGaitHigh) {
    rc.gait_index = 2;
  } else {
    rc.gait_index = 1;
  }

  const bool kill_sw = rc.channels_us[kChKill] > kSwitchHigh;
  const bool arm_sw = rc.channels_us[kChArm] > kSwitchHigh;
  rc.kill = kill_sw;
  rc.armed = arm_sw && !kill_sw;
  rc.autonomy = rc.channels_us[kChAutonomy] > kSwitchHigh;

  rc.failsafe = false;
  rc.last_frame_ms = now_ms;
  rc.ever_seen = true;
}

void evaluateFailsafe(RcStatus& rc, uint32_t now_ms, uint32_t timeout_ms) {
  const bool stale = !rc.ever_seen || (now_ms - rc.last_frame_ms) > timeout_ms;
  if (stale) {
    rc.failsafe = true;
    rc.armed = false;
    rc.kill = true;
    rc.autonomy = false;
  }
}

}  // namespace crsf
