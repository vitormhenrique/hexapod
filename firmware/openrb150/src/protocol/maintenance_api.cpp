#include "maintenance_api.h"

namespace protocol {
namespace {

void putU32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

uint32_t readU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

void MaintenanceApi::reset() {
  locked_ = false;
  token_ = 0;
  next_token_ = 1;
  last_hb_ms_ = 0;
  ttl_ms_ = kDefaultTtlMs;
  now_ms_ = 0;
  live_state_ = 0;
}

void MaintenanceApi::revoke() {
  locked_ = false;
  token_ = 0;
}

bool MaintenanceApi::writeResult(MaintResult r, uint8_t* out, size_t out_cap,
                                 uint16_t* out_len, uint8_t* out_flags) const {
  if (out_cap < 2) {
    out[0] = static_cast<uint8_t>(MaintResult::BadRequest);
    *out_len = 1;
    *out_flags = 0x02;  // flag::kError
    return true;
  }
  out[0] = static_cast<uint8_t>(r);
  out[1] = live_state_;
  *out_len = 2;
  *out_flags = (r == MaintResult::BadRequest) ? 0x02 : 0x00;
  return true;
}

bool MaintenanceApi::handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
                            uint8_t* out, size_t out_cap, uint16_t* out_len,
                            uint8_t* out_flags) {
  if (!maintmsg::isMaintMsg(msg_id)) return false;

  // Drop a stale lock before evaluating any command so a lapsed holder cannot
  // block a fresh ENTER and EXIT/HEARTBEAT see the true state.
  if (locked_ && (now_ms_ - last_hb_ms_) > ttl_ms_) {
    locked_ = false;
    token_ = 0;
  }

  switch (msg_id) {
    case maintmsg::kEnter: {
      if (!canEnter()) {
        return writeResult(MaintResult::Rejected, out, out_cap, out_len,
                           out_flags);
      }
      if (locked_) {
        // A valid lock is held: idempotent for the same... we cannot know the
        // caller's token on ENTER, so a held lock is reported Busy.
        return writeResult(MaintResult::Busy, out, out_cap, out_len, out_flags);
      }
      // Grant a fresh token.
      token_ = next_token_++;
      if (next_token_ == 0) next_token_ = 1;  // never hand out 0
      locked_ = true;
      last_hb_ms_ = now_ms_;
      if (out_cap < 6) {
        return writeResult(MaintResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      out[0] = static_cast<uint8_t>(MaintResult::Ok);
      out[1] = live_state_;
      putU32(&out[2], token_);
      *out_len = 6;
      *out_flags = 0x00;
      return true;
    }
    case maintmsg::kExit: {
      if (req_len < 4) {
        return writeResult(MaintResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      const uint32_t tok = readU32(req);
      if (!locked_ || tok != token_) {
        return writeResult(MaintResult::BadToken, out, out_cap, out_len,
                           out_flags);
      }
      locked_ = false;
      token_ = 0;
      return writeResult(MaintResult::Ok, out, out_cap, out_len, out_flags);
    }
    case maintmsg::kHeartbeat: {
      if (req_len < 4) {
        return writeResult(MaintResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      const uint32_t tok = readU32(req);
      if (!locked_ || tok != token_) {
        return writeResult(MaintResult::BadToken, out, out_cap, out_len,
                           out_flags);
      }
      last_hb_ms_ = now_ms_;
      return writeResult(MaintResult::Ok, out, out_cap, out_len, out_flags);
    }
    default:
      // Reserved-but-unimplemented maintenance id: fall through to UnknownMsg.
      return false;
  }
}

}  // namespace protocol
