#include "passive_api.h"

namespace protocol {
namespace {

constexpr uint8_t kErrorFlag = 0x02;  // mirrors api::flag::kError

uint16_t readU16(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

void putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

}  // namespace

void PassiveApi::reset() {
  requested_ = false;
  rate_hz_ = passiverate::kDefaultHz;
  zero_seq_ = 0;
  live_state_ = 0;
}

bool PassiveApi::writeResult(PassiveResult r, uint8_t* out, size_t out_cap,
                             uint16_t* out_len, uint8_t* out_flags,
                             const uint8_t* extra, uint16_t extra_len) const {
  const uint16_t need = static_cast<uint16_t>(2 + extra_len);
  if (out_cap < need) {
    out[0] = static_cast<uint8_t>(PassiveResult::BadRequest);
    *out_len = 1;
    *out_flags = kErrorFlag;
    return true;
  }
  out[0] = static_cast<uint8_t>(r);
  out[1] = live_state_;
  for (uint16_t i = 0; i < extra_len; ++i) out[2 + i] = extra[i];
  *out_len = need;
  *out_flags = (r == PassiveResult::BadRequest) ? kErrorFlag : 0x00;
  return true;
}

bool PassiveApi::handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
                        uint8_t* out, size_t out_cap, uint16_t* out_len,
                        uint8_t* out_flags) {
  if (!passivemsg::isPassiveMsg(msg_id)) return false;
  (void)req;

  switch (msg_id) {
    case passivemsg::kEnter: {
      // Only from a maintenance-safe state (or already streaming: idempotent).
      if (!canEnter()) {
        return writeResult(PassiveResult::Rejected, out, out_cap, out_len,
                           out_flags);
      }
      requested_ = true;
      return writeResult(PassiveResult::Ok, out, out_cap, out_len, out_flags);
    }
    case passivemsg::kExit: {
      // Always honoured; it only ever reduces authority.
      requested_ = false;
      return writeResult(PassiveResult::Ok, out, out_cap, out_len, out_flags);
    }
    case passivemsg::kSetStreamRate: {
      if (req_len < 2) {
        return writeResult(PassiveResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      // Only meaningful while passive streaming is requested.
      if (!requested_) {
        return writeResult(PassiveResult::Rejected, out, out_cap, out_len,
                           out_flags);
      }
      const uint16_t rate = readU16(req);
      if (rate < passiverate::kMinHz || rate > passiverate::kMaxHz) {
        return writeResult(PassiveResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      rate_hz_ = rate;
      uint8_t extra[2];
      putU16(extra, rate_hz_);
      return writeResult(PassiveResult::Ok, out, out_cap, out_len, out_flags,
                         extra, 2);
    }
    case passivemsg::kZeroReference: {
      // Only meaningful while passive streaming is requested.
      if (!requested_) {
        return writeResult(PassiveResult::Rejected, out, out_cap, out_len,
                           out_flags);
      }
      ++zero_seq_;
      return writeResult(PassiveResult::Ok, out, out_cap, out_len, out_flags);
    }
    default:
      return false;
  }
}

}  // namespace protocol
