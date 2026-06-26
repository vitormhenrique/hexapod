#include "telemetry.h"

namespace protocol {

namespace {

// Per-stream safe maximum publish rate (Hz), indexed by StreamId. These cap
// what a client may request so the USB link / MCU are not overloaded
// (AGENTS.md 6.3 typical rates).
constexpr uint16_t kMaxRate[kNumStreams] = {
    10,   // Health
    100,  // ControlState
    50,   // ServoStatus
    100,  // ContactState
    100,  // I2cSensorsRaw
    50,   // RcInput
    10,   // ApiStats
    50,   // JointState
    50,   // ServoGoals
    50,   // LegState
};

inline uint16_t readU16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

inline void writeU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void writeU32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

uint16_t clampRate(StreamId s, uint16_t rate_hz) {
  const uint16_t max = SubscriptionManager::maxRateHz(s);
  if (rate_hz < kMinRateHz) return kMinRateHz;
  if (rate_hz > max) return max;
  return rate_hz;
}

}  // namespace

uint16_t SubscriptionManager::maxRateHz(StreamId stream) {
  const uint8_t i = static_cast<uint8_t>(stream);
  if (i >= kNumStreams) return 0;
  return kMaxRate[i];
}

void SubscriptionManager::reset() {
  for (uint8_t i = 0; i < kNumStreams; ++i) {
    streams_[i] = StreamState{};
  }
  tx_backlog_ = 0;
}

uint16_t SubscriptionManager::subscribe(StreamId stream, uint16_t rate_hz) {
  if (!validStream(static_cast<uint8_t>(stream))) return 0;
  StreamState* st = at(stream);
  const uint16_t eff = clampRate(stream, rate_hz);
  st->enabled = true;
  st->primed = false;  // re-prime phase on (re)subscribe
  st->rate_hz = eff;
  st->interval_ms = static_cast<uint16_t>(1000u / eff);
  if (st->interval_ms == 0) st->interval_ms = 1;
  return eff;
}

uint16_t SubscriptionManager::setRate(StreamId stream, uint16_t rate_hz) {
  if (!validStream(static_cast<uint8_t>(stream))) return 0;
  StreamState* st = at(stream);
  const uint16_t eff = clampRate(stream, rate_hz);
  st->rate_hz = eff;
  st->interval_ms = static_cast<uint16_t>(1000u / eff);
  if (st->interval_ms == 0) st->interval_ms = 1;
  st->primed = false;  // phase realigns to the new interval
  return eff;
}

void SubscriptionManager::unsubscribe(StreamId stream) {
  if (!validStream(static_cast<uint8_t>(stream))) return;
  StreamState* st = at(stream);
  st->enabled = false;
  st->primed = false;
}

bool SubscriptionManager::enabled(StreamId stream) const {
  if (!validStream(static_cast<uint8_t>(stream))) return false;
  return at(stream)->enabled;
}

uint16_t SubscriptionManager::rateHz(StreamId stream) const {
  if (!validStream(static_cast<uint8_t>(stream))) return 0;
  return at(stream)->rate_hz;
}

uint32_t SubscriptionManager::emitted(StreamId stream) const {
  if (!validStream(static_cast<uint8_t>(stream))) return 0;
  return at(stream)->emitted;
}

uint32_t SubscriptionManager::dropped(StreamId stream) const {
  if (!validStream(static_cast<uint8_t>(stream))) return 0;
  return at(stream)->dropped;
}

bool SubscriptionManager::shouldEmit(StreamId stream, uint32_t now_ms) {
  if (!validStream(static_cast<uint8_t>(stream))) return false;
  StreamState* st = at(stream);
  if (!st->enabled || st->interval_ms == 0) return false;

  // First emission after subscribing: prime the phase, no drop accounting.
  if (!st->primed) {
    st->primed = true;
    st->last_emit_ms = now_ms;
    st->emitted++;
    return true;
  }

  const uint32_t elapsed = now_ms - st->last_emit_ms;
  if (elapsed < st->interval_ms) {
    return false;  // rate enforced: too soon
  }
  // One slot is on time; any extra whole slots were missed -> dropped.
  const uint32_t slots = elapsed / st->interval_ms;
  if (slots > 1) {
    st->dropped += (slots - 1);
  }
  // Advance the phase by whole intervals so the rate stays stable.
  st->last_emit_ms += slots * st->interval_ms;
  st->emitted++;
  return true;
}

bool SubscriptionManager::handle(uint8_t msg_id, const uint8_t* req,
                                 uint16_t req_len, uint8_t* out,
                                 uint16_t out_cap, uint16_t* out_len,
                                 uint8_t* out_flags) {
  if (!telemsg::isTelemetryMsg(msg_id)) return false;
  *out_flags = 0;
  *out_len = 0;

  switch (msg_id) {
    case telemsg::kSubscribe:
    case telemsg::kSetStreamRate: {
      if (req_len < 3 || out_cap < 4) {
        out[0] = static_cast<uint8_t>(SubResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;  // protocol error flag
        return true;
      }
      const uint8_t sid = req[0];
      const uint16_t rate = readU16(&req[1]);
      if (!validStream(sid)) {
        out[0] = static_cast<uint8_t>(SubResult::BadStream);
        out[1] = sid;
        *out_len = 2;
        *out_flags = 0x02;
        return true;
      }
      const StreamId s = static_cast<StreamId>(sid);
      const uint16_t eff = (msg_id == telemsg::kSubscribe)
                               ? subscribe(s, rate)
                               : setRate(s, rate);
      out[0] = static_cast<uint8_t>(SubResult::Ok);
      out[1] = sid;
      writeU16(&out[2], eff);
      *out_len = 4;
      return true;
    }
    case telemsg::kUnsubscribe: {
      if (req_len < 1 || out_cap < 2) {
        out[0] = static_cast<uint8_t>(SubResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      const uint8_t sid = req[0];
      if (!validStream(sid)) {
        out[0] = static_cast<uint8_t>(SubResult::BadStream);
        out[1] = sid;
        *out_len = 2;
        *out_flags = 0x02;
        return true;
      }
      unsubscribe(static_cast<StreamId>(sid));
      out[0] = static_cast<uint8_t>(SubResult::Ok);
      out[1] = sid;
      *out_len = 2;
      return true;
    }
    case telemsg::kGetStreamStats: {
      // Response: count(1), tx_backlog(4), then per stream 12 bytes:
      //   stream_id(1) enabled(1) rate_hz(2) emitted(4) dropped(4)
      const uint16_t need = 1 + 4 + kNumStreams * 12;
      if (out_cap < need) {
        out[0] = static_cast<uint8_t>(SubResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      uint16_t o = 0;
      out[o++] = kNumStreams;
      writeU32(&out[o], tx_backlog_);
      o += 4;
      for (uint8_t i = 0; i < kNumStreams; ++i) {
        const StreamState& st = streams_[i];
        out[o++] = i;
        out[o++] = st.enabled ? 1 : 0;
        writeU16(&out[o], st.rate_hz);
        o += 2;
        writeU32(&out[o], st.emitted);
        o += 4;
        writeU32(&out[o], st.dropped);
        o += 4;
      }
      *out_len = o;
      return true;
    }
    default:
      return false;
  }
}

}  // namespace protocol
