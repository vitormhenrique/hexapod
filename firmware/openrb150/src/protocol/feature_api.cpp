#include "feature_api.h"

namespace protocol {
namespace {

inline bool validFeature(uint8_t id) { return id < kFeatureCount; }

}  // namespace

void FeatureApi::reset() {
  for (uint8_t i = 0; i < kFeatureCount; ++i) {
    state_[i].desired = kFeatureDefaultEnabled[i];
    state_[i].available = false;
    state_[i].reason = FeatureReason::NotImplemented;
  }
  live_state_ = 0;
  seq_ = 0;
}

void FeatureApi::applyDefaults(uint32_t mask) {
  bool changed = false;
  for (uint8_t i = 0; i < kFeatureCount; ++i) {
    const bool want = ((mask >> i) & 1u) != 0u;
    if (state_[i].desired != want) {
      state_[i].desired = want;
      changed = true;
    }
  }
  if (changed) ++seq_;
}

void FeatureApi::setAvailability(Feature f, bool available,
                                 FeatureReason reason) {
  const uint8_t i = static_cast<uint8_t>(f);
  if (i >= kFeatureCount) return;
  state_[i].available = available;
  state_[i].reason = reason;
}

bool FeatureApi::desiredEnabled(Feature f) const {
  const uint8_t i = static_cast<uint8_t>(f);
  if (i >= kFeatureCount) return false;
  return state_[i].desired;
}

bool FeatureApi::effectiveEnabled(Feature f) const {
  const uint8_t i = static_cast<uint8_t>(f);
  if (i >= kFeatureCount) return false;
  return state_[i].desired && state_[i].available;
}

uint32_t FeatureApi::availableMask() const {
  uint32_t mask = 0;
  for (uint8_t i = 0; i < kFeatureCount; ++i) {
    if (state_[i].available) mask |= (1u << i);
  }
  return mask;
}

FeatureApplyResult FeatureApi::applyDesired(Feature f, bool enable) {
  FeatureApplyResult r;
  const uint8_t i = static_cast<uint8_t>(f);
  if (i >= kFeatureCount) {
    r.result = FeatureResult::BadRequest;
    r.feature = i;
    r.available = false;
    r.enabled = false;
    r.reason = FeatureReason::None;
    return r;
  }
  State& s = state_[i];
  // Enabling a feature the firmware says is unavailable is rejected with the
  // reason echoed; disabling is always honoured (only reduces authority).
  if (enable && !s.available) {
    r.result = FeatureResult::Rejected;
  } else {
    if (s.desired != enable) ++seq_;
    s.desired = enable;
    r.result = FeatureResult::Ok;
  }
  r.feature = i;
  r.available = s.available;
  r.enabled = s.desired && s.available;
  r.reason = s.reason;
  return r;
}

uint16_t FeatureApi::writeFullState(uint8_t* out, uint16_t off) const {
  for (uint8_t i = 0; i < kFeatureCount; ++i) {
    out[off++] = i;
    out[off++] = state_[i].available ? 1 : 0;
    out[off++] = (state_[i].desired && state_[i].available) ? 1 : 0;
    out[off++] = static_cast<uint8_t>(state_[i].reason);
  }
  return off;
}

bool FeatureApi::handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
                        uint8_t* out, size_t out_cap, uint16_t* out_len,
                        uint8_t* out_flags) {
  if (!featuremsg::isFeatureMsg(msg_id)) return false;
  *out_flags = 0;

  switch (msg_id) {
    case featuremsg::kGet: {
      // [state, count, {id, available, enabled, reason} x count]
      const uint16_t need = 2 + 4 * kFeatureCount;
      if (out_cap < need) {
        out[0] = static_cast<uint8_t>(FeatureResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      out[0] = live_state_;
      out[1] = kFeatureCount;
      *out_len = writeFullState(out, 2);
      return true;
    }
    case featuremsg::kSet: {
      // req: [feature, enable]. resp: [result, state, id, available, enabled,
      // reason]
      if (req_len < 2) {
        out[0] = static_cast<uint8_t>(FeatureResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      const uint8_t fid = req[0];
      const bool enable = req[1] != 0;
      if (!validFeature(fid)) {
        out[0] = static_cast<uint8_t>(FeatureResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      const FeatureApplyResult r =
          applyDesired(static_cast<Feature>(fid), enable);
      out[0] = static_cast<uint8_t>(r.result);
      out[1] = live_state_;
      out[2] = r.feature;
      out[3] = r.available ? 1 : 0;
      out[4] = r.enabled ? 1 : 0;
      out[5] = static_cast<uint8_t>(r.reason);
      *out_len = 6;
      return true;
    }
    case featuremsg::kGetReasons: {
      // [state, count, {id, reason} x count]
      const uint16_t need = 2 + 2 * kFeatureCount;
      if (out_cap < need) {
        out[0] = static_cast<uint8_t>(FeatureResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      out[0] = live_state_;
      out[1] = kFeatureCount;
      uint16_t off = 2;
      for (uint8_t i = 0; i < kFeatureCount; ++i) {
        out[off++] = i;
        out[off++] = static_cast<uint8_t>(state_[i].reason);
      }
      *out_len = off;
      return true;
    }
    case featuremsg::kResetDefaults: {
      // resp: [result, state, count, {id, available, enabled, reason} x count]
      const uint16_t need = 3 + 4 * kFeatureCount;
      if (out_cap < need) {
        out[0] = static_cast<uint8_t>(FeatureResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      bool changed = false;
      for (uint8_t i = 0; i < kFeatureCount; ++i) {
        if (state_[i].desired != kFeatureDefaultEnabled[i]) changed = true;
        state_[i].desired = kFeatureDefaultEnabled[i];
      }
      if (changed) ++seq_;
      out[0] = static_cast<uint8_t>(FeatureResult::Ok);
      out[1] = live_state_;
      out[2] = kFeatureCount;
      *out_len = writeFullState(out, 3);
      return true;
    }
    default:
      return false;
  }
}

}  // namespace protocol
