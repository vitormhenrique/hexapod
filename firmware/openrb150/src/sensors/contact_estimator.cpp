#include "contact_estimator.h"

namespace sensors {

namespace {

// Saturating helper so confidence math stays in 0..255 without <algorithm>.
inline uint8_t clampU8(int32_t v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return static_cast<uint8_t>(v);
}

}  // namespace

void ContactEstimator::configure(const config::FootSensorCal (&cal)[kNumFeet],
                                 const ContactParams& params) {
  params_ = params;
  if (params_.release_den == 0) params_.release_den = 1;
  for (uint8_t i = 0; i < kNumFeet; ++i) {
    FootCtx& c = ctx_[i];
    c.enabled = cal[i].enabled != 0;
    c.near_thresh = cal[i].near_thresh;
    c.touch_thresh = cal[i].touch_thresh;
    c.load_thresh = cal[i].load_thresh;
    // Release threshold is a hysteresis fraction of the touch threshold.
    c.release_thresh = (static_cast<int32_t>(cal[i].touch_thresh) *
                        params_.release_num) /
                       params_.release_den;
    c.touch_count = 0;
    c.load_count = 0;
    c.release_count = 0;
    c.fault_count = 0;
    c.last_ok_ms = 0;
    c.seen = false;
    feet_[i].pressure_baseline = cal[i].pressure_baseline;
  }
}

void ContactEstimator::setThresholds(uint8_t leg, uint16_t near_thresh,
                                     uint16_t touch_thresh,
                                     uint16_t load_thresh) {
  if (leg >= kNumFeet) return;
  if (params_.release_den == 0) params_.release_den = 1;
  FootCtx& c = ctx_[leg];
  c.near_thresh = near_thresh;
  c.touch_thresh = touch_thresh;
  c.load_thresh = load_thresh;
  // Keep the release hysteresis fraction consistent with configure().
  c.release_thresh =
      (static_cast<int32_t>(touch_thresh) * params_.release_num) /
      params_.release_den;
}

void ContactEstimator::reset() {
  for (uint8_t i = 0; i < kNumFeet; ++i) {
    const int32_t keep_baseline = feet_[i].pressure_baseline;
    feet_[i] = LegContactState{};
    feet_[i].pressure_baseline = keep_baseline;
    ctx_[i].touch_count = 0;
    ctx_[i].load_count = 0;
    ctx_[i].release_count = 0;
    ctx_[i].fault_count = 0;
    ctx_[i].last_ok_ms = 0;
    ctx_[i].seen = false;
  }
}

uint8_t ContactEstimator::confidenceFor(const FootCtx& ctx,
                                        const LegContactState& st,
                                        int32_t delta) {
  switch (st.state) {
    case ContactState::Loaded: {
      if (ctx.load_thresh <= 0) return 255;
      // Scale how far past the load threshold we are, capped at full.
      const int32_t margin = delta - ctx.load_thresh;
      return clampU8(200 + (margin * 55) / (ctx.load_thresh + 1));
    }
    case ContactState::Touch:
      return 160;
    case ContactState::Near:
      return 96;
    case ContactState::Release:
      return 64;
    case ContactState::Air:
      return 0;
    default:
      return 0;
  }
}

void ContactEstimator::update(uint8_t leg, const FootSample& sample,
                              uint32_t now_ms) {
  if (leg >= kNumFeet) return;
  FootCtx& c = ctx_[leg];
  LegContactState& st = feet_[leg];

  // Failed read: count toward FAULT, never block. After fault_limit
  // consecutive failures the foot is marked FAULT until a good sample returns.
  if (!sample.ok) {
    if (c.fault_count < 255) c.fault_count++;
    if (c.fault_count >= params_.fault_limit) {
      st.state = ContactState::Fault;
      st.fault = true;
      st.near_surface = st.touch = st.loaded = st.release = false;
      st.confidence = 0;
    }
    return;
  }
  c.fault_count = 0;
  st.fault = false;
  c.last_ok_ms = now_ms;
  c.seen = true;
  st.timestamp_ms = now_ms;
  st.stale = false;

  st.proximity_raw = sample.proximity_raw;
  st.pressure_raw = sample.pressure_raw;

  // Disabled (uncalibrated) feet only mirror raw values and stay AIR.
  if (!c.enabled) {
    st.state = ContactState::Air;
    st.pressure_delta = 0;
    st.near_surface = st.touch = st.loaded = st.release = false;
    st.confidence = 0;
    return;
  }

  const int32_t delta = sample.pressure_raw - st.pressure_baseline;
  st.pressure_delta = delta;
  const bool near = sample.proximity_raw >= c.near_thresh;
  st.near_surface = near;

  // Debounced touch / load / release counters.
  if (delta >= c.touch_thresh) {
    if (c.touch_count < 255) c.touch_count++;
    c.release_count = 0;
  } else {
    c.touch_count = 0;
  }
  if (delta >= c.load_thresh) {
    if (c.load_count < 255) c.load_count++;
  } else {
    c.load_count = 0;
  }
  if (delta <= c.release_thresh && !near) {
    if (c.release_count < 255) c.release_count++;
  } else {
    c.release_count = 0;
  }

  const ContactState prev = st.state;
  ContactState next = prev;

  switch (prev) {
    case ContactState::Air:
    case ContactState::Stale:
    case ContactState::Fault:
      if (c.load_count >= params_.load_debounce) {
        next = ContactState::Loaded;
      } else if (c.touch_count >= params_.touch_debounce) {
        next = ContactState::Touch;
      } else if (near) {
        next = ContactState::Near;
      } else {
        next = ContactState::Air;
      }
      break;
    case ContactState::Near:
      if (c.load_count >= params_.load_debounce) {
        next = ContactState::Loaded;
      } else if (c.touch_count >= params_.touch_debounce) {
        next = ContactState::Touch;
      } else if (!near) {
        next = ContactState::Air;
      }
      break;
    case ContactState::Touch:
      if (c.load_count >= params_.load_debounce) {
        next = ContactState::Loaded;
      } else if (c.release_count >= params_.release_debounce) {
        next = ContactState::Release;
      }
      break;
    case ContactState::Loaded:
      if (c.release_count >= params_.release_debounce) {
        next = ContactState::Release;
      }
      break;
    case ContactState::Release:
      if (c.load_count >= params_.load_debounce) {
        next = ContactState::Loaded;
      } else if (c.touch_count >= params_.touch_debounce) {
        next = ContactState::Touch;
      } else if (!near) {
        next = ContactState::Air;
      } else {
        next = ContactState::Near;
      }
      break;
  }

  st.state = next;
  st.touch = (next == ContactState::Touch || next == ContactState::Loaded);
  st.loaded = (next == ContactState::Loaded);
  st.release = (next == ContactState::Release);

  // Slowly track the baseline toward the live reading while unloaded so slow
  // drift does not masquerade as contact. Never drift while bearing weight.
  if ((next == ContactState::Air || next == ContactState::Near) &&
      params_.baseline_track != 0) {
    if (sample.pressure_raw > st.pressure_baseline) {
      st.pressure_baseline += params_.baseline_track;
    } else if (sample.pressure_raw < st.pressure_baseline) {
      st.pressure_baseline -= params_.baseline_track;
    }
  }

  st.confidence = confidenceFor(c, st, delta);
}

void ContactEstimator::tickStaleness(uint32_t now_ms) {
  for (uint8_t i = 0; i < kNumFeet; ++i) {
    FootCtx& c = ctx_[i];
    LegContactState& st = feet_[i];
    if (st.state == ContactState::Fault) continue;  // FAULT clears on good read
    if (!c.seen) continue;  // never had a sample: leave at AIR
    if (now_ms - c.last_ok_ms >= params_.stale_timeout_ms) {
      st.state = ContactState::Stale;
      st.stale = true;
      st.near_surface = st.touch = st.loaded = st.release = false;
      st.confidence = 0;
    }
  }
}

uint8_t ContactEstimator::loadedMask() const {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < kNumFeet; ++i) {
    if (feet_[i].state == ContactState::Loaded) {
      mask = static_cast<uint8_t>(mask | (1u << i));
    }
  }
  return mask;
}

}  // namespace sensors
