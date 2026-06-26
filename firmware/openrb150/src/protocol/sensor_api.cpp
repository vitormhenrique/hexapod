#include "sensor_api.h"

namespace protocol {
namespace {

inline uint16_t get16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline uint16_t put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  return 2;
}

inline SensorResult mapResult(FeatureResult r) {
  switch (r) {
    case FeatureResult::Ok:
      return SensorResult::Ok;
    case FeatureResult::Rejected:
      return SensorResult::Rejected;
    default:
      return SensorResult::BadRequest;
  }
}

}  // namespace

void SensorApi::reset() {
  for (uint8_t i = 0; i < kSensorNumFeet; ++i) {
    thresholds_.near_thresh[i] = 0;
    thresholds_.touch_thresh[i] = 0;
    thresholds_.load_thresh[i] = 0;
  }
  threshold_seq_ = 0;
  leveling_.max_tilt_mdeg = 0;
  leveling_.rate_mdeg_s = 0;
  leveling_.response_x255 = 0;
  leveling_seq_ = 0;
}

bool SensorApi::applyFeature(Feature f, bool enable, uint8_t* out,
                            size_t out_cap, uint16_t* out_len,
                            uint8_t* out_flags) {
  if (features_ == nullptr || out_cap < 5) {
    out[0] = static_cast<uint8_t>(SensorResult::Rejected);
    *out_len = 1;
    *out_flags = 0x02;
    return true;
  }
  const FeatureApplyResult r = features_->applyDesired(f, enable);
  out[0] = static_cast<uint8_t>(mapResult(r.result));
  out[1] = features_->liveState();
  out[2] = r.available ? 1 : 0;
  out[3] = r.enabled ? 1 : 0;
  out[4] = static_cast<uint8_t>(r.reason);
  *out_len = 5;
  *out_flags = (r.result == FeatureResult::Ok) ? 0 : 0x02;
  return true;
}

bool SensorApi::handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
                       uint8_t* out, size_t out_cap, uint16_t* out_len,
                       uint8_t* out_flags) {
  if (!sensormsg::isSensorMsg(msg_id)) return false;
  *out_flags = 0;

  switch (msg_id) {
    case sensormsg::kContactEnable:
      return applyFeature(Feature::FootContact, true, out, out_cap, out_len,
                          out_flags);
    case sensormsg::kContactDisable:
      return applyFeature(Feature::FootContact, false, out, out_cap, out_len,
                          out_flags);
    case sensormsg::kLevelingEnable:
      return applyFeature(Feature::TerrainLeveling, true, out, out_cap, out_len,
                          out_flags);
    case sensormsg::kLevelingDisable:
      return applyFeature(Feature::TerrainLeveling, false, out, out_cap,
                          out_len, out_flags);

    case sensormsg::kContactSetThresholds: {
      // req: [foot, near u16, touch u16, load u16]. resp echoes applied values.
      if (req_len < 7) {
        out[0] = static_cast<uint8_t>(SensorResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      const uint8_t foot = req[0];
      if (foot >= kSensorNumFeet) {
        out[0] = static_cast<uint8_t>(SensorResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      const uint16_t near_t = get16(&req[1]);
      const uint16_t touch_t = get16(&req[3]);
      const uint16_t load_t = get16(&req[5]);
      thresholds_.near_thresh[foot] = near_t;
      thresholds_.touch_thresh[foot] = touch_t;
      thresholds_.load_thresh[foot] = load_t;
      ++threshold_seq_;
      out[0] = static_cast<uint8_t>(SensorResult::Ok);
      out[1] = foot;
      uint16_t o = 2;
      o += put16(&out[o], near_t);
      o += put16(&out[o], touch_t);
      o += put16(&out[o], load_t);
      *out_len = o;
      return true;
    }

    case sensormsg::kLevelingSetParams: {
      // req: [max_tilt_mdeg u16, rate_mdeg_s u16, response_x255 u16].
      if (req_len < 6) {
        out[0] = static_cast<uint8_t>(SensorResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      leveling_.max_tilt_mdeg = get16(&req[0]);
      leveling_.rate_mdeg_s = get16(&req[2]);
      leveling_.response_x255 = get16(&req[4]);
      ++leveling_seq_;
      out[0] = static_cast<uint8_t>(SensorResult::Ok);
      uint16_t o = 1;
      o += put16(&out[o], leveling_.max_tilt_mdeg);
      o += put16(&out[o], leveling_.rate_mdeg_s);
      o += put16(&out[o], leveling_.response_x255);
      *out_len = o;
      return true;
    }

    default:
      // Unhandled ids in the 0x70-0x7F block (I2C/sensor query + calibrate) are
      // added by ubs.5.2; for now report unknown so the dispatcher errors.
      return false;
  }
}

}  // namespace protocol
