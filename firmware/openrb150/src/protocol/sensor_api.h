#pragma once

// ===========================================================================
// Sensor / contact / leveling command group (portable, no Arduino deps).
//
// USB command block 0x70-0x7F. This translation unit (ubs.5.1) covers the
// contact + leveling *control* surface:
//   CONTACT_ENABLE / CONTACT_DISABLE          -> FeatureApi(FootContact)
//   LEVELING_ENABLE / LEVELING_DISABLE        -> FeatureApi(TerrainLeveling)
//   CONTACT_SET_THRESHOLDS                     -> per-foot near/touch/load
//   LEVELING_SET_PARAMS                        -> staged leveling tunables
//
// Enable/disable route through the shared FeatureApi so the desired-feature set
// has a single source of truth (AGENTS.md 1.3): enabling a feature the firmware
// reports unavailable (sensors missing/stale) is Rejected with the reason
// echoed, never forced on. Thresholds/params are staged here and consumed by
// i2cTask (the sole owner of Wire + the contact estimator). The I2C scan /
// topology / sensor status / rate / calibrate commands (0x76-0x7B) are added by
// ubs.5.2 in this same handler block.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

#include "feature_api.h"

namespace protocol {

namespace sensormsg {
constexpr uint8_t kContactEnable = 0x70;
constexpr uint8_t kContactDisable = 0x71;
constexpr uint8_t kContactSetThresholds = 0x72;
constexpr uint8_t kLevelingEnable = 0x73;
constexpr uint8_t kLevelingDisable = 0x74;
constexpr uint8_t kLevelingSetParams = 0x75;

constexpr uint8_t kFirst = 0x70;
constexpr uint8_t kLast = 0x7F;
inline bool isSensorMsg(uint8_t id) { return id >= kFirst && id <= kLast; }
}  // namespace sensormsg

// Foot count mirrors the contact estimator / config (6 muxed foot sensors).
constexpr uint8_t kSensorNumFeet = 6;

// Result byte returned by sensor commands.
enum class SensorResult : uint8_t {
  Ok = 0,          // accepted
  Rejected = 1,    // understood but not permitted (e.g. feature unavailable)
  BadRequest = 2,  // malformed payload / bad index
};

// Per-foot near/touch/load thresholds staged by CONTACT_SET_THRESHOLDS and
// consumed by i2cTask via ContactEstimator::setThresholds().
struct ContactThresholds {
  uint16_t near_thresh[kSensorNumFeet];
  uint16_t touch_thresh[kSensorNumFeet];
  uint16_t load_thresh[kSensorNumFeet];
};

// Leveling tunables staged by LEVELING_SET_PARAMS. No leveling engine consumes
// these yet (terrain leveling is not implemented); they are stored and
// reflected so the host UI can round-trip the intent.
struct LevelingParams {
  uint16_t max_tilt_mdeg;   // max body roll/pitch correction (millidegrees)
  uint16_t rate_mdeg_s;     // correction slew rate (millidegrees / second)
  uint16_t response_x255;   // low-pass blend factor, 0..255
};

class SensorApi {
 public:
  SensorApi() { reset(); }

  // Restore staged thresholds/params to zero and clear sequences.
  void reset();

  // Wire the shared FeatureApi used for CONTACT/LEVELING enable/disable. Must
  // be set before those commands are handled (else they return Rejected).
  void setFeatureApi(FeatureApi* features) { features_ = features; }

  // --- Intent consumed by i2cTask (estimator owner) ------------------------
  // Bumps each time CONTACT_SET_THRESHOLDS records a change so the task can
  // apply without diffing every foot.
  uint32_t thresholdSeq() const { return threshold_seq_; }
  const ContactThresholds& thresholds() const { return thresholds_; }

  // Staged leveling params (no engine yet; reflected to the host).
  uint32_t levelingSeq() const { return leveling_seq_; }
  const LevelingParams& levelingParams() const { return leveling_; }

  // --- Command handling ----------------------------------------------------
  // Dispatch a sensor-group command. Returns false if `msg_id` is not in the
  // 0x70-0x7F block (so the dispatcher can try the next group). On a handled
  // command writes the response payload and returns true.
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, size_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

 private:
  // Apply an enable/disable to a feature via FeatureApi and write the standard
  // [result, state, available, enabled, reason] response.
  bool applyFeature(Feature f, bool enable, uint8_t* out, size_t out_cap,
                    uint16_t* out_len, uint8_t* out_flags);

  FeatureApi* features_ = nullptr;
  ContactThresholds thresholds_;
  uint32_t threshold_seq_ = 0;
  LevelingParams leveling_;
  uint32_t leveling_seq_ = 0;
};

}  // namespace protocol
