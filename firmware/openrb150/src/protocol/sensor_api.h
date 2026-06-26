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
// i2cTask (the sole owner of Wire + the contact estimator). ubs.5.2 adds the
// I2C scan / topology / sensor status / rate / calibrate commands (0x76-0x7B):
//   I2C_SCAN           -> bumps a sequence i2cTask re-runs scanAll on
//   I2C_GET_TOPOLOGY   -> encodes the published topology snapshot
//   SENSOR_GET_STATUS  -> encodes the published foot-status snapshot
//   SENSOR_SET_RATE    -> stages a target poll rate (intent, reflected)
//   CONTACT_CALIBRATE  -> stages a per-foot/all baseline-capture request
//   SENSOR_CALIBRATE   -> stages an all-feet baseline-capture request
// The topology/status reads use snapshots that i2cTask keeps current; the scan
// and calibrate requests are fire-and-forget (i2cTask owns Wire + estimator).
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
// ubs.5.2 I2C + sensor query / calibration.
constexpr uint8_t kI2cScan = 0x76;
constexpr uint8_t kI2cGetTopology = 0x77;
constexpr uint8_t kSensorGetStatus = 0x78;
constexpr uint8_t kSensorSetRate = 0x79;
constexpr uint8_t kContactCalibrate = 0x7A;
constexpr uint8_t kSensorCalibrate = 0x7B;

constexpr uint8_t kFirst = 0x70;
constexpr uint8_t kLast = 0x7F;
inline bool isSensorMsg(uint8_t id) { return id >= kFirst && id <= kLast; }
}  // namespace sensormsg

// Foot count mirrors the contact estimator / config (6 muxed foot sensors).
constexpr uint8_t kSensorNumFeet = 6;

// TCA9548A has 8 channels (mirrors i2c::kNumMuxChannels).
constexpr uint8_t kSensorNumChannels = 8;

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

// Portable mirror of the discovered I2C topology for I2C_GET_TOPOLOGY. Filled
// by the Wire owner (i2cTask) after each scan; read by SensorApi in apiTask.
struct TopologySnapshot {
  uint8_t valid;          // 0 until i2cTask has published a scan
  uint8_t mux_present;
  uint8_t eeprom_present;
  uint8_t num_channels;   // up to kSensorNumChannels
  struct Channel {
    uint8_t scanned;
    uint8_t vcnl_present;
    uint8_t lps_present;
    uint8_t device_count;
    uint8_t state;        // i2c::FootSensorState (Missing/Present/Fault)
  } channels[kSensorNumChannels];
};

// Portable mirror of the fused foot-contact snapshot for SENSOR_GET_STATUS.
struct StatusSnapshot {
  uint8_t valid;          // 0 until i2cTask has published a snapshot
  uint8_t num_feet;
  uint8_t present_mask;   // bit N = foot N present
  uint8_t polling_enabled;
  struct Foot {
    uint8_t state;        // sensors::ContactState
    uint8_t confidence;   // 0..255
    uint16_t proximity;   // raw VCNL4040
    int16_t pressure_delta;
    uint8_t flags;        // bit0 near,1 touch,2 loaded,3 release,4 stale,5 fault
  } feet[kSensorNumFeet];
};

class SensorApi {
 public:
  SensorApi() { reset(); }

  // Restore staged thresholds/params to zero and clear sequences.
  void reset();

  // Wire the shared FeatureApi used for CONTACT/LEVELING enable/disable. Must
  // be set before those commands are handled (else they return Rejected).
  void setFeatureApi(FeatureApi* features) { features_ = features; }

  // Wire the published topology / foot-status snapshots that i2cTask keeps
  // current. I2C_GET_TOPOLOGY / SENSOR_GET_STATUS read these (apiTask is a
  // reader only; i2cTask owns Wire). Either may be null in tests.
  void setSnapshots(const TopologySnapshot* topo, const StatusSnapshot* status) {
    topo_ = topo;
    status_ = status;
  }

  // --- Intent consumed by i2cTask (estimator owner) ------------------------
  // Bumps each time CONTACT_SET_THRESHOLDS records a change so the task can
  // apply without diffing every foot.
  uint32_t thresholdSeq() const { return threshold_seq_; }
  const ContactThresholds& thresholds() const { return thresholds_; }

  // Staged leveling params (no engine yet; reflected to the host).
  uint32_t levelingSeq() const { return leveling_seq_; }
  const LevelingParams& levelingParams() const { return leveling_; }

  // I2C_SCAN intent: bumps on each request so i2cTask re-runs scanAll.
  uint32_t scanSeq() const { return scan_seq_; }

  // CONTACT_CALIBRATE / SENSOR_CALIBRATE intent: bumps on each request. mask
  // holds the feet (bit N) to re-baseline for the latest request.
  uint32_t calibrateSeq() const { return calibrate_seq_; }
  uint8_t calibrateMask() const { return calibrate_mask_; }

  // SENSOR_SET_RATE intent: staged target poll rate (Hz). 0 = unset.
  uint32_t rateSeq() const { return rate_seq_; }
  uint16_t sensorRateHz() const { return rate_hz_; }

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

  // Encode the published topology snapshot for I2C_GET_TOPOLOGY.
  bool encodeTopology(uint8_t* out, size_t out_cap, uint16_t* out_len);

  // Encode the published foot-status snapshot for SENSOR_GET_STATUS.
  bool encodeStatus(uint8_t* out, size_t out_cap, uint16_t* out_len);

  // Stage a calibrate (baseline-capture) request for the feet in `mask`. When a
  // status snapshot is wired and none of those feet are present, the request is
  // Rejected; otherwise the mask + sequence are recorded for i2cTask to apply.
  bool stageCalibrate(uint8_t mask, uint8_t* out, uint16_t* out_len,
                      uint8_t* out_flags);

  FeatureApi* features_ = nullptr;
  const TopologySnapshot* topo_ = nullptr;
  const StatusSnapshot* status_ = nullptr;
  ContactThresholds thresholds_;
  uint32_t threshold_seq_ = 0;
  LevelingParams leveling_;
  uint32_t leveling_seq_ = 0;
  uint32_t scan_seq_ = 0;
  uint8_t calibrate_mask_ = 0;
  uint32_t calibrate_seq_ = 0;
  uint16_t rate_hz_ = 0;
  uint32_t rate_seq_ = 0;
};

}  // namespace protocol
