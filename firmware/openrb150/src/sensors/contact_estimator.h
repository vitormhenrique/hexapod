#pragma once

// ===========================================================================
// Foot contact estimator (portable, no Arduino deps).
//
// Turns the raw per-foot Robotic Finger Sensor v2 readings (VCNL4040 proximity
// + LPS25HB pressure) into a debounced contact state per leg (AGENTS.md 5.4).
// The Arduino-only reader (finger_sensor.{h,cpp}) performs the Wire/mux I/O and
// feeds samples in here; this translation unit holds only the state machine and
// thresholding so it can be unit-tested on the host (pio test -e native).
//
// State machine (AGENTS.md 5.4):
//   AIR   -> NEAR     proximity crosses near threshold
//   NEAR  -> TOUCH    pressure delta crosses touch threshold (debounced)
//   TOUCH -> LOADED   pressure delta stays above the load threshold
//   *     -> RELEASE  pressure delta + proximity drop back below release
//   any   -> STALE    no valid sample within the stale timeout
//   any   -> FAULT    repeated sensor read failures
//
// Pressure is treated as a signed delta from a per-foot baseline. The baseline
// comes from calibration when enabled; when a foot is unloaded (AIR/NEAR) the
// estimator also slowly tracks the baseline so slow sensor drift does not look
// like contact. Thresholds are applied with hysteresis (touch/load to assert,
// release to clear) to avoid chatter.
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"

namespace sensors {

constexpr uint8_t kNumFeet = config::kNumFootSensors;  // 6

// Debounced per-foot contact classification.
enum class ContactState : uint8_t {
  Air = 0,      // foot clear of the ground
  Near = 1,     // proximity says ground is close, no load yet
  Touch = 2,    // light contact detected
  Loaded = 3,   // bearing weight (stance)
  Release = 4,  // load just dropped away
  Stale = 5,    // no fresh sample within the timeout
  Fault = 6,    // sensor read errors / device fault
};

// One raw sample handed in by the Arduino reader. `ok` is false when the I2C
// read failed; the estimator counts failures toward FAULT and never blocks.
struct FootSample {
  uint16_t proximity_raw = 0;  // VCNL4040 PS_DATA (0 = far)
  int32_t pressure_raw = 0;    // LPS25HB pressure (raw counts)
  bool ok = false;             // read succeeded
};

// Debounced per-foot state published as telemetry (subset of AGENTS.md 5.4).
struct LegContactState {
  uint32_t timestamp_ms = 0;   // host/MCU time of the last valid sample
  uint16_t proximity_raw = 0;
  int32_t pressure_raw = 0;
  int32_t pressure_baseline = 0;
  int32_t pressure_delta = 0;  // pressure_raw - baseline
  ContactState state = ContactState::Air;
  uint8_t confidence = 0;      // 0..255
  bool near_surface = false;
  bool touch = false;
  bool loaded = false;
  bool release = false;
  bool stale = false;
  bool fault = false;
};

// Tunables shared by all feet. Defaults are conservative; the per-foot
// thresholds come from config::FootSensorCal.
struct ContactParams {
  uint8_t touch_debounce = 2;   // consecutive samples to assert TOUCH
  uint8_t load_debounce = 2;    // consecutive samples to assert LOADED
  uint8_t release_debounce = 2; // consecutive samples to clear back to AIR
  uint8_t fault_limit = 5;      // consecutive failed reads -> FAULT
  uint32_t stale_timeout_ms = 200;  // no valid sample within this -> STALE
  // Release uses a fraction of the touch/load thresholds for hysteresis. The
  // release threshold is (touch_thresh * release_num / release_den).
  uint16_t release_num = 1;
  uint16_t release_den = 2;
  // Baseline tracking step (raw counts) applied per update while a foot is
  // unloaded, nudging the baseline toward the live reading. 0 disables drift.
  int32_t baseline_track = 1;
};

class ContactEstimator {
 public:
  ContactEstimator() { reset(); }

  // Load per-foot calibration (baseline + thresholds + enable) and shared
  // params. Safe to call again at runtime when config changes.
  void configure(const config::FootSensorCal (&cal)[kNumFeet],
                 const ContactParams& params);

  // Update just the near/touch/load thresholds for one foot at runtime (host
  // CONTACT_SET_THRESHOLDS) without disturbing baselines, counters, or the live
  // state. The release threshold is recomputed from the new touch threshold.
  void setThresholds(uint8_t leg, uint16_t near_thresh, uint16_t touch_thresh,
                     uint16_t load_thresh);

  // Capture the current pressure reading as the new per-foot baseline (host
  // CONTACT_CALIBRATE / SENSOR_CALIBRATE). The foot must be at rest/unloaded
  // when called; the live delta drops to ~0. Counters/state are left intact so
  // the next update re-classifies against the new baseline. Owned by i2cTask.
  void captureBaseline(uint8_t leg);

  // Enable/disable per-foot contact classification at runtime. CONTACT_CALIBRATE
  // enables a freshly-baselined foot so it starts classifying instead of staying
  // AIR (a disabled foot only mirrors raw values). Enabling is refused unless the
  // foot carries a usable pressure calibration (touch/load > 0 and load >= touch),
  // mirroring validateRobotConfig, so an uncalibrated foot can never classify
  // noise. Returns true when the foot is left in the requested state.
  bool setEnabled(uint8_t leg, bool enabled);

  // Reset all feet to AIR with zero counters (baselines kept from configure).
  void reset();

  // Feed a fresh sample for one foot taken at `now_ms`. A foot whose
  // calibration is disabled stays AIR and only stores the raw values. Bounded,
  // non-blocking, no allocation.
  void update(uint8_t leg, const FootSample& sample, uint32_t now_ms);

  // Re-evaluate staleness for every foot at `now_ms` without a new sample. Call
  // each control/telemetry tick so a silent sensor decays to STALE.
  void tickStaleness(uint32_t now_ms);

  const LegContactState& foot(uint8_t leg) const { return feet_[leg]; }

  // Bit N set when foot N is LOADED (bearing weight). Useful for the support
  // plane / safety checks.
  uint8_t loadedMask() const;

 private:
  struct FootCtx {
    int32_t near_thresh = 0;
    int32_t touch_thresh = 0;
    int32_t load_thresh = 0;
    int32_t release_thresh = 0;
    bool enabled = false;
    uint8_t touch_count = 0;
    uint8_t load_count = 0;
    uint8_t release_count = 0;
    uint8_t fault_count = 0;
    uint32_t last_ok_ms = 0;
    bool seen = false;  // at least one valid sample arrived
  };

  static uint8_t confidenceFor(const FootCtx& ctx, const LegContactState& st,
                               int32_t delta);

  LegContactState feet_[kNumFeet];
  FootCtx ctx_[kNumFeet];
  ContactParams params_;
};

}  // namespace sensors
