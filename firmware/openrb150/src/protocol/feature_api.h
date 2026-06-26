#pragma once

// ===========================================================================
// Feature flag command group (USB API, AGENTS.md 1.3 + 6.2 "Feature flags").
//
// Exposes the runtime-toggleable capability flags so a host can read and change
// which optional behaviours are active:
//
//   FEATURE_GET            (0x39): full state of every feature
//   FEATURE_SET            (0x3A): request enable/disable of one feature
//   FEATURE_GET_REASONS    (0x3B): per-feature reason it is unavailable/off
//   FEATURE_RESET_DEFAULTS (0x3C): restore the compiled default enable set
//
// This handler is purely the command surface + host-intent store. The control
// task remains the authority: each cycle it publishes whether a feature is
// *available* (hardware present, calibrated, safe state) and a reason code via
// setAvailability(); the host's desired-enable is only *honoured* (effective
// enabled = desired && available). FEATURE_SET can never force a feature on
// when the firmware says it is unavailable -- it is Rejected with the reason
// echoed back (AGENTS.md 1.3 "firmware may reject or auto-disable a feature").
//
// Portable (no Arduino deps) so it runs in the native unit tests. All payloads
// little-endian. No servo is ever commanded here.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace protocol {

// Feature command msg-ids (reserved within the 0x30-0x3F control block; safety
// control is 0x30-0x33, motion is 0x34-0x38).
namespace featuremsg {
constexpr uint8_t kGet = 0x39;
constexpr uint8_t kSet = 0x3A;
constexpr uint8_t kGetReasons = 0x3B;
constexpr uint8_t kResetDefaults = 0x3C;
constexpr uint8_t kFirst = kGet;
constexpr uint8_t kLast = kResetDefaults;
inline bool isFeatureMsg(uint8_t id) { return id >= kFirst && id <= kLast; }
}  // namespace featuremsg

// Runtime feature ids (wire-stable: APPEND new features, never renumber).
enum class Feature : uint8_t {
  FootContact = 0,      // touchdown/contact estimation feeds the gait engine
  TerrainLeveling = 1,  // slow body leveling from the stance support plane
  SensorPolling = 2,    // raw foot-sensor proximity/pressure polling
  JetsonControl = 3,    // accept high-level Jetson autonomy commands
  PassivePose = 4,      // torque-off present-position streaming
  Count = 5,
};

constexpr uint8_t kFeatureCount = static_cast<uint8_t>(Feature::Count);

// Why a feature is not currently enabled / cannot be enabled. `None` means the
// feature is available with no blocker.
enum class FeatureReason : uint8_t {
  None = 0,
  HardwareMissing = 1,  // required device absent (mux / sensors)
  NotCalibrated = 2,    // sensors present but not calibrated
  UnsafeState = 3,      // robot state disallows it right now
  StaleData = 4,        // sensor data too stale to trust
  DependencyOff = 5,    // depends on another feature that is off
  NotImplemented = 6,   // engine/path not yet wired in this build
};

// Result byte returned in FEATURE_SET / FEATURE_RESET_DEFAULTS responses.
enum class FeatureResult : uint8_t {
  Ok = 0,          // request accepted (intent recorded)
  Rejected = 1,    // understood but not permitted (unavailable)
  BadRequest = 2,  // malformed payload / unknown feature id
};

// Compiled default enable set. SensorPolling defaults on so present boards
// stream raw data; all richer/safety features default off until requested.
constexpr bool kFeatureDefaultEnabled[kFeatureCount] = {
    false,  // FootContact
    false,  // TerrainLeveling
    true,   // SensorPolling
    false,  // JetsonControl
    false,  // PassivePose
};

class FeatureApi {
 public:
  FeatureApi() { reset(); }

  // Restore the compiled default enable set and clear reflected availability.
  void reset();

  // --- Reflected state, published by the control task each cycle ------------
  // Mark whether a feature is currently available and why (when not). Effective
  // enabled = desired && available, so flipping availability off auto-disables
  // the feature's effect without losing the host's desired setting.
  void setAvailability(Feature f, bool available, FeatureReason reason);

  // Publish the live safety-state byte echoed in responses for context.
  void setLiveState(uint8_t state) { live_state_ = state; }

  // --- Host intent consumed by the control task ----------------------------
  // The host's requested enable for `f` (default-seeded). The task ANDs this
  // with availability to decide the real effect.
  bool desiredEnabled(Feature f) const;
  // Effective enabled = desired && available (what the wire reports).
  bool effectiveEnabled(Feature f) const;
  // Increments whenever the desired set changes (SET / RESET_DEFAULTS) so the
  // task can detect host changes without diffing each flag.
  uint32_t seq() const { return seq_; }

  // --- Command handling ----------------------------------------------------
  // Dispatch a feature command. Returns false if `msg_id` is not in the feature
  // range (so the api dispatcher can try the next group). On a handled command
  // writes the response payload and returns true.
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, size_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

 private:
  struct State {
    bool desired;
    bool available;
    FeatureReason reason;
  };

  // Append the 4-byte per-feature record [id, available, enabled, reason].
  uint16_t writeFullState(uint8_t* out, uint16_t off) const;

  State state_[kFeatureCount];
  uint8_t live_state_ = 0;
  uint32_t seq_ = 0;
};

}  // namespace protocol
