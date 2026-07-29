#pragma once

// ===========================================================================
// Logical RC input conditioning (portable, heap-free).
//
// The bridge normalizes physical controller values before handing them here.
// This module owns the configured analog filter state and response helpers;
// it deliberately has no knowledge of CRSF, ChannelPack, FreeRTOS, or servo
// targets. One Conditioner state exists per logical analog source.
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"

namespace controller {

enum class InputFilterMode : uint8_t {
  Ema = 0,
  Median3Ema = 1,
  NoneDiagnostic = 2,
};

class RcInputConditioner {
 public:
  void configure(const config::RcInputCalibration& calibration);
  void reset();

  // `NoneDiagnostic` is only accepted while the caller has established that
  // the robot is disarmed. EMA and Median3+EMA preserve the current filtered
  // output when switching, avoiding a one-frame command jump.
  bool setMode(InputFilterMode mode, bool diagnostic_mode_allowed);
  InputFilterMode mode() const { return mode_; }

  // Input is already normalized: [-1, 1] for centred analog controls and
  // [0, 1] for unipolar/relative controls. `dt_ms` comes from the input task's
  // monotonic frame timestamp; first samples seed state without a startup lag.
  float update(uint8_t source, float input, uint32_t dt_ms);

  bool isCentered(uint8_t source) const;
  float deadband(uint8_t source) const;
  float expo(uint8_t source) const;

  static float applyDeadband(float value, float deadband);
  static float applyExpo(float value, float expo);

 private:
  struct ChannelState {
    float output = 0.0f;
    float median[3] = {};
    uint8_t median_count = 0;
    uint8_t median_next = 0;
    bool initialized = false;
  };

  const config::RcChannelCalibration* find(uint8_t source) const;
  static float median3(float a, float b, float c);
  static float clamp(float value, float low, float high);
  float medianInput(ChannelState& state, float input);

  config::RcInputCalibration calibration_{};
  ChannelState channels_[config::kNumRcAnalogInputs] = {};
  InputFilterMode mode_ = InputFilterMode::Ema;
};

// Stable-position debouncer for non-safety three-position controls. The
// bridge deliberately keeps physical E-stop evaluation immediate.
class RcTriSwitchDebouncer {
 public:
  void reset() { initialized_ = false; }
  uint8_t update(uint8_t raw, uint32_t now_ms, uint16_t debounce_ms);

 private:
  uint8_t stable_ = 1;
  uint8_t candidate_ = 1;
  uint32_t candidate_since_ms_ = 0;
  bool initialized_ = false;
};

}  // namespace controller