#pragma once

// ===========================================================================
// High-level body command shaper (portable, heap-free).
//
// This is the single production owner of body-twist acceleration limits. The
// gait engine receives already-shaped twist commands and no longer applies a
// second spring-damper response. Pose and height changes are rate-limited in
// their physical units so gimbal/host inputs cannot jump a planted-foot body
// transform in one control step.
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"
#include "../gait/body_ik.h"

namespace controller {

enum class CommandShaperMode : uint8_t {
  AccelLimited = 0,
  DirectDiagnostic = 1,
};

// Normal 100 Hz jitter uses measured dt. An overrun beyond this window is
// still reported by ControllerTime but cannot turn into one giant body-command
// integration step before the control loop recovers.
constexpr uint32_t kMaxBodyCommandIntegrationMs = 50;

struct BodyCommand {
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
  float body_height_mm = 132.0f;
  gait::BodyPose pose{};
};

class BodyCommandShaper {
 public:
  void configure(const config::BodyCommandLimits& limits);
  void reset(float body_height_mm);

  // Direct diagnostic output is intentionally gated by the caller. Normal
  // firmware keeps AccelLimited selected; future runtime mode selection must
  // pass false while walking.
  bool setMode(CommandShaperMode mode, bool diagnostic_mode_allowed);
  CommandShaperMode mode() const { return mode_; }

  const BodyCommand& update(const BodyCommand& desired, uint32_t dt_ms);
  const BodyCommand& current() const { return current_; }

 private:
  static float clamp(float value, float low, float high);
  static float magnitude(float value) {
    return value < 0.0f ? -value : value;
  }
  static float moveToward(float current, float target, float max_delta);
  float shapeTwist(float current, float target, float positive_max,
                   float negative_max, float accel, float decel,
                   float dt_s) const;
  BodyCommand clampDesired(const BodyCommand& desired) const;

  config::BodyCommandLimits limits_{};
  BodyCommand current_{};
  CommandShaperMode mode_ = CommandShaperMode::AccelLimited;
};

}  // namespace controller