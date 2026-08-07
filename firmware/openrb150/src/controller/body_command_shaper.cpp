#include "body_command_shaper.h"

namespace controller {

namespace {

constexpr float kMilliScale = 1.0f / 1000.0f;

}  // namespace

void BodyCommandShaper::configure(const config::BodyCommandLimits& limits) {
  if (config::validateBodyCommandLimits(limits)) limits_ = limits;
}

void BodyCommandShaper::reset(float body_height_mm) {
  current_ = BodyCommand{};
  current_.body_height_mm = body_height_mm;
  mode_ = CommandShaperMode::AccelLimited;
}

bool BodyCommandShaper::setMode(CommandShaperMode mode,
                                bool diagnostic_mode_allowed) {
  if (mode == CommandShaperMode::DirectDiagnostic &&
      !diagnostic_mode_allowed) {
    return false;
  }
  mode_ = mode;
  return true;
}

const BodyCommand& BodyCommandShaper::update(const BodyCommand& desired,
                                              uint32_t dt_ms) {
  const BodyCommand target = clampDesired(desired);
  if (mode_ == CommandShaperMode::DirectDiagnostic) {
    current_ = target;
    return current_;
  }

  const uint32_t bounded_dt_ms =
      dt_ms > kMaxBodyCommandIntegrationMs ? kMaxBodyCommandIntegrationMs
                                            : dt_ms;
  const float dt_s = static_cast<float>(bounded_dt_ms) / 1000.0f;
  if (dt_s <= 0.0f) return current_;
  current_.vx = shapeTwist(
      current_.vx, target.vx,
      static_cast<float>(limits_.max_forward_milli) * kMilliScale,
      static_cast<float>(limits_.max_reverse_milli) * kMilliScale,
      static_cast<float>(limits_.forward_accel_milli_per_s) * kMilliScale,
      static_cast<float>(limits_.forward_decel_milli_per_s) * kMilliScale,
      dt_s);
  current_.vy = shapeTwist(
      current_.vy, target.vy,
      static_cast<float>(limits_.max_lateral_milli) * kMilliScale,
      static_cast<float>(limits_.max_lateral_milli) * kMilliScale,
      static_cast<float>(limits_.lateral_accel_milli_per_s) * kMilliScale,
      static_cast<float>(limits_.lateral_decel_milli_per_s) * kMilliScale,
      dt_s);
  current_.wz = shapeTwist(
      current_.wz, target.wz,
      static_cast<float>(limits_.max_yaw_milli) * kMilliScale,
      static_cast<float>(limits_.max_yaw_milli) * kMilliScale,
      static_cast<float>(limits_.yaw_accel_milli_per_s) * kMilliScale,
      static_cast<float>(limits_.yaw_decel_milli_per_s) * kMilliScale,
      dt_s);

  const float height_rate = target.body_height_mm >= current_.body_height_mm
      ? static_cast<float>(limits_.height_rise_mm_per_s)
      : static_cast<float>(limits_.height_lower_mm_per_s);
  float bounded_height_rate = height_rate;
  if (target.height_rate_override_mm_per_s > 0.0f) {
    bounded_height_rate = target.height_rate_override_mm_per_s;
    if (bounded_height_rate > config::kBodyHeightRateMaxMmPerS) {
      bounded_height_rate = config::kBodyHeightRateMaxMmPerS;
    }
  }
  current_.body_height_mm = moveToward(current_.body_height_mm,
                                       target.body_height_mm,
                                       bounded_height_rate * dt_s);

  const float translation_delta =
      static_cast<float>(limits_.pose_translation_rate_mm_per_s) * dt_s;
  current_.pose.x_mm = moveToward(current_.pose.x_mm, target.pose.x_mm,
                                  translation_delta);
  current_.pose.y_mm = moveToward(current_.pose.y_mm, target.pose.y_mm,
                                  translation_delta);
  current_.pose.z_mm = moveToward(current_.pose.z_mm, target.pose.z_mm,
                                  translation_delta);
  const float rotation_delta =
      static_cast<float>(limits_.pose_rotation_rate_millirad_per_s) *
      kMilliScale * dt_s;
  current_.pose.roll = moveToward(current_.pose.roll, target.pose.roll,
                                  rotation_delta);
  current_.pose.pitch = moveToward(current_.pose.pitch, target.pose.pitch,
                                   rotation_delta);
  current_.pose.yaw = moveToward(current_.pose.yaw, target.pose.yaw,
                                 rotation_delta);
  return current_;
}

float BodyCommandShaper::clamp(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float BodyCommandShaper::moveToward(float current, float target,
                                     float max_delta) {
  if (max_delta <= 0.0f) return current;
  const float delta = target - current;
  if (delta > max_delta) return current + max_delta;
  if (delta < -max_delta) return current - max_delta;
  return target;
}

float BodyCommandShaper::shapeTwist(float current, float target,
                                    float positive_max, float negative_max,
                                    float accel, float decel,
                                    float dt_s) const {
  target = clamp(target, -negative_max, positive_max);
  const bool accelerating = current == 0.0f ||
      (current * target > 0.0f && magnitude(target) > magnitude(current));
  const float max_delta = (accelerating ? accel : decel) * dt_s;
  return moveToward(current, target, max_delta);
}

BodyCommand BodyCommandShaper::clampDesired(const BodyCommand& desired) const {
  BodyCommand clamped = desired;
  clamped.vx = clamp(
      desired.vx,
      -static_cast<float>(limits_.max_reverse_milli) * kMilliScale,
      static_cast<float>(limits_.max_forward_milli) * kMilliScale);
  clamped.vy = clamp(
      desired.vy,
      -static_cast<float>(limits_.max_lateral_milli) * kMilliScale,
      static_cast<float>(limits_.max_lateral_milli) * kMilliScale);
  clamped.wz = clamp(
      desired.wz,
      -static_cast<float>(limits_.max_yaw_milli) * kMilliScale,
      static_cast<float>(limits_.max_yaw_milli) * kMilliScale);
  return clamped;
}

}  // namespace controller