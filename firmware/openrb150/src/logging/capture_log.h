#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../dxl/dxl_status.h"
#include "../gait/gait_pipeline.h"

namespace logging {

struct RemoteCapture {
  uint32_t timestamp_ms = 0;
  uint32_t frame_sequence = 0;
  int16_t gimbal[4] = {0};
  int16_t pot[2] = {0};
  int32_t encoder[2] = {0};
  int16_t twist_milli[3] = {0};
  int16_t pose[6] = {0};
  uint8_t switch_mask = 0;
  uint8_t button_mask = 0;
  uint8_t toggle[2] = {0};
  uint8_t nav_mask[2] = {0};
  uint8_t flags = 0;
  uint8_t mode = 0;
  uint8_t gait = 0;
  uint8_t speed_x255 = 0;
  uint8_t body_height_x255 = 0;
  uint8_t stride_x255 = 0;
  uint8_t step_height_x255 = 0;
  uint8_t duty_x255 = 0;
  uint32_t capture_toggle_seq = 0;
  uint32_t capture_start_seq = 0;
};

constexpr uint8_t kAppliedMotionFlagMotionGate = 1u << 0;
constexpr uint8_t kAppliedMotionFlagGoalValid = 1u << 1;
constexpr uint8_t kAppliedMotionFlagGoalClamped = 1u << 2;
constexpr uint8_t kAppliedMotionFlagGoalUnreachable = 1u << 3;
constexpr uint8_t kAppliedMotionFlagGoalReachLimited = 1u << 4;

// Controller output paired with a final gait/IK goal frame. Shape values are
// the safety-clamped values actually passed to the gait pipeline.
struct AppliedMotionCapture {
  uint32_t goal_sequence = 0;
  uint16_t body_height_mm = 0;
  uint16_t stride_mm = 0;
  uint16_t step_height_mm = 0;
  uint8_t command_source = 0;
  uint8_t safety_state = 0;
  uint8_t gait = 0;
  uint8_t duty_x255 = 0;
  uint8_t speed_x255 = 0;
  uint8_t flags = 0;
};

// Final per-joint actuator request paired with its logical calibrated angle.
struct GoalCapture {
  uint16_t goal_tick = 0;
  int16_t goal_angle_centideg = 0;
  uint8_t id = 0;
  uint8_t leg = 0;
  uint8_t joint = 0;
  uint8_t flags = 0;
};

constexpr uint8_t kGoalCaptureFlagClamped = 1u << 0;

size_t formatCaptureMarker(bool begin, uint32_t session, uint32_t timestamp_ms,
                           uint32_t samples, char* out, size_t out_cap);
size_t formatRemoteCaptureRow(uint32_t session, uint32_t sample,
                              const RemoteCapture& remote, char* out,
                              size_t out_cap);
size_t formatAppliedMotionCaptureRow(uint32_t session, uint32_t sample,
                                     uint32_t timestamp_ms,
                                     const AppliedMotionCapture& motion,
                                     char* out, size_t out_cap);
size_t formatLegCaptureRow(uint32_t session, uint32_t sample,
                           uint32_t timestamp_ms, uint32_t goal_sequence,
                           const gait::PipelineLegTarget* legs, uint8_t count,
                           char* out, size_t out_cap);
size_t formatGoalCaptureRow(uint32_t session, uint32_t sample,
                            uint32_t timestamp_ms, uint32_t goal_sequence,
                            const GoalCapture* goals, uint8_t count, char* out,
                            size_t out_cap);
size_t formatServoCaptureRow(uint32_t session, uint32_t sample,
                             uint32_t timestamp_ms,
                             const dxl::ServoStatus& servo,
                             int16_t present_angle_centideg, char* out,
                             size_t out_cap);

}  // namespace logging