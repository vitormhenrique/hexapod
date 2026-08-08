#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../dxl/dxl_status.h"

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
};

size_t formatCaptureMarker(bool begin, uint32_t session, uint32_t timestamp_ms,
                           uint32_t samples, char* out, size_t out_cap);
size_t formatRemoteCaptureRow(uint32_t session, uint32_t sample,
                              const RemoteCapture& remote, char* out,
                              size_t out_cap);
size_t formatServoCaptureRow(uint32_t session, uint32_t sample,
                             uint32_t timestamp_ms,
                             const dxl::ServoStatus& servo, char* out,
                             size_t out_cap);

}  // namespace logging