#pragma once

#include <stddef.h>
#include <stdint.h>

namespace crsf {
namespace telemetry {

constexpr uint8_t kFrameTypeBattery = 0x08;
constexpr uint8_t kFrameTypeAttitude = 0x1E;
constexpr uint8_t kFrameTypeHexapodStatus = 0x80;
constexpr uint8_t kHexapodMagic0 = 0x48;  // H
constexpr uint8_t kHexapodMagic1 = 0x58;  // X
constexpr uint8_t kHexapodVersion = 2;
constexpr uint8_t kHexapodPayloadBytes = 28;
constexpr uint8_t kBatteryPayloadBytes = 8;
constexpr uint8_t kAttitudePayloadBytes = 6;

namespace flag {
constexpr uint8_t kArmed = 1u << 0;
constexpr uint8_t kMotionGate = 1u << 1;
constexpr uint8_t kKill = 1u << 2;
constexpr uint8_t kFailsafe = 1u << 3;
constexpr uint8_t kImuPresent = 1u << 4;
constexpr uint8_t kImuFresh = 1u << 5;
constexpr uint8_t kFault = 1u << 6;
constexpr uint8_t kBatteryValid = 1u << 7;
}  // namespace flag

// Second flag byte (v2), carrying the gait-tune editor state and the severity
// of the reported error. Bits 4-5 hold the selected GaitTuneParam.
namespace tuneflag {
constexpr uint8_t kTuneActive = 1u << 0;
constexpr uint8_t kPreviewActive = 1u << 1;
constexpr uint8_t kSavePending = 1u << 2;
constexpr uint8_t kConfigVolatile = 1u << 3;
constexpr uint8_t kParamShift = 4;
constexpr uint8_t kParamMask = 0x03u << kParamShift;
constexpr uint8_t kSeverityShift = 6;
constexpr uint8_t kSeverityMask = 0x03u << kSeverityShift;
}  // namespace tuneflag

struct HexapodStatus {
  uint8_t flags = 0;
  uint8_t safety_state = 0;
  uint8_t command_source = 0;
  uint8_t gait = 0;
  uint8_t control_mode = 0;
  uint8_t fault_reason = 0;
  uint8_t speed_x255 = 0;
  uint8_t duty_x255 = 0;
  uint8_t imu_calibration = 0;
  uint16_t battery_mv = 0;
  uint16_t body_height_mm = 0;
  uint16_t stride_mm = 0;
  uint16_t step_height_mm = 0;
  // --- v2 ---
  uint8_t tune_flags = 0;      // see namespace tuneflag
  uint8_t error_code = 0;      // safety::ErrorCode of the latest incident
  uint8_t error_detail = 0;    // code-specific detail (servo id, foot, ...)
  uint8_t error_sequence = 0;  // bumped per announced incident; 0 = none
  uint16_t error_count = 0;    // occurrences within the current incident
  uint16_t error_suppressed = 0;  // duplicates the journal did not send
};

// Encode fixed-size payloads. Multi-byte CRSF sensor values are big-endian.
uint8_t encodeHexapodStatus(const HexapodStatus& status,
                            uint8_t out[kHexapodPayloadBytes]);
uint8_t encodeBattery(uint16_t battery_mv, uint8_t remaining_percent,
                      uint8_t out[kBatteryPayloadBytes]);
uint8_t encodeAttitude(int16_t pitch_cdeg, int16_t roll_cdeg,
                       int16_t yaw_cdeg,
                       uint8_t out[kAttitudePayloadBytes]);

// Build [address][length][type][payload][crc8]. Returns total bytes or zero.
uint8_t buildFrame(uint8_t type, const uint8_t* payload, uint8_t payload_len,
                   uint8_t* out, size_t out_capacity);

}  // namespace telemetry
}  // namespace crsf
