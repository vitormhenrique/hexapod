#include "crsf_telemetry.h"

#include <math.h>

#include "crsf_parser.h"

namespace crsf {
namespace telemetry {
namespace {

void putBe16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value >> 8);
  out[1] = static_cast<uint8_t>(value & 0xFF);
}

int16_t centidegreesToCrsfRadians(int16_t centidegrees) {
  const float radians_x10000 =
      static_cast<float>(centidegrees) * 1.745329252f;
  if (radians_x10000 > 32767.0f) return 32767;
  if (radians_x10000 < -32768.0f) return -32768;
  return static_cast<int16_t>(lroundf(radians_x10000));
}

}  // namespace

uint8_t encodeHexapodStatus(const HexapodStatus& status,
                            uint8_t out[kHexapodPayloadBytes]) {
  if (out == nullptr) return 0;
  out[0] = kHexapodMagic0;
  out[1] = kHexapodMagic1;
  out[2] = kHexapodVersion;
  out[3] = status.flags;
  out[4] = status.safety_state;
  out[5] = status.command_source;
  out[6] = status.gait;
  out[7] = status.control_mode;
  out[8] = status.fault_reason;
  out[9] = status.speed_x255;
  out[10] = status.duty_x255;
  out[11] = status.imu_calibration;
  putBe16(&out[12], status.battery_mv);
  putBe16(&out[14], status.body_height_mm);
  putBe16(&out[16], status.stride_mm);
  putBe16(&out[18], status.step_height_mm);
  return kHexapodPayloadBytes;
}

uint8_t encodeBattery(uint16_t battery_mv, uint8_t remaining_percent,
                      uint8_t out[kBatteryPayloadBytes]) {
  if (out == nullptr) return 0;
  const uint16_t voltage_x10 =
      static_cast<uint16_t>((static_cast<uint32_t>(battery_mv) + 50u) / 100u);
  putBe16(&out[0], voltage_x10);
  putBe16(&out[2], 0);  // current unavailable
  out[4] = 0;           // capacity unavailable (24-bit, big-endian)
  out[5] = 0;
  out[6] = 0;
  out[7] = remaining_percent > 100 ? 100 : remaining_percent;
  return kBatteryPayloadBytes;
}

uint8_t encodeAttitude(int16_t pitch_cdeg, int16_t roll_cdeg,
                       int16_t yaw_cdeg,
                       uint8_t out[kAttitudePayloadBytes]) {
  if (out == nullptr) return 0;
  putBe16(&out[0], static_cast<uint16_t>(
                       centidegreesToCrsfRadians(pitch_cdeg)));
  putBe16(&out[2], static_cast<uint16_t>(
                       centidegreesToCrsfRadians(roll_cdeg)));
  putBe16(&out[4], static_cast<uint16_t>(
                       centidegreesToCrsfRadians(yaw_cdeg)));
  return kAttitudePayloadBytes;
}

uint8_t buildFrame(uint8_t type, const uint8_t* payload, uint8_t payload_len,
                   uint8_t* out, size_t out_capacity) {
  const size_t frame_len = static_cast<size_t>(payload_len) + 4u;
  if (out == nullptr || (payload_len > 0 && payload == nullptr) ||
      frame_len > kMaxFrameLen || out_capacity < frame_len) {
    return 0;
  }
  out[0] = kSyncByte;
  out[1] = static_cast<uint8_t>(payload_len + 2u);
  out[2] = type;
  for (uint8_t i = 0; i < payload_len; ++i) out[3 + i] = payload[i];
  out[3 + payload_len] = crc8(&out[2], static_cast<uint8_t>(payload_len + 1u));
  return static_cast<uint8_t>(frame_len);
}

}  // namespace telemetry
}  // namespace crsf
