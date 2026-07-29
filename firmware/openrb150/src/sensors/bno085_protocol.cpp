#include "bno085_protocol.h"

#include <math.h>

namespace sensors {
namespace bno085 {
namespace {

constexpr float kQ14Scale = 1.0f / 16384.0f;
constexpr float kRadToCentiDeg = 18000.0f / 3.14159265358979323846f;

int16_t readLe16(const uint8_t* data) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(data[0]) |
      static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8));
}

int16_t radiansToCentiDegrees(float radians) {
  float value = radians * kRadToCentiDeg;
  if (value > 18000.0f) value = 18000.0f;
  if (value < -18000.0f) value = -18000.0f;
  return static_cast<int16_t>(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

}  // namespace

uint8_t buildRotationVectorFeature(uint32_t interval_us,
                                   uint8_t out[kSetFeatureBytes]) {
  if (out == nullptr) return 0;
  for (uint8_t index = 0; index < kSetFeatureBytes; ++index) out[index] = 0;
  out[0] = kReportSetFeature;
  out[1] = kReportRotationVector;
  out[5] = static_cast<uint8_t>(interval_us);
  out[6] = static_cast<uint8_t>(interval_us >> 8);
  out[7] = static_cast<uint8_t>(interval_us >> 16);
  out[8] = static_cast<uint8_t>(interval_us >> 24);
  return kSetFeatureBytes;
}

uint8_t packQuality(uint8_t accuracy) {
  accuracy &= 0x03;
  return static_cast<uint8_t>((accuracy << 6) | (accuracy << 4) |
                              (accuracy << 2) | accuracy);
}

bool decodeRotationVector(const uint8_t* payload, uint16_t payload_len,
                          OrientationSample& sample) {
  sample = OrientationSample{};
  if (payload == nullptr || payload_len < 19 ||
      payload[0] != kReportBaseTimestamp ||
      payload[5] != kReportRotationVector) {
    return false;
  }

  float x = static_cast<float>(readLe16(&payload[9])) * kQ14Scale;
  float y = static_cast<float>(readLe16(&payload[11])) * kQ14Scale;
  float z = static_cast<float>(readLe16(&payload[13])) * kQ14Scale;
  float w = static_cast<float>(readLe16(&payload[15])) * kQ14Scale;
  const float norm = sqrtf(x * x + y * y + z * z + w * w);
  if (!(norm > 0.1f) || !isfinite(norm)) return false;
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  const float roll = atan2f(2.0f * (w * x + y * z),
                            1.0f - 2.0f * (x * x + y * y));
  float pitch_term = 2.0f * (w * y - z * x);
  if (pitch_term > 1.0f) pitch_term = 1.0f;
  if (pitch_term < -1.0f) pitch_term = -1.0f;
  const float pitch = asinf(pitch_term);
  const float yaw = atan2f(2.0f * (w * z + x * y),
                           1.0f - 2.0f * (y * y + z * z));

  sample.roll_cdeg = radiansToCentiDegrees(roll);
  sample.pitch_cdeg = radiansToCentiDegrees(pitch);
  sample.yaw_cdeg = radiansToCentiDegrees(yaw);
  sample.quality = static_cast<uint8_t>(payload[7] & 0x03);
  sample.ok = true;
  return true;
}

}  // namespace bno085
}  // namespace sensors