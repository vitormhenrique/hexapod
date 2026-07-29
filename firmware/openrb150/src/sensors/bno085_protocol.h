#pragma once

#include <stddef.h>
#include <stdint.h>

namespace sensors {
namespace bno085 {

constexpr uint8_t kAddressA = 0x4A;
constexpr uint8_t kAddressB = 0x4B;
constexpr uint8_t kChannelExecutable = 1;
constexpr uint8_t kChannelControl = 2;
constexpr uint8_t kChannelReports = 3;
constexpr uint8_t kResetComplete = 0x01;
constexpr uint8_t kReportRotationVector = 0x05;
constexpr uint8_t kReportProductIdResponse = 0xF8;
constexpr uint8_t kReportProductIdRequest = 0xF9;
constexpr uint8_t kReportBaseTimestamp = 0xFB;
constexpr uint8_t kReportSetFeature = 0xFD;
constexpr uint8_t kSetFeatureBytes = 17;

struct OrientationSample {
  int16_t pitch_cdeg = 0;
  int16_t roll_cdeg = 0;
  int16_t yaw_cdeg = 0;
  uint8_t quality = 0;
  bool ok = false;
};

uint8_t buildRotationVectorFeature(uint32_t interval_us,
                                   uint8_t out[kSetFeatureBytes]);
bool decodeRotationVector(const uint8_t* payload, uint16_t payload_len,
                          OrientationSample& sample);
uint8_t packQuality(uint8_t accuracy);

}  // namespace bno085
}  // namespace sensors