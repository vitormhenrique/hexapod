#include "bno085.h"

#include <Arduino.h>

namespace sensors {
namespace {

constexpr uint8_t kI2cChunkBytes = 32;
constexpr uint16_t kMaxShtpPacketBytes = 384;
constexpr uint32_t kRotationVectorIntervalUs = 50000;

}  // namespace

bool Bno085::sendPacket(uint8_t channel, const uint8_t* payload,
                        uint8_t payload_len) {
  if (address_ == 0 || channel >= sizeof(sequence_) ||
      (payload_len > 0 && payload == nullptr)) {
    return false;
  }
  uint8_t packet[4 + bno085::kSetFeatureBytes];
  if (payload_len > bno085::kSetFeatureBytes) return false;
  const uint16_t packet_len = static_cast<uint16_t>(payload_len + 4u);
  packet[0] = static_cast<uint8_t>(packet_len);
  packet[1] = static_cast<uint8_t>(packet_len >> 8);
  packet[2] = channel;
  packet[3] = sequence_[channel]++;
  for (uint8_t index = 0; index < payload_len; ++index) {
    packet[4 + index] = payload[index];
  }
  return bus_.write(address_, packet, static_cast<uint8_t>(packet_len));
}

bool Bno085::receivePacket(uint8_t& channel, uint8_t* payload,
                           uint16_t payload_capacity,
                           uint16_t& payload_len) {
  payload_len = 0;
  uint8_t header[4];
  if (address_ == 0 || !bus_.read(address_, header, sizeof(header))) {
    return false;
  }
  uint16_t packet_len = static_cast<uint16_t>(header[0]) |
                        static_cast<uint16_t>(header[1] << 8);
  packet_len &= 0x7FFFu;
  if (packet_len < 4 || packet_len > kMaxShtpPacketBytes) return false;
  channel = header[2];
  uint16_t remaining = static_cast<uint16_t>(packet_len - 4u);
  uint16_t stored = 0;
  while (remaining > 0) {
    const uint8_t cargo = static_cast<uint8_t>(
        remaining > kI2cChunkBytes - 4u ? kI2cChunkBytes - 4u : remaining);
    uint8_t chunk[kI2cChunkBytes];
    const uint8_t request = static_cast<uint8_t>(cargo + 4u);
    if (!bus_.read(address_, chunk, request)) return false;
    for (uint8_t index = 0; index < cargo; ++index) {
      if (stored < payload_capacity && payload != nullptr) {
        payload[stored] = chunk[4 + index];
      }
      ++stored;
    }
    remaining = static_cast<uint16_t>(remaining - cargo);
  }
  payload_len = stored > payload_capacity ? payload_capacity : stored;
  return true;
}

bool Bno085::enableRotationVector() {
  uint8_t feature[bno085::kSetFeatureBytes];
  const uint8_t length = bno085::buildRotationVectorFeature(
      kRotationVectorIntervalUs, feature);
  return sendPacket(bno085::kChannelControl, feature, length);
}

bool Bno085::begin() {
  present_ = false;
  address_ = 0;
  const uint8_t candidates[] = {bno085::kAddressA, bno085::kAddressB};
  for (uint8_t candidate : candidates) {
    if (bus_.probe(candidate)) {
      address_ = candidate;
      break;
    }
  }
  if (address_ == 0) return false;
  for (uint8_t& sequence : sequence_) sequence = 0;

  const uint8_t reset = 1;
  if (!sendPacket(bno085::kChannelExecutable, &reset, 1)) return false;
  delay(300);

  uint8_t channel = 0;
  uint8_t payload[32];
  uint16_t payload_len = 0;
  for (uint8_t packet = 0; packet < 8; ++packet) {
    if (!receivePacket(channel, payload, sizeof(payload), payload_len)) break;
  }

  const uint8_t product_request[2] = {
      bno085::kReportProductIdRequest, 0};
  if (!sendPacket(bno085::kChannelControl, product_request,
                  sizeof(product_request))) {
    return false;
  }
  bool identified = false;
  for (uint8_t attempt = 0; attempt < 20; ++attempt) {
    if (receivePacket(channel, payload, sizeof(payload), payload_len) &&
        channel == bno085::kChannelControl && payload_len > 0 &&
        payload[0] == bno085::kReportProductIdResponse) {
      identified = true;
      break;
    }
    delay(10);
  }
  if (!identified || !enableRotationVector()) return false;
  present_ = true;
  return true;
}

ImuSample Bno085::read() {
  ImuSample sample;
  if (!present_) return sample;

  for (uint8_t packet = 0; packet < 3; ++packet) {
    uint8_t channel = 0;
    uint8_t payload[32];
    uint16_t payload_len = 0;
    if (!receivePacket(channel, payload, sizeof(payload), payload_len)) {
      return sample;
    }
    if (channel == bno085::kChannelExecutable && payload_len > 0 &&
        payload[0] == bno085::kResetComplete) {
      (void)enableRotationVector();
      continue;
    }
    bno085::OrientationSample orientation;
    if (channel == bno085::kChannelReports &&
        bno085::decodeRotationVector(payload, payload_len, orientation)) {
      sample.pitch_cdeg = orientation.pitch_cdeg;
      sample.roll_cdeg = orientation.roll_cdeg;
      sample.yaw_cdeg = orientation.yaw_cdeg;
      sample.calibration = bno085::packQuality(orientation.quality);
      sample.ok = true;
      return sample;
    }
  }
  return sample;
}

}  // namespace sensors