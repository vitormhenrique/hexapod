#pragma once

#include <stdint.h>

namespace dxl {

// Let the switched rail rise and all servo MCUs boot before sending packets.
// This is an elapsed-time guard, not a blocking delay or proof of rail voltage.
constexpr uint32_t kPowerOnSettleMs = 1000;
constexpr uint32_t kDiscoveryRetryDelayMs = 50;
constexpr uint8_t kDiscoveryAttempts = 3;

class PowerOnSettling {
 public:
  void setPowered(bool powered, uint32_t now_ms) {
    if (powered && !powered_) enabled_ms_ = now_ms;
    powered_ = powered;
  }

  bool ready(uint32_t now_ms) const {
    return powered_ && (now_ms - enabled_ms_) >= kPowerOnSettleMs;
  }

 private:
  uint32_t enabled_ms_ = 0;
  bool powered_ = false;
};

}  // namespace dxl