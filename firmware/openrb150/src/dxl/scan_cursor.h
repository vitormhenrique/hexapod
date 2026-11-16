#pragma once

#include <stdint.h>

#include "discovery_timing.h"

namespace dxl {

// Small portable state machine for maintenance or configured-slot discovery.
// The DXL task attempts at most one ID per turn, with scheduler-spaced retries
// of missing IDs. Call ready(now) before pinging and supply the completion
// timestamp to advanceAfterPing(found, now). Config-slot users must bound the
// range to their array size; this cursor has no knowledge of the servo map.
class ScanCursor {
 public:
  bool begin(uint8_t first_id, uint8_t last_id) {
    if (first_id == 0 || last_id < first_id) {
      reset();
      return false;
    }
    current_id_ = first_id;
    last_id_ = last_id;
    attempts_ = 0;
    waiting_retry_ = false;
    active_ = true;
    return true;
  }

  void reset() {
    active_ = false;
    current_id_ = 0;
    last_id_ = 0;
    attempts_ = 0;
    waiting_retry_ = false;
  }

  bool active() const { return active_; }
  uint8_t currentId() const { return current_id_; }

  bool ready(uint32_t now_ms) const {
    return active_ && (!waiting_retry_ ||
                       (now_ms - failed_ms_) >= kDiscoveryRetryDelayMs);
  }

  // Retry only failed IDs, on a later task turn after a quiet interval.
  // Returns true while work remains (including a retry of the same ID).
  bool advanceAfterPing(bool found = true, uint32_t now_ms = 0) {
    if (!active_) return false;
    if (!found && ++attempts_ < kDiscoveryAttempts) {
      failed_ms_ = now_ms;
      waiting_retry_ = true;
      return true;
    }
    attempts_ = 0;
    waiting_retry_ = false;
    if (current_id_ == last_id_) {
      active_ = false;
      return false;
    }
    ++current_id_;
    return true;
  }

 private:
  uint8_t current_id_ = 0;
  uint8_t last_id_ = 0;
  uint8_t attempts_ = 0;
  uint32_t failed_ms_ = 0;
  bool waiting_retry_ = false;
  bool active_ = false;
};

}  // namespace dxl