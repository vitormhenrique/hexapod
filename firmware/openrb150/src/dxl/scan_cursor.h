#pragma once

#include <stdint.h>

namespace dxl {

// Small portable state machine for a maintenance scan. The DXL task consumes
// exactly one ID per call so absent-servo timeouts cannot monopolize a task
// iteration or starve the cooperative watchdog.
class ScanCursor {
 public:
  bool begin(uint8_t first_id, uint8_t last_id) {
    if (first_id == 0 || last_id < first_id) {
      reset();
      return false;
    }
    current_id_ = first_id;
    last_id_ = last_id;
    active_ = true;
    return true;
  }

  void reset() {
    active_ = false;
    current_id_ = 0;
    last_id_ = 0;
  }

  bool active() const { return active_; }
  uint8_t currentId() const { return current_id_; }

  // Call after pinging currentId(). Returns true when another ID remains.
  bool advanceAfterPing() {
    if (!active_) return false;
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
  bool active_ = false;
};

}  // namespace dxl