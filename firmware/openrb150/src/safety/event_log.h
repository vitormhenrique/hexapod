#pragma once

#include <stddef.h>
#include <stdint.h>

#include "error_journal.h"
#include "fault_capture.h"

namespace safety {

struct PersistentEvent {
  uint32_t timestamp_ms = 0;
  ErrorCode code = ErrorCode::None;
  uint8_t detail = 0;
  ErrorSeverity severity = ErrorSeverity::Info;
};

class PersistentEventQueue {
 public:
  static constexpr uint8_t kCapacity = 8;

  bool push(const PersistentEvent& event);
  bool peek(PersistentEvent& event) const;
  bool pop();
  uint8_t size() const { return count_; }
  uint16_t dropped() const { return dropped_; }

 private:
  PersistentEvent events_[kCapacity];
  uint8_t head_ = 0;
  uint8_t tail_ = 0;
  uint8_t count_ = 0;
  uint16_t dropped_ = 0;
};

size_t formatEventLog(const PersistentEvent& event, char* out, size_t out_cap);
size_t formatCrashLog(const fault_capture::Snapshot& crash, char* out,
                      size_t out_cap);
size_t formatWatchdogResetLog(uint32_t missed_mask, uint8_t dxl_progress,
                              uint8_t control_progress, uint8_t safety_state,
                              char* out, size_t out_cap);

}  // namespace safety