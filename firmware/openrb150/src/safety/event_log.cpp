#include "event_log.h"

namespace safety {
namespace {

class LineWriter {
 public:
  LineWriter(char* out, size_t capacity) : out_(out), capacity_(capacity) {}

  void text(const char* value) {
    while (*value != '\0') character(*value++);
  }

  void unsignedValue(uint32_t value) {
    char digits[10];
    uint8_t count = 0;
    do {
      digits[count++] = static_cast<char>('0' + value % 10u);
      value /= 10u;
    } while (value > 0 && count < sizeof(digits));
    while (count > 0) character(digits[--count]);
  }

  void hex32(uint32_t value) {
    constexpr char kHex[] = "0123456789ABCDEF";
    text("0x");
    for (int8_t shift = 28; shift >= 0; shift -= 4) {
      character(kHex[(value >> shift) & 0x0Fu]);
    }
  }

  size_t finish() {
    character('\n');
    if (capacity_ > 0) out_[length_ < capacity_ ? length_ : capacity_ - 1] = '\0';
    return overflow_ ? 0 : length_;
  }

 private:
  void character(char value) {
    if (length_ + 1 < capacity_) {
      out_[length_] = value;
    } else {
      overflow_ = true;
    }
    ++length_;
  }

  char* out_;
  size_t capacity_;
  size_t length_ = 0;
  bool overflow_ = false;
};

const char* severityName(ErrorSeverity severity) {
  constexpr const char* names[] = {"INFO", "WARN", "ERROR", "CRITICAL"};
  const uint8_t index = static_cast<uint8_t>(severity);
  return index < 4 ? names[index] : "UNKNOWN";
}

const char* errorName(ErrorCode code) {
  constexpr const char* names[] = {
      "NONE",          "SAFETY_FAULT",      "DXL_WRITE_FAILED",
      "DXL_READ_FAILED", "DXL_HARDWARE_ERROR", "DXL_BUS_SILENT",
      "DXL_DISCOVERY", "I2C_FOOT_SENSOR",   "I2C_MUX_MISSING",
      "CONFIG_STORAGE_MISSING", "CONFIG_VOLATILE", "CONFIG_COMMIT_FAILED",
      "RC_FAILSAFE",   "RC_FRAME_ERRORS",   "GOAL_UNREACHABLE",
      "GOAL_CLAMPED",  "BATTERY_LOW",       "WATCHDOG_STALL",
      "GAIT_SAVE_REJECTED", "CAPTURE_UNAVAILABLE", "CAPTURE_WRITE_FAILED",
      "ARMING_STARTED"};
  const uint8_t index = static_cast<uint8_t>(code);
  return index < sizeof(names) / sizeof(names[0]) ? names[index] : "UNKNOWN";
}

}  // namespace

bool PersistentEventQueue::push(const PersistentEvent& event) {
  if (count_ >= kCapacity) {
    if (dropped_ != 0xFFFFu) ++dropped_;
    return false;
  }
  events_[tail_] = event;
  tail_ = static_cast<uint8_t>((tail_ + 1u) % kCapacity);
  ++count_;
  return true;
}

bool PersistentEventQueue::peek(PersistentEvent& event) const {
  if (count_ == 0) return false;
  event = events_[head_];
  return true;
}

bool PersistentEventQueue::pop() {
  if (count_ == 0) return false;
  head_ = static_cast<uint8_t>((head_ + 1u) % kCapacity);
  --count_;
  return true;
}

size_t formatEventLog(const PersistentEvent& event, char* out, size_t out_cap) {
  if (out == nullptr || out_cap == 0) return 0;
  LineWriter writer(out, out_cap);
  writer.text("T=");
  writer.unsignedValue(event.timestamp_ms);
  writer.text(" SEV=");
  writer.text(severityName(event.severity));
  writer.text(" CODE=");
  writer.text(errorName(event.code));
  writer.text(" DETAIL=");
  writer.unsignedValue(event.detail);
  return writer.finish();
}

size_t formatCrashLog(const fault_capture::Snapshot& crash, char* out,
                      size_t out_cap) {
  if (out == nullptr || out_cap == 0 ||
      crash.reason == fault_capture::FatalReason::None) {
    return 0;
  }
  LineWriter writer(out, out_cap);
  writer.text("BOOT SEV=CRITICAL CRASH=");
  writer.unsignedValue(static_cast<uint8_t>(crash.reason));
  writer.text(" STAGE=");
  writer.unsignedValue(static_cast<uint8_t>(crash.stage));
  writer.text(" TASK=");
  writer.text(crash.task_name[0] == '\0' ? "UNKNOWN" : crash.task_name);
  writer.text(" PC=");
  writer.hex32(crash.pc);
  writer.text(" LR=");
  writer.hex32(crash.lr);
  return writer.finish();
}

size_t formatWatchdogResetLog(uint32_t missed_mask, uint8_t dxl_progress,
                              uint8_t control_progress, uint8_t safety_state,
                              char* out, size_t out_cap) {
  if (out == nullptr || out_cap == 0) return 0;
  LineWriter writer(out, out_cap);
  writer.text("BOOT SEV=CRITICAL WATCHDOG=1 MISSED=");
  writer.hex32(missed_mask);
  writer.text(" DXL_PROGRESS=");
  writer.unsignedValue(dxl_progress);
  writer.text(" CONTROL_PROGRESS=");
  writer.unsignedValue(control_progress);
  writer.text(" STATE=");
  writer.unsignedValue(safety_state);
  return writer.finish();
}

}  // namespace safety