#pragma once

// ===========================================================================
// Deduplicated error journal (portable, no Arduino deps).
//
// Firmware error reporting has to satisfy two conflicting requirements: an
// operator needs to know *that* something failed and *how bad it is*, but a
// fault that repeats at the control-loop rate must not flood the 5-20 Hz CRSF
// downlink or the USB telemetry stream with the same message.
//
// The journal solves that with a fixed table of distinct (code, detail) keys.
// Producers call note() as often as they like; the journal counts every
// occurrence but only marks an entry pending for transmission when it is
// genuinely new information:
//
//   * the first occurrence of a key,
//   * a repeat after kRepeatIntervalMs of continued failure (a heartbeat that
//     says "still broken", carrying the accumulated count),
//   * the first occurrence after the key had been quiet for kClearAfterMs
//     (treated as a new incident rather than a continuation).
//
// takePending() returns at most one entry per call, newest-severity-first, so a
// transport can drain the journal at its own rate without ever blocking a
// producer. The table never allocates and never grows: when it is full the
// lowest-severity, least-recently-seen entry is recycled, so a storm of noisy
// low-severity errors can never hide a critical one.
//
// Static memory only, no Arduino/FreeRTOS dependencies, so this builds and is
// unit-tested in the native environment.
// ===========================================================================

#include <stdint.h>

namespace safety {

// Error identity. Numeric values are part of the CRSF downlink and the USB
// telemetry tail, so do NOT renumber; append new codes at the end.
enum class ErrorCode : uint8_t {
  None = 0,
  SafetyFault = 1,        // detail = safety::FaultReason
  DxlWriteFailed = 2,     // detail = servo id, 0 = whole-bus write
  DxlReadFailed = 3,      // detail = servo id, 0 = whole-bus read
  DxlHardwareError = 4,   // detail = servo id
  DxlBusSilent = 5,       // detail = consecutive dead cycles / 10, saturated
  DxlDiscoveryIncomplete = 6,  // detail = servos still missing
  I2cFootSensorFault = 7,  // detail = foot index
  I2cMuxMissing = 8,       // detail = 0
  ConfigStorageMissing = 9,  // detail = 0
  ConfigVolatile = 10,     // detail = 0, config cannot be persisted
  ConfigCommitFailed = 11,  // detail = 0
  RcFailsafe = 12,          // detail = 0
  RcFrameErrors = 13,       // detail = CRC errors / 10, saturated
  GoalUnreachable = 14,     // detail = 0
  GoalClamped = 15,         // detail = 0
  BatteryLow = 16,          // detail = tenths of a volt
  WatchdogStall = 17,       // detail = 0
  GaitSaveRejected = 18,    // detail = GaitSaveReject value
  CaptureUnavailable = 19,  // detail = 0: OpenLog/SD unavailable
  CaptureWriteFailed = 20,  // detail = capture row type
  ArmingStarted = 21,       // detail = requested command source
};

// Why a gait-parameter save request was refused (ErrorCode::GaitSaveRejected).
enum class GaitSaveReject : uint8_t {
  NotPersistent = 1,  // no OpenLog storage: the config is volatile
  Busy = 2,           // a config transaction was already in flight
  Invalid = 3,        // the resulting config failed schema validation
  StoreFailed = 4,    // the storage transaction itself failed
};

// Severity drives both the recycling policy and the order takePending() drains
// the table, so a critical error is never starved by chatty warnings.
enum class ErrorSeverity : uint8_t {
  Info = 0,
  Warning = 1,
  Error = 2,
  Critical = 3,
};

// Distinct (code, detail) keys tracked at once. Sized for the whole fault set
// above with headroom for per-servo/per-foot details, at 16 bytes per entry.
constexpr uint8_t kMaxErrorEntries = 12;
// A still-failing key is re-announced at most this often.
constexpr uint32_t kRepeatIntervalMs = 5000;
// A key quiet for this long is considered resolved; its next occurrence is a
// new incident and is announced immediately.
constexpr uint32_t kClearAfterMs = 15000;

struct ErrorEntry {
  ErrorCode code = ErrorCode::None;
  uint8_t detail = 0;
  ErrorSeverity severity = ErrorSeverity::Info;
  uint16_t count = 0;       // occurrences in the current incident (saturating)
  uint32_t first_ms = 0;    // start of the current incident
  uint32_t last_ms = 0;     // most recent occurrence
  uint8_t sequence = 0;     // bumped per announced incident, wraps
};

class ErrorJournal {
 public:
  ErrorJournal() { reset(); }

  void reset();

  // Record one occurrence. Cheap and bounded: safe to call from any task at
  // control-loop rate. Returns true if this occurrence became pending for
  // transmission (i.e. it was genuinely new information).
  bool note(ErrorCode code, uint8_t detail, ErrorSeverity severity,
            uint32_t now_ms);

  // Convenience for level-triggered producers: note() while `active` is true,
  // and let the entry age out naturally once it stops being reported.
  bool noteIf(bool active, ErrorCode code, uint8_t detail,
              ErrorSeverity severity, uint32_t now_ms) {
    return active ? note(code, detail, severity, now_ms) : false;
  }

  // Drain one pending entry, highest severity first and oldest-pending first
  // within a severity. Returns false when nothing is pending.
  bool takePending(ErrorEntry* out);

  bool hasPending() const { return pending_count_ > 0; }
  uint8_t pendingCount() const { return pending_count_; }

  // The most recent announced entry, for transports that publish a "current
  // error" field rather than a queue. Valid once any entry has been announced.
  const ErrorEntry& latest() const { return latest_; }
  bool hasLatest() const { return latest_.code != ErrorCode::None; }

  // Total occurrences suppressed as duplicates since reset(). Reported so an
  // operator can tell "one glitch" from "failing 100 times a second".
  uint32_t suppressed() const { return suppressed_; }

  uint8_t size() const { return count_; }
  const ErrorEntry& at(uint8_t index) const { return entries_[index]; }

 private:
  int8_t find(ErrorCode code, uint8_t detail) const;
  // Index of the entry to recycle when the table is full: lowest severity,
  // then least recently seen, and never a still-pending entry if avoidable.
  int8_t recycleSlot(ErrorSeverity incoming, uint32_t now_ms) const;

  ErrorEntry entries_[kMaxErrorEntries];
  bool pending_[kMaxErrorEntries];
  ErrorEntry latest_;
  uint8_t count_ = 0;
  uint8_t pending_count_ = 0;
  uint8_t next_sequence_ = 1;
  uint32_t suppressed_ = 0;
};

}  // namespace safety
