#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../controller/controller_contract.h"
#include "observer_api.h"
#include "trace_codec.h"

namespace hil {
namespace trace {

constexpr uint8_t kMaxCaptureSteps = 32;
// The largest v1 logical record is a fully populated ControllerStep (951
// bytes). Keep a small fixed margin rather than retaining padded C++ value
// objects plus a second serialized config payload in SRAM.
constexpr uint16_t kMaxLogicalRecordBytes = 960;

struct RecordView {
  RecordType type = RecordType::Begin;
  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  uint32_t record_seq = 0;
  uint16_t logical_length = 0;
  uint16_t logical_crc16 = 0;
};

struct CaptureSummary {
  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  uint8_t requested_steps = 0;
  uint8_t recorded_steps = 0;
  uint8_t queue_high_water = 0;
  CaptureEndReason end_reason = CaptureEndReason::Complete;
  bool active = false;
  bool terminal_pending = false;
};

// A one-slot, single-producer/single-consumer recorder. controlTask owns
// captureStep(); apiTask owns peek()/copySlice()/acknowledge(). A full slot is
// never overwritten: the next control step aborts the capture if apiTask has
// not drained it, which preserves trace integrity without blocking control.
class TraceRecorder {
 public:
  void reset();

  bool begin(const CaptureRequest& request,
             const OutputGuardStatus& initial_guard,
             const controller::ControllerConfigSnapshot* config = nullptr);
  bool captureStep(const controller::ControllerStepInput& input,
                   const controller::RobotCommand& command,
                   const OutputGuardStatus& guard);
  bool captureStep(const controller::RobotState& state,
                   const controller::ControllerIntent& intent,
                   const controller::ControllerConfigSnapshot& config,
                   const controller::ControllerTime& time,
                   const controller::RobotCommand& command,
                   const OutputGuardStatus& guard);
  bool markNext(uint32_t marker_id);
  void abort(CaptureEndReason reason, const OutputGuardStatus& guard);

  bool active() const { return summary_.active; }
  bool terminalPending() const { return summary_.terminal_pending; }
  const CaptureSummary& summary() const { return summary_; }

  // Returns one immutable logical record at a time. `copySlice()` may be
  // called repeatedly on that view to generate fragment payloads without a
  // second full-record buffer.
  bool peek(RecordView* view);
  bool copySlice(const RecordView& view, uint16_t offset, uint8_t* out,
                 uint16_t out_cap, uint16_t length) const;
  void acknowledge(const RecordView& view);
  void abandonCurrent(CaptureEndReason reason,
                      const OutputGuardStatus& guard);
  void noteFragmentsSent(uint8_t count);
  bool takeCompletion(uint32_t* session_id, uint32_t* capture_id,
                      CaptureEndReason* reason);

 private:
  bool publish(RecordType type, uint16_t length);
  bool publishBegin();
  bool publishConfig(const controller::ControllerConfigSnapshot& config);
  bool publishMarker(uint32_t marker_id, uint32_t now_ms,
                     uint8_t safety_state, uint32_t step_sequence);
  bool publishOutputBlocked(uint32_t step_sequence,
                            const OutputGuardStatus& guard);
  bool publishStep(const controller::RobotState& state,
                   const controller::ControllerIntent& intent,
                   const controller::ControllerConfigSnapshot& config,
                   const controller::ControllerTime& time,
                   uint32_t step_sequence,
                   const controller::RobotCommand& command);
  bool publishEnd();
  bool publishDeferred();
  void finishCapture(CaptureEndReason reason, const OutputGuardStatus& guard);

  CaptureSummary summary_{};
  OutputGuardStatus initial_guard_{};
  OutputGuardStatus final_guard_{};
  OutputGuardStatus last_guard_{};
  OutputGuardStatus deferred_output_guard_{};
  const controller::ControllerConfigSnapshot* config_source_ = nullptr;
  uint32_t next_step_sequence_ = 1;
  uint32_t next_record_sequence_ = 1;
  uint32_t emitted_record_count_ = 0;
  uint32_t emitted_fragment_count_ = 0;
  uint32_t config_revision_ = 0;
  uint32_t deferred_marker_id_ = 0;
  uint32_t deferred_marker_time_ms_ = 0;
  uint32_t deferred_marker_step_sequence_ = 0;
  uint32_t deferred_output_step_sequence_ = 0;
  uint16_t config_payload_crc16_ = 0;
  uint8_t deferred_marker_safety_state_ = 0;
  bool config_pending_ = false;
  bool marker_for_next_step_ = false;
  uint32_t next_marker_id_ = 0;
  bool deferred_marker_pending_ = false;
  bool deferred_output_pending_ = false;
  bool completion_pending_ = false;
  volatile bool slot_ready_ = false;
  RecordView slot_view_{};
  uint8_t slot_[kMaxLogicalRecordBytes] = {};
};

}  // namespace trace
}  // namespace hil