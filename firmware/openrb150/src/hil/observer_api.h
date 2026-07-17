#pragma once

#include <stddef.h>
#include <stdint.h>

#include "output_guard.h"

namespace hil {

namespace observermsg {
constexpr uint8_t kGetCapability = 0x14;
constexpr uint8_t kOpenSession = 0x15;
constexpr uint8_t kCloseSession = 0x16;
constexpr uint8_t kHeartbeat = 0x17;
constexpr uint8_t kCapture = 0x18;
constexpr uint8_t kAbortCapture = 0x19;
constexpr uint8_t kMark = 0x1A;
constexpr uint8_t kGetSessionStatus = 0x1B;
constexpr uint8_t kFirst = kGetCapability;
constexpr uint8_t kLast = kGetSessionStatus;

constexpr bool isObserverMsg(uint8_t id) {
  return id >= kFirst && id <= kLast;
}
}  // namespace observermsg

enum class ObserverResult : uint8_t {
  Ok = 0,
  NotAvailable = 1,
  BadRequest = 2,
  Rejected = 3,
  BadToken = 4,
  Busy = 5,
  RateLimited = 6,
  NoCapture = 7,
};

enum class CaptureEndReason : uint8_t {
  Complete = 0,
  HostHeartbeatExpired = 1,
  TransportOverflow = 2,
  TransportTimeout = 3,
  HostAborted = 4,
  SessionClosed = 5,
};

struct CaptureRequest {
  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  uint8_t step_count = 0;
};

struct CaptureAbortRequest {
  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  CaptureEndReason reason = CaptureEndReason::HostAborted;
};

struct MarkerRequest {
  uint32_t capture_id = 0;
  uint32_t marker_id = 0;
};

// Portable command-side state for the observer-only HIL command group. This
// class accepts no motion/safety requests and owns no actuator path; the task
// adapter supplies live safety, maintenance, and output-guard snapshots.
class ObserverApi {
 public:
  static constexpr uint16_t kTraceSchemaVersion = 1;
  static constexpr uint32_t kSessionTtlMs = 1000;
  static constexpr uint32_t kCaptureStartIntervalMs = 1000;
  static constexpr uint8_t kMinCaptureSteps = 1;
  static constexpr uint8_t kMaxCaptureSteps = 32;
  static constexpr uint8_t kDisarmedState = 2;
  static constexpr uint8_t kMacMaintenanceState = 8;

  explicit ObserverApi(bool build_available = kBuildOutputDisabled)
      : build_available_(build_available) {
    reset();
  }

  void reset();
  void setNow(uint32_t now_ms) { now_ms_ = now_ms; }
  void setLiveState(uint8_t state) { live_state_ = state; }
  void setMaintenanceLock(uint32_t token, bool held) {
    maintenance_token_ = token;
    maintenance_held_ = held;
  }
  void setOutputGuard(const OutputGuardStatus& status) { guard_ = status; }
  // The board adapter updates this before dispatching a command. It represents
  // capacity in the fixed API-to-control request handoff, not an authority or
  // a timing lease.
  void setTaskRequestCapacity(bool available) {
    task_request_capacity_ = available;
  }

  // Expires an observer session independently of maintenance/Jetson TTLs.
  // If a capture was active, its abort request remains available to the task
  // adapter so it can emit the required terminal record.
  void tick();

  // Returns false only when `msg_id` is outside the HIL observer range.
  bool handle(uint8_t msg_id, const uint8_t* request, uint16_t request_len,
              uint8_t* response, uint16_t response_cap,
              uint16_t* response_len, uint8_t* response_flags);

  bool takeCaptureRequest(CaptureRequest* request);
  bool takeAbortRequest(CaptureAbortRequest* request);
  bool takeMarkerRequest(MarkerRequest* request);
  void noteCaptureFinished(uint32_t capture_id);

  bool sessionOpen() const { return session_open_; }
  bool captureActive() const { return capture_active_; }
  uint32_t sessionId() const { return session_id_; }
  uint32_t sessionToken() const { return session_token_; }

 private:
  bool available() const;
  bool hasSessionToken(const uint8_t* request, uint16_t request_len) const;
  bool canOpen(uint32_t maintenance_token) const;
  void closeSession(CaptureEndReason reason);
  bool writeResult(ObserverResult result, uint8_t* response,
                   uint16_t response_cap, uint16_t* response_len,
                   uint8_t* response_flags) const;
  bool writeStatus(ObserverResult result, uint8_t* response,
                   uint16_t response_cap, uint16_t* response_len,
                   uint8_t* response_flags) const;

  bool build_available_ = false;
  OutputGuardStatus guard_{};
  uint32_t now_ms_ = 0;
  uint8_t live_state_ = 0;
  uint32_t maintenance_token_ = 0;
  bool maintenance_held_ = false;
  bool task_request_capacity_ = true;
  uint32_t next_session_id_ = 1;
  uint32_t next_session_token_ = 1;
  uint32_t next_capture_id_ = 1;
  uint32_t session_id_ = 0;
  uint32_t session_token_ = 0;
  uint32_t last_heartbeat_ms_ = 0;
  uint32_t last_capture_start_ms_ = 0;
  bool have_capture_start_ = false;
  bool session_open_ = false;
  bool capture_active_ = false;
  bool capture_pending_ = false;
  bool abort_pending_ = false;
  bool marker_pending_ = false;
  CaptureRequest capture_request_{};
  CaptureAbortRequest abort_request_{};
  MarkerRequest marker_request_{};
};

}  // namespace hil