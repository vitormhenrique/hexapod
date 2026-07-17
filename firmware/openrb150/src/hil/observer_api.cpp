#include "observer_api.h"

namespace hil {
namespace {

constexpr uint8_t kErrorFlag = 0x02;

uint32_t readU32(const uint8_t* bytes) {
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) |
         (static_cast<uint32_t>(bytes[3]) << 24);
}

void writeU16(uint8_t* bytes, uint16_t value) {
  bytes[0] = static_cast<uint8_t>(value & 0xFF);
  bytes[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void writeU32(uint8_t* bytes, uint32_t value) {
  bytes[0] = static_cast<uint8_t>(value & 0xFF);
  bytes[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  bytes[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  bytes[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

}  // namespace

void ObserverApi::reset() {
  guard_ = OutputGuardStatus{};
  now_ms_ = 0;
  live_state_ = 0;
  maintenance_token_ = 0;
  maintenance_held_ = false;
  task_request_capacity_ = true;
  next_session_id_ = 1;
  next_session_token_ = 1;
  next_capture_id_ = 1;
  session_id_ = 0;
  session_token_ = 0;
  last_heartbeat_ms_ = 0;
  last_capture_start_ms_ = 0;
  have_capture_start_ = false;
  session_open_ = false;
  capture_active_ = false;
  capture_pending_ = false;
  abort_pending_ = false;
  marker_pending_ = false;
  capture_request_ = CaptureRequest{};
  abort_request_ = CaptureAbortRequest{};
  marker_request_ = MarkerRequest{};
}

bool ObserverApi::available() const {
  return build_available_ && guard_.output_disabled &&
         guard_.power_guard_active && guard_.torque_guard_active &&
         guard_.goal_guard_active;
}

bool ObserverApi::hasSessionToken(const uint8_t* request,
                                  uint16_t request_len) const {
  return request != nullptr && request_len >= 4 && session_open_ &&
         readU32(request) == session_token_;
}

bool ObserverApi::canOpen(uint32_t maintenance_token) const {
  const bool safe_state = live_state_ == kDisarmedState ||
                          live_state_ == kMacMaintenanceState;
  return available() && safe_state && maintenance_held_ &&
         maintenance_token != 0 && maintenance_token == maintenance_token_;
}

void ObserverApi::closeSession(CaptureEndReason reason) {
  if (capture_active_) {
    abort_request_.session_id = session_id_;
    abort_request_.capture_id = capture_request_.capture_id;
    abort_request_.reason = reason;
    abort_pending_ = true;
  }
  session_open_ = false;
  session_id_ = 0;
  session_token_ = 0;
  marker_pending_ = false;
}

void ObserverApi::tick() {
  if (session_open_ && now_ms_ - last_heartbeat_ms_ > kSessionTtlMs) {
    closeSession(CaptureEndReason::HostHeartbeatExpired);
  }
}

bool ObserverApi::writeResult(ObserverResult result, uint8_t* response,
                              uint16_t response_cap, uint16_t* response_len,
                              uint8_t* response_flags) const {
  if (response == nullptr || response_len == nullptr || response_flags == nullptr ||
      response_cap < 1) {
    return false;
  }
  response[0] = static_cast<uint8_t>(result);
  *response_len = 1;
  *response_flags = (result == ObserverResult::BadRequest) ? kErrorFlag : 0;
  return true;
}

bool ObserverApi::writeStatus(ObserverResult result, uint8_t* response,
                              uint16_t response_cap, uint16_t* response_len,
                              uint8_t* response_flags) const {
  if (response == nullptr || response_len == nullptr || response_flags == nullptr ||
      response_cap < 19) {
    return false;
  }
  response[0] = static_cast<uint8_t>(result);
  response[1] = session_open_ ? 1 : 0;
  response[2] = capture_active_ ? 1 : 0;
  response[3] = available() ? 1 : 0;
  writeU32(&response[4], session_id_);
  writeU32(&response[8], capture_request_.capture_id);
  writeU32(&response[12], last_heartbeat_ms_);
  writeU16(&response[16], kTraceSchemaVersion);
  response[18] = guard_.output_disabled ? 1 : 0;
  *response_len = 19;
  *response_flags = (result == ObserverResult::BadRequest) ? kErrorFlag : 0;
  return true;
}

bool ObserverApi::handle(uint8_t msg_id, const uint8_t* request,
                         uint16_t request_len, uint8_t* response,
                         uint16_t response_cap, uint16_t* response_len,
                         uint8_t* response_flags) {
  if (!observermsg::isObserverMsg(msg_id)) return false;
  tick();

  if (msg_id == observermsg::kGetCapability) {
    return writeStatus(available() ? ObserverResult::Ok
                                   : ObserverResult::NotAvailable,
                       response, response_cap, response_len, response_flags);
  }
  if (!available()) {
    return writeResult(ObserverResult::NotAvailable, response, response_cap,
                       response_len, response_flags);
  }

  switch (msg_id) {
    case observermsg::kOpenSession: {
      if (request == nullptr || request_len != 4) {
        return writeResult(ObserverResult::BadRequest, response, response_cap,
                           response_len, response_flags);
      }
      if (session_open_) {
        return writeResult(ObserverResult::Busy, response, response_cap,
                           response_len, response_flags);
      }
      if (!canOpen(readU32(request))) {
        return writeResult(ObserverResult::Rejected, response, response_cap,
                           response_len, response_flags);
      }
      session_id_ = next_session_id_++;
      if (next_session_id_ == 0) next_session_id_ = 1;
      session_token_ = next_session_token_++;
      if (next_session_token_ == 0) next_session_token_ = 1;
      session_open_ = true;
      last_heartbeat_ms_ = now_ms_;
      if (response_cap < 12) {
        closeSession(CaptureEndReason::SessionClosed);
        return writeResult(ObserverResult::BadRequest, response, response_cap,
                           response_len, response_flags);
      }
      response[0] = static_cast<uint8_t>(ObserverResult::Ok);
      writeU32(&response[1], session_id_);
      writeU32(&response[5], session_token_);
      writeU16(&response[9], kTraceSchemaVersion);
      response[11] = 0x0F;
      *response_len = 12;
      *response_flags = 0;
      return true;
    }
    case observermsg::kCloseSession: {
      if (!hasSessionToken(request, request_len)) {
        return writeResult(ObserverResult::BadToken, response, response_cap,
                           response_len, response_flags);
      }
      closeSession(CaptureEndReason::SessionClosed);
      return writeResult(ObserverResult::Ok, response, response_cap,
                         response_len, response_flags);
    }
    case observermsg::kHeartbeat: {
      if (!hasSessionToken(request, request_len)) {
        return writeResult(ObserverResult::BadToken, response, response_cap,
                           response_len, response_flags);
      }
      last_heartbeat_ms_ = now_ms_;
      return writeResult(ObserverResult::Ok, response, response_cap,
                         response_len, response_flags);
    }
    case observermsg::kCapture: {
      if (request == nullptr || request_len != 5) {
        return writeResult(ObserverResult::BadRequest, response, response_cap,
                           response_len, response_flags);
      }
      if (!hasSessionToken(request, request_len)) {
        return writeResult(ObserverResult::BadToken, response, response_cap,
                           response_len, response_flags);
      }
      const uint8_t step_count = request[4];
      if (step_count < kMinCaptureSteps || step_count > kMaxCaptureSteps) {
        return writeResult(ObserverResult::BadRequest, response, response_cap,
                           response_len, response_flags);
      }
      if (capture_active_) {
        return writeResult(ObserverResult::Busy, response, response_cap,
                           response_len, response_flags);
      }
      if (!task_request_capacity_) {
        return writeResult(ObserverResult::Busy, response, response_cap,
                           response_len, response_flags);
      }
      if (have_capture_start_ &&
          now_ms_ - last_capture_start_ms_ < kCaptureStartIntervalMs) {
        return writeResult(ObserverResult::RateLimited, response, response_cap,
                           response_len, response_flags);
      }
      capture_request_.session_id = session_id_;
      capture_request_.capture_id = next_capture_id_++;
      if (next_capture_id_ == 0) next_capture_id_ = 1;
      capture_request_.step_count = step_count;
      capture_active_ = true;
      capture_pending_ = true;
      last_capture_start_ms_ = now_ms_;
      have_capture_start_ = true;
      if (response_cap < 5) {
        capture_active_ = false;
        capture_pending_ = false;
        return writeResult(ObserverResult::BadRequest, response, response_cap,
                           response_len, response_flags);
      }
      response[0] = static_cast<uint8_t>(ObserverResult::Ok);
      writeU32(&response[1], capture_request_.capture_id);
      *response_len = 5;
      *response_flags = 0;
      return true;
    }
    case observermsg::kAbortCapture: {
      if (!hasSessionToken(request, request_len)) {
        return writeResult(ObserverResult::BadToken, response, response_cap,
                           response_len, response_flags);
      }
      if (!capture_active_) {
        return writeResult(ObserverResult::NoCapture, response, response_cap,
                           response_len, response_flags);
      }
      if (abort_pending_ || !task_request_capacity_) {
        return writeResult(ObserverResult::Busy, response, response_cap,
                           response_len, response_flags);
      }
      abort_request_.session_id = session_id_;
      abort_request_.capture_id = capture_request_.capture_id;
      abort_request_.reason = CaptureEndReason::HostAborted;
      abort_pending_ = true;
      return writeResult(ObserverResult::Ok, response, response_cap,
                         response_len, response_flags);
    }
    case observermsg::kMark: {
      if (request == nullptr || request_len != 8) {
        return writeResult(ObserverResult::BadRequest, response, response_cap,
                           response_len, response_flags);
      }
      if (!hasSessionToken(request, request_len)) {
        return writeResult(ObserverResult::BadToken, response, response_cap,
                           response_len, response_flags);
      }
      if (!capture_active_) {
        return writeResult(ObserverResult::NoCapture, response, response_cap,
                           response_len, response_flags);
      }
      if (marker_pending_ || !task_request_capacity_) {
        return writeResult(ObserverResult::Busy, response, response_cap,
                           response_len, response_flags);
      }
      marker_request_.capture_id = capture_request_.capture_id;
      marker_request_.marker_id = readU32(&request[4]);
      marker_pending_ = true;
      return writeResult(ObserverResult::Ok, response, response_cap,
                         response_len, response_flags);
    }
    case observermsg::kGetSessionStatus: {
      if (!hasSessionToken(request, request_len)) {
        return writeResult(ObserverResult::BadToken, response, response_cap,
                           response_len, response_flags);
      }
      return writeStatus(ObserverResult::Ok, response, response_cap,
                         response_len, response_flags);
    }
    default:
      return false;
  }
}

bool ObserverApi::takeCaptureRequest(CaptureRequest* request) {
  if (!capture_pending_ || request == nullptr) return false;
  *request = capture_request_;
  capture_pending_ = false;
  return true;
}

bool ObserverApi::takeAbortRequest(CaptureAbortRequest* request) {
  if (!abort_pending_ || request == nullptr) return false;
  *request = abort_request_;
  abort_pending_ = false;
  return true;
}

bool ObserverApi::takeMarkerRequest(MarkerRequest* request) {
  if (!marker_pending_ || request == nullptr) return false;
  *request = marker_request_;
  marker_pending_ = false;
  return true;
}

void ObserverApi::noteCaptureFinished(uint32_t capture_id) {
  if (capture_active_ && capture_request_.capture_id == capture_id) {
    capture_active_ = false;
    capture_pending_ = false;
    marker_pending_ = false;
  }
}

}  // namespace hil