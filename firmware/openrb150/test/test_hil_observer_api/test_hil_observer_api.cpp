#include <unity.h>

#include "../../src/hil/observer_api.h"

namespace {

void writeU32(uint8_t* bytes, uint32_t value) {
  bytes[0] = static_cast<uint8_t>(value & 0xFF);
  bytes[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  bytes[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  bytes[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

hil::OutputGuardStatus readyGuard() {
  hil::OutputGuardStatus status;
  status.output_disabled = true;
  status.power_guard_active = true;
  status.torque_guard_active = true;
  status.goal_guard_active = true;
  status.write_guard_active = true;
  return status;
}

uint16_t handle(hil::ObserverApi& api, uint8_t message_id,
                const uint8_t* request, uint16_t request_len,
                uint8_t* response) {
  uint16_t response_len = 0;
  uint8_t flags = 0;
  TEST_ASSERT_TRUE(api.handle(message_id, request, request_len, response, 64,
                              &response_len, &flags));
  return response_len;
}

}  // namespace

void test_normal_build_rejects_observer_commands() {
  hil::ObserverApi api(false);
  api.setNow(100);
  api.setLiveState(hil::ObserverApi::kDisarmedState);
  api.setMaintenanceLock(99, true);
  api.setOutputGuard(readyGuard());

  uint8_t request[4] = {};
  writeU32(request, 99);
  uint8_t response[64] = {};
  TEST_ASSERT_EQUAL_UINT16(1, handle(api, hil::observermsg::kOpenSession,
                                     request, sizeof(request), response));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::NotAvailable),
                          response[0]);
}

void test_open_requires_disabled_guard_disarmed_and_maintenance_token() {
  hil::ObserverApi api(true);
  api.setNow(100);
  api.setOutputGuard(readyGuard());
  uint8_t request[4] = {};
  uint8_t response[64] = {};
  writeU32(request, 44);

  api.setLiveState(3);
  api.setMaintenanceLock(44, true);
  handle(api, hil::observermsg::kOpenSession, request, sizeof(request),
         response);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Rejected),
                          response[0]);

  api.setLiveState(hil::ObserverApi::kDisarmedState);
  api.setMaintenanceLock(45, true);
  handle(api, hil::observermsg::kOpenSession, request, sizeof(request),
         response);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Rejected),
                          response[0]);

  api.setMaintenanceLock(44, true);
  TEST_ASSERT_EQUAL_UINT16(12, handle(api, hil::observermsg::kOpenSession,
                                      request, sizeof(request), response));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Ok),
                          response[0]);
  TEST_ASSERT_TRUE(api.sessionOpen());
  TEST_ASSERT_NOT_EQUAL(0u, api.sessionId());
  TEST_ASSERT_NOT_EQUAL(0u, api.sessionToken());
}

void test_open_accepts_maintenance_gated_safe_state() {
  hil::ObserverApi api(true);
  api.setNow(100);
  api.setLiveState(hil::ObserverApi::kMacMaintenanceState);
  api.setMaintenanceLock(44, true);
  api.setOutputGuard(readyGuard());
  uint8_t request[4] = {};
  writeU32(request, 44);
  uint8_t response[64] = {};

  TEST_ASSERT_EQUAL_UINT16(12, handle(api, hil::observermsg::kOpenSession,
                                      request, sizeof(request), response));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Ok),
                          response[0]);
  TEST_ASSERT_TRUE(api.sessionOpen());
}

void test_capture_is_bounded_rate_limited_and_expires_without_authority() {
  hil::ObserverApi api(true);
  api.setNow(100);
  api.setLiveState(hil::ObserverApi::kDisarmedState);
  api.setMaintenanceLock(44, true);
  api.setOutputGuard(readyGuard());
  uint8_t maintenance_request[4] = {};
  writeU32(maintenance_request, 44);
  uint8_t response[64] = {};
  handle(api, hil::observermsg::kOpenSession, maintenance_request,
         sizeof(maintenance_request), response);

  uint8_t capture_request[5] = {};
  writeU32(capture_request, api.sessionToken());
  capture_request[4] = hil::ObserverApi::kMaxCaptureSteps + 1;
  handle(api, hil::observermsg::kCapture, capture_request,
         sizeof(capture_request), response);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::BadRequest),
                          response[0]);

  capture_request[4] = 3;
  TEST_ASSERT_EQUAL_UINT16(5, handle(api, hil::observermsg::kCapture,
                                     capture_request, sizeof(capture_request),
                                     response));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Ok),
                          response[0]);
  hil::CaptureRequest capture;
  TEST_ASSERT_TRUE(api.takeCaptureRequest(&capture));
  TEST_ASSERT_EQUAL_UINT8(3, capture.step_count);
  TEST_ASSERT_TRUE(api.captureActive());

  handle(api, hil::observermsg::kCapture, capture_request,
         sizeof(capture_request), response);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Busy),
                          response[0]);

  api.setNow(1101);
  api.tick();
  TEST_ASSERT_FALSE(api.sessionOpen());
  hil::CaptureAbortRequest abort;
  TEST_ASSERT_TRUE(api.takeAbortRequest(&abort));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::CaptureEndReason::HostHeartbeatExpired),
      static_cast<uint8_t>(abort.reason));
  TEST_ASSERT_EQUAL_UINT32(capture.capture_id, abort.capture_id);
}

  void test_capture_controls_respect_task_handoff_capacity() {
    hil::ObserverApi api(true);
    api.setNow(100);
    api.setLiveState(hil::ObserverApi::kDisarmedState);
    api.setMaintenanceLock(44, true);
    api.setOutputGuard(readyGuard());
    uint8_t maintenance_request[4] = {};
    writeU32(maintenance_request, 44);
    uint8_t response[64] = {};
    handle(api, hil::observermsg::kOpenSession, maintenance_request,
      sizeof(maintenance_request), response);

    uint8_t capture_request[5] = {};
    writeU32(capture_request, api.sessionToken());
    capture_request[4] = 1;
    api.setTaskRequestCapacity(false);
    handle(api, hil::observermsg::kCapture, capture_request,
      sizeof(capture_request), response);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Busy),
             response[0]);
    TEST_ASSERT_FALSE(api.captureActive());

    uint8_t mark_request[8] = {};
    writeU32(mark_request, api.sessionToken());
    writeU32(&mark_request[4], 77);
    api.setTaskRequestCapacity(true);
    handle(api, hil::observermsg::kMark, mark_request, sizeof(mark_request),
      response);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::NoCapture),
             response[0]);

    handle(api, hil::observermsg::kCapture, capture_request,
      sizeof(capture_request), response);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Ok),
             response[0]);
    api.setTaskRequestCapacity(false);
    handle(api, hil::observermsg::kMark, mark_request, sizeof(mark_request),
      response);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Busy),
             response[0]);
  }

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_normal_build_rejects_observer_commands);
  RUN_TEST(test_open_requires_disabled_guard_disarmed_and_maintenance_token);
  RUN_TEST(test_open_accepts_maintenance_gated_safe_state);
  RUN_TEST(test_capture_is_bounded_rate_limited_and_expires_without_authority);
  RUN_TEST(test_capture_controls_respect_task_handoff_capacity);
  return UNITY_END();
}