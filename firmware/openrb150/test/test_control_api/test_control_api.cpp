// Native (host) Unity tests for the safety control command group
// (ESTOP / CLEAR_FAULT / SET_ARMING / SET_MODE) handled by ControlApi and
// routed through the protocol api dispatcher.
//
// Run with:  pio test -e native -f test_control_api

#include <string.h>

#include <unity.h>

#include "protocol/api.h"
#include "protocol/control_api.h"
#include "protocol/framing.h"

using namespace protocol;

namespace {

api::DeviceInfo makeInfo() {
  api::DeviceInfo info;
  info.fw_major = 0;
  info.fw_minor = 1;
  info.fw_patch = 0;
  info.feature_bits = 0;
  const char name[] = "OpenRB150-Hex";
  memset(info.device_name, 0, sizeof(info.device_name));
  memcpy(info.device_name, name, sizeof(name) - 1);
  return info;
}

api::StatusSnapshot makeStatus(uint8_t state = 2) {
  api::StatusSnapshot st;
  st.uptime_ms = 1000;
  st.state = state;
  st.battery_mv = 11800;
  return st;
}

// Build a Command request wire frame for msg_id/seq with an optional payload.
size_t buildRequest(uint8_t msg_id, uint16_t seq, const uint8_t* payload,
                    uint16_t payload_len, uint8_t* out, size_t cap) {
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = msg_id;
  h.seq = seq;
  h.payload_len = payload_len;
  return encodeFrame(h, payload, out, cap);
}

// Run one control request through handleRequest with the given ControlApi and
// decode the response.
DecodeStatus runControl(ControlApi& ctrl, uint8_t msg_id, uint16_t seq,
                        const uint8_t* payload, uint16_t payload_len,
                        Header* resp_h, uint8_t* resp_payload,
                        size_t* resp_len) {
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(msg_id, seq, payload, payload_len, req, sizeof(req));

  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n =
      api::handleRequest(req + 1, req_n - 2, info, st, resp, sizeof(resp),
                         nullptr, nullptr, &ctrl);
  TEST_ASSERT_TRUE(resp_n > 0);
  return decodeFrameBody(resp + 1, resp_n - 2, resp_h, resp_payload,
                         kMaxPayload, resp_len);
}

void test_estop_latches() {
  ControlApi ctrl;
  ctrl.setLiveState(2, 0);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runControl(ctrl, ctrlmsg::kEstop, 1, nullptr, 0, &h, p,
                               &len));
  TEST_ASSERT_EQUAL_UINT(3, len);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(CtrlResult::Ok), p[0]);
  TEST_ASSERT_TRUE(ctrl.estopActive());
}

void test_clear_fault_releases_estop_and_pulses() {
  ControlApi ctrl;
  ctrl.setLiveState(12, 2);  // Estop / HostEstop
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  // Latch first.
  runControl(ctrl, ctrlmsg::kEstop, 1, nullptr, 0, &h, p, &len);
  TEST_ASSERT_TRUE(ctrl.estopActive());
  // Clear.
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runControl(ctrl, ctrlmsg::kClearFault, 2, nullptr, 0, &h, p,
                               &len));
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(CtrlResult::Ok), p[0]);
  TEST_ASSERT_FALSE(ctrl.estopActive());
  // The clear-fault pulse is one-shot.
  TEST_ASSERT_TRUE(ctrl.consumeClearFault());
  TEST_ASSERT_FALSE(ctrl.consumeClearFault());
}

void test_set_arming_disarm_latches() {
  ControlApi ctrl;
  ctrl.setLiveState(4, 0);  // StandReady
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  uint8_t disarm = static_cast<uint8_t>(ArmingRequest::Disarm);
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runControl(ctrl, ctrlmsg::kSetArming, 1, &disarm, 1, &h, p,
                               &len));
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(CtrlResult::Ok), p[0]);
  TEST_ASSERT_TRUE(ctrl.disarmRequested());
  // Arm releases the latch.
  uint8_t arm = static_cast<uint8_t>(ArmingRequest::Arm);
  runControl(ctrl, ctrlmsg::kSetArming, 2, &arm, 1, &h, p, &len);
  TEST_ASSERT_FALSE(ctrl.disarmRequested());
}

void test_set_arming_bad_request() {
  ControlApi ctrl;
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runControl(ctrl, ctrlmsg::kSetArming, 1, nullptr, 0, &h, p,
                               &len));
  TEST_ASSERT_EQUAL_UINT(1, len);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(CtrlResult::BadRequest), p[0]);
  TEST_ASSERT_TRUE((h.flags & api::flag::kError) != 0);
}

void test_set_mode_honors_disarm_and_estop_rejects_others() {
  ControlApi ctrl;
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  uint8_t disarmed = 2;
  runControl(ctrl, ctrlmsg::kSetMode, 1, &disarmed, 1, &h, p, &len);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(CtrlResult::Ok), p[0]);
  TEST_ASSERT_TRUE(ctrl.disarmRequested());

  uint8_t estop = 12;
  runControl(ctrl, ctrlmsg::kSetMode, 2, &estop, 1, &h, p, &len);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(CtrlResult::Ok), p[0]);
  TEST_ASSERT_TRUE(ctrl.estopActive());

  uint8_t rc_manual = 5;
  runControl(ctrl, ctrlmsg::kSetMode, 3, &rc_manual, 1, &h, p, &len);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(CtrlResult::Rejected), p[0]);
}

void test_unknown_without_ctrl_is_error() {
  // Without a ControlApi the dispatcher rejects the control range as unknown.
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(ctrlmsg::kEstop, 1, nullptr, 0, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(req + 1, req_n - 2, info, st, resp,
                                           sizeof(resp));
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    decodeFrameBody(resp + 1, resp_n - 2, &h, p, kMaxPayload,
                                    &len));
  TEST_ASSERT_TRUE((h.flags & api::flag::kError) != 0);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(api::Error::UnknownMsg), p[0]);
}

void test_jetson_heartbeat_notes_once_and_echoes_state() {
  // JETSON_HEARTBEAT (lmt.13) records a one-shot liveness flag on the ControlApi
  // and answers with uptime_ms(4) + state(1) so the Jetson can read back the
  // live state and learn whether it has authority yet.
  ControlApi ctrl;
  ctrl.setLiveState(7, 0);  // JetsonAssisted
  // No heartbeat yet.
  TEST_ASSERT_FALSE(ctrl.consumeJetsonHeartbeat());

  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(api::msg::kJetsonHeartbeat, 9, nullptr, 0, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus(7);
  const size_t resp_n =
      api::handleRequest(req + 1, req_n - 2, info, st, resp, sizeof(resp),
                         nullptr, nullptr, &ctrl);
  TEST_ASSERT_TRUE(resp_n > 0);
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    decodeFrameBody(resp + 1, resp_n - 2, &h, p, kMaxPayload,
                                    &len));
  TEST_ASSERT_EQUAL_UINT(5, len);
  TEST_ASSERT_EQUAL_HEX8(7, p[4]);  // echoes the live state
  // The flag is one-shot: drained exactly once.
  TEST_ASSERT_TRUE(ctrl.consumeJetsonHeartbeat());
  TEST_ASSERT_FALSE(ctrl.consumeJetsonHeartbeat());
}

void test_jetson_heartbeat_without_ctrl_still_answers() {
  // The dispatcher answers JETSON_HEARTBEAT even without a ControlApi (it just
  // cannot record liveness); it must not be reported as an unknown message.
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(api::msg::kJetsonHeartbeat, 1, nullptr, 0, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(req + 1, req_n - 2, info, st, resp,
                                           sizeof(resp));
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_TRUE(resp_n > 0);
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    decodeFrameBody(resp + 1, resp_n - 2, &h, p, kMaxPayload,
                                    &len));
  TEST_ASSERT_FALSE((h.flags & api::flag::kError) != 0);
  TEST_ASSERT_EQUAL_UINT(5, len);
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_estop_latches);
  RUN_TEST(test_clear_fault_releases_estop_and_pulses);
  RUN_TEST(test_set_arming_disarm_latches);
  RUN_TEST(test_set_arming_bad_request);
  RUN_TEST(test_set_mode_honors_disarm_and_estop_rejects_others);
  RUN_TEST(test_unknown_without_ctrl_is_error);
  RUN_TEST(test_jetson_heartbeat_notes_once_and_echoes_state);
  RUN_TEST(test_jetson_heartbeat_without_ctrl_still_answers);
  return UNITY_END();
}
