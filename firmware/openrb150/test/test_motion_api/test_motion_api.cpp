// Native (host) Unity tests for the motion command group
// (SET_GAIT / SET_GAIT_PARAMS / SET_BODY_TWIST / SET_BODY_POSE / STOP_MOTION)
// handled by MotionApi and routed through the protocol api dispatcher.
//
// Run with:  pio test -e native -f test_motion_api

#include <string.h>

#include <unity.h>

#include "protocol/api.h"
#include "protocol/framing.h"
#include "protocol/motion_api.h"

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

api::StatusSnapshot makeStatus(uint8_t state = 5) {
  api::StatusSnapshot st;
  st.uptime_ms = 1000;
  st.state = state;
  st.battery_mv = 11800;
  return st;
}

size_t buildRequest(uint8_t msg_id, uint16_t seq, const uint8_t* payload,
                    uint16_t payload_len, uint8_t* out, size_t cap) {
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = msg_id;
  h.seq = seq;
  h.payload_len = payload_len;
  return encodeFrame(h, payload, out, cap);
}

DecodeStatus runMotion(MotionApi& motion, uint8_t msg_id, uint16_t seq,
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
                         nullptr, nullptr, nullptr, &motion);
  TEST_ASSERT_TRUE(resp_n > 0);
  return decodeFrameBody(resp + 1, resp_n - 2, resp_h, resp_payload,
                         kMaxPayload, resp_len);
}

void putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
void putI16(uint8_t* p, int16_t v) { putU16(p, static_cast<uint16_t>(v)); }

}  // namespace

void test_set_gait_selects_and_bumps_seq() {
  MotionApi m;
  m.reset();
  m.setLiveState(5, true);
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  const uint8_t gait = motiongait::kTripod;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runMotion(m, motionmsg::kSetGait, 1, &gait, 1, &h, pl, &n));
  TEST_ASSERT_EQUAL_UINT(3, n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotionResult::Ok), pl[0]);
  TEST_ASSERT_EQUAL_UINT8(5, pl[1]);   // live state echo
  TEST_ASSERT_EQUAL_UINT8(1, pl[2]);   // motion allowed echo
  TEST_ASSERT_EQUAL_UINT8(motiongait::kTripod, m.intent().gait);
  TEST_ASSERT_EQUAL_UINT32(1, m.intent().seq);
}

void test_set_gait_rejects_out_of_range() {
  MotionApi m;
  m.reset();
  m.setLiveState(5, false);
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  const uint8_t bad = 99;
  runMotion(m, motionmsg::kSetGait, 2, &bad, 1, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotionResult::Rejected), pl[0]);
  TEST_ASSERT_EQUAL_UINT8(0, pl[2]);   // motion not allowed echo
  TEST_ASSERT_EQUAL_UINT8(motiongait::kStand, m.intent().gait);
  TEST_ASSERT_EQUAL_UINT32(0, m.intent().seq);
}

void test_set_gait_params_clamps() {
  MotionApi m;
  m.reset();
  uint8_t req[8];
  putU16(&req[0], 500);  // body height -> clamp 120
  putU16(&req[2], 999);  // stride -> clamp 80
  putU16(&req[4], 999);  // step -> clamp 50
  req[6] = 140;          // duty
  req[7] = 200;          // speed
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMotion(m, motionmsg::kSetGaitParams, 3, req, sizeof(req), &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotionResult::Ok), pl[0]);
  TEST_ASSERT_EQUAL_UINT16(120, m.intent().body_height_mm);
  TEST_ASSERT_EQUAL_UINT16(80, m.intent().stride_len_mm);
  TEST_ASSERT_EQUAL_UINT16(50, m.intent().step_height_mm);
  TEST_ASSERT_EQUAL_UINT8(140, m.intent().duty_x255);
  TEST_ASSERT_EQUAL_UINT8(200, m.intent().speed_x255);
}

void test_set_body_twist_decodes_and_clamps() {
  MotionApi m;
  m.reset();
  uint8_t req[6];
  putI16(&req[0], 500);    // vx -> 0.5
  putI16(&req[2], -250);   // vy -> -0.25
  putI16(&req[4], 5000);   // wz -> clamp 1.0
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMotion(m, motionmsg::kSetBodyTwist, 4, req, sizeof(req), &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotionResult::Ok), pl[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, m.intent().twist_vx);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.25f, m.intent().twist_vy);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, m.intent().twist_wz);
}

void test_set_body_pose_decodes_and_clamps() {
  MotionApi m;
  m.reset();
  uint8_t req[12];
  putI16(&req[0], 10);      // x mm
  putI16(&req[2], -20);     // y mm
  putI16(&req[4], 999);     // z mm -> clamp 50
  putI16(&req[6], 5000);    // roll 5 deg
  putI16(&req[8], -5000);   // pitch -5 deg
  putI16(&req[10], 30000);  // yaw 30 deg -> clamp ~25 deg
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMotion(m, motionmsg::kSetBodyPose, 5, req, sizeof(req), &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotionResult::Ok), pl[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, m.intent().pose_x_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -20.0f, m.intent().pose_y_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, m.intent().pose_z_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0872665f, m.intent().pose_roll);   // 5 deg
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.0872665f, m.intent().pose_pitch);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, motionlim::kMaxPoseRotRad, m.intent().pose_yaw);
}

void test_stop_motion_zeros_twist_and_holds_stand() {
  MotionApi m;
  m.reset();
  // First command a moving tripod gait with twist.
  uint8_t tw[6];
  putI16(&tw[0], 800);
  putI16(&tw[2], 0);
  putI16(&tw[4], 0);
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  const uint8_t g = motiongait::kTripod;
  runMotion(m, motionmsg::kSetGait, 6, &g, 1, &h, pl, &n);
  runMotion(m, motionmsg::kSetBodyTwist, 7, tw, sizeof(tw), &h, pl, &n);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.8f, m.intent().twist_vx);
  // STOP_MOTION must zero twist and force Stand.
  runMotion(m, motionmsg::kStopMotion, 8, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotionResult::Ok), pl[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, m.intent().twist_vx);
  TEST_ASSERT_EQUAL_UINT8(motiongait::kStand, m.intent().gait);
}

void test_bad_request_on_short_payload() {
  MotionApi m;
  m.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  // SET_GAIT_PARAMS needs 8 bytes; give 3.
  uint8_t shrt[3] = {1, 2, 3};
  runMotion(m, motionmsg::kSetGaitParams, 9, shrt, sizeof(shrt), &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotionResult::BadRequest), pl[0]);
  TEST_ASSERT_TRUE(h.flags & api::flag::kError);
  TEST_ASSERT_EQUAL_UINT32(0, m.intent().seq);
}

void test_motion_rejected_in_passive_pose_stream() {
  // While the live state is PassivePoseStream (9), gait/twist/pose commands are
  // rejected (torque-off passive mode); STOP_MOTION stays honoured.
  MotionApi m;
  m.reset();
  m.setLiveState(motionstate::kPassivePoseStream, false);
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  const uint8_t g = motiongait::kTripod;
  runMotion(m, motionmsg::kSetGait, 20, &g, 1, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotionResult::Rejected), pl[0]);
  TEST_ASSERT_EQUAL_UINT32(0, m.intent().seq);  // intent untouched
  // STOP_MOTION is always honoured, even in passive streaming.
  runMotion(m, motionmsg::kStopMotion, 21, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotionResult::Ok), pl[0]);
}

void test_unknown_without_motion_is_error() {
  // Dispatcher with no MotionApi must answer motion ids with UnknownMsg.
  uint8_t req[kMaxWireFrame];
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = motionmsg::kSetGait;
  h.seq = 10;
  h.payload_len = 0;
  const size_t req_n = encodeFrame(h, nullptr, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(req + 1, req_n - 2, info, st, resp,
                                           sizeof(resp));
  TEST_ASSERT_TRUE(resp_n > 0);
  Header rh;
  uint8_t pl[kMaxPayload];
  size_t pn = 0;
  decodeFrameBody(resp + 1, resp_n - 2, &rh, pl, kMaxPayload, &pn);
  TEST_ASSERT_TRUE(rh.flags & api::flag::kError);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(api::Error::UnknownMsg), pl[0]);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_set_gait_selects_and_bumps_seq);
  RUN_TEST(test_set_gait_rejects_out_of_range);
  RUN_TEST(test_set_gait_params_clamps);
  RUN_TEST(test_set_body_twist_decodes_and_clamps);
  RUN_TEST(test_set_body_pose_decodes_and_clamps);
  RUN_TEST(test_stop_motion_zeros_twist_and_holds_stand);
  RUN_TEST(test_motion_rejected_in_passive_pose_stream);
  RUN_TEST(test_bad_request_on_short_payload);
  RUN_TEST(test_unknown_without_motion_is_error);
  return UNITY_END();
}
