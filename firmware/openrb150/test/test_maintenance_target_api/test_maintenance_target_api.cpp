// Native (host) Unity tests for the maintenance leg/joint target command group
// (SET_LEG_TARGET / SET_JOINT_TARGET) handled by MaintTargetApi and routed
// through the protocol api dispatcher.
//
// The tests cross-check the handler against the same libraries it uses
// (gait::BodyKinematics + dxl::ServoMap on the default config) so they validate
// the wiring without re-deriving the kinematics.
//
// Run with:  pio test -e native -f test_maintenance_target_api

#include <math.h>
#include <string.h>

#include <unity.h>

#include "config/config_schema.h"
#include "dxl/servo_map.h"
#include "gait/body_ik.h"
#include "protocol/api.h"
#include "protocol/framing.h"
#include "protocol/maintenance_target_api.h"

using namespace protocol;

namespace {

constexpr uint8_t kMacMaintenance = 8;
constexpr uint8_t kDisarmed = 2;

api::DeviceInfo makeInfo() {
  api::DeviceInfo info;
  info.fw_major = 0;
  info.fw_minor = 1;
  info.fw_patch = 0;
  info.feature_bits = 0;
  memset(info.device_name, 0, sizeof(info.device_name));
  return info;
}

api::StatusSnapshot makeStatus() {
  api::StatusSnapshot st;
  st.uptime_ms = 1000;
  st.state = kMacMaintenance;
  st.battery_mv = 11800;
  return st;
}

void putI16(uint8_t* p, int16_t v) {
  p[0] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);
  p[1] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
}
uint16_t readU16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
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

DecodeStatus runTarget(MaintTargetApi& api_obj, uint8_t msg_id,
                       const uint8_t* payload, uint16_t payload_len,
                       Header* resp_h, uint8_t* resp_payload, size_t* resp_len) {
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(msg_id, 1, payload, payload_len, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(
      req + 1, req_n - 2, info, st, resp, sizeof(resp), nullptr, nullptr,
      nullptr, nullptr, nullptr, &api_obj);
  TEST_ASSERT_TRUE(resp_n > 0);
  return decodeFrameBody(resp + 1, resp_n - 2, resp_h, resp_payload,
                         kMaxPayload, resp_len);
}

// Body-frame coordinates (rounded to int16 mm) of leg `leg`'s home foot, which
// the IK maps to all-zero joint angles. Derived by inverting footBodyToCoxa for
// the documented coxa-frame home (127, 0, -44.55 mm).
void homeBodyTarget(const config::RobotConfig& cfg, uint8_t leg, int16_t& bx,
                    int16_t& by, int16_t& bz) {
  const config::LegGeometry& g = cfg.legs[leg];
  const float hip_x = g.mount_x_dmm / 10.0f;
  const float hip_y = g.mount_y_dmm / 10.0f;
  const float z_off = g.mount_z_dmm / 10.0f + gait::kCoxaLiftMm;
  const float yaw = g.mount_yaw_cdeg * (3.14159265358979323846f / 180.0f / 100.0f);
  const float a = -(yaw + 3.14159265358979323846f / 2.0f);
  const float ca = cosf(a), sa = sinf(a);
  const float cx = gait::kHomeRadiusMm, cy = 0.0f, cz = gait::kHomeFootZMm;
  // Inverse rotation: [dx;dy] = R^T [cx;cy].
  const float dx = ca * cx + sa * cy;
  const float dy = -sa * cx + ca * cy;
  bx = static_cast<int16_t>(lroundf(hip_x + dx));
  by = static_cast<int16_t>(lroundf(hip_y + dy));
  bz = static_cast<int16_t>(lroundf(cz + z_off));
}

}  // namespace

void test_leg_target_reachable_matches_library_and_stores() {
  config::RobotConfig cfg;
  config::defaultRobotConfig(cfg);
  MaintTargetApi api_obj;
  api_obj.reset();
  api_obj.setConfig(&cfg);
  api_obj.setLiveState(kMacMaintenance, true);

  const uint8_t leg = 0;
  int16_t bx, by, bz;
  homeBodyTarget(cfg, leg, bx, by, bz);

  gait::BodyKinematics bk(cfg);
  const gait::IkResult ik = bk.solveBody(
      leg, static_cast<float>(bx), static_cast<float>(by),
      static_cast<float>(bz));
  TEST_ASSERT_TRUE(ik.reachable);
  dxl::ServoMap sm(cfg);
  const float ang[3] = {ik.coxa, ik.femur, ik.tibia};
  uint16_t want[3];
  for (uint8_t j = 0; j < 3; ++j) want[j] = sm.angleToTick(leg, j, ang[j]).tick;

  uint8_t pl[7];
  pl[0] = leg;
  putI16(&pl[1], bx);
  putI16(&pl[3], by);
  putI16(&pl[5], bz);
  Header h;
  uint8_t rp[kMaxPayload];
  size_t n = 0;
  runTarget(api_obj, mainttargetmsg::kSetLegTarget, pl, 7, &h, rp, &n);

  TEST_ASSERT_EQUAL_UINT(11, n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintTargetResult::Ok), rp[0]);
  TEST_ASSERT_EQUAL_UINT8(kMacMaintenance, rp[1]);
  TEST_ASSERT_EQUAL_UINT8(1, rp[2]);  // reachable
  TEST_ASSERT_EQUAL_UINT16(want[0], readU16(&rp[5]));
  TEST_ASSERT_EQUAL_UINT16(want[1], readU16(&rp[7]));
  TEST_ASSERT_EQUAL_UINT16(want[2], readU16(&rp[9]));

  // Stored under the (leg, joint) slots.
  const MaintTargetSet& t = api_obj.target();
  TEST_ASSERT_EQUAL_UINT32(1, t.seq);
  for (uint8_t j = 0; j < 3; ++j) {
    TEST_ASSERT_TRUE(t.set[leg][j]);
    TEST_ASSERT_EQUAL_UINT16(want[j], t.tick[leg][j]);
  }
}

void test_leg_target_unreachable_reports_and_does_not_store() {
  config::RobotConfig cfg;
  config::defaultRobotConfig(cfg);
  MaintTargetApi api_obj;
  api_obj.reset();
  api_obj.setConfig(&cfg);
  api_obj.setLiveState(kMacMaintenance, true);

  // Way outside the two-link annulus.
  const uint8_t leg = 1;
  uint8_t pl[7];
  pl[0] = leg;
  putI16(&pl[1], 2000);
  putI16(&pl[3], 2000);
  putI16(&pl[5], 0);
  Header h;
  uint8_t rp[kMaxPayload];
  size_t n = 0;
  runTarget(api_obj, mainttargetmsg::kSetLegTarget, pl, 7, &h, rp, &n);

  TEST_ASSERT_EQUAL_UINT(11, n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintTargetResult::Unreachable),
                          rp[0]);
  TEST_ASSERT_EQUAL_UINT8(0, rp[2]);  // not reachable
  const MaintTargetSet& t = api_obj.target();
  TEST_ASSERT_EQUAL_UINT32(0, t.seq);
  TEST_ASSERT_FALSE(t.set[leg][0]);
}

void test_joint_target_matches_library_and_stores() {
  config::RobotConfig cfg;
  config::defaultRobotConfig(cfg);
  MaintTargetApi api_obj;
  api_obj.reset();
  api_obj.setConfig(&cfg);
  api_obj.setLiveState(kMacMaintenance, true);

  const uint8_t leg = 2, joint = 1;
  const int16_t cdeg = 3000;  // 30.00 deg, within +/-90 travel
  dxl::ServoMap sm(cfg);
  const float ang = cdeg * (3.14159265358979323846f / 180.0f / 100.0f);
  const dxl::JointCommand want = sm.angleToTick(leg, joint, ang);
  TEST_ASSERT_FALSE(want.clamped_low);
  TEST_ASSERT_FALSE(want.clamped_high);

  uint8_t pl[4];
  pl[0] = leg;
  pl[1] = joint;
  putI16(&pl[2], cdeg);
  Header h;
  uint8_t rp[kMaxPayload];
  size_t n = 0;
  runTarget(api_obj, mainttargetmsg::kSetJointTarget, pl, 4, &h, rp, &n);

  TEST_ASSERT_EQUAL_UINT(6, n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintTargetResult::Ok), rp[0]);
  TEST_ASSERT_EQUAL_UINT8(0, rp[2]);  // clamp_low
  TEST_ASSERT_EQUAL_UINT8(0, rp[3]);  // clamp_high
  TEST_ASSERT_EQUAL_UINT16(want.tick, readU16(&rp[4]));

  const MaintTargetSet& t = api_obj.target();
  TEST_ASSERT_TRUE(t.set[leg][joint]);
  TEST_ASSERT_EQUAL_UINT16(want.tick, t.tick[leg][joint]);
  TEST_ASSERT_FALSE(t.clamped[leg][joint]);  // within travel -> not clamped
}

void test_joint_target_clamps_high() {
  config::RobotConfig cfg;
  config::defaultRobotConfig(cfg);
  MaintTargetApi api_obj;
  api_obj.reset();
  api_obj.setConfig(&cfg);
  api_obj.setLiveState(kMacMaintenance, true);

  // 120 deg exceeds the default +/-90 deg travel -> clamp at max_tick (3072).
  uint8_t pl[4];
  pl[0] = 0;
  pl[1] = 0;
  putI16(&pl[2], 12000);
  Header h;
  uint8_t rp[kMaxPayload];
  size_t n = 0;
  runTarget(api_obj, mainttargetmsg::kSetJointTarget, pl, 4, &h, rp, &n);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintTargetResult::Ok), rp[0]);
  // One of the two clamp directions must fire (sign depends on the leg).
  TEST_ASSERT_TRUE(rp[2] != 0 || rp[3] != 0);
  const uint16_t tick = readU16(&rp[4]);
  TEST_ASSERT_TRUE(tick == 1024 || tick == 3072);
  // The saturated goal is flagged in the stored set so servo_goals can show it.
  const MaintTargetSet& t = api_obj.target();
  TEST_ASSERT_TRUE(t.clamped[0][0]);
}

void test_rejected_when_not_in_maintenance() {
  config::RobotConfig cfg;
  config::defaultRobotConfig(cfg);
  MaintTargetApi api_obj;
  api_obj.reset();
  api_obj.setConfig(&cfg);
  api_obj.setLiveState(kDisarmed, true);  // wrong state

  uint8_t pl[4] = {0, 0, 0, 0};
  Header h;
  uint8_t rp[kMaxPayload];
  size_t n = 0;
  runTarget(api_obj, mainttargetmsg::kSetJointTarget, pl, 4, &h, rp, &n);
  TEST_ASSERT_EQUAL_UINT(2, n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintTargetResult::Rejected),
                          rp[0]);
}

void test_rejected_when_lock_not_held() {
  config::RobotConfig cfg;
  config::defaultRobotConfig(cfg);
  MaintTargetApi api_obj;
  api_obj.reset();
  api_obj.setConfig(&cfg);
  api_obj.setLiveState(kMacMaintenance, false);  // no lock

  uint8_t pl[4] = {0, 0, 0, 0};
  Header h;
  uint8_t rp[kMaxPayload];
  size_t n = 0;
  runTarget(api_obj, mainttargetmsg::kSetJointTarget, pl, 4, &h, rp, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintTargetResult::Rejected),
                          rp[0]);
}

void test_rejected_when_no_config() {
  MaintTargetApi api_obj;
  api_obj.reset();  // cfg_ stays null
  api_obj.setLiveState(kMacMaintenance, true);

  uint8_t pl[4] = {0, 0, 0, 0};
  Header h;
  uint8_t rp[kMaxPayload];
  size_t n = 0;
  runTarget(api_obj, mainttargetmsg::kSetJointTarget, pl, 4, &h, rp, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintTargetResult::Rejected),
                          rp[0]);
}

void test_bad_request_short_and_bad_index() {
  config::RobotConfig cfg;
  config::defaultRobotConfig(cfg);
  MaintTargetApi api_obj;
  api_obj.reset();
  api_obj.setConfig(&cfg);
  api_obj.setLiveState(kMacMaintenance, true);

  // Short leg-target payload.
  uint8_t shrt[3] = {0, 0, 0};
  Header h;
  uint8_t rp[kMaxPayload];
  size_t n = 0;
  runTarget(api_obj, mainttargetmsg::kSetLegTarget, shrt, 3, &h, rp, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintTargetResult::BadRequest),
                          rp[0]);
  TEST_ASSERT_TRUE(h.flags & api::flag::kError);

  // Out-of-range leg index on a joint target.
  uint8_t bad[4] = {99, 0, 0, 0};
  runTarget(api_obj, mainttargetmsg::kSetJointTarget, bad, 4, &h, rp, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintTargetResult::BadRequest),
                          rp[0]);
}

void test_unknown_target_without_handler_is_error() {
  uint8_t req[kMaxWireFrame];
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = mainttargetmsg::kSetLegTarget;
  h.seq = 9;
  h.payload_len = 0;
  const size_t req_n = encodeFrame(h, nullptr, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n =
      api::handleRequest(req + 1, req_n - 2, info, st, resp, sizeof(resp));
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
  RUN_TEST(test_leg_target_reachable_matches_library_and_stores);
  RUN_TEST(test_leg_target_unreachable_reports_and_does_not_store);
  RUN_TEST(test_joint_target_matches_library_and_stores);
  RUN_TEST(test_joint_target_clamps_high);
  RUN_TEST(test_rejected_when_not_in_maintenance);
  RUN_TEST(test_rejected_when_lock_not_held);
  RUN_TEST(test_rejected_when_no_config);
  RUN_TEST(test_bad_request_short_and_bad_index);
  RUN_TEST(test_unknown_target_without_handler_is_error);
  return UNITY_END();
}
