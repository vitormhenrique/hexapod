// Native (host) Unity tests for the controller command group
// (CONTROLLER_GET_STATE / GET_BINDINGS / SET_BINDINGS) handled by ControllerApi
// and routed through the protocol api dispatcher, plus the portable state /
// bindings codecs that back the controller_state telemetry stream (oha.4).
//
// Run with:  pio test -e native -f test_controller_api

#include <string.h>

#include <unity.h>

#include "input/controller_bridge.h"
#include "protocol/api.h"
#include "protocol/controller_api.h"
#include "protocol/framing.h"

using namespace protocol;

namespace {

api::DeviceInfo makeInfo() {
  api::DeviceInfo info;
  info.fw_major = 0;
  info.fw_minor = 1;
  info.fw_patch = 0;
  info.feature_bits = 0;
  memset(info.device_name, 0, sizeof(info.device_name));
  const char name[] = "OpenRB150-Hex";
  memcpy(info.device_name, name, sizeof(name) - 1);
  return info;
}

api::StatusSnapshot makeStatus(uint8_t state = 3) {
  api::StatusSnapshot st;
  st.uptime_ms = 1000;
  st.state = state;
  st.battery_mv = 11800;
  return st;
}

// Build + dispatch one controller request through the api, decode the response.
DecodeStatus runController(ControllerApi& ctl, uint8_t msg_id,
                           const uint8_t* payload, uint16_t payload_len,
                           Header* resp_h, uint8_t* resp_payload,
                           size_t* resp_len) {
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = msg_id;
  h.seq = 7;
  h.payload_len = payload_len;
  uint8_t req[kMaxWireFrame];
  const size_t req_n = encodeFrame(h, payload, req, sizeof(req));

  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(
      req + 1, req_n - 2, info, st, resp, sizeof(resp), nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      &ctl);
  TEST_ASSERT_TRUE(resp_n > 0);
  return decodeFrameBody(resp + 1, resp_n - 2, resp_h, resp_payload,
                         kMaxPayload, resp_len);
}

controller::ControllerCommand makeCommand() {
  controller::ControllerCommand c;
  c.valid = true;
  c.failsafe = false;
  c.ever_seen = true;
  c.arm_request = true;
  c.estop = false;
  c.host_authority = true;
  c.feat_foot_contact = true;
  c.feat_terrain_leveling = false;
  c.feat_passive_pose = true;
  c.mode = controller::ControlMode::RotateBody;  // 2
  c.gait_index = 1;
  c.trick = controller::TrickId::Wave;  // 3
  c.twist_vx = 0.5f;
  c.twist_vy = -0.25f;
  c.twist_wz = 1.0f;
  c.pose_x_mm = 10.0f;
  c.pose_y_mm = -20.0f;
  c.pose_z_mm = 5.0f;
  c.pose_roll = 0.1f;
  c.pose_pitch = -0.2f;
  c.pose_yaw = 0.0f;
  c.trim_roll = 0.05f;
  c.trim_pitch = -0.05f;
  c.speed = 1.0f;
  c.body_height = 0.5f;
  c.stride = 0.0f;
  c.step_height = 0.25f;
  return c;
}

ChannelPackInputs_t makeRaw() {
  ChannelPackInputs_t r{};
  r.gimbal[0] = 100;
  r.gimbal[1] = -200;
  r.gimbal[2] = 300;
  r.gimbal[3] = -400;
  r.pot[0] = 500;
  r.pot[1] = 600;
  r.encoder[0] = 1000;
  r.encoder[1] = -2000;
  r.switches[0] = true;
  r.switches[5] = true;
  r.buttons[1] = true;
  r.toggles[0] = 2;
  r.toggles[1] = 0;
  r.nav[0][0] = true;
  r.nav[0][4] = true;
  r.nav[1][2] = true;
  return r;
}

bool axisEqual(const controller::AxisBinding& a,
               const controller::AxisBinding& b) {
  if (a.source != b.source) return false;
  if (a.invert != b.invert) return false;
  const float d = a.deadband - b.deadband;
  return (d < 0.002f && d > -0.002f);
}

bool bindingsEqual(const controller::BindingConfig& a,
                   const controller::BindingConfig& b) {
  if (!axisEqual(a.walk_forward, b.walk_forward)) return false;
  if (!axisEqual(a.walk_strafe, b.walk_strafe)) return false;
  if (!axisEqual(a.walk_yaw, b.walk_yaw)) return false;
  if (!axisEqual(a.body_x, b.body_x)) return false;
  if (!axisEqual(a.body_y, b.body_y)) return false;
  if (!axisEqual(a.body_z, b.body_z)) return false;
  if (!axisEqual(a.body_roll, b.body_roll)) return false;
  if (!axisEqual(a.body_pitch, b.body_pitch)) return false;
  if (!axisEqual(a.body_yaw, b.body_yaw)) return false;
  if (!axisEqual(a.speed, b.speed)) return false;
  if (!axisEqual(a.body_height, b.body_height)) return false;
  if (!axisEqual(a.stride, b.stride)) return false;
  if (!axisEqual(a.step_height, b.step_height)) return false;
  if (a.mode_select != b.mode_select) return false;
  if (a.gait_select != b.gait_select) return false;
  if (a.arm != b.arm) return false;
  if (a.estop != b.estop) return false;
  if (a.feat_foot_contact != b.feat_foot_contact) return false;
  if (a.feat_terrain_leveling != b.feat_terrain_leveling) return false;
  if (a.feat_passive_pose != b.feat_passive_pose) return false;
  if (a.host_authority != b.host_authority) return false;
  if (a.trim_pitch_up != b.trim_pitch_up) return false;
  if (a.trim_pitch_down != b.trim_pitch_down) return false;
  if (a.trim_roll_left != b.trim_roll_left) return false;
  if (a.trim_roll_right != b.trim_roll_right) return false;
  if (a.trim_reset != b.trim_reset) return false;
  for (uint8_t i = 0; i < controller::kMaxTrickBindings; ++i) {
    if (a.tricks[i].source != b.tricks[i].source) return false;
    if (a.tricks[i].trick != b.tricks[i].trick) return false;
  }
  return true;
}

}  // namespace

void test_encode_state_golden_bytes() {
  const controller::ControllerCommand c = makeCommand();
  const ChannelPackInputs_t r = makeRaw();
  uint8_t buf[64];
  const uint16_t n = ControllerApi::encodeState(c, r, buf);
  TEST_ASSERT_EQUAL_UINT16(kControllerStateLen, n);
  TEST_ASSERT_EQUAL_UINT16(57, n);

  const uint8_t expect[57] = {
      0x6D, 0x01, 0x02, 0x01, 0x03,              // f1,f2,mode,gait,trick
      0xF4, 0x01, 0x06, 0xFF, 0xE8, 0x03,        // twist vx=500,vy=-250,wz=1000
      0x0A, 0x00, 0xEC, 0xFF, 0x05, 0x00,        // pose x=10,y=-20,z=5
      0x64, 0x00, 0x38, 0xFF, 0x00, 0x00,        // roll=100,pitch=-200,yaw=0
      0x32, 0x00, 0xCE, 0xFF,                    // trim roll=50,pitch=-50
      0xFF, 0x80, 0x00, 0x40,                    // speed,body_h,stride,step_h
      0x64, 0x00, 0x38, 0xFF, 0x2C, 0x01, 0x70, 0xFE,  // gimbal 100,-200,300,-400
      0xF4, 0x01, 0x58, 0x02,                    // pot 500,600
      0xE8, 0x03, 0x00, 0x00, 0x30, 0xF8, 0xFF, 0xFF,  // enc 1000,-2000
      0x21, 0x02, 0x02, 0x00, 0x11, 0x04};       // sw,btn,tog0,tog1,nav1,nav2
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, buf, 57);
}

void test_get_state_returns_encoded_snapshot() {
  ControllerApi ctl;
  ctl.setSnapshot(makeCommand(), makeRaw());

  Header rh;
  uint8_t pl[kMaxPayload];
  size_t pn = 0;
  const DecodeStatus ds = runController(ctl, controllermsg::kGetState, nullptr,
                                        0, &rh, pl, &pn);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(DecodeStatus::Ok), static_cast<int>(ds));
  TEST_ASSERT_FALSE(rh.flags & api::flag::kError);
  TEST_ASSERT_EQUAL_size_t(kControllerStateLen, pn);

  uint8_t expect[64];
  const uint16_t n = ControllerApi::encodeState(makeCommand(), makeRaw(), expect);
  TEST_ASSERT_EQUAL_UINT16(kControllerStateLen, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, pl, kControllerStateLen);
}

void test_bindings_default_round_trips() {
  const controller::BindingConfig def = controller::defaultBindings();
  uint8_t buf[128];
  const uint16_t n = ControllerApi::encodeBindings(def, buf);
  TEST_ASSERT_EQUAL_UINT16(kControllerBindingsLen, n);
  TEST_ASSERT_EQUAL_UINT16(81, n);

  controller::BindingConfig out;
  TEST_ASSERT_TRUE(ControllerApi::decodeBindings(buf, n, &out));
  TEST_ASSERT_TRUE(bindingsEqual(def, out));
}

void test_get_bindings_matches_encode() {
  ControllerApi ctl;
  const controller::BindingConfig def = controller::defaultBindings();
  ctl.setBindings(def);

  Header rh;
  uint8_t pl[kMaxPayload];
  size_t pn = 0;
  runController(ctl, controllermsg::kGetBindings, nullptr, 0, &rh, pl, &pn);
  TEST_ASSERT_EQUAL_size_t(kControllerBindingsLen, pn);

  uint8_t expect[128];
  ControllerApi::encodeBindings(def, expect);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, pl, kControllerBindingsLen);
}

void test_set_bindings_applies_and_stages_pending() {
  ControllerApi ctl;
  // Build a non-default config (swap a couple of sources + invert an axis).
  controller::BindingConfig cfg = controller::defaultBindings();
  cfg.walk_forward = controller::AxisBinding(controller::AxisSource::GimbalRY,
                                             true, 0.1f);
  cfg.mode_select = controller::TriSource::SwF;
  cfg.arm = controller::BoolSource::Btn1;
  cfg.tricks[0] = controller::TrickBinding(controller::BoolSource::Nav2Center,
                                           controller::TrickId::DanceLoop);
  uint8_t payload[128];
  const uint16_t plen = ControllerApi::encodeBindings(cfg, payload);

  Header rh;
  uint8_t pl[kMaxPayload];
  size_t pn = 0;
  runController(ctl, controllermsg::kSetBindings, payload, plen, &rh, pl, &pn);
  TEST_ASSERT_FALSE(rh.flags & api::flag::kError);
  TEST_ASSERT_EQUAL_size_t(1, pn);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ControllerResult::Ok), pl[0]);

  // The staged config is handed off exactly once.
  controller::BindingConfig taken;
  TEST_ASSERT_TRUE(ctl.takePending(&taken));
  TEST_ASSERT_TRUE(bindingsEqual(cfg, taken));
  TEST_ASSERT_FALSE(ctl.takePending(&taken));

  // GET_BINDINGS now reflects the applied config.
  runController(ctl, controllermsg::kGetBindings, nullptr, 0, &rh, pl, &pn);
  uint8_t expect[128];
  ControllerApi::encodeBindings(cfg, expect);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, pl, kControllerBindingsLen);
}

void test_set_bindings_rejects_bad_length() {
  ControllerApi ctl;
  uint8_t payload[10] = {0};
  Header rh;
  uint8_t pl[kMaxPayload];
  size_t pn = 0;
  runController(ctl, controllermsg::kSetBindings, payload, sizeof(payload), &rh,
                pl, &pn);
  TEST_ASSERT_TRUE(rh.flags & api::flag::kError);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ControllerResult::BadRequest),
                          pl[0]);
  controller::BindingConfig taken;
  TEST_ASSERT_FALSE(ctl.takePending(&taken));
}

void test_set_bindings_rejects_out_of_range_source() {
  ControllerApi ctl;
  uint8_t payload[128];
  ControllerApi::encodeBindings(controller::defaultBindings(), payload);
  payload[0] = 99;  // walk_forward source out of AxisSource range
  Header rh;
  uint8_t pl[kMaxPayload];
  size_t pn = 0;
  runController(ctl, controllermsg::kSetBindings, payload, kControllerBindingsLen,
                &rh, pl, &pn);
  TEST_ASSERT_TRUE(rh.flags & api::flag::kError);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ControllerResult::BadRequest),
                          pl[0]);
}

void test_decode_bindings_rejects_bad_trick() {
  uint8_t buf[128];
  ControllerApi::encodeBindings(controller::defaultBindings(), buf);
  // Last byte is tricks[7].trick; push it out of TrickId range.
  buf[kControllerBindingsLen - 1] = 200;
  controller::BindingConfig out;
  TEST_ASSERT_FALSE(
      ControllerApi::decodeBindings(buf, kControllerBindingsLen, &out));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_encode_state_golden_bytes);
  RUN_TEST(test_get_state_returns_encoded_snapshot);
  RUN_TEST(test_bindings_default_round_trips);
  RUN_TEST(test_get_bindings_matches_encode);
  RUN_TEST(test_set_bindings_applies_and_stages_pending);
  RUN_TEST(test_set_bindings_rejects_bad_length);
  RUN_TEST(test_set_bindings_rejects_out_of_range_source);
  RUN_TEST(test_decode_bindings_rejects_bad_trick);
  return UNITY_END();
}
