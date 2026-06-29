// Native (host) Unity tests for the ChannelPack controller bridge (oha.2).
//
// Drives the bridge exactly as the receiver will: build a ChannelPackInputs_t,
// pack it into a 16-channel CRSF frame with the vendored ChannelPack packer,
// and feed the raw ticks to ControllerBridge::update(). This proves the bridge
// decodes the real wire layout, the table-driven mapping, all three control
// modes, shape params, tricks, trim, and failsafe.
//
// Run with:  pio test -e native -f test_controller_bridge

#include <unity.h>

#include "input/controller_bridge.h"

using namespace controller;

namespace {

ChannelPackInputs_t makeNeutral() {
  ChannelPackInputs_t in;
  for (int i = 0; i < 4; ++i) in.gimbal[i] = 0;
  in.pot[0] = in.pot[1] = 0;
  in.encoder[0] = in.encoder[1] = 0;
  for (int i = 0; i < 8; ++i) in.switches[i] = false;
  for (int i = 0; i < 4; ++i) in.buttons[i] = false;
  in.toggles[0] = in.toggles[1] = 0;
  for (int s = 0; s < 2; ++s)
    for (int d = 0; d < 5; ++d) in.nav[s][d] = false;
  return in;
}

const ControllerCommand& feed(ControllerBridge& b, const ChannelPackInputs_t& in,
                              uint32_t t, bool link = true) {
  uint16_t ch[CPACK_NUM_CHANNELS];
  ChannelPack::packInputs(&in, ch);
  return b.update(ch, link, t);
}

// --- modes / twist ---------------------------------------------------------

void test_walk_mode_twist_from_default_bindings() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 0;       // SwE UP -> Walk
  in.gimbal[1] = 1000;     // LY full forward -> walk_forward
  in.gimbal[0] = -1000;    // LX full left    -> walk_yaw
  in.gimbal[2] = 500;      // RX half         -> walk_strafe
  const ControllerCommand& c = feed(b, in, 100);
  TEST_ASSERT_TRUE(c.valid);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::Walk),
                         static_cast<uint8_t>(c.mode));
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, c.twist_vx);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, -1.0f, c.twist_wz);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.5f, c.twist_vy);
  // No body pose in walk mode.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.pose_x_mm);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.pose_roll);
}

void test_translate_body_mode_planted_feet() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 1;     // SwE CENTER -> TranslateBody
  in.gimbal[3] = 1000;   // RY -> body_x
  in.gimbal[2] = -1000;  // RX -> body_y
  in.gimbal[1] = 500;    // LY -> body_z
  const ControllerCommand& c = feed(b, in, 100);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::TranslateBody),
                         static_cast<uint8_t>(c.mode));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, poselim::kMaxTransMm, c.pose_x_mm);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -poselim::kMaxTransMm, c.pose_y_mm);
  TEST_ASSERT_FLOAT_WITHIN(2.0f, poselim::kMaxTransMm * 0.5f, c.pose_z_mm);
  // Feet planted: no twist while shifting the core.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.twist_vx);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.twist_vy);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.twist_wz);
}

void test_rotate_body_mode_clamped_to_envelope() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 2;    // SwE DOWN -> RotateBody
  in.gimbal[2] = 1000;  // RX -> roll
  in.gimbal[3] = 1000;  // RY -> pitch
  in.gimbal[0] = 1000;  // LX -> yaw
  const ControllerCommand& c = feed(b, in, 100);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::RotateBody),
                         static_cast<uint8_t>(c.mode));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, poselim::kMaxRotRad, c.pose_roll);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, poselim::kMaxRotRad, c.pose_pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, poselim::kMaxRotRad, c.pose_yaw);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.twist_wz);
}

void test_gait_index_from_select_toggle() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[1] = 0;
  TEST_ASSERT_EQUAL_UINT(0, feed(b, in, 10).gait_index);
  in.toggles[1] = 1;
  TEST_ASSERT_EQUAL_UINT(1, feed(b, in, 20).gait_index);
  in.toggles[1] = 2;
  TEST_ASSERT_EQUAL_UINT(2, feed(b, in, 30).gait_index);
}

// --- safety ----------------------------------------------------------------

void test_arm_switch_requires_no_kill() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.switches[0] = true;  // SwA arm
  const ControllerCommand& c = feed(b, in, 10);
  TEST_ASSERT_TRUE(c.arm_request);
  TEST_ASSERT_FALSE(c.estop);
}

void test_kill_switch_forces_estop_and_disarm() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.switches[0] = true;  // SwA arm
  in.switches[1] = true;  // SwB kill
  const ControllerCommand& c = feed(b, in, 10);
  TEST_ASSERT_TRUE(c.estop);
  TEST_ASSERT_FALSE(c.arm_request);
}

void test_failsafe_on_link_down() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.switches[0] = true;
  in.gimbal[1] = 1000;
  feed(b, in, 10);                            // good frame first
  const ControllerCommand& c = feed(b, in, 20, /*link=*/false);
  TEST_ASSERT_FALSE(c.valid);
  TEST_ASSERT_TRUE(c.failsafe);
  TEST_ASSERT_TRUE(c.estop);
  TEST_ASSERT_FALSE(c.arm_request);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.twist_vx);
  TEST_ASSERT_TRUE(c.ever_seen);  // remembers it has seen the link
}

void test_failsafe_on_stale_timeout() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  feed(b, in, 1000);
  b.evaluateFailsafe(1100, kDefaultFailsafeMs);  // within window
  TEST_ASSERT_FALSE(b.command().failsafe);
  b.evaluateFailsafe(1400, kDefaultFailsafeMs);  // > 250 ms stale
  TEST_ASSERT_TRUE(b.command().failsafe);
}

// --- shape params ----------------------------------------------------------

void test_shape_params_from_pots() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.pot[0] = 750;  // speed
  in.pot[1] = 250;  // body height
  const ControllerCommand& c = feed(b, in, 10);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.75f, c.speed);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.25f, c.body_height);
}

void test_encoder_integrates_stride_trim() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.encoder[0] = 600;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, feed(b, in, 10).stride);  // seed = mid
  in.encoder[0] = 600 + 256;  // +256 counts = +0.25 of 1024 full-scale
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.75f, feed(b, in, 20).stride);
  in.encoder[0] = 600;  // back down -256 -> 0.5
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, feed(b, in, 30).stride);
}

void test_encoder_wrap_is_shortest_delta() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.encoder[0] = 10;
  feed(b, in, 10);
  // Wrap 10 -> 2046 is a -12 step, not +2036.
  in.encoder[0] = 2046;
  const ControllerCommand& c = feed(b, in, 20);
  // 0.5 + (-12/1024) ~= 0.488
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f - 12.0f / 1024.0f, c.stride);
}

// --- features --------------------------------------------------------------

void test_feature_toggle_levels() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.switches[2] = true;  // SwC foot contact
  in.switches[4] = true;  // SwG passive pose
  const ControllerCommand& c = feed(b, in, 10);
  TEST_ASSERT_TRUE(c.feat_foot_contact);
  TEST_ASSERT_FALSE(c.feat_terrain_leveling);
  TEST_ASSERT_TRUE(c.feat_passive_pose);
}

void test_host_authority_switch() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.switches[5] = true;  // SwH
  TEST_ASSERT_TRUE(feed(b, in, 10).host_authority);
}

// --- tricks ----------------------------------------------------------------

void test_trick_fires_once_per_press() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.buttons[0] = true;  // Btn1 -> StandUp
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::StandUp),
                         static_cast<uint8_t>(feed(b, in, 100).trick));
  // Held: no repeat.
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::None),
                         static_cast<uint8_t>(feed(b, in, 150).trick));
  // Release, then press again past the refractory window: fires again.
  in.buttons[0] = false;
  feed(b, in, 200);
  in.buttons[0] = true;
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::StandUp),
                         static_cast<uint8_t>(feed(b, in, 400).trick));
}

void test_trick_refractory_debounce() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.buttons[1] = true;  // Btn2 -> SitDown
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::SitDown),
                         static_cast<uint8_t>(feed(b, in, 100).trick));
  in.buttons[1] = false;
  feed(b, in, 120);
  in.buttons[1] = true;  // re-press only 30 ms later -> debounced
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::None),
                         static_cast<uint8_t>(feed(b, in, 130).trick));
}

void test_nav_cluster_trick_binding() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.nav[1][CPACK_NAV_UP] = true;  // NAV2 up -> Twirl
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::Twirl),
                         static_cast<uint8_t>(feed(b, in, 100).trick));
}

// --- pose trim -------------------------------------------------------------

void test_pose_trim_nudge_and_reset() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.nav[0][CPACK_NAV_UP] = true;  // NAV1 up -> pitch trim +
  const ControllerCommand& c1 = feed(b, in, 100);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, kTrimStepRad, c1.trim_pitch);
  // Hold does not keep adding (edge only).
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, kTrimStepRad, feed(b, in, 150).trim_pitch);
  // Release + press again -> second step.
  in.nav[0][CPACK_NAV_UP] = false;
  feed(b, in, 200);
  in.nav[0][CPACK_NAV_UP] = true;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f * kTrimStepRad, feed(b, in, 400).trim_pitch);
  // Reset zeroes the trim.
  in.nav[0][CPACK_NAV_UP] = false;
  in.nav[0][CPACK_NAV_CENTER] = true;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, feed(b, in, 600).trim_pitch);
}

// --- deadband / invert / remap --------------------------------------------

void test_deadband_kills_centre_jitter() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 0;    // Walk
  in.gimbal[1] = 20;    // 0.02 < 0.05 deadband
  TEST_ASSERT_EQUAL_FLOAT(0.0f, feed(b, in, 10).twist_vx);
}

void test_invert_flips_axis() {
  ControllerBridge b;
  BindingConfig cfg = defaultBindings();
  cfg.walk_forward.invert = true;
  b.setBindings(cfg);
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 0;
  in.gimbal[1] = 1000;  // forward, but inverted
  TEST_ASSERT_FLOAT_WITHIN(0.02f, -1.0f, feed(b, in, 10).twist_vx);
}

void test_setbindings_remaps_source() {
  ControllerBridge b;
  BindingConfig cfg = defaultBindings();
  cfg.walk_forward.source = AxisSource::GimbalRX;  // forward now on right X
  b.setBindings(cfg);
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 0;
  in.gimbal[1] = 1000;  // old source -> should be ignored now
  in.gimbal[2] = 1000;  // new source
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, feed(b, in, 10).twist_vx);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_walk_mode_twist_from_default_bindings);
  RUN_TEST(test_translate_body_mode_planted_feet);
  RUN_TEST(test_rotate_body_mode_clamped_to_envelope);
  RUN_TEST(test_gait_index_from_select_toggle);
  RUN_TEST(test_arm_switch_requires_no_kill);
  RUN_TEST(test_kill_switch_forces_estop_and_disarm);
  RUN_TEST(test_failsafe_on_link_down);
  RUN_TEST(test_failsafe_on_stale_timeout);
  RUN_TEST(test_shape_params_from_pots);
  RUN_TEST(test_encoder_integrates_stride_trim);
  RUN_TEST(test_encoder_wrap_is_shortest_delta);
  RUN_TEST(test_feature_toggle_levels);
  RUN_TEST(test_host_authority_switch);
  RUN_TEST(test_trick_fires_once_per_press);
  RUN_TEST(test_trick_refractory_debounce);
  RUN_TEST(test_nav_cluster_trick_binding);
  RUN_TEST(test_pose_trim_nudge_and_reset);
  RUN_TEST(test_deadband_kills_centre_jitter);
  RUN_TEST(test_invert_flips_axis);
  RUN_TEST(test_setbindings_remaps_source);
  return UNITY_END();
}
