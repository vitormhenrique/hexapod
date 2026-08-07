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

// Detection needs kProfileDetectFrames agreeing link-up frames before it locks
// a profile (glitch resistance), so a single logical frame would otherwise sit
// in failsafe. feedCh()/feed() prime the lock with detection-only frames (which
// do NOT decode inputs or advance encoder/edge state) so the returned frame is
// the one that locks + decodes -- letting the existing single-frame tests keep
// their exact semantics.
const ControllerCommand& feedCh(ControllerBridge& b,
                                const uint16_t ch[CPACK_NUM_CHANNELS], uint32_t t,
                                bool link = true) {
  if (link && !b.profileLocked()) {
    for (int i = 0; i < kProfileDetectFrames - 1; ++i) b.update(ch, true, t);
  }
  return b.update(ch, link, t);
}

const ControllerCommand& feed(ControllerBridge& b, const ChannelPackInputs_t& in,
                              uint32_t t, bool link = true) {
  uint16_t ch[CPACK_NUM_CHANNELS];
  ChannelPack::packInputs(&in, ch);
  return feedCh(b, ch, t, link);
}

// --- TX16S MK3 direct frame builders (conventional per-channel CRSF) --------

// Unit 0..1 -> conventional CRSF value in the 191..1792 span.
uint16_t crsfFromUnit(float u) {
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;
  return static_cast<uint16_t>(CPACK_CRSF_MIN +
                               u * (CPACK_CRSF_MAX - CPACK_CRSF_MIN));
}

// CRSF value that lands in the centre of discrete bin `idx` of `bins`.
uint16_t crsfForBin(uint8_t idx, uint8_t bins) {
  return crsfFromUnit((static_cast<float>(idx) + 0.5f) / static_cast<float>(bins));
}

// A neutral TX16S direct frame: sticks centred, knobs/sliders low, SE/SD
// centred, masks 0, action off, reserved centred. Classifies as Tx16sMk3Direct
// (CH9 at MID is not a valid packed custom switch field).
void makeTxNeutral(uint16_t ch[CPACK_NUM_CHANNELS]) {
  ch[0] = ch[1] = ch[2] = ch[3] = CPACK_CRSF_MID;  // sticks centred
  ch[4] = ch[5] = CPACK_CRSF_MIN;                  // S1/S2 knobs low
  ch[6] = ch[7] = CPACK_CRSF_MIN;                  // LS/RS sliders low
  ch[8] = CPACK_CRSF_MID;                          // SE mode -> CENTER
  ch[9] = CPACK_CRSF_MID;                          // SD gait -> CENTER
  ch[10] = CPACK_CRSF_MIN;                         // safety mask 0
  ch[11] = CPACK_CRSF_MIN;                         // feature mask 0
  ch[12] = CPACK_CRSF_MIN;                         // action selector 0
  ch[13] = CPACK_CRSF_MIN;                         // action fire off
  ch[14] = ch[15] = CPACK_CRSF_MID;                // MAX 0 reserved centre
}


// --- modes / twist ---------------------------------------------------------

void test_walk_mode_twist_from_default_bindings() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[1] = 0;       // SwF UP -> Yaw (right X steers)
  in.gimbal[1] = 1000;     // LY full forward -> walk_forward
  in.gimbal[0] = -1000;    // LX full left    -> walk_strafe (inverted)
  in.gimbal[2] = 500;      // RX half right   -> walk_yaw (inverted -> CW)
  const ControllerCommand& c = feed(b, in, 100);
  TEST_ASSERT_TRUE(c.valid);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::Yaw),
                         static_cast<uint8_t>(c.mode));
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, c.twist_vx);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, c.twist_vy);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, -0.5f, c.twist_wz);
  // No body pose in walk mode.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.pose_x_mm);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.pose_roll);
}

void test_translate_body_mode_pose_overlay_keeps_walking() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[1] = 1;     // SwF CENTER -> TranslateBody
  in.gimbal[3] = 1000;   // RY -> body_x
  in.gimbal[2] = -1000;  // RX -> body_y
  in.gimbal[1] = 500;    // LY -> walk_forward (left stick keeps walking)
  const ControllerCommand& c = feed(b, in, 100);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::TranslateBody),
                         static_cast<uint8_t>(c.mode));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, poselim::kMaxTransMm, c.pose_x_mm);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -poselim::kMaxTransMm, c.pose_y_mm);
  // body_z is unbound by default (height lives on the pot).
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.pose_z_mm);
  // Left gimbal still walks while the body shifts; right stick no longer
  // strafes in this mode.
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.5f, c.twist_vx);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.twist_vy);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.twist_wz);
}

void test_rotate_body_mode_clamped_to_envelope() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[1] = 2;    // SwF DOWN -> RotateBody
  in.gimbal[2] = 1000;  // RX -> roll
  in.gimbal[3] = 1000;  // RY -> pitch
  in.gimbal[0] = 1000;  // LX full right -> walk_strafe (inverted -> right)
  const ControllerCommand& c = feed(b, in, 100);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::RotateBody),
                         static_cast<uint8_t>(c.mode));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, poselim::kMaxRotRad, c.pose_roll);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, poselim::kMaxRotRad, c.pose_pitch);
  // body_yaw is unbound by default; left X remains planar walking.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.pose_yaw);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, -1.0f, c.twist_vy);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.twist_wz);
}

void test_gait_index_from_select_toggle() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 0;  // SwE UP -> Wave
  TEST_ASSERT_EQUAL_UINT(0, feed(b, in, 10).gait_index);
  in.toggles[0] = 1;  // SwE CENTER -> Ripple (after debounce)
  TEST_ASSERT_EQUAL_UINT(0, feed(b, in, 20).gait_index);
  TEST_ASSERT_EQUAL_UINT(1, feed(b, in, 60).gait_index);
  in.toggles[0] = 2;  // SwE DOWN -> Tripod
  TEST_ASSERT_EQUAL_UINT(1, feed(b, in, 70).gait_index);
  TEST_ASSERT_EQUAL_UINT(2, feed(b, in, 110).gait_index);
}

// The gait family is live in EVERY right-gimbal mode, so the operator can walk
// with a chosen gait while translating or rotating the body.
void test_gait_selection_is_live_in_every_mode() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[1] = 0;  // SwF UP -> Yaw
  in.toggles[0] = 2;  // SwE DOWN -> Tripod
  feed(b, in, 10);
  const ControllerCommand& walking = feed(b, in, 60);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::Yaw),
                         static_cast<uint8_t>(walking.mode));
  TEST_ASSERT_EQUAL_UINT(2, walking.gait_index);

  in.toggles[1] = 1;  // TranslateBody
  in.toggles[0] = 0;  // Wave: applies immediately, no latch
  feed(b, in, 70);
  const ControllerCommand& translating = feed(b, in, 120);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::TranslateBody),
                         static_cast<uint8_t>(translating.mode));
  TEST_ASSERT_EQUAL_UINT(0, translating.gait_index);

  in.toggles[0] = 1;  // Ripple while still translating
  feed(b, in, 130);
  const ControllerCommand& retuned = feed(b, in, 180);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::TranslateBody),
                         static_cast<uint8_t>(retuned.mode));
  TEST_ASSERT_EQUAL_UINT(1, retuned.gait_index);
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

void test_arm_switch_uses_guarded_pot_channel_band() {
  ChannelPackInputs_t in = makeNeutral();
  in.pot[0] = 500;
  uint16_t ch[CPACK_NUM_CHANNELS];

  ChannelPack::packInputs(&in, ch);
  TEST_ASSERT_TRUE(ch[CPACK_CH_POT1] < CPACK_CRSF_MID);
  ControllerBridge off_bridge;
  const ControllerCommand& off = feedCh(off_bridge, ch, 10);
  TEST_ASSERT_FALSE(off.arm_request);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, off.speed);

  in.switches[0] = true;
  ChannelPack::packInputs(&in, ch);
  TEST_ASSERT_EQUAL_UINT16(CPACK_CRSF_MIN, ch[CPACK_CH_SWITCHES]);
  TEST_ASSERT_TRUE(ch[CPACK_CH_POT1] > CPACK_CRSF_MID);
  --ch[CPACK_CH_POT1];
  ControllerBridge arm_bridge;
  const ControllerCommand& armed = feedCh(arm_bridge, ch, 10);
  TEST_ASSERT_TRUE(armed.arm_request);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, armed.speed);
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

void test_arm_release_rejects_short_glitch_but_accepts_stable_release() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.switches[0] = true;
  TEST_ASSERT_TRUE(feed(b, in, 10).arm_request);

  in.switches[0] = false;
  TEST_ASSERT_TRUE(feed(b, in, 20).arm_request);
  TEST_ASSERT_TRUE(feed(b, in, 20 + kArmReleaseDebounceMs - 1).arm_request);

  in.switches[0] = true;
  TEST_ASSERT_TRUE(feed(b, in, 30 + kArmReleaseDebounceMs).arm_request);
  in.switches[0] = false;
  TEST_ASSERT_TRUE(feed(b, in, 40 + kArmReleaseDebounceMs).arm_request);
  TEST_ASSERT_FALSE(
      feed(b, in, 40 + 2 * kArmReleaseDebounceMs).arm_request);
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

// The NAV1 cluster now owns stride / step height / duty, so the encoders are
// free: turning one must not move a gait parameter with the default bindings.
void test_encoders_no_longer_drive_gait_shape() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.encoder[0] = 600;
  in.encoder[1] = 600;
  const ControllerCommand& seed = feed(b, in, 10);
  const float stride = seed.stride;
  const float step = seed.step_height;
  in.encoder[0] = 664;  // +64 counts: would previously be +0.5 of full scale
  in.encoder[1] = 536;  // -64 counts
  const ControllerCommand& c = feed(b, in, 20);
  TEST_ASSERT_EQUAL_FLOAT(stride, c.stride);
  TEST_ASSERT_EQUAL_FLOAT(step, c.step_height);
}

// --- gait-tune editor (SW_G enters, NAV1 edits) -----------------------------

// Press a boolean input for one frame and release it on the next, advancing
// past the edge refractory window each time.
void pulse(ControllerBridge& b, ChannelPackInputs_t& in, bool& input,
           uint32_t& t) {
  input = true;
  t += kEdgeRefractoryMs + 10;
  feed(b, in, t);
  input = false;
  t += 10;
  feed(b, in, t);
}

void test_gait_tune_toggle_switches_nav1_from_trim_to_parameters() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  uint32_t t = 100;
  feed(b, in, t);
  TEST_ASSERT_FALSE(b.command().gait_tune_active);

  // NAV1 Up trims pitch while the editor is closed.
  pulse(b, in, in.nav[0][CPACK_NAV_UP], t);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, kTrimStepRad, b.command().trim_pitch);

  // SW_G ON engages the editor.
  in.switches[4] = true;
  t += 10;
  feed(b, in, t);
  TEST_ASSERT_TRUE(b.command().gait_tune_active);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(GaitTuneParam::StepHeight),
                         static_cast<uint8_t>(b.command().gait_tune_param));

  // The same NAV1 Up now selects the next parameter and leaves trim alone.
  const float trim_before = b.command().trim_pitch;
  pulse(b, in, in.nav[0][CPACK_NAV_UP], t);
  TEST_ASSERT_EQUAL_FLOAT(trim_before, b.command().trim_pitch);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(GaitTuneParam::Stride),
                         static_cast<uint8_t>(b.command().gait_tune_param));

  // SW_G OFF leaves the editor and restores trim control.
  in.switches[4] = false;
  t += 10;
  feed(b, in, t);
  TEST_ASSERT_FALSE(b.command().gait_tune_active);
  pulse(b, in, in.nav[0][CPACK_NAV_UP], t);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f * kTrimStepRad, b.command().trim_pitch);
}

void test_gait_tune_adjusts_selected_parameter_and_clamps() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  uint32_t t = 100;
  b.setGaitTuneFractions(0.5f, 0.5f, 0.5f);
  feed(b, in, t);
  in.switches[4] = true;  // engage, on StepHeight
  t += 10;
  feed(b, in, t);

  pulse(b, in, in.nav[0][CPACK_NAV_RIGHT], t);  // increase
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f + kGaitTuneStepFrac,
                           b.command().step_height);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, b.command().stride);

  pulse(b, in, in.nav[0][CPACK_NAV_LEFT], t);  // decrease back
  pulse(b, in, in.nav[0][CPACK_NAV_LEFT], t);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f - kGaitTuneStepFrac,
                           b.command().step_height);

  // Select stride and drive it to the top clamp.
  pulse(b, in, in.nav[0][CPACK_NAV_UP], t);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(GaitTuneParam::Stride),
                         static_cast<uint8_t>(b.command().gait_tune_param));
  for (int i = 0; i < 30; ++i) pulse(b, in, in.nav[0][CPACK_NAV_RIGHT], t);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, b.command().stride);
  for (int i = 0; i < 40; ++i) pulse(b, in, in.nav[0][CPACK_NAV_LEFT], t);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, b.command().stride);
}

void test_gait_tune_save_bumps_sequence_once_per_press() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  uint32_t t = 100;
  feed(b, in, t);
  in.switches[4] = true;
  t += 10;
  feed(b, in, t);
  const uint32_t before = b.command().gait_tune_save_seq;

  // Held across many frames: still exactly one save request.
  in.nav[0][CPACK_NAV_CENTER] = true;
  for (int i = 0; i < 5; ++i) {
    t += 10;
    feed(b, in, t);
  }
  TEST_ASSERT_EQUAL_UINT32(before + 1, b.command().gait_tune_save_seq);

  in.nav[0][CPACK_NAV_CENTER] = false;
  t += 10;
  feed(b, in, t);
  pulse(b, in, in.nav[0][CPACK_NAV_CENTER], t);
  TEST_ASSERT_EQUAL_UINT32(before + 2, b.command().gait_tune_save_seq);
}

void test_gait_tune_defaults_track_config_until_editor_opens() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  uint32_t t = 100;
  b.setGaitTuneFractions(0.2f, 0.4f, 0.6f);
  const ControllerCommand& seeded = feed(b, in, t);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, seeded.step_height);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, seeded.stride);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.6f, seeded.duty);

  in.switches[4] = true;  // editor engaged
  t += 10;
  feed(b, in, t);
  b.setGaitTuneFractions(0.9f, 0.9f, 0.9f);     // config revision lands
  t += 10;
  const ControllerCommand& held = feed(b, in, t);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, held.step_height);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, held.stride);
}


// --- features --------------------------------------------------------------

void test_feature_toggle_levels() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.switches[2] = true;  // SwC foot contact
  const ControllerCommand& c = feed(b, in, 10);
  TEST_ASSERT_TRUE(c.feat_foot_contact);
  TEST_ASSERT_FALSE(c.feat_terrain_leveling);
  TEST_ASSERT_FALSE(c.feat_passive_pose);
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
  struct Nav2Case {
    uint8_t direction;
    TrickId trick;
  };
  const Nav2Case cases[] = {
      {CPACK_NAV_UP, TrickId::Twirl},
      {CPACK_NAV_DOWN, TrickId::Stretch},
      {CPACK_NAV_LEFT, TrickId::JumpKick},
      {CPACK_NAV_RIGHT, TrickId::SpiderAttack},
      {CPACK_NAV_CENTER, TrickId::DanceLoop},
  };
  for (const Nav2Case& test_case : cases) {
    ControllerBridge b;
    ChannelPackInputs_t in = makeNeutral();
    in.nav[1][test_case.direction] = true;
    TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(test_case.trick),
                           static_cast<uint8_t>(feed(b, in, 100).trick));
  }
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

void test_asymmetric_gimbal_calibration_maps_center_and_endpoints() {
  ControllerBridge b;
  config::RcInputCalibration calibration;
  config::defaultRcInputCalibration(calibration);
  config::RcChannelCalibration& forward = calibration.channels[1];
  forward.min_raw = -800;
  forward.center_raw = 100;
  forward.max_raw = 700;
  forward.filter_tau_ms = 0;
  b.setCalibration(calibration);

  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 0;
  in.gimbal[1] = 100;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, feed(b, in, 10).twist_vx);
  in.gimbal[1] = 700;
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, feed(b, in, 20).twist_vx);
  in.gimbal[1] = -800;
  TEST_ASSERT_FLOAT_WITHIN(0.02f, -1.0f, feed(b, in, 30).twist_vx);
}

void test_calibration_reverse_and_invalid_input_rejection() {
  ControllerBridge b;
  config::RcInputCalibration calibration;
  config::defaultRcInputCalibration(calibration);
  calibration.channels[1].reversed = 1;
  calibration.channels[1].filter_tau_ms = 0;
  b.setCalibration(calibration);

  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 0;
  in.gimbal[1] = 500;
  TEST_ASSERT_FLOAT_WITHIN(0.03f, -0.5f, feed(b, in, 10).twist_vx);

  // The custom wire format clamps its logical gimbal range to +/-1000. Make
  // the accepted calibration narrower, then prove a valid wire value outside
  // that calibrated window is rejected rather than becoming full speed.
  calibration.channels[1].min_raw = -500;
  calibration.channels[1].max_raw = 500;
  b.setCalibration(calibration);
  in.gimbal[1] = 1000;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, feed(b, in, 20).twist_vx);
}

void test_unipolar_calibration_maps_pot_range() {
  ControllerBridge b;
  config::RcInputCalibration calibration;
  config::defaultRcInputCalibration(calibration);
  config::RcChannelCalibration& speed = calibration.channels[4];
  speed.min_raw = 200;
  speed.center_raw = 200;
  speed.max_raw = 800;
  speed.filter_tau_ms = 0;
  b.setCalibration(calibration);

  ChannelPackInputs_t in = makeNeutral();
  in.pot[0] = 200;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, feed(b, in, 10).speed);
  in.pot[0] = 500;
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, feed(b, in, 20).speed);
  in.pot[0] = 800;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, feed(b, in, 30).speed);
}

void test_default_ema_smooths_a_stick_step_after_initial_seed() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 0;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, feed(b, in, 10).twist_vx);

  in.gimbal[1] = 1000;
  const ControllerCommand& stepped = feed(b, in, 20);
  TEST_ASSERT_TRUE(stepped.twist_vx > 0.0f);
  TEST_ASSERT_TRUE(stepped.twist_vx < 0.2f);
}

void test_centered_noise_remains_zero_after_conditioning() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.toggles[0] = 0;
  feed(b, in, 10);
  const int16_t noise[] = {20, -30, 35, -40, 10, -15, 25, -20};
  for (uint8_t index = 0; index < sizeof(noise) / sizeof(noise[0]); ++index) {
    in.gimbal[1] = noise[index];
    const ControllerCommand& command = feed(b, in, 20 + index * 10);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.twist_vx);
  }
}

// --- input-profile detection & lock ----------------------------------------

void test_custom_profile_locks_after_stable_frames() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  uint16_t ch[CPACK_NUM_CHANNELS];
  ChannelPack::packInputs(&in, ch);
  // First two link-up frames only accumulate the streak -> failsafe hold.
  const ControllerCommand& c1 = b.update(ch, true, 10);
  TEST_ASSERT_FALSE(b.profileLocked());
  TEST_ASSERT_TRUE(c1.failsafe);
  TEST_ASSERT_FALSE(c1.valid);
  b.update(ch, true, 20);
  TEST_ASSERT_FALSE(b.profileLocked());
  // Third agreeing frame locks the custom profile and decodes.
  const ControllerCommand& c3 = b.update(ch, true, 30);
  TEST_ASSERT_TRUE(b.profileLocked());
  TEST_ASSERT_EQUAL_UINT(
      static_cast<uint8_t>(InputProfile::CustomControllerChannelPack),
      static_cast<uint8_t>(b.detectedProfile()));
  TEST_ASSERT_TRUE(c3.valid);
}

void test_custom_btn4_uses_four_buttons() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.buttons[3] = true;  // Btn4 -> CrouchToggle proves CH10 packs 4 buttons
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::CrouchToggle),
                         static_cast<uint8_t>(feed(b, in, 100).trick));
}

void test_custom_digital_fields_use_valid_crsf_range_and_round_trip() {
  ChannelPackInputs_t in = makeNeutral();
  for (int i = 0; i < 6; ++i) in.switches[i] = true;
  in.buttons[3] = true;
  in.toggles[0] = 2;
  in.toggles[1] = 2;
  in.nav[0][CPACK_NAV_LEFT] = true;
  in.nav[1][CPACK_NAV_CENTER] = true;

  uint16_t ch[CPACK_NUM_CHANNELS];
  ChannelPack::packInputs(&in, ch);
  for (int i = CPACK_CH_SWITCHES; i <= CPACK_CH_NAV; ++i) {
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(CPACK_CRSF_MIN, ch[i]);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(CPACK_CRSF_MAX, ch[i]);
  }

  ChannelPackInputs_t out = makeNeutral();
  ChannelPack::unpackChannels(ch, &out);
  for (int i = 0; i < 6; ++i) TEST_ASSERT_TRUE(out.switches[i]);
  for (int i = 0; i < 3; ++i) TEST_ASSERT_FALSE(out.buttons[i]);
  TEST_ASSERT_TRUE(out.buttons[3]);
  TEST_ASSERT_EQUAL_UINT8(2, out.toggles[0]);
  TEST_ASSERT_EQUAL_UINT8(2, out.toggles[1]);
  TEST_ASSERT_TRUE(out.nav[0][CPACK_NAV_LEFT]);
  TEST_ASSERT_TRUE(out.nav[1][CPACK_NAV_CENTER]);
}

void test_tx16s_profile_locks_after_stable_frames() {
  ControllerBridge b;
  uint16_t ch[CPACK_NUM_CHANNELS];
  makeTxNeutral(ch);
  b.update(ch, true, 10);
  TEST_ASSERT_FALSE(b.profileLocked());
  b.update(ch, true, 20);
  TEST_ASSERT_FALSE(b.profileLocked());
  const ControllerCommand& c3 = b.update(ch, true, 30);
  TEST_ASSERT_TRUE(b.profileLocked());
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(InputProfile::Tx16sMk3Direct),
                         static_cast<uint8_t>(b.detectedProfile()));
  TEST_ASSERT_TRUE(c3.valid);
}

// --- TX16S direct decode ---------------------------------------------------

void test_tx16s_gimbals_and_pots() {
  ControllerBridge b;
  uint16_t ch[CPACK_NUM_CHANNELS];
  makeTxNeutral(ch);
  ch[0] = CPACK_CRSF_MAX;   // Rud full -> gimbal LX +1000
  ch[1] = CPACK_CRSF_MIN;   // Thr low  -> gimbal LY -1000
  ch[4] = CPACK_CRSF_MAX;   // S1 full  -> pot0 1000 (speed 1.0)
  ch[5] = crsfFromUnit(0.25f);
  const ControllerCommand& c = feedCh(b, ch, 10);
  TEST_ASSERT_TRUE(c.valid);
  TEST_ASSERT_INT_WITHIN(5, 1000, b.rawInputs().gimbal[0]);
  TEST_ASSERT_INT_WITHIN(5, -1000, b.rawInputs().gimbal[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, c.speed);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.25f, c.body_height);
}

void test_tx16s_encoders_are_absolute() {
  ControllerBridge b;
  uint16_t ch[CPACK_NUM_CHANNELS];
  makeTxNeutral(ch);
  ch[6] = CPACK_CRSF_MAX;   // LS full -> stride 1.0
  ch[7] = CPACK_CRSF_MID;   // RS mid  -> step_height ~0.5
  const ControllerCommand& c = feedCh(b, ch, 10);
  TEST_ASSERT_INT_WITHIN(4, 2047, b.rawInputs().encoder[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, c.stride);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.5f, c.step_height);
  // Absolute, not integrated: feeding the same value again does not drift.
  const ControllerCommand& c2 = feedCh(b, ch, 20);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, c2.stride);
}

void test_tx16s_toggles_gait_and_mode() {
  ControllerBridge b;
  uint16_t ch[CPACK_NUM_CHANNELS];
  makeTxNeutral(ch);
  ch[8] = CPACK_CRSF_MAX;  // SE DOWN -> gait 2 (tripod)
  ch[9] = CPACK_CRSF_MIN;  // SD UP -> Yaw
  const ControllerCommand& c = feedCh(b, ch, 10);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(ControlMode::Yaw),
                         static_cast<uint8_t>(c.mode));
  TEST_ASSERT_EQUAL_UINT(2, c.gait_index);
}

void test_tx16s_safety_mask() {
  // mask 1 = armed, no estop.
  {
    ControllerBridge b;
    uint16_t ch[CPACK_NUM_CHANNELS];
    makeTxNeutral(ch);
    ch[10] = crsfForBin(1, 4);
    const ControllerCommand& c = feedCh(b, ch, 10);
    TEST_ASSERT_TRUE(c.arm_request);
    TEST_ASSERT_FALSE(c.estop);
  }
  // mask 2 = disarmed, estop.
  {
    ControllerBridge b;
    uint16_t ch[CPACK_NUM_CHANNELS];
    makeTxNeutral(ch);
    ch[10] = crsfForBin(2, 4);
    const ControllerCommand& c = feedCh(b, ch, 10);
    TEST_ASSERT_TRUE(c.estop);
    TEST_ASSERT_FALSE(c.arm_request);
  }
  // mask 3 (MAX endpoint) = armed + estop -> estop wins, disarmed.
  {
    ControllerBridge b;
    uint16_t ch[CPACK_NUM_CHANNELS];
    makeTxNeutral(ch);
    ch[10] = CPACK_CRSF_MAX;
    const ControllerCommand& c = feedCh(b, ch, 10);
    TEST_ASSERT_TRUE(c.estop);
    TEST_ASSERT_FALSE(c.arm_request);
  }
}

void test_tx16s_feature_mask() {
  // All four feature bits set (MAX endpoint -> bin 15): bit 2 is SW_G,
  // which now enters gait tuning rather than requesting passive pose.
  {
    ControllerBridge b;
    uint16_t ch[CPACK_NUM_CHANNELS];
    makeTxNeutral(ch);
    ch[11] = CPACK_CRSF_MAX;
    const ControllerCommand& c = feedCh(b, ch, 10);
    TEST_ASSERT_TRUE(c.feat_foot_contact);
    TEST_ASSERT_TRUE(c.feat_terrain_leveling);
    TEST_ASSERT_TRUE(c.gait_tune_active);
    TEST_ASSERT_FALSE(c.feat_passive_pose);
    TEST_ASSERT_TRUE(c.host_authority);
  }
  // Bits 0 and 2 only (foot contact + gait tune).
  {
    ControllerBridge b;
    uint16_t ch[CPACK_NUM_CHANNELS];
    makeTxNeutral(ch);
    ch[11] = crsfForBin(0x5, 16);
    const ControllerCommand& c = feedCh(b, ch, 10);
    TEST_ASSERT_TRUE(c.feat_foot_contact);
    TEST_ASSERT_FALSE(c.feat_terrain_leveling);
    TEST_ASSERT_TRUE(c.gait_tune_active);
    TEST_ASSERT_FALSE(c.feat_passive_pose);
    TEST_ASSERT_FALSE(c.host_authority);
  }
}

void test_tx16s_action_selector_and_fire() {
  // Selector 5 + fire -> Btn1 -> StandUp trick.
  {
    ControllerBridge b;
    uint16_t ch[CPACK_NUM_CHANNELS];
    makeTxNeutral(ch);
    ch[12] = crsfForBin(5, 13);
    ch[13] = CPACK_CRSF_MAX;  // fire
    const ControllerCommand& c = feedCh(b, ch, 100);
    TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::StandUp),
                           static_cast<uint8_t>(c.trick));
    TEST_ASSERT_TRUE(b.rawInputs().buttons[0]);
  }
  // Selector 0 + fire -> Nav1Up -> pitch trim up.
  {
    ControllerBridge b;
    uint16_t ch[CPACK_NUM_CHANNELS];
    makeTxNeutral(ch);
    ch[12] = crsfForBin(0, 13);
    ch[13] = CPACK_CRSF_MAX;
    const ControllerCommand& c = feedCh(b, ch, 100);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, kTrimStepRad, c.trim_pitch);
  }
}

void test_tx16s_action_fire_inactive_no_fire() {
  ControllerBridge b;
  uint16_t ch[CPACK_NUM_CHANNELS];
  makeTxNeutral(ch);
  ch[12] = crsfForBin(5, 13);  // would be StandUp
  ch[13] = CPACK_CRSF_MIN;     // fire inactive
  const ControllerCommand& c = feedCh(b, ch, 100);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::None),
                         static_cast<uint8_t>(c.trick));
  TEST_ASSERT_FALSE(b.rawInputs().buttons[0]);
}

void test_tx16s_nav2right_unmapped() {
  ControllerBridge b;
  uint16_t ch[CPACK_NUM_CHANNELS];
  makeTxNeutral(ch);
  // Sweep every action selector with fire held; Nav2Right must never be set.
  for (uint8_t a = 0; a < 13; ++a) {
    ch[12] = crsfForBin(a, 13);
    ch[13] = CPACK_CRSF_MAX;
    feedCh(b, ch, 100 + a * 10);
    TEST_ASSERT_FALSE(b.rawInputs().nav[1][CPACK_NAV_RIGHT]);
  }
}

// --- lock-once behaviour ---------------------------------------------------

void test_lock_once_custom_then_tx_ignored() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  feed(b, in, 10);  // primes + locks custom
  TEST_ASSERT_EQUAL_UINT(
      static_cast<uint8_t>(InputProfile::CustomControllerChannelPack),
      static_cast<uint8_t>(b.detectedProfile()));
  // Now feed a TX16S-looking frame many times: profile must not switch.
  uint16_t tx[CPACK_NUM_CHANNELS];
  makeTxNeutral(tx);
  for (int i = 0; i < 5; ++i) b.update(tx, true, 100 + i * 10);
  TEST_ASSERT_EQUAL_UINT(
      static_cast<uint8_t>(InputProfile::CustomControllerChannelPack),
      static_cast<uint8_t>(b.detectedProfile()));
}

void test_lock_once_tx_then_custom_ignored() {
  ControllerBridge b;
  uint16_t tx[CPACK_NUM_CHANNELS];
  makeTxNeutral(tx);
  feedCh(b, tx, 10);  // primes + locks TX16S
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(InputProfile::Tx16sMk3Direct),
                         static_cast<uint8_t>(b.detectedProfile()));
  ChannelPackInputs_t in = makeNeutral();
  uint16_t cp[CPACK_NUM_CHANNELS];
  ChannelPack::packInputs(&in, cp);
  for (int i = 0; i < 5; ++i) b.update(cp, true, 100 + i * 10);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(InputProfile::Tx16sMk3Direct),
                         static_cast<uint8_t>(b.detectedProfile()));
}

// --- link loss behaviour ---------------------------------------------------

void test_link_loss_after_lock_keeps_profile() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  in.switches[0] = true;  // SwA arm
  uint16_t ch[CPACK_NUM_CHANNELS];
  ChannelPack::packInputs(&in, ch);
  feedCh(b, ch, 10);  // locks custom
  TEST_ASSERT_TRUE(b.profileLocked());
  // Link drops -> failsafe, but the locked profile is retained.
  const ControllerCommand& down = b.update(ch, /*link=*/false, 20);
  TEST_ASSERT_TRUE(down.failsafe);
  TEST_ASSERT_TRUE(b.profileLocked());
  TEST_ASSERT_EQUAL_UINT(
      static_cast<uint8_t>(InputProfile::CustomControllerChannelPack),
      static_cast<uint8_t>(b.detectedProfile()));
  // Link returns -> the very next frame decodes (no 3-frame re-detection).
  const ControllerCommand& up = b.update(ch, true, 30);
  TEST_ASSERT_TRUE(up.valid);
  TEST_ASSERT_TRUE(up.arm_request);
}

void test_link_loss_before_lock_clears_streak() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  uint16_t ch[CPACK_NUM_CHANNELS];
  ChannelPack::packInputs(&in, ch);
  b.update(ch, true, 10);  // streak = 1, not locked
  TEST_ASSERT_FALSE(b.profileLocked());
  b.update(ch, false, 20);  // link drop clears the pending streak
  TEST_ASSERT_FALSE(b.profileLocked());
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(InputProfile::Unknown),
                         static_cast<uint8_t>(b.detectedProfile()));
  // Needs a fresh full streak: two frames still not locked, third locks.
  b.update(ch, true, 30);
  b.update(ch, true, 40);
  TEST_ASSERT_FALSE(b.profileLocked());
  b.update(ch, true, 50);
  TEST_ASSERT_TRUE(b.profileLocked());
}

void test_failsafe_before_profile_locked() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  uint16_t ch[CPACK_NUM_CHANNELS];
  ChannelPack::packInputs(&in, ch);
  const ControllerCommand& c = b.update(ch, true, 10);  // 1st frame, not locked
  TEST_ASSERT_FALSE(b.profileLocked());
  TEST_ASSERT_TRUE(c.failsafe);
  TEST_ASSERT_FALSE(c.valid);
  TEST_ASSERT_TRUE(c.estop);
}

// --- clamp / normalisation -------------------------------------------------

void test_tx16s_crsf_clamps_out_of_range() {
  ControllerBridge b;
  uint16_t ch[CPACK_NUM_CHANNELS];
  makeTxNeutral(ch);
  feedCh(b, ch, 10);  // lock TX16S first
  // Out-of-range values must clamp before conversion (no unsigned underflow).
  ch[0] = 100;   // below CPACK_CRSF_MIN -> gimbal -1000
  ch[1] = 1900;  // above CPACK_CRSF_MAX -> gimbal +1000
  b.update(ch, true, 20);
  TEST_ASSERT_INT_WITHIN(2, -1000, b.rawInputs().gimbal[0]);
  TEST_ASSERT_INT_WITHIN(2, 1000, b.rawInputs().gimbal[1]);
}

void test_tx16s_reserved_channels_have_no_effect() {
  ControllerBridge b;
  uint16_t ch[CPACK_NUM_CHANNELS];
  makeTxNeutral(ch);
  ch[14] = CPACK_CRSF_MAX;  // MAX 0 spare channels driven high
  ch[15] = CPACK_CRSF_MAX;
  const ControllerCommand& c = feedCh(b, ch, 10);
  TEST_ASSERT_TRUE(c.valid);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(TrickId::None),
                         static_cast<uint8_t>(c.trick));
  TEST_ASSERT_FALSE(c.arm_request);
  TEST_ASSERT_FALSE(c.feat_foot_contact);
}

void test_reset_returns_to_unknown_unlocked() {
  ControllerBridge b;
  ChannelPackInputs_t in = makeNeutral();
  feed(b, in, 10);  // locks custom
  TEST_ASSERT_TRUE(b.profileLocked());
  b.reset();
  TEST_ASSERT_FALSE(b.profileLocked());
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(InputProfile::Unknown),
                         static_cast<uint8_t>(b.detectedProfile()));
}

}  // namespace


void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_walk_mode_twist_from_default_bindings);
  RUN_TEST(test_translate_body_mode_pose_overlay_keeps_walking);
  RUN_TEST(test_rotate_body_mode_clamped_to_envelope);
  RUN_TEST(test_gait_index_from_select_toggle);
  RUN_TEST(test_gait_selection_is_live_in_every_mode);
  RUN_TEST(test_arm_switch_requires_no_kill);
  RUN_TEST(test_arm_switch_uses_guarded_pot_channel_band);
  RUN_TEST(test_kill_switch_forces_estop_and_disarm);
  RUN_TEST(test_arm_release_rejects_short_glitch_but_accepts_stable_release);
  RUN_TEST(test_failsafe_on_link_down);
  RUN_TEST(test_failsafe_on_stale_timeout);
  RUN_TEST(test_shape_params_from_pots);
  RUN_TEST(test_encoders_no_longer_drive_gait_shape);
  RUN_TEST(test_gait_tune_toggle_switches_nav1_from_trim_to_parameters);
  RUN_TEST(test_gait_tune_adjusts_selected_parameter_and_clamps);
  RUN_TEST(test_gait_tune_save_bumps_sequence_once_per_press);
  RUN_TEST(test_gait_tune_defaults_track_config_until_editor_opens);
  RUN_TEST(test_feature_toggle_levels);
  RUN_TEST(test_host_authority_switch);
  RUN_TEST(test_trick_fires_once_per_press);
  RUN_TEST(test_trick_refractory_debounce);
  RUN_TEST(test_nav_cluster_trick_binding);
  RUN_TEST(test_pose_trim_nudge_and_reset);
  RUN_TEST(test_deadband_kills_centre_jitter);
  RUN_TEST(test_invert_flips_axis);
  RUN_TEST(test_setbindings_remaps_source);
  RUN_TEST(test_asymmetric_gimbal_calibration_maps_center_and_endpoints);
  RUN_TEST(test_calibration_reverse_and_invalid_input_rejection);
  RUN_TEST(test_unipolar_calibration_maps_pot_range);
  RUN_TEST(test_default_ema_smooths_a_stick_step_after_initial_seed);
  RUN_TEST(test_centered_noise_remains_zero_after_conditioning);
  // Input-profile detection & lock.
  RUN_TEST(test_custom_profile_locks_after_stable_frames);
  RUN_TEST(test_custom_btn4_uses_four_buttons);
  RUN_TEST(test_custom_digital_fields_use_valid_crsf_range_and_round_trip);
  RUN_TEST(test_tx16s_profile_locks_after_stable_frames);
  // TX16S direct decode.
  RUN_TEST(test_tx16s_gimbals_and_pots);
  RUN_TEST(test_tx16s_encoders_are_absolute);
  RUN_TEST(test_tx16s_toggles_gait_and_mode);
  RUN_TEST(test_tx16s_safety_mask);
  RUN_TEST(test_tx16s_feature_mask);
  RUN_TEST(test_tx16s_action_selector_and_fire);
  RUN_TEST(test_tx16s_action_fire_inactive_no_fire);
  RUN_TEST(test_tx16s_nav2right_unmapped);
  // Lock-once & link-loss behaviour.
  RUN_TEST(test_lock_once_custom_then_tx_ignored);
  RUN_TEST(test_lock_once_tx_then_custom_ignored);
  RUN_TEST(test_link_loss_after_lock_keeps_profile);
  RUN_TEST(test_link_loss_before_lock_clears_streak);
  RUN_TEST(test_failsafe_before_profile_locked);
  RUN_TEST(test_reset_returns_to_unknown_unlocked);
  // Clamp / normalisation.
  RUN_TEST(test_tx16s_crsf_clamps_out_of_range);
  RUN_TEST(test_tx16s_reserved_channels_have_no_effect);
  return UNITY_END();
}
