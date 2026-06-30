// Native (host) Unity tests for the trick / choreography engine (oha.5).
//
// Drives TrickEngine exactly as controlTask will: trigger a TrickId, then step
// update(dt, sticks_active) on the 10 ms control period and assert the body
// motion the engine produces. Proves the keyframe interpolation, gait
// selection, body-height override, one-shot completion, hold programs, the
// crouch toggle latch, the dance loop, and stick / cancel pre-emption.
//
// Run with:  pio test -e native -f test_trick_engine

#include <unity.h>

#include "gait/trick_engine.h"

using namespace gait;
using controller::TrickId;

namespace {

constexpr uint32_t kDt = 10;  // control period (period_ms::kControl)

// Advance the engine by `ms` of idle (no stick) time and return the output.
const TrickOutput& step(TrickEngine& e, uint32_t ms) {
  const TrickOutput* o = &e.output();
  for (uint32_t t = 0; t < ms; t += kDt) {
    o = &e.update(kDt, false);
  }
  return *o;
}

// --- one-shot programs -----------------------------------------------------

void test_stand_up_ramps_height_and_completes() {
  TrickEngine e;
  e.trigger(TrickId::StandUp, 0.30f, 0);
  TEST_ASSERT_TRUE(e.active());
  // Mid-ramp (~350 ms of a 700 ms segment): height between entry 0.30 and 0.80.
  const TrickOutput& mid = step(e, 350);
  TEST_ASSERT_TRUE(mid.active);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Stand),
                          static_cast<uint8_t>(mid.gait));
  TEST_ASSERT_TRUE(mid.override_height);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.55f, mid.body_height_frac);
  // Past the segment end the one-shot completes and goes inactive.
  const TrickOutput& done = step(e, 450);
  TEST_ASSERT_FALSE(done.active);
  TEST_ASSERT_FALSE(e.active());
}

void test_sit_down_selects_sit_gait_low() {
  TrickEngine e;
  e.trigger(TrickId::SitDown, 0.60f, 0);
  const TrickOutput& mid = step(e, 350);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Sit),
                          static_cast<uint8_t>(mid.gait));
  TEST_ASSERT_TRUE(mid.override_height);
  // Heading toward the 0.05 sit height (below the 0.60 entry).
  TEST_ASSERT_TRUE(mid.body_height_frac < 0.60f);
  const TrickOutput& done = step(e, 450);
  TEST_ASSERT_FALSE(done.active);
}

void test_twirl_ramps_yaw_then_stops() {
  TrickEngine e;
  e.trigger(TrickId::Twirl, 0.50f, 0);
  // Inside the hold segment the yaw twist is near full and the gait is Tripod.
  const TrickOutput& spin = step(e, 800);
  TEST_ASSERT_TRUE(spin.active);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Tripod),
                          static_cast<uint8_t>(spin.gait));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.80f, spin.twist_wz);
  TEST_ASSERT_FALSE(spin.override_height);  // twirl leaves height alone
  // After the full 2100 ms program it completes with no residual twist.
  const TrickOutput& done = step(e, 1500);
  TEST_ASSERT_FALSE(done.active);
}

void test_wave_uses_stand_and_no_height_override() {
  TrickEngine e;
  e.trigger(TrickId::Wave, 0.50f, 0);
  const TrickOutput& w = step(e, 150);
  TEST_ASSERT_TRUE(w.active);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Stand),
                          static_cast<uint8_t>(w.gait));
  TEST_ASSERT_FALSE(w.override_height);
  // The wave tips the body (non-zero roll/pitch) but never walks.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, w.twist_vx);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, w.twist_wz);
  // 4 x 300 ms = 1200 ms, then completes and levels out.
  const TrickOutput& done = step(e, 1300);
  TEST_ASSERT_FALSE(done.active);
}

void test_stretch_oscillates_height_then_completes() {
  TrickEngine e;
  e.trigger(TrickId::Stretch, 0.50f, 0);
  // First dip toward 0.20. Copy the value: step() returns a reference to the
  // engine's single output, so it is overwritten by the next step().
  const float dip_h = step(e, 500).body_height_frac;
  TEST_ASSERT_TRUE(e.output().override_height);
  // Push tall toward 0.90 (sampled mid second segment).
  const float up_h = step(e, 250).body_height_frac;
  TEST_ASSERT_TRUE(up_h > dip_h);
  const TrickOutput& done = step(e, 1300);
  TEST_ASSERT_FALSE(done.active);
}

// --- hold programs ---------------------------------------------------------

void test_lean_look_holds_until_cancelled() {
  TrickEngine e;
  e.trigger(TrickId::LeanLook, 0.50f, 0);
  // After the ramp it stays active, frozen at the lean attitude.
  const TrickOutput& held = step(e, 5000);
  TEST_ASSERT_TRUE(held.active);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.20f, held.pose.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.15f, held.pose.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.20f, held.pose.yaw);
  e.cancel();
  TEST_ASSERT_FALSE(e.active());
  TEST_ASSERT_FALSE(e.update(kDt, false).active);
}

void test_crouch_toggle_latches_low_then_tall() {
  TrickEngine e;
  // First press crouches and holds low.
  e.trigger(TrickId::CrouchToggle, 0.50f, 0);
  const TrickOutput& low = step(e, 2000);
  TEST_ASSERT_TRUE(low.active);
  TEST_ASSERT_TRUE(low.override_height);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.15f, low.body_height_frac);
  // Second press toggles back tall and holds high.
  e.trigger(TrickId::CrouchToggle, low.body_height_frac, 0);
  const TrickOutput& high = step(e, 2000);
  TEST_ASSERT_TRUE(high.active);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.85f, high.body_height_frac);
}

// --- loop program ----------------------------------------------------------

void test_dance_loop_stays_active() {
  TrickEngine e;
  e.trigger(TrickId::DanceLoop, 0.50f, 0);
  // 4 x 350 ms = 1400 ms per loop; well past several loops it never completes.
  const TrickOutput& d = step(e, 5000);
  TEST_ASSERT_TRUE(d.active);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Stand),
                          static_cast<uint8_t>(d.gait));
  TEST_ASSERT_TRUE(d.override_height);
}

// --- cancellation ----------------------------------------------------------

void test_sticks_active_cancels_trick() {
  TrickEngine e;
  e.trigger(TrickId::DanceLoop, 0.50f, 0);
  (void)step(e, 200);
  TEST_ASSERT_TRUE(e.active());
  const TrickOutput& cancelled = e.update(kDt, /*sticks_active=*/true);
  TEST_ASSERT_FALSE(cancelled.active);
  TEST_ASSERT_FALSE(e.active());
}

void test_none_trigger_cancels_active() {
  TrickEngine e;
  e.trigger(TrickId::LeanLook, 0.50f, 0);
  TEST_ASSERT_TRUE(e.active());
  e.trigger(TrickId::None, 0.50f, 0);
  TEST_ASSERT_FALSE(e.active());
}

void test_retrigger_restarts_program() {
  TrickEngine e;
  e.trigger(TrickId::Twirl, 0.50f, 0);
  (void)step(e, 800);  // into the spin hold
  // Re-trigger restarts from segment 0: yaw ramps up again from ~0.
  e.trigger(TrickId::Twirl, 0.50f, 0);
  const TrickOutput& restart = e.update(kDt, false);
  TEST_ASSERT_TRUE(restart.active);
  TEST_ASSERT_TRUE(restart.twist_wz < 0.2f);  // back near the start of the ramp
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_stand_up_ramps_height_and_completes);
  RUN_TEST(test_sit_down_selects_sit_gait_low);
  RUN_TEST(test_twirl_ramps_yaw_then_stops);
  RUN_TEST(test_wave_uses_stand_and_no_height_override);
  RUN_TEST(test_stretch_oscillates_height_then_completes);
  RUN_TEST(test_lean_look_holds_until_cancelled);
  RUN_TEST(test_crouch_toggle_latches_low_then_tall);
  RUN_TEST(test_dance_loop_stays_active);
  RUN_TEST(test_sticks_active_cancels_trick);
  RUN_TEST(test_none_trigger_cancels_active);
  RUN_TEST(test_retrigger_restarts_program);
  return UNITY_END();
}
