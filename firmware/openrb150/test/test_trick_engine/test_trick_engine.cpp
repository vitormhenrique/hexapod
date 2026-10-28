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
  e.trigger(TrickId::StandUp, 0.10f, 0);
  TEST_ASSERT_TRUE(e.active());
  // Mid-ramp (300 ms of a 600 ms segment): height between entry 0.10 and 0.50.
  const TrickOutput& mid = step(e, 300);
  TEST_ASSERT_TRUE(mid.active);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Stand),
                          static_cast<uint8_t>(mid.gait));
  TEST_ASSERT_TRUE(mid.override_height);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.30f, mid.body_height_frac);
  TEST_ASSERT_TRUE(mid.height_rate_mm_per_s > 0.0f);
  TEST_ASSERT_TRUE(mid.pose_rate_scale > 1.0f);
  // Past the segment end the one-shot completes and goes inactive.
  const TrickOutput& done = step(e, 400);
  TEST_ASSERT_FALSE(done.active);
  TEST_ASSERT_FALSE(e.active());
}

void test_sit_down_holds_full_crouch_on_stand_gait() {
  TrickEngine e;
  e.trigger(TrickId::SitDown, 0.60f, 0);
  const TrickOutput& mid = step(e, 450);
  // The sit is a planted Stand-gait crouch (the Sit gait would snap the feet
  // 70 mm in one step); it heads for the bottom of the height envelope.
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Stand),
                          static_cast<uint8_t>(mid.gait));
  TEST_ASSERT_TRUE(mid.override_height);
  TEST_ASSERT_TRUE(mid.body_height_frac < 0.60f);
  // ...and holds there instead of popping back up when the frame ends.
  const TrickOutput& held = step(e, 3000);
  TEST_ASSERT_TRUE(held.active);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, held.body_height_frac);
  // StandUp replaces the hold and rises again.
  e.trigger(TrickId::StandUp, held.body_height_frac, 0);
  const TrickOutput& up = step(e, 600);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.50f, up.body_height_frac);
}

void test_twirl_ramps_yaw_then_stops() {
  TrickEngine e;
  e.trigger(TrickId::Twirl, 0.50f, 0);
  // Inside the hold segment the yaw twist is near full and the gait is Tripod.
  const TrickOutput& spin = step(e, 900);
  TEST_ASSERT_TRUE(spin.active);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Tripod),
                          static_cast<uint8_t>(spin.gait));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.80f, spin.twist_wz);
  TEST_ASSERT_FALSE(spin.override_height);  // twirl leaves height alone
  // After the full 2200 ms program it completes with no residual twist.
  const TrickOutput& done = step(e, 1500);
  TEST_ASSERT_FALSE(done.active);
}

void test_twirl_floors_live_height_when_entered_crouched() {
  TrickEngine e;
  // Entered at the 65 mm crouch: swing lift there would pin the femur on its
  // travel clamp, so the program floors the height to 0.35 and ramps to it.
  e.trigger(TrickId::Twirl, 0.0f, 0);
  const TrickOutput& start = step(e, 200);
  TEST_ASSERT_TRUE(start.override_height);
  TEST_ASSERT_TRUE(start.body_height_frac > 0.10f);
  TEST_ASSERT_TRUE(start.body_height_frac < 0.35f);
  TEST_ASSERT_TRUE(start.height_rate_mm_per_s > 0.0f);
  const TrickOutput& spin = step(e, 800);
  TEST_ASSERT_TRUE(spin.override_height);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.35f, spin.body_height_frac);
  // Above the floor the live height is untouched.
  e.trigger(TrickId::Twirl, 0.40f, 0);
  TEST_ASSERT_FALSE(step(e, 500).override_height);
}

void test_wave_lifts_front_left_leg_and_swings_it() {
  TrickEngine e;
  e.trigger(TrickId::Wave, 0.50f, 0);
  // Settle frame: lowered, weight shifted right, no leg lifted yet.
  const TrickOutput& settle = step(e, 390);
  TEST_ASSERT_TRUE(settle.active);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Stand),
                          static_cast<uint8_t>(settle.gait));
  TEST_ASSERT_TRUE(settle.override_height);
  TEST_ASSERT_TRUE(settle.body_height_frac < 0.50f);
  TEST_ASSERT_TRUE(settle.pose.y_mm < -10.0f);
  TEST_ASSERT_EQUAL_UINT8(0, settle.lift_mask);
  // Leg raised (front-left = leg index 4) and waving side to side.
  const TrickOutput& up = step(e, 310);
  TEST_ASSERT_EQUAL_UINT8(0x10, up.lift_mask);
  TEST_ASSERT_TRUE(up.lift_mm > 50.0f);
  const float swing_a = step(e, 250).lift_swing_mm;
  const float swing_b = step(e, 250).lift_swing_mm;
  TEST_ASSERT_TRUE(swing_a > 20.0f);
  TEST_ASSERT_TRUE(swing_b < -20.0f);
  // The wave never walks.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, e.output().twist_vx);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, e.output().twist_wz);
  // The foot is ramped back to the ground while still masked (no snap), then
  // the program levels out and completes (2750 ms total).
  const TrickOutput& down = step(e, 1140);
  TEST_ASSERT_EQUAL_UINT8(0x10, down.lift_mask);
  TEST_ASSERT_TRUE(down.lift_mm < 5.0f);
  const TrickOutput& done = step(e, 700);
  TEST_ASSERT_FALSE(done.active);
}

void test_stretch_oscillates_height_then_completes() {
  TrickEngine e;
  e.trigger(TrickId::Stretch, 0.50f, 0);
  // First dip toward 0.12. Copy the value: step() returns a reference to the
  // engine's single output, so it is overwritten by the next step().
  const float dip_h = step(e, 450).body_height_frac;
  TEST_ASSERT_TRUE(e.output().override_height);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 200.0f, e.output().height_rate_mm_per_s);
  // Push tall toward 0.95 (sampled mid second segment).
  const float up_h = step(e, 225).body_height_frac;
  TEST_ASSERT_TRUE(up_h > dip_h);
  // 6 x 450/500 ms = 2750 ms, then completes.
  const TrickOutput& done = step(e, 2200);
  TEST_ASSERT_FALSE(done.active);
}

// --- hold programs ---------------------------------------------------------

void test_lean_look_holds_until_cancelled() {
  TrickEngine e;
  e.trigger(TrickId::LeanLook, 0.50f, 0);
  // After the ramp it stays active, frozen at the lean attitude.
  const TrickOutput& held = step(e, 5000);
  TEST_ASSERT_TRUE(held.active);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.12f, held.pose.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, -0.12f, held.pose.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.22f, held.pose.yaw);
  TEST_ASSERT_TRUE(held.override_height);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.40f, held.body_height_frac);
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
  // 4 x 320 ms = 1280 ms per loop; well past several loops it never completes.
  const TrickOutput& d = step(e, 5000);
  TEST_ASSERT_TRUE(d.active);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Stand),
                          static_cast<uint8_t>(d.gait));
  TEST_ASSERT_TRUE(d.override_height);
  TEST_ASSERT_TRUE(d.pose_rate_scale > 1.0f);
}

void test_spider_attack_rears_up_and_lifts_front_legs() {
  TrickEngine e;
  e.trigger(TrickId::SpiderAttack, 0.50f, 0);
  // First beat: body back and nose up on all six feet.
  const TrickOutput& rear = step(e, 440);
  TEST_ASSERT_TRUE(rear.active);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Stand),
                          static_cast<uint8_t>(rear.gait));
  TEST_ASSERT_TRUE(rear.override_height);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.32f, rear.body_height_frac);
  TEST_ASSERT_TRUE(rear.pose.x_mm < -15.0f);
  TEST_ASSERT_TRUE(rear.pose.pitch < -0.10f);
  TEST_ASSERT_EQUAL_UINT8(0, rear.lift_mask);
  // Then both front legs (indices 3 and 4) come up and extend; held.
  const TrickOutput& attack = step(e, 2000);
  TEST_ASSERT_TRUE(attack.active);
  TEST_ASSERT_EQUAL_UINT8(0x18, attack.lift_mask);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 80.0f, attack.lift_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 40.0f, attack.lift_reach_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, -0.22f, attack.pose.pitch);
}

void test_jump_kick_coils_springs_kicks_and_lands() {
  TrickEngine e;
  e.trigger(TrickId::JumpKick, 0.50f, 0);
  // Coil: deep crouch at the capped fast rate, no gait-shape override needed
  // (the jump never steps).
  const TrickOutput& crouch = step(e, 490);
  TEST_ASSERT_TRUE(crouch.active);
  TEST_ASSERT_TRUE(crouch.override_height);
  TEST_ASSERT_TRUE(crouch.body_height_frac < 0.15f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 200.0f, crouch.height_rate_mm_per_s);
  TEST_ASSERT_FALSE(crouch.override_gait_shape);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Stand),
                          static_cast<uint8_t>(crouch.gait));
  // Spring: full height with the nose coming up.
  const TrickOutput& spring = step(e, 330);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 1.0f, spring.body_height_frac);
  TEST_ASSERT_TRUE(spring.pose.pitch < -0.10f);
  TEST_ASSERT_EQUAL_UINT8(0, spring.lift_mask);
  // Kick: both front legs out at the top.
  const TrickOutput& kick = step(e, 210);
  TEST_ASSERT_EQUAL_UINT8(0x18, kick.lift_mask);
  TEST_ASSERT_TRUE(kick.lift_mm > 50.0f);
  TEST_ASSERT_TRUE(kick.lift_reach_mm > 30.0f);
  // Land: feet ramp down while still masked, then settle and complete.
  const TrickOutput& land = step(e, 300);
  TEST_ASSERT_EQUAL_UINT8(0x18, land.lift_mask);
  TEST_ASSERT_TRUE(land.lift_mm < 5.0f);
  const TrickOutput& done = step(e, 400);
  TEST_ASSERT_FALSE(done.active);
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
  RUN_TEST(test_sit_down_holds_full_crouch_on_stand_gait);
  RUN_TEST(test_twirl_ramps_yaw_then_stops);
  RUN_TEST(test_twirl_floors_live_height_when_entered_crouched);
  RUN_TEST(test_wave_lifts_front_left_leg_and_swings_it);
  RUN_TEST(test_stretch_oscillates_height_then_completes);
  RUN_TEST(test_lean_look_holds_until_cancelled);
  RUN_TEST(test_crouch_toggle_latches_low_then_tall);
  RUN_TEST(test_dance_loop_stays_active);
  RUN_TEST(test_spider_attack_rears_up_and_lifts_front_legs);
  RUN_TEST(test_jump_kick_coils_springs_kicks_and_lands);
  RUN_TEST(test_sticks_active_cancels_trick);
  RUN_TEST(test_none_trigger_cancels_active);
  RUN_TEST(test_retrigger_restarts_program);
  return UNITY_END();
}
