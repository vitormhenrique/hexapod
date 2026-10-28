// Trick / choreography engine (oha.5) -- see trick_engine.h.

#include "trick_engine.h"

namespace gait {

namespace {

using config::GaitId;
using controller::TrickId;

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

inline float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

// --- Choreography programs -------------------------------------------------
//
// Pose translations are mm and rotations rad, well inside the bridge's pose
// envelope (controller::poselim: +/-50 mm, +/-0.4363 rad). Heights are 0..1
// fractions of the configured safe body-height range (gait::rcBodyHeightMm:
// 0 = 65 mm crouch, 0.5 = 132 mm neutral, 1 = 150 mm tall). Frames list every
// field in TrickKeyframe order; trailing zeros are omitted via aggregate init.
// height_frac < 0 means "do not override the live body height".
//
// Every program below was measured through the real ControllerCore path
// (test/test_trick_envelope): no servo reaches its travel clamp and no foot
// leaves the IK workspace, from any Pot2 entry height. Two rules keep it that
// way:
//   * Body-pose tricks (roll / pitch / yaw / shift) first bring the body to
//     ~0.32..0.40 (105..119 mm). The home stance at the 132 mm neutral is
//     already at ~87% of the leg reach, so a 0.25 rad roll there only
//     straightens the high-side legs against the reach limiter and looks like
//     the robot is "stuck"; 15..25 mm lower buys the vertical room instead.
//   * Nothing lifts a foot or steps while the body is at the 65 mm crouch:
//     the femur sits at about +60 deg there and any swing lift pins it on the
//     +90 deg clamp. Stepping tricks therefore floor the live height.
// Explicit height rates (hard-capped at config::kBodyHeightRateMaxMmPerS,
// 200 mm/s) and pose-rate scales (hard-capped by the shaper) make each
// keyframe reachable within its duration; the persisted RC rates (20/30 mm/s)
// would otherwise reduce every trick to a slow drift.
//
// Column order: duration gait vx vy wz x y z roll pitch yaw height height_rate
//               stride step speed pose_rate_scale lift_mask lift reach swing
//
// Leg bits (config leg index order): 0x01 rear-left, 0x02 rear-right,
// 0x04 mid-right, 0x08 front-right, 0x10 front-left, 0x20 mid-left.
constexpr uint8_t kFrontLegs = 0x18;
constexpr uint8_t kFrontLeftLeg = 0x10;
constexpr float kPoseSnappy = 3.0f;
constexpr float kPoseFast = 4.0f;

// Rise back to the neutral, level stance.
constexpr TrickKeyframe kStandUpFrames[] = {
    {600, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.50f, 150.0f, 0, 0, 0,
     kPoseSnappy},
};

// Settle into the full crouch and stay there until a stick moves or StandUp
// is pressed (held, so the body does not pop back up when the frame ends).
constexpr TrickKeyframe kSitDownFrames[] = {
    {900, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00f, 120.0f, 0, 0, 0,
     kPoseSnappy},
};

// Classic leg wave: shift the weight to the right, lift the front-left leg,
// wave it side to side twice, put it back down and return to neutral.
constexpr TrickKeyframe kWaveFrames[] = {
    {400, GaitId::Stand, 0, 0, 0, 0, -20, 0, 0, -0.06f, 0, 0.40f, 150.0f, 0, 0,
     0, kPoseSnappy},
    {300, GaitId::Stand, 0, 0, 0, 0, -20, 0, 0, -0.06f, 0, 0.40f, 150.0f, 0, 0,
     0, kPoseSnappy, kFrontLeftLeg, 55, 10, 0},
    {250, GaitId::Stand, 0, 0, 0, 0, -20, 0, 0, -0.06f, 0, 0.40f, 150.0f, 0, 0,
     0, kPoseSnappy, kFrontLeftLeg, 60, 10, 30},
    {250, GaitId::Stand, 0, 0, 0, 0, -20, 0, 0, -0.06f, 0, 0.40f, 150.0f, 0, 0,
     0, kPoseSnappy, kFrontLeftLeg, 60, 10, -30},
    {250, GaitId::Stand, 0, 0, 0, 0, -20, 0, 0, -0.06f, 0, 0.40f, 150.0f, 0, 0,
     0, kPoseSnappy, kFrontLeftLeg, 60, 10, 30},
    {250, GaitId::Stand, 0, 0, 0, 0, -20, 0, 0, -0.06f, 0, 0.40f, 150.0f, 0, 0,
     0, kPoseSnappy, kFrontLeftLeg, 60, 10, -30},
    {300, GaitId::Stand, 0, 0, 0, 0, -20, 0, 0, -0.06f, 0, 0.40f, 150.0f, 0, 0,
     0, kPoseSnappy, kFrontLeftLeg, 50, 10, 0},
    {350, GaitId::Stand, 0, 0, 0, 0, -20, 0, 0, -0.06f, 0, 0.40f, 150.0f, 0, 0,
     0, kPoseSnappy, kFrontLeftLeg, 0, 0, 0},
    {400, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.50f, 150.0f, 0, 0, 0,
     kPoseSnappy},
};

// Crouch down and hold (toggle target A).
constexpr TrickKeyframe kCrouchDownFrames[] = {
    {600, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.15f, 120.0f, 0, 0, 0,
     kPoseSnappy},
};

// Rise tall and hold (toggle target B).
constexpr TrickKeyframe kCrouchUpFrames[] = {
    {600, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.85f, 120.0f, 0, 0, 0,
     kPoseSnappy},
};

// In-place yaw spin: ramp up a yaw twist on a walking gait, hold, then stop.
// Keeps the live height, floored at 0.35 (112 mm) so the swing lift never
// saturates the femur when the operator is crouched; the fast height rate
// only matters in that floored case (otherwise no height override is active).
constexpr TrickKeyframe kTwirlFrames[] = {
    {400, GaitId::Tripod, 0, 0, 0.80f, 0, 0, 0, 0, 0, 0, -1.0f, 200.0f},
    {1500, GaitId::Tripod, 0, 0, 0.80f, 0, 0, 0, 0, 0, 0, -1.0f, 200.0f},
    {300, GaitId::Tripod, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0f, 200.0f},
};

// Push-ups: three full-range dips at the capped height rate, then settle.
constexpr TrickKeyframe kStretchFrames[] = {
    {450, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.12f, 200.0f, 0, 0, 0,
     kPoseSnappy},
    {450, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.95f, 200.0f, 0, 0, 0,
     kPoseSnappy},
    {450, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.12f, 200.0f, 0, 0, 0,
     kPoseSnappy},
    {450, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.95f, 200.0f, 0, 0, 0,
     kPoseSnappy},
    {450, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.12f, 200.0f, 0, 0, 0,
     kPoseSnappy},
    {500, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.50f, 200.0f, 0, 0, 0,
     kPoseSnappy},
};

// Hold a lean / look attitude until cancelled (slightly lowered for reach).
constexpr TrickKeyframe kLeanLookFrames[] = {
    {600, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0.12f, -0.12f, 0.22f, 0.40f, 150.0f,
     0, 0, 0, 2.0f},
};

// Looping dance: sway + yaw wiggle + body bob, repeating until cancelled.
constexpr TrickKeyframe kDanceFrames[] = {
    {320, GaitId::Stand, 0, 0, 0, 0, 12, 0, 0.16f, 0, 0, 0.35f, 200.0f, 0, 0, 0,
     kPoseFast},
    {320, GaitId::Stand, 0, 0, 0, 0, -12, 0, -0.16f, 0, 0.14f, 0.45f, 200.0f, 0,
     0, 0, kPoseFast},
    {320, GaitId::Stand, 0, 0, 0, 0, 12, 0, 0.16f, 0, -0.14f, 0.35f, 200.0f, 0,
     0, 0, kPoseFast},
    {320, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, -0.08f, 0, 0.45f, 200.0f, 0, 0, 0,
     kPoseFast},
};

// Rear-up threat pose, held until a stick cancels it: shift the body back
// over the mid/rear legs, raise the nose, then lift and extend both front
// legs in two beats. The body pose stays inside the normal envelope and the
// planted-foot IK reach limiter remains authoritative.
constexpr TrickKeyframe kSpiderAttackFrames[] = {
    {450, GaitId::Stand, 0, 0, 0, -20, 0, 0, 0, -0.15f, 0, 0.32f, 200.0f, 0, 0,
     0, kPoseSnappy},
    {350, GaitId::Stand, 0, 0, 0, -20, 0, 0, 0, -0.22f, 0, 0.32f, 200.0f, 0, 0,
     0, kPoseSnappy, kFrontLegs, 60, 30, 0},
    {300, GaitId::Stand, 0, 0, 0, -20, 0, 0, 0, -0.22f, 0, 0.32f, 200.0f, 0, 0,
     0, kPoseSnappy, kFrontLegs, 80, 40, 0},
};

// Spring jump with a front-leg kick. Not ballistic: the robot coils into a
// deep crouch, extends to the tall stance at the firmware's 200 mm/s hard
// height-rate cap with the nose coming up, kicks both front legs out at the
// top, then lands and settles. Everything still flows through the normal
// gait -> IK -> servo-map path; no raw servo targets or direct diagnostic
// mode are involved.
constexpr TrickKeyframe kJumpKickFrames[] = {
    {500, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0.06f, 0, 0.10f, 200.0f, 0, 0, 0,
     kPoseFast},
    {330, GaitId::Stand, 0, 0, 0, 10, 0, 0, 0, -0.14f, 0, 1.00f, 200.0f, 0, 0,
     0, kPoseFast},
    {220, GaitId::Stand, 0, 0, 0, 10, 0, 0, 0, -0.14f, 0, 1.00f, 200.0f, 0, 0,
     0, kPoseFast, kFrontLegs, 60, 35, 0},
    {300, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.55f, 200.0f, 0, 0, 0,
     kPoseFast, kFrontLegs, 0, 0, 0},
    {300, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.50f, 200.0f, 0, 0, 0,
     kPoseFast},
};

constexpr TrickProgram kStandUpProg{kStandUpFrames, 1, false, false, 0.0f};
constexpr TrickProgram kSitDownProg{kSitDownFrames, 1, false, true, 0.0f};
constexpr TrickProgram kWaveProg{kWaveFrames, 9, false, false, 0.0f};
constexpr TrickProgram kCrouchDownProg{kCrouchDownFrames, 1, false, true, 0.0f};
constexpr TrickProgram kCrouchUpProg{kCrouchUpFrames, 1, false, true, 0.0f};
constexpr TrickProgram kTwirlProg{kTwirlFrames, 3, false, false, 0.35f};
constexpr TrickProgram kStretchProg{kStretchFrames, 6, false, false, 0.0f};
constexpr TrickProgram kLeanLookProg{kLeanLookFrames, 1, false, true, 0.0f};
constexpr TrickProgram kDanceProg{kDanceFrames, 4, true, false, 0.0f};
constexpr TrickProgram kSpiderAttackProg{kSpiderAttackFrames, 3, false, true,
                                         0.0f};
constexpr TrickProgram kJumpKickProg{kJumpKickFrames, 5, false, false, 0.0f};

}  // namespace

void TrickEngine::reset() {
  active_ = false;
  held_ = false;
  trick_ = TrickId::None;
  prog_ = nullptr;
  seg_ = 0;
  seg_elapsed_ms_ = 0;
  from_vx_ = from_vy_ = from_wz_ = 0.0f;
  from_x_ = from_y_ = from_z_ = 0.0f;
  from_roll_ = from_pitch_ = from_yaw_ = 0.0f;
  from_height_ = 0.0f;
  from_lift_ = from_lift_reach_ = from_lift_swing_ = 0.0f;
  entry_height_ = 0.0f;
  crouched_ = false;
  out_ = TrickOutput();
}

const TrickProgram* TrickEngine::selectProgram(TrickId trick) {
  switch (trick) {
    case TrickId::StandUp:
      return &kStandUpProg;
    case TrickId::SitDown:
      return &kSitDownProg;
    case TrickId::Wave:
      return &kWaveProg;
    case TrickId::CrouchToggle:
      crouched_ = !crouched_;
      return crouched_ ? &kCrouchDownProg : &kCrouchUpProg;
    case TrickId::Twirl:
      return &kTwirlProg;
    case TrickId::Stretch:
      return &kStretchProg;
    case TrickId::LeanLook:
      return &kLeanLookProg;
    case TrickId::DanceLoop:
      return &kDanceProg;
    case TrickId::SpiderAttack:
      return &kSpiderAttackProg;
    case TrickId::JumpKick:
      return &kJumpKickProg;
    default:
      return nullptr;
  }
}

void TrickEngine::trigger(TrickId trick, float entry_height_frac,
                          uint32_t /*now_ms*/) {
  if (trick == TrickId::None) {
    cancel();
    return;
  }
  const TrickProgram* p = selectProgram(trick);
  if (p == nullptr || p->count == 0) {
    return;  // unknown trick: leave any current one running
  }
  prog_ = p;
  trick_ = trick;
  active_ = true;
  held_ = false;
  seg_ = 0;
  seg_elapsed_ms_ = 0;
  // Ramp the first segment from the live pose: neutral twist/pose, current
  // body height. Subsequent segments ramp from the previous frame's targets.
  from_vx_ = from_vy_ = from_wz_ = 0.0f;
  from_x_ = from_y_ = from_z_ = 0.0f;
  from_roll_ = from_pitch_ = from_yaw_ = 0.0f;
  from_height_ = clamp01(entry_height_frac);
  from_lift_ = from_lift_reach_ = from_lift_swing_ = 0.0f;
  entry_height_ = from_height_;
  // Seed the output at t=0 of the first segment so output() is valid before the
  // first update() (equals the entry pose).
  writeOutputs(prog_->frames[0], 0.0f);
}

void TrickEngine::cancel() {
  active_ = false;
  held_ = false;
  trick_ = TrickId::None;
  prog_ = nullptr;
  out_ = TrickOutput();
}

void TrickEngine::captureTargets(const TrickKeyframe& kf) {
  from_vx_ = kf.vx;
  from_vy_ = kf.vy;
  from_wz_ = kf.wz;
  from_x_ = kf.x_mm;
  from_y_ = kf.y_mm;
  from_z_ = kf.z_mm;
  from_roll_ = kf.roll;
  from_pitch_ = kf.pitch;
  from_yaw_ = kf.yaw;
  const float h = effectiveHeight(kf);
  if (h >= 0.0f) {
    from_height_ = h;
  }
  from_lift_ = kf.lift_mm;
  from_lift_reach_ = kf.lift_reach_mm;
  from_lift_swing_ = kf.lift_swing_mm;
}

float TrickEngine::effectiveHeight(const TrickKeyframe& kf) const {
  if (kf.height_frac >= 0.0f) return kf.height_frac;
  // Live-height frame: only the program's floor can turn it into an override.
  if (prog_ != nullptr && prog_->min_live_height_frac > 0.0f &&
      entry_height_ < prog_->min_live_height_frac) {
    return prog_->min_live_height_frac;
  }
  return -1.0f;
}

void TrickEngine::writeOutputs(const TrickKeyframe& kf, float t) {
  out_.active = true;
  out_.gait = kf.gait;
  out_.twist_vx = lerp(from_vx_, kf.vx, t);
  out_.twist_vy = lerp(from_vy_, kf.vy, t);
  out_.twist_wz = lerp(from_wz_, kf.wz, t);
  out_.pose.x_mm = lerp(from_x_, kf.x_mm, t);
  out_.pose.y_mm = lerp(from_y_, kf.y_mm, t);
  out_.pose.z_mm = lerp(from_z_, kf.z_mm, t);
  out_.pose.roll = lerp(from_roll_, kf.roll, t);
  out_.pose.pitch = lerp(from_pitch_, kf.pitch, t);
  out_.pose.yaw = lerp(from_yaw_, kf.yaw, t);
  const float h = effectiveHeight(kf);
  if (h >= 0.0f) {
    out_.override_height = true;
    out_.body_height_frac = clamp01(lerp(from_height_, h, t));
  } else {
    out_.override_height = false;
  }
  out_.height_rate_mm_per_s =
      kf.height_rate_mm_per_s > 0.0f ? kf.height_rate_mm_per_s : 0.0f;
  out_.override_gait_shape =
      kf.stride_mm > 0 || kf.step_height_mm > 0 || kf.speed_x255 > 0;
  out_.stride_mm = kf.stride_mm;
  out_.step_height_mm = kf.step_height_mm;
  out_.speed_x255 = kf.speed_x255;
  out_.pose_rate_scale = kf.pose_rate_scale;
  out_.lift_mask = kf.lift_mask;
  out_.lift_mm = lerp(from_lift_, kf.lift_mm, t);
  out_.lift_reach_mm = lerp(from_lift_reach_, kf.lift_reach_mm, t);
  out_.lift_swing_mm = lerp(from_lift_swing_, kf.lift_swing_mm, t);
}

const TrickOutput& TrickEngine::update(uint32_t dt_ms, bool sticks_active) {
  if (!active_) {
    out_.active = false;
    return out_;
  }
  if (sticks_active) {
    cancel();
    return out_;
  }
  if (held_) {
    // Frozen at the last frame of a hold program; out_ already holds it.
    return out_;
  }

  // ControllerClock normally rejects large time jumps. Keep this layer
  // independently bounded so a looping program cannot spend millions of
  // iterations consuming an externally supplied elapsed interval.
  constexpr uint32_t kMaxAdvanceMs = 1000;
  if (dt_ms > kMaxAdvanceMs) dt_ms = kMaxAdvanceMs;
  seg_elapsed_ms_ += dt_ms;
  for (;;) {
    const TrickKeyframe& kf = prog_->frames[seg_];
    const uint16_t dur = kf.duration_ms != 0 ? kf.duration_ms : 1;
    if (seg_elapsed_ms_ < dur) {
      writeOutputs(kf, static_cast<float>(seg_elapsed_ms_) /
                           static_cast<float>(dur));
      break;
    }
    // Segment finished: latch its exact targets and consume its time.
    seg_elapsed_ms_ -= dur;
    captureTargets(kf);
    if (seg_ + 1 < prog_->count) {
      ++seg_;
      continue;  // carry the remainder into the next segment
    }
    // End of the program.
    if (prog_->loop) {
      seg_ = 0;
      continue;  // restart, ramping from the last frame's targets
    }
    if (prog_->hold_last) {
      writeOutputs(kf, 1.0f);  // exact last-frame targets
      held_ = true;
      out_.active = true;
      return out_;
    }
    active_ = false;
    out_.active = false;
    return out_;
  }
  out_.active = true;
  return out_;
}

}  // namespace gait
