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
// fractions of the configured safe body-height range. Frames list every field
// in TrickKeyframe order; trailing zeros are omitted via aggregate init.
// height_frac < 0 means "do not override the live body height".

// Rise to a tall, level stance.
constexpr TrickKeyframe kStandUpFrames[] = {
    {700, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.80f},
};

// Settle into the low sit pose.
constexpr TrickKeyframe kSitDownFrames[] = {
    {700, GaitId::Sit, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05f},
};

// Friendly body wave: tip the nose up and rock side to side, then level out.
constexpr TrickKeyframe kWaveFrames[] = {
    {300, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0.25f, -0.15f, 0, -1.0f},
    {300, GaitId::Stand, 0, 0, 0, 0, 0, 0, -0.10f, -0.15f, 0, -1.0f},
    {300, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0.25f, -0.15f, 0, -1.0f},
    {300, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0f},
};

// Crouch down and hold (toggle target A).
constexpr TrickKeyframe kCrouchDownFrames[] = {
    {500, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.15f},
};

// Rise tall and hold (toggle target B).
constexpr TrickKeyframe kCrouchUpFrames[] = {
    {500, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.85f},
};

// In-place yaw spin: ramp up a yaw twist on a walking gait, hold, then stop.
constexpr TrickKeyframe kTwirlFrames[] = {
    {300, GaitId::Tripod, 0, 0, 0.80f, 0, 0, 0, 0, 0, 0, -1.0f},
    {1500, GaitId::Tripod, 0, 0, 0.80f, 0, 0, 0, 0, 0, 0, -1.0f},
    {300, GaitId::Tripod, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0f},
};

// Stretch / push-up: dip low, push tall, dip again, settle to mid.
constexpr TrickKeyframe kStretchFrames[] = {
    {500, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.20f},
    {500, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.90f},
    {500, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.20f},
    {500, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.60f},
};

// Hold a lean / look attitude until cancelled.
constexpr TrickKeyframe kLeanLookFrames[] = {
    {500, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0.20f, 0.15f, 0.20f, -1.0f},
};

// Looping dance: sway + yaw wiggle + body bob, repeating until cancelled.
constexpr TrickKeyframe kDanceFrames[] = {
    {350, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0.25f, 0, 0, 0.40f},
    {350, GaitId::Stand, 0, 0, 0, 0, 0, 0, -0.25f, 0, 0.20f, 0.70f},
    {350, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0.25f, 0, -0.20f, 0.40f},
    {350, GaitId::Stand, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.60f},
};

constexpr TrickProgram kStandUpProg{kStandUpFrames, 1, false, false};
constexpr TrickProgram kSitDownProg{kSitDownFrames, 1, false, false};
constexpr TrickProgram kWaveProg{kWaveFrames, 4, false, false};
constexpr TrickProgram kCrouchDownProg{kCrouchDownFrames, 1, false, true};
constexpr TrickProgram kCrouchUpProg{kCrouchUpFrames, 1, false, true};
constexpr TrickProgram kTwirlProg{kTwirlFrames, 3, false, false};
constexpr TrickProgram kStretchProg{kStretchFrames, 4, false, false};
constexpr TrickProgram kLeanLookProg{kLeanLookFrames, 1, false, true};
constexpr TrickProgram kDanceProg{kDanceFrames, 4, true, false};

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
  if (kf.height_frac >= 0.0f) {
    from_height_ = kf.height_frac;
  }
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
  if (kf.height_frac >= 0.0f) {
    out_.override_height = true;
    out_.body_height_frac = clamp01(lerp(from_height_, kf.height_frac, t));
  } else {
    out_.override_height = false;
  }
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
