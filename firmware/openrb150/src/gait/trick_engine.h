#pragma once

// ===========================================================================
// Trick / choreography engine (oha.5). Portable, heap-free, native-testable.
//
// Consumes the controller bridge's edge-triggered TrickId (oha.2/oha.3) and
// turns each trick into a deterministic, time-stepped *body* choreography:
// stand up / sit down, a friendly body wave, a crouch<->tall toggle, an
// in-place yaw twirl, a body-z stretch/push-up, a held lean/look attitude, and
// a looping dance. Every trick is expressed purely as a sequence of gait id +
// body twist + 6-DOF body-pose + body-height keyframes -- the SAME quantities
// the RC walk path already feeds into the gait pipeline -- so a trick can never
// command a raw servo position; it only shapes intent that the existing IK +
// servo-map + safety gate still validate (AGENTS.md 1.1 / 5.2).
//
// The engine is a small linear-interpolating keyframe player. A trick is a
// TrickProgram: a fixed table of TrickKeyframe targets, each reached by ramping
// from the previous frame's targets over its duration. Programs are one-shot
// (auto-complete), looping (DanceLoop, until cancelled), or holding (crouch /
// lean, frozen at the last frame until cancelled or re-triggered).
//
// Cancellation is first-class: the control task passes `sticks_active` each
// cycle, so any operator stick motion immediately reclaims control, and the
// safety layer calls cancel() on a closed motion gate or E-stop. All state is
// fixed-size (no heap, no Arduino deps), safe to own from controlTask and to
// exercise under pio test -e native.
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"
#include "../input/controller_bridge.h"  // controller::TrickId
#include "body_ik.h"                      // gait::BodyPose

namespace gait {

// One choreography keyframe: the absolute target the player ramps to over
// `duration_ms` (from the previous frame's targets, or the trick's entry pose
// for the first frame). `gait` takes effect immediately on segment entry; the
// twist / pose / height are interpolated. `height_frac` is a 0..1 body-height
// override (mapped onto the configured safe range by the control task); a
// negative value means "do not override height" (leave the live RC/host value).
//
// Plain aggregate (no default member initializers) so the constexpr program
// tables brace-initialize under gnu++11; omitted trailing fields value-init to
// zero, so a frame that only moves a few axes stays terse.
struct TrickKeyframe {
  uint16_t duration_ms;
  config::GaitId gait;
  float vx;
  float vy;
  float wz;
  float x_mm;
  float y_mm;
  float z_mm;
  float roll;
  float pitch;
  float yaw;
  float height_frac;
};

// A trick program: an ordered keyframe table plus end-of-sequence behaviour.
struct TrickProgram {
  const TrickKeyframe* frames;
  uint8_t count;
  bool loop;       // restart at frame 0 when the last frame finishes (dance)
  bool hold_last;  // freeze at the last frame until cancelled / re-triggered
};

// The motion the control layer applies this cycle while a trick runs.
struct TrickOutput {
  bool active = false;  // a trick is currently driving the body
  config::GaitId gait = config::GaitId::Stand;
  float twist_vx = 0.0f;
  float twist_vy = 0.0f;
  float twist_wz = 0.0f;
  BodyPose pose;                   // body-pose offset (planted-foot moves)
  bool override_height = false;    // true while the trick drives body height
  float body_height_frac = 0.0f;   // 0..1 height target when override_height
};

class TrickEngine {
 public:
  TrickEngine() { reset(); }

  // Clear to idle (no trick, neutral output). Keeps the crouch toggle latch.
  void reset();

  // Begin a trick. `entry_height_frac` is the live commanded body-height
  // fraction so the first segment ramps smoothly from the current pose.
  // CrouchToggle flips between a crouched and a tall hold using internal latch
  // state. TrickId::None (or an unknown id) cancels / is ignored.
  void trigger(controller::TrickId trick, float entry_height_frac,
               uint32_t now_ms);

  // Cancel any active trick immediately (E-stop, motion gate closed, source
  // change). The next update() returns an inactive output.
  void cancel();

  bool active() const { return active_; }
  controller::TrickId current() const { return trick_; }

  // Advance the active trick by `dt_ms`. If `sticks_active`, the operator
  // reclaims control and the trick cancels. Returns the body motion to apply
  // this cycle (inactive when no trick is running).
  const TrickOutput& update(uint32_t dt_ms, bool sticks_active);

  const TrickOutput& output() const { return out_; }

 private:
  // Select the program for a trick id (CrouchToggle flips the latch).
  const TrickProgram* selectProgram(controller::TrickId trick);
  // Interpolate the current segment's keyframe at fraction t in [0,1].
  void writeOutputs(const TrickKeyframe& kf, float t);
  // Latch the segment's exact targets as the next segment's ramp origin.
  void captureTargets(const TrickKeyframe& kf);

  bool active_;
  bool held_;  // frozen at the last frame of a hold_last program
  controller::TrickId trick_;
  const TrickProgram* prog_;
  uint8_t seg_;
  uint32_t seg_elapsed_ms_;

  // Ramp origin (previous frame's targets, or the trick entry pose).
  float from_vx_, from_vy_, from_wz_;
  float from_x_, from_y_, from_z_;
  float from_roll_, from_pitch_, from_yaw_;
  float from_height_;  // 0..1, always valid (entry or last overridden frame)

  bool crouched_;  // CrouchToggle latch: false = tall, true = crouched
  TrickOutput out_;
};

}  // namespace gait
