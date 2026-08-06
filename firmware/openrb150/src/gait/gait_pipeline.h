#pragma once

// ===========================================================================
// Gait -> servo goal pipeline (portable, no Arduino deps, issue lmt.1 / 22l.5).
//
// Bridges the high-level motion intent to concrete DYNAMIXEL goal ticks, the
// missing link that turns a stored MotionIntent into something dxlTask can Sync
// Write. It owns the three portable stages and runs them as one bounded loop:
//
//   GaitEngine.update(dt)            -> 6 foot targets in body frame B (mm)
//   BodyKinematics.solveBody(leg,..) -> per-leg coxa/femur/tibia angles (rad)
//   ServoMap.angleToTick(leg,j,..)   -> clamped goal tick + clamp flags
//
// The output is a flat list of (servo id, tick) goals plus the leg/joint and
// clamp flag for each, ready for both the Sync Write path (dxl/dxl_bus.h
// GoalTarget = {id, tick}) and the servo_goals telemetry stream. Only mapped
// joints are emitted, so an unmapped slot never produces a bogus goal.
//
// Every stage is already individually clamped (gait stroke/lift caps, the IK
// reachable annulus, and the configured servo travel), so this pipeline can
// never emit a tick outside the configured [min_tick, max_tick]; it is the
// final servo-target generation stage before the safety gate in dxlTask. Before
// IK it also applies a reachability-aware stride limit (lmt.14): each foot is
// pulled radially inward to stay within kReachMarginFrac of full leg reach, so
// the documented near-boundary home stance plus a commanded stride can never
// drive a leg off the workspace. It reports `any_reach_limited` when that clamp
// engaged and `any_unreachable` if a target still left the workspace.
//
// Static memory only: the engine, body transform and servo map are all held by
// value and constructed once from the active RobotConfig. Deterministic float
// math, safe to run at >= 50 Hz in controlTask.
//
// Arduino-free (uses dxl/servo_map.h, NOT dxl/dxl_bus.h) so the whole pipeline
// runs in the native unit tests (pio test -e native).
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"
#include "../dxl/servo_map.h"
#include "body_ik.h"
#include "gait_engine.h"

namespace gait {

// Goal-tick slew range (ticks/second), mapped from the speed knob (Pot1) in
// setParams. Used ONLY for the arm-to-stance transition: after seedGoal()
// each joint ramps from its actual present position until it reaches the
// commanded trajectory once, then the limiter disengages so it can never
// distort (clip) live gait trajectories into flatten-and-catch-up jerk.
constexpr float kMinGoalSlewTicksPerSec = 600.0f;
constexpr float kMaxGoalSlewTicksPerSec = 4500.0f;
// First-order filter time constant for the commanded body pose. Stick noise
// in translate/rotate modes lands on the filter target, not the servos.
constexpr float kPoseFilterTau = 0.12f;  // seconds
// Pose snap-to-neutral thresholds (imperceptible residuals).
constexpr float kPoseSnapMm = 0.5f;
constexpr float kPoseSnapRad = 0.005f;
// Gait-tune preview cycle time. Deliberately slower than a walking step so the
// operator can watch one full swing after each parameter nudge.
constexpr float kPreviewCycleSeconds = 2.0f;

// One resolved joint goal: the DXL id + clamped goal tick, plus the leg/joint
// slot and whether the tick was saturated against the configured servo travel.
struct PipelineJoint {
  uint8_t id = 0;
  uint16_t tick = config::kServoCenterTick;
  uint8_t leg = 0;
  uint8_t joint = 0;
  bool clamped = false;
};

// Result of one pipeline tick: up to one goal per mapped servo (18 max).
struct PipelineOutput {
  PipelineJoint joints[config::kNumServos];
  uint8_t count = 0;            // number of mapped joints written
  bool any_unreachable = false;  // a leg IK target left the reachable workspace
  bool any_reach_limited = false;  // a foot was pulled in to the reach margin (lmt.14)
};

class GaitPipeline {
 public:
  explicit GaitPipeline(const config::RobotConfig& cfg);

  // Seed the gait engine from the config's persisted gait defaults (gait id,
  // body height, stride, step height, speed). Does not bump the phase.
  void configureFromConfig();

  // Re-apply the active RobotConfig after it changed (boot adopt / CFG_COMMIT,
  // lmt.7). Rebuilds the cached body transform + leg IK from the new link
  // lengths / leg geometry and re-seeds the gait defaults. The servo map reads
  // the config reference live, so it needs no rebuild. The cfg reference itself
  // is stable (ConfigApi::config() returns a fixed member); only its contents
  // change, so callers re-run this whenever ConfigApi::revision() advances.
  void reconfigure();

  // Select the active gait (stand/sit/tripod/ripple/wave/crawl).
  void setGait(config::GaitId g);

  // Apply gait parameters (body height, stride, step, duty, speed). Mark III
  // APG tables own duty/support timing; duty_x255 remains a wire-compatible
  // input but does not alter those fixed safety patterns.
  // clamped to the gait engine's safe maxima.
  void setParams(uint16_t body_height_mm, uint16_t stride_len_mm,
                 uint16_t step_height_mm, uint8_t duty_x255, uint8_t speed_x255);

  // Set the normalised body twist (forward/lateral/yaw, each clamped to +/-1).
  void setTwist(float vx, float vy, float wz);

  // Set a 6-DOF body pose offset applied to the (planted) stance feet so the
  // body can translate/rotate over fixed footholds -- "move the core without
  // moving the legs" (oha.3). A neutral (all-zero) pose restores the normal
  // walking path (with reachability-aware stride limiting); a non-neutral pose
  // re-expresses each gait foot target in the moved body frame via
  // BodyKinematics::solveBodyPoseLimited and clamps it to the safe IK annulus.
  // Translation is in mm and rotation in radians; callers should still clamp
  // to their command envelope (controller::poselim / protocol::motionlim).
  void setBodyPose(const BodyPose& pose);

  // Reset the gait cycle phase to 0 (e.g. when motion is (re)authorised).
  void resetPhase();

  // Single-leg tuning preview: `leg` (0..kNumLegs-1) repeatedly traces the
  // configured swing arc in place while the other five legs keep their normal
  // stance, so an operator adjusting stride / step height from the handset can
  // see the effect on the robot itself. Pass a negative value to disable.
  // The preview target goes through the same reach-margin clamp and servo
  // travel clamp as a walking target, so it can never command an unsafe pose.
  // Callers must only enable it while the twist command is neutral.
  void setPreviewLeg(int8_t leg);
  int8_t previewLeg() const { return preview_leg_; }

  // Seed the goal slew limiter for one servo from its known present position
  // (raw ticks). Called on the motion-gate rising edge with fresh status reads
  // so the joint ramps smoothly from where it actually is onto the commanded
  // trajectory; once it reaches the trajectory the limiter disengages.
  void seedGoal(uint8_t id, uint16_t present_tick);

  // Advance the gait by dt_ms and fill `out` with goal ticks for every mapped
  // joint. Bounded, no heap, never blocks.
  void update(uint32_t dt_ms, PipelineOutput& out);

  const GaitEngine& engine() const { return engine_; }

 private:
  const config::RobotConfig& cfg_;
  GaitEngine engine_;
  BodyKinematics body_;
  dxl::ServoMap map_;
  BodyPose pose_target_;   // latest commanded body pose (raw, from RC/host)
  BodyPose pose_;          // filtered pose actually applied to the feet
  bool apply_pose_ = false;  // true while the filtered pose is non-neutral
  int8_t preview_leg_ = -1;    // gait-tune preview leg, negative = disabled
  float preview_phase_ = 0.0f;  // preview cycle position in [0, 1)
  // Per-slot arm-transition ramp state ((leg, joint) slot order). seedGoal()
  // arms a slot; it disengages permanently once the joint reaches the live
  // trajectory, so steady-state gait output is NEVER rate-clipped.
  float goal_slew_ticks_per_s_ = kMinGoalSlewTicksPerSec;
  uint16_t last_tick_[config::kNumServos] = {};
  bool ramping_[config::kNumServos] = {};
};

}  // namespace gait
