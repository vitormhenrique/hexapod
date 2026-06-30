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

  // Apply gait parameters (body height, stride, step, duty, speed). Values are
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
  // BodyKinematics::solveBodyPose. Translation in mm, rotation in radians; the
  // caller is responsible for clamping to a safe envelope (the bridge clamps to
  // controller::poselim, which mirrors protocol::motionlim).
  void setBodyPose(const BodyPose& pose);

  // Reset the gait cycle phase to 0 (e.g. when motion is (re)authorised).
  void resetPhase();

  // Advance the gait by dt_ms and fill `out` with goal ticks for every mapped
  // joint. Bounded, no heap, never blocks.
  void update(uint32_t dt_ms, PipelineOutput& out);

  const GaitEngine& engine() const { return engine_; }

 private:
  const config::RobotConfig& cfg_;
  GaitEngine engine_;
  BodyKinematics body_;
  dxl::ServoMap map_;
  BodyPose pose_;          // body offset applied to planted feet (oha.3)
  bool apply_pose_ = false;  // true while pose_ is non-neutral
};

}  // namespace gait
