// Trick envelope measurement + regression (native).
//
// Runs every choreography through the real ControllerCore path (RC authority,
// BodyCommandShaper, GaitPipeline, servo map) and records, per trick, the
// per-joint tick extremes, how many control steps saturated a servo travel
// clamp, how many steps left the IK workspace, and how many were pulled back
// by the reach limiter. The table is printed so a tune can be eyeballed; the
// assertions keep every trick clamp-free and reachable.
//
// Run with: pio test -e native -f test_trick_envelope -v

#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "../../src/controller/command_frame.h"
#include "../../src/controller/controller_core.h"
#include "../../src/gait/gait_engine.h"

namespace {

using controller::TrickId;

constexpr float kTicksPerDeg = 4096.0f / 360.0f;

controller::ControllerConfigSnapshot defaultConfigSnapshot() {
  controller::ControllerConfigSnapshot snapshot;
  config::defaultRobotConfig(snapshot.robot);
  snapshot.revision = 1;
  snapshot.valid = true;
  snapshot.persistent = false;
  return snapshot;
}

controller::RobotState readyState(
    const controller::ControllerConfigSnapshot& config_snapshot) {
  controller::RobotState state;
  state.config_ready = true;
  state.battery.millivolts = 12000;
  state.battery.valid = true;
  state.battery.validity = controller::SnapshotValidity::Fresh;
  state.dxl.servo_count = config::kNumServos;
  state.dxl.validity = controller::SnapshotValidity::Fresh;
  state.dxl.configured_servo_coverage = true;
  state.dxl.config_revision = config_snapshot.revision;
  state.dxl.pose_known_mask = (1u << config::kNumServos) - 1u;
  state.dxl.arming_ready = true;
  state.dxl.torque_off = false;
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    state.dxl.servos[index].id = config_snapshot.robot.servos[index].id;
    state.dxl.servos[index].present_position = config::kServoCenterTick;
    state.dxl.servos[index].ok = true;
  }
  return state;
}

controller::ControllerIntent safeRcIntent(float height_frac) {
  controller::ControllerIntent intent;
  intent.rc.kill = false;
  intent.rc.failsafe = false;
  intent.rc.ever_seen = true;
  intent.rc.command.valid = true;
  intent.rc.command.body_height = height_frac;
  intent.rc.command.speed = 0.5f;
  intent.rc.command.stride = 60.0f / config::kMaxGaitStrideMm;
  intent.rc.command.step_height = 30.0f / config::kMaxGaitStepMm;
  intent.motion.duty_x255 = 128;
  return intent;
}

controller::ControllerTime timeAt(uint32_t now_ms) {
  controller::ControllerTime time;
  time.now_ms = now_ms;
  time.dt_ms = 10;
  time.valid = true;
  return time;
}

struct JointStat {
  float min_deg = 1e9f;
  float max_deg = -1e9f;
  uint32_t clamped_steps = 0;
};

struct TrickReport {
  const char* name;
  JointStat joint[config::kNumLegs][config::kJointsPerLeg];
  uint32_t steps = 0;
  uint32_t active_steps = 0;
  uint32_t clamped_steps = 0;      // steps with any clamped joint
  uint32_t unreachable_steps = 0;  // steps with any_goal_unreachable
  uint32_t reach_limited_steps = 0;
  float min_height_mm = 1e9f;
  float max_height_mm = -1e9f;
  float first_clamp_s = -1.0f;
};

const char* trickName(TrickId id) {
  switch (id) {
    case TrickId::StandUp: return "StandUp";
    case TrickId::SitDown: return "SitDown";
    case TrickId::Wave: return "Wave";
    case TrickId::CrouchToggle: return "CrouchToggle";
    case TrickId::Twirl: return "Twirl";
    case TrickId::Stretch: return "Stretch";
    case TrickId::LeanLook: return "LeanLook";
    case TrickId::DanceLoop: return "DanceLoop";
    case TrickId::SpiderAttack: return "SpiderAttack";
    case TrickId::JumpKick: return "JumpKick";
    default: return "?";
  }
}

// Drive one trick from a settled RC stance at `entry_height_frac` for up to
// `max_ms` (hold/loop tricks never finish on their own) and collect stats.
TrickReport runTrick(TrickId id, float entry_height_frac, uint32_t max_ms) {
  const controller::ControllerConfigSnapshot cfg = defaultConfigSnapshot();
  const controller::RobotState state = readyState(cfg);
  controller::ControllerIntent intent = safeRcIntent(entry_height_frac);
  controller::ControllerCore core;
  controller::RobotCommand command;
  const config::RobotConfig& robot = cfg.robot;

  uint32_t t = 0;
  // Arm and let the stance settle (height ramps at the configured rate).
  for (; t < 60; t += 10) {
    if (t == 30) intent.rc.armed = true;
    core.step(state, intent, cfg, timeAt(t), command);
  }
  for (; t < 6000; t += 10) core.step(state, intent, cfg, timeAt(t), command);
  TEST_ASSERT_TRUE(command.motion_gate);

  TrickReport r;
  r.name = trickName(id);
  intent.rc.command.trick = id;
  core.step(state, intent, cfg, timeAt(t), command);
  t += 10;
  intent.rc.command.trick = TrickId::None;
  const uint32_t t_start = t;
  bool was_active = false;
  for (; t < t_start + max_ms; t += 10) {
    core.step(state, intent, cfg, timeAt(t), command);
    ++r.steps;
    // Trick activity is not exported directly; use the applied height and
    // goal activity as the window. We simply measure the whole window plus a
    // settle tail, which is what the servos actually experience.
    bool any_clamp = false;
    for (uint8_t i = 0; i < command.goals.count; ++i) {
      const gait::PipelineJoint& j = command.goals.joints[i];
      const config::ServoConfig& sc =
          robot.servos[j.leg * config::kJointsPerLeg + j.joint];
      const float deg = static_cast<float>(
                            static_cast<int32_t>(j.tick) -
                            config::kServoCenterTick - sc.trim_ticks) /
                        kTicksPerDeg * static_cast<float>(sc.sign);
      JointStat& s = r.joint[j.leg][j.joint];
      if (deg < s.min_deg) s.min_deg = deg;
      if (deg > s.max_deg) s.max_deg = deg;
      if (j.clamped) {
        ++s.clamped_steps;
        any_clamp = true;
      }
    }
    if (any_clamp) {
      ++r.clamped_steps;
      if (r.first_clamp_s < 0.0f) {
        r.first_clamp_s = static_cast<float>(t - t_start) / 1000.0f;
      }
    }
    if (command.diagnostics.any_goal_unreachable) ++r.unreachable_steps;
    if (command.diagnostics.any_goal_reach_limited) ++r.reach_limited_steps;
    const float h = static_cast<float>(command.diagnostics.applied_body_height_mm);
    if (h < r.min_height_mm) r.min_height_mm = h;
    if (h > r.max_height_mm) r.max_height_mm = h;
    (void)was_active;
  }
  return r;
}

void printReport(const TrickReport& r) {
  printf("\n== %s: %lu steps, clamped %lu (first %.2fs), unreachable %lu, "
         "reach-limited %lu, height %.0f..%.0f mm\n",
         r.name, (unsigned long)r.steps, (unsigned long)r.clamped_steps,
         (double)r.first_clamp_s, (unsigned long)r.unreachable_steps,
         (unsigned long)r.reach_limited_steps, (double)r.min_height_mm,
         (double)r.max_height_mm);
  static const char* kJointNames[3] = {"coxa", "femur", "tibia"};
  for (uint8_t j = 0; j < config::kJointsPerLeg; ++j) {
    printf("   %-5s", kJointNames[j]);
    for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
      const JointStat& s = r.joint[leg][j];
      printf(" L%u[%6.1f..%5.1f%s]", leg + 1, (double)s.min_deg,
             (double)s.max_deg, s.clamped_steps ? "!" : " ");
    }
    printf("\n");
  }
}

struct Case {
  TrickId id;
  float entry_height;
  uint32_t max_ms;
};

constexpr Case kCases[] = {
    {TrickId::StandUp, 0.5f, 3000},
    {TrickId::SitDown, 0.5f, 4000},
    {TrickId::Wave, 0.5f, 2500},
    {TrickId::CrouchToggle, 0.5f, 4000},
    {TrickId::Twirl, 0.5f, 3500},
    {TrickId::Stretch, 0.5f, 6000},
    {TrickId::LeanLook, 0.5f, 2500},
    {TrickId::DanceLoop, 0.5f, 6000},
    {TrickId::SpiderAttack, 0.5f, 3000},
    {TrickId::JumpKick, 0.5f, 4000},
    // Tricks started from the crouch / tall ends of the Pot2 envelope.
    {TrickId::Wave, 0.0f, 2500},
    {TrickId::Wave, 1.0f, 2500},
    {TrickId::LeanLook, 1.0f, 2500},
    {TrickId::DanceLoop, 1.0f, 4000},
    {TrickId::SpiderAttack, 1.0f, 3000},
    {TrickId::Twirl, 0.0f, 3500},
    {TrickId::JumpKick, 1.0f, 4000},
};

}  // namespace

void test_tricks_never_hit_servo_travel_or_leave_workspace() {
  bool all_ok = true;
  for (const Case& c : kCases) {
    const TrickReport r = runTrick(c.id, c.entry_height, c.max_ms);
    printReport(r);
    if (r.clamped_steps != 0 || r.unreachable_steps != 0) all_ok = false;
  }
  TEST_ASSERT_TRUE_MESSAGE(all_ok,
                           "a trick saturated a servo clamp or left the IK "
                           "workspace (see table above)");
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_tricks_never_hit_servo_travel_or_leave_workspace);
  return UNITY_END();
}
