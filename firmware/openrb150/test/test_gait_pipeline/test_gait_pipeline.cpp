// Native (host) unit tests for the gait -> servo goal pipeline (lmt.1).
// Run with: pio test -e native

#include <math.h>
#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/dxl/servo_map.h"
#include "../../src/gait/gait_pipeline.h"
#include "../../src/gait/leg_ik.h"

using namespace gait;
using namespace config;

namespace {

RobotConfig defaultCfg() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  return cfg;
}

// Locate the servo config for a DXL id in the given config.
const ServoConfig* servoById(const RobotConfig& cfg, uint8_t id) {
  for (uint8_t k = 0; k < kNumServos; ++k) {
    if (cfg.servos[k].id == id) return &cfg.servos[k];
  }
  return nullptr;
}

bool emittedFootBody(const RobotConfig& cfg, const PipelineOutput& out,
                     uint8_t leg, float& body_x, float& body_y,
                     float& body_z) {
  dxl::ServoMap map(cfg);
  float angles[kJointsPerLeg] = {};
  bool found[kJointsPerLeg] = {};
  for (uint8_t index = 0; index < out.count; ++index) {
    const PipelineJoint& goal = out.joints[index];
    if (goal.leg != leg || goal.joint >= kJointsPerLeg) continue;
    angles[goal.joint] = map.tickToAngle(leg, goal.joint, goal.tick);
    found[goal.joint] = true;
  }
  for (uint8_t joint = 0; joint < kJointsPerLeg; ++joint) {
    if (!found[joint]) return false;
  }

  LegIk ik(cfg.links.coxa_cmm / 100.0f, cfg.links.femur_cmm / 100.0f,
           cfg.links.tibia_cmm / 100.0f,
           cfg.geometry.home_radius_cmm / 100.0f,
           cfg.geometry.home_foot_z_cmm / 100.0f);
  float coxa_x, coxa_y, coxa_z;
  ik.forwardRaw(angles[0], angles[1] + ik.femurRest(),
                angles[2] + ik.tibiaRest(), coxa_x, coxa_y, coxa_z);

  constexpr float kPi = 3.14159265358979323846f;
  const LegGeometry& geometry = cfg.legs[leg];
  const float yaw = geometry.mount_yaw_cdeg * (kPi / 18000.0f);
  const float a = -(yaw + kPi / 2.0f);
  const float cos_a = cosf(a);
  const float sin_a = sinf(a);
  body_x = geometry.mount_x_dmm / 10.0f +
           cos_a * coxa_x + sin_a * coxa_y;
  body_y = geometry.mount_y_dmm / 10.0f -
           sin_a * coxa_x + cos_a * coxa_y;
  body_z = coxa_z + geometry.mount_z_dmm / 10.0f +
           cfg.geometry.coxa_lift_cmm / 100.0f;
  return true;
}

}  // namespace

// Stand emits a goal for every one of the 18 mapped joints, each within that
// servo's configured travel.
void test_stand_emits_all_mapped_joints_within_travel() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Stand);
  PipelineOutput out;
  pipe.update(20, out);

  TEST_ASSERT_EQUAL_UINT8(kNumServos, out.count);
  for (uint8_t i = 0; i < out.count; ++i) {
    const PipelineJoint& j = out.joints[i];
    const ServoConfig* sc = servoById(cfg, j.id);
    TEST_ASSERT_NOT_NULL(sc);
    TEST_ASSERT_TRUE(j.tick >= sc->min_tick);
    TEST_ASSERT_TRUE(j.tick <= sc->max_tick);
  }
}

void test_default_stand_uses_natural_joint_pose() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Stand);
  PipelineOutput out;
  pipe.update(20, out);

  for (uint8_t i = 0; i < out.count; ++i) {
    const PipelineJoint& joint = out.joints[i];
    // The default integer body height (132 mm) sits 0.27 mm off the exact
    // measured centered stance (131.73 mm), worth ~3 ticks on the knee.
    TEST_ASSERT_UINT16_WITHIN(4, kServoCenterTick, joint.tick);
  }
}

// Seeding present positions on the motion-gate rising edge makes the first
// authorised goals ramp toward the stance instead of snapping in one write,
// with the ramp rate bounded by the Pot1-driven goal slew limit.
void test_seeded_goals_ramp_from_present_pose() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Stand);
  // Speed 0 -> slowest slew (600 ticks/s = 12 ticks per 20 ms cycle).
  pipe.setParams(132, 60, 30, 128, 0);

  // All servos start 400 ticks below center (robot slumped after torque-off).
  for (uint8_t index = 0; index < kNumServos; ++index) {
    pipe.seedGoal(cfg.servos[index].id,
                  static_cast<uint16_t>(kServoCenterTick - 400));
  }

  PipelineOutput first;
  pipe.update(20, first);
  const long max_step = 12;  // 600 ticks/s * 0.02 s
  bool any_moving = false;
  for (uint8_t i = 0; i < first.count; ++i) {
    const long delta = static_cast<long>(first.joints[i].tick) -
                       static_cast<long>(kServoCenterTick - 400);
    TEST_ASSERT_TRUE(delta >= -max_step && delta <= max_step);
    if (delta != 0) any_moving = true;
  }
  TEST_ASSERT_TRUE(any_moving);

  // The ramp converges to the zero-centered CAD home pose.
  PipelineOutput out;
  for (int i = 0; i < 400; ++i) pipe.update(20, out);
  for (uint8_t i = 0; i < out.count; ++i) {
    const ServoConfig* servo = servoById(cfg, out.joints[i].id);
    TEST_ASSERT_NOT_NULL(servo);
    int32_t neutral = static_cast<int32_t>(kServoCenterTick) +
              servo->trim_ticks;
    if (neutral < servo->min_tick) neutral = servo->min_tick;
    if (neutral > servo->max_tick) neutral = servo->max_tick;
    TEST_ASSERT_UINT16_WITHIN(4, neutral, out.joints[i].tick);
  }
}

// The emitted (leg, joint, id) triples match the physical servo map.
void test_joint_ids_match_default_servo_map() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Stand);
  PipelineOutput out;
  pipe.update(20, out);

  TEST_ASSERT_EQUAL_UINT8(kNumServos, out.count);
  uint8_t idx = 0;
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    for (uint8_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const PipelineJoint& j = out.joints[idx++];
      TEST_ASSERT_EQUAL_UINT8(leg, j.leg);
      TEST_ASSERT_EQUAL_UINT8(joint, j.joint);
        TEST_ASSERT_EQUAL_UINT8(
          cfg.servos[leg * kJointsPerLeg + joint].id, j.id);
    }
  }
}

// Advancing a stepping gait with a forward twist changes the goal ticks over
// time (the phase actually drives the legs).
void test_tripod_phase_advance_changes_goals() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  pipe.setTwist(1.0f, 0.0f, 0.0f);  // full forward

  PipelineOutput first;
  pipe.update(0, first);  // dt 0 -> phase stays at 0
  PipelineOutput later;
  for (int i = 0; i < 10; ++i) {
    pipe.update(20, later);  // advance ~200 ms
  }

  bool changed = false;
  for (uint8_t i = 0; i < first.count; ++i) {
    if (first.joints[i].tick != later.joints[i].tick) changed = true;
  }
  TEST_ASSERT_TRUE(changed);
}

// A forward body twist moves the stance feet, producing different goals than
// the zero-twist neutral pose at the same phase.
void test_forward_twist_changes_goals_vs_neutral() {
  RobotConfig cfg = defaultCfg();

  GaitPipeline neutral(cfg);
  neutral.setGait(GaitId::Tripod);
  neutral.setTwist(0.0f, 0.0f, 0.0f);
  PipelineOutput z;
  neutral.update(20, z);

  GaitPipeline forward(cfg);
  forward.setGait(GaitId::Tripod);
  forward.setTwist(1.0f, 0.0f, 0.0f);
  PipelineOutput f;
  forward.update(20, f);

  bool changed = false;
  for (uint8_t i = 0; i < z.count; ++i) {
    if (z.joints[i].tick != f.joints[i].tick) changed = true;
  }
  TEST_ASSERT_TRUE(changed);
}

void test_centered_walking_gait_keeps_home_goals_stable() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  pipe.setTwist(0.0f, 0.0f, 0.0f);

  PipelineOutput first;
  pipe.update(20, first);
  for (int i = 0; i < 50; ++i) {
    PipelineOutput later;
    pipe.update(20, later);
    TEST_ASSERT_EQUAL_UINT8(first.count, later.count);
    for (uint8_t joint = 0; joint < first.count; ++joint) {
      TEST_ASSERT_EQUAL_UINT8(first.joints[joint].id, later.joints[joint].id);
      TEST_ASSERT_EQUAL_UINT16(first.joints[joint].tick,
                               later.joints[joint].tick);
    }
  }
}

// A narrow servo travel saturates the goal tick and the clamp flag is reported
// through the pipeline (clamp telemetry source for the servo_goals stream).
void test_narrow_travel_sets_clamp_flag() {
  RobotConfig cfg = defaultCfg();
  for (uint8_t k = 0; k < kNumServos; ++k) {
    cfg.servos[k].min_tick = 3000;  // well above the ~2048 home ticks
    cfg.servos[k].max_tick = 3005;
  }
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Stand);
  PipelineOutput out;
  pipe.update(20, out);

  bool any_clamped = false;
  for (uint8_t i = 0; i < out.count; ++i) {
    const PipelineJoint& j = out.joints[i];
    TEST_ASSERT_TRUE(j.tick >= 3000);
    TEST_ASSERT_TRUE(j.tick <= 3005);
    if (j.clamped) any_clamped = true;
  }
  TEST_ASSERT_TRUE(any_clamped);
}

// setParams keeps the currently selected gait (it must not snap back to the
// config default gait).
void test_set_params_preserves_selected_gait() {
  RobotConfig cfg = defaultCfg();  // default gait = Stand
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  pipe.setParams(45, 60, 30, 128, 128);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GaitId::Tripod),
                          static_cast<uint8_t>(pipe.engine().gait()));
}

// lmt.7: the pipeline caches body-IK leg geometry by value, so a mutation to the
// referenced config must NOT take effect until reconfigure() is called -- and
// then it must. Shifting each leg's mount Z moves the foot in the coxa frame
// (not absorbed by the IK home offset), so the solved ticks change.
void test_reconfigure_rebuilds_cached_body_ik() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Stand);

  PipelineOutput before;
  pipe.update(20, before);

  // Mutate the referenced geometry WITHOUT reconfiguring: the cached body IK
  // must still produce the original solution.
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    cfg.legs[leg].mount_z_dmm =
        static_cast<int16_t>(cfg.legs[leg].mount_z_dmm + 300);  // +30 mm
  }
  PipelineOutput stale;
  pipe.update(20, stale);
  bool changed_without_reconfig = false;
  for (uint8_t i = 0; i < before.count; ++i) {
    if (before.joints[i].tick != stale.joints[i].tick) {
      changed_without_reconfig = true;
    }
  }
  TEST_ASSERT_FALSE(changed_without_reconfig);

  // After reconfigure the new geometry must change at least one solved tick.
  pipe.reconfigure();
  pipe.setGait(GaitId::Stand);
  PipelineOutput after;
  pipe.update(20, after);
  bool changed_after_reconfig = false;
  for (uint8_t i = 0; i < before.count; ++i) {
    if (before.joints[i].tick != after.joints[i].tick) {
      changed_after_reconfig = true;
    }
  }
  TEST_ASSERT_TRUE(changed_after_reconfig);
}

// lmt.14: with a realistic (large) stride the home stance + stroke extremes
// over-reach the leg workspace. The reach-margin clamp must keep every commanded
// foot reachable (any_unreachable stays false) while reporting any_reach_limited
// when it engages, and the emitted ticks must still sit inside servo travel.
void test_large_stride_is_reach_limited_not_unreachable() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  // Tall stance + max stride + full speed: the stroke extremes over-reach the
  // annulus (the default 132 mm stance with a 60 mm stride does NOT -- see
  // test_cad_geometry's test_default_stride_is_not_reach_limited).
  pipe.setParams(150, 80, 30, 128, 255);
  pipe.setTwist(1.0f, 0.0f, 0.0f);

  bool saw_reach_limit = false;
  for (int i = 0; i < 60; ++i) {  // ~1.2 s, a few full cycles
    PipelineOutput out;
    pipe.update(20, out);
    // The clamp guarantees no commanded foot ever leaves the workspace.
    TEST_ASSERT_FALSE(out.any_unreachable);
    if (out.any_reach_limited) saw_reach_limit = true;
    for (uint8_t k = 0; k < out.count; ++k) {
      const ServoConfig* sc = servoById(cfg, out.joints[k].id);
      TEST_ASSERT_NOT_NULL(sc);
      TEST_ASSERT_TRUE(out.joints[k].tick >= sc->min_tick);
      TEST_ASSERT_TRUE(out.joints[k].tick <= sc->max_tick);
    }
  }
  TEST_ASSERT_TRUE(saw_reach_limit);
}

void assertStanceDirectionPreserved(float vx, float vy, float wz) {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  pipe.setParams(132, cfg.gait.stride_len_mm, 30, 128, 255);
  pipe.setTwist(vx, vy, wz);

  GaitEngine reference;
  reference.configure(cfg.gait);
  GaitDefaults reference_params = cfg.gait;
  reference_params.body_height_mm = 132;
  reference_params.stride_len_mm = cfg.gait.stride_len_mm;
  reference_params.step_height_mm = 30;
  reference_params.duty_x255 = 128;
  reference_params.speed_x255 = 255;
  reference.configure(reference_params);
  reference.setGait(GaitId::Tripod);
  BodyTwist twist;
  twist.vx = vx;
  twist.vy = vy;
  twist.wz = wz;
  reference.setTwist(twist);

  GaitOutput previous_reference;
  PipelineOutput previous_output;
  // Warm up past the walking-stance-bias / reach-limit convergence: during
  // onset the stance centres deliberately slide toward the hips, which is a
  // commanded repositioning, not a trajectory direction error. Steady-state
  // stance sweeps must then never reverse.
  for (uint16_t step = 0; step < 400; ++step) {
    reference.update(5, previous_reference);
    pipe.update(5, previous_output);
  }
  reference.update(5, previous_reference);
  pipe.update(5, previous_output);
  float previous_x[kNumLegs] = {};
  float previous_y[kNumLegs] = {};
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    float z;
    TEST_ASSERT_TRUE(emittedFootBody(cfg, previous_output, leg,
                                    previous_x[leg], previous_y[leg], z));
  }

  uint16_t checked[kNumLegs] = {};
  uint16_t reversed[kNumLegs] = {};
  for (uint16_t step = 0; step < 500; ++step) {
    GaitOutput current_reference;
    PipelineOutput current_output;
    reference.update(5, current_reference);
    pipe.update(5, current_output);
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      float x, y, z;
      TEST_ASSERT_TRUE(emittedFootBody(cfg, current_output, leg, x, y, z));
      if (!previous_reference.feet[leg].swing &&
          !current_reference.feet[leg].swing) {
        const float intended_x = current_reference.feet[leg].x_mm -
                                 previous_reference.feet[leg].x_mm;
        const float intended_y = current_reference.feet[leg].y_mm -
                                 previous_reference.feet[leg].y_mm;
        const float emitted_x = x - previous_x[leg];
        const float emitted_y = y - previous_y[leg];
        const float intended_magnitude =
            sqrtf(intended_x * intended_x + intended_y * intended_y);
        const float emitted_magnitude =
            sqrtf(emitted_x * emitted_x + emitted_y * emitted_y);
        if (intended_magnitude > 1e-3f) {
          const float direction_dot =
              emitted_x * intended_x + emitted_y * intended_y;
          if (direction_dot < 0.0f && emitted_magnitude > 0.02f) {
            ++reversed[leg];
          }
          ++checked[leg];
        }
      }
      previous_x[leg] = x;
      previous_y[leg] = y;
    }
    previous_reference = current_reference;
  }
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_TRUE(checked[leg] > 20);
  }
  TEST_ASSERT_EQUAL_UINT16(0, reversed[0]);
  TEST_ASSERT_EQUAL_UINT16(0, reversed[1]);
  TEST_ASSERT_EQUAL_UINT16(0, reversed[2]);
  TEST_ASSERT_EQUAL_UINT16(0, reversed[3]);
  TEST_ASSERT_EQUAL_UINT16(0, reversed[4]);
  TEST_ASSERT_EQUAL_UINT16(0, reversed[5]);
}

void test_default_forward_stance_never_reverses_after_ik() {
  assertStanceDirectionPreserved(0.0f, 1.0f, 0.0f);
}

void test_default_lateral_stance_never_reverses_after_ik() {
  assertStanceDirectionPreserved(1.0f, 0.0f, 0.0f);
}

void test_default_yaw_stance_never_reverses_after_ik() {
  assertStanceDirectionPreserved(0.0f, 0.0f, 1.0f);
}

// The static home stance is within the reach margin, so it is never flagged.
void test_stand_is_not_reach_limited() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Stand);
  PipelineOutput out;
  pipe.update(20, out);
  TEST_ASSERT_FALSE(out.any_reach_limited);
  TEST_ASSERT_FALSE(out.any_unreachable);
}

// oha.3: a non-neutral body pose shifts/tilts the body over planted (Stand)
// feet, changing the solved goal ticks vs the neutral stance -- this is the
// "move the core without moving the legs" path. A modest pose stays reachable.
void test_body_pose_moves_core_over_planted_feet() {
  RobotConfig cfg = defaultCfg();

  GaitPipeline neutral(cfg);
  neutral.setGait(GaitId::Stand);
  PipelineOutput z;
  neutral.update(20, z);

  GaitPipeline posed(cfg);
  posed.setGait(GaitId::Stand);
  BodyPose pose;
  pose.z_mm = -10.0f;    // lower body (retracts legs -> stays reachable)
  pose.roll = 0.05f;     // gentle roll (~2.9 deg)
  posed.setBodyPose(pose);
  PipelineOutput p;
  posed.update(20, p);

  TEST_ASSERT_EQUAL_UINT8(z.count, p.count);
  TEST_ASSERT_FALSE(p.any_unreachable);
  bool changed = false;
  for (uint8_t i = 0; i < z.count; ++i) {
    if (z.joints[i].tick != p.joints[i].tick) changed = true;
  }
  TEST_ASSERT_TRUE(changed);
}

// oha.3: setting a body pose then clearing it back to neutral restores the
// normal walking path (identical goals to never having posed).
void test_body_pose_neutral_restores_walk_path() {
  RobotConfig cfg = defaultCfg();

  GaitPipeline ref(cfg);
  ref.setGait(GaitId::Stand);
  PipelineOutput want;
  ref.update(20, want);

  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Stand);
  BodyPose pose;
  pose.y_mm = 15.0f;
  pose.roll = 0.1f;
  pipe.setBodyPose(pose);
  PipelineOutput posed;
  pipe.update(20, posed);
  // Now clear back to neutral. The goal slew limiter ramps the return (no
  // snap), so run a few cycles and require convergence to the walk path.
  pipe.setBodyPose(BodyPose{});
  PipelineOutput cleared;
  for (int i = 0; i < 25; ++i) pipe.update(20, cleared);

  TEST_ASSERT_EQUAL_UINT8(want.count, cleared.count);
  for (uint8_t i = 0; i < want.count; ++i) {
    TEST_ASSERT_EQUAL_UINT16(want.joints[i].tick, cleared.joints[i].tick);
  }
}

// The protocol/controller pose envelope is not itself an IK workspace. Even a
// worst-case valid combined pose must be pulled into the safe annulus rather
// than producing unreachable joint targets.
void test_extreme_body_pose_is_reach_limited_not_unreachable() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Stand);
  BodyPose pose;
  pose.x_mm = 50.0f;
  pose.y_mm = -50.0f;
  pose.z_mm = 50.0f;
  pose.roll = 0.4363f;
  pose.pitch = -0.4363f;
  pose.yaw = 0.4363f;
  pipe.setBodyPose(pose);

  PipelineOutput out;
  pipe.update(20, out);
  // The controller clamps a valid command to the HexNav short-link workspace.
  TEST_ASSERT_TRUE(out.any_reach_limited);
  TEST_ASSERT_FALSE(out.any_unreachable);
}

// Regression for the "robot does not walk" collapse: the broken short-tibia
// model used to path-scale a 60 mm commanded stride down to ~10 mm. With the
// measured CAD geometry the default full-stick forward walk must emit real
// steps: a near-full horizontal sweep and a real swing lift on every leg.
void test_default_forward_walk_takes_real_steps() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  pipe.setParams(132, 60, 30, 159, 255);
  pipe.setTwist(0.0f, 1.0f, 0.0f);  // full forward (+Y mechanical front)

  // Let the stance-bias / shape filters converge (~1 s).
  PipelineOutput out;
  for (int i = 0; i < 100; ++i) pipe.update(10, out);

  float min_y[kNumLegs], max_y[kNumLegs], min_z[kNumLegs], max_z[kNumLegs];
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    min_y[leg] = min_z[leg] = 1e9f;
    max_y[leg] = max_z[leg] = -1e9f;
  }
  // > 2 full cycles at max cadence (1.2 Hz).
  for (int i = 0; i < 300; ++i) {
    pipe.update(10, out);
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      float x, y, z;
      TEST_ASSERT_TRUE(emittedFootBody(cfg, out, leg, x, y, z));
      if (y < min_y[leg]) min_y[leg] = y;
      if (y > max_y[leg]) max_y[leg] = y;
      if (z < min_z[leg]) min_z[leg] = z;
      if (z > max_z[leg]) max_z[leg] = z;
    }
  }
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    // The full 60 mm commanded stroke survives (no reach-limit collapse).
    TEST_ASSERT_TRUE(max_y[leg] - min_y[leg] >= 40.0f);
    // Swing lift reaches the configured 30 mm step height.
    TEST_ASSERT_TRUE(max_z[leg] - min_z[leg] >= 25.0f);
  }
}

void test_full_rc_stride_uses_meaningful_coxa_range_on_every_leg() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  pipe.setParams(132, config::kMaxGaitStrideMm, 30, 159, 128);
  pipe.setTwist(0.0f, 1.0f, 0.0f);
  dxl::ServoMap servo_map(cfg);

  PipelineOutput output;
  for (int step = 0; step < 100; ++step) pipe.update(10, output);

  float minimum[kNumLegs];
  float maximum[kNumLegs];
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    minimum[leg] = 1e9f;
    maximum[leg] = -1e9f;
  }
  for (int step = 0; step < 160; ++step) {
    pipe.update(10, output);
    TEST_ASSERT_FALSE(output.any_unreachable);
    for (uint8_t index = 0; index < output.count; ++index) {
      const PipelineJoint& goal = output.joints[index];
      if (goal.joint != static_cast<uint8_t>(JointRole::Coxa)) continue;
      const float angle = servo_map.tickToAngle(goal.leg, goal.joint, goal.tick);
      if (angle < minimum[goal.leg]) minimum[goal.leg] = angle;
      if (angle > maximum[goal.leg]) maximum[goal.leg] = angle;
    }
  }

  constexpr float kRadiansToDegrees = 180.0f / 3.14159265358979323846f;
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    const float excursion_degrees =
        (maximum[leg] - minimum[leg]) * kRadiansToDegrees;
    TEST_ASSERT_TRUE(excursion_degrees >= 3.0f);
  }
}

// Phoenix-style simultaneous control: a body-pose offset applied WHILE a
// forward twist is walking must keep the gait stepping (goals keep changing)
// and stay reachable.
void test_body_pose_overlay_keeps_gait_stepping() {
  RobotConfig cfg = defaultCfg();
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  pipe.setParams(132, 60, 30, 128, 255);
  pipe.setTwist(0.0f, 1.0f, 0.0f);
  BodyPose pose;
  pose.x_mm = 15.0f;
  pose.roll = 0.1f;
  pipe.setBodyPose(pose);

  PipelineOutput prev;
  for (int i = 0; i < 100; ++i) pipe.update(10, prev);
  bool stepping = false;
  for (int i = 0; i < 100; ++i) {
    PipelineOutput cur;
    pipe.update(10, cur);
    TEST_ASSERT_FALSE(cur.any_unreachable);
    for (uint8_t k = 0; k < cur.count; ++k) {
      const int delta = static_cast<int>(cur.joints[k].tick) -
                        static_cast<int>(prev.joints[k].tick);
      if (delta > 5 || delta < -5) stepping = true;
    }
    prev = cur;
  }
  TEST_ASSERT_TRUE(stepping);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_stand_emits_all_mapped_joints_within_travel);
  RUN_TEST(test_default_stand_uses_natural_joint_pose);
  RUN_TEST(test_seeded_goals_ramp_from_present_pose);
  RUN_TEST(test_joint_ids_match_default_servo_map);
  RUN_TEST(test_tripod_phase_advance_changes_goals);
  RUN_TEST(test_forward_twist_changes_goals_vs_neutral);
  RUN_TEST(test_centered_walking_gait_keeps_home_goals_stable);
  RUN_TEST(test_narrow_travel_sets_clamp_flag);
  RUN_TEST(test_set_params_preserves_selected_gait);
  RUN_TEST(test_reconfigure_rebuilds_cached_body_ik);
  RUN_TEST(test_large_stride_is_reach_limited_not_unreachable);
  RUN_TEST(test_default_forward_stance_never_reverses_after_ik);
  RUN_TEST(test_default_lateral_stance_never_reverses_after_ik);
  RUN_TEST(test_default_yaw_stance_never_reverses_after_ik);
  RUN_TEST(test_stand_is_not_reach_limited);
  RUN_TEST(test_body_pose_moves_core_over_planted_feet);
  RUN_TEST(test_body_pose_neutral_restores_walk_path);
  RUN_TEST(test_extreme_body_pose_is_reach_limited_not_unreachable);
  RUN_TEST(test_default_forward_walk_takes_real_steps);
  RUN_TEST(test_full_rc_stride_uses_meaningful_coxa_range_on_every_leg);
  RUN_TEST(test_body_pose_overlay_keeps_gait_stepping);
  return UNITY_END();
}
