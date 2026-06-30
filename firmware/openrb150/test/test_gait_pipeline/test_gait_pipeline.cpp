// Native (host) unit tests for the gait -> servo goal pipeline (lmt.1).
// Run with: pio test -e native

#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/gait/gait_pipeline.h"

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

// The emitted (leg, joint, id) triples match the default servo map order
// (id = joint*6 + leg + 1), so dxlTask can Sync Write them directly.
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
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(joint * kNumLegs + leg + 1),
                              j.id);
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
  neutral.update(0, z);

  GaitPipeline forward(cfg);
  forward.setGait(GaitId::Tripod);
  forward.setTwist(1.0f, 0.0f, 0.0f);
  PipelineOutput f;
  forward.update(0, f);

  bool changed = false;
  for (uint8_t i = 0; i < z.count; ++i) {
    if (z.joints[i].tick != f.joints[i].tick) changed = true;
  }
  TEST_ASSERT_TRUE(changed);
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
  // Max stride, modest lift, full speed, forward twist: drives the stroke
  // extremes well past the near-boundary home stance.
  pipe.setParams(40, 80, 30, 128, 255);
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
  // Now clear back to neutral.
  pipe.setBodyPose(BodyPose{});
  PipelineOutput cleared;
  pipe.update(20, cleared);

  TEST_ASSERT_EQUAL_UINT8(want.count, cleared.count);
  for (uint8_t i = 0; i < want.count; ++i) {
    TEST_ASSERT_EQUAL_UINT16(want.joints[i].tick, cleared.joints[i].tick);
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_stand_emits_all_mapped_joints_within_travel);
  RUN_TEST(test_joint_ids_match_default_servo_map);
  RUN_TEST(test_tripod_phase_advance_changes_goals);
  RUN_TEST(test_forward_twist_changes_goals_vs_neutral);
  RUN_TEST(test_narrow_travel_sets_clamp_flag);
  RUN_TEST(test_set_params_preserves_selected_gait);
  RUN_TEST(test_reconfigure_rebuilds_cached_body_ik);
  RUN_TEST(test_large_stride_is_reach_limited_not_unreachable);
  RUN_TEST(test_stand_is_not_reach_limited);
  RUN_TEST(test_body_pose_moves_core_over_planted_feet);
  RUN_TEST(test_body_pose_neutral_restores_walk_path);
  return UNITY_END();
}
