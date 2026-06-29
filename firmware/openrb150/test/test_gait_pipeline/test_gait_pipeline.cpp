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

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_stand_emits_all_mapped_joints_within_travel);
  RUN_TEST(test_joint_ids_match_default_servo_map);
  RUN_TEST(test_tripod_phase_advance_changes_goals);
  RUN_TEST(test_forward_twist_changes_goals_vs_neutral);
  RUN_TEST(test_narrow_travel_sets_clamp_flag);
  RUN_TEST(test_set_params_preserves_selected_gait);
  RUN_TEST(test_reconfigure_rebuilds_cached_body_ik);
  return UNITY_END();
}
