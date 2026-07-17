// Native tests for validated ControllerCore configuration snapshots.
// Run with: pio test -e native -f test_controller_config

#include <unity.h>

#include "../../src/controller/controller_config.h"
#include "../../src/gait/gait_pipeline.h"

using controller::ConfigSnapshotCache;
using controller::ConfigSnapshotUpdate;

namespace {

config::RobotConfig defaultConfig() {
  config::RobotConfig robot;
  config::defaultRobotConfig(robot);
  return robot;
}

}  // namespace

void test_default_snapshot_preserves_compiled_safe_defaults() {
  const config::RobotConfig defaults = defaultConfig();
  ConfigSnapshotCache cache;

  TEST_ASSERT_TRUE(cache.snapshot().valid);
  TEST_ASSERT_FALSE(cache.snapshot().persistent);
  TEST_ASSERT_EQUAL_UINT32(0, cache.snapshot().revision);
  TEST_ASSERT_EQUAL_UINT16(defaults.gait.body_height_mm,
                           cache.snapshot().robot.gait.body_height_mm);
  TEST_ASSERT_EQUAL_UINT8(defaults.gait.gait,
                          cache.snapshot().robot.gait.gait);
  TEST_ASSERT_EQUAL_UINT8(defaults.servos[0].id,
                          cache.snapshot().robot.servos[0].id);
}

void test_revision_change_reseeds_pipeline_from_snapshot_config() {
  const config::RobotConfig defaults = defaultConfig();
  ConfigSnapshotCache cache;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConfigSnapshotUpdate::Updated),
      static_cast<uint8_t>(cache.apply(defaults, 1, false)));

  gait::GaitPipeline pipeline(cache.snapshot().robot);
  TEST_ASSERT_EQUAL_UINT8(defaults.gait.gait,
                          static_cast<uint8_t>(pipeline.engine().gait()));

  config::RobotConfig changed = defaults;
  changed.gait.gait = static_cast<uint8_t>(config::GaitId::Ripple);
  changed.gait.body_height_mm = 55;
  TEST_ASSERT_TRUE(config::validateRobotConfig(changed));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConfigSnapshotUpdate::Updated),
      static_cast<uint8_t>(cache.apply(changed, 2, true)));

  // The cache owns a stable RobotConfig address, so GaitPipeline sees the new
  // geometry/defaults when its caller reacts to the explicit revision update.
  pipeline.reconfigure();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Ripple),
                          static_cast<uint8_t>(pipeline.engine().gait()));
  TEST_ASSERT_EQUAL_UINT16(55, cache.snapshot().robot.gait.body_height_mm);
  TEST_ASSERT_TRUE(cache.snapshot().persistent);
}

void test_invalid_or_unversioned_config_never_replaces_known_good_snapshot() {
  const config::RobotConfig defaults = defaultConfig();
  ConfigSnapshotCache cache;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConfigSnapshotUpdate::Updated),
      static_cast<uint8_t>(cache.apply(defaults, 5, false)));

  config::RobotConfig same_revision = defaults;
  same_revision.gait.gait = static_cast<uint8_t>(config::GaitId::Crawl);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConfigSnapshotUpdate::Unchanged),
      static_cast<uint8_t>(cache.apply(same_revision, 5, true)));
  TEST_ASSERT_EQUAL_UINT8(defaults.gait.gait,
                          cache.snapshot().robot.gait.gait);
  TEST_ASSERT_FALSE(cache.snapshot().persistent);

  config::RobotConfig invalid = defaults;
  invalid.servos[0].min_tick = 2000;
  invalid.servos[0].max_tick = 1999;
  TEST_ASSERT_FALSE(config::validateRobotConfig(invalid));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConfigSnapshotUpdate::Rejected),
      static_cast<uint8_t>(cache.apply(invalid, 6, true)));
  TEST_ASSERT_EQUAL_UINT32(5, cache.snapshot().revision);
  TEST_ASSERT_EQUAL_UINT8(defaults.gait.gait,
                          cache.snapshot().robot.gait.gait);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_default_snapshot_preserves_compiled_safe_defaults);
  RUN_TEST(test_revision_change_reseeds_pipeline_from_snapshot_config);
  RUN_TEST(test_invalid_or_unversioned_config_never_replaces_known_good_snapshot);
  return UNITY_END();
}