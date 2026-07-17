#include "controller_config.h"

namespace controller {

bool makeControllerConfigSnapshot(const config::RobotConfig& robot,
                                  uint32_t revision, bool persistent,
                                  ControllerConfigSnapshot& out) {
  if (!config::validateRobotConfig(robot)) return false;

  out.robot = robot;
  out.revision = revision;
  out.valid = true;
  out.persistent = persistent;
  return true;
}

ConfigSnapshotCache::ConfigSnapshotCache() {
  config::defaultRobotConfig(snapshot_.robot);
  snapshot_.valid = config::validateRobotConfig(snapshot_.robot);
  snapshot_.revision = 0;
  snapshot_.persistent = false;
}

ConfigSnapshotUpdate ConfigSnapshotCache::apply(
    const config::RobotConfig& robot, uint32_t revision, bool persistent) {
  if (!config::validateRobotConfig(robot)) {
    return ConfigSnapshotUpdate::Rejected;
  }
  if (snapshot_.valid && revision == snapshot_.revision) {
    return ConfigSnapshotUpdate::Unchanged;
  }

  snapshot_.robot = robot;
  snapshot_.revision = revision;
  snapshot_.valid = true;
  snapshot_.persistent = persistent;
  return ConfigSnapshotUpdate::Updated;
}

}  // namespace controller