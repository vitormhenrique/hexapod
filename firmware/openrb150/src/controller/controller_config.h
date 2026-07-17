#pragma once

// ===========================================================================
// Validated configuration snapshots for ControllerCore (hexapod_src-4ju.5).
//
// EEPROM, Wire, ConfigApi, and task synchronization remain adapter concerns.
// This portable cache accepts only a complete RobotConfig that passes the
// existing schema validator, keeps a stable in-memory address for GaitPipeline,
// and reports a revision change exactly once to its single owner.
// ===========================================================================

#include <stdint.h>

#include "controller_contract.h"

namespace controller {

enum class ConfigSnapshotUpdate : uint8_t {
  Rejected = 0,   // invalid candidate; existing known-good value is retained
  Unchanged = 1,  // valid candidate carries the currently applied revision
  Updated = 2,    // validated value and revision replaced the current snapshot
};

// Build one self-contained controller configuration input. On failure `out` is
// left untouched so an adapter cannot accidentally replace a known-good input
// with invalid data.
bool makeControllerConfigSnapshot(const config::RobotConfig& robot,
                                  uint32_t revision, bool persistent,
                                  ControllerConfigSnapshot& out);

class ConfigSnapshotCache {
 public:
  ConfigSnapshotCache();

  // A revision identifies an immutable, schema-validated ConfigApi shadow.
  // Repeating an already-applied revision is intentionally ignored even if the
  // caller supplies different bytes: adapters must advance revisions when a
  // configuration changes, so this prevents unversioned mutation from reaching
  // cached kinematics, servo limits, or calibration.
  ConfigSnapshotUpdate apply(const config::RobotConfig& robot,
                             uint32_t revision, bool persistent);

  const ControllerConfigSnapshot& snapshot() const { return snapshot_; }

 private:
  ControllerConfigSnapshot snapshot_;
};

}  // namespace controller