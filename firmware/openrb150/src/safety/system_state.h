#pragma once

// ===========================================================================
// Safety state machine state IDs (AGENTS.md 5.3).
//
// This header defines the shared vocabulary only. The transition logic is owned
// by the control/safety code in a later Phase 1/2 task; for now the system
// stays in DISARMED with DYNAMIXEL power off. The numeric values are part of
// the wire protocol (reported in GET_STATUS / HEARTBEAT), so do not renumber.
// ===========================================================================

#include <stdint.h>

namespace safety {

enum class State : uint8_t {
  Boot = 0,
  ConfigLoad = 1,
  Disarmed = 2,
  ArmingChecks = 3,
  StandReady = 4,
  RcManual = 5,
  ContactTerrain = 6,
  JetsonAssisted = 7,
  MacMaintenance = 8,
  PassivePoseStream = 9,
  FaultSoft = 10,
  FaultHard = 11,
  Estop = 12,
};

}  // namespace safety
