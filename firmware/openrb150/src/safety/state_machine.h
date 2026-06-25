#pragma once

// ===========================================================================
// Firmware safety state machine (AGENTS.md 5.3).
//
// This is a PORTABLE module: it contains no Arduino includes and is unit-tested
// on the host. The control task feeds it a snapshot of the latest health, RC,
// arbiter, and feature inputs each cycle and reads back the authoritative
// system State plus a fault reason. The numeric State values come from
// system_state.h and are part of the wire protocol, so they must not change.
//
// The machine is deterministic and side-effect free: it only computes the next
// state. The caller is responsible for acting on it (gating torque, goal
// writes, etc.). Motion and torque are only permitted in the states reported by
// stateAllowsMotion()/stateAllowsTorque().
// ===========================================================================

#include <stdint.h>

#include "system_state.h"

namespace safety {

// Why the machine is (or last was) in a fault/estop state. Reported in
// telemetry so an operator can see the cause of a safe stop.
enum class FaultReason : uint8_t {
  None = 0,
  RcKill = 1,
  HostEstop = 2,
  RcLinkLost = 3,
  BatteryLow = 4,
  Watchdog = 5,
  DxlHardware = 6,
};

// Tunable thresholds. Defaults are conservative; the config layer may override.
struct StateParams {
  uint16_t battery_min_mv = 10000;  // below this (when valid) -> Estop
};

// One cycle of inputs sampled from the rest of the firmware. All fields are
// plain values copied from cross-task snapshots so the machine never blocks.
struct StateInputs {
  // Boot / config.
  bool config_loaded = false;  // a config slot (or safe default) is loaded

  // Health.
  uint16_t battery_mv = 0;
  bool battery_valid = false;  // true only when a real pack reading exists
  bool watchdog_fault = false;
  bool dxl_hard_fault = false;  // repeated bus failures / servo HW error

  // Estop / kill sources.
  bool host_estop = false;
  bool rc_kill = false;
  bool rc_failsafe = false;  // RC link lost

  // Arming.
  bool rc_ever_seen = false;
  bool rc_armed = false;          // RC arm switch asserted
  bool arming_checks_pass = false;  // battery + DXL scan + config + pose ok

  // Command authority (from CommandArbiter).
  uint8_t command_source = 0;  // safety::CommandSource value
  bool jetson_fresh = false;
  bool rc_autonomy = false;  // RC grants autonomy authority
  bool mac_lock_held = false;
  bool maintenance_request = false;  // explicit enter-maintenance command

  // Passive pose streaming.
  bool passive_request = false;
  bool torque_off = false;  // all servo torque confirmed off

  // Contact-aware terrain.
  bool contact_enabled = false;
  bool contact_confident = false;
};

// Returns true when servo goal writes are permitted in the given state.
bool stateAllowsMotion(State s);

// Returns true when servo torque may be enabled in the given state. Passive
// pose streaming is explicitly excluded (torque must stay off there).
bool stateAllowsTorque(State s);

class StateMachine {
 public:
  void configure(const StateParams& params) { params_ = params; }

  // Reset to the power-on state. Call once at task start.
  void reset();

  // Advance one cycle and return the resulting state.
  State update(const StateInputs& in, uint32_t now_ms);

  // Operator request to clear a latched hard fault. Takes effect on the next
  // update() if the underlying fault condition has cleared.
  void requestClearFault() { clear_fault_requested_ = true; }

  State state() const { return state_; }
  FaultReason faultReason() const { return reason_; }

 private:
  StateParams params_{};
  State state_ = State::Boot;
  FaultReason reason_ = FaultReason::None;
  bool clear_fault_requested_ = false;
};

}  // namespace safety
