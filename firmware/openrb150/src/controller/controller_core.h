#pragma once

// ===========================================================================
// Deterministic portable controller orchestration (hexapod_src-4ju.6).
//
// ControllerCore owns the algorithmic state currently distributed through
// controlTask: command authority, safety transitions, configuration-derived
// gait/IK state, trick state, and output diagnostics. Adapters own every
// peripheral, task, lock, protocol parser, and actuator side effect.
// ===========================================================================

#include <stdint.h>

#include "../dxl/servo_map.h"
#include "../gait/trick_engine.h"
#include "body_command_shaper.h"
#include "controller_config.h"

namespace controller {

// RC three-position gait switch: low/centre/high select only moving gait
// families. Stand and Sit are robot states, not walking-pattern choices.
config::GaitId rcGaitFromIndex(uint8_t gait_index);

class ControllerCore {
 public:
  ControllerCore();

  // Reset controller-owned state without touching any adapter-owned I/O or
  // replacing the active configuration snapshot.
  void reset();

  // Configure safety-state tunables (idle auto-disarm, battery thresholds).
  // Firmware keeps the conservative defaults; SIL adapters may disable
  // hardware-oriented timeouts that make no sense without physical servos.
  // Surviving reset() is intentional: StateMachine::reset() preserves params.
  void configureSafety(const safety::StateParams& params) {
    state_machine_.configure(params);
  }

  // Bounded, non-blocking, allocation-free controller step. Input objects are
  // immutable snapshots; output is fully overwritten on every invocation.
  void step(const RobotState& state, const ControllerIntent& intent,
            const ControllerConfigSnapshot& config,
            const ControllerTime& time, RobotCommand& command);

  safety::FaultReason lastFaultReason() const {
    return state_machine_.lastFaultReason();
  }
  uint32_t lastFaultTimestampMs() const {
    return state_machine_.lastFaultTimestampMs();
  }

 private:
  ConfigSnapshotCache config_cache_;
  gait::GaitPipeline pipeline_;
  dxl::ServoMap servo_map_;
  safety::CommandArbiter arbiter_;
  safety::StateMachine state_machine_;
  gait::TrickEngine trick_engine_;
  BodyCommandShaper body_command_shaper_;

  uint32_t applied_intent_sequence_ = 0xFFFFFFFFu;
  uint8_t applied_gait_ = 0xFF;
  uint32_t idle_seen_intent_sequence_ = 0xFFFFFFFFu;
  controller::TrickId previous_rc_trick_ = controller::TrickId::None;
  bool previous_motion_gate_ = false;
  bool previous_maintenance_authority_ = false;
};

}  // namespace controller