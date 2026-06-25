// Safety state machine implementation. See state_machine.h for the contract.

#include "state_machine.h"

#include "command_arbiter.h"  // safety::CommandSource

namespace safety {

namespace {

inline bool isOperational(State s) {
  switch (s) {
    case State::ArmingChecks:
    case State::StandReady:
    case State::RcManual:
    case State::ContactTerrain:
    case State::JetsonAssisted:
      return true;
    default:
      return false;
  }
}

inline bool srcIs(uint8_t src, CommandSource want) {
  return src == static_cast<uint8_t>(want);
}

}  // namespace

bool stateAllowsMotion(State s) {
  switch (s) {
    case State::RcManual:
    case State::ContactTerrain:
    case State::JetsonAssisted:
    case State::MacMaintenance:
      return true;
    default:
      return false;
  }
}

bool stateAllowsTorque(State s) {
  // Torque may be held in StandReady (to keep a stance) and in any motion
  // state, but never in passive pose streaming or while disarmed/faulted.
  if (s == State::StandReady) return true;
  return stateAllowsMotion(s);
}

void StateMachine::reset() {
  state_ = State::Boot;
  reason_ = FaultReason::None;
  clear_fault_requested_ = false;
}

State StateMachine::update(const StateInputs& in, uint32_t /*now_ms*/) {
  // --- 1. Latched hard fault: repeated bus failures / servo HW errors. -----
  // Requires both an explicit operator clear request and the condition to be
  // gone before it releases back to Disarmed.
  if (in.dxl_hard_fault) {
    state_ = State::FaultHard;
    reason_ = FaultReason::DxlHardware;
    clear_fault_requested_ = false;
    return state_;
  }
  if (state_ == State::FaultHard) {
    if (clear_fault_requested_) {
      clear_fault_requested_ = false;
      state_ = State::Disarmed;
      reason_ = FaultReason::None;
    }
    return state_;
  }

  // --- 2. Estop sources (any state). Auto-releases to Disarmed when clear. --
  const bool batt_unsafe = in.battery_valid && in.battery_mv < params_.battery_min_mv;
  const bool failsafe_stop = in.rc_failsafe && (isOperational(state_) || state_ == State::Estop);
  FaultReason estop_reason = FaultReason::None;
  if (in.host_estop) {
    estop_reason = FaultReason::HostEstop;
  } else if (in.rc_kill) {
    estop_reason = FaultReason::RcKill;
  } else if (batt_unsafe) {
    estop_reason = FaultReason::BatteryLow;
  } else if (in.watchdog_fault) {
    estop_reason = FaultReason::Watchdog;
  } else if (failsafe_stop) {
    estop_reason = FaultReason::RcLinkLost;
  }
  if (estop_reason != FaultReason::None) {
    state_ = State::Estop;
    reason_ = estop_reason;
    return state_;
  }
  if (state_ == State::Estop) {
    // All estop sources cleared: require re-arming, so fall back to Disarmed.
    // Settle here for at least one cycle so a still-held arm switch cannot
    // immediately re-enter the arming sequence on the same update.
    state_ = State::Disarmed;
    reason_ = FaultReason::None;
    return state_;
  }

  // --- 3. Normal progression. ----------------------------------------------
  // Host force-disarm: a SET_ARMING(disarm) drops any operational/maintenance/
  // passive state straight back to Disarmed. It only ever reduces authority, so
  // it is honored unconditionally (RC still owns re-arming).
  if (in.host_disarm) {
    switch (state_) {
      case State::ArmingChecks:
      case State::StandReady:
      case State::RcManual:
      case State::ContactTerrain:
      case State::JetsonAssisted:
      case State::MacMaintenance:
      case State::PassivePoseStream:
        state_ = State::Disarmed;
        reason_ = FaultReason::None;
        return state_;
      default:
        break;
    }
  }

  switch (state_) {
    case State::Boot:
      state_ = State::ConfigLoad;
      break;

    case State::ConfigLoad:
      if (in.config_loaded) state_ = State::Disarmed;
      break;

    case State::Disarmed:
      if (in.passive_request && in.torque_off) {
        state_ = State::PassivePoseStream;
      } else if (in.maintenance_request && in.mac_lock_held) {
        state_ = State::MacMaintenance;
      } else if (in.rc_armed) {
        state_ = State::ArmingChecks;
      }
      break;

    case State::ArmingChecks:
      if (!in.rc_armed) {
        state_ = State::Disarmed;
      } else if (in.arming_checks_pass) {
        state_ = State::StandReady;
      }
      break;

    case State::StandReady:
      if (!in.rc_armed) {
        state_ = State::Disarmed;
      } else if (in.maintenance_request && in.mac_lock_held) {
        state_ = State::MacMaintenance;
      } else if (in.jetson_fresh && in.rc_autonomy &&
                 srcIs(in.command_source, CommandSource::Jetson)) {
        state_ = State::JetsonAssisted;
      } else if (in.contact_enabled && in.contact_confident &&
                 srcIs(in.command_source, CommandSource::Rc)) {
        state_ = State::ContactTerrain;
      } else if (srcIs(in.command_source, CommandSource::Rc)) {
        state_ = State::RcManual;
      }
      break;

    case State::RcManual:
      if (!in.rc_armed) {
        state_ = State::Disarmed;
      } else if (in.jetson_fresh && in.rc_autonomy &&
                 srcIs(in.command_source, CommandSource::Jetson)) {
        state_ = State::JetsonAssisted;
      } else if (in.contact_enabled && in.contact_confident) {
        state_ = State::ContactTerrain;
      } else if (!srcIs(in.command_source, CommandSource::Rc)) {
        state_ = State::StandReady;
      }
      break;

    case State::ContactTerrain:
      if (!in.rc_armed) {
        state_ = State::Disarmed;
      } else if (!(in.contact_enabled && in.contact_confident)) {
        state_ = State::RcManual;  // lost contact confidence: nominal gait
      } else if (in.jetson_fresh && in.rc_autonomy &&
                 srcIs(in.command_source, CommandSource::Jetson)) {
        state_ = State::JetsonAssisted;
      }
      break;

    case State::JetsonAssisted:
      if (!in.rc_armed) {
        state_ = State::Disarmed;
      } else if (!(in.jetson_fresh && in.rc_autonomy)) {
        state_ = State::StandReady;  // lost Jetson heartbeat / autonomy grant
      }
      break;

    case State::MacMaintenance:
      if (!in.maintenance_request || !in.mac_lock_held) {
        state_ = State::Disarmed;
      }
      break;

    case State::PassivePoseStream:
      if (!in.passive_request) {
        state_ = State::Disarmed;
      }
      break;

    case State::FaultSoft:
      // Reserved for future recoverable soft faults; clear to Disarmed.
      if (clear_fault_requested_) {
        clear_fault_requested_ = false;
        state_ = State::Disarmed;
        reason_ = FaultReason::None;
      }
      break;

    case State::FaultHard:
    case State::Estop:
      // Handled above.
      break;
  }
  return state_;
}

}  // namespace safety
