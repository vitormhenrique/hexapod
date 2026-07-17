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

// States eligible for the idle auto-disarm: armed, torque-holding, operator-
// driven. ArmingChecks has its own bounded timeout; MacMaintenance sessions
// are governed by the maintenance-lock TTL; PassivePoseStream is torque-off
// by contract and explicitly entered.
inline bool idleDisarmEligible(State s) {
  switch (s) {
    case State::StandReady:
    case State::RcManual:
    case State::ContactTerrain:
    case State::JetsonAssisted:
      return true;
    default:
      return false;
  }
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

bool stateAllowsDxlPower(State s) {
  return s == State::ArmingChecks || s == State::PassivePoseStream ||
         stateAllowsTorque(s);
}

void StateMachine::reset() {
  state_ = State::Boot;
  reason_ = FaultReason::None;
  last_fault_reason_ = FaultReason::None;
  last_fault_timestamp_ms_ = 0;
  clear_fault_requested_ = false;
  batt_low_active_ = false;
  batt_low_since_ms_ = 0;
  arming_since_ms_ = 0;
  last_activity_ms_ = 0;
  arm_release_required_ = true;
}

void StateMachine::latchFault(FaultReason reason, uint32_t now_ms) {
  if (reason == FaultReason::None || reason == reason_) return;
  last_fault_reason_ = reason;
  last_fault_timestamp_ms_ = now_ms;
}

State StateMachine::update(const StateInputs& in, uint32_t now_ms) {
  // Arming is edge-qualified: after boot, any operational state, or a fault /
  // E-stop, the operator must return the physical arm switch low before a new
  // arm request is accepted. This prevents a held switch from automatically
  // restarting arming after CLEAR_FAULT or an E-stop source disappears.
  if (state_ != State::Boot && state_ != State::ConfigLoad &&
      state_ != State::Disarmed && in.rc_armed) {
    arm_release_required_ = true;
  }

  // --- 1. Latched hard fault: repeated bus failures / servo HW errors. -----
  // Requires both an explicit operator clear request and the condition to be
  // gone before it releases back to Disarmed.
  if (in.dxl_hard_fault) {
    arm_release_required_ = true;
    state_ = State::FaultHard;
    latchFault(FaultReason::DxlHardware, now_ms);
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
  // Battery-low is debounced: the reading must stay below the cutoff for
  // battery_low_debounce_ms of consecutive updates before it counts. DXL
  // power-on inrush from 18 servos sags the supply/ADC sense for a few
  // control cycles, and a single-sample Estop tore down maintenance sessions
  // the moment DXL power was enabled (HIL bug 7, hexapod-hil passive-pose).
  const bool batt_low_now =
      in.battery_valid && in.battery_mv < params_.battery_min_mv;
  if (batt_low_now) {
    if (!batt_low_active_) {
      batt_low_active_ = true;
      batt_low_since_ms_ = now_ms;
    }
  } else {
    batt_low_active_ = false;
  }
  const bool batt_unsafe =
      batt_low_active_ &&
      (now_ms - batt_low_since_ms_) >= params_.battery_low_debounce_ms;
  // Failsafe counts only once an RC link has existed (same rationale as the
  // rc_kill guard below): with no receiver attached the bridge holds failsafe
  // permanently, and without the ever_seen gate any transient Estop entry
  // (host ESTOP, boot fault) would re-latch as RcLinkLost every cycle and make
  // CLEAR_FAULT useless on the bench (AGENTS.md mode 4). A link that existed
  // and then dropped keeps ever_seen latched, so in-flight loss still stops.
  const bool failsafe_stop = in.rc_failsafe && in.rc_ever_seen &&
                             (isOperational(state_) || state_ == State::Estop);
  FaultReason estop_reason = FaultReason::None;
  if (in.host_estop) {
    estop_reason = FaultReason::HostEstop;
  } else if (in.rc_kill && in.rc_ever_seen) {
    // RC kill counts only once an RC link has existed: the bridge's failsafe
    // hold synthesises kill when no receiver is attached at all, which must
    // not lock a bench robot out of Disarmed (Mac development mode needs
    // maintenance/passive entry with no RC transmitter, AGENTS.md mode 4).
    // Arming still requires the RC arm switch, so motion stays impossible
    // without a live link; a link that drops mid-operation keeps ever_seen
    // latched, so the in-flight kill/failsafe path is unchanged.
    estop_reason = FaultReason::RcKill;
  } else if (batt_unsafe) {
    estop_reason = FaultReason::BatteryLow;
  } else if (in.watchdog_fault) {
    estop_reason = FaultReason::Watchdog;
  } else if (failsafe_stop) {
    estop_reason = FaultReason::RcLinkLost;
  }
  if (estop_reason != FaultReason::None) {
    arm_release_required_ = true;
    state_ = State::Estop;
    latchFault(estop_reason, now_ms);
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
  // Idle auto-disarm (power saving): an armed robot holding torque with no
  // motion activity for idle_disarm_ms drops to Disarmed, which cuts DXL
  // power via the state policy. Entering an armed state or any activity
  // refreshes the timer; re-powering requires a full re-arm (the arm switch
  // release edge is enforced below) or an explicit maintenance power command.
  if (in.motion_active || !idleDisarmEligible(state_)) {
    last_activity_ms_ = now_ms;
  }
  if (params_.idle_disarm_ms != 0 && idleDisarmEligible(state_) &&
      (now_ms - last_activity_ms_) >= params_.idle_disarm_ms) {
    arm_release_required_ = true;
    state_ = State::Disarmed;
    reason_ = FaultReason::None;
    return state_;
  }

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
        arm_release_required_ = true;
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
      if (!in.rc_armed) arm_release_required_ = false;
      if (in.host_disarm) {
        break;
      } else if (in.passive_request && in.torque_off) {
        state_ = State::PassivePoseStream;
      } else if (in.maintenance_request && in.mac_lock_held) {
        state_ = State::MacMaintenance;
      } else if (in.rc_armed && !arm_release_required_) {
        state_ = State::ArmingChecks;
        reason_ = FaultReason::None;
        arming_since_ms_ = now_ms;
      }
      break;

    case State::ArmingChecks:
      if (!in.rc_armed) {
        state_ = State::Disarmed;
      } else if (in.arming_checks_pass && in.battery_valid &&
                 in.battery_mv >= params_.battery_min_mv) {
        state_ = State::StandReady;
      } else if ((now_ms - arming_since_ms_) >= params_.arming_timeout_ms) {
        arm_release_required_ = true;
        state_ = State::FaultSoft;
        latchFault(FaultReason::ArmingTimeout, now_ms);
        reason_ = FaultReason::ArmingTimeout;
      }
      break;

    case State::StandReady:
      if (!in.rc_armed) {
        state_ = State::Disarmed;
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
      } else if (!srcIs(in.command_source, CommandSource::Rc)) {
        state_ = State::StandReady;
      } else if (in.contact_enabled && in.contact_confident) {
        state_ = State::ContactTerrain;
      }
      break;

    case State::ContactTerrain:
      if (!in.rc_armed) {
        state_ = State::Disarmed;
      } else if (in.jetson_fresh && in.rc_autonomy &&
                 srcIs(in.command_source, CommandSource::Jetson)) {
        state_ = State::JetsonAssisted;
      } else if (!srcIs(in.command_source, CommandSource::Rc)) {
        state_ = State::StandReady;
      } else if (!(in.contact_enabled && in.contact_confident)) {
        state_ = State::RcManual;  // lost contact confidence: nominal gait
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
      if (in.passive_request && in.torque_off) {
        // Direct maintenance -> passive handoff (hexapod_src-nkb): the bench
        // workflow is maintenance (power DXL, scan, torque off) then passive
        // streaming. Going through Disarmed would cut DXL power and lose the
        // present-position reads passive mode exists for (AGENTS.md 5.5).
        state_ = State::PassivePoseStream;
      } else if (!in.maintenance_request || !in.mac_lock_held) {
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
