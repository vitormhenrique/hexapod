// Native (host) unit tests for the safety state machine. No Arduino deps.
// Run with: pio test -e native -f test_state_machine

#include <unity.h>

#include "../../src/safety/command_arbiter.h"
#include "../../src/safety/state_machine.h"

using namespace safety;

namespace {

StateMachine makeMachine() {
  StateMachine m;
  StateParams p;
  p.battery_min_mv = 10000;
  m.configure(p);
  m.reset();
  return m;
}

// A healthy baseline input set: config loaded, good battery, no faults.
StateInputs healthy() {
  StateInputs in;
  in.config_loaded = true;
  in.battery_mv = 12000;
  in.battery_valid = true;
  return in;
}

// Drive Boot -> ... -> StandReady with an armed, passing RC.
StateMachine atStandReady() {
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  in.rc_ever_seen = true;
  in.rc_armed = true;
  in.arming_checks_pass = true;
  m.update(in, 0);  // Boot -> ConfigLoad
  m.update(in, 10); // ConfigLoad -> Disarmed
  m.update(in, 20); // Disarmed -> ArmingChecks
  m.update(in, 30); // ArmingChecks -> StandReady
  return m;
}

}  // namespace

void test_boot_progresses_to_disarmed() {
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  TEST_ASSERT_EQUAL(State::ConfigLoad, m.update(in, 0));
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 10));
}

void test_config_not_loaded_holds_in_configload() {
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  in.config_loaded = false;
  m.update(in, 0);  // Boot -> ConfigLoad
  TEST_ASSERT_EQUAL(State::ConfigLoad, m.update(in, 10));
  in.config_loaded = true;
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 20));
}

void test_disarmed_requires_arm_for_arming_checks() {
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  m.update(in, 0);
  m.update(in, 10);  // Disarmed
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 20));  // no arm -> stay
  in.rc_armed = true;
  TEST_ASSERT_EQUAL(State::ArmingChecks, m.update(in, 30));
}

void test_arming_checks_gate_stand_ready() {
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  in.rc_armed = true;
  m.update(in, 0);
  m.update(in, 10);
  m.update(in, 20);  // ArmingChecks
  TEST_ASSERT_EQUAL(State::ArmingChecks, m.update(in, 30));  // checks not passed
  in.arming_checks_pass = true;
  TEST_ASSERT_EQUAL(State::StandReady, m.update(in, 40));
}

void test_arming_checks_timeout_reports_soft_fault() {
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  in.rc_armed = true;
  m.update(in, 0);
  m.update(in, 10);
  TEST_ASSERT_EQUAL(State::ArmingChecks, m.update(in, 20));
  TEST_ASSERT_EQUAL(State::ArmingChecks, m.update(in, 10019));
  TEST_ASSERT_EQUAL(State::FaultSoft, m.update(in, 10020));
  TEST_ASSERT_EQUAL(FaultReason::ArmingTimeout, m.faultReason());
}

void test_dxl_power_allowed_during_arming_and_operation() {
  TEST_ASSERT_FALSE(stateAllowsDxlPower(State::Disarmed));
  TEST_ASSERT_TRUE(stateAllowsDxlPower(State::ArmingChecks));
  TEST_ASSERT_FALSE(stateAllowsTorque(State::ArmingChecks));
  TEST_ASSERT_TRUE(stateAllowsDxlPower(State::StandReady));
  TEST_ASSERT_TRUE(stateAllowsDxlPower(State::RcManual));
  TEST_ASSERT_TRUE(stateAllowsDxlPower(State::PassivePoseStream));
  TEST_ASSERT_FALSE(stateAllowsTorque(State::PassivePoseStream));
  TEST_ASSERT_FALSE(stateAllowsDxlPower(State::Estop));
  TEST_ASSERT_FALSE(stateAllowsDxlPower(State::FaultHard));
}

void test_stand_ready_to_rc_manual() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.arming_checks_pass = true;
  in.command_source = static_cast<uint8_t>(CommandSource::Rc);
  TEST_ASSERT_EQUAL(State::RcManual, m.update(in, 50));
  TEST_ASSERT_TRUE(stateAllowsMotion(State::RcManual));
}

void test_rc_manual_to_contact_terrain_and_back() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.command_source = static_cast<uint8_t>(CommandSource::Rc);
  m.update(in, 50);  // RcManual
  in.contact_enabled = true;
  in.contact_confident = true;
  TEST_ASSERT_EQUAL(State::ContactTerrain, m.update(in, 60));
  // Losing confidence falls back to nominal gait, not a stop.
  in.contact_confident = false;
  TEST_ASSERT_EQUAL(State::RcManual, m.update(in, 70));
}

void test_stand_ready_to_jetson_assisted() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.command_source = static_cast<uint8_t>(CommandSource::Jetson);
  in.jetson_fresh = true;
  in.rc_autonomy = true;
  TEST_ASSERT_EQUAL(State::JetsonAssisted, m.update(in, 50));
  // Stale heartbeat drops back to StandReady.
  in.jetson_fresh = false;
  TEST_ASSERT_EQUAL(State::StandReady, m.update(in, 60));
}

void test_disarm_returns_from_motion() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.command_source = static_cast<uint8_t>(CommandSource::Rc);
  m.update(in, 50);  // RcManual
  in.rc_armed = false;
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 60));
}

void test_kill_forces_estop_and_recovers() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_ever_seen = true;  // kill comes from a live RC link
  in.rc_armed = true;
  in.command_source = static_cast<uint8_t>(CommandSource::Rc);
  m.update(in, 50);  // RcManual
  in.rc_kill = true;
  TEST_ASSERT_EQUAL(State::Estop, m.update(in, 60));
  TEST_ASSERT_EQUAL(FaultReason::RcKill, m.faultReason());
  // Releasing kill recovers to Disarmed (must re-arm).
  in.rc_kill = false;
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 70));
}

void test_kill_without_rc_ever_seen_stays_disarmed() {
  // Bench robot with no RC receiver: the bridge's failsafe hold synthesises
  // kill, but with no RC link ever seen the FSM must settle in Disarmed so
  // Mac maintenance / passive pose / DXL power stay reachable (AGENTS.md
  // mode 4). Arming still requires the RC arm switch, so no motion is
  // possible.
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  in.rc_kill = true;
  in.rc_failsafe = true;
  in.rc_ever_seen = false;
  m.update(in, 0);  // Boot -> ConfigLoad
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 10));
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 20));
  TEST_ASSERT_EQUAL(FaultReason::None, m.faultReason());
  // Once an RC link has existed, the same kill input is honoured.
  in.rc_ever_seen = true;
  TEST_ASSERT_EQUAL(State::Estop, m.update(in, 30));
  TEST_ASSERT_EQUAL(FaultReason::RcKill, m.faultReason());
}

void test_host_estop_forces_estop() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.host_estop = true;
  TEST_ASSERT_EQUAL(State::Estop, m.update(in, 50));
  TEST_ASSERT_EQUAL(FaultReason::HostEstop, m.faultReason());
}

void test_estop_recovers_on_bench_with_failsafe_held() {
  // Bench robot with no RC receiver: the bridge holds failsafe=true with
  // ever_seen=false. A host ESTOP must still be recoverable -- without the
  // ever_seen gate on failsafe_stop, the held failsafe re-latched the Estop
  // state as RcLinkLost every cycle and CLEAR_FAULT could never release it
  // (hexapod_src-jqy, found by hexapod-hil mode-safety).
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  in.rc_failsafe = true;
  in.rc_ever_seen = false;
  m.update(in, 0);  // Boot -> ConfigLoad
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 10));
  in.host_estop = true;
  TEST_ASSERT_EQUAL(State::Estop, m.update(in, 20));
  TEST_ASSERT_EQUAL(FaultReason::HostEstop, m.faultReason());
  // Host estop released: must fall back to Disarmed, not RcLinkLost.
  in.host_estop = false;
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 30));
  TEST_ASSERT_EQUAL(FaultReason::None, m.faultReason());
  // Once a link has existed, the same held failsafe does keep Estop latched.
  in.host_estop = true;
  m.update(in, 40);
  in.host_estop = false;
  in.rc_ever_seen = true;
  TEST_ASSERT_EQUAL(State::Estop, m.update(in, 50));
  TEST_ASSERT_EQUAL(FaultReason::RcLinkLost, m.faultReason());
}

void test_low_battery_forces_estop() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.battery_mv = 9500;  // below 10000 cutoff
  // A sustained low reading trips the Estop once the debounce window elapses.
  m.update(in, 50);
  TEST_ASSERT_EQUAL(State::Estop, m.update(in, 50 + 250));
  TEST_ASSERT_EQUAL(FaultReason::BatteryLow, m.faultReason());
}

void test_transient_battery_dip_does_not_estop() {
  // DXL power-on inrush from 18 servos sags the supply below the cutoff for a
  // few control cycles (HIL bug 7). A dip shorter than the debounce window
  // must not tear down the session with a BatteryLow Estop.
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.battery_mv = 9000;  // inrush sag
  TEST_ASSERT_NOT_EQUAL(State::Estop, m.update(in, 50));
  TEST_ASSERT_NOT_EQUAL(State::Estop, m.update(in, 70));  // 20 ms into dip
  in.battery_mv = 12000;  // recovered before the debounce window
  TEST_ASSERT_NOT_EQUAL(State::Estop, m.update(in, 90));
  TEST_ASSERT_EQUAL(FaultReason::None, m.faultReason());
  // Recovery resets the debounce: a later dip needs a full window again.
  in.battery_mv = 9000;
  TEST_ASSERT_NOT_EQUAL(State::Estop, m.update(in, 500));
  TEST_ASSERT_NOT_EQUAL(State::Estop, m.update(in, 500 + 249));
  TEST_ASSERT_EQUAL(State::Estop, m.update(in, 500 + 250));
  TEST_ASSERT_EQUAL(FaultReason::BatteryLow, m.faultReason());
}

void test_invalid_battery_reading_does_not_estop() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.command_source = static_cast<uint8_t>(CommandSource::Rc);
  in.battery_mv = 0;
  in.battery_valid = false;  // e.g. USB-only bench, no pack sense
  TEST_ASSERT_EQUAL(State::RcManual, m.update(in, 50));
}

void test_watchdog_fault_forces_estop() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.watchdog_fault = true;
  TEST_ASSERT_EQUAL(State::Estop, m.update(in, 50));
  TEST_ASSERT_EQUAL(FaultReason::Watchdog, m.faultReason());
}

void test_rc_failsafe_stops_when_operational() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.rc_ever_seen = true;  // link existed (required to be operational at all)
  in.command_source = static_cast<uint8_t>(CommandSource::Rc);
  m.update(in, 50);  // RcManual
  in.rc_failsafe = true;
  TEST_ASSERT_EQUAL(State::Estop, m.update(in, 60));
  TEST_ASSERT_EQUAL(FaultReason::RcLinkLost, m.faultReason());
}

void test_dxl_hard_fault_latches_until_cleared() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.dxl_hard_fault = true;
  TEST_ASSERT_EQUAL(State::FaultHard, m.update(in, 50));
  // Condition gone but no clear request -> still latched.
  in.dxl_hard_fault = false;
  TEST_ASSERT_EQUAL(State::FaultHard, m.update(in, 60));
  // Clear request releases to Disarmed.
  m.requestClearFault();
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 70));
}

void test_clear_request_ignored_while_fault_active() {
  StateMachine m = atStandReady();
  StateInputs in = healthy();
  in.rc_armed = true;
  in.dxl_hard_fault = true;
  m.update(in, 50);  // FaultHard
  m.requestClearFault();
  // Still faulting -> stays latched and the request is dropped.
  TEST_ASSERT_EQUAL(State::FaultHard, m.update(in, 60));
}

void test_passive_pose_requires_torque_off() {
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  m.update(in, 0);
  m.update(in, 10);  // Disarmed
  in.passive_request = true;
  in.torque_off = false;
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 20));  // torque still on
  in.torque_off = true;
  TEST_ASSERT_EQUAL(State::PassivePoseStream, m.update(in, 30));
  TEST_ASSERT_FALSE(stateAllowsTorque(State::PassivePoseStream));
  in.passive_request = false;
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 40));
}

void test_maintenance_requires_lock() {
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  m.update(in, 0);
  m.update(in, 10);  // Disarmed
  in.maintenance_request = true;
  in.mac_lock_held = false;
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 20));  // no lock
  in.mac_lock_held = true;
  TEST_ASSERT_EQUAL(State::MacMaintenance, m.update(in, 30));
  TEST_ASSERT_TRUE(stateAllowsMotion(State::MacMaintenance));
  in.maintenance_request = false;
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 40));
}

void test_maintenance_to_passive_handoff() {
  // Bench passive workflow (hexapod_src-nkb): power + scan happen under the
  // maintenance lock; a passive request with torque confirmed off must hand
  // off directly (never dropping through Disarmed, which would let dxlTask
  // cut DXL power and kill the present-position stream). Torque still on
  // must hold the handoff.
  StateMachine m = makeMachine();
  StateInputs in = healthy();
  m.update(in, 0);
  m.update(in, 10);  // Disarmed
  in.maintenance_request = true;
  in.mac_lock_held = true;
  TEST_ASSERT_EQUAL(State::MacMaintenance, m.update(in, 20));
  in.passive_request = true;
  in.torque_off = false;
  TEST_ASSERT_EQUAL(State::MacMaintenance, m.update(in, 30));  // torque on
  in.torque_off = true;
  TEST_ASSERT_EQUAL(State::PassivePoseStream, m.update(in, 40));
  // Exit passive with the lock still held: falls to Disarmed, then the held
  // lock re-enters maintenance on the next cycle.
  in.passive_request = false;
  TEST_ASSERT_EQUAL(State::Disarmed, m.update(in, 50));
  TEST_ASSERT_EQUAL(State::MacMaintenance, m.update(in, 60));
}

void test_torque_gate_excludes_disarmed_and_estop() {
  TEST_ASSERT_FALSE(stateAllowsTorque(State::Disarmed));
  TEST_ASSERT_FALSE(stateAllowsTorque(State::Estop));
  TEST_ASSERT_FALSE(stateAllowsMotion(State::StandReady));  // hold only
  TEST_ASSERT_TRUE(stateAllowsTorque(State::StandReady));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_boot_progresses_to_disarmed);
  RUN_TEST(test_config_not_loaded_holds_in_configload);
  RUN_TEST(test_disarmed_requires_arm_for_arming_checks);
  RUN_TEST(test_arming_checks_gate_stand_ready);
  RUN_TEST(test_arming_checks_timeout_reports_soft_fault);
  RUN_TEST(test_dxl_power_allowed_during_arming_and_operation);
  RUN_TEST(test_stand_ready_to_rc_manual);
  RUN_TEST(test_rc_manual_to_contact_terrain_and_back);
  RUN_TEST(test_stand_ready_to_jetson_assisted);
  RUN_TEST(test_disarm_returns_from_motion);
  RUN_TEST(test_kill_forces_estop_and_recovers);
  RUN_TEST(test_kill_without_rc_ever_seen_stays_disarmed);
  RUN_TEST(test_host_estop_forces_estop);
  RUN_TEST(test_estop_recovers_on_bench_with_failsafe_held);
  RUN_TEST(test_low_battery_forces_estop);
  RUN_TEST(test_transient_battery_dip_does_not_estop);
  RUN_TEST(test_invalid_battery_reading_does_not_estop);
  RUN_TEST(test_watchdog_fault_forces_estop);
  RUN_TEST(test_rc_failsafe_stops_when_operational);
  RUN_TEST(test_dxl_hard_fault_latches_until_cleared);
  RUN_TEST(test_clear_request_ignored_while_fault_active);
  RUN_TEST(test_passive_pose_requires_torque_off);
  RUN_TEST(test_maintenance_requires_lock);
  RUN_TEST(test_maintenance_to_passive_handoff);
  RUN_TEST(test_torque_gate_excludes_disarmed_and_estop);
  return UNITY_END();
}
