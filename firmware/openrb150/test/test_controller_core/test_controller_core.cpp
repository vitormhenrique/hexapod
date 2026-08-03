// Native focused behavior tests for the portable ControllerCore.
// Run with: pio test -e native -f test_controller_core

#include <unity.h>

#include "../../src/controller/command_frame.h"
#include "../../src/controller/controller_core.h"

namespace {

controller::ControllerConfigSnapshot defaultConfigSnapshot() {
  controller::ControllerConfigSnapshot snapshot;
  config::defaultRobotConfig(snapshot.robot);
  snapshot.revision = 1;
  snapshot.valid = true;
  snapshot.persistent = false;
  return snapshot;
}

controller::RobotState readyState(
    const controller::ControllerConfigSnapshot& config_snapshot) {
  controller::RobotState state;
  state.config_ready = true;
  state.battery.millivolts = 12000;
  state.battery.valid = true;
  state.battery.validity = controller::SnapshotValidity::Fresh;
  state.dxl.servo_count = config::kNumServos;
  state.dxl.validity = controller::SnapshotValidity::Fresh;
  state.dxl.configured_servo_coverage = true;
  state.dxl.config_revision = config_snapshot.revision;
  state.dxl.pose_known_mask = (1u << config::kNumServos) - 1u;
  state.dxl.torque_off = false;
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    state.dxl.servos[index].id = config_snapshot.robot.servos[index].id;
    state.dxl.servos[index].present_position = config::kServoCenterTick;
    state.dxl.servos[index].ok = true;
  }
  return state;
}

controller::ControllerIntent safeRcIntent() {
  controller::ControllerIntent intent;
  intent.rc.kill = false;
  intent.rc.failsafe = false;
  intent.rc.ever_seen = true;
  intent.rc.command.valid = true;
  intent.rc.command.body_height = 0.5f;
  intent.rc.command.speed = 0.5f;
  intent.rc.command.stride = 0.5f;
  intent.rc.command.step_height = 0.5f;
  intent.motion.duty_x255 = 128;
  return intent;
}

controller::ControllerTime timeAt(uint32_t now_ms) {
  controller::ControllerTime time;
  time.now_ms = now_ms;
  time.dt_ms = 10;
  time.valid = true;
  return time;
}

void driveToRcManual(controller::ControllerCore& core,
                     const controller::RobotState& state,
                     controller::ControllerIntent& intent,
                     const controller::ControllerConfigSnapshot& config_snapshot,
                     controller::RobotCommand& command) {
  core.step(state, intent, config_snapshot, timeAt(0), command);
  core.step(state, intent, config_snapshot, timeAt(10), command);
  core.step(state, intent, config_snapshot, timeAt(20), command);
  intent.rc.armed = true;
  core.step(state, intent, config_snapshot, timeAt(30), command);
  core.step(state, intent, config_snapshot, timeAt(40), command);
  core.step(state, intent, config_snapshot, timeAt(50), command);
}

}  // namespace

void test_rc_stand_produces_clamped_servo_goals() {
  const controller::ControllerConfigSnapshot config_snapshot =
      defaultConfigSnapshot();
  const controller::RobotState state = readyState(config_snapshot);
  controller::ControllerIntent intent = safeRcIntent();
  controller::ControllerCore core;
  controller::RobotCommand command;

  driveToRcManual(core, state, intent, config_snapshot, command);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::RcManual),
                          static_cast<uint8_t>(command.safety_state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::CommandSource::Rc),
                          static_cast<uint8_t>(command.command_source));
  TEST_ASSERT_TRUE(command.motion_gate);
  TEST_ASSERT_TRUE(command.goal_valid);
  TEST_ASSERT_EQUAL_UINT8(config::kNumServos, command.goals.count);
  for (uint8_t index = 0; index < command.goals.count; ++index) {
    TEST_ASSERT_TRUE(command.goals.joints[index].tick <= config::kServoMaxTick);
  }
}

void test_rc_gait_input_advances_the_same_portable_goal_path() {
  const controller::ControllerConfigSnapshot config_snapshot =
      defaultConfigSnapshot();
  const controller::RobotState state = readyState(config_snapshot);
  controller::ControllerIntent intent = safeRcIntent();
  controller::ControllerCore core;
  controller::RobotCommand command;

  driveToRcManual(core, state, intent, config_snapshot, command);
  intent.rc.command.gait_index = 1;
  intent.rc.command.twist_vx = 0.6f;
  core.step(state, intent, config_snapshot, timeAt(60), command);

  TEST_ASSERT_TRUE(command.goal_valid);
  TEST_ASSERT_EQUAL_UINT8(config::kNumServos, command.goals.count);
  TEST_ASSERT_FALSE(command.diagnostics.any_goal_unreachable);
}

void test_rc_twist_step_reaches_pipeline_through_accel_limiter() {
  const controller::ControllerConfigSnapshot config_snapshot =
      defaultConfigSnapshot();
  const controller::RobotState state = readyState(config_snapshot);
  controller::ControllerIntent neutral_intent = safeRcIntent();
  controller::ControllerIntent stepped_intent = safeRcIntent();
  controller::ControllerCore neutral_core;
  controller::ControllerCore stepped_core;
  controller::RobotCommand neutral;
  controller::RobotCommand stepped;

  driveToRcManual(neutral_core, state, neutral_intent, config_snapshot,
                  neutral);
  driveToRcManual(stepped_core, state, stepped_intent, config_snapshot,
                  stepped);
  neutral_intent.rc.command.gait_index = 2;
  stepped_intent.rc.command.gait_index = 2;
  stepped_intent.rc.command.twist_vx = 1.0f;
  // Mark III starts on a zero-longitudinal keyframe. Advance 100 ms so the
  // shaped command and interpolated gait both produce a measurable target.
  for (uint32_t now_ms = 60; now_ms <= 150; now_ms += 10) {
    neutral_core.step(state, neutral_intent, config_snapshot, timeAt(now_ms),
                      neutral);
    stepped_core.step(state, stepped_intent, config_snapshot, timeAt(now_ms),
                      stepped);
  }

  TEST_ASSERT_TRUE(stepped.goal_valid);
  TEST_ASSERT_EQUAL_UINT8(neutral.goals.count, stepped.goals.count);
  bool changed = false;
  for (uint8_t index = 0; index < stepped.goals.count; ++index) {
    const int32_t delta = static_cast<int32_t>(stepped.goals.joints[index].tick) -
                          static_cast<int32_t>(neutral.goals.joints[index].tick);
    if (delta != 0) changed = true;
    // The command is still acceleration-limited, so it must remain far below
    // an instantaneous full-stride servo jump.
    TEST_ASSERT_TRUE(delta >= -200 && delta <= 200);
  }
  TEST_ASSERT_TRUE(changed);
}

void test_estop_clears_motion_and_requests_adapter_cleanup() {
  const controller::ControllerConfigSnapshot config_snapshot =
      defaultConfigSnapshot();
  const controller::RobotState state = readyState(config_snapshot);
  controller::ControllerIntent intent = safeRcIntent();
  controller::ControllerCore core;
  controller::RobotCommand command;

  driveToRcManual(core, state, intent, config_snapshot, command);
  intent.host_estop = true;
  core.step(state, intent, config_snapshot, timeAt(60), command);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::Estop),
                          static_cast<uint8_t>(command.safety_state));
  TEST_ASSERT_FALSE(command.motion_gate);
  TEST_ASSERT_FALSE(command.goal_valid);
  TEST_ASSERT_FALSE(command.allow_torque);
  TEST_ASSERT_TRUE(command.diagnostics.clear_maintenance_lock);
  TEST_ASSERT_TRUE(command.diagnostics.clear_passive_request);
}

void test_maintenance_target_requires_entry_edge_then_emits_mapped_goal() {
  const controller::ControllerConfigSnapshot config_snapshot =
      defaultConfigSnapshot();
  const controller::RobotState state = readyState(config_snapshot);
  controller::ControllerIntent intent = safeRcIntent();
  intent.rc.ever_seen = false;
  intent.maintenance.lock_held = true;
  intent.maintenance.lock_token = 1;
  controller::ControllerCore core;
  controller::RobotCommand command;

  core.step(state, intent, config_snapshot, timeAt(0), command);
  core.step(state, intent, config_snapshot, timeAt(10), command);
  core.step(state, intent, config_snapshot, timeAt(20), command);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::MacMaintenance),
                          static_cast<uint8_t>(command.safety_state));
  TEST_ASSERT_TRUE(command.diagnostics.maintenance_session_started);
  TEST_ASSERT_TRUE(command.diagnostics.clear_maintenance_targets);
  TEST_ASSERT_FALSE(command.goal_valid);

  intent.maintenance.targets.set[0][0] = true;
  intent.maintenance.targets.tick[0][0] = config::kServoCenterTick + 10;
  core.step(state, intent, config_snapshot, timeAt(30), command);

  TEST_ASSERT_TRUE(command.goal_valid);
  TEST_ASSERT_EQUAL_UINT8(1, command.goals.count);
  TEST_ASSERT_EQUAL_UINT8(config_snapshot.robot.servos[0].id,
                          command.goals.joints[0].id);
  TEST_ASSERT_EQUAL_UINT16(config::kServoCenterTick + 10,
                           command.goals.joints[0].tick);
}

void test_identical_step_sequences_produce_identical_goals() {
  const controller::ControllerConfigSnapshot config_snapshot =
      defaultConfigSnapshot();
  const controller::RobotState state = readyState(config_snapshot);
  controller::ControllerIntent intent = safeRcIntent();
  controller::ControllerCore first;
  controller::ControllerCore second;
  controller::RobotCommand first_command;
  controller::RobotCommand second_command;

  for (uint32_t now_ms = 0; now_ms <= 50; now_ms += 10) {
    if (now_ms >= 30) intent.rc.armed = true;
    first.step(state, intent, config_snapshot, timeAt(now_ms), first_command);
    second.step(state, intent, config_snapshot, timeAt(now_ms), second_command);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(first_command.safety_state),
                            static_cast<uint8_t>(second_command.safety_state));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(first_command.command_source),
                            static_cast<uint8_t>(second_command.command_source));
    TEST_ASSERT_EQUAL_UINT8(first_command.goals.count,
                            second_command.goals.count);
    TEST_ASSERT_EQUAL(first_command.goal_valid, second_command.goal_valid);
    for (uint8_t index = 0; index < first_command.goals.count; ++index) {
      TEST_ASSERT_EQUAL_UINT8(first_command.goals.joints[index].id,
                              second_command.goals.joints[index].id);
      TEST_ASSERT_EQUAL_UINT16(first_command.goals.joints[index].tick,
                               second_command.goals.joints[index].tick);
      TEST_ASSERT_EQUAL(first_command.goals.joints[index].clamped,
                        second_command.goals.joints[index].clamped);
    }
  }
}

void test_reset_drops_live_motion_state_before_restart() {
  const controller::ControllerConfigSnapshot config_snapshot =
      defaultConfigSnapshot();
  const controller::RobotState state = readyState(config_snapshot);
  controller::ControllerIntent intent = safeRcIntent();
  controller::ControllerCore core;
  controller::RobotCommand command;

  driveToRcManual(core, state, intent, config_snapshot, command);
  TEST_ASSERT_TRUE(command.motion_gate);

  core.reset();
  core.step(state, intent, config_snapshot, timeAt(60), command);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::ConfigLoad),
                          static_cast<uint8_t>(command.safety_state));
  TEST_ASSERT_FALSE(command.motion_gate);
  TEST_ASSERT_FALSE(command.goal_valid);
}

void test_command_frame_maps_forward_left_onto_body_axes() {
  // URDF body frame B: +Y is the mechanical front, +X is the robot's right.
  float body_x = 99.0f;
  float body_y = 99.0f;
  controller::commandPlanarToBody(1.0f, 0.0f, body_x, body_y);  // forward
  TEST_ASSERT_EQUAL_FLOAT(0.0f, body_x);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, body_y);
  controller::commandPlanarToBody(0.0f, 1.0f, body_x, body_y);  // left
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, body_x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, body_y);

  // Command roll leans about the forward (+Y) axis: a body pitch. Command
  // pitch leans about the left (-X) axis: a negated body roll.
  float body_roll = 99.0f;
  float body_pitch = 99.0f;
  controller::commandAttitudeToBody(0.2f, 0.0f, body_roll, body_pitch);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, body_roll);
  TEST_ASSERT_EQUAL_FLOAT(0.2f, body_pitch);
  controller::commandAttitudeToBody(0.0f, 0.3f, body_roll, body_pitch);
  TEST_ASSERT_EQUAL_FLOAT(-0.3f, body_roll);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, body_pitch);
}

void test_rc_gait_switch_maps_only_moving_gaits() {
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(config::GaitId::Wave),
                          static_cast<uint8_t>(controller::rcGaitFromIndex(0)));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(config::GaitId::Ripple),
      static_cast<uint8_t>(controller::rcGaitFromIndex(1)));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(config::GaitId::Tripod),
      static_cast<uint8_t>(controller::rcGaitFromIndex(2)));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(config::GaitId::Tripod),
      static_cast<uint8_t>(controller::rcGaitFromIndex(255)));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_rc_stand_produces_clamped_servo_goals);
  RUN_TEST(test_rc_gait_input_advances_the_same_portable_goal_path);
  RUN_TEST(test_rc_twist_step_reaches_pipeline_through_accel_limiter);
  RUN_TEST(test_estop_clears_motion_and_requests_adapter_cleanup);
  RUN_TEST(test_maintenance_target_requires_entry_edge_then_emits_mapped_goal);
  RUN_TEST(test_identical_step_sequences_produce_identical_goals);
  RUN_TEST(test_reset_drops_live_motion_state_before_restart);
  RUN_TEST(test_command_frame_maps_forward_left_onto_body_axes);
  RUN_TEST(test_rc_gait_switch_maps_only_moving_gaits);
  return UNITY_END();
}