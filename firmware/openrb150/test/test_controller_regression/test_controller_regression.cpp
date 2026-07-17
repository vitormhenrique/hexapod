// End-to-end semantic regression tests for portable ControllerCore.
// Run with: pio test -e native -f test_controller_regression

#include <unity.h>

#include "../support/controller_sim_adapter.h"

namespace {

struct SemanticFrame {
  uint8_t safety_state = 0;
  uint8_t command_source = 0;
  uint32_t config_revision = 0;
  uint8_t goal_count = 0;
  bool motion_gate = false;
  bool goal_valid = false;
  bool allow_torque = false;
  bool gate_rising = false;
  bool gate_falling = false;
  bool any_goal_clamped = false;
  uint8_t ids[config::kNumServos] = {};
  uint16_t ticks[config::kNumServos] = {};
  bool clamped[config::kNumServos] = {};
};

void configureSafeRcIntent(controller::ControllerIntent& intent) {
  intent.rc.kill = false;
  intent.rc.failsafe = false;
  intent.rc.ever_seen = true;
  intent.rc.command.valid = true;
  intent.rc.command.body_height = 0.5f;
  intent.rc.command.speed = 0.5f;
  intent.rc.command.stride = 0.5f;
  intent.rc.command.step_height = 0.5f;
  intent.motion.duty_x255 = 128;
}

SemanticFrame capture(const controller::RobotCommand& command) {
  SemanticFrame frame;
  frame.safety_state = static_cast<uint8_t>(command.safety_state);
  frame.command_source = static_cast<uint8_t>(command.command_source);
  frame.config_revision = command.diagnostics.config_revision;
  frame.goal_count = command.goals.count;
  frame.motion_gate = command.motion_gate;
  frame.goal_valid = command.goal_valid;
  frame.allow_torque = command.allow_torque;
  frame.gate_rising = command.diagnostics.motion_gate_rising;
  frame.gate_falling = command.diagnostics.motion_gate_falling;
  frame.any_goal_clamped = command.diagnostics.any_goal_clamped;
  for (uint8_t index = 0; index < command.goals.count; ++index) {
    frame.ids[index] = command.goals.joints[index].id;
    frame.ticks[index] = command.goals.joints[index].tick;
    frame.clamped[index] = command.goals.joints[index].clamped;
  }
  return frame;
}

void assertSame(const SemanticFrame& expected, const SemanticFrame& actual) {
  TEST_ASSERT_EQUAL_UINT8(expected.safety_state, actual.safety_state);
  TEST_ASSERT_EQUAL_UINT8(expected.command_source, actual.command_source);
  TEST_ASSERT_EQUAL_UINT32(expected.config_revision, actual.config_revision);
  TEST_ASSERT_EQUAL_UINT8(expected.goal_count, actual.goal_count);
  TEST_ASSERT_EQUAL(expected.motion_gate, actual.motion_gate);
  TEST_ASSERT_EQUAL(expected.goal_valid, actual.goal_valid);
  TEST_ASSERT_EQUAL(expected.allow_torque, actual.allow_torque);
  TEST_ASSERT_EQUAL(expected.gate_rising, actual.gate_rising);
  TEST_ASSERT_EQUAL(expected.gate_falling, actual.gate_falling);
  TEST_ASSERT_EQUAL(expected.any_goal_clamped, actual.any_goal_clamped);
  for (uint8_t index = 0; index < expected.goal_count; ++index) {
    TEST_ASSERT_EQUAL_UINT8(expected.ids[index], actual.ids[index]);
    TEST_ASSERT_EQUAL_UINT16(expected.ticks[index], actual.ticks[index]);
    TEST_ASSERT_EQUAL(expected.clamped[index], actual.clamped[index]);
  }
}

bool hasDifferentGoalTick(const SemanticFrame& first,
                          const SemanticFrame& second) {
  if (first.goal_count != second.goal_count) return true;
  for (uint8_t index = 0; index < first.goal_count; ++index) {
    if (first.ids[index] != second.ids[index] ||
        first.ticks[index] != second.ticks[index]) {
      return true;
    }
  }
  return false;
}

void configureReadyAdapter(controller::sim::ControllerSimAdapter& adapter,
                           uint32_t revision = 1) {
  TEST_ASSERT_TRUE(adapter.configureDefault(revision));
  TEST_ASSERT_TRUE(adapter.setReadyDxl());
  configureSafeRcIntent(adapter.intent());
}

void advanceBoth(controller::sim::ControllerSimAdapter& first,
                 controller::sim::ControllerSimAdapter& second,
                 SemanticFrame* frames, uint8_t& count) {
  first.advance(10);
  second.advance(10);
  const SemanticFrame first_frame = capture(first.command());
  const SemanticFrame second_frame = capture(second.command());
  assertSame(first_frame, second_frame);
  frames[count++] = first_frame;
}

}  // namespace

void test_identical_sequence_has_stable_semantic_outputs() {
  controller::sim::ControllerSimAdapter first;
  controller::sim::ControllerSimAdapter second;
  configureReadyAdapter(first, 11);
  configureReadyAdapter(second, 11);
  SemanticFrame frames[12];
  uint8_t count = 0;

  advanceBoth(first, second, frames, count);  // Boot -> ConfigLoad
  advanceBoth(first, second, frames, count);  // ConfigLoad -> Disarmed
  advanceBoth(first, second, frames, count);  // arm release qualification
  first.intent().rc.armed = true;
  second.intent().rc.armed = true;
  advanceBoth(first, second, frames, count);  // ArmingChecks
  advanceBoth(first, second, frames, count);  // StandReady
  advanceBoth(first, second, frames, count);  // RcManual, stand
  const SemanticFrame stand = frames[count - 1];

  first.intent().rc.command.gait_index = 1;
  first.intent().rc.command.twist_vx = 0.65f;
  second.intent().rc.command.gait_index = 1;
  second.intent().rc.command.twist_vx = 0.65f;
  advanceBoth(first, second, frames, count);  // walking gait
  const SemanticFrame walking = frames[count - 1];

  first.intent().rc.armed = false;
  second.intent().rc.armed = false;
  advanceBoth(first, second, frames, count);  // authority loss -> Disarmed
  const SemanticFrame authority_loss = frames[count - 1];

  first.intent().host_estop = true;
  second.intent().host_estop = true;
  advanceBoth(first, second, frames, count);  // Estop from Disarmed
  const SemanticFrame estop = frames[count - 1];

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::RcManual),
                          stand.safety_state);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::CommandSource::Rc),
                          stand.command_source);
  TEST_ASSERT_TRUE(stand.motion_gate);
  TEST_ASSERT_TRUE(stand.goal_valid);
  TEST_ASSERT_TRUE(hasDifferentGoalTick(stand, walking));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::CommandSource::Rc),
                          walking.command_source);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::Disarmed),
                          authority_loss.safety_state);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::CommandSource::None),
                          authority_loss.command_source);
  TEST_ASSERT_FALSE(authority_loss.motion_gate);
  TEST_ASSERT_TRUE(authority_loss.gate_falling);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::Estop),
                          estop.safety_state);
  TEST_ASSERT_FALSE(estop.goal_valid);
  TEST_ASSERT_FALSE(estop.allow_torque);
}

void test_stale_contact_and_invalid_feedback_fail_closed() {
  controller::sim::ControllerSimAdapter stale_contact;
  configureReadyAdapter(stale_contact);
  stale_contact.intent().features.foot_contact_enabled = true;
  stale_contact.state().contact.validity = controller::SnapshotValidity::Stale;
  for (uint8_t leg = 0; leg < sensors::kNumFeet; ++leg) {
    stale_contact.state().contact.feet[leg].confidence = 255;
    stale_contact.state().contact.feet[leg].stale = true;
  }
  stale_contact.advance(10);
  stale_contact.advance(10);
  stale_contact.advance(10);
  stale_contact.intent().rc.armed = true;
  stale_contact.advance(10);
  stale_contact.advance(10);
  stale_contact.advance(10);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::RcManual),
                          static_cast<uint8_t>(
                              stale_contact.command().safety_state));

  controller::sim::ControllerSimAdapter invalid_feedback;
  configureReadyAdapter(invalid_feedback);
  invalid_feedback.state().dxl.servo_count = config::kNumServos - 1;
  invalid_feedback.state().dxl.configured_servo_coverage = false;
  invalid_feedback.state().dxl.pose_known_mask = 0;
  invalid_feedback.advance(10);
  invalid_feedback.advance(10);
  invalid_feedback.advance(10);
  invalid_feedback.intent().rc.armed = true;
  invalid_feedback.advance(10);
  invalid_feedback.advance(10);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::ArmingChecks),
                          static_cast<uint8_t>(
                              invalid_feedback.command().safety_state));
  TEST_ASSERT_FALSE(invalid_feedback.command().motion_gate);
  TEST_ASSERT_FALSE(invalid_feedback.command().allow_torque);
}

void test_config_revision_and_tick_saturation_are_reported() {
  controller::sim::ControllerSimAdapter adapter;
  TEST_ASSERT_TRUE(adapter.configureDefault(21));
  adapter.config().robot.servos[0].min_tick = 0;
  adapter.config().robot.servos[0].max_tick = 1;
  adapter.config().valid = config::validateRobotConfig(adapter.config().robot);
  TEST_ASSERT_TRUE(adapter.config().valid);
  TEST_ASSERT_TRUE(adapter.setReadyDxl());
  configureSafeRcIntent(adapter.intent());

  adapter.advance(10);
  adapter.advance(10);
  adapter.advance(10);
  adapter.intent().rc.armed = true;
  adapter.advance(10);
  adapter.advance(10);
  adapter.advance(10);

  TEST_ASSERT_TRUE(adapter.command().diagnostics.any_goal_clamped);
  TEST_ASSERT_TRUE(adapter.command().goals.joints[0].clamped);

  adapter.config().robot.gait.stride_len_mm = 50;
  adapter.config().revision = 22;
  adapter.config().valid = config::validateRobotConfig(adapter.config().robot);
  adapter.state().dxl.config_revision = adapter.config().revision;
  TEST_ASSERT_TRUE(adapter.config().valid);
  adapter.advance(10);

  TEST_ASSERT_TRUE(adapter.command().diagnostics.config_reapplied);
  TEST_ASSERT_EQUAL_UINT32(22, adapter.command().diagnostics.config_revision);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_identical_sequence_has_stable_semantic_outputs);
  RUN_TEST(test_stale_contact_and_invalid_feedback_fail_closed);
  RUN_TEST(test_config_revision_and_tick_saturation_are_reported);
  return UNITY_END();
}