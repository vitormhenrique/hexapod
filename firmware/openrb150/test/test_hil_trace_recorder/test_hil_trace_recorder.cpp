#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/hil/trace_recorder.h"
#include "../../src/protocol/crc16.h"
#include "../../src/protocol/framing.h"

namespace {

hil::OutputGuardStatus outputGuard(uint32_t goal_writes = 0) {
  hil::OutputGuardStatus guard;
  guard.output_disabled = true;
  guard.power_guard_active = true;
  guard.torque_guard_active = true;
  guard.goal_guard_active = true;
  guard.write_guard_active = true;
  guard.blocked_goal_write = goal_writes;
  return guard;
}

controller::ControllerStepInput stepInput(uint32_t now_ms, uint32_t revision) {
  controller::ControllerStepInput input;
  config::defaultRobotConfig(input.config.robot);
  input.config.revision = revision;
  input.config.valid = true;
  input.config.persistent = false;
  input.time.now_ms = now_ms;
  input.time.dt_ms = 10;
  input.time.valid = true;
  input.state.config_ready = true;
  input.state.battery.valid = true;
  input.state.battery.millivolts = 12000;
  input.state.dxl.servo_count = config::kNumServos;
  return input;
}

controller::RobotCommand command() {
  controller::RobotCommand result;
  result.safety_state = safety::State::RcManual;
  result.goal_valid = true;
  result.goals.count = 1;
  result.goals.joints[0].id = 1;
  result.goals.joints[0].tick = 2048;
  result.goals.joints[0].leg = 0;
  result.goals.joints[0].joint = 0;
  return result;
}

void acknowledge(hil::trace::TraceRecorder& recorder,
                 const hil::trace::RecordView& view) {
  uint8_t bytes[128] = {};
  uint16_t crc = protocol::kCrc16Init;
  for (uint16_t offset = 0; offset < view.logical_length;) {
    const uint16_t remaining =
        static_cast<uint16_t>(view.logical_length - offset);
    const uint16_t length =
        remaining < sizeof(bytes) ? remaining : sizeof(bytes);
    TEST_ASSERT_TRUE(
        recorder.copySlice(view, offset, bytes, sizeof(bytes), length));
    crc = protocol::crc16Update(crc, bytes, length);
    offset = static_cast<uint16_t>(offset + length);
  }
  TEST_ASSERT_EQUAL_UINT16(view.logical_crc16, crc);
  recorder.acknowledge(view);
}

  void assertReassembles(hil::trace::TraceRecorder& recorder,
               const hil::trace::RecordView& view) {
    uint8_t rebuilt[hil::trace::kMaxLogicalRecordBytes] = {};
    uint8_t payload[protocol::kMaxPayload] = {};
    hil::trace::ReassemblyState state;
    const uint16_t fragment_capacity =
      hil::trace::maxFragmentData(sizeof(payload));
    const uint8_t fragment_count = static_cast<uint8_t>(
      (view.logical_length + fragment_capacity - 1u) / fragment_capacity);
    TEST_ASSERT_GREATER_THAN_UINT8(1, fragment_count);
    for (uint8_t index = 0; index < fragment_count; ++index) {
    const uint16_t offset = static_cast<uint16_t>(
      static_cast<uint32_t>(index) * fragment_capacity);
    const uint16_t remaining =
      static_cast<uint16_t>(view.logical_length - offset);
    const uint16_t length =
      remaining < fragment_capacity ? remaining : fragment_capacity;
    TEST_ASSERT_TRUE(recorder.copySlice(
      view, offset, &payload[hil::trace::kFragmentPrefixBytes],
      fragment_capacity, length));
    hil::trace::FragmentHeader header;
    header.session_id = view.session_id;
    header.capture_id = view.capture_id;
    header.record_seq = view.record_seq;
    header.record_type = view.type;
    header.fragment_index = index;
    header.fragment_count = fragment_count;
    header.logical_length = view.logical_length;
    header.logical_crc16 = view.logical_crc16;
    uint16_t payload_length = 0;
    TEST_ASSERT_TRUE(hil::trace::encodeFragmentSlice(
      header, &payload[hil::trace::kFragmentPrefixBytes], length, payload,
      sizeof(payload), &payload_length));
    const hil::trace::ReassemblyResult result = hil::trace::acceptFragment(
      payload, payload_length, rebuilt, sizeof(rebuilt), &state);
    if (index + 1u == fragment_count) {
      TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(hil::trace::ReassemblyResult::Complete),
        static_cast<uint8_t>(result));
    } else {
      TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(hil::trace::ReassemblyResult::Accepted),
        static_cast<uint8_t>(result));
    }
    }
    TEST_ASSERT_EQUAL_UINT16(view.logical_crc16,
                 protocol::crc16(rebuilt, view.logical_length));
  }

}  // namespace

void test_recorder_emits_begin_config_step_output_and_end_in_order() {
  hil::trace::TraceRecorder recorder;
  hil::CaptureRequest request;
  request.session_id = 4;
  request.capture_id = 9;
  request.step_count = 2;
  controller::ControllerStepInput first_input = stepInput(10, 1);
  controller::ControllerStepInput second_input = stepInput(20, 1);
  TEST_ASSERT_TRUE(recorder.begin(request, outputGuard(), &first_input.config));
  TEST_ASSERT_TRUE(recorder.markNext(77));

  // The recorder sends the self-contained begin/config records before its
  // bounded slot starts retaining actual controller-step observations.
  const hil::trace::RecordType startup[] = {
      hil::trace::RecordType::Begin, hil::trace::RecordType::Config};
  for (const hil::trace::RecordType type : startup) {
    hil::trace::RecordView view;
    TEST_ASSERT_TRUE(recorder.peek(&view));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(type),
                            static_cast<uint8_t>(view.type));
    TEST_ASSERT_GREATER_THAN_UINT16(0, view.logical_length);
    acknowledge(recorder, view);
    if (type == hil::trace::RecordType::Begin) {
      TEST_ASSERT_TRUE(
          recorder.captureStep(first_input, command(), outputGuard()));
    }
  }

  TEST_ASSERT_TRUE(
      recorder.captureStep(first_input, command(), outputGuard()));
  hil::trace::RecordView first_step;
  TEST_ASSERT_TRUE(recorder.peek(&first_step));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::trace::RecordType::Step),
                          static_cast<uint8_t>(first_step.type));
  acknowledge(recorder, first_step);

  hil::trace::RecordView marker;
  TEST_ASSERT_TRUE(recorder.peek(&marker));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::trace::RecordType::Marker),
      static_cast<uint8_t>(marker.type));
  acknowledge(recorder, marker);

  TEST_ASSERT_TRUE(
      recorder.captureStep(second_input, command(), outputGuard(1)));
  TEST_ASSERT_FALSE(recorder.active());
  const hil::trace::RecordType expected_finish[] = {
      hil::trace::RecordType::Step,
      hil::trace::RecordType::OutputBlocked,
      hil::trace::RecordType::End,
  };
  for (const hil::trace::RecordType type : expected_finish) {
    hil::trace::RecordView view;
    TEST_ASSERT_TRUE(recorder.peek(&view));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(type),
                            static_cast<uint8_t>(view.type));
    acknowledge(recorder, view);
  }

  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  hil::CaptureEndReason reason = hil::CaptureEndReason::TransportOverflow;
  TEST_ASSERT_TRUE(recorder.takeCompletion(&session_id, &capture_id, &reason));
  TEST_ASSERT_EQUAL_UINT32(request.session_id, session_id);
  TEST_ASSERT_EQUAL_UINT32(request.capture_id, capture_id);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::CaptureEndReason::Complete),
                          static_cast<uint8_t>(reason));
}

void test_recorder_aborts_when_api_handoff_is_not_drained() {
  hil::trace::TraceRecorder recorder;
  hil::CaptureRequest request;
  request.session_id = 4;
  request.capture_id = 9;
  request.step_count = 2;
    controller::ControllerStepInput first_input = stepInput(10, 1);
    controller::ControllerStepInput second_input = stepInput(20, 1);
    TEST_ASSERT_TRUE(recorder.begin(request, outputGuard(), &first_input.config));

    hil::trace::RecordView view;
    TEST_ASSERT_TRUE(recorder.peek(&view));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::trace::RecordType::Begin),
                static_cast<uint8_t>(view.type));
    acknowledge(recorder, view);
    TEST_ASSERT_TRUE(
      recorder.captureStep(first_input, command(), outputGuard()));
    TEST_ASSERT_TRUE(recorder.peek(&view));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::trace::RecordType::Config),
                static_cast<uint8_t>(view.type));
    acknowledge(recorder, view);
    TEST_ASSERT_TRUE(
      recorder.captureStep(first_input, command(), outputGuard()));
    TEST_ASSERT_FALSE(
      recorder.captureStep(second_input, command(), outputGuard()));
  TEST_ASSERT_FALSE(recorder.active());
  TEST_ASSERT_TRUE(recorder.terminalPending());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::CaptureEndReason::TransportOverflow),
      static_cast<uint8_t>(recorder.summary().end_reason));
}

void test_recorder_accepts_a_maximum_canonical_step() {
  hil::trace::TraceRecorder recorder;
  controller::ControllerStepInput input = stepInput(10, 1);
  controller::RobotCommand maximum = command();
  maximum.goals.count = config::kNumServos;
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    maximum.goals.joints[index].id = static_cast<uint8_t>(index + 1);
    maximum.goals.joints[index].tick =
        static_cast<uint16_t>(2000 + index);
    maximum.goals.joints[index].leg = index / config::kJointsPerLeg;
    maximum.goals.joints[index].joint = index % config::kJointsPerLeg;
    maximum.goals.joints[index].clamped = (index & 1u) != 0;
  }

  hil::CaptureRequest request;
  request.session_id = 4;
  request.capture_id = 10;
  request.step_count = 1;
  TEST_ASSERT_TRUE(recorder.begin(request, outputGuard(), &input.config));

  hil::trace::RecordView view;
  TEST_ASSERT_TRUE(recorder.peek(&view));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::trace::RecordType::Begin),
                          static_cast<uint8_t>(view.type));
  acknowledge(recorder, view);
  TEST_ASSERT_TRUE(recorder.captureStep(input, maximum, outputGuard()));
  TEST_ASSERT_TRUE(recorder.peek(&view));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::trace::RecordType::Config),
                          static_cast<uint8_t>(view.type));
  acknowledge(recorder, view);
  TEST_ASSERT_TRUE(recorder.captureStep(input, maximum, outputGuard()));
  TEST_ASSERT_TRUE(recorder.peek(&view));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::trace::RecordType::Step),
                          static_cast<uint8_t>(view.type));
  TEST_ASSERT_TRUE(view.logical_length <= hil::trace::kMaxLogicalRecordBytes);
  assertReassembles(recorder, view);
  acknowledge(recorder, view);
}

void test_recorder_aborts_on_config_revision_change() {
  hil::trace::TraceRecorder recorder;
  controller::ControllerStepInput initial = stepInput(10, 1);
  controller::ControllerStepInput changed = stepInput(20, 2);
  hil::CaptureRequest request;
  request.session_id = 4;
  request.capture_id = 11;
  request.step_count = 1;
  TEST_ASSERT_TRUE(recorder.begin(request, outputGuard(), &initial.config));

  hil::trace::RecordView view;
  TEST_ASSERT_TRUE(recorder.peek(&view));
  acknowledge(recorder, view);
  TEST_ASSERT_FALSE(recorder.captureStep(changed, command(), outputGuard()));
  TEST_ASSERT_FALSE(recorder.active());
  TEST_ASSERT_TRUE(recorder.terminalPending());
  TEST_ASSERT_TRUE(recorder.peek(&view));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::trace::RecordType::End),
                          static_cast<uint8_t>(view.type));
  acknowledge(recorder, view);
  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  hil::CaptureEndReason reason = hil::CaptureEndReason::Complete;
  TEST_ASSERT_TRUE(recorder.takeCompletion(&session_id, &capture_id, &reason));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::CaptureEndReason::TransportOverflow),
      static_cast<uint8_t>(reason));
}

void test_recorder_enforces_capture_step_bounds() {
  const uint8_t valid_step_counts[] = {1, hil::trace::kMaxCaptureSteps};
  for (uint8_t index = 0; index < sizeof(valid_step_counts); ++index) {
    const uint8_t step_count = valid_step_counts[index];
    hil::trace::TraceRecorder recorder;
    hil::CaptureRequest request;
    request.session_id = 4;
    request.capture_id = step_count;
    request.step_count = step_count;
    TEST_ASSERT_TRUE(recorder.begin(request, outputGuard()));
  }

  hil::trace::TraceRecorder recorder;
  hil::CaptureRequest request;
  request.session_id = 4;
  request.capture_id = 1;
  request.step_count = 0;
  TEST_ASSERT_FALSE(recorder.begin(request, outputGuard()));
  request.step_count = hil::trace::kMaxCaptureSteps + 1;
  TEST_ASSERT_FALSE(recorder.begin(request, outputGuard()));
}

void test_recorder_emits_terminal_record_after_transport_abort() {
  hil::trace::TraceRecorder recorder;
  hil::CaptureRequest request;
  request.session_id = 4;
  request.capture_id = 12;
  request.step_count = 1;
  TEST_ASSERT_TRUE(recorder.begin(request, outputGuard()));
  recorder.abandonCurrent(hil::CaptureEndReason::TransportTimeout,
                          outputGuard());
  TEST_ASSERT_FALSE(recorder.active());
  TEST_ASSERT_TRUE(recorder.terminalPending());

  hil::trace::RecordView view;
  TEST_ASSERT_TRUE(recorder.peek(&view));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::trace::RecordType::End),
                          static_cast<uint8_t>(view.type));
  acknowledge(recorder, view);
  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  hil::CaptureEndReason reason = hil::CaptureEndReason::Complete;
  TEST_ASSERT_TRUE(recorder.takeCompletion(&session_id, &capture_id, &reason));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::CaptureEndReason::TransportTimeout),
      static_cast<uint8_t>(reason));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_recorder_emits_begin_config_step_output_and_end_in_order);
  RUN_TEST(test_recorder_aborts_when_api_handoff_is_not_drained);
  RUN_TEST(test_recorder_accepts_a_maximum_canonical_step);
  RUN_TEST(test_recorder_aborts_on_config_revision_change);
  RUN_TEST(test_recorder_enforces_capture_step_bounds);
  RUN_TEST(test_recorder_emits_terminal_record_after_transport_abort);
  return UNITY_END();
}