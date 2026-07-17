#include "trace_recorder.h"

#include <string.h>

#include "../config/config_schema.h"
#include "../protocol/crc16.h"
#include "../protocol/framing.h"

namespace hil {
namespace trace {
namespace {

class Writer {
 public:
  Writer(uint8_t* out, uint16_t out_cap) : out_(out), out_cap_(out_cap) {}

  void u8(uint8_t value) { byte(value); }
  void i8(int8_t value) { byte(static_cast<uint8_t>(value)); }
  void boolean(bool value) { byte(value ? 1 : 0); }
  void u16(uint16_t value) {
    byte(static_cast<uint8_t>(value & 0xFF));
    byte(static_cast<uint8_t>((value >> 8) & 0xFF));
  }
  void i16(int16_t value) { u16(static_cast<uint16_t>(value)); }
  void u32(uint32_t value) {
    byte(static_cast<uint8_t>(value & 0xFF));
    byte(static_cast<uint8_t>((value >> 8) & 0xFF));
    byte(static_cast<uint8_t>((value >> 16) & 0xFF));
    byte(static_cast<uint8_t>((value >> 24) & 0xFF));
  }
  void i32(int32_t value) { u32(static_cast<uint32_t>(value)); }
  void f32(float value) {
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    u32(bits);
  }
  void bytes(const uint8_t* values, uint16_t count) {
    for (uint16_t index = 0; index < count; ++index) byte(values[index]);
  }

  uint16_t total() const { return total_; }
  bool ok() const { return ok_; }

 private:
  void byte(uint8_t value) {
    if (out_ == nullptr || total_ >= out_cap_) {
      ok_ = false;
      return;
    }
    out_[total_++] = value;
  }

  uint8_t* out_ = nullptr;
  uint16_t out_cap_ = 0;
  uint16_t total_ = 0;
  bool ok_ = true;
};

void writeGuard(Writer& writer, const OutputGuardStatus& guard) {
  writer.boolean(guard.output_disabled);
  writer.boolean(guard.power_guard_active);
  writer.boolean(guard.torque_guard_active);
  writer.boolean(guard.goal_guard_active);
  writer.boolean(guard.write_guard_active);
  writer.u32(guard.blocked_power_enable);
  writer.u32(guard.blocked_torque_enable);
  writer.u32(guard.blocked_goal_write);
  writer.u32(guard.blocked_dxl_write);
  writer.u32(guard.last_goal_sequence);
  writer.u8(guard.last_goal_count);
}

void writeControllerCommand(Writer& writer,
                            const controller::ControllerCommand& command) {
  writer.boolean(command.valid);
  writer.boolean(command.failsafe);
  writer.boolean(command.ever_seen);
  writer.u32(command.frame_ms);
  writer.boolean(command.arm_request);
  writer.boolean(command.estop);
  writer.boolean(command.host_authority);
  writer.u8(static_cast<uint8_t>(command.mode));
  writer.u8(command.gait_index);
  writer.f32(command.twist_vx);
  writer.f32(command.twist_vy);
  writer.f32(command.twist_wz);
  writer.f32(command.pose_x_mm);
  writer.f32(command.pose_y_mm);
  writer.f32(command.pose_z_mm);
  writer.f32(command.pose_roll);
  writer.f32(command.pose_pitch);
  writer.f32(command.pose_yaw);
  writer.f32(command.trim_roll);
  writer.f32(command.trim_pitch);
  writer.f32(command.speed);
  writer.f32(command.body_height);
  writer.f32(command.stride);
  writer.f32(command.step_height);
  writer.boolean(command.feat_foot_contact);
  writer.boolean(command.feat_terrain_leveling);
  writer.boolean(command.feat_passive_pose);
  writer.u8(static_cast<uint8_t>(command.trick));
}

void writeRobotState(Writer& writer, const controller::RobotState& state) {
  writer.u16(state.battery.millivolts);
  writer.boolean(state.battery.valid);
  writer.u8(static_cast<uint8_t>(state.battery.validity));

  writer.u8(state.dxl.servo_count);
  writer.u8(static_cast<uint8_t>(state.dxl.validity));
  writer.boolean(state.dxl.configured_servo_coverage);
  writer.u32(state.dxl.pose_known_mask);
  writer.u32(state.dxl.config_revision);
  writer.boolean(state.dxl.torque_off);
  writer.boolean(state.dxl.hard_fault);
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    const dxl::ServoStatus& servo = state.dxl.servos[index];
    writer.u8(servo.id);
    writer.i32(servo.present_position);
    writer.i32(servo.present_velocity);
    writer.i32(servo.present_load);
    writer.u16(servo.present_voltage_mv);
    writer.i8(servo.present_temperature_c);
    writer.u8(servo.hardware_error);
    writer.boolean(servo.torque_enabled);
    writer.boolean(servo.ok);
  }

  writer.u8(state.contact.present_mask);
  writer.u8(static_cast<uint8_t>(state.contact.validity));
  for (uint8_t leg = 0; leg < sensors::kNumFeet; ++leg) {
    const sensors::LegContactState& foot = state.contact.feet[leg];
    writer.u32(foot.timestamp_ms);
    writer.u16(foot.proximity_raw);
    writer.i32(foot.pressure_raw);
    writer.i32(foot.pressure_baseline);
    writer.i32(foot.pressure_delta);
    writer.u8(static_cast<uint8_t>(foot.state));
    writer.u8(foot.confidence);
    writer.boolean(foot.near_surface);
    writer.boolean(foot.touch);
    writer.boolean(foot.loaded);
    writer.boolean(foot.release);
    writer.boolean(foot.stale);
    writer.boolean(foot.fault);
  }
  writer.boolean(state.config_ready);
  writer.boolean(state.watchdog_fault);
}

void writeMotionIntent(Writer& writer, const protocol::MotionIntent& intent) {
  writer.u32(intent.seq);
  writer.u8(intent.gait);
  writer.u16(intent.body_height_mm);
  writer.u16(intent.stride_len_mm);
  writer.u16(intent.step_height_mm);
  writer.u8(intent.duty_x255);
  writer.u8(intent.speed_x255);
  writer.f32(intent.twist_vx);
  writer.f32(intent.twist_vy);
  writer.f32(intent.twist_wz);
  writer.f32(intent.pose_x_mm);
  writer.f32(intent.pose_y_mm);
  writer.f32(intent.pose_z_mm);
  writer.f32(intent.pose_roll);
  writer.f32(intent.pose_pitch);
  writer.f32(intent.pose_yaw);
}

void writeMaintenanceIntent(Writer& writer,
                            const controller::MaintenanceIntent& intent) {
  writer.boolean(intent.lock_held);
  writer.u32(intent.lock_token);
  writer.u8(static_cast<uint8_t>(intent.control_mode));
  const protocol::MaintTargetSet& targets = intent.targets;
  writer.u32(targets.seq);
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    for (uint8_t joint = 0; joint < config::kJointsPerLeg; ++joint) {
      writer.u16(targets.tick[leg][joint]);
    }
  }
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    for (uint8_t joint = 0; joint < config::kJointsPerLeg; ++joint) {
      writer.boolean(targets.set[leg][joint]);
    }
  }
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    for (uint8_t joint = 0; joint < config::kJointsPerLeg; ++joint) {
      writer.boolean(targets.clamped[leg][joint]);
    }
  }
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    writer.i16(targets.foot_x_mm[leg]);
    writer.i16(targets.foot_y_mm[leg]);
    writer.i16(targets.foot_z_mm[leg]);
    writer.boolean(targets.leg_target_set[leg]);
    writer.boolean(targets.leg_reachable[leg]);
    writer.boolean(targets.leg_clamped[leg]);
  }
}

void writeControllerIntent(Writer& writer,
                           const controller::ControllerIntent& intent) {
  writeControllerCommand(writer, intent.rc.command);
  writer.boolean(intent.rc.ever_seen);
  writer.boolean(intent.rc.kill);
  writer.boolean(intent.rc.armed);
  writer.boolean(intent.rc.failsafe);
  writer.boolean(intent.rc.autonomy_enabled);
  writeMotionIntent(writer, intent.motion);
  writeMaintenanceIntent(writer, intent.maintenance);
  writer.boolean(intent.features.foot_contact_enabled);
  writer.boolean(intent.features.terrain_leveling_enabled);
  writer.boolean(intent.features.sensor_polling_enabled);
  writer.boolean(intent.features.jetson_control_enabled);
  writer.boolean(intent.features.passive_pose_enabled);
  writer.boolean(intent.host_estop);
  writer.boolean(intent.host_disarm);
  writer.boolean(intent.clear_fault_requested);
  writer.boolean(intent.passive_requested);
  writer.boolean(intent.jetson_heartbeat_received);
}

void writeCommand(Writer& writer, const controller::RobotCommand& command) {
  writer.u8(static_cast<uint8_t>(command.safety_state));
  writer.u8(static_cast<uint8_t>(command.fault_reason));
  writer.u8(static_cast<uint8_t>(command.command_source));
  writer.boolean(command.motion_authorized);
  writer.boolean(command.motion_gate);
  writer.boolean(command.allow_dxl_power);
  writer.boolean(command.allow_torque);
  writer.boolean(command.goal_valid);
  const controller::ControllerDiagnostics& diagnostics = command.diagnostics;
  writer.u32(diagnostics.config_revision);
  writer.u32(diagnostics.intent_sequence);
  writer.u8(diagnostics.confident_contact_feet);
  writer.boolean(diagnostics.motion_gate_rising);
  writer.boolean(diagnostics.motion_gate_falling);
  writer.boolean(diagnostics.config_reapplied);
  writer.boolean(diagnostics.maintenance_session_started);
  writer.boolean(diagnostics.clear_maintenance_targets);
  writer.boolean(diagnostics.clear_maintenance_lock);
  writer.boolean(diagnostics.clear_passive_request);
  writer.boolean(diagnostics.any_goal_clamped);
  writer.boolean(diagnostics.any_goal_unreachable);
  writer.boolean(diagnostics.any_goal_reach_limited);
  writer.u8(command.goals.count);
  writer.boolean(command.goals.any_unreachable);
  writer.boolean(command.goals.any_reach_limited);
  const uint8_t count = command.goals.count > config::kNumServos
                            ? config::kNumServos
                            : command.goals.count;
  for (uint8_t index = 0; index < count; ++index) {
    const gait::PipelineJoint& goal = command.goals.joints[index];
    writer.u8(goal.id);
    writer.u16(goal.tick);
    writer.u8(goal.leg);
    writer.u8(goal.joint);
    writer.boolean(goal.clamped);
  }
}

bool guardChanged(const OutputGuardStatus& previous,
                  const OutputGuardStatus& current) {
  return previous.blocked_power_enable != current.blocked_power_enable ||
         previous.blocked_torque_enable != current.blocked_torque_enable ||
         previous.blocked_goal_write != current.blocked_goal_write ||
         previous.blocked_dxl_write != current.blocked_dxl_write ||
         previous.last_goal_sequence != current.last_goal_sequence;
}

}  // namespace

void TraceRecorder::reset() {
  summary_ = CaptureSummary{};
  initial_guard_ = OutputGuardStatus{};
  final_guard_ = OutputGuardStatus{};
  last_guard_ = OutputGuardStatus{};
  deferred_output_guard_ = OutputGuardStatus{};
  config_source_ = nullptr;
  next_step_sequence_ = 1;
  next_record_sequence_ = 1;
  emitted_record_count_ = 0;
  emitted_fragment_count_ = 0;
  config_revision_ = 0;
  config_payload_crc16_ = 0;
  marker_for_next_step_ = false;
  next_marker_id_ = 0;
  deferred_marker_pending_ = false;
  deferred_marker_id_ = 0;
  deferred_marker_time_ms_ = 0;
  deferred_marker_safety_state_ = 0;
  deferred_marker_step_sequence_ = 0;
  deferred_output_pending_ = false;
  deferred_output_step_sequence_ = 0;
  config_pending_ = false;
  completion_pending_ = false;
  slot_ready_ = false;
  slot_view_ = RecordView{};
  memset(slot_, 0, sizeof(slot_));
}

bool TraceRecorder::begin(const CaptureRequest& request,
                          const OutputGuardStatus& initial_guard,
                          const controller::ControllerConfigSnapshot* config) {
  if (summary_.active || summary_.terminal_pending || request.session_id == 0 ||
      request.capture_id == 0 || request.step_count < 1 ||
      request.step_count > kMaxCaptureSteps) {
    return false;
  }
  reset();
  summary_.session_id = request.session_id;
  summary_.capture_id = request.capture_id;
  summary_.requested_steps = request.step_count;
  summary_.active = true;
  initial_guard_ = initial_guard;
  final_guard_ = initial_guard;
  last_guard_ = initial_guard;
  config_source_ = config;
  if (config != nullptr) {
    config_revision_ = config->revision;
    if (config->valid) {
      const uint16_t config_length = config::serializeRobotConfig(
          config->robot, slot_, sizeof(slot_));
      if (config_length == 0) {
        reset();
        return false;
      }
      config_payload_crc16_ = protocol::crc16(slot_, config_length);
    }
  }
  config_pending_ = true;
  return publishBegin();
}

void TraceRecorder::finishCapture(CaptureEndReason reason,
                                  const OutputGuardStatus& guard) {
  summary_.active = false;
  summary_.terminal_pending = true;
  summary_.end_reason = reason;
  final_guard_ = guard;
}

bool TraceRecorder::captureStep(const controller::ControllerStepInput& input,
                                const controller::RobotCommand& command,
                                const OutputGuardStatus& guard) {
  return captureStep(input.state, input.intent, input.config, input.time,
                     command, guard);
}

bool TraceRecorder::captureStep(
    const controller::RobotState& state,
    const controller::ControllerIntent& intent,
  const controller::ControllerConfigSnapshot& config_snapshot,
    const controller::ControllerTime& time,
    const controller::RobotCommand& command,
    const OutputGuardStatus& guard) {
  if (!summary_.active) return false;
  if (slot_ready_) {
    // Begin/config records establish a self-contained trace before the first
    // captured step. Waiting for those tiny records does not lose a step
    // because recording has not started yet. Once a step is in flight, fail
    // closed rather than overwrite it.
    if (summary_.recorded_steps == 0) return false;
    finishCapture(CaptureEndReason::TransportOverflow, guard);
    return false;
  }
  if (command.goals.count > config::kNumServos) {
    finishCapture(CaptureEndReason::TransportOverflow, guard);
    return false;
  }

  if (config_source_ == nullptr) {
    config_source_ = &config_snapshot;
    config_revision_ = config_snapshot.revision;
    if (config_snapshot.valid) {
      const uint16_t config_length = config::serializeRobotConfig(
          config_snapshot.robot, slot_, sizeof(slot_));
      if (config_length == 0) {
        finishCapture(CaptureEndReason::TransportOverflow, guard);
        return false;
      }
      config_payload_crc16_ = protocol::crc16(slot_, config_length);
    }
  }
  if (config_snapshot.revision != config_revision_) {
    finishCapture(CaptureEndReason::TransportOverflow, guard);
    return false;
  }
  if (config_pending_) {
    config_pending_ = false;
    if (!publishConfig(config_snapshot)) {
      finishCapture(CaptureEndReason::TransportOverflow, guard);
      return false;
    }
    return true;
  }

  const uint32_t step_sequence = next_step_sequence_++;
  if (marker_for_next_step_) {
    deferred_marker_pending_ = true;
    deferred_marker_id_ = next_marker_id_;
    deferred_marker_time_ms_ = time.now_ms;
    deferred_marker_safety_state_ =
        static_cast<uint8_t>(command.safety_state);
    deferred_marker_step_sequence_ = step_sequence;
    marker_for_next_step_ = false;
    next_marker_id_ = 0;
  }
  if (guardChanged(last_guard_, guard)) {
    deferred_output_pending_ = true;
    deferred_output_guard_ = guard;
    deferred_output_step_sequence_ = step_sequence;
  }
  if (!publishStep(state, intent, config_snapshot, time, step_sequence,
                   command)) {
    finishCapture(CaptureEndReason::TransportOverflow, guard);
    return false;
  }
  last_guard_ = guard;
  ++summary_.recorded_steps;
  summary_.queue_high_water = 1;
  if (summary_.recorded_steps >= summary_.requested_steps) {
    finishCapture(CaptureEndReason::Complete, guard);
  }
  return true;
}

bool TraceRecorder::markNext(uint32_t marker_id) {
  if (!summary_.active || marker_for_next_step_) return false;
  marker_for_next_step_ = true;
  next_marker_id_ = marker_id;
  return true;
}

void TraceRecorder::abort(CaptureEndReason reason,
                          const OutputGuardStatus& guard) {
  if (!summary_.active && !summary_.terminal_pending) return;
  finishCapture(reason, guard);
}

bool TraceRecorder::publish(RecordType type, uint16_t length) {
  if (length == 0 || length > sizeof(slot_)) return false;
  slot_view_.type = type;
  slot_view_.session_id = summary_.session_id;
  slot_view_.capture_id = summary_.capture_id;
  slot_view_.record_seq = next_record_sequence_++;
  slot_view_.logical_length = length;
  slot_view_.logical_crc16 = protocol::crc16(slot_, length);
  slot_ready_ = true;
  return true;
}

bool TraceRecorder::publishBegin() {
  Writer writer(slot_, sizeof(slot_));
  writer.u16(kSchemaVersion);
  writer.u8(1);  // Atsamd21OutputDisabled
  writer.u8(protocol::kVersionMajor);
  writer.u8(protocol::kVersionMinor);
  writer.u32(summary_.session_id);
  writer.u32(summary_.capture_id);
  writer.u32(config_revision_);
  writer.u16(config_payload_crc16_);
  writer.u8(summary_.requested_steps);
  writeGuard(writer, initial_guard_);
  return writer.ok() && publish(RecordType::Begin, writer.total());
}

bool TraceRecorder::publishConfig(
    const controller::ControllerConfigSnapshot& config) {
  constexpr uint16_t kHeaderLength = 10;
  uint16_t payload_length = 0;
  uint16_t payload_crc = 0;
  if (config.valid) {
    payload_length = config::serializeRobotConfig(
        config.robot, &slot_[kHeaderLength],
        static_cast<uint16_t>(sizeof(slot_) - kHeaderLength));
    if (payload_length == 0) return false;
    payload_crc = protocol::crc16(&slot_[kHeaderLength], payload_length);
    if (payload_crc != config_payload_crc16_) return false;
  }
  Writer writer(slot_, sizeof(slot_));
  writer.u32(config.revision);
  writer.boolean(config.valid);
  writer.boolean(config.persistent);
  writer.u16(payload_length);
  writer.u16(payload_crc);
  writer.bytes(&slot_[kHeaderLength], payload_length);
  return writer.ok() && publish(RecordType::Config, writer.total());
}

bool TraceRecorder::publishMarker(uint32_t marker_id, uint32_t now_ms,
                                  uint8_t safety_state,
                                  uint32_t step_sequence) {
  Writer writer(slot_, sizeof(slot_));
  writer.u32(marker_id);
  writer.u32(now_ms);
  writer.u8(safety_state);
  writer.u32(step_sequence);
  return writer.ok() && publish(RecordType::Marker, writer.total());
}

bool TraceRecorder::publishOutputBlocked(
    uint32_t step_sequence, const OutputGuardStatus& guard) {
  Writer writer(slot_, sizeof(slot_));
  writer.u32(step_sequence);
  writeGuard(writer, guard);
  return writer.ok() && publish(RecordType::OutputBlocked, writer.total());
}

bool TraceRecorder::publishStep(
    const controller::RobotState& state,
    const controller::ControllerIntent& intent,
    const controller::ControllerConfigSnapshot& config,
    const controller::ControllerTime& time, uint32_t step_sequence,
    const controller::RobotCommand& command) {
  Writer writer(slot_, sizeof(slot_));
  writer.u32(step_sequence);
  writer.u32(time.now_ms);
  writer.u32(time.dt_ms);
  writer.boolean(time.valid);
  writeRobotState(writer, state);
  writeControllerIntent(writer, intent);
  writer.u32(config.revision);
  writer.boolean(config.valid);
  writer.boolean(config.persistent);
  writeCommand(writer, command);
  return writer.ok() && publish(RecordType::Step, writer.total());
}

bool TraceRecorder::publishEnd() {
  Writer writer(slot_, sizeof(slot_));
  writer.u8(static_cast<uint8_t>(summary_.end_reason));
  writer.u8(summary_.requested_steps);
  writer.u8(summary_.recorded_steps);
  writer.u8(summary_.queue_high_water);
  writer.u32(emitted_record_count_);
  writer.u32(emitted_fragment_count_);
  writeGuard(writer, final_guard_);
  return writer.ok() && publish(RecordType::End, writer.total());
}

bool TraceRecorder::publishDeferred() {
  if (deferred_marker_pending_) {
    deferred_marker_pending_ = false;
    return publishMarker(deferred_marker_id_, deferred_marker_time_ms_,
                         deferred_marker_safety_state_,
                         deferred_marker_step_sequence_);
  }
  if (deferred_output_pending_) {
    deferred_output_pending_ = false;
    return publishOutputBlocked(deferred_output_step_sequence_,
                                deferred_output_guard_);
  }
  return false;
}

bool TraceRecorder::peek(RecordView* view) {
  if (view == nullptr) return false;
  if (!slot_ready_ && summary_.terminal_pending) {
    if (!publishEnd()) return false;
  }
  if (!slot_ready_) return false;
  *view = slot_view_;
  return true;
}

bool TraceRecorder::copySlice(const RecordView& view, uint16_t offset,
                              uint8_t* out, uint16_t out_cap,
                              uint16_t length) const {
  if (!slot_ready_ || view.record_seq != slot_view_.record_seq ||
      view.logical_length != slot_view_.logical_length ||
      offset > view.logical_length || length > view.logical_length - offset ||
      out == nullptr || out_cap < length) {
    return false;
  }
  for (uint16_t index = 0; index < length; ++index) {
    out[index] = slot_[offset + index];
  }
  return true;
}

void TraceRecorder::acknowledge(const RecordView& view) {
  if (!slot_ready_ || view.record_seq != slot_view_.record_seq) {
    return;
  }
  const RecordType type = slot_view_.type;
  ++emitted_record_count_;
  if (type == RecordType::End) {
    slot_ready_ = false;
    summary_.terminal_pending = false;
    completion_pending_ = true;
    return;
  }

  // Keep slot_ready_ asserted while replacing an acknowledged Step with an
  // associated marker/output record. If controlTask preempts this short copy,
  // it sees a full slot and aborts the capture instead of overwriting it.
  if (publishDeferred()) return;

  slot_ready_ = false;
  if (summary_.terminal_pending) {
    (void)publishEnd();
  }
}

void TraceRecorder::abandonCurrent(CaptureEndReason reason,
                                   const OutputGuardStatus& guard) {
  slot_ready_ = false;
  deferred_marker_pending_ = false;
  deferred_output_pending_ = false;
  finishCapture(reason, guard);
}

void TraceRecorder::noteFragmentsSent(uint8_t count) {
  emitted_fragment_count_ += count;
}

bool TraceRecorder::takeCompletion(uint32_t* session_id, uint32_t* capture_id,
                                   CaptureEndReason* reason) {
  if (!completion_pending_ || session_id == nullptr || capture_id == nullptr ||
      reason == nullptr) {
    return false;
  }
  *session_id = summary_.session_id;
  *capture_id = summary_.capture_id;
  *reason = summary_.end_reason;
  completion_pending_ = false;
  return true;
}

}  // namespace trace
}  // namespace hil