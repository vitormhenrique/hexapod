#include "capture_log.h"

namespace logging {
namespace {

class CsvWriter {
 public:
  CsvWriter(char* out, size_t capacity) : out_(out), capacity_(capacity) {}

  void text(const char* value) {
    while (*value != '\0') character(*value++);
  }

  void comma() { character(','); }

  void unsignedValue(uint32_t value) {
    char digits[10];
    uint8_t count = 0;
    do {
      digits[count++] = static_cast<char>('0' + value % 10u);
      value /= 10u;
    } while (value > 0 && count < sizeof(digits));
    while (count > 0) character(digits[--count]);
  }

  void signedValue(int32_t value) {
    if (value < 0) {
      character('-');
      unsignedValue(static_cast<uint32_t>(-(static_cast<int64_t>(value))));
    } else {
      unsignedValue(static_cast<uint32_t>(value));
    }
  }

  size_t finish() {
    character('\n');
    if (capacity_ > 0) {
      out_[length_ < capacity_ ? length_ : capacity_ - 1] = '\0';
    }
    return overflow_ ? 0 : length_;
  }

 private:
  void character(char value) {
    if (length_ + 1 < capacity_) {
      out_[length_] = value;
    } else {
      overflow_ = true;
    }
    ++length_;
  }

  char* out_;
  size_t capacity_;
  size_t length_ = 0;
  bool overflow_ = false;
};

void unsignedField(CsvWriter& writer, uint32_t value) {
  writer.comma();
  writer.unsignedValue(value);
}

void signedField(CsvWriter& writer, int32_t value) {
  writer.comma();
  writer.signedValue(value);
}

}  // namespace

size_t formatCaptureMarker(bool begin, uint32_t session, uint32_t timestamp_ms,
                           uint32_t samples, char* out, size_t out_cap) {
  if (out == nullptr || out_cap == 0) return 0;
  CsvWriter writer(out, out_cap);
  writer.text(begin ? "BEGIN" : "END");
  unsignedField(writer, session);
  unsignedField(writer, timestamp_ms);
  unsignedField(writer, samples);
  if (begin) unsignedField(writer, 2);  // CAPTURE.CSV schema version.
  return writer.finish();
}

size_t formatRemoteCaptureRow(uint32_t session, uint32_t sample,
                              const RemoteCapture& remote, char* out,
                              size_t out_cap) {
  if (out == nullptr || out_cap == 0) return 0;
  CsvWriter writer(out, out_cap);
  writer.text("R");
  unsignedField(writer, session);
  unsignedField(writer, sample);
  unsignedField(writer, remote.timestamp_ms);
  unsignedField(writer, remote.frame_sequence);
  unsignedField(writer, remote.flags);
  unsignedField(writer, remote.mode);
  unsignedField(writer, remote.gait);
  for (uint8_t i = 0; i < 4; ++i) signedField(writer, remote.gimbal[i]);
  for (uint8_t i = 0; i < 2; ++i) signedField(writer, remote.pot[i]);
  for (uint8_t i = 0; i < 2; ++i) signedField(writer, remote.encoder[i]);
  unsignedField(writer, remote.switch_mask);
  unsignedField(writer, remote.button_mask);
  unsignedField(writer, remote.toggle[0]);
  unsignedField(writer, remote.toggle[1]);
  unsignedField(writer, remote.nav_mask[0]);
  unsignedField(writer, remote.nav_mask[1]);
  for (uint8_t i = 0; i < 3; ++i) signedField(writer, remote.twist_milli[i]);
  for (uint8_t i = 0; i < 6; ++i) signedField(writer, remote.pose[i]);
  unsignedField(writer, remote.speed_x255);
  unsignedField(writer, remote.body_height_x255);
  unsignedField(writer, remote.stride_x255);
  unsignedField(writer, remote.step_height_x255);
  unsignedField(writer, remote.duty_x255);
  return writer.finish();
}

size_t formatAppliedMotionCaptureRow(uint32_t session, uint32_t sample,
                                     uint32_t timestamp_ms,
                                     const AppliedMotionCapture& motion,
                                     char* out, size_t out_cap) {
  if (out == nullptr || out_cap == 0) return 0;
  CsvWriter writer(out, out_cap);
  writer.text("C");
  unsignedField(writer, session);
  unsignedField(writer, sample);
  unsignedField(writer, timestamp_ms);
  unsignedField(writer, motion.goal_sequence);
  unsignedField(writer, motion.command_source);
  unsignedField(writer, motion.safety_state);
  unsignedField(writer, motion.gait);
  unsignedField(writer, motion.flags);
  unsignedField(writer, motion.body_height_mm);
  unsignedField(writer, motion.stride_mm);
  unsignedField(writer, motion.step_height_mm);
  unsignedField(writer, motion.duty_x255);
  unsignedField(writer, motion.speed_x255);
  return writer.finish();
}

size_t formatLegCaptureRow(uint32_t session, uint32_t sample,
                           uint32_t timestamp_ms, uint32_t goal_sequence,
                           const gait::PipelineLegTarget* legs, uint8_t count,
                           char* out, size_t out_cap) {
  if (out == nullptr || out_cap == 0 || (count > 0 && legs == nullptr)) return 0;
  CsvWriter writer(out, out_cap);
  writer.text("L");
  unsignedField(writer, session);
  unsignedField(writer, sample);
  unsignedField(writer, timestamp_ms);
  unsignedField(writer, goal_sequence);
  unsignedField(writer, count);
  for (uint8_t leg = 0; leg < count; ++leg) {
    unsignedField(writer, leg);
    signedField(writer, legs[leg].foot_x_mm);
    signedField(writer, legs[leg].foot_y_mm);
    signedField(writer, legs[leg].foot_z_mm);
    unsignedField(writer, legs[leg].flags);
  }
  return writer.finish();
}

size_t formatGoalCaptureRow(uint32_t session, uint32_t sample,
                            uint32_t timestamp_ms, uint32_t goal_sequence,
                            const GoalCapture* goals, uint8_t count, char* out,
                            size_t out_cap) {
  if (out == nullptr || out_cap == 0 || (count > 0 && goals == nullptr)) return 0;
  CsvWriter writer(out, out_cap);
  writer.text("G");
  unsignedField(writer, session);
  unsignedField(writer, sample);
  unsignedField(writer, timestamp_ms);
  unsignedField(writer, goal_sequence);
  unsignedField(writer, count);
  for (uint8_t index = 0; index < count; ++index) {
    const GoalCapture& goal = goals[index];
    unsignedField(writer, goal.id);
    unsignedField(writer, goal.leg);
    unsignedField(writer, goal.joint);
    unsignedField(writer, goal.goal_tick);
    signedField(writer, goal.goal_angle_centideg);
    unsignedField(writer, goal.flags);
  }
  return writer.finish();
}

size_t formatServoCaptureRow(uint32_t session, uint32_t sample,
                             uint32_t timestamp_ms,
                             const dxl::ServoStatus& servo,
                             int16_t present_angle_centideg, char* out,
                             size_t out_cap) {
  if (out == nullptr || out_cap == 0) return 0;
  CsvWriter writer(out, out_cap);
  writer.text("S");
  unsignedField(writer, session);
  unsignedField(writer, sample);
  unsignedField(writer, timestamp_ms);
  unsignedField(writer, servo.id);
  unsignedField(writer, servo.ok ? 1u : 0u);
  signedField(writer, servo.present_position);
  signedField(writer, present_angle_centideg);
  signedField(writer, servo.present_velocity);
  signedField(writer, servo.present_load);
  unsignedField(writer, servo.present_voltage_mv);
  signedField(writer, servo.present_temperature_c);
  unsignedField(writer, servo.hardware_error);
  unsignedField(writer, servo.torque_enabled ? 1u : 0u);
  return writer.finish();
}

}  // namespace logging