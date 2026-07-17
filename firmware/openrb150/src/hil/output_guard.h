#pragma once

#include <stdint.h>

#include "../config/config_schema.h"

namespace hil {

#if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
constexpr bool kBuildOutputDisabled = true;
#else
constexpr bool kBuildOutputDisabled = false;
#endif

struct OutputGuardStatus {
  bool output_disabled = false;
  bool power_guard_active = false;
  bool torque_guard_active = false;
  bool goal_guard_active = false;
  bool write_guard_active = false;
  uint32_t blocked_power_enable = 0;
  uint32_t blocked_torque_enable = 0;
  uint32_t blocked_goal_write = 0;
  uint32_t blocked_dxl_write = 0;
  uint32_t last_goal_sequence = 0;
  uint8_t last_goal_count = 0;
};

struct GoalTargetRecord {
  uint8_t id = 0;
  int32_t tick = 0;
};

constexpr uint8_t kMaxRecordedGoalTargets = config::kNumServos;

// Goal snapshots are only consumed by the physical-output-disabled HIL image.
// Host tests retain them so OutputGuard(true) stays fully testable.
#if defined(HEXAPOD_HIL_OUTPUT_DISABLED) || !defined(ARDUINO_ARCH_SAMD)
#define HEXAPOD_OUTPUT_GUARD_RECORD_GOALS 1
#else
#define HEXAPOD_OUTPUT_GUARD_RECORD_GOALS 0
#endif

class OutputGuard {
 public:
  explicit OutputGuard(bool output_disabled = kBuildOutputDisabled)
      : output_disabled_(output_disabled) {}

  void reset();

  bool outputDisabled() const { return output_disabled_; }
  bool allowPowerEnable();
  bool allowTorque(bool on);
  bool allowGoalWrite(uint8_t goal_count = 0);
  void recordBlockedGoal(uint8_t index, uint8_t id, int32_t tick);
  void finishBlockedGoalWrite();
  uint8_t copyLastBlockedGoals(GoalTargetRecord* out, uint8_t out_cap) const;
  bool allowDxlWrite();
  OutputGuardStatus status() const;

 private:
  bool output_disabled_ = false;
  volatile uint32_t blocked_power_enable_ = 0;
  volatile uint32_t blocked_torque_enable_ = 0;
  volatile uint32_t blocked_goal_write_ = 0;
  volatile uint32_t blocked_dxl_write_ = 0;
  volatile uint32_t last_goal_sequence_ = 0;
  volatile uint8_t last_goal_count_ = 0;
  volatile bool goal_capture_in_progress_ = false;
#if HEXAPOD_OUTPUT_GUARD_RECORD_GOALS
  GoalTargetRecord last_blocked_goals_[kMaxRecordedGoalTargets] = {};
#endif
};

OutputGuard& outputGuard();
bool outputDisabled();

}  // namespace hil