#include "output_guard.h"

namespace hil {
namespace {

OutputGuard g_outputGuard(kBuildOutputDisabled);

}  // namespace

void OutputGuard::reset() {
  blocked_power_enable_ = 0;
  blocked_torque_enable_ = 0;
  blocked_goal_write_ = 0;
  blocked_dxl_write_ = 0;
  last_goal_sequence_ = 0;
  last_goal_count_ = 0;
  goal_capture_in_progress_ = false;
#if HEXAPOD_OUTPUT_GUARD_RECORD_GOALS
  for (uint8_t i = 0; i < kMaxRecordedGoalTargets; ++i) {
    last_blocked_goals_[i] = GoalTargetRecord{};
  }
#endif
}

bool OutputGuard::allowPowerEnable() {
  if (!output_disabled_) return true;
  ++blocked_power_enable_;
  return false;
}

bool OutputGuard::allowTorque(bool on) {
  if (!output_disabled_ || !on) return true;
  ++blocked_torque_enable_;
  return false;
}

bool OutputGuard::allowGoalWrite(uint8_t goal_count) {
  if (!output_disabled_) return true;
  ++blocked_goal_write_;
  goal_capture_in_progress_ = true;
  last_goal_count_ =
      goal_count > kMaxRecordedGoalTargets ? kMaxRecordedGoalTargets : goal_count;
  ++last_goal_sequence_;
  return false;
}

void OutputGuard::recordBlockedGoal(uint8_t index, uint8_t id, int32_t tick) {
  if (!output_disabled_ || index >= last_goal_count_) return;
#if HEXAPOD_OUTPUT_GUARD_RECORD_GOALS
  last_blocked_goals_[index].id = id;
  last_blocked_goals_[index].tick = tick;
#else
  (void)id;
  (void)tick;
#endif
}

void OutputGuard::finishBlockedGoalWrite() {
  if (!output_disabled_) return;
  goal_capture_in_progress_ = false;
}

uint8_t OutputGuard::copyLastBlockedGoals(GoalTargetRecord* out,
                                          uint8_t out_cap) const {
  if (out == nullptr || goal_capture_in_progress_) return 0;
#if HEXAPOD_OUTPUT_GUARD_RECORD_GOALS
  const uint8_t count = last_goal_count_ < out_cap ? last_goal_count_ : out_cap;
  for (uint8_t i = 0; i < count; ++i) {
    out[i] = last_blocked_goals_[i];
  }
  return count;
#else
  (void)out_cap;
  return 0;
#endif
}

bool OutputGuard::allowDxlWrite() {
  if (!output_disabled_) return true;
  ++blocked_dxl_write_;
  return false;
}

OutputGuardStatus OutputGuard::status() const {
  OutputGuardStatus snapshot;
  snapshot.output_disabled = output_disabled_;
  snapshot.power_guard_active = output_disabled_;
  snapshot.torque_guard_active = output_disabled_;
  snapshot.goal_guard_active = output_disabled_;
  snapshot.write_guard_active = output_disabled_;
  snapshot.blocked_power_enable = blocked_power_enable_;
  snapshot.blocked_torque_enable = blocked_torque_enable_;
  snapshot.blocked_goal_write = blocked_goal_write_;
  snapshot.blocked_dxl_write = blocked_dxl_write_;
  snapshot.last_goal_sequence = last_goal_sequence_;
  snapshot.last_goal_count = last_goal_count_;
  return snapshot;
}

OutputGuard& outputGuard() { return g_outputGuard; }

bool outputDisabled() { return g_outputGuard.outputDisabled(); }

}  // namespace hil