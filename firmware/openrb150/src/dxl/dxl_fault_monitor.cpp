#include "dxl_fault_monitor.h"

namespace dxl {

void FaultMonitor::reset() {
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    streak_[index] = 0;
  }
  faulted_ = false;
}

void FaultMonitor::observe(uint8_t servo_index, TableKind table_kind,
                           uint8_t error_bits) {
  if (faulted_ || servo_index >= config::kNumServos) return;

  uint8_t immediate_mask = 0;
  uint8_t persistent_mask = 0;
  if (table_kind == TableKind::Mx28V2) {
    // MX(2.0): overheating, motor encoder, and electrical shock require an
    // immediate stop. Input-voltage and overload alarms must repeat because
    // power-up/motion transients can set one status sample.
    immediate_mask = 0x1C;
    persistent_mask = 0x21;
  } else {
    // Protocol 1: only overheating is unambiguously an immediate hardware
    // condition. Voltage, angle-limit, and overload alarms must repeat.
    // Range/checksum/instruction bits describe the packet/command and are not
    // a persistent servo hardware failure.
    immediate_mask = 0x04;
    persistent_mask = 0x23;
  }

  if ((error_bits & immediate_mask) != 0) {
    faulted_ = true;
    return;
  }
  if ((error_bits & persistent_mask) == 0) {
    streak_[servo_index] = 0;
    return;
  }
  if (streak_[servo_index] < 0xFF) ++streak_[servo_index];
  if (streak_[servo_index] >= kPersistentSamples) faulted_ = true;
}

}  // namespace dxl
