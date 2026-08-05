#pragma once

#include <stdint.h>

#include "dxl_model.h"
#include "../config/config_schema.h"

namespace dxl {

class FaultMonitor {
 public:
  static constexpr uint8_t kPersistentSamples = 3;

  void reset();
  void observe(uint8_t servo_index, TableKind table_kind, uint8_t error_bits);
  bool faulted() const { return faulted_; }

 private:
  uint8_t streak_[config::kNumServos] = {};
  bool faulted_ = false;
};

}  // namespace dxl
