#pragma once

#include <stdint.h>

// The debug OLED is compiled out in HIL builds and in the no-OLED diagnostic
// image (-D HEXAPOD_NO_DEBUG_OLED), which exists to A/B the display's SRAM
// contribution (SparkFun object + framebuffer + Wire) against overflow bugs.
#if !defined(HEXAPOD_HIL_OUTPUT_DISABLED) && !defined(HEXAPOD_NO_DEBUG_OLED)
#define HEXAPOD_DEBUG_OLED_AVAILABLE 1
#else
#define HEXAPOD_DEBUG_OLED_AVAILABLE 0
#endif

#if HEXAPOD_DEBUG_OLED_AVAILABLE
#include <SparkFun_Qwiic_OLED.h>
#endif

#include "i2c_bus.h"

namespace sensors {

struct DebugDisplayState {
  uint16_t body_height_mm = 0;
  uint16_t stride_mm = 0;
  uint16_t step_height_mm = 0;
  int16_t trim_roll_cdeg = 0;
  int16_t trim_pitch_cdeg = 0;
  uint8_t duty_x255 = 0;
  uint8_t speed_x255 = 0;
  uint8_t gait = 0;
  uint8_t safety_state = 0;
  uint8_t fault_reason = 0;
  uint8_t tune_param = 0;
  uint8_t servo_count = 0;
  uint8_t foot_present_mask = 0;
  uint8_t i2c_error = 0;
  uint16_t battery_mv = 0;
  bool battery_valid = false;
  bool rc_seen = false;
  bool rc_failsafe = true;
  bool rc_armed = false;
  bool dxl_power = false;
  bool dxl_hard_fault = false;
  bool config_storage = false;
  bool mux_present = false;
  bool tune_active = false;
  bool capture_recording = false;
  uint32_t capture_samples = 0;
  uint8_t view = 0;
};

class QwiicDebugOled {
 public:
  static constexpr uint8_t kDefaultAddress = 0x3D;
  static constexpr uint8_t kAltAddress = 0x3C;
  static constexpr uint8_t kPageCount = 8;
  static constexpr uint8_t kViewCount = 2;

  explicit QwiicDebugOled(i2c::I2cBus& bus) : bus_(bus) {}

  bool begin(uint8_t address);
  bool drawPage(uint8_t page, const DebugDisplayState& state);
  bool ready() const { return ready_; }

 private:
  void formatLine(uint8_t page, const DebugDisplayState& state,
                  char out[22]) const;

  i2c::I2cBus& bus_;
  bool ready_ = false;
#if HEXAPOD_DEBUG_OLED_AVAILABLE
  Qwiic1in3OLED oled_;
#endif
};

}  // namespace sensors