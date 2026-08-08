#include "qwiic_debug_oled.h"

#include <string.h>
#if HEXAPOD_DEBUG_OLED_AVAILABLE
#include <res/qw_fnt_5x7.h>
#endif

namespace sensors {
namespace {

void appendChar(char out[22], uint8_t& pos, char value) {
  if (pos < 21) out[pos++] = value;
}

void appendText(char out[22], uint8_t& pos, const char* text) {
  while (*text != '\0' && pos < 21) out[pos++] = *text++;
}

void appendUnsigned(char out[22], uint8_t& pos, uint32_t value) {
  char digits[10];
  uint8_t count = 0;
  do {
    digits[count++] = static_cast<char>('0' + value % 10u);
    value /= 10u;
  } while (value > 0 && count < sizeof(digits));
  while (count > 0) appendChar(out, pos, digits[--count]);
}

void appendHexByte(char out[22], uint8_t& pos, uint8_t value) {
  constexpr char kHex[] = "0123456789ABCDEF";
  appendChar(out, pos, kHex[value >> 4]);
  appendChar(out, pos, kHex[value & 0x0Fu]);
}

void appendSignedCdeg(char out[22], uint8_t& pos, int16_t value) {
  appendChar(out, pos, value < 0 ? '-' : '+');
  const uint16_t magnitude = value < 0
                                 ? static_cast<uint16_t>(-static_cast<int32_t>(value))
                                 : static_cast<uint16_t>(value);
  appendUnsigned(out, pos, magnitude / 100u);
  appendChar(out, pos, '.');
  appendUnsigned(out, pos, (magnitude / 10u) % 10u);
}

const char* gaitName(uint8_t gait) {
  constexpr const char* names[] = {"STAND", "SIT", "TRIPOD",
                                    "RIPPLE", "WAVE", "CRAWL"};
  return gait < 6 ? names[gait] : "UNKNOWN";
}

const char* stateName(uint8_t state) {
  constexpr const char* names[] = {
      "BOOT",       "CONFIG",  "DISARMED", "ARM CHECK", "READY",
      "RC MANUAL",  "TERRAIN", "JETSON",   "MAINT",     "PASSIVE",
      "FAULT SOFT", "FAULT HARD", "ESTOP"};
  return state < 13 ? names[state] : "UNKNOWN";
}

}  // namespace

bool QwiicDebugOled::begin(uint8_t address) {
  ready_ = false;
#if !HEXAPOD_DEBUG_OLED_AVAILABLE
  (void)address;
  return false;
#else
  if (!oled_.begin(bus_.wire(), address)) return false;
  oled_.setFont(&QW_FONT_5X7);
  oled_.erase();
  oled_.display();
  ready_ = true;
  return true;
#endif
}

void QwiicDebugOled::formatLine(uint8_t page,
                                const DebugDisplayState& state,
                                char out[22]) const {
  memset(out, 0, 22);
  uint8_t pos = 0;

  if ((state.view % kViewCount) == 0) {
    switch (page) {
      case 0:
        appendText(out, pos, state.capture_recording ? "HEXAPOD RECORDING"
                                                     : "HEXAPOD STATUS");
        break;
      case 1:
        appendText(out, pos, "STATE ");
        appendText(out, pos, stateName(state.safety_state));
        break;
      case 2:
        appendText(out, pos, "FAULT ");
        appendUnsigned(out, pos, state.fault_reason);
        appendText(out, pos, " RC ");
        appendText(out, pos, state.rc_failsafe ? "FAIL" : state.rc_seen ? "OK" : "NONE");
        break;
      case 3:
        appendText(out, pos, "BAT ");
        if (state.battery_valid) {
          appendUnsigned(out, pos, state.battery_mv / 1000u);
          appendChar(out, pos, '.');
          appendUnsigned(out, pos, (state.battery_mv / 100u) % 10u);
          appendChar(out, pos, 'V');
        } else {
          appendText(out, pos, "INVALID");
        }
        appendText(out, pos, state.rc_armed ? " ARMED" : " SAFE");
        break;
      case 4:
        appendText(out, pos, "DXL ");
        appendUnsigned(out, pos, state.servo_count);
        appendText(out, pos, state.dxl_power ? " PWR" : " OFF");
        appendText(out, pos, state.dxl_hard_fault ? " FAULT" : " OK");
        break;
      case 5:
        appendText(out, pos, "I2C E");
        appendUnsigned(out, pos, state.i2c_error);
        appendText(out, pos, " FEET ");
        appendHexByte(out, pos, state.foot_present_mask);
        break;
      case 6:
        appendText(out, pos, "MUX ");
        appendText(out, pos, state.mux_present ? "OK" : "MISS");
        appendText(out, pos, " SD ");
        appendText(out, pos, state.config_storage ? "OK" : "MISS");
        break;
      default:
        appendText(out, pos, "GAIT ");
        appendText(out, pos, gaitName(state.gait));
        if (state.capture_recording) {
          appendText(out, pos, " #");
          appendUnsigned(out, pos, state.capture_samples);
        }
        break;
    }
    return;
  }

  switch (page) {
    case 0:
      appendText(out, pos, "MOTION / TUNING");
      break;
    case 1:
      appendText(out, pos, "GAIT ");
      appendText(out, pos, gaitName(state.gait));
      break;
    case 2:
      appendText(out, pos, "HEIGHT ");
      appendUnsigned(out, pos, state.body_height_mm);
      appendText(out, pos, " STRIDE ");
      appendUnsigned(out, pos, state.stride_mm);
      break;
    case 3:
      appendText(out, pos, "STEP ");
      appendUnsigned(out, pos, state.step_height_mm);
      appendText(out, pos, " DUTY ");
      appendUnsigned(out, pos, static_cast<uint16_t>(state.duty_x255) * 100u / 255u);
      appendChar(out, pos, '%');
      break;
    case 4:
      appendText(out, pos, "SPEED ");
      appendUnsigned(out, pos, static_cast<uint16_t>(state.speed_x255) * 100u / 255u);
      appendChar(out, pos, '%');
      break;
    case 5:
      appendText(out, pos, "TRIM R");
      appendSignedCdeg(out, pos, state.trim_roll_cdeg);
      appendText(out, pos, " P");
      appendSignedCdeg(out, pos, state.trim_pitch_cdeg);
      break;
    case 6:
      appendText(out, pos, state.tune_active ? "TUNE " : "TUNE OFF ");
      if (state.tune_active) {
        constexpr const char* params[] = {"STEP ", "STRIDE ", "DUTY "};
        appendText(out, pos, state.tune_param < 3 ? params[state.tune_param]
                                                  : "UNKNOWN ");
      }
      break;
    default:
      appendText(out, pos, state.capture_recording ? "REC SAMPLES " : "CAPTURE OFF #");
      appendUnsigned(out, pos, state.capture_samples);
      break;
  }
}

bool QwiicDebugOled::drawPage(uint8_t page,
                              const DebugDisplayState& state) {
#if !HEXAPOD_DEBUG_OLED_AVAILABLE
  (void)page;
  (void)state;
  return false;
#else
  if (!ready_ || page >= kPageCount) return false;
  char line[22];
  formatLine(page, state, line);
  const uint8_t y = static_cast<uint8_t>(page * 8u);
  oled_.rectangleFill(0, y, 128, 8, COLOR_BLACK);
  oled_.text(0, y, line, COLOR_WHITE);
  oled_.display();
  return true;
#endif
}

}  // namespace sensors
