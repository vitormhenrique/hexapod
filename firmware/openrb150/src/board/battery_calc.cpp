#include "battery_calc.h"

namespace battery {
namespace {

// Round a non-negative float to the nearest integer and clamp to the uint16
// range so a conversion result can never wrap or overflow downstream fields.
uint16_t roundSaturateU16(float v) {
  if (v <= 0.0f) {
    return 0;
  }
  const float r = v + 0.5f;  // round half up (v is known non-negative)
  if (r >= 65535.0f) {
    return 65535u;
  }
  return static_cast<uint16_t>(r);
}

}  // namespace

uint16_t pinMilliVolts(uint16_t raw, uint16_t adc_max, float ref_mv) {
  if (adc_max == 0) {
    return 0;
  }
  if (raw > adc_max) {
    raw = adc_max;  // out-of-range guard: a railed reading can't exceed full scale
  }
  const float mv = (static_cast<float>(raw) * ref_mv) / static_cast<float>(adc_max);
  return roundSaturateU16(mv);
}

uint16_t packMilliVolts(uint16_t raw, uint16_t adc_max, float ref_mv,
                        float divider_ratio) {
  if (adc_max == 0) {
    return 0;
  }
  if (raw > adc_max) {
    raw = adc_max;
  }
  const float pin_mv =
      (static_cast<float>(raw) * ref_mv) / static_cast<float>(adc_max);
  return roundSaturateU16(pin_mv * divider_ratio);
}

}  // namespace battery
