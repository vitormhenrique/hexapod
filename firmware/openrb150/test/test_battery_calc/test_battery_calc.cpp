// Native (host) unit tests for the portable battery voltage conversion math
// (board/battery_calc.cpp). This feeds the low-voltage E-stop threshold, so the
// rounding, divider-ratio application, and out-of-range saturation are checked
// here on the host. Run with: pio test -e native -f test_battery_calc

#include <unity.h>

#include "../../src/board/battery_calc.h"

using namespace battery;

namespace {
constexpr uint16_t kAdcMax = 4095;     // 12-bit
constexpr float kRefMv = 3300.0f;      // nominal 3.3 V rail
constexpr float kRatio = 4.0f;         // OpenRB-150 default divider
}  // namespace

void setUp() {}
void tearDown() {}

// Full scale -> full reference at the pin.
void test_pin_full_scale_is_reference() {
  TEST_ASSERT_EQUAL_UINT16(3300, pinMilliVolts(kAdcMax, kAdcMax, kRefMv));
}

// Zero counts -> zero volts.
void test_pin_zero_is_zero() {
  TEST_ASSERT_EQUAL_UINT16(0, pinMilliVolts(0, kAdcMax, kRefMv));
}

// Half scale rounds to half reference (2047/4095 * 3300 = 1649.6 -> 1650).
void test_pin_half_scale_rounds() {
  TEST_ASSERT_EQUAL_UINT16(1650, pinMilliVolts(2047, kAdcMax, kRefMv));
}

// Pack voltage is the pin voltage scaled by the divider ratio: full charge.
void test_pack_full_scale_applies_ratio() {
  // 3300 mV at the pin * 4.0 = 13200 mV pack.
  TEST_ASSERT_EQUAL_UINT16(13200, packMilliVolts(kAdcMax, kAdcMax, kRefMv, kRatio));
}

// A representative 3S pack: ~11.1 V nominal -> pin ~2775 mV -> raw ~3443.
// raw 3443: pin = 3443/4095*3300 = 2774.9 -> 2775; pack = 2775*4 = 11100.
void test_pack_nominal_3s() {
  const uint16_t pin = pinMilliVolts(3443, kAdcMax, kRefMv);
  TEST_ASSERT_UINT16_WITHIN(2, 2775, pin);
  const uint16_t pack = packMilliVolts(3443, kAdcMax, kRefMv, kRatio);
  TEST_ASSERT_UINT16_WITHIN(8, 11100, pack);
}

// Out-of-range raw (> full scale) is clamped, never wraps.
void test_raw_above_full_scale_clamps() {
  TEST_ASSERT_EQUAL_UINT16(3300, pinMilliVolts(60000, kAdcMax, kRefMv));
  TEST_ASSERT_EQUAL_UINT16(13200,
                           packMilliVolts(60000, kAdcMax, kRefMv, kRatio));
}

// Degenerate adc_max guard.
void test_zero_adc_max_is_safe() {
  TEST_ASSERT_EQUAL_UINT16(0, pinMilliVolts(100, 0, kRefMv));
  TEST_ASSERT_EQUAL_UINT16(0, packMilliVolts(100, 0, kRefMv, kRatio));
}

// A calibrated (non-default) ratio + reference is honoured, so a HIL trim flows
// straight through the conversion.
void test_calibrated_ratio_and_reference() {
  // Measured rail 3287 mV, measured divider 3.92.
  const uint16_t pack = packMilliVolts(kAdcMax, kAdcMax, 3287.0f, 3.92f);
  // 3287 * 3.92 = 12885.04 -> 12885.
  TEST_ASSERT_EQUAL_UINT16(12885, pack);
}

// Saturation: an absurd ratio cannot overflow the uint16 return.
void test_pack_saturates_to_u16_max() {
  TEST_ASSERT_EQUAL_UINT16(65535, packMilliVolts(kAdcMax, kAdcMax, kRefMv, 50.0f));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_pin_full_scale_is_reference);
  RUN_TEST(test_pin_zero_is_zero);
  RUN_TEST(test_pin_half_scale_rounds);
  RUN_TEST(test_pack_full_scale_applies_ratio);
  RUN_TEST(test_pack_nominal_3s);
  RUN_TEST(test_raw_above_full_scale_clamps);
  RUN_TEST(test_zero_adc_max_is_safe);
  RUN_TEST(test_calibrated_ratio_and_reference);
  RUN_TEST(test_pack_saturates_to_u16_max);
  return UNITY_END();
}
