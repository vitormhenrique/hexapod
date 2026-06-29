#pragma once

// ===========================================================================
// Battery voltage conversion math (portable, no Arduino deps).
//
// Split out from the board HAL so the safety-relevant ADC -> millivolt
// conversion (which feeds the low-voltage E-stop threshold) can be unit-tested
// on the host. The HAL (board.cpp) only does the raw analogRead(); all scaling,
// rounding, divider-ratio application, and saturation live here.
//
// Calibration inputs (ADC reference and divider ratio) are passed in as
// parameters so a value measured during HIL bring-up can be applied without
// touching this logic. board.h supplies the OpenRB-150 defaults (and build-flag
// overrides) it calls these with.
// ===========================================================================

#include <stdint.h>

namespace battery {

// Voltage present at the ADC sense pin (after the on-board divider), in
// millivolts, from a raw N-bit ADC reading.
//   raw     : raw ADC counts (0..adc_max)
//   adc_max : full-scale count (e.g. 4095 for 12-bit)
//   ref_mv  : ADC reference voltage in millivolts (nominally 3300)
// Rounds to the nearest millivolt and saturates raw to adc_max so an out-of-
// range reading can never wrap.
uint16_t pinMilliVolts(uint16_t raw, uint16_t adc_max, float ref_mv);

// Estimated battery pack voltage in millivolts: the pin voltage scaled back up
// by the divider ratio (Vbatt / Vpin). Saturates to the uint16 range.
uint16_t packMilliVolts(uint16_t raw, uint16_t adc_max, float ref_mv,
                        float divider_ratio);

}  // namespace battery
