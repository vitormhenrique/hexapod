#pragma once

// ===========================================================================
// OpenRB-150 centralized pin map.
//
// Single source of truth for board pins/constants used by the firmware. Values
// come from the OpenRB-150 variant (see ../../variants/OpenRB-150/variant.h and
// the e-Manual pin definitions). Build with [env:openrb150] for the correct
// hardware mapping; see ../../doc/mkrzero-vs-openrb150.md.
//
// Only board.* should consume the raw pin numbers below. The rest of the
// firmware should call the board HAL, not poke these pins directly.
// ===========================================================================

#include <Arduino.h>

namespace board {
namespace pins {

// --- User-controllable status LED (USER, orange) ---------------------------
// PIN_LED / LED_BUILTIN == 32 on both the OpenRB-150 variant and the mkrzero
// fallback, so this is always valid.
#if defined(PIN_LED)
constexpr uint8_t kUserLed = PIN_LED;
#elif defined(LED_BUILTIN)
constexpr uint8_t kUserLed = LED_BUILTIN;
#else
constexpr uint8_t kUserLed = 32;
#endif

// --- Battery monitor ADC ---------------------------------------------------
// ADC_BATTERY == 33 (PB09). Defined on both variants.
#if defined(ADC_BATTERY)
constexpr uint8_t kBatteryAdc = ADC_BATTERY;
#else
constexpr uint8_t kBatteryAdc = 33;
#endif

// --- DYNAMIXEL power FET enable --------------------------------------------
// BDPIN_DXL_PWR_EN == 31. OpenRB-150 ONLY: this gates 12 V power to the four
// DYNAMIXEL ports and lights the red DXL LED when high. Not present on the
// mkrzero fallback variant, so guard all use behind kHasDxlPowerControl.
#if defined(BDPIN_DXL_PWR_EN)
constexpr uint8_t kDxlPowerEnable = BDPIN_DXL_PWR_EN;
constexpr bool kHasDxlPowerControl = true;
#else
constexpr uint8_t kDxlPowerEnable = 31;  // nominal; inert without the FET
constexpr bool kHasDxlPowerControl = false;
#endif

}  // namespace pins
}  // namespace board
