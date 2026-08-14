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

// --- Serial routing (preprocessor-visible for the task layer) ---------------
// DYNAMIXEL TTL bus is Serial1 (SERCOM2) with hardware auto-direction, so no
// direction pin is needed (kDxlDirPin == -1 below).
#define HEXAPOD_DXL_SERIAL Serial1
// ExpressLRS CRSF receiver on the expansion UART header (Serial3, D14 TX /
// D13 RX). Only present on the real OpenRB-150 variant; the mkrzero fallback
// has no Serial3, in which case the CRSF link is compiled out.
#if defined(PIN_SERIAL3_RX) && defined(PIN_SERIAL3_TX)
#define HEXAPOD_CRSF_SERIAL Serial3
#define HEXAPOD_HAS_CRSF_UART 1
#else
#define HEXAPOD_HAS_CRSF_UART 0
#endif

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

// --- DYNAMIXEL bus direction control ----------------------------------------
// The OpenRB-150 DXL transceiver handles direction automatically, so
// Dynamixel2Arduino is constructed without a direction pin.
constexpr int16_t kDxlDirPin = -1;

}  // namespace pins
}  // namespace board
