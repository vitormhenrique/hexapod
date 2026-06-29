#pragma once

// ===========================================================================
// OpenRB-150 board HAL.
//
// Centralizes safe boot behavior and low-level board I/O so the rest of the
// firmware never pokes raw pins. Safety-critical invariant: the DYNAMIXEL power
// FET is held OFF at boot and stays off until something explicitly arms it.
//
// Static, allocation-free, ISR-safe for the trivial accessors. See AGENTS.md
// section 1 (safety + engineering rules).
// ===========================================================================

#include <Arduino.h>
#include <stdint.h>

namespace board {

// Board ADC resolution used for battery conversion.
constexpr uint8_t kAdcResolutionBits = 12;
constexpr uint16_t kAdcMaxCount = (1u << kAdcResolutionBits) - 1u;  // 4095

// ADC reference in millivolts, used by the battery conversion. The SAMD21
// analog reference is the 3.3 V analog rail (AR_DEFAULT). Override with
// -DHEXAPOD_BATTERY_REF_MV=<measured_mv> during HIL bring-up after measuring the
// actual 3V3 rail (it is typically a few tens of mV off nominal).
#ifndef HEXAPOD_BATTERY_REF_MV
#define HEXAPOD_BATTERY_REF_MV 3300.0f
#endif
constexpr float kBatteryReferenceMv = HEXAPOD_BATTERY_REF_MV;

// Battery divider ratio (Vbatt / Vpin). The OpenRB-150 accepts up to a 3S LiPo
// (12.6 V full charge, ROBOTIS spec) on a 3.3 V ADC, so the on-board divider is
// ~4x. This default keeps the full-charge pin voltage ~3.15 V (under the 3.3 V
// limit), but the exact ratio depends on resistor tolerance and the real ADC
// reference, so it MUST be trimmed against a measured pack voltage before the
// low-voltage E-stop is trusted (issue 4sa.3 / rbg.10).
//
// HIL calibration (no recompile of this file needed): read the reported pin
// millivolts and a meter reading of the pack, then build with
//   -DHEXAPOD_BATTERY_DIVIDER_RATIO=<Vpack_mv / Vpin_mv>
// (and optionally -DHEXAPOD_BATTERY_REF_MV=<measured rail>).
#ifndef HEXAPOD_BATTERY_DIVIDER_RATIO
#define HEXAPOD_BATTERY_DIVIDER_RATIO 4.0f
#endif
constexpr float kBatteryDividerRatio = HEXAPOD_BATTERY_DIVIDER_RATIO;

// Maximum design input (3S LiPo full charge, 12.6 V). Used only for the safety
// guard below.
constexpr float kBatteryMaxInputMv = 12600.0f;

// SAFETY GUARD: a divider ratio that is too small lets the full-charge pin
// voltage exceed the ADC reference, so the ADC rails at full scale and a LOW
// battery would read the same as a full one -- defeating the low-voltage
// E-stop. Reject such a (mis)calibration at compile time.
static_assert(kBatteryDividerRatio >= kBatteryMaxInputMv / kBatteryReferenceMv,
              "Battery divider ratio too small: full-charge pin voltage would "
              "exceed the ADC reference and clip, hiding a low battery.");

// Initialize board I/O and enforce safe boot defaults:
//   - DYNAMIXEL power FET driven OFF (servos unpowered)
//   - USER LED configured as output, off
//   - Battery ADC configured (resolution set)
// Call once early in setup(), before any DXL/motion code.
void init();

// --- USER (orange) status LED ---------------------------------------------
void setUserLed(bool on);
void toggleUserLed();
bool userLedOn();
// Raw pin number of the USER LED, for low-level consumers (e.g. RTOS error LED).
uint8_t pinUserLed();

// --- DYNAMIXEL power FET ----------------------------------------------------
// On the mkrzero fallback variant there is no FET; these calls are inert and
// dxlPowerEnabled() always reports false. hasDxlPowerControl() lets callers
// detect a build that cannot actually gate servo power.
bool hasDxlPowerControl();
void setDxlPower(bool on);
bool dxlPowerEnabled();

// --- Battery monitor --------------------------------------------------------
// Raw 12-bit ADC counts at the battery sense pin (0..4095).
uint16_t readBatteryRaw();
// Voltage at the ADC pin itself (after the on-board divider), in millivolts.
// This is the value to read against a meter when calibrating the divider ratio.
uint16_t readBatteryPinMilliVolts();
// Estimated pack voltage in millivolts, using kBatteryDividerRatio. Accurate
// only once that ratio (and optionally kBatteryReferenceMv) is HIL-calibrated
// via the build-flag overrides documented above.
uint16_t readBatteryMilliVolts();

}  // namespace board
