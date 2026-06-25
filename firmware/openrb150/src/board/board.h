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

// Board ADC reference and resolution used for battery conversion.
constexpr float kAdcReferenceVolts = 3.3f;
constexpr uint8_t kAdcResolutionBits = 12;
constexpr uint16_t kAdcMaxCount = (1u << kAdcResolutionBits) - 1u;  // 4095

// PROVISIONAL battery divider ratio (Vbatt / Vpin). The OpenRB-150 supports up
// to a 3S LiPo (~12.6 V) on a 3.3 V ADC, so the on-board divider is ~4x. This
// value MUST be calibrated against a known battery voltage during Phase 1 HIL
// bring-up (issue rbg.10) before it is trusted for any safety threshold.
constexpr float kBatteryDividerRatio = 4.0f;

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
uint16_t readBatteryPinMilliVolts();
// Estimated pack voltage in millivolts, using kBatteryDividerRatio.
// PROVISIONAL until the divider ratio is HIL-calibrated.
uint16_t readBatteryMilliVolts();

}  // namespace board
