#pragma once

// ===========================================================================
// MicroMod Teensy (IMXRT1062) pin map for the MicroMod Carrier V1.1.
//
// Single source of truth for board pins/constants used by the firmware on the
// Teensy MicroMod target. Values come from the carrier hardware contract in
// board.md (repo root): the "Arduino Pin Map by Processor Board" table maps
// each carrier signal to a Teensy Arduino pin number.
//
// Only board.* should consume the raw GPIO pin numbers below. The serial-port
// routing macros are consumed by the task layer (which owns the UARTs).
// ===========================================================================

#include <Arduino.h>

// --- Serial routing (preprocessor-visible for the task layer) ---------------
// DYNAMIXEL TTL bus: carrier UART2 (MicroMod contacts 20/22) -> Teensy Serial2
// (pins 16 RX / 17 TX) through the half-duplex transceiver. Direction is
// controlled by DXL_TXEN (see kDxlDirPin below).
#define HEXAPOD_DXL_SERIAL Serial2
// ExpressLRS CRSF receiver: carrier J8 UART1 -> Teensy Serial1 (pins 0 RX /
// 1 TX), 3.3 V logic.
#define HEXAPOD_CRSF_SERIAL Serial1
#define HEXAPOD_HAS_CRSF_UART 1

namespace board {
namespace pins {

// --- User-controllable status LED -------------------------------------------
// Teensy LED_BUILTIN is pin 13, which shares the EYESPI SPI clock. This
// firmware does not drive the EYESPI display, so the LED is safe to use.
constexpr uint8_t kUserLed = 13;

// --- Battery monitor ADC -----------------------------------------------------
// Carrier BAT_SENSE (MicroMod A0, contact 34) -> Teensy pin 14 / A0.
constexpr uint8_t kBatteryAdc = 14;

// --- DYNAMIXEL power switch --------------------------------------------------
// Carrier G3 (contact 46) -> Teensy pin 43 drives the DYNAMIXEL high-side
// power switch. High enables switched servo power; low disables it.
constexpr uint8_t kDxlPowerEnable = 43;
constexpr bool kHasDxlPowerControl = true;

// --- DYNAMIXEL bus direction control ------------------------------------------
// Carrier G2 (contact 44) -> Teensy pin 42 drives DXL_TXEN. High: transmit.
// Low: receive / default-safe state. Passed to Dynamixel2Arduino as dir_pin.
constexpr int16_t kDxlDirPin = 42;

}  // namespace pins
}  // namespace board
