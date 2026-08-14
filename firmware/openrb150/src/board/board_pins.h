#pragma once

// ===========================================================================
// Board pin-map dispatcher.
//
// Selects the centralized pin/constant header for the build target:
//   - SparkFun MicroMod Teensy on the MicroMod Carrier V1.1 (board.md)
//   - ROBOTIS OpenRB-150 (default / legacy target)
//
// Include this header instead of a board-specific pins header.
// ===========================================================================

#if defined(ARDUINO_TEENSY_MICROMOD)
#include "micromod_teensy_pins.h"
#else
#include "openrb150_pins.h"
#endif
