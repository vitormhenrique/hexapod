#pragma once

// ===========================================================================
// RTOS task configuration: stack depths (in FreeRTOS WORDS, 4 bytes each),
// priorities, and loop periods. Centralized so stack budgeting and timing are
// reviewable in one place.
//
// Priority ordering (higher number = higher priority) follows AGENTS.md 5.1:
//   control / dxl  : high   (real-time motion + servo bus)
//   rc             : medium-high (failsafe-relevant input)
//   api            : medium
//   i2c / health   : low
// ===========================================================================

#include <stdint.h>

namespace app {

// Stack depth in words. configMINIMAL_STACK_SIZE on this port is small; these
// give comfortable headroom that the health task verifies via high-water marks.
namespace stack_words {
constexpr uint16_t kControl = 256;
constexpr uint16_t kDxl = 256;
constexpr uint16_t kRc = 192;
constexpr uint16_t kApi = 256;
constexpr uint16_t kI2c = 192;
constexpr uint16_t kHealth = 256;
constexpr uint16_t kBlink = 96;   // tiny LED test task
}  // namespace stack_words

// FreeRTOS task priorities. tskIDLE_PRIORITY == 0.
namespace priority {
constexpr uint8_t kControl = 3;
constexpr uint8_t kDxl = 3;
constexpr uint8_t kRc = 2;
constexpr uint8_t kApi = 2;
constexpr uint8_t kI2c = 1;
constexpr uint8_t kHealth = 1;
constexpr uint8_t kBlink = 1;
}  // namespace priority

// Nominal loop periods in milliseconds for the skeleton. Real rates are tuned
// as each task is fleshed out in later Phase 1/2 tasks.
namespace period_ms {
constexpr uint32_t kControl = 10;   // 100 Hz
constexpr uint32_t kDxl = 20;       // 50 Hz
constexpr uint32_t kRc = 10;        // 100 Hz
constexpr uint32_t kApi = 5;        // 200 Hz poll
constexpr uint32_t kI2c = 20;       // 50 Hz
constexpr uint32_t kHealth = 500;   // 2 Hz reporting + watchdog evaluate
constexpr uint32_t kBlink = 100;    // 5 Hz LED toggle (FreeRTOS liveness test)
}  // namespace period_ms

}  // namespace app
