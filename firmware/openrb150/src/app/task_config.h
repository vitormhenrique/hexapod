#pragma once

// ===========================================================================
// RTOS task configuration: stack depths (in FreeRTOS WORDS, 4 bytes each),
// priorities, and loop periods. Centralized so stack budgeting and timing are
// reviewable in one place.
//
// Priority ordering (higher number = higher priority) follows AGENTS.md 5.1:
//   control / dxl  : high   (real-time motion + servo bus)
//   rc             : high (failsafe-relevant UART drain)
//   api            : medium
//   health         : highest (brief watchdog evaluation only)
//   i2c            : low
// ===========================================================================

#include <stdint.h>

namespace app {

// Stack depth in words. configMINIMAL_STACK_SIZE on this port is small; these
// give comfortable headroom that the health task verifies via high-water marks.
namespace stack_words {
// Armed RC motion runs gait + body IK + six leg IK solves in this task. Live
// high-water tracing showed only 31 words free with a 384-word stack during
// production startup. Leave 95 words of margin at that observed peak for
// deeper commanded-pose and trick paths.
constexpr uint16_t kControl = 448;
// dxl: Dynamixel2Arduino call chains (syncRead/readControlTableItem parsing)
// plus the 160-byte maintenance-job result buffer run on this stack; 256 words
// hard-faulted the MCU on the first status cycle after a real-servo scan
// (HIL hexapod_src-2e8 bring-up). Health-task high-water marks verify actual
// usage.
constexpr uint16_t kDxl = 512;
constexpr uint16_t kRc = 192;
// Request/response buffers are static; static stack analysis bounds the normal
// dispatcher below 576 words. HIL trace fragmentation adds a separate path.
#if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
constexpr uint16_t kApi = 768;
#else
constexpr uint16_t kApi = 576;
#endif
constexpr uint16_t kI2c = 384;   // boot: scanAll() + config load; deep call chain
constexpr uint16_t kHealth = 256;
}  // namespace stack_words

// FreeRTOS task priorities. tskIDLE_PRIORITY == 0.
namespace priority {
constexpr uint8_t kControl = 3;
constexpr uint8_t kDxl = 3;
constexpr uint8_t kRc = 3;
constexpr uint8_t kApi = 2;
constexpr uint8_t kI2c = 1;
constexpr uint8_t kHealth = 4;
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

// Runtime-tunable bounds for the i2c/sensor poll loop period (SENSOR_SET_RATE,
// lmt.9). The host requests a poll rate in Hz; the i2c task derives its loop
// period from it and clamps to this window so a host cannot starve the loop
// (which also services config commits) or spin it pointlessly fast.
constexpr uint32_t kI2cMinMs = 5;    // 200 Hz ceiling
constexpr uint32_t kI2cMaxMs = 100;  // 10 Hz floor
}  // namespace period_ms

}  // namespace app
