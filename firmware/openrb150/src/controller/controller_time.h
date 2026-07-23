#pragma once

// ===========================================================================
// Adapter-fed monotonic time for ControllerCore (hexapod_src-4ju.4).
//
// The portable controller never reads a hardware, FreeRTOS, ROS, or wall clock.
// An adapter supplies an absolute unsigned millisecond sample and this utility
// derives the elapsed time passed to ControllerCore. Unsigned subtraction keeps
// ordinary 32-bit millisecond wraparound correct.
// ===========================================================================

#include <stdint.h>

#include "controller_contract.h"

namespace controller {

// A controller loop delayed by more than this is not stepped with a giant time
// delta. It is reported invalid so the future core can fail closed and rebase
// its timing. Normal scheduling overruns remain visible as their real elapsed
// time, rather than being silently clamped to the nominal 10 ms period.
constexpr uint32_t kDefaultMaxControllerElapsedMs = 1000;

class ControllerClock {
 public:
  explicit ControllerClock(
      uint32_t nominal_period_ms = 0,
      uint32_t max_elapsed_ms = kDefaultMaxControllerElapsedMs)
      : nominal_period_ms_(nominal_period_ms),
        max_elapsed_ms_(max_elapsed_ms) {}

  void reset();

  // Derive a step time from an adapter-owned monotonic millisecond counter.
  // The first valid sample establishes the origin and has zero elapsed time.
  ControllerTime sampleMilliseconds(uint32_t now_ms);

 private:
  uint32_t nominal_period_ms_;
  uint32_t max_elapsed_ms_;
  uint32_t last_now_ms_ = 0;
  bool initialized_ = false;
};

// Shared conversion for FreeRTOS and native adapters. `tick_period_ms` must
// be nonzero; a zero period yields an invalid time without changing clock
// state. Multiplication is intentionally uint32_t so a tick-counter rollover
// has the same unsigned-wrap semantics as any other millisecond source.
ControllerTime controllerTimeFromTickCount(ControllerClock& clock,
                                           uint32_t tick_count,
                                           uint32_t tick_period_ms);

}  // namespace controller