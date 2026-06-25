#pragma once

// ===========================================================================
// Cooperative software watchdog (STUB).
//
// Each RTOS task calls checkIn() on every iteration. The health task calls
// evaluate() periodically to detect any task that has stopped running (never
// checked in during the window) and records a missed-task bitmask.
//
// This is a software liveness check only. Wiring the SAMD21 hardware WDT
// (Early Warning + reset) is deferred to the safety/fault work in Phase 1/2;
// see the TODO in watchdog.cpp. Kept allocation-free and ISR-safe: each task
// writes only its own counter slot (single producer), health reads.
// ===========================================================================

#include <stdint.h>

namespace watchdog {

enum class TaskId : uint8_t {
  Control = 0,
  Dxl,
  Rc,
  Api,
  I2c,
  Health,
  Count,
};

constexpr uint8_t kTaskCount = static_cast<uint8_t>(TaskId::Count);

// Reset counters/state. Call once before the scheduler starts.
void init();

// Task heartbeat. Each task calls this every iteration. Single producer per id.
void checkIn(TaskId id);

// Snapshot heartbeats since the previous call. Updates the missed-task bitmask
// for any task that did not check in during the window. (HW WDT pet is a stub.)
void evaluate();

// Bitmask of tasks that missed their heartbeat at the last evaluate().
// Bit position == TaskId value. 0 means all tasks are live.
uint32_t missedMask();

}  // namespace watchdog
