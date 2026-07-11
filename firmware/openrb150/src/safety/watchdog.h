#pragma once

// ===========================================================================
// Cooperative software watchdog + SAMD21 hardware WDT backstop.
//
// Each RTOS task calls checkIn() on every iteration. The health task calls
// evaluate() periodically to detect any task that has stopped running (never
// checked in during the window) and records a missed-task bitmask.
//
// evaluate() also drives the SAMD21 hardware watchdog: it arms the WDT on the
// first call (lazily, once the scheduler is confirmed running so boot work
// cannot trip it) and pets it on every subsequent call -- UNLESS a motion-
// critical task (Control or Dxl) has stalled, in which case the pet is withheld
// so the WDT times out and resets the MCU into a safe (de-energised) state.
// A total health-task hang also stops the pet and forces the same reset.
//
// Kept allocation-free and ISR-safe: each task writes only its own counter slot
// (single producer), the health task reads. The hardware register access is
// SAMD21-only (guarded by ARDUINO_ARCH_SAMD) so the portable liveness logic
// stays host-testable.
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
void init(uint8_t reset_cause = 0);

// Task heartbeat. Each task calls this every iteration. Single producer per id.
void checkIn(TaskId id);

// Snapshot heartbeats since the previous call. Updates the missed-task bitmask
// for any task that did not check in during the window, then drives the SAMD21
// hardware WDT: arms it on the first call, pets it while all motion-critical
// tasks are live, and withholds the pet (forcing an MCU reset) when a critical
// task has stalled. On host builds the hardware steps are no-ops.
void evaluate();

// Bitmask of tasks that missed their heartbeat at the last evaluate().
// Bit position == TaskId value. 0 means all tasks are live.
uint32_t missedMask();

// Validated missed-task mask retained from the evaluation that preceded the
// last hardware-watchdog reset. Zero also covers a health-task/CPU stall that
// prevented evaluate() from recording a critical task.
uint32_t lastResetMissedMask();

// Record a coarse high-priority operation phase in retained SRAM. This is
// diagnostic only; zero means no marked operation was in progress.
void markProgress(uint8_t marker);
uint8_t lastResetProgressMarker();
void markControlProgress(uint8_t marker);
void markSafetyState(uint8_t state);
uint8_t lastResetControlProgress();
uint8_t lastResetSafetyState();

// True when a motion-critical task (Control or Dxl) missed its heartbeat at the
// last evaluate() -- the condition under which evaluate() withholds the WDT pet
// so the hardware watchdog resets the MCU.
bool criticalStalled();

}  // namespace watchdog
