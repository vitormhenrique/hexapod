#include "watchdog.h"

namespace watchdog {
namespace {

// Per-task heartbeat counter, incremented by each task (single producer).
volatile uint32_t g_beats[kTaskCount] = {0};
// Snapshot of g_beats at the previous evaluate(), owned by the health task.
uint32_t g_lastSeen[kTaskCount] = {0};
// Bitmask of tasks that missed their heartbeat at the last evaluate().
volatile uint32_t g_missedMask = 0;

}  // namespace

void init() {
  for (uint8_t i = 0; i < kTaskCount; ++i) {
    g_beats[i] = 0;
    g_lastSeen[i] = 0;
  }
  g_missedMask = 0;
}

void checkIn(TaskId id) {
  const uint8_t i = static_cast<uint8_t>(id);
  if (i < kTaskCount) {
    g_beats[i]++;
  }
}

void evaluate() {
  uint32_t missed = 0;
  for (uint8_t i = 0; i < kTaskCount; ++i) {
    const uint32_t now = g_beats[i];
    if (now == g_lastSeen[i]) {
      // No heartbeat since the previous window: task is stalled.
      missed |= (1u << i);
    }
    g_lastSeen[i] = now;
  }
  g_missedMask = missed;

  // TODO(rbg safety): pet the SAMD21 hardware WDT here once it is enabled, and
  // deliberately withhold the pet (or trigger Early Warning) when `missed`
  // indicates a high-priority task has stalled, so the MCU resets into a safe
  // state. Stubbed for the task skeleton.
}

uint32_t missedMask() { return g_missedMask; }

}  // namespace watchdog
