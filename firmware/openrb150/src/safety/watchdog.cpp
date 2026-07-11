#include "watchdog.h"

#if defined(ARDUINO_ARCH_SAMD)
#include <Arduino.h>
#endif

namespace watchdog {
namespace {

// Per-task heartbeat counter, incremented by each task (single producer).
volatile uint32_t g_beats[kTaskCount] = {0};
// Snapshot of g_beats at the previous evaluate(), owned by the health task.
uint32_t g_lastSeen[kTaskCount] = {0};
// Bitmask of tasks that missed their heartbeat at the last evaluate().
volatile uint32_t g_missedMask = 0;
uint32_t g_lastResetMissedMask = 0;
uint8_t g_lastResetProgressMarker = 0;
uint8_t g_lastResetControlProgress = 0;
uint8_t g_lastResetSafetyState = 0;

#if defined(ARDUINO_ARCH_SAMD)
struct RetainedWatchdogState {
  uint32_t magic;
  uint32_t missed_mask;
  uint32_t missed_mask_inverse;
  uint32_t progress_marker;
  uint32_t progress_marker_inverse;
  uint32_t control_progress;
  uint32_t safety_state;
};

constexpr uint32_t kRetainedMagic = 0x57445431u;  // "WDT1"
__attribute__((section(".noinit"))) volatile RetainedWatchdogState g_retainedState;
#endif

// Motion-critical tasks: a sustained stall of either one is a hard safety
// failure, so the hardware WDT pet is withheld and the MCU resets. The other
// tasks (Rc/Api/I2c) are handled by the software fault path (missedMask -> FSM)
// without forcing a reset, so a momentarily-busy host link or EEPROM commit
// cannot reboot a robot whose motion loop is healthy.
constexpr uint32_t kCriticalMask =
    (1u << static_cast<uint8_t>(TaskId::Control)) |
    (1u << static_cast<uint8_t>(TaskId::Dxl));

// Armed lazily on the first evaluate() so boot work cannot trip the WDT before
// the scheduler and health task are confirmed running. Owned by the health
// task (single writer), so no synchronisation is required.
bool g_hwArmed = false;

#if defined(ARDUINO_ARCH_SAMD)
// SAMD21 hardware watchdog. Clocked from the always-on ultra-low-power 32.768
// kHz oscillator (OSCULP32K) via GCLK generator 2 divided to ~1.024 kHz, so the
// WDT keeps running even if the main clock/PLL fails. Normal-mode timeout is
// ~2.0 s (2048 cycles); the health task pets every 500 ms, giving a 4x margin
// against scheduling jitter while still resetting within ~2 s of a true hang.
void hwEnable() {
  // GCLK2 = OSCULP32K / 32 = 1.024 kHz (DIVSEL=1 -> divide by 2^(DIV+1)).
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  while (GCLK->STATUS.bit.SYNCBUSY) {
  }
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_DIVSEL | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  }
  // Route GCLK2 to the WDT peripheral clock.
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  }

  // Disable before (re)configuring, then set the timeout period and clear.
  WDT->CTRL.reg = 0;
  while (WDT->STATUS.bit.SYNCBUSY) {
  }
  WDT->INTENCLR.reg = WDT_INTENCLR_EW;  // no early-warning interrupt; reset only
  WDT->CONFIG.bit.PER = 0x8;            // 2048 cycles @ 1.024 kHz ~= 2.0 s
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY) {
  }
  WDT->CTRL.reg = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY) {
  }
}

void hwPet() {
  // Only issue the clear key when the peripheral is not mid-sync, otherwise the
  // write is discarded. A missed pet simply shortens this window; the next
  // evaluate() pets again well within the timeout.
  if (!WDT->STATUS.bit.SYNCBUSY) {
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  }
}
#else
// Host build: hardware steps compile to no-ops so the liveness logic stays
// testable on the native target.
void hwEnable() {}
void hwPet() {}
#endif

}  // namespace

void init(uint8_t reset_cause) {
#if defined(ARDUINO_ARCH_SAMD)
  const bool retained_valid =
      g_retainedState.magic == kRetainedMagic &&
      g_retainedState.missed_mask_inverse == ~g_retainedState.missed_mask &&
      g_retainedState.progress_marker_inverse ==
        ~g_retainedState.progress_marker;
  g_lastResetMissedMask =
      ((reset_cause & 0x20u) != 0 && retained_valid)
          ? g_retainedState.missed_mask
          : 0;
  g_lastResetProgressMarker =
      ((reset_cause & 0x20u) != 0 && retained_valid)
          ? static_cast<uint8_t>(g_retainedState.progress_marker)
          : 0;
        g_lastResetControlProgress =
          ((reset_cause & 0x20u) != 0 && retained_valid)
            ? static_cast<uint8_t>(g_retainedState.control_progress)
            : 0;
        g_lastResetSafetyState =
          ((reset_cause & 0x20u) != 0 && retained_valid)
            ? static_cast<uint8_t>(g_retainedState.safety_state)
            : 0;
  g_retainedState.magic = kRetainedMagic;
  g_retainedState.missed_mask = 0;
  g_retainedState.missed_mask_inverse = ~0u;
  g_retainedState.progress_marker = 0;
  g_retainedState.progress_marker_inverse = ~0u;
  g_retainedState.control_progress = 0;
  g_retainedState.safety_state = 0;
#else
  (void)reset_cause;
  g_lastResetMissedMask = 0;
  g_lastResetProgressMarker = 0;
  g_lastResetControlProgress = 0;
  g_lastResetSafetyState = 0;
#endif
  for (uint8_t i = 0; i < kTaskCount; ++i) {
    g_beats[i] = 0;
    g_lastSeen[i] = 0;
  }
  g_missedMask = 0;
  g_hwArmed = false;
}

void checkIn(TaskId id) {
  const uint8_t i = static_cast<uint8_t>(id);
  if (i < kTaskCount) {
    g_beats[i]++;
  }
}

void evaluate() {
  // Arm the hardware WDT on the first pass: by now the scheduler is running and
  // the health task has executed at least once, so no long boot operation can
  // trip the timer. Establish the baseline (pet) on the same pass.
  if (!g_hwArmed) {
    hwEnable();
    g_hwArmed = true;
  }

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
#if defined(ARDUINO_ARCH_SAMD)
  g_retainedState.missed_mask = missed;
  g_retainedState.missed_mask_inverse = ~missed;
#endif

  // Drive the hardware WDT: pet while the motion-critical tasks are live, and
  // withhold the pet when one has stalled so the WDT times out and resets the
  // MCU into a safe (de-energised) state. A total health-task hang also stops
  // the pet (evaluate() simply stops being called), giving the same backstop.
  if ((missed & kCriticalMask) == 0) {
    hwPet();
  }
}

uint32_t missedMask() { return g_missedMask; }

uint32_t lastResetMissedMask() { return g_lastResetMissedMask; }

void markProgress(uint8_t marker) {
#if defined(ARDUINO_ARCH_SAMD)
  g_retainedState.progress_marker = marker;
  g_retainedState.progress_marker_inverse = ~static_cast<uint32_t>(marker);
#else
  (void)marker;
#endif
}

uint8_t lastResetProgressMarker() { return g_lastResetProgressMarker; }

void markControlProgress(uint8_t marker) {
#if defined(ARDUINO_ARCH_SAMD)
  g_retainedState.control_progress = marker;
#else
  (void)marker;
#endif
}

void markSafetyState(uint8_t state) {
#if defined(ARDUINO_ARCH_SAMD)
  g_retainedState.safety_state = state;
#else
  (void)state;
#endif
}

uint8_t lastResetControlProgress() { return g_lastResetControlProgress; }

uint8_t lastResetSafetyState() { return g_lastResetSafetyState; }

bool criticalStalled() { return (g_missedMask & kCriticalMask) != 0; }


}  // namespace watchdog
