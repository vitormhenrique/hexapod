#include "board.h"

#include <wiring_private.h>

#include "battery_calc.h"
#include "../hil/output_guard.h"
#include "openrb150_pins.h"

namespace board {
namespace {

bool g_userLedOn = false;
bool g_dxlPowerOn = false;
uint32_t g_dxlPowerTransitions = 0;

// The Arduino SAMD core's analogRead() waits indefinitely for ADC register
// synchronization and conversion completion. A stalled ADC must not prevent
// the control task from yielding to the USB API, so every wait is bounded.
constexpr uint32_t kAdcWaitIterations = 200000u;

bool waitForAdcSync() {
  for (uint32_t iteration = 0; iteration < kAdcWaitIterations; ++iteration) {
    if (!ADC->STATUS.bit.SYNCBUSY) return true;
  }
  return false;
}

bool waitForAdcResult() {
  for (uint32_t iteration = 0; iteration < kAdcWaitIterations; ++iteration) {
    if (ADC->INTFLAG.bit.RESRDY) return true;
  }
  return false;
}

void disableAdc() {
  ADC->CTRLA.bit.ENABLE = 0;
  (void)waitForAdcSync();
}

}  // namespace

void init() {
  hil::outputGuard().reset();

  // --- DYNAMIXEL power: force OFF first, before anything else. -------------
  // Safety invariant (AGENTS.md 1.1): servos stay unpowered at boot.
  if (pins::kHasDxlPowerControl) {
    pinMode(pins::kDxlPowerEnable, OUTPUT);
    digitalWrite(pins::kDxlPowerEnable, LOW);
  }
  g_dxlPowerOn = false;
  g_dxlPowerTransitions = 0;

  // --- USER LED: output, off. ---------------------------------------------
  pinMode(pins::kUserLed, OUTPUT);
  digitalWrite(pins::kUserLed, LOW);
  g_userLedOn = false;

  // --- Battery ADC. -------------------------------------------------------
  // Conversion setup happens in the bounded reader below. Do not call the
  // Arduino analogReadResolution() helper here: it has an unbounded ADC sync
  // wait and could block the USB API before the scheduler starts.
  pinMode(pins::kBatteryAdc, INPUT);
}

void setUserLed(bool on) {
  g_userLedOn = on;
  digitalWrite(pins::kUserLed, on ? HIGH : LOW);
}

void toggleUserLed() { setUserLed(!g_userLedOn); }

bool userLedOn() { return g_userLedOn; }

uint8_t pinUserLed() { return pins::kUserLed; }

bool hasDxlPowerControl() { return pins::kHasDxlPowerControl; }

void setDxlPower(bool on) {
  if (on && !hil::outputGuard().allowPowerEnable()) {
    return;
  }
  if (!pins::kHasDxlPowerControl) {
    // No FET on this build/variant: cannot actually gate servo power.
    g_dxlPowerOn = false;
    return;
  }
  if (on == g_dxlPowerOn) return;
  digitalWrite(pins::kDxlPowerEnable, on ? HIGH : LOW);
  g_dxlPowerOn = on;
  ++g_dxlPowerTransitions;
}

bool dxlPowerEnabled() { return g_dxlPowerOn; }

uint32_t dxlPowerTransitions() { return g_dxlPowerTransitions; }

bool readBatteryRaw(uint16_t& raw) {
  raw = 0;
  const PinDescription& pin = g_APinDescription[pins::kBatteryAdc];
  pinPeripheral(pins::kBatteryAdc, PIO_ANALOG);

  if (!waitForAdcSync()) return false;
  ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
  if (!waitForAdcSync()) return false;
  ADC->INPUTCTRL.bit.MUXPOS = pin.ulADCChannelNumber;
  if (!waitForAdcSync()) return false;

  ADC->CTRLA.bit.ENABLE = 1;
  if (!waitForAdcSync()) {
    disableAdc();
    return false;
  }

  // Discard the first sample after enabling the ADC, matching the SAMD core.
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  ADC->SWTRIG.bit.START = 1;
  if (!waitForAdcResult()) {
    disableAdc();
    return false;
  }
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  if (!waitForAdcSync()) {
    disableAdc();
    return false;
  }
  ADC->SWTRIG.bit.START = 1;
  if (!waitForAdcResult()) {
    disableAdc();
    return false;
  }
  raw = static_cast<uint16_t>(ADC->RESULT.reg);
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  disableAdc();
  return true;
}

bool readBatteryPinMilliVolts(uint16_t& millivolts) {
  uint16_t raw = 0;
  if (!readBatteryRaw(raw)) {
    millivolts = 0;
    return false;
  }
  millivolts = battery::pinMilliVolts(raw, kAdcMaxCount, kBatteryReferenceMv);
  return true;
}

bool readBatteryMilliVolts(uint16_t& millivolts) {
  uint16_t raw = 0;
  if (!readBatteryRaw(raw)) {
    millivolts = 0;
    return false;
  }
  millivolts = battery::packMilliVolts(raw, kAdcMaxCount,
                                       kBatteryReferenceMv,
                                       kBatteryDividerRatio);
  return true;
}

}  // namespace board
