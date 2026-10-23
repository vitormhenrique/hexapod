#include "board.h"

#if defined(ARDUINO_ARCH_SAMD)
#include <wiring_private.h>
#endif

#include "battery_calc.h"
#include "../hil/output_guard.h"
#include "board_pins.h"

namespace board {
namespace {

bool g_userLedOn = false;
bool g_dxlPowerOn = false;
uint32_t g_dxlPowerTransitions = 0;

#if defined(ARDUINO_ARCH_SAMD)
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
#endif  // ARDUINO_ARCH_SAMD

#if defined(ARDUINO_TEENSY_MICROMOD)
// --- Onboard WS2812 status pixel (carrier PWM1 -> pin 2) --------------------
// Single-pixel bit-banged encoder using the DWT cycle counter (enabled by the
// Teensy 4 core at startup). One 24-bit GRB frame takes ~30 us with
// interrupts masked, which is well inside every UART FIFO budget. The encoder
// only runs when the color actually changes, and enforces the >=300 us latch
// gap between frames.
constexpr uint32_t kWs2812LatchUs = 300;
// Global brightness (0..255) applied before transmission: the pixel is an
// indicator, not a headlight.
constexpr uint8_t kRgbBrightness = 48;

uint32_t g_lastGrb = 0;
bool g_grbValid = false;
uint32_t g_lastShowUs = 0;

void ws2812Show(uint32_t grb) {
  // Respect the reset/latch time so back-to-back updates cannot merge frames.
  const uint32_t since_us = micros() - g_lastShowUs;
  if (g_grbValid && since_us < kWs2812LatchUs) {
    delayMicroseconds(kWs2812LatchUs - since_us);
  }

  const uint32_t cyc_period = F_CPU_ACTUAL / 800000u;   // 1.25 us bit slot
  const uint32_t cyc_t0h = F_CPU_ACTUAL / 2500000u;     // 0.40 us high = 0
  const uint32_t cyc_t1h = F_CPU_ACTUAL / 1250000u;     // 0.80 us high = 1

  noInterrupts();
  uint32_t start = ARM_DWT_CYCCNT;
  for (uint32_t mask = 1u << 23; mask != 0; mask >>= 1) {
    while (ARM_DWT_CYCCNT - start < cyc_period) {
    }
    start = ARM_DWT_CYCCNT;
    digitalWriteFast(pins::kNeoPixel, HIGH);
    const uint32_t high_cycles = (grb & mask) ? cyc_t1h : cyc_t0h;
    while (ARM_DWT_CYCCNT - start < high_cycles) {
    }
    digitalWriteFast(pins::kNeoPixel, LOW);
  }
  interrupts();

  g_lastShowUs = micros();
  g_lastGrb = grb;
  g_grbValid = true;
}
#endif  // ARDUINO_TEENSY_MICROMOD

}  // namespace

void init() {
  hil::outputGuard().reset();

  // --- DYNAMIXEL direction and power: force both LOW before UART setup. -----
  // The MicroMod carrier uses an explicit half-duplex direction GPIO. Preload
  // receive before enabling the output so reset cannot briefly drive the bus.
  if (pins::kDxlDirPin >= 0) {
    digitalWrite(static_cast<uint8_t>(pins::kDxlDirPin), LOW);
    pinMode(static_cast<uint8_t>(pins::kDxlDirPin), OUTPUT);
  }
  // Safety invariant (AGENTS.md 1.1): servos stay unpowered at boot.
  if (pins::kHasDxlPowerControl) {
    digitalWrite(pins::kDxlPowerEnable, LOW);
    pinMode(pins::kDxlPowerEnable, OUTPUT);
#if defined(HEXAPOD_DXL_POWER_LATCHED_ON)
    digitalWrite(pins::kDxlPowerEnable, HIGH);
#endif
  }
#if defined(HEXAPOD_DXL_POWER_LATCHED_ON)
  g_dxlPowerOn = pins::kHasDxlPowerControl;
  g_dxlPowerTransitions = g_dxlPowerOn ? 1 : 0;
#else
  g_dxlPowerOn = false;
  g_dxlPowerTransitions = 0;
#endif

  // --- USER LED: output, off. ---------------------------------------------
  pinMode(pins::kUserLed, OUTPUT);
  digitalWrite(pins::kUserLed, LOW);
  g_userLedOn = false;

#if defined(ARDUINO_TEENSY_MICROMOD)
  // --- RGB status pixel: output low, then latch an explicit black frame so
  // whatever color survived a warm reset is cleared before tasks start.
  pinMode(pins::kNeoPixel, OUTPUT);
  digitalWrite(pins::kNeoPixel, LOW);
  g_grbValid = false;
  ws2812Show(0);
#endif

  // --- Battery ADC. -------------------------------------------------------
#if defined(ARDUINO_ARCH_SAMD)
  // Conversion setup happens in the bounded reader below. Do not call the
  // Arduino analogReadResolution() helper here: it has an unbounded ADC sync
  // wait and could block the USB API before the scheduler starts.
  pinMode(pins::kBatteryAdc, INPUT);
#else
  // Teensy 4.x: analogRead() is bounded (the hardware ADC always completes).
  // INPUT_DISABLE turns off the digital keeper so the divider reading is not
  // distorted; the resolution matches kAdcResolutionBits (12 -> 0..4095).
  pinMode(pins::kBatteryAdc, INPUT_DISABLE);
  analogReadResolution(kAdcResolutionBits);
#endif
}

void setUserLed(bool on) {
  g_userLedOn = on;
  digitalWrite(pins::kUserLed, on ? HIGH : LOW);
}

void toggleUserLed() { setUserLed(!g_userLedOn); }

bool userLedOn() { return g_userLedOn; }

uint8_t pinUserLed() { return pins::kUserLed; }

#if defined(ARDUINO_TEENSY_MICROMOD)
bool hasRgbStatusLed() { return true; }

void setRgbStatusLed(uint8_t r, uint8_t g, uint8_t b) {
  const uint32_t scale = kRgbBrightness;
  const uint32_t grb = ((g * scale / 255u) << 16) |
                       ((r * scale / 255u) << 8) |
                       (b * scale / 255u);
  if (g_grbValid && grb == g_lastGrb) return;  // no bus traffic when static
  ws2812Show(grb);
}
#else
bool hasRgbStatusLed() { return false; }

void setRgbStatusLed(uint8_t, uint8_t, uint8_t) {}
#endif

bool hasDxlPowerControl() { return pins::kHasDxlPowerControl; }

void setDxlPower(bool on) {
#if defined(HEXAPOD_DXL_POWER_LATCHED_ON)
  if (!on) return;
#endif
  if (on && !hil::outputGuard().allowPowerEnable()) {
    return;
  }
  if (!pins::kHasDxlPowerControl) {
    // No FET on this build/variant: cannot actually gate servo power.
    g_dxlPowerOn = false;
    return;
  }
  if (pins::kDxlDirPin >= 0 && (on != g_dxlPowerOn || !on)) {
    digitalWrite(static_cast<uint8_t>(pins::kDxlDirPin), LOW);
  }
  if (on == g_dxlPowerOn) return;
  digitalWrite(pins::kDxlPowerEnable, on ? HIGH : LOW);
  g_dxlPowerOn = on;
  ++g_dxlPowerTransitions;
}

bool dxlPowerEnabled() { return g_dxlPowerOn; }

[[noreturn]] void systemReset() {
  noInterrupts();
  if (pins::kHasDxlPowerControl) {
    // Direct write: a reset must never leave the 12 V rail on behind a dead
    // controller, whatever the guard / latch state was.
    pinMode(pins::kDxlPowerEnable, OUTPUT);
    digitalWrite(pins::kDxlPowerEnable, LOW);
  }
#if defined(ARDUINO_ARCH_SAMD)
  __DSB();
  SCB->AIRCR = (0x5FAul << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
  __DSB();
#elif defined(__IMXRT1062__)
  asm volatile("dsb");
  SCB_AIRCR = 0x05FA0004;  // VECTKEY | SYSRESETREQ
  asm volatile("dsb");
#endif
  for (;;) {
  }
}

uint32_t dxlPowerTransitions() { return g_dxlPowerTransitions; }

bool readBatteryRaw(uint16_t& raw) {
#if defined(ARDUINO_ARCH_SAMD)
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
#else
  // Teensy 4.x: the core's analogRead() is a bounded synchronous conversion.
  raw = static_cast<uint16_t>(analogRead(pins::kBatteryAdc));
  return true;
#endif
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
