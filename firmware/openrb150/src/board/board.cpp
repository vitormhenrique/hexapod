#include "board.h"

#include "battery_calc.h"
#include "openrb150_pins.h"

namespace board {
namespace {

bool g_userLedOn = false;
bool g_dxlPowerOn = false;
uint32_t g_dxlPowerTransitions = 0;

}  // namespace

void init() {
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
  analogReadResolution(kAdcResolutionBits);
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

uint16_t readBatteryRaw() {
  return static_cast<uint16_t>(analogRead(pins::kBatteryAdc));
}

uint16_t readBatteryPinMilliVolts() {
  return battery::pinMilliVolts(readBatteryRaw(), kAdcMaxCount,
                                kBatteryReferenceMv);
}

uint16_t readBatteryMilliVolts() {
  return battery::packMilliVolts(readBatteryRaw(), kAdcMaxCount,
                                 kBatteryReferenceMv, kBatteryDividerRatio);
}

}  // namespace board
