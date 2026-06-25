#include <Arduino.h>

// ---------------------------------------------------------------------------
// OpenRB-150 / SAMD21 "blink + heartbeat" upload smoke test.
//
// Build/flash with the default env: `pio run -e openrb150 -t upload`. That env
// uses the bundled custom board (boards/openrb150.json) + variant
// (variants/OpenRB-150) so Serial1 = DXL bus, Serial2/3 exist, and
// BDPIN_DXL_PWR_EN is the real power FET. See doc/mkrzero-vs-openrb150.md.
//
// This sketch only blinks the USER LED + prints to USB CDC so it is a clean
// toolchain/upload check that works on BOTH envs (openrb150 and the mkrzero
// fallback). LED_BUILTIN (pin 32) is the OpenRB-150 USER (orange) LED.
// ---------------------------------------------------------------------------

#ifndef LED_BUILTIN
#define LED_BUILTIN 32  // OpenRB-150 USER LED (orange) == MKR Zero pin 32
#endif

static const uint32_t kBlinkIntervalMs = 500;

static uint32_t g_lastToggleMs = 0;
static bool g_ledOn = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // USB CDC (Serial) on both OpenRB-150 and mkrzero. Non-blocking: do not wait
  // for the host so the board still runs when nothing is connected.
  Serial.begin(115200);
}

void loop() {
  const uint32_t now = millis();
  if (now - g_lastToggleMs >= kBlinkIntervalMs) {
    g_lastToggleMs = now;
    g_ledOn = !g_ledOn;
    digitalWrite(LED_BUILTIN, g_ledOn ? HIGH : LOW);

    if (Serial) {
      Serial.print("blink ");
      Serial.println(g_ledOn ? "ON" : "OFF");
    }
  }
}