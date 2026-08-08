#if defined(HEXAPOD_CRASH_RECOVERY_BOOT)

#include <Arduino.h>

#include "board/board.h"
#include "safety/fault_capture.h"
#include "safety/watchdog.h"

namespace {

void printHex32(uint32_t value) {
  constexpr char kHex[] = "0123456789ABCDEF";
  Serial.print("0x");
  for (int8_t shift = 28; shift >= 0; shift -= 4) {
    Serial.print(kHex[(value >> shift) & 0x0Fu]);
  }
}

void printSnapshot() {
  const fault_capture::Snapshot& snapshot = fault_capture::lastSnapshot();
  Serial.println("HEXAPOD RETAINED FAULT RECOVERY");
  Serial.print("reason=");
  Serial.print(static_cast<uint8_t>(snapshot.reason));
  Serial.print(" stage=");
  Serial.print(static_cast<uint8_t>(snapshot.stage));
  Serial.print(" task=");
  Serial.println(snapshot.task_name[0] == '\0' ? "(none)" : snapshot.task_name);
  Serial.print("sp=");
  printHex32(snapshot.stack_pointer);
  Serial.print(" exc_return=");
  printHex32(snapshot.exception_return);
  Serial.println();
  Serial.print("pc=");
  printHex32(snapshot.pc);
  Serial.print(" lr=");
  printHex32(snapshot.lr);
  Serial.print(" xpsr=");
  printHex32(snapshot.xpsr);
  Serial.println();
  Serial.println("DXL power forced OFF; normal application not started.");
  Serial.print("wdt_raw_valid=");
  Serial.print(watchdog::retainedDiagnosticsValidRaw() ? 1 : 0);
  Serial.print(" missed=0x");
  Serial.print(watchdog::retainedMissedMaskRaw(), HEX);
  Serial.print(" dxl_progress=");
  Serial.print(watchdog::retainedProgressMarkerRaw());
  Serial.print(" control_progress=");
  Serial.print(watchdog::retainedControlProgressRaw());
  Serial.print(" safety_state=");
  Serial.println(watchdog::retainedSafetyStateRaw());
}

}  // namespace

void setup() {
  fault_capture::init();
  board::init();
  Serial.begin(115200);
}

void loop() {
  printSnapshot();
  delay(1000);
}

#endif