#include <Arduino.h>

#include "app/tasks.h"
#include "board/board.h"
#include "safety/fault_capture.h"

// ---------------------------------------------------------------------------
// OpenRB-150 firmware entry point.
//
// Build/flash with the default env: `pio run -e openrb150 -t upload`. That env
// uses the bundled custom board (boards/openrb150.json) + variant
// (variants/OpenRB-150) so Serial1 = DXL bus, Serial3 = CRSF, optional Serial2
// is enabled with HEXAPOD_ENABLE_SERIAL2, and
// BDPIN_DXL_PWR_EN is the real power FET. See doc/mkrzero-vs-openrb150.md.
//
// Phase 1 (rbg.3) state: board HAL safe boot + FreeRTOS task skeleton. After
// init, setup() starts the scheduler which runs the six Phase 1 tasks (control,
// dxl, rc, api, i2c, health). DYNAMIXEL power stays OFF at boot; the health task
// reports stack high-water marks, loop counts, and the software watchdog mask
// over USB CDC. No motion / servo bus / I2C traffic yet.
// ---------------------------------------------------------------------------

void setup() {
  fault_capture::init();

  // PM->RCAUSE identifies the previous reset (POR/BOD/EXT/WDT/SYST). Capture it
  // before peripheral initialization so post-reset diagnostics are definitive.
  app::setResetCause(PM->RCAUSE.reg);

  // Safe boot: configure pins and force DYNAMIXEL power OFF before anything.
  board::init();
  fault_capture::markStartupStage(
      fault_capture::StartupStage::BoardInitialized);

  // USB CDC (Serial) host link. Non-blocking: do not wait for the host so the
  // board still runs when nothing is connected.
  Serial.begin(115200);
  fault_capture::markStartupStage(fault_capture::StartupStage::SerialStarted);

  // Create the RTOS tasks and start the scheduler. Does not return.
  fault_capture::markStartupStage(fault_capture::StartupStage::AppStarting);
  app::start();
}

void loop() {
  // Unused: the FreeRTOS scheduler owns the CPU after app::start().
}