#include <Arduino.h>

#include "app/tasks.h"
#include "board/board.h"
#include "safety/fault_capture.h"

// ---------------------------------------------------------------------------
// Hexapod firmware entry point for the MicroMod Teensy carrier and OpenRB-150.
//
// Build/flash the default MicroMod Teensy target with
// `pio run -e teensymm -t upload`. Board-specific pin maps select Serial2,
// G2 direction, and G3 switched servo power on the carrier, or the original
// Serial1/automatic-direction/power-FET wiring on OpenRB-150.
//
// setup() starts the six control, DXL, RC, API, I2C, and health tasks.
// DYNAMIXEL direction and switched power remain LOW until the bounded arming
// sequence validates the battery, configured servos, status, and hold pose.
// ---------------------------------------------------------------------------

void setup() {
  fault_capture::init();

#if defined(ARDUINO_ARCH_SAMD)
  // PM->RCAUSE identifies the previous reset (POR/BOD/EXT/WDT/SYST). Capture it
  // before peripheral initialization so post-reset diagnostics are definitive.
  app::setResetCause(PM->RCAUSE.reg);
#elif defined(__IMXRT1062__)
  // SRC_SRSR accumulates reset causes (POR/WDOG/JTAG/...). Capture and clear
  // (write-1-to-clear) so the next boot reports fresh bits.
  const uint32_t srsr = SRC_SRSR;
  SRC_SRSR = srsr;
  app::setResetCause(static_cast<uint8_t>(srsr));
#else
  app::setResetCause(0);
#endif

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