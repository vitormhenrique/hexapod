#include "tasks.h"

#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>

#include "../board/board.h"
#include "../safety/watchdog.h"
#include "task_config.h"

namespace app {
namespace {

// Task handles, indexed by watchdog::TaskId, so the health task can read each
// task's stack high-water mark.
TaskHandle_t g_handles[watchdog::kTaskCount] = {nullptr};

// Per-task loop counters (single producer each), reported by the health task.
volatile uint32_t g_loops[watchdog::kTaskCount] = {0};

inline void tick(watchdog::TaskId id) {
  const uint8_t i = static_cast<uint8_t>(id);
  g_loops[i]++;
  watchdog::checkIn(id);
}

// --- Task bodies ----------------------------------------------------------
// Each task runs a fixed-period loop with vTaskDelayUntil so timing does not
// drift with body execution time. Bodies are stubs for the skeleton.

void controlTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Control);
    // TODO: mode arbitration, gait phase, IK, servo target generation.
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kControl));
  }
}

void dxlTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Dxl);
    // TODO: DYNAMIXEL sync write/read; bus owns Serial1 exclusively.
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kDxl));
  }
}

void rcTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Rc);
    // TODO: parse CRSF from Serial2; normalize channels; failsafe detection.
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kRc));
  }
}

void apiTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Api);
    // TODO: host protocol framing over USB CDC; commands/telemetry.
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kApi));
  }
}

void i2cTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::I2c);
    // TODO: TCA9548A mux select, sensor reads, EEPROM config jobs.
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kI2c));
  }
}

// Minimal FreeRTOS liveness test: toggle the USER LED every 100 ms. If the LED
// blinks at 5 Hz the scheduler is running and ticking correctly. This owns the
// LED so no other task should write it. Not part of the watchdog set.
void blinkTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    board::toggleUserLed();
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kBlink));
  }
}

void healthTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Health);

    // Evaluate the software watchdog over the elapsed window. (The USER LED is
    // driven by blinkTask as the FreeRTOS liveness indicator.)
    watchdog::evaluate();

    if (Serial) {
      const uint32_t up_ms =
          static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
      Serial.print("[health] up_ms=");
      Serial.print(up_ms);
      Serial.print(" missed=0x");
      Serial.print(watchdog::missedMask(), HEX);
      Serial.print(" dxl_power=");
      Serial.print(board::dxlPowerEnabled() ? "ON" : "OFF");
      Serial.print(" vbatt_mv=");
      Serial.print(board::readBatteryMilliVolts());

      // Stack high-water marks (words still free; lower = tighter).
      Serial.print(" stack_free[ctrl,dxl,rc,api,i2c,hlth]=");
      for (uint8_t i = 0; i < watchdog::kTaskCount; ++i) {
        if (g_handles[i] != nullptr) {
          Serial.print(uxTaskGetStackHighWaterMark(g_handles[i]));
        } else {
          Serial.print('?');
        }
        Serial.print(i + 1 < watchdog::kTaskCount ? ',' : ' ');
      }

      Serial.print("loops[ctrl,dxl,rc,api,i2c,hlth]=");
      for (uint8_t i = 0; i < watchdog::kTaskCount; ++i) {
        Serial.print(g_loops[i]);
        Serial.print(i + 1 < watchdog::kTaskCount ? ',' : '\n');
      }
    }

    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kHealth));
  }
}

}  // namespace

void start() {
  watchdog::init();

  // Route FreeRTOS fault reporting to the USER LED and USB CDC.
  vSetErrorLed(board::pinUserLed(), HIGH);
  vSetErrorSerial(&Serial);

  // Task stacks are allocated once here, at boot, before the scheduler runs;
  // there is no runtime heap churn afterward (AGENTS.md 1.2).
  xTaskCreate(controlTask, "control", stack_words::kControl, nullptr,
              priority::kControl, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Control)]);
  xTaskCreate(dxlTask, "dxl", stack_words::kDxl, nullptr,
              priority::kDxl, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Dxl)]);
  xTaskCreate(rcTask, "rc", stack_words::kRc, nullptr,
              priority::kRc, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Rc)]);
  xTaskCreate(apiTask, "api", stack_words::kApi, nullptr,
              priority::kApi, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Api)]);
  xTaskCreate(i2cTask, "i2c", stack_words::kI2c, nullptr,
              priority::kI2c, &g_handles[static_cast<uint8_t>(watchdog::TaskId::I2c)]);
  xTaskCreate(healthTask, "health", stack_words::kHealth, nullptr,
              priority::kHealth, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Health)]);

  // Standalone FreeRTOS liveness test task (USER LED @ 5 Hz). Not tracked by
  // the watchdog; handle discarded.
  xTaskCreate(blinkTask, "blink", stack_words::kBlink, nullptr,
              priority::kBlink, nullptr);

  // Hands the CPU to the scheduler; does not return.
  vTaskStartScheduler();
}

}  // namespace app
