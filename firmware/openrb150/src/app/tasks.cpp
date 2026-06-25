#include "tasks.h"

#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>

#include "../board/board.h"
#include "../protocol/api.h"
#include "../protocol/frame_reader.h"
#include "../safety/system_state.h"
#include "../safety/watchdog.h"
#include "task_config.h"

namespace app {
namespace {

// Task handles, indexed by watchdog::TaskId, so the health task can read each
// task's stack high-water mark.
TaskHandle_t g_handles[watchdog::kTaskCount] = {nullptr};

// Per-task loop counters (single producer each), reported by the health task.
volatile uint32_t g_loops[watchdog::kTaskCount] = {0};

// Static description of this firmware build, reported by HELLO/GET_CAPABILITIES.
protocol::api::DeviceInfo g_deviceInfo;

void initDeviceInfo() {
  g_deviceInfo.fw_major = 0;
  g_deviceInfo.fw_minor = 1;
  g_deviceInfo.fw_patch = 0;
  g_deviceInfo.feature_bits = 0;  // populated as features land in Phase 2
  const char name[] = "OpenRB150-Hex";
  size_t i = 0;
  for (; name[i] != '\0' && i < protocol::api::kDeviceNameLen; ++i) {
    g_deviceInfo.device_name[i] = name[i];
  }
  for (; i < protocol::api::kDeviceNameLen; ++i) {
    g_deviceInfo.device_name[i] = 0;
  }
}

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
  static protocol::FrameReader reader;
  static uint8_t out[protocol::kMaxWireFrame];
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Api);

    // Drain any received bytes, framing them into complete request bodies.
    while (Serial.available() > 0) {
      const uint8_t b = static_cast<uint8_t>(Serial.read());
      if (!reader.push(b)) {
        continue;
      }

      // Refresh the live status snapshot for this request.
      protocol::api::StatusSnapshot st;
      st.uptime_ms =
          static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
      st.state = static_cast<uint8_t>(safety::State::Disarmed);
      st.dxl_power = board::dxlPowerEnabled();
      st.dxl_power_control = board::hasDxlPowerControl();
      st.battery_mv = board::readBatteryMilliVolts();
      st.watchdog_missed = watchdog::missedMask();

      const size_t n = protocol::api::handleRequest(
          reader.body(), reader.length(), g_deviceInfo, st, out, sizeof(out));
      if (n > 0) {
        Serial.write(out, n);
      }
    }

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
    // driven by blinkTask as the FreeRTOS liveness indicator. USB CDC is owned
    // by the api task per AGENTS.md 5.1, so the host reads health via the
    // GET_STATUS command rather than text printed here.)
    watchdog::evaluate();

    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kHealth));
  }
}

}  // namespace

void start() {
  watchdog::init();
  initDeviceInfo();

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
