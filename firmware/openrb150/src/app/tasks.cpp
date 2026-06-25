#include "tasks.h"

#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>

#include "../board/board.h"
#include "../config/config_api.h"
#include "../config/eeprom_24lc32.h"
#include "../dxl/dxl_bus.h"
#include "../input/crsf_parser.h"
#include "../protocol/api.h"
#include "../protocol/frame_reader.h"
#include "../sensors/i2c_bus.h"
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

// Single owner of the DYNAMIXEL TTL bus (Serial1). Only dxlTask touches this,
// satisfying the AGENTS.md rule that one task owns Dynamixel2Arduino/Serial1.
dxl::DxlBus g_dxlBus(Serial1);

// Latest per-servo status snapshot, published by dxlTask via Sync Read and read
// by telemetry/safety consumers. Single writer (dxlTask); readers take a copy.
dxl::ServoStatus g_servoStatus[dxl::DxlBus::kMaxServos];
volatile uint8_t g_servoStatusCount = 0;

// CRSF/ExpressLRS RC input state. Owned exclusively by rcTask (Serial2).
crsf::Parser g_crsfParser;
crsf::RcStatus g_rcStatus;

// CRSF runs at 420000 baud on the OpenRB-150 4-pin UART (Serial2).
constexpr uint32_t kCrsfBaud = 420000;

// Single owner of the root I2C bus (Wire): TCA9548A mux, 24LC32 EEPROM, and the
// muxed foot sensors. Only i2cTask touches this. Topology is the boot-scan
// result describing which optional devices were found.
i2c::I2cBus g_i2cBus(Wire);
i2c::I2cTopology g_i2cTopology;

// Persistent robot config in the 24LC32 EEPROM (root bus). When the EEPROM is
// missing or holds no valid slot the config is marked volatile and the firmware
// must run on compiled defaults and reject commits (AGENTS.md 4.3). At boot the
// i2cTask loads any valid slot and hands it to apiTask; thereafter the config
// API (apiTask) edits a RAM shadow and routes CFG_COMMIT back to i2cTask.
config::Eeprom24LC32 g_eeprom(Wire);
config::ConfigStore g_configStore(g_eeprom);
bool g_configVolatile = true;

// --- Cross-task config plumbing (AGENTS.md 5.1: only i2cTask touches Wire) ---
//
// The config API runs in apiTask (it parses USB frames), but the EEPROM commit
// is a Wire transaction that only i2cTask is allowed to perform. So apiTask
// edits/validates a RAM shadow locally, and a CFG_COMMIT hands the validated
// serialized payload to i2cTask through this mailbox and blocks (bounded) for
// the result. A separate one-shot boot-load buffer lets i2cTask pass a valid
// persisted config to apiTask so the ConfigApi shadow is still touched by only
// one task.
struct CommitMailbox {
  bool requested = false;
  bool ok = false;
  uint16_t len = 0;
  uint8_t payload[config::kConfigPayloadSize] = {0};
};
CommitMailbox g_commit;
SemaphoreHandle_t g_commitMutex = nullptr;  // guards g_commit
SemaphoreHandle_t g_commitDone = nullptr;   // i2cTask -> apiTask completion

struct BootLoad {
  bool ready = false;     // a valid persisted payload was loaded at boot
  bool consumed = false;  // apiTask has adopted it
  uint16_t len = 0;
  uint8_t payload[config::kConfigPayloadSize] = {0};
};
BootLoad g_bootLoad;

// Persistence sink used by the config API. commitPayload() is called from
// apiTask; it forwards the bytes to i2cTask and waits for the transaction.
class TaskConfigPersistence : public config::ConfigPersistence {
 public:
  bool commitPayload(const uint8_t* payload, uint16_t len) override {
    if (g_commitMutex == nullptr || g_commitDone == nullptr) return false;
    if (len > sizeof(g_commit.payload)) return false;
    xSemaphoreTake(g_commitMutex, portMAX_DELAY);
    memcpy(g_commit.payload, payload, len);
    g_commit.len = len;
    g_commit.ok = false;
    g_commit.requested = true;
    xSemaphoreGive(g_commitMutex);
    // Wait for i2cTask to run the EEPROM transaction (normally < 200 ms).
    if (xSemaphoreTake(g_commitDone, pdMS_TO_TICKS(1500)) != pdTRUE) {
      return false;  // timed out
    }
    xSemaphoreTake(g_commitMutex, portMAX_DELAY);
    const bool ok = g_commit.ok;
    xSemaphoreGive(g_commitMutex);
    return ok;
  }
  bool persistent() const override { return !g_configVolatile; }
};

TaskConfigPersistence g_configPersist;
config::ConfigApi g_configApi(g_configPersist);

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
  // Bring up the DXL UART once. This only initializes Serial1; it does NOT
  // enable DXL power (board HAL owns that) or servo torque, so it is safe at
  // boot. Scanning is deferred to a maintenance command once power is on.
  g_dxlBus.begin();
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Dxl);
    // Publish a fresh present-position snapshot for all discovered servos in a
    // single Sync Read per control table. This is a no-op until a maintenance
    // scan populates the servo table (DXL power is OFF at boot), and never
    // enables torque or writes goals. The goal Sync-Write path (writeGoal-
    // Positions) is implemented and unit-tested at the driver level; its task-
    // level activation is gated by arming/arbitration (22l.8/22l.10).
    if (g_dxlBus.servoCount() > 0) {
      const uint8_t n =
          g_dxlBus.syncReadStatus(g_servoStatus, dxl::DxlBus::kMaxServos);
      g_servoStatusCount = n;
    }
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kDxl));
  }
}

void rcTask(void*) {
  crsf::initRcStatus(g_rcStatus);
#if defined(PIN_SERIAL2_RX)
  // Serial2 is the ExpressLRS CRSF receiver link; rcTask owns it exclusively.
  Serial2.begin(kCrsfBaud);
#endif
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Rc);
    const uint32_t now_ms =
        static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
#if defined(PIN_SERIAL2_RX)
    crsf::ChannelData frame;
    while (Serial2.available() > 0) {
      const uint8_t b = static_cast<uint8_t>(Serial2.read());
      if (g_crsfParser.push(b, frame)) {
        crsf::applyFrame(g_rcStatus, frame, now_ms);
      }
    }
#endif
    // Raise failsafe if no valid RC frame has arrived within the timeout.
    crsf::evaluateFailsafe(g_rcStatus, now_ms);
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kRc));
  }
}

void apiTask(void*) {
  static protocol::FrameReader reader;
  static uint8_t out[protocol::kMaxWireFrame];
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Api);

    // Adopt a persisted config once i2cTask has loaded one at boot. Done here
    // (not at task start) because i2cTask's scan may finish after this task
    // begins; the ConfigApi shadow is thus only ever touched by apiTask.
    if (g_bootLoad.ready && !g_bootLoad.consumed) {
      g_configApi.adoptPayload(g_bootLoad.payload, g_bootLoad.len);
      g_bootLoad.consumed = true;
    }

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
          reader.body(), reader.length(), g_deviceInfo, st, out, sizeof(out),
          &g_configApi);
      if (n > 0) {
        Serial.write(out, n);
      }
    }

    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kApi));
  }
}

void i2cTask(void*) {
  // Bring up the root I2C bus and run a one-time discovery scan so capabilities
  // (mux/EEPROM presence, per-channel foot sensors) are known early. Running it
  // here keeps the blocking probe work off the control loop.
  g_i2cBus.begin();
  g_i2cBus.scanAll(g_i2cTopology);
  // If the config EEPROM is present and holds a valid slot, load it and hand it
  // to apiTask to adopt as the active config. Otherwise stay volatile so the
  // firmware runs on compiled defaults and CFG_COMMIT is rejected (AGENTS.md
  // 4.3).
  if (g_i2cTopology.eeprom_present) {
    config::SlotStatus slots[config::kSlotCount];
    g_configStore.inspect(slots);
    if (slots[0].valid || slots[1].valid) {
      uint16_t n = 0;
      if (g_configStore.load(g_bootLoad.payload, sizeof(g_bootLoad.payload),
                             n)) {
        g_bootLoad.len = n;
        g_bootLoad.ready = true;
        g_configVolatile = false;
      } else {
        g_configVolatile = true;
      }
    } else {
      g_configVolatile = true;
    }
  } else {
    g_configVolatile = true;
  }
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::I2c);

    // Service a config commit handed over by apiTask. i2cTask is the sole owner
    // of Wire/EEPROM, so the transactional store write happens here.
    bool do_commit = false;
    if (g_commitMutex != nullptr) {
      xSemaphoreTake(g_commitMutex, portMAX_DELAY);
      do_commit = g_commit.requested;
      xSemaphoreGive(g_commitMutex);
    }
    if (do_commit) {
      const bool ok = g_configStore.commit(g_commit.payload, g_commit.len);
      xSemaphoreTake(g_commitMutex, portMAX_DELAY);
      g_commit.ok = ok;
      g_commit.requested = false;
      xSemaphoreGive(g_commitMutex);
      if (ok) g_configVolatile = false;  // a valid slot now exists
      xSemaphoreGive(g_commitDone);
    }

    // TODO (Phase 2): staggered sensor reads. This task owns Wire exclusively;
    // mux channels are selected one-hot per access.
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

  // Sync primitives for the apiTask <-> i2cTask config commit hand-off. Created
  // here at boot (before the scheduler), not at runtime.
  g_commitMutex = xSemaphoreCreateMutex();
  g_commitDone = xSemaphoreCreateBinary();

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
