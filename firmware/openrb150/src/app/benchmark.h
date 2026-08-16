#pragma once

#include <stdint.h>

namespace app {
namespace benchmark {

// Keep this stable: the benchmark telemetry payload is decoded by the
// companion and compared across OpenRB-150 and MicroMod Teensy builds.
enum class Task : uint8_t {
  Control = 0,
  Dxl = 1,
  Rc = 2,
  Api = 3,
  I2c = 4,
  Health = 5,
  Count = 6,
};

constexpr uint8_t kTaskCount = static_cast<uint8_t>(Task::Count);

struct Metric {
  uint32_t count = 0;
  uint32_t total_us = 0;
  uint32_t max_us = 0;
  uint32_t max_release_jitter_us = 0;
  uint32_t overruns = 0;
};

struct Snapshot {
  Metric tasks[kTaskCount];
  Metric control_step;
  Metric motion_control_step;
  Metric motion_dxl_cycle;
  Metric i2c_oled;
  Metric i2c_imu;
  Metric i2c_storage;
};

#if defined(HEXAPOD_BENCHMARK)

class Collector {
 public:
  void reset();
  void recordTask(Task task, uint32_t started_us, uint32_t finished_us,
                  uint32_t nominal_period_us);
  void recordControlStep(uint32_t runtime_us, bool motion_gate);
  void recordDxlCycle(uint32_t runtime_us, bool motion_gate);
  void recordI2cOled(uint32_t runtime_us);
  void recordI2cImu(uint32_t runtime_us);
  void recordI2cStorage(uint32_t runtime_us);
  Snapshot snapshot() const;

 private:
  static void update(Metric& metric, uint32_t runtime_us);

  Metric tasks_[kTaskCount];
  uint32_t last_started_us_[kTaskCount] = {0};
  bool have_started_[kTaskCount] = {false};
  Metric control_step_;
  Metric motion_control_step_;
  Metric motion_dxl_cycle_;
  Metric i2c_oled_;
  Metric i2c_imu_;
  Metric i2c_storage_;
};

extern Collector g_collector;

#else

class Collector {
 public:
  void reset() {}
  void recordTask(Task, uint32_t, uint32_t, uint32_t) {}
  void recordControlStep(uint32_t, bool) {}
  void recordDxlCycle(uint32_t, bool) {}
  void recordI2cOled(uint32_t) {}
  void recordI2cImu(uint32_t) {}
  void recordI2cStorage(uint32_t) {}
  Snapshot snapshot() const { return Snapshot{}; }
};

extern Collector g_collector;

#endif

}  // namespace benchmark
}  // namespace app