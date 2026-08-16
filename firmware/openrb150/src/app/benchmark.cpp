#include "benchmark.h"

namespace app {
namespace benchmark {

Collector g_collector;

#if defined(HEXAPOD_BENCHMARK)

void Collector::reset() {
  for (uint8_t i = 0; i < kTaskCount; ++i) {
    tasks_[i] = Metric{};
    last_started_us_[i] = 0;
    have_started_[i] = false;
  }
  control_step_ = Metric{};
  motion_control_step_ = Metric{};
  motion_dxl_cycle_ = Metric{};
  i2c_oled_ = Metric{};
  i2c_imu_ = Metric{};
  i2c_storage_ = Metric{};
}

void Collector::update(Metric& metric, uint32_t runtime_us) {
  ++metric.count;
  metric.total_us += runtime_us;
  if (runtime_us > metric.max_us) metric.max_us = runtime_us;
}

void Collector::recordTask(Task task, uint32_t started_us, uint32_t finished_us,
                           uint32_t nominal_period_us) {
  const uint8_t index = static_cast<uint8_t>(task);
  if (index >= kTaskCount) return;

  Metric& metric = tasks_[index];
  const uint32_t runtime_us = finished_us - started_us;
  update(metric, runtime_us);
  if (runtime_us > nominal_period_us) ++metric.overruns;

  if (have_started_[index]) {
    const uint32_t actual_period_us = started_us - last_started_us_[index];
    const uint32_t jitter_us = actual_period_us > nominal_period_us
                                   ? actual_period_us - nominal_period_us
                                   : nominal_period_us - actual_period_us;
    if (jitter_us > metric.max_release_jitter_us) {
      metric.max_release_jitter_us = jitter_us;
    }
  }
  last_started_us_[index] = started_us;
  have_started_[index] = true;
}

void Collector::recordControlStep(uint32_t runtime_us, bool motion_gate) {
  update(control_step_, runtime_us);
  if (motion_gate) update(motion_control_step_, runtime_us);
}

void Collector::recordDxlCycle(uint32_t runtime_us, bool motion_gate) {
  if (motion_gate) update(motion_dxl_cycle_, runtime_us);
}

void Collector::recordI2cOled(uint32_t runtime_us) {
  update(i2c_oled_, runtime_us);
}

void Collector::recordI2cImu(uint32_t runtime_us) {
  update(i2c_imu_, runtime_us);
}

void Collector::recordI2cStorage(uint32_t runtime_us) {
  update(i2c_storage_, runtime_us);
}

Snapshot Collector::snapshot() const {
  Snapshot out;
  for (uint8_t i = 0; i < kTaskCount; ++i) out.tasks[i] = tasks_[i];
  out.control_step = control_step_;
  out.motion_control_step = motion_control_step_;
  out.motion_dxl_cycle = motion_dxl_cycle_;
  out.i2c_oled = i2c_oled_;
  out.i2c_imu = i2c_imu_;
  out.i2c_storage = i2c_storage_;
  return out;
}

#endif

}  // namespace benchmark
}  // namespace app