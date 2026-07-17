#include "controller_time.h"

namespace controller {

void ControllerClock::reset() {
  last_now_ms_ = 0;
  initialized_ = false;
}

ControllerTime ControllerClock::sampleMilliseconds(uint32_t now_ms) {
  ControllerTime time;
  time.now_ms = now_ms;

  if (!initialized_) {
    initialized_ = true;
    last_now_ms_ = now_ms;
    time.valid = true;
    return time;
  }

  const uint32_t elapsed_ms = now_ms - last_now_ms_;
  last_now_ms_ = now_ms;
  if (elapsed_ms > max_elapsed_ms_) {
    return time;
  }

  time.dt_ms = elapsed_ms;
  time.valid = true;
  return time;
}

ControllerTime controllerTimeFromTickCount(ControllerClock& clock,
                                           uint32_t tick_count,
                                           uint32_t tick_period_ms) {
  if (tick_period_ms == 0) return ControllerTime{};
  return clock.sampleMilliseconds(tick_count * tick_period_ms);
}

}  // namespace controller