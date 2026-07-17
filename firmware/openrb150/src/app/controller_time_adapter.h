#pragma once

// Target-only conversion from the FreeRTOS scheduler clock to the portable
// ControllerTime contract. ControllerCore includes controller_time.h directly;
// only the ATSAMD21 task adapter includes this header.

#include <FreeRTOS_SAMD21.h>

#include "../controller/controller_time.h"

namespace app {

inline controller::ControllerTime controllerTimeFromFreeRtos(
    controller::ControllerClock& clock) {
  return controller::controllerTimeFromTickCount(
      clock, static_cast<uint32_t>(xTaskGetTickCount()),
      static_cast<uint32_t>(portTICK_PERIOD_MS));
}

}  // namespace app