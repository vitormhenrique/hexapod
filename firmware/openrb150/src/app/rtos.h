#pragma once

// ===========================================================================
// FreeRTOS include shim.
//
// Selects the FreeRTOS port headers for the build target so task/adapter code
// stays port-agnostic:
//   - SAMD21 (OpenRB-150): briscoetech/FreeRTOS_SAMD21 with the project's
//     freertos_config.h force-included from platformio.ini.
//   - IMXRT1062 (MicroMod Teensy): tsandmann/freertos-teensy. The env defines
//     USE_ARDUINO_DEFINES so arduino_freertos.h keeps the stock Arduino
//     macros (HIGH/LOW/OUTPUT/...) intact.
// ===========================================================================

#if defined(ARDUINO_ARCH_SAMD)
#include <FreeRTOS_SAMD21.h>
#elif defined(__IMXRT1062__) || defined(TEENSYDUINO)
#include "arduino_freertos.h"
#include "semphr.h"
#else
#error "No FreeRTOS port configured for this target"
#endif
