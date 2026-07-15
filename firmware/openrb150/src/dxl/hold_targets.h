#pragma once

#include <stdint.h>

#include "dxl_status.h"
#include "dxl_sync.h"

namespace dxl {

// Build a hold-position goal for every requested servo ID. Returns false unless
// every ID has a fresh, single-turn present-position sample and the output can
// hold the complete set. No partial result may be used for torque enable.
bool buildHoldTargets(const uint8_t* ids, uint8_t id_count,
                      const ServoStatus* statuses, uint8_t status_count,
                      GoalTarget* out, uint8_t out_cap);

// Power-off is inherently torque-safe. With DXL power present, passive mode
// requires the full configured bus to be discovered and every profile off.
bool torqueOffConfirmed(bool power_enabled, bool configured_coverage,
                        bool all_discovered_off);

}  // namespace dxl