#include "hold_targets.h"

#include "../config/config_schema.h"

namespace dxl {

bool buildHoldTargets(const uint8_t* ids, uint8_t id_count,
                      const ServoStatus* statuses, uint8_t status_count,
                      GoalTarget* out, uint8_t out_cap) {
  if (ids == nullptr || statuses == nullptr || out == nullptr || id_count == 0 ||
      out_cap < id_count) {
    return false;
  }

  for (uint8_t i = 0; i < id_count; ++i) {
    bool found = false;
    for (uint8_t s = 0; s < status_count; ++s) {
      if (statuses[s].id != ids[i]) continue;
      if (!statuses[s].ok || statuses[s].present_position < 0 ||
          statuses[s].present_position > config::kServoMaxTick) {
        return false;
      }
      out[i].id = ids[i];
      out[i].tick = static_cast<uint16_t>(statuses[s].present_position);
      found = true;
      break;
    }
    if (!found) return false;
  }
  return true;
}

bool torqueOffConfirmed(bool power_enabled, bool configured_coverage,
                        bool all_discovered_off) {
  return !power_enabled || (configured_coverage && all_discovered_off);
}

}  // namespace dxl