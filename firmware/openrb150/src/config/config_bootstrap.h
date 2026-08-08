#pragma once

#include <stdint.h>

#include "config_store.h"

namespace config {

enum class BootstrapResult : uint8_t {
  Loaded = 0,
  InitializedDefaults = 1,
  RecoveredDefaults = 2,
  StorageError = 3,
};

// Load the newest valid config record. A missing or empty file is initialized
// with `defaults`; a non-empty invalid prefix is preserved and followed by a
// complete default record so an interrupted first boot can recover.
BootstrapResult loadOrInitializeConfig(
    ConfigFile& file, ConfigStore& store, const uint8_t* defaults,
    uint16_t defaults_len, uint8_t* out, uint16_t out_capacity,
    uint16_t& out_len);

}  // namespace config