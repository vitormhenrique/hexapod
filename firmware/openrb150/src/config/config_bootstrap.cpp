#include "config_bootstrap.h"

namespace config {

BootstrapResult loadOrInitializeConfig(
    ConfigFile& file, ConfigStore& store, const uint8_t* defaults,
    uint16_t defaults_len, uint8_t* out, uint16_t out_capacity,
    uint16_t& out_len) {
  out_len = 0;
  uint32_t file_size = 0;
  if (!file.size(file_size)) return BootstrapResult::StorageError;

  if (store.load(out, out_capacity, out_len)) {
    return BootstrapResult::Loaded;
  }
  if (defaults == nullptr || defaults_len == 0 ||
      !store.commit(defaults, defaults_len) ||
      !store.load(out, out_capacity, out_len)) {
    out_len = 0;
    return BootstrapResult::StorageError;
  }
  return file_size == 0 ? BootstrapResult::InitializedDefaults
                        : BootstrapResult::RecoveredDefaults;
}

}  // namespace config