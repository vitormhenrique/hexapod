#include "dxl_model.h"

namespace dxl {

TableKind tableKindFromModel(uint16_t model_number) {
  switch (model_number) {
    case kModelMx28Legacy:
      return TableKind::Mx28Legacy;
    case kModelMx28_2:
      return TableKind::Mx28V2;
    default:
      return TableKind::Unknown;
  }
}

uint8_t defaultProtocolForTable(TableKind kind) {
  switch (kind) {
    case TableKind::Mx28Legacy:
      return 1;
    case TableKind::Mx28V2:
      return 2;
    default:
      return 0;
  }
}

void fillProfileFromModel(ServoProfile& profile, uint16_t model_number,
                          uint8_t firmware_version) {
  profile.model_number = model_number;
  profile.firmware_version = firmware_version;
  profile.table_kind = tableKindFromModel(model_number);
  profile.protocol_version = defaultProtocolForTable(profile.table_kind);

  switch (profile.table_kind) {
    case TableKind::Mx28Legacy:
      // Protocol 1.0 table: joint bounds are CW/CCW Angle Limit; speed is
      // Moving Speed; no Sync Read and no Bus Watchdog register.
      profile.supports_cw_ccw_angle_limits = true;
      profile.supports_min_max_position_limits = false;
      profile.supports_profile_velocity = false;
      profile.supports_bus_watchdog = false;
      profile.supports_sync_read = false;
      profile.supports_fast_sync_read = false;
      break;

    case TableKind::Mx28V2:
      // Protocol 2.0 table: joint bounds are Min/Max Position Limit; Profile
      // Velocity, Bus Watchdog and Sync Read are available. Fast Sync Read
      // depends on firmware version.
      profile.supports_cw_ccw_angle_limits = false;
      profile.supports_min_max_position_limits = true;
      profile.supports_profile_velocity = true;
      profile.supports_bus_watchdog = true;
      profile.supports_sync_read = true;
      profile.supports_fast_sync_read =
          (firmware_version >= kFastSyncReadMinFw);
      break;

    case TableKind::Unknown:
    default:
      profile.supports_cw_ccw_angle_limits = false;
      profile.supports_min_max_position_limits = false;
      profile.supports_profile_velocity = false;
      profile.supports_bus_watchdog = false;
      profile.supports_sync_read = false;
      profile.supports_fast_sync_read = false;
      break;
  }
}

}  // namespace dxl
