#pragma once

// ===========================================================================
// DYNAMIXEL servo model / capability profile (portable, no Arduino deps).
//
// The MX-28AT actuators on this robot can appear with two different control
// tables (AGENTS.md 4.2):
//
//   * legacy MX-28 (Protocol 1.0 table)  -> model number 29
//   * MX-28(2.0)   (Protocol 2.0 table)  -> model number 30
//
// The firmware must NOT hard-code one table. At scan time the bus manager pings
// each servo, reads its model number + firmware version, and builds a per-servo
// ServoProfile describing which logical operations are available and which
// control table addressing applies. The actual register addresses are resolved
// by Dynamixel2Arduino's ControlTableItem mapping; this header only classifies
// capabilities so higher layers (limits, telemetry, param API) can branch
// safely.
//
// This translation unit is deliberately free of Arduino/Dynamixel2Arduino
// includes so it can be unit-tested on the host (pio test -e native).
// ===========================================================================

#include <stdint.h>

namespace dxl {

// Known DYNAMIXEL model numbers relevant to this project.
constexpr uint16_t kModelMx28Legacy = 29;  // MX-28 / MX-28AT, Protocol 1.0 table
constexpr uint16_t kModelMx28_2 = 30;      // MX-28(2.0), Protocol 2.0 table

// Firmware version at/above which MX-28(2.0) is assumed to support Fast Sync
// Read. PROVISIONAL: confirm against real servos during HIL bring-up (rbg.10).
constexpr uint8_t kFastSyncReadMinFw = 42;

// Which control table layout a servo uses. Drives address selection for limit
// writes (CW/CCW angle limit vs min/max position limit) and telemetry.
enum class TableKind : uint8_t {
  Unknown = 0,
  Mx28Legacy = 1,  // Protocol 1.0 control table
  Mx28V2 = 2,      // MX-28(2.0) / Protocol 2.0 control table
};

// Per-servo capability profile, built at scan time (AGENTS.md 4.2).
struct ServoProfile {
  uint8_t id = 0;
  uint16_t model_number = 0;
  uint8_t firmware_version = 0;
  uint8_t protocol_version = 0;  // 1 or 2; 0 = unknown
  TableKind table_kind = TableKind::Unknown;
  bool present = false;  // responded to ping during the last scan

  // Capability flags derived from the table kind / firmware version.
  bool supports_sync_read = false;
  bool supports_fast_sync_read = false;
  bool supports_cw_ccw_angle_limits = false;
  bool supports_min_max_position_limits = false;
  bool supports_profile_velocity = false;
  bool supports_bus_watchdog = false;

  // Live-ish state captured alongside the profile.
  bool torque_enabled = false;
  uint8_t last_error = 0;  // last DXL/library error code seen for this servo
};

// Classify a model number into its control table kind.
TableKind tableKindFromModel(uint16_t model_number);

// Default protocol version (1 or 2; 0 = unknown) implied by a table kind.
uint8_t defaultProtocolForTable(TableKind kind);

// Populate the capability flags of `profile` from a model number and firmware
// version. Leaves id/present/torque_enabled/last_error untouched so the caller
// can fill those from the live ping/read.
void fillProfileFromModel(ServoProfile& profile, uint16_t model_number,
                          uint8_t firmware_version);

}  // namespace dxl
