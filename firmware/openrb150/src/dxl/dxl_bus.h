#pragma once

// ===========================================================================
// DYNAMIXEL bus manager (single owner of Serial1 / the DXL TTL bus).
//
// Per AGENTS.md 5.1 only ONE task (task_dxl) may touch Dynamixel2Arduino and
// Serial1. This class encapsulates that ownership: nothing else in the firmware
// includes Dynamixel2Arduino. It provides the maintenance-safe scan / ping /
// status-read path required by Phase 1 (rbg.6).
//
// Maintenance-safe means: this class never enables servo torque and never
// writes goal positions. It only pings, classifies servos (legacy MX-28 vs
// MX-28(2.0), see dxl_model.h), and reads present status. Bus/communication
// failures are counted, not retried indefinitely, and never block the caller
// for more than the per-transaction timeout.
//
// Arduino-only: this header pulls in Dynamixel2Arduino and therefore is NOT
// part of the host/native build. The portable capability logic lives in
// dxl_model.{h,cpp}, which is unit-tested separately.
// ===========================================================================

#include <Dynamixel2Arduino.h>
#include <stdint.h>

#include "dxl_model.h"
#include "dxl_sync.h"

namespace dxl {

// Present-status snapshot from a single servo (torque-off read).
struct ServoStatus {
  uint8_t id = 0;
  int32_t present_position = 0;   // raw ticks
  int32_t present_velocity = 0;   // raw (legacy Present Speed / 2.0 Present Velocity)
  int32_t present_load = 0;       // signed raw load (legacy) / 0.1% (MX 2.0)
  uint16_t present_voltage_mv = 0;  // millivolts (raw 0.1 V units * 100)
  int8_t present_temperature_c = 0;
  uint8_t hardware_error = 0;  // MX(2.0) Hardware Error Status; 0 on legacy
  bool torque_enabled = false;
  bool ok = false;  // true if at least the present position read succeeded
};

// One goal-position command for the sync-write path.
struct GoalTarget {
  uint8_t id = 0;
  uint16_t tick = 2048;
};


// Aggregate bus health counters (reported via telemetry/diagnostics later).
struct BusStats {
  uint32_t scans = 0;
  uint32_t pings_ok = 0;
  uint32_t pings_fail = 0;
  uint32_t reads_ok = 0;
  uint32_t reads_fail = 0;
  uint32_t writes_ok = 0;
  uint32_t writes_fail = 0;
  uint8_t last_error = 0;  // last library/protocol error code observed
};

class DxlBus {
 public:
  // Storage for discovered servos. The hexapod has 18; keep headroom.
  static constexpr uint8_t kMaxServos = 24;
  static constexpr uint8_t kMaxServoId = 252;  // 253/0xFE is broadcast
  static constexpr uint32_t kDefaultBaud = 57600;  // DXL factory default

  explicit DxlBus(HardwareSerial& port);

  // Initialize the DXL UART. Does NOT enable DXL power (board HAL owns that)
  // and does NOT enable torque. Safe to call once before any scan.
  void begin(uint32_t baud = kDefaultBaud);
  bool isReady() const { return ready_; }
  uint32_t baud() const { return baud_; }

  // Ping a single ID, trying Protocol 2.0 then 1.0, and classify it. Returns
  // true and fills `out` on success. Maintenance-safe (no torque/goal writes).
  bool ping(uint8_t id, ServoProfile& out);

  // Scan the inclusive ID range [first_id, last_id], rebuilding the profile
  // table. Returns the number of servos found and stored. Requires DXL power
  // to be ON to detect anything.
  uint8_t scan(uint8_t first_id, uint8_t last_id);

  // Read present status (torque-off safe) from a known/just-pinged servo.
  bool readStatus(uint8_t id, ServoStatus& out);

  // --- Motion path (Phase 2, 22l.6) ----------------------------------------
  // These DO command the servos and must only be called from an armed/torque-on
  // motion context. They are no-ops if the bus is not ready.

  // Enable or disable torque on every discovered servo. Returns the number of
  // servos that acknowledged. Maintenance/arming use only (never per cycle).
  uint8_t setTorqueAll(bool on);

  // Sync Write goal positions for `count` targets in as few bus transactions as
  // possible (one Sync Write per control-table group, chunked to the library
  // node limit). Targets whose ID is not a discovered servo are skipped.
  // Returns true if every group write succeeded. Updates write stats.
  bool writeGoalPositions(const GoalTarget* targets, uint8_t count);

  // Sync Read the present-status block from all discovered servos (grouped by
  // control table) and decode into `out` (indexed to match servo order; see
  // servoCount()/profile()). Returns the number of servos with a fresh read.
  uint8_t syncReadStatus(ServoStatus* out, uint8_t out_cap);

  // --- Logical parameter access (maintenance, ubs.4.4) ----------------------
  // Raw register read/write by address, used by the logical-parameter API. The
  // caller resolves the (table-aware) address/length from dxl/dxl_params.h and
  // passes them here; this layer only selects the protocol and moves bytes.

  // Read `len` (1/2/4) bytes from `addr` on servo `id` using `table`'s
  // protocol, decoding into a (optionally signed) int32. Returns false on a
  // failed transaction.
  bool readRegister(uint8_t id, TableKind table, uint16_t addr, uint8_t len,
                    bool is_signed, int32_t& out);

  // Write `value` (low `len` bytes, little-endian) to `addr` on servo `id`.
  // Returns false on a failed transaction. Does NOT manage torque; EEPROM
  // writes require the caller to disable torque first (see setTorqueOne).
  bool writeRegister(uint8_t id, TableKind table, uint16_t addr, uint8_t len,
                     int32_t value);

  // Enable/disable torque on a single servo. Returns true on ack.
  bool setTorqueOne(uint8_t id, TableKind table, bool on);

  // Read a single servo's torque-enable state. Returns false on a failed read.
  bool torqueState(uint8_t id, TableKind table, bool& on);

  // Firmware-authoritative torque view (lmt.6): true only when every discovered
  // servo's last commanded torque state is OFF. dxlTask is the sole bus owner,
  // so the per-servo torque flag cached by setTorqueAll/setTorqueOne is the
  // source of truth for passive-pose gating without extra bus reads. A servo
  // that self-shuts-down on a hardware fault is only ever reported torque ON
  // here, so this never falsely reports torque off (the safe direction).
  bool allTorqueOff() const;

  // Discovered-servo accessors.
  uint8_t servoCount() const { return count_; }
  const ServoProfile& profile(uint8_t index) const { return servos_[index]; }
  const ServoProfile* profileById(uint8_t id) const;

  const BusStats& stats() const { return stats_; }

 private:
  // Read one logical control-table item, reporting failure via the library
  // error code. Returns true on a clean read.
  bool readItem(uint8_t item_idx, uint8_t id, int32_t& value);

  // Select the bus protocol version (1.0/2.0) implied by a table kind.
  void selectProtocol(TableKind kind);

  Dynamixel2Arduino dxl_;
  ServoProfile servos_[kMaxServos];
  uint8_t count_ = 0;
  BusStats stats_;
  uint32_t baud_ = kDefaultBaud;
  bool ready_ = false;

  // Library scratch params for grouped Sync Write / Sync Read. Kept as members
  // (not on the stack) because they are large and this class has a single
  // owning task, so there is no reentrancy.
  ParamForSyncWriteInst_t sw_param_;
  ParamForSyncReadInst_t sr_param_;
  RecvInfoFromStatusInst_t sr_recv_;
};

}  // namespace dxl
