#pragma once

// ===========================================================================
// DYNAMIXEL sync-write / status-read addressing + (de)serialization (portable).
//
// The bus driver (dxl/dxl_bus, Arduino-only) performs the actual Sync Write and
// Sync Read instructions, but the protocol-version-dependent *layout* logic is
// pure data manipulation and lives here so it can be unit-tested on the host
// against golden byte vectors (AGENTS.md "treat the serial API as a public
// interface; add tests" + the portable-module pattern).
//
// Two control tables are supported (see dxl_model.h / AGENTS.md 4.2):
//   * legacy MX-28 (Protocol 1.0): Goal Position @30 (2B), present status block
//     @36..43 (8B): position/speed/load 2B each, voltage+temp 1B each. Speed and
//     load are sign-magnitude with bit 10 (0x400) as the direction bit.
//   * MX-28(2.0) (Protocol 2.0): Goal Position @116 (4B), present status block
//     @124..146 (23B): pwm/load 2B, velocity/position 4B (signed two's
//     complement), input voltage 2B, temperature 1B.
//
// All multi-byte fields are little-endian on the wire. Functions here never
// touch hardware; they only compute addresses, encode a goal tick, and decode a
// status block that the driver already read.
// ===========================================================================

#include <stdint.h>

#include "dxl_model.h"

namespace dxl {

// Goal Position register address per control table.
constexpr uint16_t kGoalAddrLegacy = 30;
constexpr uint16_t kGoalAddrV2 = 116;
constexpr uint8_t kGoalLenLegacy = 2;
constexpr uint8_t kGoalLenV2 = 4;

// Contiguous present-status block per control table.
constexpr uint16_t kStatusAddrLegacy = 36;
constexpr uint8_t kStatusLenLegacy = 8;   // 36..43
constexpr uint16_t kStatusAddrV2 = 124;
constexpr uint8_t kStatusLenV2 = 23;      // 124..146

// Present Position register (the high-rate field used for the all-servos Sync
// Read and for passive pose streaming / clamp checks).
constexpr uint16_t kPosAddrLegacy = 36;
constexpr uint8_t kPosLenLegacy = 2;
constexpr uint16_t kPosAddrV2 = 132;
constexpr uint8_t kPosLenV2 = 4;

// Max status block size across tables (for fixed buffers).
constexpr uint8_t kStatusBlockMax = kStatusLenV2;

// Decoded present-status fields (table-agnostic units).
struct StatusFields {
  int32_t position = 0;       // raw ticks
  int32_t velocity = 0;       // signed raw (table units)
  int32_t load = 0;           // signed raw (table units; 0.1% on V2)
  uint16_t voltage_mv = 0;    // millivolts
  int8_t temperature_c = 0;   // celsius
};

// Goal Position address / length for a table kind. Unknown falls back to the
// legacy layout (the MX-28AT factory default) so a mis-detected servo still
// uses the conservative 2-byte single-turn register.
uint16_t goalAddr(TableKind kind);
uint8_t goalLen(TableKind kind);

// Present-status block address / length for a table kind.
uint16_t statusAddr(TableKind kind);
uint8_t statusLen(TableKind kind);

// Present Position address / length for a table kind (single field).
uint16_t posAddr(TableKind kind);
uint8_t posLen(TableKind kind);

// Decode a present-position field (already read) into raw ticks. `buf` must
// hold at least posLen(kind) bytes. Legacy is unsigned 2B; V2 is signed 4B.
int32_t decodePosition(TableKind kind, const uint8_t* buf);

// Encode a goal position tick into `out` (little-endian) using the table's
// goal length. `out` must hold at least goalLen(kind) bytes. Returns the number
// of bytes written.
uint8_t encodeGoal(TableKind kind, uint16_t tick, uint8_t* out);

// Decode a present-status block (already read from the servo) into normalized
// fields. `block` must hold at least statusLen(kind) bytes. Returns false if
// kind is Unknown (caller should treat as a failed read).
bool decodeStatus(TableKind kind, const uint8_t* block, StatusFields& out);

// Convert a legacy Protocol 1.0 sign-magnitude 11-bit field (bit 10 = sign)
// into a signed value. Exposed for testing.
int32_t decodeLegacySignMag(uint16_t raw);

}  // namespace dxl
