// DYNAMIXEL sync-write / status-read addressing + (de)serialization (portable).
// See dxl_sync.h. Host-tested via test/test_dxl_sync.

#include "dxl_sync.h"

namespace dxl {
namespace {

inline uint16_t rdU16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline int32_t rdI32(const uint8_t* p) {
  return static_cast<int32_t>(static_cast<uint32_t>(p[0]) |
                              (static_cast<uint32_t>(p[1]) << 8) |
                              (static_cast<uint32_t>(p[2]) << 16) |
                              (static_cast<uint32_t>(p[3]) << 24));
}

}  // namespace

uint16_t goalAddr(TableKind kind) {
  return (kind == TableKind::Mx28V2) ? kGoalAddrV2 : kGoalAddrLegacy;
}

uint8_t goalLen(TableKind kind) {
  return (kind == TableKind::Mx28V2) ? kGoalLenV2 : kGoalLenLegacy;
}

uint16_t statusAddr(TableKind kind) {
  return (kind == TableKind::Mx28V2) ? kStatusAddrV2 : kStatusAddrLegacy;
}

uint8_t statusLen(TableKind kind) {
  return (kind == TableKind::Mx28V2) ? kStatusLenV2 : kStatusLenLegacy;
}

uint16_t posAddr(TableKind kind) {
  return (kind == TableKind::Mx28V2) ? kPosAddrV2 : kPosAddrLegacy;
}

uint8_t posLen(TableKind kind) {
  return (kind == TableKind::Mx28V2) ? kPosLenV2 : kPosLenLegacy;
}

int32_t decodePosition(TableKind kind, const uint8_t* buf) {
  if (kind == TableKind::Mx28V2) {
    return rdI32(buf);
  }
  return static_cast<int32_t>(rdU16(buf));
}

uint8_t encodeGoal(TableKind kind, uint16_t tick, uint8_t* out) {
  const uint8_t len = goalLen(kind);
  out[0] = static_cast<uint8_t>(tick & 0xFF);
  out[1] = static_cast<uint8_t>((tick >> 8) & 0xFF);
  if (len == 4) {
    // Single-turn position never exceeds 0x0FFF, so the high bytes are 0.
    out[2] = 0;
    out[3] = 0;
  }
  return len;
}

int32_t decodeLegacySignMag(uint16_t raw) {
  const int32_t mag = static_cast<int32_t>(raw & 0x3FF);
  return (raw & 0x400) ? -mag : mag;
}

bool decodeStatus(TableKind kind, const uint8_t* block, StatusFields& out) {
  if (kind == TableKind::Mx28V2) {
    // 124 pwm(2) 126 load(2) 128 velocity(4) 132 position(4)
    // 136 vel_traj(4) 140 pos_traj(4) 144 voltage(2) 146 temp(1)
    out.load = static_cast<int16_t>(rdU16(block + 2));   // signed 0.1%
    out.velocity = rdI32(block + 4);                     // signed
    out.position = rdI32(block + 8);                     // raw ticks
    out.voltage_mv = static_cast<uint16_t>(rdU16(block + 20) * 100);
    out.temperature_c = static_cast<int8_t>(block[22]);
    return true;
  }
  if (kind == TableKind::Mx28Legacy) {
    // 36 position(2) 38 speed(2) 40 load(2) 42 voltage(1) 43 temp(1)
    out.position = static_cast<int32_t>(rdU16(block + 0));
    out.velocity = decodeLegacySignMag(rdU16(block + 2));
    out.load = decodeLegacySignMag(rdU16(block + 4));
    out.voltage_mv = static_cast<uint16_t>(block[6] * 100);
    out.temperature_c = static_cast<int8_t>(block[7]);
    return true;
  }
  return false;
}

}  // namespace dxl
