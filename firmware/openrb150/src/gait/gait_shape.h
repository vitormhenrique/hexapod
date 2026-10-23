#pragma once

// ===========================================================================
// Live gait-shape tunables (portable, no Arduino deps).
//
// These are the operator-facing numbers that decide how walking *looks*:
// stroke length, swing clearance, turn rate, stance width, cadence and swing
// profile. They are streamed as ABSOLUTE values from the handset over the
// ChannelPack parameter uplink (CH12 id / CH13-14 value, see
// lib/ChannelPack/ChannelPack.h) and may later be set by a USB host. The
// firmware keeps the compiled defaults below until a source provides a value,
// clamps every value to the table, applies the result through the gait
// engine's parameter filter, and echoes a CRC of the active set on the CRSF
// Hexapod status downlink so the handset can show "synced".
//
// The id numbers and the CRC layout are shared with the ESP32 handset
// (src/telemetry/hexapod_gait_params.h). Never renumber or reorder; append.
// ===========================================================================

#include <stdint.h>

namespace gait {

enum class TuneId : uint8_t {
  None = 0,
  StrideMm = 1,          // horizontal stroke at full stick, mm
  StepHeightMm = 2,      // swing clearance, mm
  YawStrokeDeg = 3,      // body yaw per gait cycle at full yaw stick, deg
  StanceRadiusMm = 4,    // home foot distance from the coxa axis, mm
  StepPeriodFastMs = 5,  // Mark III step period at speed = 1 (Pot1 max), ms
  StepPeriodSlowMs = 6,  // Mark III step period at speed = 0 (Pot1 min), ms
  HalfLiftPct = 7,       // swing mid-keyframe height, % of step height
  Count = 8,
  SaveRequest = 63,      // value = handset save sequence (persist GaitDefaults)
};

constexpr uint8_t kNumTuneIds = static_cast<uint8_t>(TuneId::Count);

struct TuneLimit {
  uint16_t min;
  uint16_t max;
  uint16_t def;
};

// Indexed by TuneId; slot 0 is unused. Stride / step maxima mirror
// config::kMaxGaitStrideMm / kMaxGaitStepMm. The handset table must be
// identical so both sides clamp -- and CRC -- the same numbers.
constexpr TuneLimit kTuneLimits[kNumTuneIds] = {
    {0, 0, 0},         // None
    {0, 140, 60},      // StrideMm
    {0, 80, 30},       // StepHeightMm
    {0, 45, 20},       // YawStrokeDeg (Phoenix Mark III used 32)
    {100, 165, 127},   // StanceRadiusMm (measured CAD home 126.75)
    {30, 250, 60},     // StepPeriodFastMs (Phoenix: 35..50)
    {30, 400, 120},    // StepPeriodSlowMs
    {0, 100, 50},      // HalfLiftPct (Phoenix HalfLiftHeight 3 -> 50 %)
};

inline bool tuneIdValid(uint8_t id) {
  return id >= 1 && id < kNumTuneIds;
}

inline uint16_t clampTune(TuneId id, uint16_t value) {
  const TuneLimit& l = kTuneLimits[static_cast<uint8_t>(id)];
  if (value < l.min) return l.min;
  if (value > l.max) return l.max;
  return value;
}

// The complete live set in native units. Default-constructed = compiled
// defaults, which is exactly what the gait engine ran before the uplink
// existed (60 / 30 / 13.4-deg-equivalent -> 20 / 126.75 -> 127 / 60 / 120 / 50).
struct GaitShape {
  uint16_t value[kNumTuneIds] = {
      0,
      kTuneLimits[1].def, kTuneLimits[2].def, kTuneLimits[3].def,
      kTuneLimits[4].def, kTuneLimits[5].def, kTuneLimits[6].def,
      kTuneLimits[7].def,
  };

  uint16_t get(TuneId id) const { return value[static_cast<uint8_t>(id)]; }
  void set(TuneId id, uint16_t v) {
    value[static_cast<uint8_t>(id)] = clampTune(id, v);
  }
  bool operator==(const GaitShape& o) const {
    for (uint8_t i = 1; i < kNumTuneIds; ++i) {
      if (value[i] != o.value[i]) return false;
    }
    return true;
  }
  bool operator!=(const GaitShape& o) const { return !(*this == o); }
};

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout)
// over the values of ids 1..Count-1 in id order, each as two little-endian
// bytes. Same algorithm as protocol/crc16 and the handset mirror; kept inline
// here so the gait layer stays free of protocol dependencies.
inline uint16_t gaitShapeCrc16(const GaitShape& shape) {
  uint16_t crc = 0xFFFFu;
  for (uint8_t i = 1; i < kNumTuneIds; ++i) {
    const uint8_t bytes[2] = {
        static_cast<uint8_t>(shape.value[i] & 0xFFu),
        static_cast<uint8_t>(shape.value[i] >> 8),
    };
    for (uint8_t b = 0; b < 2; ++b) {
      crc ^= static_cast<uint16_t>(bytes[b]) << 8;
      for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                              : static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

}  // namespace gait
