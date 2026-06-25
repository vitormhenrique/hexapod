#pragma once

// ===========================================================================
// CRC-16/CCITT-FALSE.
//
//   poly   = 0x1021
//   init   = 0xFFFF
//   refin  = false, refout = false
//   xorout = 0x0000
//
// Used to protect each protocol frame (computed over header + payload, before
// COBS encoding). Portable C++: no Arduino/STL deps so it builds on the MCU and
// in native host tests against the shared golden vectors.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace protocol {

constexpr uint16_t kCrc16Init = 0xFFFF;

// Incremental update so a CRC can be computed across several buffers.
uint16_t crc16Update(uint16_t crc, const uint8_t* data, size_t len);

// One-shot CRC-16/CCITT-FALSE over a single buffer.
uint16_t crc16(const uint8_t* data, size_t len);

}  // namespace protocol
