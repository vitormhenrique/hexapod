#pragma once

// ===========================================================================
// Consistent Overhead Byte Stuffing (COBS).
//
// Encodes a buffer so the output contains no 0x00 bytes, letting 0x00 act as an
// unambiguous frame delimiter on the wire. Overhead is 1 byte per up-to-254
// bytes of input. Portable C++, no heap: caller supplies output buffers.
//
// Reference: Cheshire & Baker, "Consistent Overhead Byte Stuffing" (1999).
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace protocol {

// Worst-case encoded size for a payload of `n` bytes (excludes frame delimiters):
//   n + ceil(n / 254) + 1   (the +1 covers the n == 0 case and the leading code)
constexpr size_t cobsMaxEncodedLen(size_t n) { return n + (n / 254) + 1; }

// Encode `len` bytes from `src` into `dst`. `dst` must hold at least
// cobsMaxEncodedLen(len) bytes. Returns the number of bytes written, or 0 if
// the output buffer is too small. The output contains no 0x00 bytes.
size_t cobsEncode(const uint8_t* src, size_t len, uint8_t* dst, size_t dst_cap);

// Decode `len` COBS bytes from `src` into `dst`. `src` must NOT contain the
// 0x00 frame delimiters. `dst` must hold at least `len` bytes. Returns the
// number of bytes written, or 0 on malformed input (a zero in the stream or a
// code that runs past the end).
size_t cobsDecode(const uint8_t* src, size_t len, uint8_t* dst, size_t dst_cap);

}  // namespace protocol
