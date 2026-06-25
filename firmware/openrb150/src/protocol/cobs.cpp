#include "cobs.h"

namespace protocol {

size_t cobsEncode(const uint8_t* src, size_t len, uint8_t* dst, size_t dst_cap) {
  if (dst_cap < cobsMaxEncodedLen(len)) {
    return 0;
  }

  size_t read_idx = 0;
  size_t write_idx = 1;   // dst[0] reserved for the first code byte
  size_t code_idx = 0;    // index of the current code byte being filled
  uint8_t code = 1;       // distance to the next zero (1..255)

  while (read_idx < len) {
    if (src[read_idx] == 0) {
      // Close the current block: write the code, start a new block.
      dst[code_idx] = code;
      code_idx = write_idx++;
      code = 1;
    } else {
      dst[write_idx++] = src[read_idx];
      if (++code == 0xFF) {
        // Block reached the maximum run length; emit and start a fresh block.
        dst[code_idx] = code;
        code_idx = write_idx++;
        code = 1;
      }
    }
    ++read_idx;
  }

  // Finalize the trailing block's code byte.
  dst[code_idx] = code;
  return write_idx;
}

size_t cobsDecode(const uint8_t* src, size_t len, uint8_t* dst, size_t dst_cap) {
  size_t read_idx = 0;
  size_t write_idx = 0;

  while (read_idx < len) {
    uint8_t code = src[read_idx];
    if (code == 0) {
      // A literal zero must never appear inside COBS-encoded data.
      return 0;
    }
    if (read_idx + code > len) {
      // Code points past the end of the buffer: malformed.
      return 0;
    }
    ++read_idx;

    for (uint8_t i = 1; i < code; ++i) {
      if (write_idx >= dst_cap) {
        return 0;
      }
      dst[write_idx++] = src[read_idx++];
    }

    // A code < 0xFF implies an implicit zero, except for the final block.
    if (code != 0xFF && read_idx < len) {
      if (write_idx >= dst_cap) {
        return 0;
      }
      dst[write_idx++] = 0;
    }
  }

  return write_idx;
}

}  // namespace protocol
