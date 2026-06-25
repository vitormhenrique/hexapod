#include "crc16.h"

namespace protocol {

uint16_t crc16Update(uint16_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

uint16_t crc16(const uint8_t* data, size_t len) {
  return crc16Update(kCrc16Init, data, len);
}

}  // namespace protocol
