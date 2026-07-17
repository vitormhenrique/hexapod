#include "eeprom_24lc32.h"

namespace config {

namespace {
// Keep reads aligned with the EEPROM page size and bound each transaction.
constexpr uint8_t kReadChunk = 32;
}  // namespace

bool Eeprom24LC32::isReady() {
  return bus_.write(addr_, nullptr, 0);
}

bool Eeprom24LC32::waitWriteComplete() {
  for (uint8_t i = 0; i < kWritePollTries; ++i) {
    if (isReady()) {
      return true;
    }
    delay(1);
  }
  return false;
}

bool Eeprom24LC32::read(uint16_t addr, uint8_t* buf, uint16_t len) {
  if (static_cast<uint32_t>(addr) + len > kCapacity) {
    return false;
  }
  uint16_t off = 0;
  while (off < len) {
    const uint16_t cur = static_cast<uint16_t>(addr + off);
    uint16_t n = len - off;
    if (n > kReadChunk) {
      n = kReadChunk;
    }

    const uint8_t address[2] = {
        static_cast<uint8_t>((cur >> 8) & 0xFF),
        static_cast<uint8_t>(cur & 0xFF),
    };
    if (!bus_.writeRead(addr_, address, sizeof(address), buf + off,
                        static_cast<uint8_t>(n))) return false;
    off = static_cast<uint16_t>(off + n);
  }
  return true;
}

bool Eeprom24LC32::writePage(uint16_t addr, const uint8_t* buf, uint8_t len) {
  uint8_t data[kPageSize + 2];
  data[0] = static_cast<uint8_t>((addr >> 8) & 0xFF);
  data[1] = static_cast<uint8_t>(addr & 0xFF);
  for (uint8_t i = 0; i < len; ++i) {
    data[i + 2] = buf[i];
  }
  if (!bus_.write(addr_, data, static_cast<uint8_t>(len + 2))) return false;
  return waitWriteComplete();
}

bool Eeprom24LC32::write(uint16_t addr, const uint8_t* buf, uint16_t len) {
  if (static_cast<uint32_t>(addr) + len > kCapacity) {
    return false;
  }
  uint16_t off = 0;
  while (off < len) {
    const uint16_t cur = static_cast<uint16_t>(addr + off);
    // Bytes left until the next 32-byte page boundary.
    const uint16_t page_room = kPageSize - (cur % kPageSize);
    uint16_t n = len - off;
    if (n > page_room) {
      n = page_room;
    }
    if (!writePage(cur, buf + off, static_cast<uint8_t>(n))) {
      return false;
    }
    off = static_cast<uint16_t>(off + n);
  }
  return true;
}

}  // namespace config
