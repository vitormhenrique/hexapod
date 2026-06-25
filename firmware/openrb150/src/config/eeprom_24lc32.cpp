#include "eeprom_24lc32.h"

namespace config {

namespace {
// Keep read/write chunks within the SAMD Wire buffer (256 bytes); 32 also keeps
// writes inside a single EEPROM page.
constexpr uint8_t kReadChunk = 32;
}  // namespace

bool Eeprom24LC32::isReady() {
  wire_.beginTransmission(addr_);
  return wire_.endTransmission() == 0;
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

    // Set the word address with a repeated start (no stop) before reading.
    wire_.beginTransmission(addr_);
    wire_.write(static_cast<uint8_t>((cur >> 8) & 0xFF));
    wire_.write(static_cast<uint8_t>(cur & 0xFF));
    if (wire_.endTransmission(false) != 0) {
      return false;
    }

    const uint8_t got =
        wire_.requestFrom(addr_, static_cast<uint8_t>(n), static_cast<uint8_t>(true));
    if (got != n) {
      return false;
    }
    for (uint16_t i = 0; i < n; ++i) {
      buf[off + i] = static_cast<uint8_t>(wire_.read());
    }
    off = static_cast<uint16_t>(off + n);
  }
  return true;
}

bool Eeprom24LC32::writePage(uint16_t addr, const uint8_t* buf, uint8_t len) {
  wire_.beginTransmission(addr_);
  wire_.write(static_cast<uint8_t>((addr >> 8) & 0xFF));
  wire_.write(static_cast<uint8_t>(addr & 0xFF));
  for (uint8_t i = 0; i < len; ++i) {
    wire_.write(buf[i]);
  }
  if (wire_.endTransmission() != 0) {
    return false;
  }
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
