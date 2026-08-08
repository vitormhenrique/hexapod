#include "qwiic_openlog.h"

#include <string.h>

namespace config {

namespace {
constexpr uint8_t kStatusSdReady = 1u << 0;
constexpr uint32_t kOpenLogTimeoutUs = 100000;
constexpr char kHex[] = "0123456789ABCDEF";
}  // namespace

bool QwiicOpenLog::copyName(char out[13], const char* name) {
  if (name == nullptr) return false;
  const size_t len = strlen(name);
  if (len == 0 || len >= 13) return false;
  memcpy(out, name, len + 1);
  return true;
}

bool QwiicOpenLog::command(uint8_t reg, const uint8_t* data, uint8_t len) {
  if (len > 31) return false;
  uint8_t packet[32];
  packet[0] = reg;
  if (len > 0) {
    if (data == nullptr) return false;
    memcpy(&packet[1], data, len);
  }
    const uint32_t previous_timeout = bus_.transactionTimeoutUs();
    bus_.setTransactionTimeoutUs(kOpenLogTimeoutUs);
    const bool acknowledged =
        bus_.write(address_, packet, static_cast<uint8_t>(len + 1u));
    bus_.setTransactionTimeoutUs(previous_timeout);
#if defined(__SAMD21G18A__)
  // SparkFun's reference Qwiic OpenLog library intentionally ignores
  // endTransmission errors on SAMD21: clock-stretched SD commands can report a
  // false transport failure even though OpenLog accepted the command. Reads,
  // status bits, sync, and config readback remain the end-to-end verification.
  (void)acknowledged;
  return true;
#else
  return acknowledged;
#endif
}

bool QwiicOpenLog::busRead(uint8_t* data, uint8_t len) {
  TwoWire& wire = bus_.wire();
  if (wire.requestFrom(address_, static_cast<size_t>(len)) != len) return false;
  for (uint8_t index = 0; index < len; ++index) {
    const int value = wire.read();
    if (value < 0) return false;
    data[index] = static_cast<uint8_t>(value);
  }
  return true;
}

bool QwiicOpenLog::begin(uint8_t address) {
  address_ = address;
  ready_ = false;
  append_file_[0] = '\0';
  read_file_[0] = '\0';
  if (!command(kStatus, nullptr, 0)) return false;
  uint8_t status = 0;
  if (!busRead(&status, 1)) return false;
  ready_ = (status & kStatusSdReady) != 0;
  return ready_;
}

bool QwiicOpenLog::fileSize(const char* name, uint32_t& size, bool& exists) {
  size = 0;
  exists = false;
  if (!ready_ || name == nullptr) return false;
  const size_t name_len = strlen(name);
  if (name_len == 0 || name_len > 31) return false;
  if (!command(kFileSize, reinterpret_cast<const uint8_t*>(name),
               static_cast<uint8_t>(name_len))) {
    return false;
  }
  uint8_t raw[4];
  if (!busRead(raw, sizeof(raw))) return false;
  size = (static_cast<uint32_t>(raw[0]) << 24) |
         (static_cast<uint32_t>(raw[1]) << 16) |
         (static_cast<uint32_t>(raw[2]) << 8) |
         static_cast<uint32_t>(raw[3]);
  exists = size != 0xFFFFFFFFu;
  if (!exists) size = 0;
  append_file_[0] = '\0';
  read_file_[0] = '\0';
  return true;
}

bool QwiicOpenLog::beginRead(const char* name) {
  uint32_t size = 0;
  bool exists = false;
  if (!fileSize(name, size, exists) || !exists) return false;

  const uint8_t zero = 0;
  if (!command(kStartPosition, &zero, 1)) return false;
  const size_t name_len = strlen(name);
  if (!command(kReadFile, reinterpret_cast<const uint8_t*>(name),
               static_cast<uint8_t>(name_len))) {
    return false;
  }
  if (!copyName(read_file_, name)) return false;
  append_file_[0] = '\0';
  read_file_size_ = size;
  read_source_offset_ = 0;
  read_client_offset_ = 0;
  read_cache_pos_ = 0;
  read_cache_len_ = 0;
  return true;
}

bool QwiicOpenLog::fillReadCache() {
  if (read_source_offset_ >= read_file_size_) return false;
  uint32_t remaining = read_file_size_ - read_source_offset_;
  read_cache_len_ = remaining < sizeof(read_cache_)
                        ? static_cast<uint8_t>(remaining)
                        : static_cast<uint8_t>(sizeof(read_cache_));
  if (!busRead(read_cache_, read_cache_len_)) return false;
  read_source_offset_ += read_cache_len_;
  read_cache_pos_ = 0;
  return true;
}

bool QwiicOpenLog::readAt(const char* name, uint32_t offset, uint8_t* out,
                          uint16_t len) {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (readAtOnce(name, offset, out, len)) return true;
    // A failed request may leave OpenLog's sequential response cursor at an
    // unknown point. Force beginRead() to reopen from zero before retrying the
    // complete logical read.
    read_file_[0] = '\0';
    read_cache_pos_ = 0;
    read_cache_len_ = 0;
  }
  return false;
}

bool QwiicOpenLog::readAtOnce(const char* name, uint32_t offset, uint8_t* out,
                              uint16_t len) {
  if (!ready_ || (len > 0 && out == nullptr)) return false;
  if (strcmp(read_file_, name) != 0 || offset < read_client_offset_) {
    if (!beginRead(name)) return false;
  }

  while (read_client_offset_ < offset) {
    if (read_cache_pos_ >= read_cache_len_ && !fillReadCache()) return false;
    const uint32_t skip_needed = offset - read_client_offset_;
    const uint8_t cached = static_cast<uint8_t>(read_cache_len_ - read_cache_pos_);
    const uint8_t skip = skip_needed < cached
                             ? static_cast<uint8_t>(skip_needed)
                             : cached;
    read_cache_pos_ = static_cast<uint8_t>(read_cache_pos_ + skip);
    read_client_offset_ += skip;
  }

  uint16_t copied = 0;
  while (copied < len) {
    if (read_cache_pos_ >= read_cache_len_ && !fillReadCache()) return false;
    const uint8_t cached = static_cast<uint8_t>(read_cache_len_ - read_cache_pos_);
    const uint16_t needed = static_cast<uint16_t>(len - copied);
    const uint8_t take = needed < cached ? static_cast<uint8_t>(needed) : cached;
    memcpy(out + copied, &read_cache_[read_cache_pos_], take);
    copied = static_cast<uint16_t>(copied + take);
    read_cache_pos_ = static_cast<uint8_t>(read_cache_pos_ + take);
    read_client_offset_ += take;
  }
  return true;
}

bool QwiicOpenLog::selectAppendFile(const char* name) {
  if (strcmp(append_file_, name) == 0) return true;
  const size_t name_len = strlen(name);
  if (name_len == 0 || name_len > 31) return false;
  if (!command(kOpenFile, reinterpret_cast<const uint8_t*>(name),
               static_cast<uint8_t>(name_len))) {
    return false;
  }
  if (!copyName(append_file_, name)) return false;
  read_file_[0] = '\0';
  return true;
}

bool QwiicOpenLog::append(const char* name, const uint8_t* data, uint8_t len) {
  if (!ready_ || len > kMaxWriteBytes || (len > 0 && data == nullptr)) {
    return false;
  }
  if (!selectAppendFile(name)) return false;
  return len == 0 || command(kWriteFile, data, len);
}

bool QwiicOpenLog::sync() {
  return ready_ && command(kSyncFile, nullptr, 0);
}

bool QwiicConfigFile::size(uint32_t& out) {
  uint32_t encoded_size = 0;
  bool exists = false;
  if (!openlog_.fileSize(kFileName, encoded_size, exists)) return false;
  if (!exists) {
    out = 0;
    return true;
  }
  if ((encoded_size & 1u) != 0) return false;
  out = encoded_size / 2u;
  return true;
}

bool QwiicConfigFile::decodeNibble(uint8_t c, uint8_t& nibble) {
  if (c >= '0' && c <= '9') {
    nibble = static_cast<uint8_t>(c - '0');
    return true;
  }
  if (c >= 'A' && c <= 'F') {
    nibble = static_cast<uint8_t>(c - 'A' + 10);
    return true;
  }
  return false;
}

bool QwiicConfigFile::read(uint32_t offset, uint8_t* buf, uint16_t len) {
  if (len > 0 && buf == nullptr) return false;
  uint8_t encoded[30];
  uint16_t done = 0;
  while (done < len) {
    const uint16_t remaining = static_cast<uint16_t>(len - done);
    const uint8_t bytes = remaining < 15 ? static_cast<uint8_t>(remaining) : 15;
    const uint8_t chars = static_cast<uint8_t>(bytes * 2);
    const uint32_t encoded_offset = (offset + done) * 2u;
    if (!openlog_.readAt(kFileName, encoded_offset, encoded, chars)) return false;
    for (uint8_t i = 0; i < bytes; ++i) {
      uint8_t high = 0;
      uint8_t low = 0;
      // Preserve scan progress across a damaged/non-hex prefix. Invalid pairs
      // become zero bytes; record magic/header CRC/payload CRC still reject
      // them, allowing ConfigStore to resynchronize on a later valid record.
      (void)decodeNibble(encoded[i * 2], high);
      (void)decodeNibble(encoded[i * 2 + 1], low);
      buf[done + i] = static_cast<uint8_t>((high << 4) | low);
    }
    done = static_cast<uint16_t>(done + bytes);
  }
  return true;
}

bool QwiicConfigFile::append(const uint8_t* buf, uint16_t len) {
  if (len > 0 && buf == nullptr) return false;
  uint8_t encoded[30];
  uint16_t done = 0;
  while (done < len) {
    const uint16_t remaining = static_cast<uint16_t>(len - done);
    const uint8_t bytes = remaining < 15 ? static_cast<uint8_t>(remaining) : 15;
    for (uint8_t i = 0; i < bytes; ++i) {
      encoded[i * 2] = static_cast<uint8_t>(kHex[buf[done + i] >> 4]);
      encoded[i * 2 + 1] = static_cast<uint8_t>(kHex[buf[done + i] & 0x0F]);
    }
    if (!openlog_.append(kFileName, encoded, static_cast<uint8_t>(bytes * 2))) {
      return false;
    }
    done = static_cast<uint16_t>(done + bytes);
  }
  return true;
}

}  // namespace config