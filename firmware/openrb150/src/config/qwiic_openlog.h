#pragma once

#include <stdint.h>

#include "config_store.h"
#include "../sensors/i2c_bus.h"

namespace config {

class QwiicOpenLog {
 public:
  static constexpr uint8_t kDefaultAddress = 0x2A;
  static constexpr uint8_t kJumperAddress = 0x29;
  static constexpr uint8_t kMaxWriteBytes = 30;

  explicit QwiicOpenLog(i2c::I2cBus& bus) : bus_(bus) {}

  bool begin(uint8_t address);
  bool ready() const { return ready_; }
  uint8_t address() const { return address_; }

  bool fileSize(const char* name, uint32_t& size, bool& exists);
  bool readAt(const char* name, uint32_t offset, uint8_t* out, uint16_t len);
  bool append(const char* name, const uint8_t* data, uint8_t len);
  bool sync();

 private:
  enum Register : uint8_t {
    kStatus = 0x01,
    kReadFile = 0x09,
    kStartPosition = 0x0A,
    kOpenFile = 0x0B,
    kWriteFile = 0x0C,
    kFileSize = 0x0D,
    kSyncFile = 0x11,
  };

  bool command(uint8_t reg, const uint8_t* data, uint8_t len);
  bool busRead(uint8_t* data, uint8_t len);
  bool selectAppendFile(const char* name);
  bool beginRead(const char* name);
  bool fillReadCache();
  bool readAtOnce(const char* name, uint32_t offset, uint8_t* out,
                  uint16_t len);
  bool copyName(char out[13], const char* name);

  i2c::I2cBus& bus_;
  uint8_t address_ = kDefaultAddress;
  bool ready_ = false;
  char append_file_[13] = {0};
  char read_file_[13] = {0};
  uint32_t read_file_size_ = 0;
  uint32_t read_source_offset_ = 0;
  uint32_t read_client_offset_ = 0;
  uint8_t read_cache_[32] = {0};
  uint8_t read_cache_pos_ = 0;
  uint8_t read_cache_len_ = 0;
};

class QwiicConfigFile : public ConfigFile {
 public:
  static constexpr const char* kFileName = "CONFIG.TXT";

  explicit QwiicConfigFile(QwiicOpenLog& openlog) : openlog_(openlog) {}

  bool size(uint32_t& out) override;
  bool read(uint32_t offset, uint8_t* buf, uint16_t len) override;
  bool append(const uint8_t* buf, uint16_t len) override;
  bool sync() override { return openlog_.sync(); }

 private:
  static bool decodeNibble(uint8_t c, uint8_t& nibble);

  QwiicOpenLog& openlog_;
};

}  // namespace config