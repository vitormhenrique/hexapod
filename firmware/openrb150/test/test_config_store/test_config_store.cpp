// Native (host) unit tests for the portable transactional config store.
// Uses a RAM-backed fake append file; no Arduino/Wire dependencies.
//
// Run with: pio test -e native

#include <string.h>
#include <unity.h>

#include "../../src/config/config_bootstrap.h"
#include "../../src/config/config_store.h"

using namespace config;

namespace {

class FakeFile : public ConfigFile {
 public:
  bool size(uint32_t& out) override {
    out = size_;
    return true;
  }

  bool read(uint32_t offset, uint8_t* buf, uint16_t len) override {
    if (offset + len > size_) return false;
    memcpy(buf, &mem_[offset], len);
    return true;
  }

  bool append(const uint8_t* buf, uint16_t len) override {
    if (size_ + len > sizeof(mem_)) return false;
    uint16_t accepted = len;
    if (fail_after_bytes_ >= 0) {
      if (static_cast<int32_t>(size_) >= fail_after_bytes_) return false;
      const uint32_t room = static_cast<uint32_t>(fail_after_bytes_) - size_;
      if (accepted > room) accepted = static_cast<uint16_t>(room);
    }
    memcpy(&mem_[size_], buf, accepted);
    size_ += accepted;
    return accepted == len;
  }

  bool sync() override { return !fail_sync_; }

  void corruptByte(uint32_t offset) { mem_[offset] ^= 0xFF; }
  bool appendRaw(const uint8_t* data, uint16_t len) { return append(data, len); }
  void failAfterBytes(int32_t count) { fail_after_bytes_ = count; }
  void failSync(bool fail) { fail_sync_ = fail; }
  uint32_t used() const { return size_; }

 private:
  uint8_t mem_[8192] = {0};
  uint32_t size_ = 0;
  int32_t fail_after_bytes_ = -1;
  bool fail_sync_ = false;
};

void fillPattern(uint8_t* p, uint16_t len, uint8_t seed) {
  for (uint16_t i = 0; i < len; ++i) p[i] = static_cast<uint8_t>(seed + i);
}

}  // namespace

void test_blank_file_load_fails() {
  FakeFile mem;
  ConfigStore store(mem);
  uint8_t out[64];
  uint16_t out_len = 0xFFFF;
  TEST_ASSERT_FALSE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(0, out_len);
}

void test_bootstrap_initializes_empty_file_with_defaults() {
  FakeFile file;
  ConfigStore store(file);
  uint8_t defaults[100];
  fillPattern(defaults, sizeof(defaults), 0x42);
  uint8_t out[100];
  uint16_t out_len = 0;

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(BootstrapResult::InitializedDefaults),
      static_cast<uint8_t>(loadOrInitializeConfig(
          file, store, defaults, sizeof(defaults), out, sizeof(out), out_len)));
  TEST_ASSERT_EQUAL_UINT16(sizeof(defaults), out_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(defaults, out, sizeof(defaults));
  TEST_ASSERT_GREATER_THAN(0, file.used());
}

void test_bootstrap_preserves_invalid_prefix_and_appends_defaults() {
  FakeFile file;
  const uint8_t corrupt[] = {'N', 'O', 'T', 'C', 'F', 'G'};
  TEST_ASSERT_TRUE(file.appendRaw(corrupt, sizeof(corrupt)));
  const uint32_t original_size = file.used();
  ConfigStore store(file);
  uint8_t defaults[32];
  fillPattern(defaults, sizeof(defaults), 0x55);
  uint8_t out[32];
  uint16_t out_len = 99;

    TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(BootstrapResult::RecoveredDefaults),
      static_cast<uint8_t>(loadOrInitializeConfig(
          file, store, defaults, sizeof(defaults), out, sizeof(out), out_len)));
    TEST_ASSERT_EQUAL_UINT16(sizeof(defaults), out_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(defaults, out, sizeof(defaults));
    TEST_ASSERT_GREATER_THAN(original_size, file.used());
}

void test_bootstrap_loads_valid_file_without_appending() {
  FakeFile file;
  ConfigStore store(file);
  uint8_t persisted[48];
  fillPattern(persisted, sizeof(persisted), 0x21);
  TEST_ASSERT_TRUE(store.commit(persisted, sizeof(persisted)));
  const uint32_t original_size = file.used();
  uint8_t defaults[48];
  fillPattern(defaults, sizeof(defaults), 0xA0);
  uint8_t out[48];
  uint16_t out_len = 0;

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(BootstrapResult::Loaded),
      static_cast<uint8_t>(loadOrInitializeConfig(
          file, store, defaults, sizeof(defaults), out, sizeof(out), out_len)));
  TEST_ASSERT_EQUAL_UINT16(sizeof(persisted), out_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(persisted, out, sizeof(persisted));
  TEST_ASSERT_EQUAL_UINT32(original_size, file.used());
}

void test_commit_then_load_roundtrip() {
  FakeFile mem;
  ConfigStore store(mem);
  uint8_t payload[100];
  fillPattern(payload, sizeof(payload), 0x10);

  TEST_ASSERT_TRUE(store.commit(payload, sizeof(payload)));

  uint8_t out[100];
  uint16_t out_len = 0;
  TEST_ASSERT_TRUE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(sizeof(payload), out_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, out, sizeof(payload));
}

void test_newest_record_wins() {
  FakeFile mem;
  ConfigStore store(mem);

  uint8_t a[40];
  fillPattern(a, sizeof(a), 0x01);
  uint8_t b[50];
  fillPattern(b, sizeof(b), 0x80);

  TEST_ASSERT_TRUE(store.commit(a, sizeof(a)));
  TEST_ASSERT_TRUE(store.commit(b, sizeof(b)));

  StoreStatus status;
  TEST_ASSERT_TRUE(store.inspect(status));
  TEST_ASSERT_EQUAL_UINT32(2, status.valid_records);
  TEST_ASSERT_EQUAL_UINT32(2, status.sequence);

  uint8_t out[50];
  uint16_t out_len = 0;
  TEST_ASSERT_TRUE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(sizeof(b), out_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(b, out, sizeof(b));
}

void test_corrupt_newest_falls_back_to_older() {
  FakeFile mem;
  ConfigStore store(mem);

  uint8_t a[40];
  fillPattern(a, sizeof(a), 0x01);
  uint8_t b[40];
  fillPattern(b, sizeof(b), 0x80);

  TEST_ASSERT_TRUE(store.commit(a, sizeof(a)));
  const uint32_t second_record = mem.used();
  TEST_ASSERT_TRUE(store.commit(b, sizeof(b)));
  mem.corruptByte(second_record + kHeaderSize + 5);

  // Load must now fall back to the older, still-valid slot 0 (payload a).
  uint8_t out[40];
  uint16_t out_len = 0;
  TEST_ASSERT_TRUE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(sizeof(a), out_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(a, out, sizeof(a));
}

void test_corrupt_header_rejected() {
  FakeFile mem;
  ConfigStore store(mem);
  uint8_t a[32];
  fillPattern(a, sizeof(a), 0x22);
  TEST_ASSERT_TRUE(store.commit(a, sizeof(a)));

  // Flip a header byte (the magic) -> slot invalid.
  mem.corruptByte(0);

  StoreStatus status;
  TEST_ASSERT_TRUE(store.inspect(status));
  TEST_ASSERT_FALSE(status.has_valid_record);

  uint8_t out[32];
  uint16_t out_len = 0;
  TEST_ASSERT_FALSE(store.load(out, sizeof(out), out_len));
}

void test_power_loss_during_commit_keeps_previous() {
  FakeFile mem;
  ConfigStore store(mem);

  uint8_t good[40];
  fillPattern(good, sizeof(good), 0x33);
  TEST_ASSERT_TRUE(store.commit(good, sizeof(good)));

  mem.failAfterBytes(static_cast<int32_t>(mem.used() + kHeaderSize + 10));

  uint8_t bad[40];
  fillPattern(bad, sizeof(bad), 0x99);
  TEST_ASSERT_FALSE(store.commit(bad, sizeof(bad)));

  // The original good config must still load.
  uint8_t out[40];
  uint16_t out_len = 0;
  TEST_ASSERT_TRUE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(sizeof(good), out_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(good, out, sizeof(good));
}

void test_commit_rejects_oversize_payload() {
  FakeFile mem;
  ConfigStore store(mem);
  static uint8_t big[kMaxPayload + 1];
  memset(big, 0x5A, sizeof(big));
  TEST_ASSERT_FALSE(store.commit(big, sizeof(big)));
}

void test_header_serialize_roundtrip() {
  SlotHeader h;
  h.magic = kConfigMagic;
  h.version = kConfigVersion;
  h.sequence = 0x12345678;
  h.length = 1234;
  h.payload_crc = 0xABCD;
  h.header_crc = 0x5566;

  uint8_t raw[kHeaderSize];
  serializeHeader(h, raw);
  SlotHeader d;
  deserializeHeader(raw, d);
  TEST_ASSERT_EQUAL_UINT32(h.magic, d.magic);
  TEST_ASSERT_EQUAL_UINT16(h.version, d.version);
  TEST_ASSERT_EQUAL_UINT32(h.sequence, d.sequence);
  TEST_ASSERT_EQUAL_UINT16(h.length, d.length);
  TEST_ASSERT_EQUAL_UINT16(h.payload_crc, d.payload_crc);
  TEST_ASSERT_EQUAL_UINT16(h.header_crc, d.header_crc);
}

void test_empty_payload_commit_load() {
  FakeFile mem;
  ConfigStore store(mem);
  TEST_ASSERT_TRUE(store.commit(nullptr, 0));
  uint8_t out[8];
  uint16_t out_len = 0xFFFF;
  TEST_ASSERT_TRUE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(0, out_len);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_blank_file_load_fails);
  RUN_TEST(test_bootstrap_initializes_empty_file_with_defaults);
  RUN_TEST(test_bootstrap_preserves_invalid_prefix_and_appends_defaults);
  RUN_TEST(test_bootstrap_loads_valid_file_without_appending);
  RUN_TEST(test_commit_then_load_roundtrip);
  RUN_TEST(test_newest_record_wins);
  RUN_TEST(test_corrupt_newest_falls_back_to_older);
  RUN_TEST(test_corrupt_header_rejected);
  RUN_TEST(test_power_loss_during_commit_keeps_previous);
  RUN_TEST(test_commit_rejects_oversize_payload);
  RUN_TEST(test_header_serialize_roundtrip);
  RUN_TEST(test_empty_payload_commit_load);
  return UNITY_END();
}
