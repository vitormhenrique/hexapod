// Native (host) unit tests for the portable transactional config store.
// Uses a RAM-backed fake ConfigBackend; no Arduino/Wire dependencies.
//
// Run with: pio test -e native

#include <string.h>
#include <unity.h>

#include "../../src/config/config_store.h"

using namespace config;

namespace {

// 4 KB RAM stand-in for the 24LC32, with optional fault injection to model a
// power loss partway through a commit.
class FakeEeprom : public ConfigBackend {
 public:
  FakeEeprom() { memset(mem_, 0xFF, sizeof(mem_)); }

  bool read(uint16_t addr, uint8_t* buf, uint16_t len) override {
    if (static_cast<uint32_t>(addr) + len > sizeof(mem_)) return false;
    memcpy(buf, &mem_[addr], len);
    ++reads_;
    return true;
  }

  bool write(uint16_t addr, const uint8_t* buf, uint16_t len) override {
    if (static_cast<uint32_t>(addr) + len > sizeof(mem_)) return false;
    if (fail_after_writes_ >= 0 && writes_ >= fail_after_writes_) {
      return false;  // simulate power loss / write failure
    }
    memcpy(&mem_[addr], buf, len);
    ++writes_;
    return true;
  }

  void corruptByte(uint16_t addr) { mem_[addr] ^= 0xFF; }
  void failAfter(int n) { fail_after_writes_ = n; }
  int writes() const { return writes_; }

 private:
  uint8_t mem_[4096];
  int writes_ = 0;
  int reads_ = 0;
  int fail_after_writes_ = -1;  // -1 = never fail
};

void fillPattern(uint8_t* p, uint16_t len, uint8_t seed) {
  for (uint16_t i = 0; i < len; ++i) p[i] = static_cast<uint8_t>(seed + i);
}

}  // namespace

void test_blank_eeprom_load_fails() {
  FakeEeprom mem;
  ConfigStore store(mem);
  uint8_t out[64];
  uint16_t out_len = 0xFFFF;
  TEST_ASSERT_FALSE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(0, out_len);
}

void test_commit_then_load_roundtrip() {
  FakeEeprom mem;
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

void test_commits_alternate_slots_and_newest_wins() {
  FakeEeprom mem;
  ConfigStore store(mem);

  uint8_t a[40];
  fillPattern(a, sizeof(a), 0x01);
  uint8_t b[50];
  fillPattern(b, sizeof(b), 0x80);

  TEST_ASSERT_TRUE(store.commit(a, sizeof(a)));  // -> slot 0, seq 1
  TEST_ASSERT_TRUE(store.commit(b, sizeof(b)));  // -> slot 1, seq 2

  // Both slots should now be valid (A/B), with slot 1 newest.
  SlotStatus st[kSlotCount];
  store.inspect(st);
  TEST_ASSERT_TRUE(st[0].valid);
  TEST_ASSERT_TRUE(st[1].valid);
  TEST_ASSERT_EQUAL_UINT32(1, st[0].sequence);
  TEST_ASSERT_EQUAL_UINT32(2, st[1].sequence);

  uint8_t out[50];
  uint16_t out_len = 0;
  TEST_ASSERT_TRUE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(sizeof(b), out_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(b, out, sizeof(b));
}

void test_corrupt_newest_falls_back_to_older() {
  FakeEeprom mem;
  ConfigStore store(mem);

  uint8_t a[40];
  fillPattern(a, sizeof(a), 0x01);
  uint8_t b[40];
  fillPattern(b, sizeof(b), 0x80);

  TEST_ASSERT_TRUE(store.commit(a, sizeof(a)));  // slot 0, seq 1
  TEST_ASSERT_TRUE(store.commit(b, sizeof(b)));  // slot 1, seq 2 (newest)

  // Corrupt a payload byte of the newest slot (slot 1 at kSlotSize+header).
  mem.corruptByte(static_cast<uint16_t>(kSlotAddr[1] + kHeaderSize + 5));

  // Load must now fall back to the older, still-valid slot 0 (payload a).
  uint8_t out[40];
  uint16_t out_len = 0;
  TEST_ASSERT_TRUE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(sizeof(a), out_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(a, out, sizeof(a));
}

void test_corrupt_header_rejected() {
  FakeEeprom mem;
  ConfigStore store(mem);
  uint8_t a[32];
  fillPattern(a, sizeof(a), 0x22);
  TEST_ASSERT_TRUE(store.commit(a, sizeof(a)));  // slot 0

  // Flip a header byte (the magic) -> slot invalid.
  mem.corruptByte(kSlotAddr[0]);

  SlotStatus st[kSlotCount];
  store.inspect(st);
  TEST_ASSERT_FALSE(st[0].valid);

  uint8_t out[32];
  uint16_t out_len = 0;
  TEST_ASSERT_FALSE(store.load(out, sizeof(out), out_len));
}

void test_power_loss_during_commit_keeps_previous() {
  FakeEeprom mem;
  ConfigStore store(mem);

  uint8_t good[40];
  fillPattern(good, sizeof(good), 0x33);
  TEST_ASSERT_TRUE(store.commit(good, sizeof(good)));  // slot 0, seq 1

  // Next commit writes payload (write #2) then header (write #3). Fail before
  // the header is written so the new slot never becomes valid.
  const int writes_before = mem.writes();
  mem.failAfter(writes_before + 1);  // allow payload write, block header write

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
  FakeEeprom mem;
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
  FakeEeprom mem;
  ConfigStore store(mem);
  TEST_ASSERT_TRUE(store.commit(nullptr, 0));
  uint8_t out[8];
  uint16_t out_len = 0xFFFF;
  TEST_ASSERT_TRUE(store.load(out, sizeof(out), out_len));
  TEST_ASSERT_EQUAL_UINT16(0, out_len);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_blank_eeprom_load_fails);
  RUN_TEST(test_commit_then_load_roundtrip);
  RUN_TEST(test_commits_alternate_slots_and_newest_wins);
  RUN_TEST(test_corrupt_newest_falls_back_to_older);
  RUN_TEST(test_corrupt_header_rejected);
  RUN_TEST(test_power_loss_during_commit_keeps_previous);
  RUN_TEST(test_commit_rejects_oversize_payload);
  RUN_TEST(test_header_serialize_roundtrip);
  RUN_TEST(test_empty_payload_commit_load);
  return UNITY_END();
}
