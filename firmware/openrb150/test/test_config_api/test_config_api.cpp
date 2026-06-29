// Native (host) unit tests for the portable config API (CFG_* handling).
//
// Drives ConfigApi directly through its block/validate/commit interface, with a
// fake persistence sink backed by the REAL transactional ConfigStore over a RAM
// EEPROM stand-in -- so the commit path exercises the actual A/B slot write +
// reload, proving "RAM shadow + EEPROM transaction" end to end.
//
// Run with: pio test -e native

#include <string.h>
#include <unity.h>

#include "../../src/config/config_api.h"
#include "../../src/config/config_schema.h"
#include "../../src/config/config_store.h"

using namespace config;

namespace {

// 4 KB RAM stand-in for the 24LC32.
class FakeEeprom : public ConfigBackend {
 public:
  FakeEeprom() { memset(mem_, 0xFF, sizeof(mem_)); }
  bool read(uint16_t addr, uint8_t* buf, uint16_t len) override {
    if (static_cast<uint32_t>(addr) + len > sizeof(mem_)) return false;
    memcpy(buf, &mem_[addr], len);
    return true;
  }
  bool write(uint16_t addr, const uint8_t* buf, uint16_t len) override {
    if (static_cast<uint32_t>(addr) + len > sizeof(mem_)) return false;
    if (!writable_) return false;
    memcpy(&mem_[addr], buf, len);
    ++writes_;
    return true;
  }
  void setWritable(bool w) { writable_ = w; }
  int writes() const { return writes_; }

 private:
  uint8_t mem_[4096];
  bool writable_ = true;
  int writes_ = 0;
};

// Persistence sink wrapping the real transactional store.
class StorePersistence : public ConfigPersistence {
 public:
  StorePersistence(ConfigStore& store, bool persistent)
      : store_(store), persistent_(persistent) {}
  bool commitPayload(const uint8_t* payload, uint16_t len) override {
    return store_.commit(payload, len);
  }
  bool persistent() const override { return persistent_; }
  void setPersistent(bool p) { persistent_ = p; }

 private:
  ConfigStore& store_;
  bool persistent_;
};

// Helper: drive a CFG_* command and capture the response.
struct Resp {
  bool handled;
  uint16_t len;
  uint8_t flags;
  uint8_t buf[256];
};
Resp call(ConfigApi& api, uint8_t msg_id, const uint8_t* req, uint16_t req_len) {
  Resp r{};
  r.handled = api.handle(msg_id, req, req_len, r.buf, sizeof(r.buf), &r.len,
                         &r.flags);
  return r;
}

constexpr uint8_t kErrorFlag = 0x02;

}  // namespace

void test_get_summary_reports_schema_and_persistent() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, /*persistent=*/true);
  ConfigApi api(persist);

  Resp r = call(api, cfgmsg::kGetSummary, nullptr, 0);
  TEST_ASSERT_TRUE(r.handled);
  TEST_ASSERT_EQUAL_UINT16(27, r.len);
  // schema_version
  TEST_ASSERT_EQUAL_UINT16(kSchemaVersion,
                           r.buf[0] | (r.buf[1] << 8));
  // payload_size
  TEST_ASSERT_EQUAL_UINT16(kConfigPayloadSize, r.buf[2] | (r.buf[3] << 8));
  // block_max
  TEST_ASSERT_EQUAL_UINT16(kCfgBlockMax, r.buf[4] | (r.buf[5] << 8));
  // flags: bit0 persistent, bit1 staged-valid (defaults are valid)
  TEST_ASSERT_EQUAL_UINT8(0x03, r.buf[6]);
  // robot_name (defaults to "HexNav")
  TEST_ASSERT_EQUAL_STRING("HexNav", reinterpret_cast<char*>(&r.buf[11]));
}

void test_get_summary_volatile_clears_persistent_bit() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, /*persistent=*/false);
  ConfigApi api(persist);

  Resp r = call(api, cfgmsg::kGetSummary, nullptr, 0);
  TEST_ASSERT_TRUE(r.handled);
  TEST_ASSERT_EQUAL_UINT8(0x02, r.buf[6]);  // staged-valid only, not persistent
}

void test_block_round_trip_reads_back_full_payload() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  // Read the whole serialized config out in kCfgBlockMax windows.
  uint8_t whole[kConfigPayloadSize];
  uint16_t off = 0;
  while (off < kConfigPayloadSize) {
    uint16_t len = kConfigPayloadSize - off;
    if (len > kCfgBlockMax) len = kCfgBlockMax;
    uint8_t req[4];
    req[0] = off & 0xFF;
    req[1] = off >> 8;
    req[2] = len & 0xFF;
    req[3] = len >> 8;
    Resp r = call(api, cfgmsg::kGetBlock, req, 4);
    TEST_ASSERT_TRUE(r.handled);
    TEST_ASSERT_EQUAL_UINT8(0, r.flags);
    TEST_ASSERT_EQUAL_UINT16(off, r.buf[0] | (r.buf[1] << 8));
    TEST_ASSERT_EQUAL_UINT16(len, r.buf[2] | (r.buf[3] << 8));
    memcpy(&whole[off], &r.buf[4], len);
    off += len;
  }

  // It must equal the serialized default config.
  RobotConfig def;
  defaultRobotConfig(def);
  uint8_t expected[kConfigPayloadSize];
  serializeRobotConfig(def, expected, sizeof(expected));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, whole, kConfigPayloadSize);
}

void test_get_block_rejects_out_of_range() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  uint8_t req[4];
  uint16_t off = kConfigPayloadSize - 4;
  uint16_t len = 16;  // runs past the end
  req[0] = off & 0xFF;
  req[1] = off >> 8;
  req[2] = len & 0xFF;
  req[3] = len >> 8;
  Resp r = call(api, cfgmsg::kGetBlock, req, 4);
  TEST_ASSERT_TRUE(r.handled);
  TEST_ASSERT_EQUAL_UINT8(kErrorFlag, r.flags);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgError::BadRange), r.buf[0]);
}

void test_get_block_rejects_short_request() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  uint8_t req[2] = {0, 0};
  Resp r = call(api, cfgmsg::kGetBlock, req, 2);
  TEST_ASSERT_TRUE(r.handled);
  TEST_ASSERT_EQUAL_UINT8(kErrorFlag, r.flags);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgError::BadRequest), r.buf[0]);
}

// Edit one field via SET_BLOCK and read it back via GET_BLOCK.
void test_set_block_edits_staging() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  // robot_name lives at offset 2 (after schema_version u16), 16 bytes.
  const uint16_t name_off = 2;
  uint8_t req[4 + 16];
  req[0] = name_off & 0xFF;
  req[1] = name_off >> 8;
  req[2] = 16;
  req[3] = 0;
  memset(&req[4], 0, 16);
  memcpy(&req[4], "Bench-7", 7);
  Resp w = call(api, cfgmsg::kSetBlock, req, sizeof(req));
  TEST_ASSERT_TRUE(w.handled);
  TEST_ASSERT_EQUAL_UINT8(0, w.flags);
  TEST_ASSERT_EQUAL_UINT16(name_off, w.buf[0] | (w.buf[1] << 8));
  TEST_ASSERT_EQUAL_UINT16(16, w.buf[2] | (w.buf[3] << 8));

  // Summary now reflects the staged name.
  Resp s = call(api, cfgmsg::kGetSummary, nullptr, 0);
  TEST_ASSERT_EQUAL_STRING("Bench-7", reinterpret_cast<char*>(&s.buf[11]));
}

void test_set_block_rejects_length_mismatch() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  uint8_t req[4 + 8];
  req[0] = 0;
  req[1] = 0;
  req[2] = 16;  // claims 16 but only 8 bytes follow
  req[3] = 0;
  memset(&req[4], 0, 8);
  Resp r = call(api, cfgmsg::kSetBlock, req, sizeof(req));
  TEST_ASSERT_TRUE(r.handled);
  TEST_ASSERT_EQUAL_UINT8(kErrorFlag, r.flags);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgError::BadRange), r.buf[0]);
}

void test_validate_ok_then_corrupt_then_fail() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  Resp ok = call(api, cfgmsg::kValidate, nullptr, 0);
  TEST_ASSERT_TRUE(ok.handled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgResult::Ok), ok.buf[0]);

  // Corrupt the schema_version bytes in staging via SET_BLOCK at offset 0.
  uint8_t req[4 + 2];
  req[0] = 0;
  req[1] = 0;
  req[2] = 2;
  req[3] = 0;
  req[4] = 0xEE;
  req[5] = 0xEE;
  call(api, cfgmsg::kSetBlock, req, sizeof(req));

  Resp bad = call(api, cfgmsg::kValidate, nullptr, 0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgResult::ValidationFailed),
                          bad.buf[0]);
}

// Full commit path: edit -> commit -> the bytes land in the store and reload.
void test_commit_persists_to_store() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  // Stage a recognizable name.
  uint8_t req[4 + 16];
  req[0] = 2;
  req[1] = 0;
  req[2] = 16;
  req[3] = 0;
  memset(&req[4], 0, 16);
  memcpy(&req[4], "Committed", 9);
  call(api, cfgmsg::kSetBlock, req, sizeof(req));

  Resp c = call(api, cfgmsg::kCommit, nullptr, 0);
  TEST_ASSERT_TRUE(c.handled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgResult::Ok), c.buf[0]);
  TEST_ASSERT_TRUE(mem.writes() > 0);

  // Reload from the store and confirm the committed name survived.
  uint8_t loaded[kConfigPayloadSize];
  uint16_t loaded_len = 0;
  TEST_ASSERT_TRUE(store.load(loaded, sizeof(loaded), loaded_len));
  TEST_ASSERT_EQUAL_UINT16(kConfigPayloadSize, loaded_len);
  RobotConfig got;
  TEST_ASSERT_TRUE(deserializeRobotConfig(loaded, loaded_len, got));
  TEST_ASSERT_EQUAL_STRING("Committed", got.robot_name);
}

void test_commit_rejected_when_volatile() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, /*persistent=*/false);
  ConfigApi api(persist);

  Resp c = call(api, cfgmsg::kCommit, nullptr, 0);
  TEST_ASSERT_TRUE(c.handled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgResult::Volatile), c.buf[0]);
  TEST_ASSERT_EQUAL_INT(0, mem.writes());  // nothing written
}

void test_commit_rejected_when_staging_invalid() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  // Corrupt a servo id to 0 (invalid). Servo block starts after:
  // schema(2)+name(16)+links(6)+legs(6*8=48) = 72; first servo id byte at 72.
  const uint16_t servo_id_off = 2 + 16 + 6 + kNumLegs * 8;
  uint8_t req[4 + 1];
  req[0] = servo_id_off & 0xFF;
  req[1] = servo_id_off >> 8;
  req[2] = 1;
  req[3] = 0;
  req[4] = 0;  // id 0 -> invalid
  call(api, cfgmsg::kSetBlock, req, sizeof(req));

  Resp c = call(api, cfgmsg::kCommit, nullptr, 0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgResult::ValidationFailed),
                          c.buf[0]);
  TEST_ASSERT_EQUAL_INT(0, mem.writes());
}

void test_commit_surfaces_store_failure() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  mem.setWritable(false);  // backend write will fail
  Resp c = call(api, cfgmsg::kCommit, nullptr, 0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgResult::CommitFailed),
                          c.buf[0]);
}

void test_reset_defaults_restores_staging() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  // Mutate the staged name.
  uint8_t req[4 + 16];
  req[0] = 2;
  req[1] = 0;
  req[2] = 16;
  req[3] = 0;
  memset(&req[4], 0, 16);
  memcpy(&req[4], "Scratch", 7);
  call(api, cfgmsg::kSetBlock, req, sizeof(req));

  Resp rd = call(api, cfgmsg::kResetDefaults, nullptr, 0);
  TEST_ASSERT_TRUE(rd.handled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgResult::Ok), rd.buf[0]);

  Resp s = call(api, cfgmsg::kGetSummary, nullptr, 0);
  TEST_ASSERT_EQUAL_STRING("HexNav", reinterpret_cast<char*>(&s.buf[11]));
}

void test_adopt_payload_seeds_shadow() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  RobotConfig cfg;
  defaultRobotConfig(cfg);
  strncpy(cfg.robot_name, "Booted", sizeof(cfg.robot_name) - 1);
  uint8_t payload[kConfigPayloadSize];
  uint16_t n = serializeRobotConfig(cfg, payload, sizeof(payload));

  TEST_ASSERT_TRUE(api.adoptPayload(payload, n));
  TEST_ASSERT_EQUAL_STRING("Booted", api.config().robot_name);

  Resp s = call(api, cfgmsg::kGetSummary, nullptr, 0);
  TEST_ASSERT_EQUAL_STRING("Booted", reinterpret_cast<char*>(&s.buf[11]));
}

void test_non_config_msg_not_handled() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  uint8_t buf[8];
  uint16_t len = 99;
  uint8_t flags = 99;
  // 0x03 == GET_STATUS, not a config message.
  TEST_ASSERT_FALSE(api.handle(0x03, nullptr, 0, buf, sizeof(buf), &len, &flags));
}

// lmt.7: the revision counter must advance on every change to the known-good
// shadow (adopt, commit, reset) so runtime consumers can re-apply, but NOT on
// staging-only edits (set_block) or reads (get_summary), which leave the shadow
// untouched.
void test_revision_bumps_only_on_shadow_change() {
  FakeEeprom mem;
  ConfigStore store(mem);
  StorePersistence persist(store, true);
  ConfigApi api(persist);

  // Ctor seeds defaults -> revision already advanced past 0.
  const uint32_t r0 = api.revision();
  TEST_ASSERT_NOT_EQUAL(0u, r0);

  // A read does not change the shadow.
  call(api, cfgmsg::kGetSummary, nullptr, 0);
  TEST_ASSERT_EQUAL_UINT32(r0, api.revision());

  // Staging an edit does not change the shadow either.
  uint8_t req[4 + 16];
  req[0] = 2;
  req[1] = 0;
  req[2] = 16;
  req[3] = 0;
  memset(&req[4], 0, 16);
  memcpy(&req[4], "Staged", 6);
  call(api, cfgmsg::kSetBlock, req, sizeof(req));
  TEST_ASSERT_EQUAL_UINT32(r0, api.revision());

  // Committing the staged config promotes it to the shadow -> bump.
  Resp c = call(api, cfgmsg::kCommit, nullptr, 0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CfgResult::Ok), c.buf[0]);
  const uint32_t r1 = api.revision();
  TEST_ASSERT_NOT_EQUAL(r0, r1);

  // Adopting a payload replaces the shadow -> bump.
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  strncpy(cfg.robot_name, "Adopted", sizeof(cfg.robot_name) - 1);
  uint8_t payload[kConfigPayloadSize];
  uint16_t n = serializeRobotConfig(cfg, payload, sizeof(payload));
  TEST_ASSERT_TRUE(api.adoptPayload(payload, n));
  const uint32_t r2 = api.revision();
  TEST_ASSERT_NOT_EQUAL(r1, r2);

  // A rejected adopt (bad payload) leaves the shadow + revision untouched.
  TEST_ASSERT_FALSE(api.adoptPayload(payload, n - 1));
  TEST_ASSERT_EQUAL_UINT32(r2, api.revision());

  // Reset-to-defaults rewrites the shadow -> bump.
  call(api, cfgmsg::kResetDefaults, nullptr, 0);
  TEST_ASSERT_NOT_EQUAL(r2, api.revision());
}

// lmt.7: the compiled default config must mark SensorPolling on (bit 2) so a
// freshly-defaulted or freshly-adopted config does not silently stop sensor
// polling when its feature defaults are applied.
void test_default_config_enables_sensor_polling_bit() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  TEST_ASSERT_EQUAL_UINT32(kFeatureDefaultMask, cfg.feature_defaults);
  TEST_ASSERT_TRUE((cfg.feature_defaults & (1u << 2)) != 0u);  // SensorPolling
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_get_summary_reports_schema_and_persistent);
  RUN_TEST(test_get_summary_volatile_clears_persistent_bit);
  RUN_TEST(test_block_round_trip_reads_back_full_payload);
  RUN_TEST(test_get_block_rejects_out_of_range);
  RUN_TEST(test_get_block_rejects_short_request);
  RUN_TEST(test_set_block_edits_staging);
  RUN_TEST(test_set_block_rejects_length_mismatch);
  RUN_TEST(test_validate_ok_then_corrupt_then_fail);
  RUN_TEST(test_commit_persists_to_store);
  RUN_TEST(test_commit_rejected_when_volatile);
  RUN_TEST(test_commit_rejected_when_staging_invalid);
  RUN_TEST(test_commit_surfaces_store_failure);
  RUN_TEST(test_reset_defaults_restores_staging);
  RUN_TEST(test_adopt_payload_seeds_shadow);
  RUN_TEST(test_non_config_msg_not_handled);
  RUN_TEST(test_revision_bumps_only_on_shadow_change);
  RUN_TEST(test_default_config_enables_sensor_polling_bit);
  return UNITY_END();
}
