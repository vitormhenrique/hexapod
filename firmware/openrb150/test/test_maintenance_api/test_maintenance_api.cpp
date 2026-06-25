// Native (host) Unity tests for the maintenance lock command group
// (ENTER / EXIT / HEARTBEAT maintenance) handled by MaintenanceApi and routed
// through the protocol api dispatcher.
//
// Run with:  pio test -e native -f test_maintenance_api

#include <string.h>

#include <unity.h>

#include "protocol/api.h"
#include "protocol/framing.h"
#include "protocol/maintenance_api.h"

using namespace protocol;

namespace {

api::DeviceInfo makeInfo() {
  api::DeviceInfo info;
  info.fw_major = 0;
  info.fw_minor = 1;
  info.fw_patch = 0;
  info.feature_bits = 0;
  const char name[] = "OpenRB150-Hex";
  memset(info.device_name, 0, sizeof(info.device_name));
  memcpy(info.device_name, name, sizeof(name) - 1);
  return info;
}

api::StatusSnapshot makeStatus() {
  api::StatusSnapshot st;
  st.uptime_ms = 1000;
  st.state = 2;
  st.battery_mv = 11800;
  return st;
}

size_t buildRequest(uint8_t msg_id, uint16_t seq, const uint8_t* payload,
                    uint16_t payload_len, uint8_t* out, size_t cap) {
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = msg_id;
  h.seq = seq;
  h.payload_len = payload_len;
  return encodeFrame(h, payload, out, cap);
}

DecodeStatus runMaint(MaintenanceApi& maint, uint8_t msg_id, uint16_t seq,
                      const uint8_t* payload, uint16_t payload_len,
                      Header* resp_h, uint8_t* resp_payload, size_t* resp_len) {
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(msg_id, seq, payload, payload_len, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n =
      api::handleRequest(req + 1, req_n - 2, info, st, resp, sizeof(resp),
                         nullptr, nullptr, nullptr, nullptr, &maint);
  TEST_ASSERT_TRUE(resp_n > 0);
  return decodeFrameBody(resp + 1, resp_n - 2, resp_h, resp_payload,
                         kMaxPayload, resp_len);
}

uint32_t readU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
void putU32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

// Enter at a safe state and return the granted token.
uint32_t doEnter(MaintenanceApi& m, uint32_t now, uint8_t state = 2) {
  m.setNow(now);
  m.setLiveState(state);
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMaint(m, maintmsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT(6, n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintResult::Ok), pl[0]);
  return readU32(&pl[2]);
}

}  // namespace

void test_enter_grants_token_in_safe_state() {
  MaintenanceApi m;
  m.reset();
  const uint32_t tok = doEnter(m, 1000, 2);
  TEST_ASSERT_TRUE(tok != 0);
  TEST_ASSERT_TRUE(m.lockHeld(1000));
  TEST_ASSERT_EQUAL_UINT32(tok, m.token());
}

void test_enter_rejected_in_unsafe_state() {
  MaintenanceApi m;
  m.reset();
  m.setNow(1000);
  m.setLiveState(12);  // Estop
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMaint(m, maintmsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT(2, n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintResult::Rejected), pl[0]);
  TEST_ASSERT_FALSE(m.lockHeld(1000));
}

void test_enter_busy_when_already_held() {
  MaintenanceApi m;
  m.reset();
  doEnter(m, 1000, 2);
  m.setNow(1100);
  m.setLiveState(2);
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMaint(m, maintmsg::kEnter, 2, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintResult::Busy), pl[0]);
}

void test_heartbeat_refreshes_ttl() {
  MaintenanceApi m;
  m.reset();
  const uint32_t tok = doEnter(m, 1000, 2);  // ttl default 1000
  TEST_ASSERT_TRUE(m.lockHeld(1900));
  // Heartbeat at 1900 refreshes the window to [1900, 2900].
  m.setNow(1900);
  m.setLiveState(2);
  uint8_t tb[4];
  putU32(tb, tok);
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMaint(m, maintmsg::kHeartbeat, 3, tb, 4, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintResult::Ok), pl[0]);
  TEST_ASSERT_TRUE(m.lockHeld(2800));
  TEST_ASSERT_FALSE(m.lockHeld(3000));
}

void test_heartbeat_bad_token() {
  MaintenanceApi m;
  m.reset();
  const uint32_t tok = doEnter(m, 1000, 2);
  m.setNow(1100);
  m.setLiveState(2);
  uint8_t tb[4];
  putU32(tb, tok + 7);  // wrong token
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMaint(m, maintmsg::kHeartbeat, 3, tb, 4, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintResult::BadToken), pl[0]);
}

void test_exit_releases_with_matching_token() {
  MaintenanceApi m;
  m.reset();
  const uint32_t tok = doEnter(m, 1000, 2);
  m.setNow(1100);
  m.setLiveState(2);
  uint8_t tb[4];
  putU32(tb, tok);
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMaint(m, maintmsg::kExit, 4, tb, 4, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintResult::Ok), pl[0]);
  TEST_ASSERT_FALSE(m.lockHeld(1100));
  // A second EXIT with the same (now released) token is a bad token.
  runMaint(m, maintmsg::kExit, 5, tb, 4, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintResult::BadToken), pl[0]);
}

void test_exit_bad_token_keeps_lock() {
  MaintenanceApi m;
  m.reset();
  const uint32_t tok = doEnter(m, 1000, 2);
  m.setNow(1100);
  m.setLiveState(2);
  uint8_t tb[4];
  putU32(tb, tok + 1);
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMaint(m, maintmsg::kExit, 4, tb, 4, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintResult::BadToken), pl[0]);
  TEST_ASSERT_TRUE(m.lockHeld(1100));
}

void test_ttl_expiry_allows_new_enter_with_new_token() {
  MaintenanceApi m;
  m.reset();
  const uint32_t tok1 = doEnter(m, 1000, 2);
  // Past the TTL window the lock has lapsed; ENTER grants a fresh token.
  const uint32_t tok2 = doEnter(m, 3000, 2);
  TEST_ASSERT_TRUE(tok2 != 0);
  TEST_ASSERT_TRUE(tok2 != tok1);
}

void test_revoke_releases_lock() {
  MaintenanceApi m;
  m.reset();
  doEnter(m, 1000, 2);
  TEST_ASSERT_TRUE(m.lockHeld(1000));
  m.revoke();
  TEST_ASSERT_FALSE(m.lockHeld(1000));
}

void test_bad_request_short_payload() {
  MaintenanceApi m;
  m.reset();
  doEnter(m, 1000, 2);
  m.setNow(1100);
  m.setLiveState(2);
  uint8_t shrt[2] = {1, 2};
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runMaint(m, maintmsg::kExit, 4, shrt, 2, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MaintResult::BadRequest), pl[0]);
  TEST_ASSERT_TRUE(h.flags & api::flag::kError);
  TEST_ASSERT_TRUE(m.lockHeld(1100));
}

void test_unknown_without_maint_is_error() {
  uint8_t req[kMaxWireFrame];
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = maintmsg::kEnter;
  h.seq = 9;
  h.payload_len = 0;
  const size_t req_n = encodeFrame(h, nullptr, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n =
      api::handleRequest(req + 1, req_n - 2, info, st, resp, sizeof(resp));
  TEST_ASSERT_TRUE(resp_n > 0);
  Header rh;
  uint8_t pl[kMaxPayload];
  size_t pn = 0;
  decodeFrameBody(resp + 1, resp_n - 2, &rh, pl, kMaxPayload, &pn);
  TEST_ASSERT_TRUE(rh.flags & api::flag::kError);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(api::Error::UnknownMsg), pl[0]);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_enter_grants_token_in_safe_state);
  RUN_TEST(test_enter_rejected_in_unsafe_state);
  RUN_TEST(test_enter_busy_when_already_held);
  RUN_TEST(test_heartbeat_refreshes_ttl);
  RUN_TEST(test_heartbeat_bad_token);
  RUN_TEST(test_exit_releases_with_matching_token);
  RUN_TEST(test_exit_bad_token_keeps_lock);
  RUN_TEST(test_ttl_expiry_allows_new_enter_with_new_token);
  RUN_TEST(test_revoke_releases_lock);
  RUN_TEST(test_bad_request_short_payload);
  RUN_TEST(test_unknown_without_maint_is_error);
  return UNITY_END();
}
