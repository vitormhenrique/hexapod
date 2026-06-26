// Native (host) Unity tests for the passive pose streaming command group
// (PASSIVE_ENTER / PASSIVE_EXIT / PASSIVE_SET_STREAM_RATE /
// PASSIVE_ZERO_REFERENCE) handled by PassiveApi and routed through the protocol
// api dispatcher.
//
// Run with:  pio test -e native -f test_passive_api

#include <string.h>

#include <unity.h>

#include "protocol/api.h"
#include "protocol/framing.h"
#include "protocol/passive_api.h"

using namespace protocol;

namespace {

// Live safety-state wire bytes (mirror safety::State) used by the gating tests.
constexpr uint8_t kDisarmed = 2;
constexpr uint8_t kStandReady = 4;
constexpr uint8_t kMacMaintenance = 8;
constexpr uint8_t kPassivePoseStream = 9;

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

api::StatusSnapshot makeStatus(uint8_t state) {
  api::StatusSnapshot st;
  st.uptime_ms = 1000;
  st.state = state;
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

// Run one passive command through the full dispatcher (PassiveApi is the last
// delegate). `state` is the live safety state PassiveApi gates on.
DecodeStatus runPassive(PassiveApi& passive, uint8_t state, uint8_t msg_id,
                        uint16_t seq, const uint8_t* payload,
                        uint16_t payload_len, Header* resp_h,
                        uint8_t* resp_payload, size_t* resp_len) {
  passive.setLiveState(state);
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(msg_id, seq, payload, payload_len, req, sizeof(req));

  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus(state);
  const size_t resp_n = api::handleRequest(
      req + 1, req_n - 2, info, st, resp, sizeof(resp), nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &passive);
  TEST_ASSERT_TRUE(resp_n > 0);
  return decodeFrameBody(resp + 1, resp_n - 2, resp_h, resp_payload,
                         kMaxPayload, resp_len);
}

void putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

}  // namespace

void test_enter_from_disarmed_ok() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kDisarmed, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_EQUAL_size_t(2, n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::Ok), pl[0]);
  TEST_ASSERT_EQUAL_UINT8(kDisarmed, pl[1]);
  TEST_ASSERT_TRUE(p.requested());
  TEST_ASSERT_FALSE(h.flags & api::flag::kError);
}

void test_enter_from_maintenance_ok() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kMacMaintenance, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::Ok), pl[0]);
  TEST_ASSERT_TRUE(p.requested());
}

void test_enter_rejected_when_armed() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  // StandReady is not a maintenance-safe entry state.
  runPassive(p, kStandReady, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::Rejected), pl[0]);
  TEST_ASSERT_FALSE(p.requested());
}

void test_enter_idempotent_while_streaming() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kDisarmed, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  // Re-enter while the live state already reports passive streaming.
  runPassive(p, kPassivePoseStream, passivemsg::kEnter, 2, nullptr, 0, &h, pl,
             &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::Ok), pl[0]);
  TEST_ASSERT_TRUE(p.requested());
}

void test_exit_always_clears() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kDisarmed, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_TRUE(p.requested());
  // Exit from the streaming state.
  runPassive(p, kPassivePoseStream, passivemsg::kExit, 2, nullptr, 0, &h, pl,
             &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::Ok), pl[0]);
  TEST_ASSERT_FALSE(p.requested());
}

void test_set_stream_rate_ok() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kDisarmed, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  uint8_t rate[2];
  putU16(rate, 100);
  runPassive(p, kPassivePoseStream, passivemsg::kSetStreamRate, 2, rate, 2, &h,
             pl, &n);
  TEST_ASSERT_EQUAL_size_t(4, n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::Ok), pl[0]);
  TEST_ASSERT_EQUAL_UINT8(kPassivePoseStream, pl[1]);
  const uint16_t echoed =
      static_cast<uint16_t>(pl[2] | (static_cast<uint16_t>(pl[3]) << 8));
  TEST_ASSERT_EQUAL_UINT16(100, echoed);
  TEST_ASSERT_EQUAL_UINT16(100, p.streamRateHz());
}

void test_set_stream_rate_rejected_when_not_passive() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  uint8_t rate[2];
  putU16(rate, 100);
  // No PASSIVE_ENTER first: must be rejected and the rate left at default.
  runPassive(p, kDisarmed, passivemsg::kSetStreamRate, 1, rate, 2, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::Rejected), pl[0]);
  TEST_ASSERT_EQUAL_UINT16(passiverate::kDefaultHz, p.streamRateHz());
}

void test_set_stream_rate_bad_value() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kDisarmed, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  uint8_t rate[2];
  putU16(rate, 0);  // below kMinHz
  runPassive(p, kPassivePoseStream, passivemsg::kSetStreamRate, 2, rate, 2, &h,
             pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::BadRequest),
                          pl[0]);
  TEST_ASSERT_TRUE(h.flags & api::flag::kError);
  putU16(rate, passiverate::kMaxHz + 1);  // above kMaxHz
  runPassive(p, kPassivePoseStream, passivemsg::kSetStreamRate, 3, rate, 2, &h,
             pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::BadRequest),
                          pl[0]);
}

void test_set_stream_rate_short_payload() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kDisarmed, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  uint8_t one[1] = {50};
  runPassive(p, kPassivePoseStream, passivemsg::kSetStreamRate, 2, one, 1, &h,
             pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::BadRequest),
                          pl[0]);
}

void test_zero_reference_bumps_seq() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kDisarmed, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT32(0, p.zeroSeq());
  runPassive(p, kPassivePoseStream, passivemsg::kZeroReference, 2, nullptr, 0,
             &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::Ok), pl[0]);
  TEST_ASSERT_EQUAL_UINT32(1, p.zeroSeq());
  runPassive(p, kPassivePoseStream, passivemsg::kZeroReference, 3, nullptr, 0,
             &h, pl, &n);
  TEST_ASSERT_EQUAL_UINT32(2, p.zeroSeq());
}

void test_zero_reference_rejected_when_not_passive() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kDisarmed, passivemsg::kZeroReference, 1, nullptr, 0, &h, pl,
             &n);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PassiveResult::Rejected), pl[0]);
  TEST_ASSERT_EQUAL_UINT32(0, p.zeroSeq());
}

void test_clear_drops_request() {
  PassiveApi p;
  p.reset();
  Header h;
  uint8_t pl[kMaxPayload];
  size_t n = 0;
  runPassive(p, kDisarmed, passivemsg::kEnter, 1, nullptr, 0, &h, pl, &n);
  TEST_ASSERT_TRUE(p.requested());
  p.clear();  // e.g. on E-stop / fault
  TEST_ASSERT_FALSE(p.requested());
}

void test_unknown_without_passive_is_error() {
  // Dispatcher with no PassiveApi must answer passive ids with UnknownMsg.
  uint8_t req[kMaxWireFrame];
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = passivemsg::kEnter;
  h.seq = 10;
  h.payload_len = 0;
  const size_t req_n = encodeFrame(h, nullptr, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus(kDisarmed);
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
  RUN_TEST(test_enter_from_disarmed_ok);
  RUN_TEST(test_enter_from_maintenance_ok);
  RUN_TEST(test_enter_rejected_when_armed);
  RUN_TEST(test_enter_idempotent_while_streaming);
  RUN_TEST(test_exit_always_clears);
  RUN_TEST(test_set_stream_rate_ok);
  RUN_TEST(test_set_stream_rate_rejected_when_not_passive);
  RUN_TEST(test_set_stream_rate_bad_value);
  RUN_TEST(test_set_stream_rate_short_payload);
  RUN_TEST(test_zero_reference_bumps_seq);
  RUN_TEST(test_zero_reference_rejected_when_not_passive);
  RUN_TEST(test_clear_drops_request);
  RUN_TEST(test_unknown_without_passive_is_error);
  return UNITY_END();
}
