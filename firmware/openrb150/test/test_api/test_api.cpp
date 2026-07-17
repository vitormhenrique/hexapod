// Native (host) Unity tests for the USB API v0 layer and streaming frame reader.
//
// Run with:  pio test -e native
//
// Requests/responses are built with the proven encodeFrame()/decodeFrameBody()
// (golden-checked against the Python reference in test_protocol.cpp), so this
// file avoids hand-transcribed hex. The fixed DeviceInfo/StatusSnapshot match
// API_DEVICE/API_STATUS in protocol/tests/gen_vectors.py, and the Python test
// (test_protocol.py) pins the same values against frames.json — giving an
// end-to-end byte-for-byte cross-check between firmware and host.

#include <string.h>

#include <unity.h>

#include "hil/observer_api.h"
#include "protocol/api.h"
#include "protocol/frame_reader.h"
#include "protocol/framing.h"

using namespace protocol;

namespace {

api::DeviceInfo makeInfo() {
  api::DeviceInfo info;
  info.fw_major = 0;
  info.fw_minor = 1;
  info.fw_patch = 0;
  info.feature_bits = 0;
  info.build_flags = api::capabilityflag::kHilOutputDisabled;
  const char name[] = "OpenRB150-Hex";
  memset(info.device_name, 0, sizeof(info.device_name));
  memcpy(info.device_name, name, sizeof(name) - 1);
  return info;
}

api::StatusSnapshot makeStatus() {
  api::StatusSnapshot st;
  st.uptime_ms = 123456;
  st.state = 2;  // Disarmed
  st.dxl_power = false;
  st.dxl_power_control = true;
  st.battery_mv = 11800;
  st.watchdog_missed = 0;
  st.reset_cause = 0x20;
  st.last_reset_watchdog_missed = 0x02;
  st.last_reset_progress_marker = 51;
  st.control_stack_free_words = 123;
  st.dxl_stack_free_words = 234;
  st.last_reset_control_progress = 80;
  st.last_reset_safety_state = 5;
  st.dxl_power_transitions = 7;
  st.hil_flags = api::hilflag::kOutputDisabled |
                 api::hilflag::kPowerGuardActive |
                 api::hilflag::kTorqueGuardActive |
                 api::hilflag::kGoalGuardActive |
                 api::hilflag::kWriteGuardActive;
  st.blocked_power_enable = 11;
  st.blocked_torque_enable = 12;
  st.blocked_goal_write = 13;
  st.blocked_dxl_write = 14;
  st.last_goal_sequence = 15;
  st.last_goal_count = 2;
  st.last_fault_reason = 4;
  st.last_fault_timestamp_ms = 120000;
  st.last_fatal_reason = 2;
  st.last_fatal_stage = 6;
  memcpy(st.last_fatal_task_name, "control", 8);
  st.last_fault_stack_pointer = 0x20007100;
  st.last_fault_exception_return = 0xFFFFFFFD;
  for (uint8_t index = 0; index < 8; ++index) {
    st.last_fault_registers[index] = 0x1000u + index;
  }
  return st;
}

// Build a Command request wire frame for msg_id/seq.
size_t buildRequest(uint8_t msg_id, uint16_t seq, uint8_t* out, size_t cap) {
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = msg_id;
  h.seq = seq;
  h.payload_len = 0;
  return encodeFrame(h, nullptr, out, cap);
}

// Run one request through handleRequest and return the decoded response.
DecodeStatus runRequest(uint8_t msg_id, uint16_t seq, Header* resp_h,
                        uint8_t* resp_payload, size_t* resp_len) {
  uint8_t req[kMaxWireFrame];
  const size_t req_n = buildRequest(msg_id, seq, req, sizeof(req));

  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(req + 1, req_n - 2, info, st, resp,
                                           sizeof(resp));
  TEST_ASSERT_TRUE(resp_n > 0);
  TEST_ASSERT_EQUAL_HEX8(0x00, resp[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, resp[resp_n - 1]);
  return decodeFrameBody(resp + 1, resp_n - 2, resp_h, resp_payload,
                         kMaxPayload, resp_len);
}

void test_api_hello() {
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runRequest(api::msg::kHello, 1, &h, p, &len));
  TEST_ASSERT_EQUAL(static_cast<uint8_t>(MsgType::Response), h.msg_type);
  TEST_ASSERT_EQUAL_UINT16(1, h.seq);
  TEST_ASSERT_EQUAL_UINT(21, len);
  TEST_ASSERT_EQUAL_HEX8(kVersionMajor, p[0]);
  TEST_ASSERT_EQUAL_HEX8(kVersionMinor, p[1]);
  TEST_ASSERT_EQUAL_HEX8(0, p[2]);  // fw_major
  TEST_ASSERT_EQUAL_HEX8(1, p[3]);  // fw_minor
  TEST_ASSERT_EQUAL_HEX8(0, p[4]);  // fw_patch
  TEST_ASSERT_EQUAL_STRING("OpenRB150-Hex", reinterpret_cast<char*>(&p[5]));
}

void test_api_heartbeat() {
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runRequest(api::msg::kHeartbeat, 2, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(5, len);
  const uint32_t uptime = p[0] | (p[1] << 8) | (p[2] << 16) |
                          (static_cast<uint32_t>(p[3]) << 24);
  TEST_ASSERT_EQUAL_UINT32(123456, uptime);
  TEST_ASSERT_EQUAL_HEX8(2, p[4]);  // Disarmed
}

void test_api_get_status() {
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runRequest(api::msg::kGetStatus, 3, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(113, len);
  const uint32_t uptime = p[0] | (p[1] << 8) | (p[2] << 16) |
                          (static_cast<uint32_t>(p[3]) << 24);
  TEST_ASSERT_EQUAL_UINT32(123456, uptime);
  TEST_ASSERT_EQUAL_HEX8(2, p[4]);     // state
  TEST_ASSERT_EQUAL_HEX8(0x06, p[5]);  // power control + output-disabled HIL
  const uint16_t batt = p[6] | (p[7] << 8);
  TEST_ASSERT_EQUAL_UINT16(11800, batt);
  const uint32_t missed = p[8] | (p[9] << 8) | (p[10] << 16) |
                          (static_cast<uint32_t>(p[11]) << 24);
  TEST_ASSERT_EQUAL_UINT32(0, missed);
  TEST_ASSERT_EQUAL_HEX8(0x20, p[12]);
  TEST_ASSERT_EQUAL_HEX32(0x02, static_cast<uint32_t>(p[13]) |
                                   (static_cast<uint32_t>(p[14]) << 8) |
                                   (static_cast<uint32_t>(p[15]) << 16) |
                                   (static_cast<uint32_t>(p[16]) << 24));
  TEST_ASSERT_EQUAL_UINT8(51, p[17]);
  TEST_ASSERT_EQUAL_UINT16(123, static_cast<uint16_t>(p[18]) |
                                   (static_cast<uint16_t>(p[19]) << 8));
  TEST_ASSERT_EQUAL_UINT16(234, static_cast<uint16_t>(p[20]) |
                                   (static_cast<uint16_t>(p[21]) << 8));
  TEST_ASSERT_EQUAL_UINT8(80, p[22]);
  TEST_ASSERT_EQUAL_UINT8(5, p[23]);
  TEST_ASSERT_EQUAL_UINT32(7, static_cast<uint32_t>(p[24]) |
                                 (static_cast<uint32_t>(p[25]) << 8) |
                                 (static_cast<uint32_t>(p[26]) << 16) |
                                 (static_cast<uint32_t>(p[27]) << 24));
  TEST_ASSERT_EQUAL_HEX8(0x1F, p[28]);
  TEST_ASSERT_EQUAL_UINT32(11, static_cast<uint32_t>(p[29]) |
                                  (static_cast<uint32_t>(p[30]) << 8) |
                                  (static_cast<uint32_t>(p[31]) << 16) |
                                  (static_cast<uint32_t>(p[32]) << 24));
  TEST_ASSERT_EQUAL_UINT32(12, static_cast<uint32_t>(p[33]) |
                                  (static_cast<uint32_t>(p[34]) << 8) |
                                  (static_cast<uint32_t>(p[35]) << 16) |
                                  (static_cast<uint32_t>(p[36]) << 24));
  TEST_ASSERT_EQUAL_UINT32(13, static_cast<uint32_t>(p[37]) |
                                  (static_cast<uint32_t>(p[38]) << 8) |
                                  (static_cast<uint32_t>(p[39]) << 16) |
                                  (static_cast<uint32_t>(p[40]) << 24));
  TEST_ASSERT_EQUAL_UINT32(14, static_cast<uint32_t>(p[41]) |
                                  (static_cast<uint32_t>(p[42]) << 8) |
                                  (static_cast<uint32_t>(p[43]) << 16) |
                                  (static_cast<uint32_t>(p[44]) << 24));
  TEST_ASSERT_EQUAL_UINT32(15, static_cast<uint32_t>(p[45]) |
                                  (static_cast<uint32_t>(p[46]) << 8) |
                                  (static_cast<uint32_t>(p[47]) << 16) |
                                  (static_cast<uint32_t>(p[48]) << 24));
  TEST_ASSERT_EQUAL_UINT8(2, p[49]);
  TEST_ASSERT_EQUAL_UINT8(4, p[50]);
  TEST_ASSERT_EQUAL_UINT32(120000, static_cast<uint32_t>(p[51]) |
                                       (static_cast<uint32_t>(p[52]) << 8) |
                                       (static_cast<uint32_t>(p[53]) << 16) |
                                       (static_cast<uint32_t>(p[54]) << 24));
  TEST_ASSERT_EQUAL_UINT8(2, p[55]);
  TEST_ASSERT_EQUAL_UINT8(6, p[56]);
  TEST_ASSERT_EQUAL_STRING("control", reinterpret_cast<char*>(&p[57]));
  TEST_ASSERT_EQUAL_HEX32(0x20007100, static_cast<uint32_t>(p[73]) |
                                         (static_cast<uint32_t>(p[74]) << 8) |
                                         (static_cast<uint32_t>(p[75]) << 16) |
                                         (static_cast<uint32_t>(p[76]) << 24));
  TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFD, static_cast<uint32_t>(p[77]) |
                                         (static_cast<uint32_t>(p[78]) << 8) |
                                         (static_cast<uint32_t>(p[79]) << 16) |
                                         (static_cast<uint32_t>(p[80]) << 24));
  for (uint8_t index = 0; index < 8; ++index) {
    const uint8_t offset = static_cast<uint8_t>(81 + index * 4);
    const uint32_t value = static_cast<uint32_t>(p[offset]) |
                           (static_cast<uint32_t>(p[offset + 1]) << 8) |
                           (static_cast<uint32_t>(p[offset + 2]) << 16) |
                           (static_cast<uint32_t>(p[offset + 3]) << 24);
    TEST_ASSERT_EQUAL_HEX32(0x1000u + index, value);
  }
}

void test_api_get_capabilities() {
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runRequest(api::msg::kGetCapabilities, 4, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(26, len);
  TEST_ASSERT_EQUAL_HEX8(kVersionMajor, p[0]);
  TEST_ASSERT_EQUAL_HEX8(kVersionMinor, p[1]);
  const uint32_t feat = p[5] | (p[6] << 8) | (p[7] << 16) |
                        (static_cast<uint32_t>(p[8]) << 24);
  TEST_ASSERT_EQUAL_UINT32(0, feat);
  TEST_ASSERT_EQUAL_STRING("OpenRB150-Hex", reinterpret_cast<char*>(&p[9]));
  TEST_ASSERT_EQUAL_HEX8(api::capabilityflag::kHilOutputDisabled, p[25]);
}

void test_api_unknown_msg() {
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok, runRequest(0x7E, 9, &h, p, &len));
  TEST_ASSERT_TRUE(h.flags & api::flag::kError);
  TEST_ASSERT_EQUAL_UINT(1, len);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(api::Error::UnknownMsg), p[0]);
}

void test_api_rejects_non_command() {
  // A Response-typed frame must not be answered.
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Response);
  h.msg_id = api::msg::kGetStatus;
  h.payload_len = 0;
  uint8_t wire[kMaxWireFrame];
  const size_t n = encodeFrame(h, nullptr, wire, sizeof(wire));

  uint8_t out[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  TEST_ASSERT_EQUAL_UINT(
      0, api::handleRequest(wire + 1, n - 2, info, st, out, sizeof(out)));
}

// --- FrameReader ----------------------------------------------------------
void test_frame_reader_single() {
  uint8_t req[kMaxWireFrame];
  const size_t n = buildRequest(api::msg::kHello, 1, req, sizeof(req));

  FrameReader reader;
  int frames = 0;
  size_t got_len = 0;
  for (size_t i = 0; i < n; ++i) {
    if (reader.push(req[i])) {
      ++frames;
      got_len = reader.length();
    }
  }
  TEST_ASSERT_EQUAL_INT(1, frames);
  TEST_ASSERT_EQUAL_UINT(n - 2, got_len);
}

void test_frame_reader_back_to_back() {
  uint8_t a[kMaxWireFrame];
  const size_t n = buildRequest(api::msg::kHeartbeat, 2, a, sizeof(a));

  FrameReader reader;
  int frames = 0;
  for (size_t i = 0; i < n; ++i) {
    if (reader.push(a[i])) ++frames;
  }
  // Reuse the trailing 0x00 as the next frame's leading delimiter.
  for (size_t i = 1; i < n; ++i) {
    if (reader.push(a[i])) ++frames;
  }
  TEST_ASSERT_EQUAL_INT(2, frames);
}

void test_frame_reader_ignores_empty() {
  FrameReader reader;
  TEST_ASSERT_FALSE(reader.push(0x00));
  TEST_ASSERT_FALSE(reader.push(0x00));  // consecutive delimiters: no frame
}

void test_frame_reader_roundtrip_handle() {
  // End-to-end: stream a request through the reader, dispatch, decode response.
  uint8_t req[kMaxWireFrame];
  const size_t n = buildRequest(api::msg::kGetStatus, 7, req, sizeof(req));

  FrameReader reader;
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  int answered = 0;
  for (size_t i = 0; i < n; ++i) {
    if (!reader.push(req[i])) continue;
    uint8_t out[kMaxWireFrame];
    const size_t rn = api::handleRequest(reader.body(), reader.length(), info,
                                         st, out, sizeof(out));
    TEST_ASSERT_TRUE(rn > 0);
    Header rh;
    uint8_t rp[kMaxPayload];
    size_t rl = 0;
    TEST_ASSERT_EQUAL(DecodeStatus::Ok, decodeFrameBody(out + 1, rn - 2, &rh,
                                                        rp, sizeof(rp), &rl));
    TEST_ASSERT_EQUAL_UINT16(7, rh.seq);
    ++answered;
  }
  TEST_ASSERT_EQUAL_INT(1, answered);
}

void test_frame_reader_counts_frames_and_overflow() {
  // hexapod_src-lv6: framesOk counts complete bodies; overflowsDropped counts
  // frames dropped for exceeding the buffer (once per frame, not per byte).
  uint8_t req[kMaxWireFrame];
  const size_t n = buildRequest(api::msg::kHello, 3, req, sizeof(req));

  FrameReader reader;
  TEST_ASSERT_EQUAL_UINT32(0, reader.framesOk());
  TEST_ASSERT_EQUAL_UINT32(0, reader.overflowsDropped());

  for (size_t i = 0; i < n; ++i) reader.push(req[i]);
  TEST_ASSERT_EQUAL_UINT32(1, reader.framesOk());

  // Overflow: a run of non-delimiter bytes longer than the reader buffer.
  for (size_t i = 0; i < kMaxFrameBodyCobs + 64; ++i) {
    TEST_ASSERT_FALSE(reader.push(0x55));
  }
  TEST_ASSERT_FALSE(reader.push(0x00));  // closing delimiter: frame dropped
  TEST_ASSERT_EQUAL_UINT32(1, reader.overflowsDropped());
  TEST_ASSERT_EQUAL_UINT32(1, reader.framesOk());

  // The reader recovers: the next good frame still parses and counts.
  for (size_t i = 0; i < n; ++i) reader.push(req[i]);
  TEST_ASSERT_EQUAL_UINT32(2, reader.framesOk());
  TEST_ASSERT_EQUAL_UINT32(1, reader.overflowsDropped());
}

void test_api_reports_decode_status() {
  // hexapod_src-lv6: a corrupt body yields no response but reports the decode
  // failure through the out-param so the caller can count rx_bad.
  uint8_t req[kMaxWireFrame];
  const size_t n = buildRequest(api::msg::kGetStatus, 9, req, sizeof(req));
  req[n - 2] ^= 0xFF;  // corrupt the last body byte (CRC) inside delimiters

  uint8_t out[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  DecodeStatus dst = DecodeStatus::Ok;
  TEST_ASSERT_EQUAL_UINT(
      0, api::handleRequest(req + 1, n - 2, info, st, out, sizeof(out), nullptr,
                            nullptr, nullptr, nullptr, nullptr, nullptr,
                            nullptr, nullptr, nullptr, nullptr, nullptr, &dst));
  TEST_ASSERT_TRUE(dst != DecodeStatus::Ok);

  // A valid request reports Ok and still answers.
  const size_t good_n = buildRequest(api::msg::kGetStatus, 10, req, sizeof(req));
  dst = DecodeStatus::BadCrc;
  TEST_ASSERT_TRUE(api::handleRequest(req + 1, good_n - 2, info, st, out,
                                      sizeof(out), nullptr, nullptr, nullptr,
                                      nullptr, nullptr, nullptr, nullptr,
                                      nullptr, nullptr, nullptr, nullptr,
                                      &dst) > 0);
  TEST_ASSERT_EQUAL(DecodeStatus::Ok, dst);
}

void test_api_routes_hil_observer_commands() {
  hil::ObserverApi observer(true);
  hil::OutputGuardStatus guard;
  guard.output_disabled = true;
  guard.power_guard_active = true;
  guard.torque_guard_active = true;
  guard.goal_guard_active = true;
  guard.write_guard_active = true;
  observer.setNow(123456);
  observer.setLiveState(hil::ObserverApi::kDisarmedState);
  observer.setMaintenanceLock(44, true);
  observer.setOutputGuard(guard);

  Header request;
  request.msg_type = static_cast<uint8_t>(MsgType::Command);
  request.msg_id = hil::observermsg::kGetCapability;
  request.seq = 21;
  uint8_t wire[kMaxWireFrame] = {};
  const size_t request_length =
      encodeFrame(request, nullptr, wire, sizeof(wire));
  uint8_t response[kMaxWireFrame] = {};
  DecodeStatus decode_status = DecodeStatus::Ok;
  const size_t response_length = api::handleRequest(
      wire + 1, request_length - 2, makeInfo(), makeStatus(), response,
      sizeof(response), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, &decode_status, &observer);
  TEST_ASSERT_TRUE(response_length > 0);
  TEST_ASSERT_EQUAL(DecodeStatus::Ok, decode_status);

  Header response_header;
  uint8_t payload[kMaxPayload] = {};
  size_t payload_length = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    decodeFrameBody(response + 1, response_length - 2,
                                    &response_header, payload, sizeof(payload),
                                    &payload_length));
  TEST_ASSERT_EQUAL_UINT8(hil::observermsg::kGetCapability,
                          response_header.msg_id);
  TEST_ASSERT_EQUAL_UINT16(21, response_header.seq);
  TEST_ASSERT_EQUAL_UINT(19, payload_length);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hil::ObserverResult::Ok),
                          payload[0]);

    hil::ObserverApi unavailable(false);
    const size_t unavailable_length = api::handleRequest(
      wire + 1, request_length - 2, makeInfo(), makeStatus(), response,
      sizeof(response), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, &decode_status,
      &unavailable);
    TEST_ASSERT_TRUE(unavailable_length > 0);
    TEST_ASSERT_EQUAL(DecodeStatus::Ok,
            decodeFrameBody(response + 1, unavailable_length - 2,
                    &response_header, payload, sizeof(payload),
                    &payload_length));
    TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::ObserverResult::NotAvailable), payload[0]);

    const size_t absent_length = api::handleRequest(
      wire + 1, request_length - 2, makeInfo(), makeStatus(), response,
      sizeof(response), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, &decode_status, nullptr);
    TEST_ASSERT_TRUE(absent_length > 0);
    TEST_ASSERT_EQUAL(DecodeStatus::Ok,
            decodeFrameBody(response + 1, absent_length - 2,
                    &response_header, payload, sizeof(payload),
                    &payload_length));
    TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(hil::ObserverResult::NotAvailable), payload[0]);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_api_hello);
  RUN_TEST(test_api_heartbeat);
  RUN_TEST(test_api_get_status);
  RUN_TEST(test_api_get_capabilities);
  RUN_TEST(test_api_unknown_msg);
  RUN_TEST(test_api_rejects_non_command);
  RUN_TEST(test_frame_reader_single);
  RUN_TEST(test_frame_reader_back_to_back);
  RUN_TEST(test_frame_reader_ignores_empty);
  RUN_TEST(test_frame_reader_roundtrip_handle);
  RUN_TEST(test_frame_reader_counts_frames_and_overflow);
  RUN_TEST(test_api_reports_decode_status);
  RUN_TEST(test_api_routes_hil_observer_commands);
  return UNITY_END();
}
