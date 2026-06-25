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
  TEST_ASSERT_EQUAL_UINT(12, len);
  const uint32_t uptime = p[0] | (p[1] << 8) | (p[2] << 16) |
                          (static_cast<uint32_t>(p[3]) << 24);
  TEST_ASSERT_EQUAL_UINT32(123456, uptime);
  TEST_ASSERT_EQUAL_HEX8(2, p[4]);     // state
  TEST_ASSERT_EQUAL_HEX8(0x02, p[5]);  // flags: dxl_power_control only
  const uint16_t batt = p[6] | (p[7] << 8);
  TEST_ASSERT_EQUAL_UINT16(11800, batt);
  const uint32_t missed = p[8] | (p[9] << 8) | (p[10] << 16) |
                          (static_cast<uint32_t>(p[11]) << 24);
  TEST_ASSERT_EQUAL_UINT32(0, missed);
}

void test_api_get_capabilities() {
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runRequest(api::msg::kGetCapabilities, 4, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(25, len);
  TEST_ASSERT_EQUAL_HEX8(kVersionMajor, p[0]);
  TEST_ASSERT_EQUAL_HEX8(kVersionMinor, p[1]);
  const uint32_t feat = p[5] | (p[6] << 8) | (p[7] << 16) |
                        (static_cast<uint32_t>(p[8]) << 24);
  TEST_ASSERT_EQUAL_UINT32(0, feat);
  TEST_ASSERT_EQUAL_STRING("OpenRB150-Hex", reinterpret_cast<char*>(&p[9]));
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
  return UNITY_END();
}
