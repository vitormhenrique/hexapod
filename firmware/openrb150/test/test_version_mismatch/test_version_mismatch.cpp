// Native (host) Unity tests for protocol version-mismatch handling (4sa.5).
//
// The frame layer is version-CARRYING, not version-GATING: a frame with the
// right magic + CRC but a different protocol version decodes Ok (the version
// rides in the header) so the API/handshake layer can diagnose a mismatch
// instead of the decoder silently dropping the frame. The firmware always
// advertises its OWN protocol version in responses (frame header + the
// HELLO/GET_CAPABILITIES payload), which is the host's diagnostic surface.
// protocol/tests/test_protocol.py mirrors this contract on the companion side.
//
// Run with:  pio test -e native -f test_version_mismatch

#include <string.h>

#include <unity.h>

#include "protocol/api.h"
#include "protocol/framing.h"

using namespace protocol;

namespace {

api::DeviceInfo makeInfo() {
  api::DeviceInfo info;
  info.fw_major = 0;
  info.fw_minor = 1;
  info.fw_patch = 0;
  const char name[] = "OpenRB150-Hex";
  memset(info.device_name, 0, sizeof(info.device_name));
  memcpy(info.device_name, name, sizeof(name) - 1);
  return info;
}

api::StatusSnapshot makeStatus() {
  api::StatusSnapshot st;
  st.uptime_ms = 1000;
  st.state = 2;
  return st;
}

// Build a HELLO command frame whose header advertises ver_major.ver_minor.
size_t buildHello(uint8_t ver_major, uint8_t ver_minor, uint16_t seq,
                  uint8_t* out, size_t cap) {
  Header h;
  h.version_major = ver_major;
  h.version_minor = ver_minor;
  h.msg_type = static_cast<uint8_t>(MsgType::Command);
  h.msg_id = api::msg::kHello;
  h.seq = seq;
  h.payload_len = 0;
  return encodeFrame(h, nullptr, out, cap);
}

// A frame from a different major version still has valid magic + CRC, so it
// decodes Ok and the mismatched version is preserved in the header. The decoder
// validates identity (magic) and integrity (CRC) but intentionally tolerates
// the version so the API layer can diagnose rather than silently drop.
void test_decode_preserves_mismatched_version() {
  uint8_t wire[kMaxWireFrame];
  const size_t n =
      buildHello(kVersionMajor + 7, kVersionMinor + 3, 5, wire, sizeof(wire));
  TEST_ASSERT_TRUE(n > 0);

  Header h;
  uint8_t payload[kMaxPayload];
  size_t plen = 0;
  const DecodeStatus st =
      decodeFrameBody(wire + 1, n - 2, &h, payload, sizeof(payload), &plen);
  TEST_ASSERT_EQUAL(DecodeStatus::Ok, st);
  TEST_ASSERT_EQUAL_UINT(kVersionMajor + 7, h.version_major);
  TEST_ASSERT_EQUAL_UINT(kVersionMinor + 3, h.version_minor);
  TEST_ASSERT_EQUAL_UINT(api::msg::kHello, h.msg_id);
  TEST_ASSERT_EQUAL_UINT(0, plen);
}

// handleRequest answers a HELLO whose request advertises a mismatched version,
// and the response advertises the FIRMWARE's protocol version in both the frame
// header and the HELLO payload -- the host's diagnostic surface for detecting
// the mismatch. The mismatched request is not dropped or errored.
void test_handle_answers_and_advertises_firmware_version() {
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildHello(kVersionMajor + 9, kVersionMinor, 42, req, sizeof(req));
  TEST_ASSERT_TRUE(req_n > 0);

  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot stat = makeStatus();
  const size_t resp_n = api::handleRequest(req + 1, req_n - 2, info, stat, resp,
                                           sizeof(resp));
  TEST_ASSERT_TRUE(resp_n > 0);

  Header h;
  uint8_t payload[kMaxPayload];
  size_t plen = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    decodeFrameBody(resp + 1, resp_n - 2, &h, payload,
                                    sizeof(payload), &plen));
  // Answered (not an error) and echoes the request id + seq.
  TEST_ASSERT_EQUAL_UINT(0, h.flags & api::flag::kError);
  TEST_ASSERT_EQUAL_UINT(api::msg::kHello, h.msg_id);
  TEST_ASSERT_EQUAL_UINT(42, h.seq);
  // Response frame header carries the firmware's own version, NOT the request's
  // mismatched one.
  TEST_ASSERT_EQUAL_UINT(kVersionMajor, h.version_major);
  TEST_ASSERT_EQUAL_UINT(kVersionMinor, h.version_minor);
  // HELLO payload also advertises proto_major/proto_minor for the handshake.
  TEST_ASSERT_EQUAL_UINT(21, plen);
  TEST_ASSERT_EQUAL_UINT(kVersionMajor, payload[0]);
  TEST_ASSERT_EQUAL_UINT(kVersionMinor, payload[1]);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_decode_preserves_mismatched_version);
  RUN_TEST(test_handle_answers_and_advertises_firmware_version);
  return UNITY_END();
}
