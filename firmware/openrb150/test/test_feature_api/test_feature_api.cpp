// Native (host) Unity tests for the feature flag command group
// (FEATURE_GET / FEATURE_SET / FEATURE_GET_REASONS / FEATURE_RESET_DEFAULTS)
// handled by FeatureApi and routed through the protocol api dispatcher.
//
// Run with:  pio test -e native -f test_feature_api

#include <string.h>

#include <unity.h>

#include "protocol/api.h"
#include "protocol/feature_api.h"
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

api::StatusSnapshot makeStatus(uint8_t state = 2) {
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

// Route one feature request through handleRequest (features is the final
// delegate) and decode the response.
DecodeStatus runFeature(FeatureApi& feat, uint8_t msg_id, uint16_t seq,
                        const uint8_t* payload, uint16_t payload_len,
                        Header* resp_h, uint8_t* resp_payload,
                        size_t* resp_len) {
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(msg_id, seq, payload, payload_len, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(
      req + 1, req_n - 2, info, st, resp, sizeof(resp), nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, &feat);
  TEST_ASSERT_TRUE(resp_n > 0);
  return decodeFrameBody(resp + 1, resp_n - 2, resp_h, resp_payload,
                         kMaxPayload, resp_len);
}

constexpr uint8_t kSensorPolling = static_cast<uint8_t>(Feature::SensorPolling);
constexpr uint8_t kFootContact = static_cast<uint8_t>(Feature::FootContact);

// FEATURE_GET returns the full state list with defaults reflected.
void test_get_reports_defaults() {
  FeatureApi feat;
  feat.setLiveState(2);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runFeature(feat, featuremsg::kGet, 1, nullptr, 0, &h, p,
                               &len));
  TEST_ASSERT_EQUAL_UINT(2 + 4 * kFeatureCount, len);
  TEST_ASSERT_EQUAL_HEX8(2, p[0]);             // live state echoed
  TEST_ASSERT_EQUAL_UINT(kFeatureCount, p[1]);  // count
  // Each record [id, available, enabled, reason]; nothing available by default.
  for (uint8_t i = 0; i < kFeatureCount; ++i) {
    const uint8_t* rec = &p[2 + 4 * i];
    TEST_ASSERT_EQUAL_UINT(i, rec[0]);
    TEST_ASSERT_EQUAL_UINT(0, rec[1]);  // available=0
    TEST_ASSERT_EQUAL_UINT(0, rec[2]);  // enabled=0 (desired&&available)
  }
}

// effectiveEnabled requires both desired and available.
void test_effective_requires_available() {
  FeatureApi feat;
  // SensorPolling defaults desired=true but availability is false at reset.
  TEST_ASSERT_TRUE(feat.desiredEnabled(Feature::SensorPolling));
  TEST_ASSERT_FALSE(feat.effectiveEnabled(Feature::SensorPolling));
  feat.setAvailability(Feature::SensorPolling, true, FeatureReason::None);
  TEST_ASSERT_TRUE(feat.effectiveEnabled(Feature::SensorPolling));
}

// FEATURE_SET enable on an available feature is accepted and reflected.
void test_set_enable_available_ok() {
  FeatureApi feat;
  feat.setAvailability(Feature::FootContact, true, FeatureReason::None);
  feat.setLiveState(2);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  uint8_t req[2] = {kFootContact, 1};
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runFeature(feat, featuremsg::kSet, 1, req, 2, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(6, len);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(FeatureResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_UINT(kFootContact, p[2]);
  TEST_ASSERT_EQUAL_UINT(1, p[3]);  // available
  TEST_ASSERT_EQUAL_UINT(1, p[4]);  // enabled
  TEST_ASSERT_TRUE(feat.effectiveEnabled(Feature::FootContact));
}

// FEATURE_SET enable on an unavailable feature is Rejected with the reason
// echoed; the desired flag is NOT changed.
void test_set_enable_unavailable_rejected() {
  FeatureApi feat;
  feat.setAvailability(Feature::FootContact, false,
                       FeatureReason::HardwareMissing);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  uint8_t req[2] = {kFootContact, 1};
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runFeature(feat, featuremsg::kSet, 1, req, 2, &h, p, &len));
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(FeatureResult::Rejected), p[0]);
  TEST_ASSERT_EQUAL_UINT(0, p[3]);  // available=0
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(FeatureReason::HardwareMissing),
                         p[5]);
  TEST_ASSERT_FALSE(feat.desiredEnabled(Feature::FootContact));
}

// Disabling is always honoured even when unavailable.
void test_set_disable_always_ok() {
  FeatureApi feat;
  feat.setAvailability(Feature::SensorPolling, false,
                       FeatureReason::HardwareMissing);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  uint8_t req[2] = {kSensorPolling, 0};
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runFeature(feat, featuremsg::kSet, 1, req, 2, &h, p, &len));
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(FeatureResult::Ok), p[0]);
  TEST_ASSERT_FALSE(feat.desiredEnabled(Feature::SensorPolling));
}

// Unknown feature id is a BadRequest with the error flag set.
void test_set_unknown_feature_bad_request() {
  FeatureApi feat;
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  uint8_t req[2] = {99, 1};
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runFeature(feat, featuremsg::kSet, 1, req, 2, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(1, len);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(FeatureResult::BadRequest), p[0]);
  TEST_ASSERT_TRUE((h.flags & api::flag::kError) != 0);
}

// Short FEATURE_SET payload is a BadRequest.
void test_set_short_payload_bad_request() {
  FeatureApi feat;
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  uint8_t req[1] = {kFootContact};
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runFeature(feat, featuremsg::kSet, 1, req, 1, &h, p, &len));
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(FeatureResult::BadRequest), p[0]);
}

// FEATURE_GET_REASONS returns [state, count, {id, reason} x count].
void test_get_reasons() {
  FeatureApi feat;
  feat.setAvailability(Feature::FootContact, false,
                       FeatureReason::HardwareMissing);
  feat.setLiveState(2);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runFeature(feat, featuremsg::kGetReasons, 1, nullptr, 0, &h,
                               p, &len));
  TEST_ASSERT_EQUAL_UINT(2 + 2 * kFeatureCount, len);
  TEST_ASSERT_EQUAL_UINT(kFeatureCount, p[1]);
  // FootContact reason byte should be HardwareMissing.
  const uint8_t* rec = &p[2 + 2 * kFootContact];
  TEST_ASSERT_EQUAL_UINT(kFootContact, rec[0]);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(FeatureReason::HardwareMissing),
                         rec[1]);
}

// FEATURE_RESET_DEFAULTS restores the compiled enable set.
void test_reset_defaults() {
  FeatureApi feat;
  feat.setAvailability(Feature::SensorPolling, true, FeatureReason::None);
  // Disable sensor polling away from its default.
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  uint8_t off[2] = {kSensorPolling, 0};
  runFeature(feat, featuremsg::kSet, 1, off, 2, &h, p, &len);
  TEST_ASSERT_FALSE(feat.desiredEnabled(Feature::SensorPolling));
  // Reset restores default (SensorPolling on).
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runFeature(feat, featuremsg::kResetDefaults, 2, nullptr, 0,
                               &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(3 + 4 * kFeatureCount, len);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(FeatureResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_UINT(kFeatureCount, p[2]);
  TEST_ASSERT_TRUE(feat.desiredEnabled(Feature::SensorPolling));
}

// seq increments on an actual desired change but not on a no-op set.
void test_seq_changes() {
  FeatureApi feat;
  feat.setAvailability(Feature::FootContact, true, FeatureReason::None);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  const uint32_t s0 = feat.seq();
  uint8_t on[2] = {kFootContact, 1};
  runFeature(feat, featuremsg::kSet, 1, on, 2, &h, p, &len);
  const uint32_t s1 = feat.seq();
  TEST_ASSERT_TRUE(s1 > s0);
  // Repeating the same enable is a no-op (seq unchanged).
  runFeature(feat, featuremsg::kSet, 2, on, 2, &h, p, &len);
  TEST_ASSERT_EQUAL_UINT(s1, feat.seq());
}

// Without a FeatureApi the dispatcher rejects the feature range as unknown.
void test_unknown_without_feature_api() {
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(featuremsg::kGet, 1, nullptr, 0, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(req + 1, req_n - 2, info, st, resp,
                                           sizeof(resp));
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    decodeFrameBody(resp + 1, resp_n - 2, &h, p, kMaxPayload,
                                    &len));
  TEST_ASSERT_TRUE((h.flags & api::flag::kError) != 0);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(api::Error::UnknownMsg), p[0]);
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_get_reports_defaults);
  RUN_TEST(test_effective_requires_available);
  RUN_TEST(test_set_enable_available_ok);
  RUN_TEST(test_set_enable_unavailable_rejected);
  RUN_TEST(test_set_disable_always_ok);
  RUN_TEST(test_set_unknown_feature_bad_request);
  RUN_TEST(test_set_short_payload_bad_request);
  RUN_TEST(test_get_reasons);
  RUN_TEST(test_reset_defaults);
  RUN_TEST(test_seq_changes);
  RUN_TEST(test_unknown_without_feature_api);
  return UNITY_END();
}
