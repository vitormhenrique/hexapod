// Native (host) Unity tests for the sensor / contact / leveling command group
// covered by ubs.5.1: CONTACT_ENABLE / CONTACT_DISABLE / CONTACT_SET_THRESHOLDS
// and LEVELING_ENABLE / LEVELING_DISABLE / LEVELING_SET_PARAMS, routed through
// the protocol api dispatcher (SensorApi is the 9th delegate).
//
// Run with:  pio test -e native -f test_sensor_api

#include <string.h>

#include <unity.h>

#include "protocol/api.h"
#include "protocol/feature_api.h"
#include "protocol/framing.h"
#include "protocol/sensor_api.h"

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

// Route a sensor request through handleRequest (features + sensors wired) and
// decode the response.
DecodeStatus runSensor(FeatureApi& feat, SensorApi& sen, uint8_t msg_id,
                       uint16_t seq, const uint8_t* payload,
                       uint16_t payload_len, Header* resp_h,
                       uint8_t* resp_payload, size_t* resp_len) {
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(msg_id, seq, payload, payload_len, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(
      req + 1, req_n - 2, info, st, resp, sizeof(resp), nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, &feat, &sen);
  TEST_ASSERT_TRUE(resp_n > 0);
  return decodeFrameBody(resp + 1, resp_n - 2, resp_h, resp_payload,
                         kMaxPayload, resp_len);
}

uint16_t rd16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

void wr16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

// CONTACT_ENABLE on an available feature records the desired flag and reports
// enabled=1.
void test_contact_enable_available_ok() {
  FeatureApi feat;
  feat.setLiveState(2);
  feat.setAvailability(Feature::FootContact, true, FeatureReason::None);
  SensorApi sen;
  sen.setFeatureApi(&feat);

  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactEnable, 1, nullptr,
                              0, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(5, len);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_HEX8(2, p[1]);              // live state
  TEST_ASSERT_EQUAL_UINT(1, p[2]);              // available
  TEST_ASSERT_EQUAL_UINT(1, p[3]);              // enabled
  TEST_ASSERT_TRUE(feat.desiredEnabled(Feature::FootContact));
  TEST_ASSERT_TRUE(feat.effectiveEnabled(Feature::FootContact));
}

// CONTACT_ENABLE on an unavailable feature is rejected and never forced on; the
// blocking reason is echoed and the error flag is set.
void test_contact_enable_unavailable_rejected() {
  FeatureApi feat;
  feat.setAvailability(Feature::FootContact, false,
                       FeatureReason::HardwareMissing);
  SensorApi sen;
  sen.setFeatureApi(&feat);

  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactEnable, 1, nullptr,
                              0, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Rejected), p[0]);
  TEST_ASSERT_EQUAL_UINT(0, p[2]);  // available
  TEST_ASSERT_EQUAL_UINT(0, p[3]);  // enabled
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(FeatureReason::HardwareMissing),
                         p[4]);
  TEST_ASSERT_TRUE((h.flags & api::flag::kError) != 0);
  TEST_ASSERT_FALSE(feat.desiredEnabled(Feature::FootContact));
}

// CONTACT_DISABLE always succeeds (only reduces authority).
void test_contact_disable_ok() {
  FeatureApi feat;
  feat.setAvailability(Feature::FootContact, true, FeatureReason::None);
  feat.applyDesired(Feature::FootContact, true);
  SensorApi sen;
  sen.setFeatureApi(&feat);

  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactDisable, 1, nullptr,
                              0, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_UINT(0, p[3]);  // enabled now 0
  TEST_ASSERT_FALSE(feat.desiredEnabled(Feature::FootContact));
}

// LEVELING_ENABLE while terrain leveling is unavailable is rejected with the
// reason echoed (mirrors the firmware NotImplemented availability).
void test_leveling_enable_unavailable_rejected() {
  FeatureApi feat;
  feat.setAvailability(Feature::TerrainLeveling, false,
                       FeatureReason::NotImplemented);
  SensorApi sen;
  sen.setFeatureApi(&feat);

  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kLevelingEnable, 1, nullptr,
                              0, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Rejected), p[0]);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(FeatureReason::NotImplemented),
                         p[4]);
}

// CONTACT_SET_THRESHOLDS records the per-foot values, echoes them, and bumps
// the threshold sequence so i2cTask can apply them.
void test_set_thresholds_ok() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  TEST_ASSERT_EQUAL_UINT32(0, sen.thresholdSeq());

  uint8_t req[7];
  req[0] = 3;             // foot index
  wr16(&req[1], 1200);    // near
  wr16(&req[3], 800);     // touch
  wr16(&req[5], 1500);    // load
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactSetThresholds, 1,
                              req, sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(8, len);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_UINT(3, p[1]);          // foot echoed
  TEST_ASSERT_EQUAL_UINT(1200, rd16(&p[2]));
  TEST_ASSERT_EQUAL_UINT(800, rd16(&p[4]));
  TEST_ASSERT_EQUAL_UINT(1500, rd16(&p[6]));
  TEST_ASSERT_EQUAL_UINT32(1, sen.thresholdSeq());
  TEST_ASSERT_EQUAL_UINT(1200, sen.thresholds().near_thresh[3]);
  TEST_ASSERT_EQUAL_UINT(800, sen.thresholds().touch_thresh[3]);
  TEST_ASSERT_EQUAL_UINT(1500, sen.thresholds().load_thresh[3]);
}

// A bad foot index is rejected as BadRequest and does not advance the sequence.
void test_set_thresholds_bad_foot() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  uint8_t req[7] = {kSensorNumFeet, 0, 0, 0, 0, 0, 0};  // out of range
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactSetThresholds, 1,
                              req, sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(1, len);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::BadRequest), p[0]);
  TEST_ASSERT_TRUE((h.flags & api::flag::kError) != 0);
  TEST_ASSERT_EQUAL_UINT32(0, sen.thresholdSeq());
}

// A short threshold payload is rejected as BadRequest.
void test_set_thresholds_short() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  uint8_t req[3] = {0, 0, 0};
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactSetThresholds, 1,
                              req, sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::BadRequest), p[0]);
  TEST_ASSERT_EQUAL_UINT32(0, sen.thresholdSeq());
}

// LEVELING_SET_PARAMS stores and echoes the staged tunables and bumps its seq.
void test_set_leveling_params_ok() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  TEST_ASSERT_EQUAL_UINT32(0, sen.levelingSeq());

  uint8_t req[6];
  wr16(&req[0], 5000);  // max_tilt_mdeg
  wr16(&req[2], 200);   // rate_mdeg_s
  wr16(&req[4], 64);    // response_x255
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kLevelingSetParams, 1, req,
                              sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(7, len);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_UINT(5000, rd16(&p[1]));
  TEST_ASSERT_EQUAL_UINT(200, rd16(&p[3]));
  TEST_ASSERT_EQUAL_UINT(64, rd16(&p[5]));
  TEST_ASSERT_EQUAL_UINT32(1, sen.levelingSeq());
  TEST_ASSERT_EQUAL_UINT(5000, sen.levelingParams().max_tilt_mdeg);
}

// ---- ubs.5.2: I2C scan / topology / status / rate / calibrate -------------

// I2C_SCAN bumps the scan sequence (i2cTask re-runs scanAll) and acks it.
void test_i2c_scan_bumps_seq() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  TEST_ASSERT_EQUAL_UINT32(0, sen.scanSeq());

  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kI2cScan, 1, nullptr, 0, &h,
                              p, &len));
  TEST_ASSERT_EQUAL_UINT(3, len);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_UINT(1, rd16(&p[1]));
  TEST_ASSERT_EQUAL_UINT32(1, sen.scanSeq());
}

// I2C_GET_TOPOLOGY with no snapshot wired returns a well-formed empty topology.
void test_i2c_topology_no_snapshot() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);

  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kI2cGetTopology, 1, nullptr,
                              0, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(3 + 8 * 5, len);
  TEST_ASSERT_EQUAL_UINT(0, p[0]);  // mux absent
  TEST_ASSERT_EQUAL_UINT(0, p[1]);  // eeprom absent
  TEST_ASSERT_EQUAL_UINT(8, p[2]);  // channel count
}

// I2C_GET_TOPOLOGY encodes the published snapshot.
void test_i2c_topology_snapshot() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  TopologySnapshot topo{};
  topo.valid = 1;
  topo.mux_present = 1;
  topo.eeprom_present = 1;
  topo.num_channels = kSensorNumChannels;
  topo.channels[2].scanned = 1;
  topo.channels[2].vcnl_present = 1;
  topo.channels[2].lps_present = 1;
  topo.channels[2].device_count = 2;
  topo.channels[2].state = 1;  // Present
  sen.setSnapshots(&topo, nullptr);

  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kI2cGetTopology, 1, nullptr,
                              0, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(1, p[0]);  // mux
  TEST_ASSERT_EQUAL_UINT(1, p[1]);  // eeprom
  TEST_ASSERT_EQUAL_UINT(8, p[2]);
  // Channel 2 block starts at 3 + 2*5 = 13.
  TEST_ASSERT_EQUAL_UINT(1, p[13]);      // scanned
  TEST_ASSERT_EQUAL_UINT(1, p[14]);      // vcnl
  TEST_ASSERT_EQUAL_UINT(1, p[15]);      // lps
  TEST_ASSERT_EQUAL_UINT(2, p[16]);      // device_count
  TEST_ASSERT_EQUAL_UINT(1, p[17]);      // state Present
}

// SENSOR_GET_STATUS encodes the published foot-status snapshot.
void test_sensor_status_snapshot() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  StatusSnapshot status{};
  status.valid = 1;
  status.num_feet = kSensorNumFeet;
  status.present_mask = 0x05;       // feet 0 and 2 present
  status.polling_enabled = 1;
  status.feet[2].state = 3;         // Loaded
  status.feet[2].confidence = 210;
  status.feet[2].proximity = 1234;
  status.feet[2].pressure_delta = -50;
  status.feet[2].flags = 0x04;      // loaded
  sen.setSnapshots(nullptr, &status);

  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kSensorGetStatus, 1, nullptr,
                              0, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(3 + 6 * 7, len);
  TEST_ASSERT_EQUAL_UINT(kSensorNumFeet, p[0]);
  TEST_ASSERT_EQUAL_UINT(0x05, p[1]);  // present_mask
  TEST_ASSERT_EQUAL_UINT(1, p[2]);     // polling
  // Foot 2 block starts at 3 + 2*7 = 17.
  TEST_ASSERT_EQUAL_UINT(3, p[17]);              // state
  TEST_ASSERT_EQUAL_UINT(210, p[18]);            // confidence
  TEST_ASSERT_EQUAL_UINT(1234, rd16(&p[19]));    // proximity
  TEST_ASSERT_EQUAL_INT16(-50, static_cast<int16_t>(rd16(&p[21])));
  TEST_ASSERT_EQUAL_UINT(0x04, p[23]);           // flags
}

// SENSOR_SET_RATE stages and echoes a valid poll rate.
void test_sensor_set_rate_ok() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  uint8_t req[2];
  wr16(&req[0], 50);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kSensorSetRate, 1, req,
                              sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(3, len);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_UINT(50, rd16(&p[1]));
  TEST_ASSERT_EQUAL_UINT(50, sen.sensorRateHz());
  TEST_ASSERT_EQUAL_UINT32(1, sen.rateSeq());
}

// SENSOR_SET_RATE rejects an out-of-range (zero) rate.
void test_sensor_set_rate_bad() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  uint8_t req[2];
  wr16(&req[0], 0);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kSensorSetRate, 1, req,
                              sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::BadRequest), p[0]);
  TEST_ASSERT_TRUE((h.flags & api::flag::kError) != 0);
  TEST_ASSERT_EQUAL_UINT32(0, sen.rateSeq());
}

// CONTACT_CALIBRATE for one foot stages the matching single-bit mask.
void test_contact_calibrate_one_foot() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  uint8_t req[1] = {2};
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactCalibrate, 1, req,
                              sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(2, len);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_HEX8(0x04, p[1]);
  TEST_ASSERT_EQUAL_HEX8(0x04, sen.calibrateMask());
  TEST_ASSERT_EQUAL_UINT32(1, sen.calibrateSeq());
}

// CONTACT_CALIBRATE 0xFF stages an all-feet mask.
void test_contact_calibrate_all() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  uint8_t req[1] = {0xFF};
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactCalibrate, 1, req,
                              sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_HEX8(0x3F, p[1]);
  TEST_ASSERT_EQUAL_HEX8(0x3F, sen.calibrateMask());
}

// CONTACT_CALIBRATE with a bad foot index is BadRequest and does not stage.
void test_contact_calibrate_bad_foot() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  uint8_t req[1] = {kSensorNumFeet};
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactCalibrate, 1, req,
                              sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::BadRequest), p[0]);
  TEST_ASSERT_EQUAL_UINT32(0, sen.calibrateSeq());
}

// CONTACT_CALIBRATE is rejected when the snapshot shows no present sensors.
void test_contact_calibrate_rejected_no_sensors() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  StatusSnapshot status{};
  status.valid = 1;
  status.present_mask = 0;  // no feet present
  sen.setSnapshots(nullptr, &status);
  uint8_t req[1] = {0xFF};
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kContactCalibrate, 1, req,
                              sizeof(req), &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Rejected), p[0]);
  TEST_ASSERT_TRUE((h.flags & api::flag::kError) != 0);
  TEST_ASSERT_EQUAL_UINT32(0, sen.calibrateSeq());
}

// SENSOR_CALIBRATE stages an all-feet mask regardless of payload.
void test_sensor_calibrate_all() {
  FeatureApi feat;
  SensorApi sen;
  sen.setFeatureApi(&feat);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    runSensor(feat, sen, sensormsg::kSensorCalibrate, 1, nullptr,
                              0, &h, p, &len));
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(SensorResult::Ok), p[0]);
  TEST_ASSERT_EQUAL_HEX8(0x3F, p[1]);
  TEST_ASSERT_EQUAL_HEX8(0x3F, sen.calibrateMask());
  TEST_ASSERT_EQUAL_UINT32(1, sen.calibrateSeq());
}

// Without a SensorApi delegate the sensor block is an unknown message.
void test_unknown_without_sensor_api() {
  FeatureApi feat;
  uint8_t req[kMaxWireFrame];
  const size_t req_n =
      buildRequest(sensormsg::kContactEnable, 7, nullptr, 0, req, sizeof(req));
  uint8_t resp[kMaxWireFrame];
  const api::DeviceInfo info = makeInfo();
  const api::StatusSnapshot st = makeStatus();
  const size_t resp_n = api::handleRequest(
      req + 1, req_n - 2, info, st, resp, sizeof(resp), nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, &feat, nullptr);
  TEST_ASSERT_TRUE(resp_n > 0);
  Header h;
  uint8_t p[kMaxPayload];
  size_t len = 0;
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    decodeFrameBody(resp + 1, resp_n - 2, &h, p, kMaxPayload,
                                    &len));
  TEST_ASSERT_TRUE((h.flags & api::flag::kError) != 0);
  TEST_ASSERT_EQUAL_UINT(static_cast<uint8_t>(api::Error::UnknownMsg), p[0]);
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_contact_enable_available_ok);
  RUN_TEST(test_contact_enable_unavailable_rejected);
  RUN_TEST(test_contact_disable_ok);
  RUN_TEST(test_leveling_enable_unavailable_rejected);
  RUN_TEST(test_set_thresholds_ok);
  RUN_TEST(test_set_thresholds_bad_foot);
  RUN_TEST(test_set_thresholds_short);
  RUN_TEST(test_set_leveling_params_ok);
  RUN_TEST(test_i2c_scan_bumps_seq);
  RUN_TEST(test_i2c_topology_no_snapshot);
  RUN_TEST(test_i2c_topology_snapshot);
  RUN_TEST(test_sensor_status_snapshot);
  RUN_TEST(test_sensor_set_rate_ok);
  RUN_TEST(test_sensor_set_rate_bad);
  RUN_TEST(test_contact_calibrate_one_foot);
  RUN_TEST(test_contact_calibrate_all);
  RUN_TEST(test_contact_calibrate_bad_foot);
  RUN_TEST(test_contact_calibrate_rejected_no_sensors);
  RUN_TEST(test_sensor_calibrate_all);
  RUN_TEST(test_unknown_without_sensor_api);
  return UNITY_END();
}
