// Native (host) unit tests for the foot contact estimator. No Arduino deps.
// Run with: pio test -e native -f test_contact_estimator

#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/sensors/contact_estimator.h"

using namespace sensors;
using config::FootSensorCal;

namespace {

// Build a calibration array with every foot enabled and identical thresholds.
void enabledCal(FootSensorCal (&cal)[kNumFeet], int32_t baseline = 1000,
                uint16_t near = 50, uint16_t touch = 100, uint16_t load = 300) {
  for (uint8_t i = 0; i < kNumFeet; ++i) {
    cal[i].pressure_baseline = baseline;
    cal[i].near_thresh = near;
    cal[i].touch_thresh = touch;
    cal[i].load_thresh = load;
    cal[i].enabled = 1;
  }
}

ContactParams fastParams() {
  ContactParams p;
  p.touch_debounce = 1;
  p.load_debounce = 1;
  p.release_debounce = 1;
  p.fault_limit = 3;
  p.stale_timeout_ms = 100;
  p.baseline_track = 0;  // disable drift so tests are deterministic
  return p;
}

FootSample s(uint16_t prox, int32_t press, bool ok = true) {
  FootSample fs;
  fs.proximity_raw = prox;
  fs.pressure_raw = press;
  fs.ok = ok;
  return fs;
}

}  // namespace

void test_disabled_foot_stays_air() {
  FootSensorCal cal[kNumFeet] = {};  // all disabled
  ContactEstimator est;
  est.configure(cal, fastParams());
  est.update(0, s(9999, 99999), 10);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Air),
                          static_cast<uint8_t>(est.foot(0).state));
  // Raw values are still mirrored for telemetry.
  TEST_ASSERT_EQUAL_UINT16(9999, est.foot(0).proximity_raw);
}

void test_near_then_touch_then_loaded() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal);
  ContactEstimator est;
  est.configure(cal, fastParams());

  // Proximity only -> NEAR (pressure at baseline, delta 0).
  est.update(0, s(80, 1000), 10);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Near),
                          static_cast<uint8_t>(est.foot(0).state));

  // Pressure delta crosses touch (1000+150 -> delta 150 >= 100).
  est.update(0, s(80, 1150), 20);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Touch),
                          static_cast<uint8_t>(est.foot(0).state));
  TEST_ASSERT_TRUE(est.foot(0).touch);

  // Pressure delta crosses load (1000+400 -> delta 400 >= 300).
  est.update(0, s(80, 1400), 30);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Loaded),
                          static_cast<uint8_t>(est.foot(0).state));
  TEST_ASSERT_TRUE(est.foot(0).loaded);
  TEST_ASSERT_EQUAL_UINT8(1, est.loadedMask() & 0x01);
}

void test_release_then_air() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal);
  ContactEstimator est;
  est.configure(cal, fastParams());

  est.update(0, s(80, 1400), 10);  // LOADED
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Loaded),
                          static_cast<uint8_t>(est.foot(0).state));

  // Drop below release threshold (touch/2 = 50) and below near -> RELEASE.
  est.update(0, s(10, 1000), 20);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Release),
                          static_cast<uint8_t>(est.foot(0).state));
  TEST_ASSERT_TRUE(est.foot(0).release);

  // Continued no contact -> AIR.
  est.update(0, s(10, 1000), 30);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Air),
                          static_cast<uint8_t>(est.foot(0).state));
}

void test_touch_debounce_requires_consecutive_samples() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal);
  ContactParams p = fastParams();
  p.touch_debounce = 3;
  ContactEstimator est;
  est.configure(cal, p);

  est.update(0, s(0, 1150), 10);  // delta 150 >= touch, count 1
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Air),
                          static_cast<uint8_t>(est.foot(0).state));
  est.update(0, s(0, 1150), 20);  // count 2
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Air),
                          static_cast<uint8_t>(est.foot(0).state));
  est.update(0, s(0, 1150), 30);  // count 3 -> TOUCH
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Touch),
                          static_cast<uint8_t>(est.foot(0).state));
}

void test_failed_reads_go_fault_then_recover() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal);
  ContactEstimator est;
  est.configure(cal, fastParams());  // fault_limit = 3

  est.update(0, s(0, 0, false), 10);
  est.update(0, s(0, 0, false), 20);
  TEST_ASSERT_NOT_EQUAL(static_cast<uint8_t>(ContactState::Fault),
                        static_cast<uint8_t>(est.foot(0).state));
  est.update(0, s(0, 0, false), 30);  // 3rd failure -> FAULT
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Fault),
                          static_cast<uint8_t>(est.foot(0).state));
  TEST_ASSERT_TRUE(est.foot(0).fault);

  // A good sample clears the fault.
  est.update(0, s(0, 1000), 40);
  TEST_ASSERT_FALSE(est.foot(0).fault);
  TEST_ASSERT_NOT_EQUAL(static_cast<uint8_t>(ContactState::Fault),
                        static_cast<uint8_t>(est.foot(0).state));
}

void test_staleness_after_timeout() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal);
  ContactEstimator est;
  est.configure(cal, fastParams());  // stale_timeout_ms = 100

  est.update(0, s(80, 1400), 100);  // LOADED at t=100
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Loaded),
                          static_cast<uint8_t>(est.foot(0).state));

  est.tickStaleness(150);  // 50 ms < timeout: still loaded
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Loaded),
                          static_cast<uint8_t>(est.foot(0).state));

  est.tickStaleness(220);  // 120 ms >= timeout: STALE
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Stale),
                          static_cast<uint8_t>(est.foot(0).state));
  TEST_ASSERT_TRUE(est.foot(0).stale);
}

void test_never_stale_without_a_sample() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal);
  ContactEstimator est;
  est.configure(cal, fastParams());
  est.tickStaleness(100000);  // huge gap but no sample ever seen
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Air),
                          static_cast<uint8_t>(est.foot(0).state));
}

void test_baseline_drift_tracks_while_unloaded() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal, /*baseline=*/1000);
  ContactParams p = fastParams();
  p.baseline_track = 5;  // nudge 5 counts/update
  ContactEstimator est;
  est.configure(cal, p);

  // Unloaded reading above baseline: baseline should climb toward it.
  est.update(0, s(0, 1020), 10);
  TEST_ASSERT_EQUAL_INT32(1005, est.foot(0).pressure_baseline);
  est.update(0, s(0, 1020), 20);
  TEST_ASSERT_EQUAL_INT32(1010, est.foot(0).pressure_baseline);
}

void test_loaded_mask_multiple_feet() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal);
  ContactEstimator est;
  est.configure(cal, fastParams());
  est.update(0, s(0, 1400), 10);  // LOADED
  est.update(2, s(0, 1400), 10);  // LOADED
  est.update(4, s(0, 1400), 10);  // LOADED
  TEST_ASSERT_EQUAL_UINT8(0x15, est.loadedMask());  // bits 0,2,4
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_disabled_foot_stays_air);
  RUN_TEST(test_near_then_touch_then_loaded);
  RUN_TEST(test_release_then_air);
  RUN_TEST(test_touch_debounce_requires_consecutive_samples);
  RUN_TEST(test_failed_reads_go_fault_then_recover);
  RUN_TEST(test_staleness_after_timeout);
  RUN_TEST(test_never_stale_without_a_sample);
  RUN_TEST(test_baseline_drift_tracks_while_unloaded);
  RUN_TEST(test_loaded_mask_multiple_feet);
  return UNITY_END();
}
