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

// Runtime setThresholds() (host CONTACT_SET_THRESHOLDS) changes the touch/load
// gates without disturbing baselines, so the same sample now classifies
// differently.
void test_set_thresholds_changes_classification() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal, /*baseline=*/1000, /*near=*/50, /*touch=*/100, /*load=*/300);
  ContactEstimator est;
  est.configure(cal, fastParams());

  // delta 150 >= touch(100) but < load(300) -> TOUCH.
  est.update(0, s(0, 1150), 10);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Touch),
                          static_cast<uint8_t>(est.foot(0).state));

  // Lower the load threshold below the live delta; the next equal sample now
  // reaches LOADED. Baseline is untouched (still 1000).
  est.setThresholds(0, /*near=*/50, /*touch=*/100, /*load=*/120);
  est.update(0, s(0, 1150), 20);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Loaded),
                          static_cast<uint8_t>(est.foot(0).state));
  TEST_ASSERT_EQUAL_INT32(1000, est.foot(0).pressure_baseline);
}

// setThresholds() ignores out-of-range legs (no crash / no effect).
void test_set_thresholds_bad_leg_ignored() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal);
  ContactEstimator est;
  est.configure(cal, fastParams());
  est.setThresholds(kNumFeet, 1, 2, 3);  // out of range, must be a no-op
  est.update(0, s(80, 1000), 10);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Near),
                          static_cast<uint8_t>(est.foot(0).state));
}

// captureBaseline() re-zeroes the per-foot baseline to the latest reading so a
// foot resting under load reads ~0 delta and no longer classifies as loaded.
void test_capture_baseline_rezeroes_delta() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal, /*baseline=*/1000, /*near=*/50, /*touch=*/100, /*load=*/300);
  ContactEstimator est;
  est.configure(cal, fastParams());

  // A standing offset of 400 over baseline -> LOADED.
  est.update(0, s(0, 1400), 10);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Loaded),
                          static_cast<uint8_t>(est.foot(0).state));

  // Capture the current reading (1400) as the new baseline; delta drops to 0.
  est.captureBaseline(0);
  TEST_ASSERT_EQUAL_INT32(1400, est.foot(0).pressure_baseline);
  TEST_ASSERT_EQUAL_INT32(0, est.foot(0).pressure_delta);

  // The same raw 1400 now reads delta 0 -> falls back out of LOADED.
  est.update(0, s(0, 1400), 20);
  est.update(0, s(0, 1400), 30);
  TEST_ASSERT_NOT_EQUAL(static_cast<uint8_t>(ContactState::Loaded),
                        static_cast<uint8_t>(est.foot(0).state));
}

// captureBaseline() ignores out-of-range legs (no crash / no effect).
void test_capture_baseline_bad_leg_ignored() {
  FootSensorCal cal[kNumFeet];
  enabledCal(cal);
  ContactEstimator est;
  est.configure(cal, fastParams());
  est.captureBaseline(kNumFeet);  // out of range, must be a no-op
  TEST_ASSERT_EQUAL_INT32(0, est.foot(0).pressure_delta);
}

// setEnabled() activates a disabled-but-calibrated foot so it begins
// classifying (CONTACT_CALIBRATE path), and refuses to enable a foot that has
// no usable pressure calibration so it can never classify noise.
void test_set_enabled_activates_calibrated_foot() {
  FootSensorCal cal[kNumFeet] = {};  // all disabled
  // Foot 0 carries a usable calibration but starts disabled; foot 1 has none.
  cal[0].pressure_baseline = 1000;
  cal[0].near_thresh = 50;
  cal[0].touch_thresh = 100;
  cal[0].load_thresh = 300;
  cal[0].enabled = 0;
  ContactEstimator est;
  est.configure(cal, fastParams());

  // While disabled, a big reading still classifies as AIR.
  est.update(0, s(9999, 99999), 10);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Air),
                          static_cast<uint8_t>(est.foot(0).state));

  // Enabling a calibrated foot succeeds; it now classifies load.
  TEST_ASSERT_TRUE(est.setEnabled(0, true));
  est.update(0, s(0, 1400), 20);  // delta 400 > load thresh 300
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Loaded),
                          static_cast<uint8_t>(est.foot(0).state));

  // Enabling an uncalibrated foot (zero thresholds) is refused: it stays AIR.
  TEST_ASSERT_FALSE(est.setEnabled(1, true));
  est.update(1, s(9999, 99999), 30);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Air),
                          static_cast<uint8_t>(est.foot(1).state));

  // Disabling always succeeds and returns the foot to AIR-only behaviour.
  TEST_ASSERT_TRUE(est.setEnabled(0, false));
  est.update(0, s(0, 1400), 40);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ContactState::Air),
                          static_cast<uint8_t>(est.foot(0).state));

  // Out-of-range leg is a safe no-op (returns false).
  TEST_ASSERT_FALSE(est.setEnabled(kNumFeet, true));
}

// setEnabled() refuses a foot whose LOADED threshold sits below TOUCH (the same
// inverted-ordering case validateRobotConfig rejects).
void test_set_enabled_refuses_inverted_thresholds() {
  FootSensorCal cal[kNumFeet] = {};
  cal[0].near_thresh = 50;
  cal[0].touch_thresh = 400;
  cal[0].load_thresh = 200;  // load < touch -> invalid
  cal[0].enabled = 0;
  ContactEstimator est;
  est.configure(cal, fastParams());
  TEST_ASSERT_FALSE(est.setEnabled(0, true));
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
  RUN_TEST(test_set_thresholds_changes_classification);
  RUN_TEST(test_set_thresholds_bad_leg_ignored);
  RUN_TEST(test_capture_baseline_rezeroes_delta);
  RUN_TEST(test_capture_baseline_bad_leg_ignored);
  RUN_TEST(test_set_enabled_activates_calibrated_foot);
  RUN_TEST(test_set_enabled_refuses_inverted_thresholds);
  return UNITY_END();
}
