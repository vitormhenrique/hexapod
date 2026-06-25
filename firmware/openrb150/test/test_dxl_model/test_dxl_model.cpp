// Native (host) unit tests for the portable DXL model/capability classifier.
// These exercise dxl_model.{h,cpp} with no Arduino/Dynamixel2Arduino deps.
//
// Run with: pio test -e native

#include <unity.h>

#include "../../src/dxl/dxl_model.h"

using namespace dxl;

void test_table_kind_from_model() {
  TEST_ASSERT_EQUAL(static_cast<int>(TableKind::Mx28Legacy),
                    static_cast<int>(tableKindFromModel(kModelMx28Legacy)));
  TEST_ASSERT_EQUAL(static_cast<int>(TableKind::Mx28V2),
                    static_cast<int>(tableKindFromModel(kModelMx28_2)));
  TEST_ASSERT_EQUAL(static_cast<int>(TableKind::Unknown),
                    static_cast<int>(tableKindFromModel(0)));
  TEST_ASSERT_EQUAL(static_cast<int>(TableKind::Unknown),
                    static_cast<int>(tableKindFromModel(1060)));  // some XL model
}

void test_default_protocol_for_table() {
  TEST_ASSERT_EQUAL_UINT8(1, defaultProtocolForTable(TableKind::Mx28Legacy));
  TEST_ASSERT_EQUAL_UINT8(2, defaultProtocolForTable(TableKind::Mx28V2));
  TEST_ASSERT_EQUAL_UINT8(0, defaultProtocolForTable(TableKind::Unknown));
}

void test_fill_legacy_profile() {
  ServoProfile p;
  fillProfileFromModel(p, kModelMx28Legacy, /*fw=*/26);
  TEST_ASSERT_EQUAL(static_cast<int>(TableKind::Mx28Legacy),
                    static_cast<int>(p.table_kind));
  TEST_ASSERT_EQUAL_UINT16(kModelMx28Legacy, p.model_number);
  TEST_ASSERT_EQUAL_UINT8(26, p.firmware_version);
  TEST_ASSERT_EQUAL_UINT8(1, p.protocol_version);
  TEST_ASSERT_TRUE(p.supports_cw_ccw_angle_limits);
  TEST_ASSERT_FALSE(p.supports_min_max_position_limits);
  TEST_ASSERT_FALSE(p.supports_profile_velocity);
  TEST_ASSERT_FALSE(p.supports_bus_watchdog);
  TEST_ASSERT_FALSE(p.supports_sync_read);
  TEST_ASSERT_FALSE(p.supports_fast_sync_read);
}

void test_fill_mx28_2_profile_old_fw() {
  ServoProfile p;
  // Firmware below the fast-sync-read threshold: sync read yes, fast no.
  fillProfileFromModel(p, kModelMx28_2, kFastSyncReadMinFw - 1);
  TEST_ASSERT_EQUAL(static_cast<int>(TableKind::Mx28V2),
                    static_cast<int>(p.table_kind));
  TEST_ASSERT_EQUAL_UINT8(2, p.protocol_version);
  TEST_ASSERT_FALSE(p.supports_cw_ccw_angle_limits);
  TEST_ASSERT_TRUE(p.supports_min_max_position_limits);
  TEST_ASSERT_TRUE(p.supports_profile_velocity);
  TEST_ASSERT_TRUE(p.supports_bus_watchdog);
  TEST_ASSERT_TRUE(p.supports_sync_read);
  TEST_ASSERT_FALSE(p.supports_fast_sync_read);
}

void test_fill_mx28_2_profile_new_fw() {
  ServoProfile p;
  fillProfileFromModel(p, kModelMx28_2, kFastSyncReadMinFw);
  TEST_ASSERT_TRUE(p.supports_sync_read);
  TEST_ASSERT_TRUE(p.supports_fast_sync_read);
}

void test_fill_unknown_profile() {
  ServoProfile p;
  fillProfileFromModel(p, /*model=*/9999, /*fw=*/10);
  TEST_ASSERT_EQUAL(static_cast<int>(TableKind::Unknown),
                    static_cast<int>(p.table_kind));
  TEST_ASSERT_EQUAL_UINT8(0, p.protocol_version);
  TEST_ASSERT_FALSE(p.supports_cw_ccw_angle_limits);
  TEST_ASSERT_FALSE(p.supports_min_max_position_limits);
  TEST_ASSERT_FALSE(p.supports_profile_velocity);
  TEST_ASSERT_FALSE(p.supports_bus_watchdog);
  TEST_ASSERT_FALSE(p.supports_sync_read);
  TEST_ASSERT_FALSE(p.supports_fast_sync_read);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_table_kind_from_model);
  RUN_TEST(test_default_protocol_for_table);
  RUN_TEST(test_fill_legacy_profile);
  RUN_TEST(test_fill_mx28_2_profile_old_fw);
  RUN_TEST(test_fill_mx28_2_profile_new_fw);
  RUN_TEST(test_fill_unknown_profile);
  return UNITY_END();
}
