// Native (host) Unity tests for the DXL logical parameter map (dxl_params.h):
// table-aware addressing, EEPROM/RAM regions, and the SET_SERVO_LIMITS param
// selection. These pin the address layout the Arduino bus executor relies on.
//
// Run with:  pio test -e native -f test_dxl_params

#include <unity.h>

#include "dxl/dxl_model.h"
#include "dxl/dxl_params.h"

using namespace dxl;

namespace {

ParamDescriptor desc(TableKind table, LogicalParam p) {
  ParamDescriptor d;
  TEST_ASSERT_TRUE(paramDescriptor(table, p, d));
  return d;
}

void assertAbsent(TableKind table, LogicalParam p) {
  ParamDescriptor d;
  TEST_ASSERT_FALSE(paramDescriptor(table, p, d));
}

}  // namespace

void test_legacy_addresses() {
  const TableKind t = TableKind::Mx28Legacy;
  ParamDescriptor d;

  d = desc(t, LogicalParam::Id);
  TEST_ASSERT_EQUAL_UINT16(3, d.address);
  TEST_ASSERT_EQUAL_UINT8(1, d.length);
  TEST_ASSERT_EQUAL(static_cast<int>(ParamRegion::Eeprom),
                    static_cast<int>(d.region));

  d = desc(t, LogicalParam::CwAngleLimit);
  TEST_ASSERT_EQUAL_UINT16(6, d.address);
  TEST_ASSERT_EQUAL_UINT8(2, d.length);
  TEST_ASSERT_EQUAL(static_cast<int>(ParamRegion::Eeprom),
                    static_cast<int>(d.region));

  d = desc(t, LogicalParam::CcwAngleLimit);
  TEST_ASSERT_EQUAL_UINT16(8, d.address);

  d = desc(t, LogicalParam::TorqueEnable);
  TEST_ASSERT_EQUAL_UINT16(24, d.address);
  TEST_ASSERT_EQUAL(static_cast<int>(ParamRegion::Ram),
                    static_cast<int>(d.region));

  // Legacy PID gains are single-byte at D26/I27/P28.
  TEST_ASSERT_EQUAL_UINT16(28, desc(t, LogicalParam::PidP).address);
  TEST_ASSERT_EQUAL_UINT8(1, desc(t, LogicalParam::PidP).length);
  TEST_ASSERT_EQUAL_UINT16(27, desc(t, LogicalParam::PidI).address);
  TEST_ASSERT_EQUAL_UINT16(26, desc(t, LogicalParam::PidD).address);

  d = desc(t, LogicalParam::MovingSpeed);
  TEST_ASSERT_EQUAL_UINT16(32, d.address);
  TEST_ASSERT_EQUAL(static_cast<int>(ParamRegion::Ram),
                    static_cast<int>(d.region));
  TEST_ASSERT_EQUAL_UINT16(34, desc(t, LogicalParam::TorqueLimit).address);
  TEST_ASSERT_EQUAL_UINT16(14, desc(t, LogicalParam::MaxTorque).address);
  TEST_ASSERT_EQUAL_UINT16(73, desc(t, LogicalParam::GoalAcceleration).address);
}

void test_legacy_absent_params() {
  const TableKind t = TableKind::Mx28Legacy;
  // V2-only params have no legacy equivalent.
  assertAbsent(t, LogicalParam::MinPositionLimit);
  assertAbsent(t, LogicalParam::MaxPositionLimit);
  assertAbsent(t, LogicalParam::HomingOffset);
  assertAbsent(t, LogicalParam::ProfileVelocity);
  assertAbsent(t, LogicalParam::ProfileAcceleration);
  assertAbsent(t, LogicalParam::BusWatchdog);
}

void test_v2_addresses() {
  const TableKind t = TableKind::Mx28V2;
  ParamDescriptor d;

  d = desc(t, LogicalParam::Id);
  TEST_ASSERT_EQUAL_UINT16(7, d.address);

  d = desc(t, LogicalParam::MaxPositionLimit);
  TEST_ASSERT_EQUAL_UINT16(48, d.address);
  TEST_ASSERT_EQUAL_UINT8(4, d.length);
  TEST_ASSERT_EQUAL(static_cast<int>(ParamRegion::Eeprom),
                    static_cast<int>(d.region));

  d = desc(t, LogicalParam::MinPositionLimit);
  TEST_ASSERT_EQUAL_UINT16(52, d.address);
  TEST_ASSERT_EQUAL_UINT8(4, d.length);

  d = desc(t, LogicalParam::HomingOffset);
  TEST_ASSERT_EQUAL_UINT16(20, d.address);
  TEST_ASSERT_EQUAL_UINT8(4, d.length);
  TEST_ASSERT_TRUE(d.is_signed);

  d = desc(t, LogicalParam::TorqueEnable);
  TEST_ASSERT_EQUAL_UINT16(64, d.address);
  TEST_ASSERT_EQUAL(static_cast<int>(ParamRegion::Ram),
                    static_cast<int>(d.region));

  // MX(2.0) position PID gains are 2 bytes at D80/I82/P84.
  TEST_ASSERT_EQUAL_UINT16(84, desc(t, LogicalParam::PidP).address);
  TEST_ASSERT_EQUAL_UINT8(2, desc(t, LogicalParam::PidP).length);
  TEST_ASSERT_EQUAL_UINT16(82, desc(t, LogicalParam::PidI).address);
  TEST_ASSERT_EQUAL_UINT16(80, desc(t, LogicalParam::PidD).address);

  TEST_ASSERT_EQUAL_UINT16(98, desc(t, LogicalParam::BusWatchdog).address);
  TEST_ASSERT_EQUAL_UINT16(112, desc(t, LogicalParam::ProfileVelocity).address);
  TEST_ASSERT_EQUAL_UINT16(108,
                           desc(t, LogicalParam::ProfileAcceleration).address);
  TEST_ASSERT_EQUAL_UINT16(31, desc(t, LogicalParam::TemperatureLimit).address);
  TEST_ASSERT_EQUAL_UINT16(68, desc(t, LogicalParam::StatusReturnLevel).address);
}

void test_v2_absent_params() {
  const TableKind t = TableKind::Mx28V2;
  // Legacy-only params have no V2 equivalent.
  assertAbsent(t, LogicalParam::CwAngleLimit);
  assertAbsent(t, LogicalParam::CcwAngleLimit);
  assertAbsent(t, LogicalParam::MaxTorque);
  assertAbsent(t, LogicalParam::MovingSpeed);
  assertAbsent(t, LogicalParam::TorqueLimit);
  assertAbsent(t, LogicalParam::GoalAcceleration);
}

void test_unknown_table_has_no_params() {
  assertAbsent(TableKind::Unknown, LogicalParam::Id);
  assertAbsent(TableKind::Unknown, LogicalParam::TorqueEnable);
}

void test_needs_torque_off() {
  // EEPROM params need torque off; RAM params do not.
  TEST_ASSERT_TRUE(
      paramNeedsTorqueOff(TableKind::Mx28Legacy, LogicalParam::CwAngleLimit));
  TEST_ASSERT_FALSE(
      paramNeedsTorqueOff(TableKind::Mx28Legacy, LogicalParam::MovingSpeed));
  TEST_ASSERT_TRUE(
      paramNeedsTorqueOff(TableKind::Mx28V2, LogicalParam::MaxPositionLimit));
  TEST_ASSERT_FALSE(
      paramNeedsTorqueOff(TableKind::Mx28V2, LogicalParam::BusWatchdog));
  // Absent param: not a torque-off requirement.
  TEST_ASSERT_FALSE(
      paramNeedsTorqueOff(TableKind::Mx28Legacy, LogicalParam::HomingOffset));
}

void test_servo_limit_params_table_aware() {
  LogicalParam lo, hi;
  TEST_ASSERT_TRUE(servoLimitParams(TableKind::Mx28Legacy, lo, hi));
  TEST_ASSERT_EQUAL(static_cast<int>(LogicalParam::CwAngleLimit),
                    static_cast<int>(lo));
  TEST_ASSERT_EQUAL(static_cast<int>(LogicalParam::CcwAngleLimit),
                    static_cast<int>(hi));

  TEST_ASSERT_TRUE(servoLimitParams(TableKind::Mx28V2, lo, hi));
  TEST_ASSERT_EQUAL(static_cast<int>(LogicalParam::MinPositionLimit),
                    static_cast<int>(lo));
  TEST_ASSERT_EQUAL(static_cast<int>(LogicalParam::MaxPositionLimit),
                    static_cast<int>(hi));

  TEST_ASSERT_FALSE(servoLimitParams(TableKind::Unknown, lo, hi));
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_legacy_addresses);
  RUN_TEST(test_legacy_absent_params);
  RUN_TEST(test_v2_addresses);
  RUN_TEST(test_v2_absent_params);
  RUN_TEST(test_unknown_table_has_no_params);
  RUN_TEST(test_needs_torque_off);
  RUN_TEST(test_servo_limit_params_table_aware);
  return UNITY_END();
}
