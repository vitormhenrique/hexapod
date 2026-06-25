// Native (host) unit tests for the command-source arbiter. No Arduino deps.
// Run with: pio test -e native -f test_command_arbiter

#include <unity.h>

#include "../../src/safety/command_arbiter.h"

using namespace safety;

namespace {

RcInputs rc(bool seen, bool kill, bool armed, bool autonomy) {
  RcInputs in;
  in.ever_seen = seen;
  in.kill = kill;
  in.armed = armed;
  in.autonomy_enabled = autonomy;
  return in;
}

CommandArbiter makeArbiter() {
  CommandArbiter a;
  ArbiterParams p;
  p.jetson_ttl_ms = 250;
  p.mac_lock_ttl_ms = 1000;
  a.configure(p);
  return a;
}

}  // namespace

void test_disarmed_has_no_authority() {
  CommandArbiter a = makeArbiter();
  const ArbiterOutput& o = a.update(rc(true, false, false, false), 100);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::None),
                          static_cast<uint8_t>(o.source));
  TEST_ASSERT_FALSE(o.motion_authorized);
  TEST_ASSERT_FALSE(o.kill_active);
}

void test_rc_armed_owns_motion() {
  CommandArbiter a = makeArbiter();
  const ArbiterOutput& o = a.update(rc(true, false, true, false), 100);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::Rc),
                          static_cast<uint8_t>(o.source));
  TEST_ASSERT_TRUE(o.motion_authorized);
}

void test_kill_overrides_everything() {
  CommandArbiter a = makeArbiter();
  // Even with a fresh Jetson heartbeat + autonomy, kill denies all motion.
  a.jetsonHeartbeat(100);
  const ArbiterOutput& o = a.update(rc(true, true, true, true), 110);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::None),
                          static_cast<uint8_t>(o.source));
  TEST_ASSERT_FALSE(o.motion_authorized);
  TEST_ASSERT_TRUE(o.kill_active);
}

void test_host_estop_denies_motion() {
  CommandArbiter a = makeArbiter();
  a.setHostEstop(true);
  const ArbiterOutput& o = a.update(rc(true, false, true, false), 100);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::None),
                          static_cast<uint8_t>(o.source));
  TEST_ASSERT_TRUE(o.kill_active);
  a.setHostEstop(false);
  const ArbiterOutput& o2 = a.update(rc(true, false, true, false), 110);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::Rc),
                          static_cast<uint8_t>(o2.source));
}

void test_jetson_requires_autonomy_and_fresh_heartbeat() {
  CommandArbiter a = makeArbiter();
  a.jetsonHeartbeat(100);

  // Armed + autonomy + fresh heartbeat -> Jetson.
  const ArbiterOutput& o = a.update(rc(true, false, true, true), 200);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::Jetson),
                          static_cast<uint8_t>(o.source));

  // Armed + fresh heartbeat but autonomy switch OFF -> falls back to RC.
  const ArbiterOutput& o2 = a.update(rc(true, false, true, false), 210);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::Rc),
                          static_cast<uint8_t>(o2.source));
}

void test_jetson_authority_lost_when_heartbeat_stale() {
  CommandArbiter a = makeArbiter();
  a.jetsonHeartbeat(100);
  // 250 ms later still fresh (<= ttl).
  const ArbiterOutput& o = a.update(rc(true, false, true, true), 350);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::Jetson),
                          static_cast<uint8_t>(o.source));
  // 251 ms later stale -> RC.
  const ArbiterOutput& o2 = a.update(rc(true, false, true, true), 351);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::Rc),
                          static_cast<uint8_t>(o2.source));
}

void test_mac_lock_grant_requires_disarmed() {
  CommandArbiter a = makeArbiter();
  // Establish armed state then try to grab a lock -> denied.
  a.update(rc(true, false, true, false), 100);
  TEST_ASSERT_EQUAL_UINT32(0, a.requestMacLock(100));
  // Disarm, then a lock is granted.
  a.update(rc(true, false, false, false), 110);
  const uint32_t token = a.requestMacLock(110);
  TEST_ASSERT_NOT_EQUAL(0, token);
}

void test_mac_lock_owns_motion_while_held() {
  CommandArbiter a = makeArbiter();
  a.update(rc(true, false, false, false), 100);
  const uint32_t token = a.requestMacLock(100);
  TEST_ASSERT_NOT_EQUAL(0, token);
  const ArbiterOutput& o = a.update(rc(true, false, false, false), 110);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::MacMaintenance),
                          static_cast<uint8_t>(o.source));
  TEST_ASSERT_TRUE(o.motion_authorized);
  TEST_ASSERT_TRUE(o.mac_lock_held);
  TEST_ASSERT_EQUAL_UINT32(token, o.mac_lock_token);
}

void test_mac_lock_expires_safely() {
  CommandArbiter a = makeArbiter();  // ttl 1000 ms
  a.update(rc(true, false, false, false), 100);
  const uint32_t token = a.requestMacLock(100);
  TEST_ASSERT_NOT_EQUAL(0, token);

  // Heartbeat within TTL keeps it.
  TEST_ASSERT_TRUE(a.macLockHeartbeat(token, 900));
  const ArbiterOutput& o = a.update(rc(true, false, false, false), 900);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::MacMaintenance),
                          static_cast<uint8_t>(o.source));

  // No heartbeat past TTL -> lock expires, authority returns to None.
  const ArbiterOutput& o2 = a.update(rc(true, false, false, false), 1950);
  TEST_ASSERT_FALSE(o2.mac_lock_held);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::None),
                          static_cast<uint8_t>(o2.source));
  // A late heartbeat must not resurrect the expired lock.
  TEST_ASSERT_FALSE(a.macLockHeartbeat(token, 1960));
}

void test_kill_revokes_mac_lock() {
  CommandArbiter a = makeArbiter();
  a.update(rc(true, false, false, false), 100);
  const uint32_t token = a.requestMacLock(100);
  TEST_ASSERT_NOT_EQUAL(0, token);
  // Kill asserted -> lock revoked immediately.
  const ArbiterOutput& o = a.update(rc(true, true, false, false), 110);
  TEST_ASSERT_FALSE(o.mac_lock_held);
  // The old token can no longer heartbeat.
  TEST_ASSERT_FALSE(a.macLockHeartbeat(token, 120));
}

void test_release_frees_lock_for_new_holder() {
  CommandArbiter a = makeArbiter();
  a.update(rc(true, false, false, false), 100);
  const uint32_t t1 = a.requestMacLock(100);
  TEST_ASSERT_NOT_EQUAL(0, t1);
  // A second request while held is denied.
  TEST_ASSERT_EQUAL_UINT32(0, a.requestMacLock(110));
  a.releaseMacLock(t1);
  const uint32_t t2 = a.requestMacLock(120);
  TEST_ASSERT_NOT_EQUAL(0, t2);
  TEST_ASSERT_NOT_EQUAL(t1, t2);
}

void test_no_rc_link_denies_motion() {
  CommandArbiter a = makeArbiter();
  // Never seen + kill asserted (failsafe maps to kill) -> None.
  const ArbiterOutput& o = a.update(rc(false, true, false, false), 100);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandSource::None),
                          static_cast<uint8_t>(o.source));
  TEST_ASSERT_TRUE(o.kill_active);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_disarmed_has_no_authority);
  RUN_TEST(test_rc_armed_owns_motion);
  RUN_TEST(test_kill_overrides_everything);
  RUN_TEST(test_host_estop_denies_motion);
  RUN_TEST(test_jetson_requires_autonomy_and_fresh_heartbeat);
  RUN_TEST(test_jetson_authority_lost_when_heartbeat_stale);
  RUN_TEST(test_mac_lock_grant_requires_disarmed);
  RUN_TEST(test_mac_lock_owns_motion_while_held);
  RUN_TEST(test_mac_lock_expires_safely);
  RUN_TEST(test_kill_revokes_mac_lock);
  RUN_TEST(test_release_frees_lock_for_new_holder);
  RUN_TEST(test_no_rc_link_denies_motion);
  return UNITY_END();
}
