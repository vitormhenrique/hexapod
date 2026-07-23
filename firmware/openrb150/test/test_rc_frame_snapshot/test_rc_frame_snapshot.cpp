// Native tests for the latest complete RC-frame handoff.

#include <unity.h>

#include "input/rc_frame_snapshot.h"

namespace {

controller::RcFrameSnapshot makeSnapshot(uint32_t sequence,
                                         uint32_t published_ms,
                                         float twist_vx, bool armed) {
  controller::RcFrameSnapshot snapshot;
  snapshot.command.valid = true;
  snapshot.command.failsafe = false;
  snapshot.command.frame_ms = published_ms;
  snapshot.command.twist_vx = twist_vx;
  snapshot.status.ever_seen = true;
  snapshot.status.armed = armed;
  snapshot.status.kill = !armed;
  snapshot.status.failsafe = false;
  snapshot.status.last_frame_ms = published_ms;
  snapshot.frame_sequence = sequence;
  snapshot.published_ms = published_ms;
  return snapshot;
}

}  // namespace

void test_empty_mailbox_has_no_frame() {
  controller::RcFrameMailbox mailbox;
  mailbox.reset();
  controller::RcFrameSnapshot snapshot;
  TEST_ASSERT_FALSE(mailbox.copy(snapshot));
}

void test_mailbox_returns_one_complete_published_frame() {
  controller::RcFrameMailbox mailbox;
  mailbox.reset();
  const controller::RcFrameSnapshot expected =
      makeSnapshot(/*sequence=*/7, /*published_ms=*/120, 0.6f, true);

  const uint32_t mailbox_sequence = mailbox.publish(expected);
  controller::RcFrameSnapshot actual;
  uint32_t copied_sequence = 0;

  TEST_ASSERT_TRUE(mailbox.copy(actual, &copied_sequence));
  TEST_ASSERT_EQUAL_UINT32(mailbox_sequence, copied_sequence);
  TEST_ASSERT_EQUAL_UINT32(expected.frame_sequence, actual.frame_sequence);
  TEST_ASSERT_EQUAL_UINT32(expected.published_ms, actual.published_ms);
  TEST_ASSERT_EQUAL_UINT32(expected.command.frame_ms, actual.command.frame_ms);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected.command.twist_vx,
                           actual.command.twist_vx);
  TEST_ASSERT_EQUAL(expected.status.armed, actual.status.armed);
  TEST_ASSERT_EQUAL(expected.status.kill, actual.status.kill);
  TEST_ASSERT_EQUAL_UINT32(expected.status.last_frame_ms,
                           actual.status.last_frame_ms);
}

void test_new_publication_replaces_the_entire_prior_frame() {
  controller::RcFrameMailbox mailbox;
  mailbox.reset();
  mailbox.publish(makeSnapshot(/*sequence=*/2, /*published_ms=*/20, -0.8f,
                               false));
  const controller::RcFrameSnapshot expected =
      makeSnapshot(/*sequence=*/3, /*published_ms=*/30, 0.9f, true);
  mailbox.publish(expected);

  controller::RcFrameSnapshot actual;
  TEST_ASSERT_TRUE(mailbox.copy(actual));
  TEST_ASSERT_EQUAL_UINT32(expected.frame_sequence, actual.frame_sequence);
  TEST_ASSERT_EQUAL_UINT32(expected.published_ms, actual.published_ms);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected.command.twist_vx,
                           actual.command.twist_vx);
  TEST_ASSERT_EQUAL(expected.status.armed, actual.status.armed);
  TEST_ASSERT_EQUAL(expected.status.kill, actual.status.kill);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_empty_mailbox_has_no_frame);
  RUN_TEST(test_mailbox_returns_one_complete_published_frame);
  RUN_TEST(test_new_publication_replaces_the_entire_prior_frame);
  return UNITY_END();
}