// Native (host) unit tests for the telemetry subscription manager. No Arduino.
// Run with: pio test -e native -f test_telemetry

#include <unity.h>

#include "../../src/protocol/telemetry.h"

using namespace protocol;

namespace {

constexpr uint8_t kErrFlag = 0x02;

}  // namespace

void test_subscribe_sets_enabled_and_rate() {
  SubscriptionManager m;
  TEST_ASSERT_FALSE(m.enabled(StreamId::Health));
  const uint16_t eff = m.subscribe(StreamId::Health, 5);
  TEST_ASSERT_EQUAL_UINT16(5, eff);
  TEST_ASSERT_TRUE(m.enabled(StreamId::Health));
  TEST_ASSERT_EQUAL_UINT16(5, m.rateHz(StreamId::Health));
}

void test_subscribe_clamps_to_stream_max() {
  SubscriptionManager m;
  // Health max is 10 Hz; a 1000 Hz request clamps down.
  const uint16_t eff = m.subscribe(StreamId::Health, 1000);
  TEST_ASSERT_EQUAL_UINT16(10, eff);
  TEST_ASSERT_EQUAL_UINT16(10, m.rateHz(StreamId::Health));
  // ServoStatus max is 50 Hz.
  TEST_ASSERT_EQUAL_UINT16(50, m.subscribe(StreamId::ServoStatus, 200));
}

void test_subscribe_clamps_to_min_rate() {
  SubscriptionManager m;
  const uint16_t eff = m.subscribe(StreamId::Health, 0);
  TEST_ASSERT_EQUAL_UINT16(kMinRateHz, eff);
}

void test_unsubscribe_disables() {
  SubscriptionManager m;
  m.subscribe(StreamId::RcInput, 10);
  TEST_ASSERT_TRUE(m.enabled(StreamId::RcInput));
  m.unsubscribe(StreamId::RcInput);
  TEST_ASSERT_FALSE(m.enabled(StreamId::RcInput));
  TEST_ASSERT_FALSE(m.shouldEmit(StreamId::RcInput, 1000));
}

void test_should_emit_enforces_rate() {
  SubscriptionManager m;
  m.subscribe(StreamId::ContactState, 100);  // interval 10 ms
  // First call primes + emits.
  TEST_ASSERT_TRUE(m.shouldEmit(StreamId::ContactState, 1000));
  // Too soon (5 ms later): no emit.
  TEST_ASSERT_FALSE(m.shouldEmit(StreamId::ContactState, 1005));
  // One interval later: emit.
  TEST_ASSERT_TRUE(m.shouldEmit(StreamId::ContactState, 1010));
  TEST_ASSERT_EQUAL_UINT32(2, m.emitted(StreamId::ContactState));
  TEST_ASSERT_EQUAL_UINT32(0, m.dropped(StreamId::ContactState));
}

void test_missed_slots_counted_as_dropped() {
  SubscriptionManager m;
  m.subscribe(StreamId::ServoStatus, 50);  // interval 20 ms
  TEST_ASSERT_TRUE(m.shouldEmit(StreamId::ServoStatus, 1000));  // prime
  // Jump 100 ms = 5 intervals: 1 on-time emit + 4 missed slots dropped.
  TEST_ASSERT_TRUE(m.shouldEmit(StreamId::ServoStatus, 1100));
  TEST_ASSERT_EQUAL_UINT32(2, m.emitted(StreamId::ServoStatus));
  TEST_ASSERT_EQUAL_UINT32(4, m.dropped(StreamId::ServoStatus));
}

void test_set_rate_changes_interval() {
  SubscriptionManager m;
  m.subscribe(StreamId::Health, 1);  // 1 Hz, interval 1000 ms
  TEST_ASSERT_TRUE(m.shouldEmit(StreamId::Health, 0));  // prime
  TEST_ASSERT_FALSE(m.shouldEmit(StreamId::Health, 500));
  // Raise to 10 Hz; phase re-primes so next call emits.
  TEST_ASSERT_EQUAL_UINT16(10, m.setRate(StreamId::Health, 10));
  TEST_ASSERT_TRUE(m.shouldEmit(StreamId::Health, 600));   // prime at new rate
  TEST_ASSERT_FALSE(m.shouldEmit(StreamId::Health, 605));  // <100 ms
  TEST_ASSERT_TRUE(m.shouldEmit(StreamId::Health, 700));   // +100 ms
}

void test_reset_clears_state() {
  SubscriptionManager m;
  m.subscribe(StreamId::Health, 10);
  m.shouldEmit(StreamId::Health, 0);
  m.noteTxBacklog();
  m.reset();
  TEST_ASSERT_FALSE(m.enabled(StreamId::Health));
  TEST_ASSERT_EQUAL_UINT32(0, m.emitted(StreamId::Health));
  TEST_ASSERT_EQUAL_UINT32(0, m.txBacklog());
}

void test_handle_subscribe_command() {
  SubscriptionManager m;
  // SUBSCRIBE Health(0) @ 5 Hz.
  uint8_t req[3] = {static_cast<uint8_t>(StreamId::Health), 5, 0};
  uint8_t out[64];
  uint16_t out_len = 0;
  uint8_t out_flags = 0xFF;
  const bool handled = m.handle(telemsg::kSubscribe, req, sizeof(req), out,
                                sizeof(out), &out_len, &out_flags);
  TEST_ASSERT_TRUE(handled);
  TEST_ASSERT_EQUAL_UINT8(0, out_flags);
  TEST_ASSERT_EQUAL_UINT16(4, out_len);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SubResult::Ok), out[0]);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(StreamId::Health), out[1]);
  const uint16_t eff = static_cast<uint16_t>(out[2] | (out[3] << 8));
  TEST_ASSERT_EQUAL_UINT16(5, eff);
  TEST_ASSERT_TRUE(m.enabled(StreamId::Health));
}

void test_handle_subscribe_clamps_in_response() {
  SubscriptionManager m;
  uint8_t req[3] = {static_cast<uint8_t>(StreamId::ServoStatus), 0xFF, 0xFF};
  uint8_t out[64];
  uint16_t out_len = 0;
  uint8_t out_flags = 0xFF;
  m.handle(telemsg::kSubscribe, req, sizeof(req), out, sizeof(out), &out_len,
           &out_flags);
  const uint16_t eff = static_cast<uint16_t>(out[2] | (out[3] << 8));
  TEST_ASSERT_EQUAL_UINT16(50, eff);  // ServoStatus max
}

void test_handle_bad_stream() {
  SubscriptionManager m;
  uint8_t req[3] = {99, 5, 0};  // invalid stream id
  uint8_t out[64];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  m.handle(telemsg::kSubscribe, req, sizeof(req), out, sizeof(out), &out_len,
           &out_flags);
  TEST_ASSERT_EQUAL_UINT8(kErrFlag, out_flags);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SubResult::BadStream), out[0]);
}

void test_handle_short_request() {
  SubscriptionManager m;
  uint8_t req[1] = {0};
  uint8_t out[64];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  m.handle(telemsg::kSubscribe, req, 1, out, sizeof(out), &out_len, &out_flags);
  TEST_ASSERT_EQUAL_UINT8(kErrFlag, out_flags);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SubResult::BadRequest), out[0]);
}

void test_handle_unsubscribe_command() {
  SubscriptionManager m;
  m.subscribe(StreamId::RcInput, 10);
  uint8_t req[1] = {static_cast<uint8_t>(StreamId::RcInput)};
  uint8_t out[64];
  uint16_t out_len = 0;
  uint8_t out_flags = 0xFF;
  const bool handled = m.handle(telemsg::kUnsubscribe, req, 1, out, sizeof(out),
                                &out_len, &out_flags);
  TEST_ASSERT_TRUE(handled);
  TEST_ASSERT_EQUAL_UINT8(0, out_flags);
  TEST_ASSERT_FALSE(m.enabled(StreamId::RcInput));
}

void test_handle_get_stream_stats() {
  SubscriptionManager m;
  m.subscribe(StreamId::Health, 10);
  m.shouldEmit(StreamId::Health, 0);    // emit 1
  m.noteTxBacklog();                    // backlog 1
  uint8_t out[128];
  uint16_t out_len = 0;
  uint8_t out_flags = 0xFF;
  const bool handled = m.handle(telemsg::kGetStreamStats, nullptr, 0, out,
                                sizeof(out), &out_len, &out_flags);
  TEST_ASSERT_TRUE(handled);
  TEST_ASSERT_EQUAL_UINT8(0, out_flags);
  TEST_ASSERT_EQUAL_UINT8(kNumStreams, out[0]);
  const uint32_t backlog =
      out[1] | (out[2] << 8) | (out[3] << 16) | (out[4] << 24);
  TEST_ASSERT_EQUAL_UINT32(1, backlog);
  // First stream block (Health, id 0) starts at offset 5.
  TEST_ASSERT_EQUAL_UINT8(0, out[5]);       // stream id
  TEST_ASSERT_EQUAL_UINT8(1, out[6]);       // enabled
  const uint16_t rate = static_cast<uint16_t>(out[7] | (out[8] << 8));
  TEST_ASSERT_EQUAL_UINT16(10, rate);
  const uint32_t emitted =
      out[9] | (out[10] << 8) | (out[11] << 16) | (out[12] << 24);
  TEST_ASSERT_EQUAL_UINT32(1, emitted);
}

void test_handle_rejects_non_telemetry_msg() {
  SubscriptionManager m;
  uint8_t out[16];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  TEST_ASSERT_FALSE(
      m.handle(0x01, nullptr, 0, out, sizeof(out), &out_len, &out_flags));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_subscribe_sets_enabled_and_rate);
  RUN_TEST(test_subscribe_clamps_to_stream_max);
  RUN_TEST(test_subscribe_clamps_to_min_rate);
  RUN_TEST(test_unsubscribe_disables);
  RUN_TEST(test_should_emit_enforces_rate);
  RUN_TEST(test_missed_slots_counted_as_dropped);
  RUN_TEST(test_set_rate_changes_interval);
  RUN_TEST(test_reset_clears_state);
  RUN_TEST(test_handle_subscribe_command);
  RUN_TEST(test_handle_subscribe_clamps_in_response);
  RUN_TEST(test_handle_bad_stream);
  RUN_TEST(test_handle_short_request);
  RUN_TEST(test_handle_unsubscribe_command);
  RUN_TEST(test_handle_get_stream_stats);
  RUN_TEST(test_handle_rejects_non_telemetry_msg);
  return UNITY_END();
}
