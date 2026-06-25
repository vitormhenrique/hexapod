// Native (host) Unity tests for the DXL maintenance command group
// (DXL_SCAN / DXL_PING / DXL_TORQUE / DXL_GET_SERVO_PROFILE / DXL_GET_RESULT),
// covering both the portable single-slot job queue (DxlJobQueue) and the
// framing/gating wrapper (DxlJobApi). The Arduino bus executor lives in
// tasks.cpp and is exercised on hardware; here we simulate it by driving
// claim()/complete() directly.
//
// Run with:  pio test -e native -f test_dxl_job_api

#include <string.h>

#include <unity.h>

#include "protocol/dxl_job_api.h"
#include "protocol/framing.h"

using namespace protocol;

namespace {

constexpr uint8_t kMacMaintenance = 8;
constexpr uint8_t kDisarmed = 2;
constexpr uint8_t kErrorFlag = 0x02;

// Call a DXL handler command directly (no framing) and return the handled flag.
bool runDxl(DxlJobApi& api_obj, uint8_t msg_id, const uint8_t* req,
            uint16_t req_len, uint8_t* out, uint16_t* out_len,
            uint8_t* out_flags) {
  return api_obj.handle(msg_id, req, req_len, out, kMaxPayload, out_len,
                        out_flags);
}

}  // namespace

// --- DxlJobQueue ------------------------------------------------------------

void test_queue_submit_claim_complete_poll_roundtrip() {
  DxlJobQueue q;
  q.reset();
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Empty),
                    static_cast<int>(q.slotState()));

  DxlJobRequest req;
  req.type = dxljob::Type::Ping;
  req.arg0 = 7;
  uint8_t id = 0;
  TEST_ASSERT_TRUE(q.submit(req, id));
  TEST_ASSERT_NOT_EQUAL(0, id);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Pending),
                    static_cast<int>(q.slotState()));

  // Poll while pending: no result yet.
  DxlJobResult res;
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Pending),
                    static_cast<int>(q.poll(id, res)));

  // Consumer claims it.
  DxlJobRequest got;
  uint8_t got_id = 0;
  TEST_ASSERT_TRUE(q.claim(got, got_id));
  TEST_ASSERT_EQUAL(id, got_id);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Type::Ping),
                    static_cast<int>(got.type));
  TEST_ASSERT_EQUAL(7, got.arg0);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Running),
                    static_cast<int>(q.slotState()));

  // Nothing else to claim now.
  TEST_ASSERT_FALSE(q.claim(got, got_id));

  // Consumer completes with a payload.
  const uint8_t payload[3] = {0xAA, 0xBB, 0xCC};
  q.complete(id, dxljob::Code::Ok, payload, 3);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Done),
                    static_cast<int>(q.slotState()));

  // Producer polls the result (idempotently).
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Done),
                    static_cast<int>(q.poll(id, res)));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Code::Ok),
                    static_cast<int>(res.code));
  TEST_ASSERT_EQUAL(3, res.len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, res.data, 3);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Done),
                    static_cast<int>(q.poll(id, res)));  // still readable
}

void test_queue_busy_while_in_flight() {
  DxlJobQueue q;
  q.reset();
  DxlJobRequest req;
  req.type = dxljob::Type::Scan;
  uint8_t id = 0;
  TEST_ASSERT_TRUE(q.submit(req, id));
  // Pending: a second submit is refused.
  uint8_t id2 = 0;
  TEST_ASSERT_FALSE(q.submit(req, id2));
  // Running: still refused.
  DxlJobRequest got;
  uint8_t got_id = 0;
  TEST_ASSERT_TRUE(q.claim(got, got_id));
  TEST_ASSERT_FALSE(q.submit(req, id2));
}

void test_queue_resubmit_overwrites_collected_result() {
  DxlJobQueue q;
  q.reset();
  DxlJobRequest req;
  req.type = dxljob::Type::Ping;
  req.arg0 = 1;
  uint8_t id1 = 0;
  TEST_ASSERT_TRUE(q.submit(req, id1));
  DxlJobRequest got;
  uint8_t got_id = 0;
  q.claim(got, got_id);
  q.complete(id1, dxljob::Code::Ok, nullptr, 0);
  // Done slot accepts a new job (overwrites), and the id advances.
  req.arg0 = 2;
  uint8_t id2 = 0;
  TEST_ASSERT_TRUE(q.submit(req, id2));
  TEST_ASSERT_NOT_EQUAL(id1, id2);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Pending),
                    static_cast<int>(q.slotState()));
  // The stale id no longer matches the live job.
  DxlJobResult res;
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Empty),
                    static_cast<int>(q.poll(id1, res)));
}

void test_queue_poll_unknown_id_is_empty() {
  DxlJobQueue q;
  q.reset();
  DxlJobResult res;
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Empty),
                    static_cast<int>(q.poll(0, res)));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Empty),
                    static_cast<int>(q.poll(99, res)));
}

void test_queue_complete_ignores_stale_id() {
  DxlJobQueue q;
  q.reset();
  DxlJobRequest req;
  req.type = dxljob::Type::Ping;
  uint8_t id = 0;
  q.submit(req, id);
  DxlJobRequest got;
  uint8_t got_id = 0;
  q.claim(got, got_id);
  // Completing with a wrong id is ignored; the slot stays Running.
  q.complete(static_cast<uint8_t>(id + 1), dxljob::Code::Ok, nullptr, 0);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Running),
                    static_cast<int>(q.slotState()));
}

// --- DxlJobApi gating + framing --------------------------------------------

void test_submit_rejected_when_not_in_maintenance() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kDisarmed, true);  // wrong state
  const uint8_t req[2] = {1, 18};
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kScan, req, 2, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(3, out_len);
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Rejected), out[0]);
  TEST_ASSERT_EQUAL(kErrorFlag, out_flags);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Empty),
                    static_cast<int>(api_obj.queue().slotState()));
}

void test_submit_rejected_when_lock_not_held() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, false);  // no lock
  const uint8_t req[1] = {5};
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kPing, req, 1, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Rejected), out[0]);
}

void test_scan_submit_accepted_and_executes() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, true);
  const uint8_t req[2] = {1, 18};
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kScan, req, 2, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(3, out_len);
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Accepted), out[0]);
  TEST_ASSERT_EQUAL(0, out_flags);
  const uint8_t job_id = out[1];
  TEST_ASSERT_NOT_EQUAL(0, job_id);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Pending), out[2]);

  // The host polling before execution sees Pending.
  uint8_t poll_req[1] = {job_id};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kGetResult, poll_req, 1, out,
                          &out_len, &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Pending), out[0]);
  TEST_ASSERT_EQUAL(3, out_len);

  // Simulate the dxlTask executor: claim, build a 2-servo scan result, finish.
  DxlJobRequest job;
  uint8_t got_id = 0;
  TEST_ASSERT_TRUE(api_obj.queue().claim(job, got_id));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Type::Scan),
                    static_cast<int>(job.type));
  TEST_ASSERT_EQUAL(1, job.arg0);
  TEST_ASSERT_EQUAL(18, job.arg1);
  const uint8_t result[13] = {2,                       // count
                              1, 29, 0, 40, 1, 1,      // servo 1
                              2, 30, 0, 42, 2, 2};     // servo 2
  api_obj.queue().complete(got_id, dxljob::Code::Ok, result, 13);

  // The host polls and gets the full result back.
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kGetResult, poll_req, 1, out,
                          &out_len, &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Done), out[0]);
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Code::Ok), out[1]);
  TEST_ASSERT_EQUAL(13, out[2]);
  TEST_ASSERT_EQUAL(3 + 13, out_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(result, &out[3], 13);
}

void test_get_result_allowed_even_when_gate_closed() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, true);
  const uint8_t req[1] = {5};
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kPing, req, 1, out, &out_len,
                          &out_flags));
  const uint8_t job_id = out[1];
  // Executor runs the ping.
  DxlJobRequest job;
  uint8_t got_id = 0;
  api_obj.queue().claim(job, got_id);
  const uint8_t result[7] = {5, 29, 0, 40, 1, 1, 1};
  api_obj.queue().complete(got_id, dxljob::Code::Ok, result, 7);
  // State leaves maintenance, but GET_RESULT must still return the outcome.
  api_obj.setLiveState(kDisarmed, false);
  uint8_t poll_req[1] = {job_id};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kGetResult, poll_req, 1, out,
                          &out_len, &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Done), out[0]);
  TEST_ASSERT_EQUAL(7, out[2]);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(result, &out[3], 7);
}

void test_submit_busy_when_job_in_flight() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, true);
  const uint8_t req[1] = {3};
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kPing, req, 1, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Accepted), out[0]);
  // A second submit before the first finishes is Busy.
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kTorque, req, 1, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Busy), out[0]);
  TEST_ASSERT_EQUAL(kErrorFlag, out_flags);
}

void test_bad_requests() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, true);
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  // Scan with last < first.
  const uint8_t bad_scan[2] = {10, 5};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kScan, bad_scan, 2, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::BadRequest), out[0]);
  // Ping id 0 (broadcast) is rejected.
  const uint8_t bad_ping[1] = {0};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kPing, bad_ping, 1, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::BadRequest), out[0]);
  // Torque with no payload.
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kTorque, nullptr, 0, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::BadRequest), out[0]);
  // No job was queued by any bad request.
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Slot::Empty),
                    static_cast<int>(api_obj.queue().slotState()));
}

void test_torque_and_profile_submit() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, true);
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  // Torque on.
  const uint8_t on[1] = {1};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kTorque, on, 1, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Accepted), out[0]);
  DxlJobRequest job;
  uint8_t got_id = 0;
  TEST_ASSERT_TRUE(api_obj.queue().claim(job, got_id));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Type::Torque),
                    static_cast<int>(job.type));
  TEST_ASSERT_EQUAL(1, job.arg0);
  api_obj.queue().complete(got_id, dxljob::Code::Ok, nullptr, 0);

  // GetServoProfile.
  const uint8_t pid[1] = {12};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kGetServoProfile, pid, 1, out,
                          &out_len, &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Accepted), out[0]);
  TEST_ASSERT_TRUE(api_obj.queue().claim(job, got_id));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Type::GetProfile),
                    static_cast<int>(job.type));
  TEST_ASSERT_EQUAL(12, job.arg0);
}

void test_get_param_submit() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, true);
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  // GET_PARAM id=3, param=4 (CcwAngleLimit).
  const uint8_t req[2] = {3, 4};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kGetParam, req, 2, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Accepted), out[0]);
  DxlJobRequest job;
  uint8_t got_id = 0;
  TEST_ASSERT_TRUE(api_obj.queue().claim(job, got_id));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Type::GetParam),
                    static_cast<int>(job.type));
  TEST_ASSERT_EQUAL(3, job.arg0);
  TEST_ASSERT_EQUAL(4, job.param);
  // Too-short request is BadRequest.
  const uint8_t bad[1] = {3};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kGetParam, bad, 1, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::BadRequest), out[0]);
}

void test_set_param_submit() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, true);
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  // SET_PARAM id=5, param=16 (MovingSpeed), value=0x000003E8 (1000) LE.
  const uint8_t req[6] = {5, 16, 0xE8, 0x03, 0x00, 0x00};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kSetParam, req, 6, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Accepted), out[0]);
  DxlJobRequest job;
  uint8_t got_id = 0;
  TEST_ASSERT_TRUE(api_obj.queue().claim(job, got_id));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Type::SetParam),
                    static_cast<int>(job.type));
  TEST_ASSERT_EQUAL(5, job.arg0);
  TEST_ASSERT_EQUAL(16, job.param);
  TEST_ASSERT_EQUAL_INT32(1000, job.val_a);
  // Too-short request is BadRequest.
  const uint8_t bad[5] = {5, 16, 0, 0, 0};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kSetParam, bad, 5, out, &out_len,
                          &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::BadRequest), out[0]);
}

void test_set_servo_limits_submit() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, true);
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  // SET_SERVO_LIMITS id=7, min=100, max=3900 (both i32 LE).
  const uint8_t req[9] = {7,
                          100, 0, 0, 0,
                          0x3C, 0x0F, 0, 0};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kSetServoLimits, req, 9, out,
                          &out_len, &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::Accepted), out[0]);
  DxlJobRequest job;
  uint8_t got_id = 0;
  TEST_ASSERT_TRUE(api_obj.queue().claim(job, got_id));
  TEST_ASSERT_EQUAL(static_cast<int>(dxljob::Type::SetLimits),
                    static_cast<int>(job.type));
  TEST_ASSERT_EQUAL(7, job.arg0);
  TEST_ASSERT_EQUAL_INT32(100, job.val_a);
  TEST_ASSERT_EQUAL_INT32(3900, job.val_b);
  // max < min is BadRequest (no job queued).
  const uint8_t inverted[9] = {7,
                               0x3C, 0x0F, 0, 0,
                               100, 0, 0, 0};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kSetServoLimits, inverted, 9, out,
                          &out_len, &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::BadRequest), out[0]);
  // Too-short request is BadRequest.
  const uint8_t bad[8] = {7, 0, 0, 0, 0, 0, 0, 0};
  TEST_ASSERT_TRUE(runDxl(api_obj, dxlmsg::kSetServoLimits, bad, 8, out,
                          &out_len, &out_flags));
  TEST_ASSERT_EQUAL(static_cast<int>(DxlSubmit::BadRequest), out[0]);
}

void test_handle_declines_out_of_range() {
  DxlJobApi api_obj;
  api_obj.reset();
  api_obj.setLiveState(kMacMaintenance, true);
  uint8_t out[kMaxPayload];
  uint16_t out_len = 0;
  uint8_t out_flags = 0;
  // 0x40 is outside the DXL block: handler declines.
  TEST_ASSERT_FALSE(runDxl(api_obj, 0x40, nullptr, 0, out, &out_len,
                           &out_flags));
  // 0x6F is the block tail but unassigned: declined too.
  TEST_ASSERT_FALSE(runDxl(api_obj, 0x6F, nullptr, 0, out, &out_len,
                           &out_flags));
}

void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_queue_submit_claim_complete_poll_roundtrip);
  RUN_TEST(test_queue_busy_while_in_flight);
  RUN_TEST(test_queue_resubmit_overwrites_collected_result);
  RUN_TEST(test_queue_poll_unknown_id_is_empty);
  RUN_TEST(test_queue_complete_ignores_stale_id);
  RUN_TEST(test_submit_rejected_when_not_in_maintenance);
  RUN_TEST(test_submit_rejected_when_lock_not_held);
  RUN_TEST(test_scan_submit_accepted_and_executes);
  RUN_TEST(test_get_result_allowed_even_when_gate_closed);
  RUN_TEST(test_submit_busy_when_job_in_flight);
  RUN_TEST(test_bad_requests);
  RUN_TEST(test_torque_and_profile_submit);
  RUN_TEST(test_get_param_submit);
  RUN_TEST(test_set_param_submit);
  RUN_TEST(test_set_servo_limits_submit);
  RUN_TEST(test_handle_declines_out_of_range);
  return UNITY_END();
}
