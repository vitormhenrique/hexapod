#pragma once

// ===========================================================================
// DYNAMIXEL maintenance command group (USB API, AGENTS.md 6.2 "Maintenance" /
// "DXL parameters": DXL_SCAN, DXL_PING, DXL_TORQUE, DXL_GET_SERVO_PROFILE).
//
// The DXL bus is Arduino-only and is owned exclusively by dxlTask (AGENTS.md
// 5.1: only one task may touch Dynamixel2Arduino / Serial1). The USB API runs
// in apiTask, a different task, so it must NOT touch the bus directly. Instead
// this handler enqueues a *job* into a small single-slot, single-producer /
// single-consumer queue; dxlTask claims the job, runs it against the bus, and
// writes the result back. The host retrieves the outcome by polling DXL_GET_
// RESULT with the returned job id.
//
// Split of responsibilities:
//   * DxlJobQueue   - portable SPSC handoff (no Arduino deps, fully host-tested).
//   * DxlJobApi     - portable framing: submit on DXL_SCAN/PING/TORQUE/PROFILE,
//                     poll on DXL_GET_RESULT, gated on the maintenance lock.
//   * dxlTask       - the Arduino executor: claim() a job, drive DxlBus, then
//                     complete() with the serialized result (lives in tasks.cpp).
//
// Gating: submitting a job requires the live safety state to be MacMaintenance
// AND the maintenance lock to be held -- the same chain as the maintenance
// target group. DXL_GET_RESULT is always allowed so the host can still retrieve
// a result if the state changed after submission.
//
// All payloads little-endian.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace protocol {

// DXL maintenance msg-ids. Reserve the whole 0x60-0x6F block for the DXL group;
// the param/register children (ubs.4.4 / ubs.4.5) extend it.
namespace dxlmsg {
constexpr uint8_t kScan = 0x60;
constexpr uint8_t kPing = 0x61;
constexpr uint8_t kTorque = 0x62;
constexpr uint8_t kGetServoProfile = 0x63;
constexpr uint8_t kGetResult = 0x64;
constexpr uint8_t kGetParam = 0x65;
constexpr uint8_t kSetParam = 0x66;
constexpr uint8_t kSetServoLimits = 0x67;
constexpr uint8_t kReadRegister = 0x68;
constexpr uint8_t kWriteRegister = 0x69;
constexpr uint8_t kFirst = 0x60;
constexpr uint8_t kLast = 0x6F;
inline bool isDxlMsg(uint8_t id) { return id >= kFirst && id <= kLast; }
}  // namespace dxlmsg

namespace dxljob {

// Job kind. `None` marks an empty slot.
enum class Type : uint8_t {
  None = 0,
  Scan = 1,        // arg0=first_id, arg1=last_id
  Ping = 2,        // arg0=id
  Torque = 3,      // arg0=on (0/1) -> all discovered servos
  GetProfile = 4,  // arg0=id
  GetParam = 5,    // arg0=id, param=LogicalParam
  SetParam = 6,    // arg0=id, param=LogicalParam, val_a=value
  SetLimits = 7,   // arg0=id, val_a=min_tick, val_b=max_tick
  ReadReg = 8,     // arg0=id, val_a=addr, param=len (raw, expert-gated)
  WriteReg = 9,    // arg0=id, val_a=addr, param=len, val_b=value, arg1=flags
};

// Single-slot lifecycle, written only on a state transition (the SPSC fence).
enum class Slot : uint8_t {
  Empty = 0,    // no job; producer may submit
  Pending = 1,  // submitted, awaiting the executor
  Running = 2,  // claimed by the executor, in flight
  Done = 3,     // executor finished; result is readable, producer may re-submit
};

// Result code filled by the executor (the Arduino side maps bus outcomes here).
enum class Code : uint8_t {
  Ok = 0,
  NotFound = 1,    // ping/profile/param: no servo answered
  PowerOff = 2,    // DXL power is off, the bus cannot be scanned
  BusError = 3,    // a bus/library transaction failed
  Unsupported = 4,  // job type / parameter not available on this table
  VerifyFailed = 5,  // write succeeded but read-back did not match
};

// Largest serialized job result: a full scan returns 1 count byte + up to 24
// compact servo records (6 bytes each) = 145 bytes; round up for headroom.
constexpr uint8_t kMaxResult = 160;

}  // namespace dxljob

// One enqueued job request (producer -> consumer).
struct DxlJobRequest {
  dxljob::Type type = dxljob::Type::None;
  uint8_t arg0 = 0;
  uint8_t arg1 = 0;
  uint8_t param = 0;     // LogicalParam value for GetParam/SetParam, or raw
                         // register byte length (1/2/4) for ReadReg/WriteReg
  int32_t val_a = 0;     // SetParam value, min_tick for SetLimits, or raw addr
  int32_t val_b = 0;     // max_tick for SetLimits, or raw write value
};

// One serialized job result (consumer -> producer).
struct DxlJobResult {
  dxljob::Code code = dxljob::Code::Ok;
  uint8_t len = 0;
  uint8_t data[dxljob::kMaxResult] = {0};
};

// Single-slot SPSC queue handing a DXL job from apiTask to dxlTask and the
// result back. Safe for one producer (apiTask: submit/poll) and one consumer
// (dxlTask: claim/complete) on the SAMD21: the volatile `slot_` transition is
// the release/acquire fence and the payloads are only written by the side that
// owns the slot in that phase.
class DxlJobQueue {
 public:
  // --- Producer (apiTask) ---------------------------------------------------
  // Submit a job. Succeeds only when the slot is free (Empty) or holds an
  // already-finished result (Done, which is overwritten). Returns false when a
  // job is still Pending/Running. On success assigns a fresh non-zero job id.
  bool submit(const DxlJobRequest& req, uint8_t& job_id_out);

  // Poll the slot. Returns the current lifecycle for `job_id`; if it does not
  // match the live job the slot is treated as Empty (the host's job is gone).
  // When the state is Done, copies the result into `out`.
  dxljob::Slot poll(uint8_t job_id, DxlJobResult& out) const;

  // --- Consumer (dxlTask) ---------------------------------------------------
  // Claim a pending job for execution. Returns true and fills `req`/`job_id`
  // when a job moved Pending -> Running; false when there is nothing to do.
  bool claim(DxlJobRequest& req, uint8_t& job_id);

  // Report a finished job. Ignored unless the slot is Running with a matching
  // id. `data`/`len` is the serialized result (len clamped to kMaxResult).
  void complete(uint8_t job_id, dxljob::Code code, const uint8_t* data,
                uint8_t len);

  // Test/maintenance helpers.
  void reset();
  dxljob::Slot slotState() const { return slot_; }
  uint8_t currentJobId() const { return job_id_; }

 private:
  // Next non-zero job id (0 is reserved for "no job").
  uint8_t nextJobId();

  volatile dxljob::Slot slot_ = dxljob::Slot::Empty;
  uint8_t job_id_ = 0;
  uint8_t next_id_ = 0;
  DxlJobRequest req_;
  DxlJobResult result_;
};

// Submit/poll result for the DXL_* submit commands.
enum class DxlSubmit : uint8_t {
  Accepted = 0,    // queued; poll DXL_GET_RESULT with the returned job id
  Rejected = 1,    // not in MacMaintenance / lock not held
  Busy = 2,        // a job is already in flight
  BadRequest = 3,  // malformed payload
};

// Thin framing wrapper over a DxlJobQueue: parses submit commands and the
// poll command, enforces the maintenance gate, and builds the responses. The
// executor (dxlTask) talks to queue() directly.
class DxlJobApi {
 public:
  DxlJobApi() { reset(); }

  void reset();

  // Refresh the gate: the live safety state and whether the maintenance lock is
  // held. Submits are only honored in MacMaintenance with the lock held.
  void setLiveState(uint8_t state, bool lock_held) {
    live_state_ = state;
    lock_held_ = lock_held;
  }

  // Enable/disable the expert-gated raw register commands (DXL_READ_REGISTER /
  // DXL_WRITE_REGISTER). Disabled by default: raw access bypasses the logical
  // parameter table and is for diagnostics only. Submits are Rejected unless
  // this is enabled AND the normal maintenance gate is open.
  void setRawRegisterEnabled(bool enabled) { raw_register_enabled_ = enabled; }
  bool rawRegisterEnabled() const { return raw_register_enabled_; }

  // Access the underlying queue (used by the dxlTask executor and tests).
  DxlJobQueue& queue() { return queue_; }
  const DxlJobQueue& queue() const { return queue_; }

  // Dispatch a DXL maintenance command. Returns false if `msg_id` is outside
  // the DXL range so the api dispatcher can try the next group.
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, size_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

 private:
  // Build a submit response: [result, job_id, slot].
  bool writeSubmit(DxlSubmit r, uint8_t job_id, dxljob::Slot slot, uint8_t* out,
                   size_t out_cap, uint16_t* out_len, uint8_t* out_flags) const;

  bool gateOpen() const {
    return lock_held_ && live_state_ == kMacMaintenanceState;
  }

  static constexpr uint8_t kMacMaintenanceState = 8;  // safety::State value

  DxlJobQueue queue_;
  uint8_t live_state_ = 0;
  bool lock_held_ = false;
  bool raw_register_enabled_ = false;
};

}  // namespace protocol
