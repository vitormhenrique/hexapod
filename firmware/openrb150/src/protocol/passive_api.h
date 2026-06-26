#pragma once

// ===========================================================================
// Passive pose streaming command group (USB API, AGENTS.md 5.5 / 6.2 "Passive
// pose"). Portable, no Arduino deps.
//
// Lets a host put the robot in torque-off passive pose streaming: every servo
// torque is disabled, no goal writes happen, and the firmware keeps streaming
// present positions so a user can physically pose the robot and watch the model
// follow (companion / URDF / ROS 2). The commands here only capture host intent;
// the safety state machine (safety/state_machine.h) is the sole authority that
// actually transitions to State::PassivePoseStream, and it only does so once
// torque is confirmed off (StateInputs.passive_request && torque_off).
//
//   PASSIVE_ENTER            (0x80): request passive streaming. Allowed only
//                                    from a maintenance-safe live state
//                                    (Disarmed / MacMaintenance) or while
//                                    already streaming (idempotent).
//   PASSIVE_EXIT             (0x81): drop the request (always honoured; it only
//                                    ever reduces authority).
//   PASSIVE_SET_STREAM_RATE  (0x82): stage the present-position stream rate
//                                    (Hz); only while passive is requested.
//   PASSIVE_ZERO_REFERENCE   (0x83): request a present-position zero-reference
//                                    capture (seq mailbox); only while passive
//                                    is requested.
//
// The control task reads requested()/streamRateHz()/zeroSeq() each cycle and
// folds the request into the state machine. Motion/gait/IK commands are rejected
// while passive is active (see MotionApi). Responses follow the [result, state]
// convention used by the other control groups; state is the live safety::State
// wire byte published via setLiveState(). All payloads little-endian; host
// unit-tested.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace protocol {

// Passive pose command msg-ids. The 0x80..0x83 block is reserved for passive
// pose streaming (above the sensor block 0x70-0x7F).
namespace passivemsg {
constexpr uint8_t kEnter = 0x80;
constexpr uint8_t kExit = 0x81;
constexpr uint8_t kSetStreamRate = 0x82;
constexpr uint8_t kZeroReference = 0x83;
constexpr uint8_t kFirst = kEnter;
constexpr uint8_t kLast = kZeroReference;
inline bool isPassiveMsg(uint8_t id) { return id >= kFirst && id <= kLast; }
}  // namespace passivemsg

enum class PassiveResult : uint8_t {
  Ok = 0,         // request accepted (intent recorded)
  Rejected = 1,   // understood but not permitted by safety policy / state
  BadRequest = 2,  // malformed payload
};

// Safe live-state wire bytes (mirror safety::State) from which passive pose
// streaming may be entered. Kept numeric so this layer stays free of the safety
// headers.
namespace passivestate {
constexpr uint8_t kDisarmed = 2;
constexpr uint8_t kMacMaintenance = 8;
constexpr uint8_t kPassivePoseStream = 9;
}  // namespace passivestate

// Present-position stream rate bounds (Hz) staged by PASSIVE_SET_STREAM_RATE.
namespace passiverate {
constexpr uint16_t kDefaultHz = 50;
constexpr uint16_t kMinHz = 1;
constexpr uint16_t kMaxHz = 200;
}  // namespace passiverate

class PassiveApi {
 public:
  PassiveApi() { reset(); }

  // Clear all latched intent. Call once at task start.
  void reset();

  // Publish the live safety state (wire byte) so ENTER can gate on a safe state
  // and responses can echo it. Called each cycle by the control task.
  void setLiveState(uint8_t state) { live_state_ = state; }

  // --- State consumed by the control task ----------------------------------
  // Latched passive-streaming request. Folds into StateInputs.passive_request.
  bool requested() const { return requested_; }
  // Staged present-position stream rate (Hz).
  uint16_t streamRateHz() const { return rate_hz_; }
  // Monotonic zero-reference request counter; bumps on each PASSIVE_ZERO_
  // REFERENCE. A consumer captures the present positions as the neutral pose
  // when this advances.
  uint32_t zeroSeq() const { return zero_seq_; }

  // Force-clear the passive request (e.g. on E-stop / fault). It only ever
  // reduces authority, so it is always safe to call.
  void clear() { requested_ = false; }

  // --- Command handling ----------------------------------------------------
  // Handle one passive command. Returns false if `msg_id` is not in the passive
  // range (so the dispatcher can try the next group). On a handled command
  // writes the response and returns true.
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, size_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

 private:
  bool canEnter() const {
    return live_state_ == passivestate::kDisarmed ||
           live_state_ == passivestate::kMacMaintenance ||
           live_state_ == passivestate::kPassivePoseStream;
  }
  // [result, state] (+ optional trailing rate u16).
  bool writeResult(PassiveResult r, uint8_t* out, size_t out_cap,
                   uint16_t* out_len, uint8_t* out_flags,
                   const uint8_t* extra = nullptr, uint16_t extra_len = 0) const;

  bool requested_ = false;
  uint16_t rate_hz_ = passiverate::kDefaultHz;
  uint32_t zero_seq_ = 0;
  uint8_t live_state_ = 0;
};

}  // namespace protocol
