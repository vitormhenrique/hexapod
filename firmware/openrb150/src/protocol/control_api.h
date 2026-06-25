#pragma once

// ===========================================================================
// Control API: safety command group over the USB protocol (portable).
//
// Exposes the host-facing safety controls from AGENTS.md 6.2 "Safety" group:
// ESTOP, CLEAR_FAULT, SET_ARMING, SET_MODE. The firmware safety state machine
// (safety/state_machine.h) remains the sole authority: this handler only
// captures host *intent* as a few latched/one-shot flags that the control task
// folds into the machine's inputs each cycle. It never commands a servo and it
// can never widen authority beyond what the machine and RC allow:
//
//   * ESTOP        -> latch a host E-stop (machine -> Estop).
//   * CLEAR_FAULT  -> release the host E-stop latch and pulse the machine's
//                     clear-fault request (only releases once the underlying
//                     condition is gone).
//   * SET_ARMING   -> Disarm latches a host force-disarm (always honored, it
//                     only ever reduces authority). Arm merely releases that
//                     latch; real walking-arm still requires the RC arm switch
//                     (AGENTS.md 1.1 / 5.3), so Arm never arms by itself.
//   * SET_MODE     -> only the safety-reducing modes (Disarmed, Estop) are
//                     honored here; richer modes are reached through the RC
//                     switch or the dedicated maintenance/passive commands.
//
// Responses echo [result, state, fault] using the live state/fault the control
// task publishes via setLiveState(). State/fault are plain wire bytes
// (safety::State / safety::FaultReason) so this layer stays decoupled from the
// safety headers. No Arduino deps; no heap; host unit-tested.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace protocol {

// Protocol message IDs for the control command group. The control groups live
// in 0x30..0x3F (below the telemetry frame base 0x40, above config 0x20..0x25).
namespace ctrlmsg {
constexpr uint8_t kEstop = 0x30;
constexpr uint8_t kClearFault = 0x31;
constexpr uint8_t kSetArming = 0x32;
constexpr uint8_t kSetMode = 0x33;

constexpr uint8_t kFirst = kEstop;
constexpr uint8_t kLast = kSetMode;

// True if `msg_id` belongs to the safety control command group.
constexpr bool isControlMsg(uint8_t msg_id) {
  return msg_id >= kFirst && msg_id <= kLast;
}
}  // namespace ctrlmsg

// SET_ARMING request byte.
enum class ArmingRequest : uint8_t {
  Disarm = 0,
  Arm = 1,
};

// Result byte returned in control responses.
enum class CtrlResult : uint8_t {
  Ok = 0,         // request accepted (intent recorded)
  Rejected = 1,   // understood but not permitted by safety policy
  BadRequest = 2,  // malformed payload
};

class ControlApi {
 public:
  ControlApi() { reset(); }

  // Clear all latched intent. Call once at task start.
  void reset();

  // Publish the latest authoritative state/fault (wire bytes) so command
  // responses can echo them. Called each cycle by the control task.
  void setLiveState(uint8_t state, uint8_t fault) {
    live_state_ = state;
    live_fault_ = fault;
  }

  // --- Host intent consumed by the control task ----------------------------
  // Latched host E-stop. Folds into StateInputs.host_estop.
  bool estopActive() const { return estop_; }
  // Latched host force-disarm. Folds into StateInputs.host_disarm.
  bool disarmRequested() const { return disarm_; }
  // One-shot clear-fault request: returns true at most once per CLEAR_FAULT,
  // then auto-clears. Drive safety::StateMachine::requestClearFault() with it.
  bool consumeClearFault();

  // --- Command handling ----------------------------------------------------
  // Handle one control command. Returns true if `msg_id` is in the control
  // group (response written to out/out_len/out_flags), false otherwise.
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, uint16_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

 private:
  // Write a 3-byte [result, state, fault] success payload.
  void writeStatus(uint8_t* out, uint16_t* out_len, CtrlResult result);

  bool estop_ = false;
  bool disarm_ = false;
  bool clear_fault_ = false;
  uint8_t live_state_ = 0;
  uint8_t live_fault_ = 0;
};

}  // namespace protocol
