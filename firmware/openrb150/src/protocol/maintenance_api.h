#pragma once

// ===========================================================================
// Maintenance lock command group (USB API, AGENTS.md 6.4 "USB client
// arbitration" + 6.2 "Maintenance"). Portable, no Arduino deps.
//
// Owns the maintenance lock: a USB-client construct that lets the Mac request
// low-authority bench control (MacMaintenance). The lock is a token + TTL:
//
//   ENTER_MAINTENANCE (0x50): grant a fresh nonzero token when the robot is in
//                             a safe entry state (Disarmed / StandReady) and no
//                             other valid lock is held. Response carries token.
//   EXIT_MAINTENANCE  (0x51): release the lock if the request token matches.
//   MAINT_HEARTBEAT   (0x52): refresh the lock TTL (token must match + valid).
//
// The handler ONLY owns the lock; it never commands a servo. The control task
// reads lockHeld()/maintenance intent each cycle and feeds it to the safety
// state machine (StateInputs.maintenance_request / mac_lock_held), which is the
// sole authority for entering/leaving MacMaintenance. The lock auto-expires
// without a heartbeat and is force-revoked on E-stop / fault.
//
// Responses follow the [result, state, ...] convention used by the other
// control groups; state is the live safety::State wire byte published via
// setLiveState(). All payloads little-endian; host unit-tested.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace protocol {

// Maintenance command msg-ids. The whole 0x50..0x5F block is reserved for the
// maintenance group (lock here; leg/joint targets land in a sibling child).
namespace maintmsg {
constexpr uint8_t kEnter = 0x50;
constexpr uint8_t kExit = 0x51;
constexpr uint8_t kHeartbeat = 0x52;
constexpr uint8_t kFirst = 0x50;
constexpr uint8_t kLast = 0x5F;
inline bool isMaintMsg(uint8_t id) { return id >= kFirst && id <= kLast; }
}  // namespace maintmsg

enum class MaintResult : uint8_t {
  Ok = 0,         // request accepted
  Rejected = 1,   // not permitted by safety state (wrong robot state)
  BadRequest = 2,  // malformed payload
  Busy = 3,       // a different valid lock is already held
  BadToken = 4,   // EXIT/HEARTBEAT token does not match the active lock
};

// Safe live-state wire bytes (mirror safety::State) in which a maintenance lock
// may be acquired. Kept as numerics so this layer stays free of safety headers.
namespace maintstate {
constexpr uint8_t kDisarmed = 2;
constexpr uint8_t kStandReady = 4;
constexpr uint8_t kMacMaintenance = 8;
}  // namespace maintstate

class MaintenanceApi {
 public:
  // Default lock TTL: a heartbeat must arrive within this window or the lock
  // lapses (AGENTS.md 6.4 "Lock expires unless heartbeat is fresh").
  static constexpr uint32_t kDefaultTtlMs = 1000;

  MaintenanceApi() { reset(); }

  void reset();

  // Provide the current uptime (ms) used for TTL evaluation inside handle().
  // The control/api task sets this before each handleRequest.
  void setNow(uint32_t now_ms) { now_ms_ = now_ms; }

  // Publish the live safety state (wire byte) so ENTER can gate on it and
  // responses can echo it.
  void setLiveState(uint8_t state) { live_state_ = state; }

  void setTtlMs(uint32_t ttl_ms) { ttl_ms_ = ttl_ms; }

  // --- State consumed by the control task ----------------------------------
  // True while a valid (unexpired) maintenance lock is held at `now_ms`.
  bool lockHeld(uint32_t now_ms) const {
    return locked_ && (now_ms - last_hb_ms_) <= ttl_ms_;
  }
  uint32_t token() const { return token_; }
  // Force-release the lock (call on E-stop / fault).
  void revoke();

  // --- Command handling ----------------------------------------------------
  // Handle one maintenance command. Returns false if `msg_id` is not in the
  // maintenance range (so the dispatcher can try the next group). Unimplemented
  // ids within the range also return false (-> UnknownMsg).
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, size_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

 private:
  bool canEnter() const {
    return live_state_ == maintstate::kDisarmed ||
           live_state_ == maintstate::kStandReady ||
           live_state_ == maintstate::kMacMaintenance;
  }
  // Common short responses: [result, state] (+ optional trailing token).
  bool writeResult(MaintResult r, uint8_t* out, size_t out_cap,
                   uint16_t* out_len, uint8_t* out_flags) const;

  bool locked_ = false;
  uint32_t token_ = 0;
  uint32_t next_token_ = 1;  // monotonically increasing, never 0
  uint32_t last_hb_ms_ = 0;
  uint32_t ttl_ms_ = kDefaultTtlMs;
  uint32_t now_ms_ = 0;
  uint8_t live_state_ = 0;
};

}  // namespace protocol
