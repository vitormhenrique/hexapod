#pragma once

// ===========================================================================
// Command-source arbitration and authority management (portable, no Arduino).
//
// Three clients may want to move the robot: the RC link (ExpressLRS), the
// Jetson autonomy layer, and the Mac companion (maintenance). Exactly one
// source may hold motion authority at a time, and the firmware -- not any
// client -- decides who that is (AGENTS.md 1.1 / 6.4). This module encodes the
// priority and heartbeat rules so they can be unit-tested on the host:
//
//   * RC kill / host estop overrides everything and revokes any lock.
//   * A Mac maintenance lock (token + TTL) wins over RC/Jetson while held, but
//     is only granted when the robot is disarmed and not killed, and expires
//     safely if its heartbeat goes stale.
//   * Jetson authority requires BOTH a fresh Jetson heartbeat AND the RC
//     autonomy-enable switch AND RC armed.
//   * Otherwise RC manual owns motion when armed; when disarmed nobody does.
//
// The class holds only authority state; the control task feeds it RC inputs +
// the current time each cycle and reads back the active source. Heap-free.
// ===========================================================================

#include <stdint.h>

namespace safety {

// Who currently owns motion authority. Numeric values are surfaced in telemetry
// (active command source), so do not renumber.
enum class CommandSource : uint8_t {
  None = 0,            // nobody: motion denied (disarmed / killed / no link)
  Rc = 1,              // RC manual walking
  Jetson = 2,          // Jetson-assisted (RC-approved + fresh heartbeat)
  MacMaintenance = 3,  // Mac maintenance lock (low-authority bench control)
};

// Tunable timeouts.
struct ArbiterParams {
  uint32_t jetson_ttl_ms = 250;     // max Jetson heartbeat age for authority
  uint32_t mac_lock_ttl_ms = 1000;  // maintenance lock expiry without heartbeat
};

// RC-derived inputs for one arbitration cycle (filled from crsf::RcStatus).
struct RcInputs {
  bool ever_seen = false;        // any valid RC frame seen since boot
  bool kill = false;             // kill switch OR failsafe (hard stop)
  bool armed = false;            // arm switch high and not failsafe
  bool autonomy_enabled = false; // AUX switch granting Jetson authority
};

// Result of an arbitration cycle.
struct ArbiterOutput {
  CommandSource source = CommandSource::None;
  bool motion_authorized = false;  // a source may drive servos this cycle
  bool kill_active = false;        // RC kill / host estop asserted
  bool mac_lock_held = false;      // a valid maintenance lock exists
  uint32_t mac_lock_token = 0;     // nonzero token of the lock holder, else 0
};

class CommandArbiter {
 public:
  CommandArbiter() { reset(); }

  void configure(const ArbiterParams& p) { params_ = p; }
  void reset();

  // --- Mac maintenance lock -------------------------------------------------
  // Request the lock at `now_ms`. Granted only when no other lock is held, RC
  // kill / host estop is not active, and RC is not armed (not walking). Returns
  // a nonzero token on success, 0 on denial.
  uint32_t requestMacLock(uint32_t now_ms);
  // Refresh the lock TTL. Returns false if `token` is not the active holder.
  bool macLockHeartbeat(uint32_t token, uint32_t now_ms);
  // Release the lock if `token` currently holds it (no-op otherwise).
  void releaseMacLock(uint32_t token);
  bool macLockHeld(uint32_t now_ms) const { return macLockValid(now_ms); }

  // --- Jetson heartbeat -----------------------------------------------------
  void jetsonHeartbeat(uint32_t now_ms);
  bool jetsonFresh(uint32_t now_ms) const;

  // --- Host (Mac/USB) emergency stop ---------------------------------------
  void setHostEstop(bool on) { host_estop_ = on; }
  bool hostEstop() const { return host_estop_; }

  // Evaluate the active command source for this cycle and return the result.
  const ArbiterOutput& update(const RcInputs& rc, uint32_t now_ms);
  const ArbiterOutput& output() const { return out_; }

 private:
  bool macLockValid(uint32_t now_ms) const;
  void revokeMacLock();

  ArbiterParams params_;
  ArbiterOutput out_;

  bool jetson_seen_ = false;
  uint32_t jetson_last_ms_ = 0;

  bool mac_lock_active_ = false;
  uint32_t mac_lock_token_ = 0;
  uint32_t mac_lock_last_ms_ = 0;
  uint32_t next_token_ = 1;  // monotonically increasing, never 0

  bool host_estop_ = false;
  bool last_kill_ = true;   // start safe (killed)
  bool last_armed_ = false;
};

}  // namespace safety
