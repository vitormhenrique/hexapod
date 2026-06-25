#include "command_arbiter.h"

namespace safety {

void CommandArbiter::reset() {
  out_ = ArbiterOutput{};
  out_.kill_active = true;  // safe default until the first update()
  jetson_seen_ = false;
  jetson_last_ms_ = 0;
  mac_lock_active_ = false;
  mac_lock_token_ = 0;
  mac_lock_last_ms_ = 0;
  next_token_ = 1;
  host_estop_ = false;
  last_kill_ = true;
  last_armed_ = false;
}

bool CommandArbiter::jetsonFresh(uint32_t now_ms) const {
  return jetson_seen_ && (now_ms - jetson_last_ms_) <= params_.jetson_ttl_ms;
}

void CommandArbiter::jetsonHeartbeat(uint32_t now_ms) {
  jetson_seen_ = true;
  jetson_last_ms_ = now_ms;
}

bool CommandArbiter::macLockValid(uint32_t now_ms) const {
  return mac_lock_active_ &&
         (now_ms - mac_lock_last_ms_) <= params_.mac_lock_ttl_ms;
}

void CommandArbiter::revokeMacLock() {
  mac_lock_active_ = false;
  mac_lock_token_ = 0;
  mac_lock_last_ms_ = 0;
}

uint32_t CommandArbiter::requestMacLock(uint32_t now_ms) {
  // Deny while killed (or host estop), while RC is armed (walking), or while a
  // valid lock is already held by someone else.
  if (last_kill_ || host_estop_) return 0;
  if (last_armed_) return 0;
  if (macLockValid(now_ms)) return 0;
  // A stale lock can be taken over by a new requester.
  mac_lock_active_ = true;
  mac_lock_token_ = next_token_++;
  if (next_token_ == 0) next_token_ = 1;  // never hand out 0
  mac_lock_last_ms_ = now_ms;
  return mac_lock_token_;
}

bool CommandArbiter::macLockHeartbeat(uint32_t token, uint32_t now_ms) {
  if (!mac_lock_active_ || token == 0 || token != mac_lock_token_) return false;
  // A heartbeat for an already-expired lock does not resurrect it.
  if (!macLockValid(now_ms)) {
    revokeMacLock();
    return false;
  }
  mac_lock_last_ms_ = now_ms;
  return true;
}

void CommandArbiter::releaseMacLock(uint32_t token) {
  if (mac_lock_active_ && token == mac_lock_token_) {
    revokeMacLock();
  }
}

const ArbiterOutput& CommandArbiter::update(const RcInputs& rc,
                                            uint32_t now_ms) {
  const bool kill_active = rc.kill || host_estop_;
  last_kill_ = kill_active;
  last_armed_ = rc.armed;

  out_.kill_active = kill_active;

  // RC kill / host estop overrides everything and revokes any maintenance lock.
  if (kill_active) {
    revokeMacLock();
    out_.source = CommandSource::None;
    out_.motion_authorized = false;
    out_.mac_lock_held = false;
    out_.mac_lock_token = 0;
    return out_;
  }

  const bool mac_held = macLockValid(now_ms);
  if (!mac_held) {
    // Expired lock: clear the holder so a stale token cannot refresh it.
    if (mac_lock_active_) revokeMacLock();
  }
  out_.mac_lock_held = mac_held;
  out_.mac_lock_token = mac_held ? mac_lock_token_ : 0;

  // Mac maintenance lock wins over RC/Jetson while held (bench control). It is
  // only ever granted while disarmed, so it cannot hijack active walking.
  if (mac_held) {
    out_.source = CommandSource::MacMaintenance;
    out_.motion_authorized = true;
    return out_;
  }

  // Jetson authority: needs RC armed + RC autonomy switch + a fresh heartbeat.
  if (rc.armed && rc.autonomy_enabled && jetsonFresh(now_ms)) {
    out_.source = CommandSource::Jetson;
    out_.motion_authorized = true;
    return out_;
  }

  // RC manual walking when armed; otherwise nobody owns motion.
  if (rc.armed) {
    out_.source = CommandSource::Rc;
    out_.motion_authorized = true;
    return out_;
  }

  out_.source = CommandSource::None;
  out_.motion_authorized = false;
  return out_;
}

}  // namespace safety
