#pragma once

// ===========================================================================
// Config API: CFG_* command handling over the USB protocol (portable).
//
// Exposes the persistent robot config (config_schema.h) to a host through the
// protocol layer (AGENTS.md 6.2 "Config" group): get summary, block get/set,
// validate, commit, reset-to-defaults. Implements the
// "RAM shadow + transactional EEPROM commit" model required by issue 22l.2:
//
//   * `staging_`  - the editable RAM shadow (serialized payload bytes). Host
//                   edits it in place with CFG_SET_BLOCK and reads it back with
//                   CFG_GET_BLOCK. The full payload is larger than one protocol
//                   frame, so transfers are block-windowed.
//   * `shadow_`   - the last config known-good (committed or adopted at boot).
//   * commit      - validates the staged bytes, then persists them through an
//                   abstract ConfigPersistence sink (the 24LC32 transactional
//                   store on-target). Invalid configs are rejected; a volatile
//                   store (missing EEPROM) rejects commit (AGENTS.md 4.3).
//
// The persistence sink is abstract so this whole module is unit-tested on the
// host with a fake EEPROM-backed store; on-target the sink routes the commit to
// the I2C task, which is the sole owner of Wire/EEPROM (AGENTS.md 5.1).
//
// No Arduino deps; no heap. The caller owns the response buffer.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

#include "config_schema.h"

namespace config {

// Protocol message IDs for the config command group (AGENTS.md 6.2). Kept here
// as the single source of truth; the protocol api layer dispatches this range
// to ConfigApi::handle().
namespace cfgmsg {
constexpr uint8_t kGetSummary = 0x20;
constexpr uint8_t kGetBlock = 0x21;
constexpr uint8_t kSetBlock = 0x22;
constexpr uint8_t kValidate = 0x23;
constexpr uint8_t kCommit = 0x24;
constexpr uint8_t kResetDefaults = 0x25;

// True if `msg_id` belongs to the config command group.
constexpr bool isConfigMsg(uint8_t msg_id) {
  return msg_id >= kGetSummary && msg_id <= kResetDefaults;
}
}  // namespace cfgmsg

// Largest config payload window per CFG_GET_BLOCK/CFG_SET_BLOCK frame. Bounded
// well under the 256-byte max protocol payload (leaving room for the 4-byte
// offset/len prefix).
constexpr uint16_t kCfgBlockMax = 192;

// Result byte returned in CFG_VALIDATE / CFG_COMMIT responses.
enum class CfgResult : uint8_t {
  Ok = 0,
  ValidationFailed = 1,
  Volatile = 2,      // persistent store unavailable (commit rejected)
  CommitFailed = 3,  // store write transaction failed
};

// Error byte returned (with the protocol error flag) for malformed requests.
enum class CfgError : uint8_t {
  None = 0,
  BadRequest = 1,  // request payload too short
  BadRange = 2,    // offset/len outside the config payload window
};

// Abstract persistence sink. Decouples ConfigApi from Wire/EEPROM so it is
// host-testable; on-target the implementation routes commits to the I2C task.
class ConfigPersistence {
 public:
  virtual ~ConfigPersistence() = default;
  // Persist an already-validated serialized payload transactionally. Returns
  // true on success.
  virtual bool commitPayload(const uint8_t* payload, uint16_t len) = 0;
  // Whether persistent storage is currently available. When false the config
  // is volatile and CFG_COMMIT is rejected.
  virtual bool persistent() const = 0;
};

class ConfigApi {
 public:
  explicit ConfigApi(ConfigPersistence& persist) : persist_(persist) {
    resetToDefaults();
  }

  // Load compiled defaults into both the staged shadow and the last-good copy.
  void resetToDefaults();

  // Adopt an externally loaded persisted payload (e.g. the valid EEPROM slot at
  // boot) as the staged shadow + last-good copy. Returns false (and changes
  // nothing) if the payload does not deserialize and validate.
  bool adoptPayload(const uint8_t* payload, uint16_t len);

  // Handle one CFG_* command. Returns true if `msg_id` is a config command
  // (response written to out/out_len/out_flags), false if not a config command.
  // On a malformed request, sets *out_flags to the protocol error flag and
  // writes a 1-byte CfgError payload.
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, uint16_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

  // Last known-good config (committed or adopted). Exposed for the firmware to
  // consume the active servo map / geometry / gait defaults.
  const RobotConfig& config() const { return shadow_; }

  // Monotonic revision of the known-good config(). Bumped on every shadow
  // change: reset-to-defaults, boot adopt, and a successful CFG_COMMIT (NOT on
  // CFG_SET_BLOCK, which only edits the staged bytes). Runtime consumers that
  // cache config-derived state (gait pipeline body IK, contact calibration,
  // motion + feature defaults) watch this to re-apply after boot adoption /
  // commit instead of polling each field (lmt.7).
  uint32_t revision() const { return shadow_rev_; }

 private:
  // Best-effort decode of the staged bytes; returns true if they validate.
  bool stagedValid(RobotConfig& out) const;

  ConfigPersistence& persist_;
  RobotConfig shadow_;                    // last known-good config
  uint8_t staging_[kConfigPayloadSize];   // editable serialized RAM shadow
  uint16_t staging_len_ = kConfigPayloadSize;
  uint32_t shadow_rev_ = 0;               // bumped whenever shadow_ changes
};

}  // namespace config
