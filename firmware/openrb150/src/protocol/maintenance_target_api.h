#pragma once

// ===========================================================================
// Maintenance leg/joint target command group (USB API, AGENTS.md 5.2 / 6.2
// "Maintenance": SET_LEG_TARGET, SET_JOINT_TARGET).
//
// Low-authority, human-driven bench control: nudge one foot (Cartesian, in the
// body frame) or one joint (angle) while the robot is held in MacMaintenance
// with a live maintenance lock. Targets run through the SAME safety chain as
// the gait engine -- body/leg IK (gait/body_ik.h) then the servo map + joint
// limits (dxl/servo_map.h) -- so a maintenance move can never exceed the
// configured servo travel. No raw servo ticks are ever accepted here.
//
//   SET_LEG_TARGET   (0x53): leg, foot x/y/z (mm, body frame) -> IK -> 3 ticks
//   SET_JOINT_TARGET (0x54): leg, joint, angle (centidegrees) -> 1 tick
//
// This handler only validates, solves, clamps and STORES the resulting goal
// ticks (MaintTargetSet); it never touches the DXL bus. The control/dxl task
// consumes the stored targets and only writes them when the arbiter grants
// MacMaintenance authority (gated by g_motionGate). Gating: a command is only
// honored when the live safety state is MacMaintenance and the maintenance lock
// is held; otherwise it is Rejected without altering the stored targets.
//
// A leg target whose IK falls outside the reachable workspace is reported
// Unreachable and is NOT stored (the foot is left where it is) so the leg never
// lunges to a saturated boundary pose; the computed ticks are still returned so
// the host sees how far off the request was.
//
// Portable (no Arduino deps): BodyKinematics + ServoMap are constructed on
// demand from the active RobotConfig, so this runs in the native unit tests.
// All payloads little-endian.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

#include "../config/config_schema.h"

namespace protocol {

// Maintenance target msg-ids (within the reserved 0x50-0x5F maintenance block;
// the lock commands ENTER/EXIT/HEARTBEAT occupy 0x50-0x52).
namespace mainttargetmsg {
constexpr uint8_t kSetLegTarget = 0x53;
constexpr uint8_t kSetJointTarget = 0x54;
constexpr uint8_t kFirst = kSetLegTarget;
constexpr uint8_t kLast = kSetJointTarget;
inline bool isMaintTargetMsg(uint8_t id) { return id >= kFirst && id <= kLast; }
}  // namespace mainttargetmsg

enum class MaintTargetResult : uint8_t {
  Ok = 0,           // accepted, stored, will be honored under authority
  Rejected = 1,     // not in MacMaintenance / lock not held / no config
  BadRequest = 2,   // malformed payload or bad leg/joint index
  Unreachable = 3,  // leg-target IK outside workspace (computed, NOT stored)
};

// Live safety state value that gates maintenance moves (safety::State value).
namespace mainttargetstate {
constexpr uint8_t kMacMaintenance = 8;
}  // namespace mainttargetstate

// Latest stored per-joint goal ticks, keyed by (leg, joint). `seq` bumps on
// every accepted (stored) command so a consumer can detect changes; `set` marks
// which joints have been commanded since reset (others stay at center).
struct MaintTargetSet {
  uint32_t seq = 0;
  uint16_t tick[config::kNumLegs][config::kJointsPerLeg];
  bool set[config::kNumLegs][config::kJointsPerLeg];
};

class MaintTargetApi {
 public:
  MaintTargetApi() { reset(); }

  void reset();

  // Bind the active robot config used for IK + the servo map. Until a non-null
  // config is supplied, commands are Rejected (no kinematics available).
  void setConfig(const config::RobotConfig* cfg) { cfg_ = cfg; }

  // Refresh the gate the handler enforces: the current safety state and whether
  // the maintenance lock is held. Commands are only honored when state ==
  // MacMaintenance and lock_held is true.
  void setLiveState(uint8_t state, bool lock_held) {
    live_state_ = state;
    lock_held_ = lock_held;
  }

  const MaintTargetSet& target() const { return target_; }

  // Dispatch a maintenance target command. Returns false if `msg_id` is not in
  // the maintenance-target range (so the api dispatcher can try the next
  // group). On a handled command writes the response payload and returns true.
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, size_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

 private:
  // Short [result, state] response (+ error flag for BadRequest).
  bool writeStatus(MaintTargetResult r, uint8_t* out, size_t out_cap,
                   uint16_t* out_len, uint8_t* out_flags) const;

  bool gateOpen() const {
    return cfg_ != nullptr && lock_held_ &&
           live_state_ == mainttargetstate::kMacMaintenance;
  }

  const config::RobotConfig* cfg_ = nullptr;
  uint8_t live_state_ = 0;
  bool lock_held_ = false;
  MaintTargetSet target_;
};

}  // namespace protocol
