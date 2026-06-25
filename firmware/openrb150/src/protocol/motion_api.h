#pragma once

// ===========================================================================
// Motion command group (USB API, AGENTS.md 6.2 "Motion").
//
// Decodes the high-level motion commands that feed the gait engine + body
// transform and stores a single validated, clamped MotionIntent snapshot:
//
//   SET_GAIT        (0x34): select stand/sit/tripod/ripple/wave/crawl
//   SET_GAIT_PARAMS (0x35): stride, step height, body height, duty, speed
//   SET_BODY_TWIST  (0x36): forward / lateral / yaw rate (normalised)
//   SET_BODY_POSE   (0x37): roll/pitch/yaw + x/y/z body offset
//   STOP_MOTION     (0x38): zero twist + hold (Stand)
//
// This handler only validates ranges and stores intent; it never commands a
// servo. The control task consumes the latest intent and only *honors* it when
// the safety state machine + arbiter allow motion (g_motionGate). The response
// echoes `motion_allowed` so the host knows whether the intent will take effect
// immediately or is parked until the robot is armed/authorised.
//
// Portable (no Arduino deps) so it runs in the native unit tests. All payloads
// little-endian. No raw servo positions are ever accepted here (AGENTS.md 5.2).
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace protocol {

// Motion command msg-ids (contiguous range, reserved within the 0x30-0x3F
// control block; safety control is 0x30-0x33).
namespace motionmsg {
constexpr uint8_t kSetGait = 0x34;
constexpr uint8_t kSetGaitParams = 0x35;
constexpr uint8_t kSetBodyTwist = 0x36;
constexpr uint8_t kSetBodyPose = 0x37;
constexpr uint8_t kStopMotion = 0x38;
constexpr uint8_t kFirst = kSetGait;
constexpr uint8_t kLast = kStopMotion;
inline bool isMotionMsg(uint8_t id) { return id >= kFirst && id <= kLast; }
}  // namespace motionmsg

enum class MotionResult : uint8_t {
  Ok = 0,
  Rejected = 1,
  BadRequest = 2,
};

// Safe limits applied to every accepted command (mirrors gait engine clamps in
// gait/gait_engine.h so the stored intent can never exceed the IK-reachable
// workspace, even before the engine re-clamps).
namespace motionlim {
constexpr uint16_t kMaxStrideMm = 80;
constexpr uint16_t kMaxStepMm = 50;
constexpr uint16_t kMaxBodyHeightMm = 120;
constexpr float kMaxTwist = 1.0f;       // normalised twist component
constexpr float kMaxPoseTransMm = 50.0f;
constexpr float kMaxPoseRotRad = 0.4363f;  // ~25 deg
}  // namespace motionlim

// Gait ids (mirror config::GaitId so this header stays free of the config
// schema include).
namespace motiongait {
constexpr uint8_t kStand = 0;
constexpr uint8_t kSit = 1;
constexpr uint8_t kTripod = 2;
constexpr uint8_t kRipple = 3;
constexpr uint8_t kWave = 4;
constexpr uint8_t kCrawl = 5;
constexpr uint8_t kCount = 6;
}  // namespace motiongait

// Latest validated/clamped high-level motion intent. The control task reads a
// copy each cycle. `seq` increments on every accepted command so a consumer can
// detect changes without diffing every field.
struct MotionIntent {
  uint32_t seq = 0;
  uint8_t gait = motiongait::kStand;
  uint16_t body_height_mm = 0;
  uint16_t stride_len_mm = 0;
  uint16_t step_height_mm = 0;
  uint8_t duty_x255 = 128;
  uint8_t speed_x255 = 128;
  float twist_vx = 0.0f;  // forward (+) / backward (-)  [-1,1]
  float twist_vy = 0.0f;  // left (+) / right (-)        [-1,1]
  float twist_wz = 0.0f;  // yaw CCW (+) / CW (-)         [-1,1]
  float pose_x_mm = 0.0f;
  float pose_y_mm = 0.0f;
  float pose_z_mm = 0.0f;
  float pose_roll = 0.0f;
  float pose_pitch = 0.0f;
  float pose_yaw = 0.0f;
};

class MotionApi {
 public:
  void reset();

  // Seed defaults (e.g. from config GaitDefaults) without bumping seq. Values
  // are clamped to the safe limits.
  void setDefaults(uint8_t gait, uint16_t body_height_mm, uint16_t stride_mm,
                   uint16_t step_mm, uint8_t duty_x255, uint8_t speed_x255);

  // Refresh the live gate that the response echoes; the control task supplies
  // the current safety state and whether motion is presently authorised.
  void setLiveState(uint8_t state, bool motion_allowed);

  const MotionIntent& intent() const { return intent_; }

  // Dispatch a motion command. Returns false if `msg_id` is not in the motion
  // range (so the api dispatcher can try the next group). On a handled command
  // writes the response payload and returns true.
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, size_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

 private:
  bool writeResult(MotionResult r, uint8_t* out, size_t out_cap,
                   uint16_t* out_len, uint8_t* out_flags) const;

  MotionIntent intent_;
  uint8_t live_state_ = 0;
  bool motion_allowed_ = false;
};

}  // namespace protocol
