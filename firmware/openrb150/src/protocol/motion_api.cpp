#include "motion_api.h"

namespace protocol {
namespace {

int16_t readI16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
uint16_t readU16(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

uint16_t clampU16(uint16_t v, uint16_t hi) { return v > hi ? hi : v; }

float clampF(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// twist components arrive as signed milli-units (-1000..1000 -> -1..1).
float twistFromMilli(int16_t milli) {
  return clampF(static_cast<float>(milli) / 1000.0f, -motionlim::kMaxTwist,
                motionlim::kMaxTwist);
}

}  // namespace

void MotionApi::reset() {
  intent_ = MotionIntent{};
  live_state_ = 0;
  motion_allowed_ = false;
}

void MotionApi::setDefaults(uint8_t gait, uint16_t body_height_mm,
                            uint16_t stride_mm, uint16_t step_mm,
                            uint8_t duty_x255, uint8_t speed_x255) {
  intent_.gait = gait < motiongait::kCount ? gait : motiongait::kStand;
  intent_.body_height_mm = clampU16(body_height_mm, motionlim::kMaxBodyHeightMm);
  intent_.stride_len_mm = clampU16(stride_mm, motionlim::kMaxStrideMm);
  intent_.step_height_mm = clampU16(step_mm, motionlim::kMaxStepMm);
  intent_.duty_x255 = duty_x255;
  intent_.speed_x255 = speed_x255;
}

void MotionApi::setLiveState(uint8_t state, bool motion_allowed) {
  live_state_ = state;
  motion_allowed_ = motion_allowed;
}

bool MotionApi::writeResult(MotionResult r, uint8_t* out, size_t out_cap,
                            uint16_t* out_len, uint8_t* out_flags) const {
  if (out_cap < 3) {
    out[0] = static_cast<uint8_t>(MotionResult::BadRequest);
    *out_len = 1;
    *out_flags = 0x02;  // flag::kError
    return true;
  }
  out[0] = static_cast<uint8_t>(r);
  out[1] = live_state_;
  out[2] = motion_allowed_ ? 1 : 0;
  *out_len = 3;
  *out_flags = (r == MotionResult::BadRequest) ? 0x02 : 0x00;
  return true;
}

bool MotionApi::handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
                       uint8_t* out, size_t out_cap, uint16_t* out_len,
                       uint8_t* out_flags) {
  if (!motionmsg::isMotionMsg(msg_id)) return false;

  // Reject gait/twist/pose changes while the robot is in torque-off passive
  // pose streaming (AGENTS.md 5.5). STOP_MOTION stays honoured below.
  if (live_state_ == motionstate::kPassivePoseStream &&
      msg_id != motionmsg::kStopMotion) {
    return writeResult(MotionResult::Rejected, out, out_cap, out_len,
                       out_flags);
  }

  switch (msg_id) {
    case motionmsg::kSetGait: {
      if (req_len < 1) {
        return writeResult(MotionResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      const uint8_t g = req[0];
      if (g >= motiongait::kCount) {
        return writeResult(MotionResult::Rejected, out, out_cap, out_len,
                           out_flags);
      }
      intent_.gait = g;
      ++intent_.seq;
      return writeResult(MotionResult::Ok, out, out_cap, out_len, out_flags);
    }
    case motionmsg::kSetGaitParams: {
      if (req_len < 8) {
        return writeResult(MotionResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      intent_.body_height_mm =
          clampU16(readU16(&req[0]), motionlim::kMaxBodyHeightMm);
      intent_.stride_len_mm = clampU16(readU16(&req[2]), motionlim::kMaxStrideMm);
      intent_.step_height_mm = clampU16(readU16(&req[4]), motionlim::kMaxStepMm);
      intent_.duty_x255 = req[6];
      intent_.speed_x255 = req[7];
      ++intent_.seq;
      return writeResult(MotionResult::Ok, out, out_cap, out_len, out_flags);
    }
    case motionmsg::kSetBodyTwist: {
      if (req_len < 6) {
        return writeResult(MotionResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      intent_.twist_vx = twistFromMilli(readI16(&req[0]));
      intent_.twist_vy = twistFromMilli(readI16(&req[2]));
      intent_.twist_wz = twistFromMilli(readI16(&req[4]));
      ++intent_.seq;
      return writeResult(MotionResult::Ok, out, out_cap, out_len, out_flags);
    }
    case motionmsg::kSetBodyPose: {
      if (req_len < 12) {
        return writeResult(MotionResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      const float t = motionlim::kMaxPoseTransMm;
      const float r = motionlim::kMaxPoseRotRad;
      intent_.pose_x_mm = clampF(static_cast<float>(readI16(&req[0])), -t, t);
      intent_.pose_y_mm = clampF(static_cast<float>(readI16(&req[2])), -t, t);
      intent_.pose_z_mm = clampF(static_cast<float>(readI16(&req[4])), -t, t);
      // rotation arrives in milli-degrees -> radians.
      const float kMdegToRad = 3.14159265358979f / 180000.0f;
      intent_.pose_roll =
          clampF(static_cast<float>(readI16(&req[6])) * kMdegToRad, -r, r);
      intent_.pose_pitch =
          clampF(static_cast<float>(readI16(&req[8])) * kMdegToRad, -r, r);
      intent_.pose_yaw =
          clampF(static_cast<float>(readI16(&req[10])) * kMdegToRad, -r, r);
      ++intent_.seq;
      return writeResult(MotionResult::Ok, out, out_cap, out_len, out_flags);
    }
    case motionmsg::kStopMotion: {
      // Always honoured: zero twist and hold the static Stand pose.
      intent_.twist_vx = 0.0f;
      intent_.twist_vy = 0.0f;
      intent_.twist_wz = 0.0f;
      intent_.gait = motiongait::kStand;
      ++intent_.seq;
      return writeResult(MotionResult::Ok, out, out_cap, out_len, out_flags);
    }
    default:
      return false;
  }
}

}  // namespace protocol
