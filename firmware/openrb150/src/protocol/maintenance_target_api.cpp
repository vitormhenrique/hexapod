// Maintenance leg/joint target command group (portable, host-tested).
// See maintenance_target_api.h for the gating + storage contract.

#include "maintenance_target_api.h"

#include "../dxl/servo_map.h"
#include "../gait/body_ik.h"

namespace protocol {
namespace {

int16_t readI16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

void putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

// Centidegrees -> radians (0.01 deg).
constexpr float kCentiDegToRad = 3.14159265358979323846f / 180.0f / 100.0f;

}  // namespace

void MaintTargetApi::reset() {
  cfg_ = nullptr;
  live_state_ = 0;
  lock_held_ = false;
  target_.seq = 0;
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    for (uint8_t j = 0; j < config::kJointsPerLeg; ++j) {
      target_.tick[leg][j] = config::kServoCenterTick;
      target_.set[leg][j] = false;
      target_.clamped[leg][j] = false;
    }
  }
}

bool MaintTargetApi::writeStatus(MaintTargetResult r, uint8_t* out,
                                 size_t out_cap, uint16_t* out_len,
                                 uint8_t* out_flags) const {
  if (out_cap < 2) {
    out[0] = static_cast<uint8_t>(MaintTargetResult::BadRequest);
    *out_len = 1;
    *out_flags = 0x02;  // flag::kError
    return true;
  }
  out[0] = static_cast<uint8_t>(r);
  out[1] = live_state_;
  *out_len = 2;
  *out_flags = (r == MaintTargetResult::BadRequest) ? 0x02 : 0x00;
  return true;
}

bool MaintTargetApi::handle(uint8_t msg_id, const uint8_t* req,
                            uint16_t req_len, uint8_t* out, size_t out_cap,
                            uint16_t* out_len, uint8_t* out_flags) {
  if (!mainttargetmsg::isMaintTargetMsg(msg_id)) return false;

  // Gate first: only honored in MacMaintenance with a held lock and a config.
  if (!gateOpen()) {
    return writeStatus(MaintTargetResult::Rejected, out, out_cap, out_len,
                       out_flags);
  }

  switch (msg_id) {
    case mainttargetmsg::kSetLegTarget: {
      // leg(1) + x(i16) + y(i16) + z(i16) = 7 bytes.
      if (req_len < 7) {
        return writeStatus(MaintTargetResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      const uint8_t leg = req[0];
      if (leg >= config::kNumLegs) {
        return writeStatus(MaintTargetResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      const float bx = static_cast<float>(readI16(&req[1]));
      const float by = static_cast<float>(readI16(&req[3]));
      const float bz = static_cast<float>(readI16(&req[5]));

      gait::BodyKinematics bk(*cfg_);
      const gait::IkResult ik = bk.solveBody(leg, bx, by, bz);

      dxl::ServoMap sm(*cfg_);
      const float ang[config::kJointsPerLeg] = {ik.coxa, ik.femur, ik.tibia};
      dxl::JointCommand jc[config::kJointsPerLeg];
      uint8_t clamp_low = 0;
      uint8_t clamp_high = 0;
      for (uint8_t j = 0; j < config::kJointsPerLeg; ++j) {
        jc[j] = sm.angleToTick(leg, j, ang[j]);
        if (jc[j].clamped_low) clamp_low |= static_cast<uint8_t>(1u << j);
        if (jc[j].clamped_high) clamp_high |= static_cast<uint8_t>(1u << j);
      }

      // Store only a reachable solution so the leg never lunges to a saturated
      // boundary pose; an unreachable target is reported but left uncommitted.
      if (ik.reachable) {
        for (uint8_t j = 0; j < config::kJointsPerLeg; ++j) {
          target_.tick[leg][j] = jc[j].tick;
          target_.set[leg][j] = true;
          target_.clamped[leg][j] = jc[j].clamped_low || jc[j].clamped_high;
        }
        ++target_.seq;
      }

      // [result, state, reachable, clamp_low, clamp_high, 3 x tick(u16)] = 11B.
      if (out_cap < 11) {
        return writeStatus(MaintTargetResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      out[0] = static_cast<uint8_t>(ik.reachable ? MaintTargetResult::Ok
                                                 : MaintTargetResult::Unreachable);
      out[1] = live_state_;
      out[2] = ik.reachable ? 1 : 0;
      out[3] = clamp_low;
      out[4] = clamp_high;
      putU16(&out[5], jc[0].tick);
      putU16(&out[7], jc[1].tick);
      putU16(&out[9], jc[2].tick);
      *out_len = 11;
      *out_flags = 0x00;
      return true;
    }
    case mainttargetmsg::kSetJointTarget: {
      // leg(1) + joint(1) + angle_cdeg(i16) = 4 bytes.
      if (req_len < 4) {
        return writeStatus(MaintTargetResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      const uint8_t leg = req[0];
      const uint8_t joint = req[1];
      if (leg >= config::kNumLegs || joint >= config::kJointsPerLeg) {
        return writeStatus(MaintTargetResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      const float angle_rad =
          static_cast<float>(readI16(&req[2])) * kCentiDegToRad;

      dxl::ServoMap sm(*cfg_);
      const dxl::JointCommand jc = sm.angleToTick(leg, joint, angle_rad);
      if (jc.unmapped) {
        // No servo configured for this slot: nothing to command.
        return writeStatus(MaintTargetResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }

      target_.tick[leg][joint] = jc.tick;
      target_.set[leg][joint] = true;
      target_.clamped[leg][joint] = jc.clamped_low || jc.clamped_high;
      ++target_.seq;

      // [result, state, clamp_low, clamp_high, tick(u16)] = 6 bytes.
      if (out_cap < 6) {
        return writeStatus(MaintTargetResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      out[0] = static_cast<uint8_t>(MaintTargetResult::Ok);
      out[1] = live_state_;
      out[2] = jc.clamped_low ? 1 : 0;
      out[3] = jc.clamped_high ? 1 : 0;
      putU16(&out[4], jc.tick);
      *out_len = 6;
      *out_flags = 0x00;
      return true;
    }
    default:
      return false;
  }
}

}  // namespace protocol
