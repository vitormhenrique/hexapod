#include "control_api.h"

namespace protocol {

void ControlApi::reset() {
  estop_ = false;
  disarm_ = false;
  clear_fault_ = false;
  jetson_hb_ = false;
  live_state_ = 0;
  live_fault_ = 0;
}

bool ControlApi::consumeClearFault() {
  const bool pending = clear_fault_;
  clear_fault_ = false;
  return pending;
}

bool ControlApi::consumeJetsonHeartbeat() {
  const bool pending = jetson_hb_;
  jetson_hb_ = false;
  return pending;
}

void ControlApi::writeStatus(uint8_t* out, uint16_t* out_len,
                             CtrlResult result) {
  out[0] = static_cast<uint8_t>(result);
  out[1] = live_state_;
  out[2] = live_fault_;
  *out_len = 3;
}

bool ControlApi::handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
                        uint8_t* out, uint16_t out_cap, uint16_t* out_len,
                        uint8_t* out_flags) {
  if (!ctrlmsg::isControlMsg(msg_id)) return false;

  *out_flags = 0;
  // Every control response carries [result, state, fault]; ensure capacity.
  if (out_cap < 3) {
    out[0] = static_cast<uint8_t>(CtrlResult::BadRequest);
    *out_len = 1;
    *out_flags = 0x02;  // protocol error flag (flag::kError)
    return true;
  }

  switch (msg_id) {
    case ctrlmsg::kEstop: {
      // Latch a host E-stop. Always honored: it only ever stops the robot.
      estop_ = true;
      writeStatus(out, out_len, CtrlResult::Ok);
      return true;
    }
    case ctrlmsg::kClearFault: {
      // Release the host E-stop latch and request a state-machine fault clear.
      // The machine only leaves Estop/Fault once the real condition is gone.
      estop_ = false;
      clear_fault_ = true;
      writeStatus(out, out_len, CtrlResult::Ok);
      return true;
    }
    case ctrlmsg::kSetArming: {
      if (req_len < 1) {
        out[0] = static_cast<uint8_t>(CtrlResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      const ArmingRequest arm = static_cast<ArmingRequest>(req[0]);
      if (arm == ArmingRequest::Disarm) {
        // Force-disarm: always honored (reduces authority).
        disarm_ = true;
        writeStatus(out, out_len, CtrlResult::Ok);
      } else {
        // Release the host disarm latch. Real walking-arm still needs the RC
        // arm switch, so the host cannot arm by itself: report the live state
        // (still Disarmed) so the client sees nothing was armed.
        disarm_ = false;
        writeStatus(out, out_len, CtrlResult::Ok);
      }
      return true;
    }
    case ctrlmsg::kSetMode: {
      if (req_len < 1) {
        out[0] = static_cast<uint8_t>(CtrlResult::BadRequest);
        *out_len = 1;
        *out_flags = 0x02;
        return true;
      }
      const uint8_t mode = req[0];
      // Only safety-reducing modes are honored here. State wire values:
      // 2 = Disarmed, 12 = Estop (safety/system_state.h). Richer modes are
      // entered via RC or the dedicated maintenance/passive command groups.
      if (mode == 12) {
        estop_ = true;
        writeStatus(out, out_len, CtrlResult::Ok);
      } else if (mode == 2) {
        disarm_ = true;
        writeStatus(out, out_len, CtrlResult::Ok);
      } else {
        writeStatus(out, out_len, CtrlResult::Rejected);
      }
      return true;
    }
    default:
      return false;
  }
}

}  // namespace protocol
