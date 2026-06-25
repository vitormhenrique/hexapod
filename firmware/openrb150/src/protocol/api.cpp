#include "api.h"

#include "../config/config_api.h"
#include "control_api.h"
#include "framing.h"
#include "motion_api.h"
#include "telemetry.h"

namespace protocol {
namespace api {
namespace {

inline void putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void putU32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

void copyName(uint8_t* dst, const char* src) {
  for (size_t i = 0; i < kDeviceNameLen; ++i) {
    dst[i] = static_cast<uint8_t>(src[i]);
  }
}

uint8_t buildStatusFlags(const StatusSnapshot& s) {
  uint8_t f = 0;
  if (s.dxl_power) f |= 0x01;
  if (s.dxl_power_control) f |= 0x02;
  return f;
}

// Assemble a response header echoing the request seq.
Header makeResponse(const Header& req, uint16_t payload_len, uint8_t flags) {
  Header h;
  h.msg_type = static_cast<uint8_t>(MsgType::Response);
  h.msg_id = req.msg_id;
  h.flags = flags;
  h.seq = req.seq;
  h.timestamp_ms = 0;  // filled below from status by caller-provided uptime
  h.payload_len = payload_len;
  return h;
}

}  // namespace

size_t handleRequest(const uint8_t* body, size_t body_len,
                     const DeviceInfo& info, const StatusSnapshot& status,
                     uint8_t* out, size_t out_cap, config::ConfigApi* cfg,
                     SubscriptionManager* tel, ControlApi* ctrl,
                     MotionApi* motion) {
  Header req;
  uint8_t req_payload[kMaxPayload];
  size_t req_len = 0;
  const DecodeStatus st = decodeFrameBody(body, body_len, &req, req_payload,
                                          sizeof(req_payload), &req_len);
  if (st != DecodeStatus::Ok) {
    return 0;  // undecodable: drop silently
  }
  if (req.msg_type != static_cast<uint8_t>(MsgType::Command)) {
    return 0;  // we only answer commands
  }

  uint8_t payload[kMaxPayload];
  uint16_t payload_len = 0;
  uint8_t flags = 0;

  switch (req.msg_id) {
    case msg::kHello: {
      payload[0] = kVersionMajor;
      payload[1] = kVersionMinor;
      payload[2] = info.fw_major;
      payload[3] = info.fw_minor;
      payload[4] = info.fw_patch;
      copyName(&payload[5], info.device_name);
      payload_len = 21;
      break;
    }
    case msg::kHeartbeat: {
      putU32(&payload[0], status.uptime_ms);
      payload[4] = status.state;
      payload_len = 5;
      break;
    }
    case msg::kGetStatus: {
      putU32(&payload[0], status.uptime_ms);
      payload[4] = status.state;
      payload[5] = buildStatusFlags(status);
      putU16(&payload[6], status.battery_mv);
      putU32(&payload[8], status.watchdog_missed);
      payload_len = 12;
      break;
    }
    case msg::kGetCapabilities: {
      payload[0] = kVersionMajor;
      payload[1] = kVersionMinor;
      payload[2] = info.fw_major;
      payload[3] = info.fw_minor;
      payload[4] = info.fw_patch;
      putU32(&payload[5], info.feature_bits);
      copyName(&payload[9], info.device_name);
      payload_len = 25;
      break;
    }
    default: {
      // Delegate the config command group (CFG_*) to ConfigApi when present.
      if (cfg != nullptr && req.msg_id >= kConfigMsgFirst &&
          req.msg_id <= kConfigMsgLast) {
        uint16_t cfg_len = 0;
        uint8_t cfg_flags = 0;
        if (cfg->handle(req.msg_id, req_payload,
                        static_cast<uint16_t>(req_len), payload, kMaxPayload,
                        &cfg_len, &cfg_flags)) {
          payload_len = cfg_len;
          flags = cfg_flags;
          break;
        }
      }
      // Delegate the telemetry subscription group to the manager when present.
      if (tel != nullptr && req.msg_id >= kTelemetryMsgFirst &&
          req.msg_id <= kTelemetryMsgLast) {
        uint16_t tel_len = 0;
        uint8_t tel_flags = 0;
        if (tel->handle(req.msg_id, req_payload,
                        static_cast<uint16_t>(req_len), payload, kMaxPayload,
                        &tel_len, &tel_flags)) {
          payload_len = tel_len;
          flags = tel_flags;
          break;
        }
      }
      // Delegate the safety control group (ESTOP/CLEAR_FAULT/SET_ARMING/
      // SET_MODE) to the ControlApi when present.
      if (ctrl != nullptr && req.msg_id >= kControlMsgFirst &&
          req.msg_id <= kControlMsgLast) {
        uint16_t ctrl_len = 0;
        uint8_t ctrl_flags = 0;
        if (ctrl->handle(req.msg_id, req_payload,
                         static_cast<uint16_t>(req_len), payload, kMaxPayload,
                         &ctrl_len, &ctrl_flags)) {
          payload_len = ctrl_len;
          flags = ctrl_flags;
          break;
        }
      }
      // Delegate the motion group (SET_GAIT/SET_GAIT_PARAMS/SET_BODY_TWIST/
      // SET_BODY_POSE/STOP_MOTION) to the MotionApi when present.
      if (motion != nullptr && req.msg_id >= kMotionMsgFirst &&
          req.msg_id <= kMotionMsgLast) {
        uint16_t mot_len = 0;
        uint8_t mot_flags = 0;
        if (motion->handle(req.msg_id, req_payload,
                           static_cast<uint16_t>(req_len), payload, kMaxPayload,
                           &mot_len, &mot_flags)) {
          payload_len = mot_len;
          flags = mot_flags;
          break;
        }
      }
      payload[0] = static_cast<uint8_t>(Error::UnknownMsg);
      payload_len = 1;
      flags = flag::kError;
      break;
    }
  }

  Header resp = makeResponse(req, payload_len, flags);
  resp.timestamp_ms = status.uptime_ms;
  return encodeFrame(resp, payload, out, out_cap);
}

}  // namespace api
}  // namespace protocol
