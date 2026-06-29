// Config API: CFG_* command handling (portable, host-tested).
// See config_api.h for the design (RAM shadow + transactional commit).

#include "config_api.h"

#include <string.h>

namespace config {
namespace {

void putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
void putU32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
uint16_t getU16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

// Protocol error flag bit (mirrors protocol::api::flag::kError). Duplicated here
// to keep this module independent of the protocol api header.
constexpr uint8_t kErrorFlag = 0x02;

}  // namespace

void ConfigApi::resetToDefaults() {
  defaultRobotConfig(shadow_);
  staging_len_ = serializeRobotConfig(shadow_, staging_, sizeof(staging_));
  ++shadow_rev_;  // known-good config changed (lmt.7)
}

bool ConfigApi::adoptPayload(const uint8_t* payload, uint16_t len) {
  RobotConfig tmp;
  if (!deserializeRobotConfig(payload, len, tmp)) return false;
  if (!validateRobotConfig(tmp)) return false;
  shadow_ = tmp;
  memcpy(staging_, payload, len);
  staging_len_ = len;
  ++shadow_rev_;  // known-good config changed (lmt.7)
  return true;
}

bool ConfigApi::stagedValid(RobotConfig& out) const {
  if (!deserializeRobotConfig(staging_, staging_len_, out)) return false;
  return validateRobotConfig(out);
}

bool ConfigApi::handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
                       uint8_t* out, uint16_t out_cap, uint16_t* out_len,
                       uint8_t* out_flags) {
  if (!cfgmsg::isConfigMsg(msg_id)) return false;
  *out_flags = 0;
  *out_len = 0;

  switch (msg_id) {
    case cfgmsg::kGetSummary: {
      // schema_version, payload_size, block_max, flags, feature_defaults, name
      constexpr uint16_t kLen = 2 + 2 + 2 + 1 + 4 + kRobotNameLen;  // 27
      if (out_cap < kLen) return true;  // *out_len stays 0 -> caller drops

      RobotConfig staged;
      const bool staged_ok = stagedValid(staged);
      const RobotConfig& view = staged_ok ? staged : shadow_;

      uint8_t flags = 0;
      if (persist_.persistent()) flags |= 0x01;  // bit0 persistent
      if (staged_ok) flags |= 0x02;              // bit1 staged config valid

      uint16_t o = 0;
      putU16(&out[o], kSchemaVersion);
      o += 2;
      putU16(&out[o], kConfigPayloadSize);
      o += 2;
      putU16(&out[o], kCfgBlockMax);
      o += 2;
      out[o++] = flags;
      putU32(&out[o], view.feature_defaults);
      o += 4;
      memcpy(&out[o], view.robot_name, kRobotNameLen);
      o += kRobotNameLen;
      *out_len = o;
      return true;
    }

    case cfgmsg::kGetBlock: {
      if (req_len < 4) {
        out[0] = static_cast<uint8_t>(CfgError::BadRequest);
        *out_len = 1;
        *out_flags = kErrorFlag;
        return true;
      }
      const uint16_t offset = getU16(&req[0]);
      const uint16_t len = getU16(&req[2]);
      if (len == 0 || len > kCfgBlockMax ||
          static_cast<uint32_t>(offset) + len > staging_len_ ||
          static_cast<uint32_t>(len) + 4 > out_cap) {
        out[0] = static_cast<uint8_t>(CfgError::BadRange);
        *out_len = 1;
        *out_flags = kErrorFlag;
        return true;
      }
      putU16(&out[0], offset);
      putU16(&out[2], len);
      memcpy(&out[4], &staging_[offset], len);
      *out_len = static_cast<uint16_t>(4 + len);
      return true;
    }

    case cfgmsg::kSetBlock: {
      if (req_len < 4) {
        out[0] = static_cast<uint8_t>(CfgError::BadRequest);
        *out_len = 1;
        *out_flags = kErrorFlag;
        return true;
      }
      const uint16_t offset = getU16(&req[0]);
      const uint16_t len = getU16(&req[2]);
      if (len == 0 || len > kCfgBlockMax ||
          static_cast<uint32_t>(offset) + len > staging_len_ ||
          static_cast<uint32_t>(req_len) != static_cast<uint32_t>(4) + len) {
        out[0] = static_cast<uint8_t>(CfgError::BadRange);
        *out_len = 1;
        *out_flags = kErrorFlag;
        return true;
      }
      memcpy(&staging_[offset], &req[4], len);
      putU16(&out[0], offset);
      putU16(&out[2], len);
      *out_len = 4;  // ack
      return true;
    }

    case cfgmsg::kValidate: {
      RobotConfig tmp;
      const bool ok = stagedValid(tmp);
      out[0] = static_cast<uint8_t>(ok ? CfgResult::Ok
                                       : CfgResult::ValidationFailed);
      *out_len = 1;
      return true;
    }

    case cfgmsg::kCommit: {
      RobotConfig tmp;
      CfgResult result;
      if (!stagedValid(tmp)) {
        result = CfgResult::ValidationFailed;
      } else if (!persist_.persistent()) {
        result = CfgResult::Volatile;
      } else if (!persist_.commitPayload(staging_, staging_len_)) {
        result = CfgResult::CommitFailed;
      } else {
        shadow_ = tmp;  // staged config is now the known-good baseline
        ++shadow_rev_;  // known-good config changed (lmt.7)
        result = CfgResult::Ok;
      }
      out[0] = static_cast<uint8_t>(result);
      *out_len = 1;
      return true;
    }

    case cfgmsg::kResetDefaults: {
      resetToDefaults();
      out[0] = static_cast<uint8_t>(CfgResult::Ok);
      *out_len = 1;
      return true;
    }

    default:
      return false;  // unreachable: isConfigMsg already filtered
  }
}

}  // namespace config
