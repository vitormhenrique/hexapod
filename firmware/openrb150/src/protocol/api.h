#pragma once

// ===========================================================================
// USB API v0 message layer.
//
// Decodes a request frame body, dispatches by msg_id, and encodes a response
// frame. Portable (no Arduino deps) so it runs in native unit tests; the task
// in app/tasks.cpp fills the snapshots from the board HAL and moves bytes over
// USB CDC.
//
// Request msg_type = Command, response msg_type = Response with the same seq.
// All payloads little-endian. See protocol/README.md.
//
// Payload layouts (response):
//   HELLO            (21B): proto_major, proto_minor, fw_major, fw_minor,
//                           fw_patch, name[16]
//   HEARTBEAT         (5B): uptime_ms(4), state(1)
//   GET_STATUS       (12B): uptime_ms(4), state(1), status_flags(1),
//                           battery_mv(2), watchdog_missed(4)
//   GET_CAPABILITIES (25B): proto_major, proto_minor, fw_major, fw_minor,
//                           fw_patch, feature_bits(4), name[16]
//   error frame       (1B): error_code  (header flags has kError set)
//
//   status_flags: bit0 = dxl_power, bit1 = dxl_power_control
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

// Forward declaration: the config command group (CFG_*) is handled by a
// portable ConfigApi defined in config/config_api.h. handleRequest delegates
// that msg-id range to it when a non-null instance is supplied. Kept a forward
// decl so the protocol layer does not pull the config schema into builds (e.g.
// the protocol-only native tests) that do not exercise config.
namespace config {
class ConfigApi;
}

// Forward declaration: the telemetry subscription command group (SUBSCRIBE /
// UNSUBSCRIBE / SET_STREAM_RATE / GET_STREAM_STATS) is handled by a portable
// SubscriptionManager in protocol/telemetry.h. handleRequest delegates that
// msg-id range to it when a non-null instance is supplied.
namespace protocol {
class SubscriptionManager;
}

// Forward declaration: the safety control command group (ESTOP / CLEAR_FAULT /
// SET_ARMING / SET_MODE) is handled by a portable ControlApi in
// protocol/control_api.h. handleRequest delegates that msg-id range to it when
// a non-null instance is supplied.
namespace protocol {
class ControlApi;
}

// Forward declaration: the motion command group (SET_GAIT / SET_GAIT_PARAMS /
// SET_BODY_TWIST / SET_BODY_POSE / STOP_MOTION) is handled by a portable
// MotionApi in protocol/motion_api.h. handleRequest delegates that msg-id range
// to it when a non-null instance is supplied.
namespace protocol {
class MotionApi;
}

namespace protocol {
namespace api {

// Message IDs (command and its response share the id; msg_type distinguishes).
namespace msg {
constexpr uint8_t kHello = 0x01;
constexpr uint8_t kHeartbeat = 0x02;
constexpr uint8_t kGetStatus = 0x03;
constexpr uint8_t kGetCapabilities = 0x04;
}  // namespace msg

// Header flag bits.
namespace flag {
constexpr uint8_t kAckReq = 0x01;
constexpr uint8_t kError = 0x02;
constexpr uint8_t kFragment = 0x04;
}  // namespace flag

enum class Error : uint8_t {
  None = 0,
  UnknownMsg = 1,
  BadRequest = 2,
};

// Config command msg-id range (CFG_*), mirrored from config::cfgmsg so the
// dispatcher can recognize it without including the config header here.
constexpr uint8_t kConfigMsgFirst = 0x20;
constexpr uint8_t kConfigMsgLast = 0x25;

// Telemetry subscription command msg-id range, mirrored from protocol::telemsg.
constexpr uint8_t kTelemetryMsgFirst = 0x10;
constexpr uint8_t kTelemetryMsgLast = 0x13;

// Safety control command msg-id range, mirrored from protocol::ctrlmsg.
constexpr uint8_t kControlMsgFirst = 0x30;
constexpr uint8_t kControlMsgLast = 0x33;

// Motion command msg-id range, mirrored from protocol::motionmsg.
constexpr uint8_t kMotionMsgFirst = 0x34;
constexpr uint8_t kMotionMsgLast = 0x38;

constexpr size_t kDeviceNameLen = 16;

// Static description of this firmware build. Filled once at boot.
struct DeviceInfo {
  uint8_t fw_major = 0;
  uint8_t fw_minor = 0;
  uint8_t fw_patch = 0;
  uint32_t feature_bits = 0;
  char device_name[kDeviceNameLen] = {0};
};

// Live status snapshot, refreshed by the caller before each handleRequest().
struct StatusSnapshot {
  uint32_t uptime_ms = 0;
  uint8_t state = 0;
  bool dxl_power = false;
  bool dxl_power_control = false;
  uint16_t battery_mv = 0;
  uint32_t watchdog_missed = 0;
};

// Decode one COBS-encoded request body (bytes between 0x00 delimiters),
// dispatch it, and write a full wire response (including delimiters) to `out`.
// Returns the response length, or 0 if no response should be sent (the request
// could not be decoded, or was not a Command).
//
// `cfg` (optional) handles the CFG_* command group; when null those messages
// are answered with an UnknownMsg error.
size_t handleRequest(const uint8_t* body, size_t body_len,
                     const DeviceInfo& info, const StatusSnapshot& status,
                     uint8_t* out, size_t out_cap,
                     config::ConfigApi* cfg = nullptr,
                     SubscriptionManager* tel = nullptr,
                     ControlApi* ctrl = nullptr,
                     MotionApi* motion = nullptr);

}  // namespace api
}  // namespace protocol
