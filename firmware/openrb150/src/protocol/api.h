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
//                           feature_bits: per-Feature availability bitmask,
//                           bit i == Feature i available (see feature_api.h).
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

// Forward declaration: the feature flag command group (FEATURE_GET / FEATURE_SET
// / FEATURE_GET_REASONS / FEATURE_RESET_DEFAULTS) is handled by a portable
// FeatureApi in protocol/feature_api.h. handleRequest delegates that msg-id
// range to it when a non-null instance is supplied.
namespace protocol {
class FeatureApi;
}

// Forward declaration: the maintenance lock command group (ENTER/EXIT/HEARTBEAT
// maintenance) is handled by a portable MaintenanceApi in
// protocol/maintenance_api.h. handleRequest delegates that msg-id range to it
// when a non-null instance is supplied.
namespace protocol {
class MaintenanceApi;
}

// Forward declaration: the maintenance leg/joint target command group
// (SET_LEG_TARGET / SET_JOINT_TARGET) is handled by a portable MaintTargetApi
// in protocol/maintenance_target_api.h. handleRequest delegates its msg-ids
// (within the maintenance block) to it when a non-null instance is supplied.
namespace protocol {
class MaintTargetApi;
}

// Forward declaration: the DXL maintenance command group (DXL_SCAN / DXL_PING /
// DXL_TORQUE / DXL_GET_SERVO_PROFILE / DXL_GET_RESULT) is handled by a portable
// DxlJobApi in protocol/dxl_job_api.h. handleRequest delegates its msg-id range
// to it when a non-null instance is supplied.
namespace protocol {
class DxlJobApi;
}

// Forward declaration: the sensor / contact / leveling command group
// (CONTACT_*/LEVELING_*/I2C_*/SENSOR_*) is handled by a portable SensorApi in
// protocol/sensor_api.h. handleRequest delegates its 0x70-0x7F msg-id block to
// it when a non-null instance is supplied.
namespace protocol {
class SensorApi;
}

// Forward declaration: the passive pose streaming command group (PASSIVE_ENTER
// / EXIT / SET_STREAM_RATE / ZERO_REFERENCE) is handled by a portable
// PassiveApi in protocol/passive_api.h. handleRequest delegates its 0x80-0x83
// msg-id block to it when a non-null instance is supplied.
namespace protocol {
class PassiveApi;
}

// Forward declaration: the controller command group (CONTROLLER_GET_STATE /
// GET_BINDINGS / SET_BINDINGS) is handled by a portable ControllerApi in
// protocol/controller_api.h. handleRequest delegates its 0x90-0x9F msg-id block
// to it when a non-null instance is supplied.
namespace protocol {
class ControllerApi;
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

// Feature flag command msg-id range, mirrored from protocol::featuremsg.
constexpr uint8_t kFeatureMsgFirst = 0x39;
constexpr uint8_t kFeatureMsgLast = 0x3C;

// Maintenance command msg-id range (0x50..0x5F block), from protocol::maintmsg.
constexpr uint8_t kMaintenanceMsgFirst = 0x50;
constexpr uint8_t kMaintenanceMsgLast = 0x5F;

// Maintenance leg/joint target msg-id range (within the 0x50-0x5F block), from
// protocol::mainttargetmsg.
constexpr uint8_t kMaintTargetMsgFirst = 0x53;
constexpr uint8_t kMaintTargetMsgLast = 0x54;

// DXL maintenance command msg-id range (0x60..0x6F block), from protocol::dxlmsg.
constexpr uint8_t kDxlMsgFirst = 0x60;
constexpr uint8_t kDxlMsgLast = 0x6F;

// Sensor / contact / leveling command msg-id block (0x70..0x7F), from
// protocol::sensormsg.
constexpr uint8_t kSensorMsgFirst = 0x70;
constexpr uint8_t kSensorMsgLast = 0x7F;

// Passive pose streaming command msg-id block (0x80..0x83), from
// protocol::passivemsg.
constexpr uint8_t kPassiveMsgFirst = 0x80;
constexpr uint8_t kPassiveMsgLast = 0x83;

// Controller command msg-id block (0x90..0x9F), from protocol::controllermsg.
constexpr uint8_t kControllerMsgFirst = 0x90;
constexpr uint8_t kControllerMsgLast = 0x9F;

constexpr size_t kDeviceNameLen = 16;

// Description of this firmware build. fw_* and device_name are filled once at
// boot; feature_bits is refreshed live from runtime feature availability before
// each request (4sa.4), so GET_CAPABILITIES reports honest capabilities.
struct DeviceInfo {
  uint8_t fw_major = 0;
  uint8_t fw_minor = 0;
  uint8_t fw_patch = 0;
  // Per-Feature availability bitmask, bit i == Feature i available (see
  // protocol::Feature / FeatureApi::availableMask). 0 == nothing available yet.
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
                     MotionApi* motion = nullptr,
                     MaintenanceApi* maint = nullptr,
                     MaintTargetApi* maint_target = nullptr,
                     DxlJobApi* dxl_jobs = nullptr,
                     FeatureApi* features = nullptr,
                     SensorApi* sensors = nullptr,
                     PassiveApi* passive = nullptr,
                     ControllerApi* controller = nullptr);

}  // namespace api
}  // namespace protocol
