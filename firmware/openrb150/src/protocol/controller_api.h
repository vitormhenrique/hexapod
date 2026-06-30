#pragma once

// ===========================================================================
// Controller command group (USB API, oha.4). Exposes the ChannelPack CRSF hand
// controller over USB so everything the physical remote can do is also
// scriptable from a host (Mac/Jetson/CLI):
//
//   CONTROLLER_GET_STATE    (0x90): decoded controller intent + raw inputs
//   CONTROLLER_GET_BINDINGS (0x91): the active remap table (BindingConfig)
//   CONTROLLER_SET_BINDINGS (0x92): replace the remap table (validated)
//
// and a controller_state telemetry stream that carries the same GET_STATE
// payload (see telemetry.h StreamId::ControllerState). The decoded state +
// raw-input encoder and the BindingConfig (de)serializers are static and
// portable (no Arduino deps) so the byte layouts are unit-tested on the host
// and mirrored by protocol/python + golden vectors.
//
// This handler never commands a servo. A SET_BINDINGS only *stages* a validated
// BindingConfig; the api task hands it to the rcTask-owned ControllerBridge via
// takePending() (single producer/consumer), keeping the one-owner-per-task rule
// intact. All payloads little-endian.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

#include "../input/controller_bridge.h"  // controller::* structs + ChannelPack.h

namespace protocol {

// Controller command msg-ids (0x90..0x9F block).
namespace controllermsg {
constexpr uint8_t kGetState = 0x90;
constexpr uint8_t kGetBindings = 0x91;
constexpr uint8_t kSetBindings = 0x92;
constexpr uint8_t kFirst = kGetState;
constexpr uint8_t kLast = kSetBindings;
inline bool isControllerMsg(uint8_t id) { return id >= kFirst && id <= kLast; }
}  // namespace controllermsg

enum class ControllerResult : uint8_t {
  Ok = 0,
  Rejected = 1,
  BadRequest = 2,
};

// Exact serialized sizes (asserted by the native test + Python mirror).
//   state:    31 decoded + 26 raw  = 57 bytes
//   bindings: 13*4 axis + 2 tri + 11 bool + 8*2 trick = 81 bytes
constexpr uint16_t kControllerStateLen = 57;
constexpr uint16_t kControllerBindingsLen = 81;

class ControllerApi {
 public:
  ControllerApi();

  void reset();

  // --- Live snapshot the api reports (refreshed by the api task each request).
  void setSnapshot(const controller::ControllerCommand& cmd,
                   const ChannelPackInputs_t& raw);
  // Current active bindings reported by GET_BINDINGS.
  void setBindings(const controller::BindingConfig& cfg);

  // After handle(), the api task polls this: if a SET_BINDINGS validated a new
  // table, copies it out (for the bridge to adopt) and clears the staged flag.
  bool takePending(controller::BindingConfig* out);

  // Dispatch one controller command. Returns false if msg_id is not in the
  // controller range (so the api dispatcher can try the next group).
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, uint16_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

  // --- Portable codecs (also used by the telemetry builder) ----------------
  // Encode the decoded command + raw inputs into the 57-byte controller_state
  // payload. Returns bytes written (kControllerStateLen).
  static uint16_t encodeState(const controller::ControllerCommand& cmd,
                              const ChannelPackInputs_t& raw, uint8_t* out);
  // Serialize a BindingConfig into the 81-byte payload. Returns bytes written.
  static uint16_t encodeBindings(const controller::BindingConfig& cfg,
                                 uint8_t* out);
  // Parse + range-validate an 81-byte BindingConfig payload. Returns false if
  // the length is wrong or any enum source/trick is out of range.
  static bool decodeBindings(const uint8_t* in, uint16_t len,
                             controller::BindingConfig* out);

 private:
  bool writeResult(ControllerResult r, uint8_t* out, uint16_t out_cap,
                   uint16_t* out_len, uint8_t* out_flags) const;

  controller::ControllerCommand cmd_;
  ChannelPackInputs_t raw_;
  controller::BindingConfig cfg_;
  controller::BindingConfig pending_;
  bool pending_valid_;
};

}  // namespace protocol
