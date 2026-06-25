#pragma once

// ===========================================================================
// Rate-limited telemetry subscriptions (portable, no Arduino deps).
//
// Host clients (Mac/Jetson/CLI) subscribe to named telemetry streams at a
// requested rate; the firmware enforces a safe per-stream maximum so a client
// cannot overload the USB link or the MCU (AGENTS.md 6.3). This module owns:
//
//   * the per-stream subscription state (enabled + interval + counters),
//   * the SUBSCRIBE / UNSUBSCRIBE / SET_STREAM_RATE / GET_STREAM_STATS command
//     handler (mirrors the ConfigApi handle() pattern), and
//   * the emit gate shouldEmit(stream, now): true at most once per interval,
//     counting any missed slots as dropped frames so backlog is visible.
//
// It holds no payload data and does no I/O, so it is fully unit-tested on the
// host. The api task instantiates one manager, routes the telemetry command
// range to handle(), and walks the streams each loop calling shouldEmit() to
// decide which telemetry frames to build and write.
// ===========================================================================

#include <stddef.h>
#include <stdint.h>

namespace protocol {

// Telemetry streams the firmware can publish. Numeric values are part of the
// wire protocol (subscribe ids + telemetry frame msg-ids), so do not renumber.
enum class StreamId : uint8_t {
  Health = 0,         // uptime, state, watchdog, battery
  ControlState = 1,   // command source, motion authorized, kill
  ServoStatus = 2,    // per-servo position/velocity/load/voltage/temp/error
  ContactState = 3,   // per-foot fused contact state + confidence
  I2cSensorsRaw = 4,  // per-foot raw proximity/pressure
  RcInput = 5,        // normalized RC channels + flags
  ApiStats = 6,       // protocol rx/tx + drop counters
};

constexpr uint8_t kNumStreams = 7;

// Telemetry frame msg-id base: a telemetry frame for StreamId s uses header
// msg_id = kTelemetryFrameMsgBase + s. Chosen above the command ranges
// (session 0x01-0x04, telemetry cmds 0x10-0x13, config 0x20-0x25) to avoid
// collisions with command msg-ids.
constexpr uint8_t kTelemetryFrameMsgBase = 0x40;

// Telemetry/session command msg-ids (AGENTS.md 6.2 Session + Logging groups).
namespace telemsg {
constexpr uint8_t kSubscribe = 0x10;
constexpr uint8_t kUnsubscribe = 0x11;
constexpr uint8_t kSetStreamRate = 0x12;
constexpr uint8_t kGetStreamStats = 0x13;

constexpr uint8_t kFirst = kSubscribe;
constexpr uint8_t kLast = kGetStreamStats;

constexpr bool isTelemetryMsg(uint8_t msg_id) {
  return msg_id >= kFirst && msg_id <= kLast;
}
}  // namespace telemsg

// Result byte returned in subscribe/unsubscribe/set-rate responses.
enum class SubResult : uint8_t {
  Ok = 0,
  BadStream = 1,   // unknown stream id
  BadRequest = 2,  // request payload too short
};

// Lowest rate a client may request (Hz). 0 is rejected (use UNSUBSCRIBE).
constexpr uint16_t kMinRateHz = 1;

class SubscriptionManager {
 public:
  SubscriptionManager() { reset(); }

  // Clear all subscriptions and counters.
  void reset();

  // --- Subscription control (also reachable via handle()) ------------------
  // Subscribe `stream` at `rate_hz`, clamped to the stream's safe maximum.
  // Returns the effective rate (0 if the stream id is invalid).
  uint16_t subscribe(StreamId stream, uint16_t rate_hz);
  // Change the rate of a stream (does not enable a disabled stream's emission
  // counters but does set its rate; enabling is via subscribe). Returns the
  // effective clamped rate, 0 if invalid.
  uint16_t setRate(StreamId stream, uint16_t rate_hz);
  // Stop emitting `stream`. Counters are retained until reset().
  void unsubscribe(StreamId stream);

  bool enabled(StreamId stream) const;
  uint16_t rateHz(StreamId stream) const;
  uint32_t emitted(StreamId stream) const;
  uint32_t dropped(StreamId stream) const;

  // The maximum safe rate (Hz) for a stream.
  static uint16_t maxRateHz(StreamId stream);

  // --- Emit gate -----------------------------------------------------------
  // Returns true at most once per interval for an enabled stream. The first
  // call after subscribing primes the phase (emits, no drop). Subsequent calls
  // that arrive more than one interval late count the missed slots as dropped.
  bool shouldEmit(StreamId stream, uint32_t now_ms);

  // Count a frame that could not be written because the transport was full.
  void noteTxBacklog() { ++tx_backlog_; }
  uint32_t txBacklog() const { return tx_backlog_; }

  // --- Command handling ----------------------------------------------------
  // Handle one telemetry command (SUBSCRIBE/UNSUBSCRIBE/SET_STREAM_RATE/
  // GET_STREAM_STATS). Returns true if `msg_id` is a telemetry command (with
  // the response written to out/out_len/out_flags), false otherwise. Malformed
  // requests set *out_flags to the protocol error flag with a 1-byte result.
  bool handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
              uint8_t* out, uint16_t out_cap, uint16_t* out_len,
              uint8_t* out_flags);

 private:
  struct StreamState {
    bool enabled = false;
    bool primed = false;
    uint16_t rate_hz = 0;
    uint16_t interval_ms = 0;
    uint32_t last_emit_ms = 0;
    uint32_t emitted = 0;
    uint32_t dropped = 0;
  };

  static bool validStream(uint8_t id) { return id < kNumStreams; }
  StreamState* at(StreamId s) { return &streams_[static_cast<uint8_t>(s)]; }
  const StreamState* at(StreamId s) const {
    return &streams_[static_cast<uint8_t>(s)];
  }

  StreamState streams_[kNumStreams];
  uint32_t tx_backlog_ = 0;
};

}  // namespace protocol
