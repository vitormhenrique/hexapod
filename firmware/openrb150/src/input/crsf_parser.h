#pragma once

// ===========================================================================
// CRSF (Crossfire / ExpressLRS) RC parser and RC/failsafe status mapping.
//
// The ExpressLRS receiver delivers CRSF frames on Serial2 at 420000 baud. This
// module is split into two portable, heap-free pieces so the logic can be
// unit-tested on the host (pio test -e native):
//
//   * Parser    - byte-wise streaming state machine that validates the CRSF
//                 sync/length/CRC8 framing and unpacks the 16 x 11-bit RC
//                 channel payload (frame type 0x16).
//   * RcStatus  - normalizes channels to microseconds and maps the arming,
//                 kill, and gait channels; failsafe is raised when no valid
//                 frame has arrived within a timeout.
//
// Only the RC task (which owns Serial2) feeds bytes into a Parser; nothing here
// touches Arduino APIs.
// ===========================================================================

#include <stdint.h>

namespace crsf {

constexpr uint8_t kNumChannels = 16;
constexpr uint8_t kSyncByte = 0xC8;             // receiver -> flight controller
constexpr uint8_t kFrameTypeRcChannels = 0x16;  // RC_CHANNELS_PACKED
constexpr uint8_t kFrameTypeLinkStats = 0x14;   // LINK_STATISTICS (signal quality)
constexpr uint8_t kRcPayloadBytes = 22;         // 16 channels * 11 bits / 8
constexpr uint8_t kLinkStatsPayloadBytes = 10;  // CRSF LinkStatistics payload
constexpr uint8_t kMaxFrameLen = 64;            // CRSF max total frame size

// Raw 11-bit CRSF channel range and the corresponding RC microsecond range.
constexpr uint16_t kTicksMin = 172;
constexpr uint16_t kTicksMid = 992;
constexpr uint16_t kTicksMax = 1811;
constexpr uint16_t kMicrosMid = 1500;

// CRSF CRC8 (DVB-S2, polynomial 0xD5) over `len` bytes of `data`.
uint8_t crc8(const uint8_t* data, uint8_t len);

// Convert a raw 11-bit CRSF tick value (~172..1811) to microseconds (~988..2012).
uint16_t ticksToMicros(uint16_t ticks);

// Stick-to-command mapping (lmt.3). A centred stick (kMicrosMid) maps to 0; the
// magnitude ramps linearly from 0 at the deadband edge to +/-1 at +/-half-span.
constexpr uint16_t kStickDeadbandUs = 25;   // centre deadband (kills jitter)
constexpr uint16_t kStickHalfSpanUs = 500;  // us from centre that maps to +/-1

// Map a normalized RC channel value in microseconds to a symmetric [-1,1]
// body-twist command using the constants above. Values inside the centre
// deadband return exactly 0; values beyond +/-half-span clamp to +/-1.
float stickToUnit(uint16_t micros);

// Decoded RC channel ticks from one valid RC_CHANNELS_PACKED frame.
struct ChannelData {
  uint16_t channels[kNumChannels];  // raw 11-bit ticks
};

// Decoded CRSF LINK_STATISTICS (0x14) payload. RSSI values are the CRSF
// convention of a positive magnitude of a negative dBm (e.g. up_rssi_ant1 == 70
// means -70 dBm); link quality is a percentage; SNR is signed dB. tx_power is
// the CRSF power-table index, not milliwatts. All fields are as-transmitted so
// the host can present them without losing precision.
struct LinkStatistics {
  uint8_t up_rssi_ant1;      // uplink RSSI antenna 1 (-dBm magnitude)
  uint8_t up_rssi_ant2;      // uplink RSSI antenna 2 (-dBm magnitude)
  uint8_t up_link_quality;   // uplink link quality (%)
  int8_t up_snr;             // uplink SNR (dB)
  uint8_t active_antenna;    // 0 or 1
  uint8_t rf_mode;           // RF mode / packet-rate index
  uint8_t up_tx_power;       // uplink TX power table index
  uint8_t down_rssi;         // downlink RSSI (-dBm magnitude)
  uint8_t down_link_quality; // downlink link quality (%)
  int8_t down_snr;           // downlink SNR (dB)
};

// Streaming byte-wise CRSF frame parser. Static storage, single consumer.
class Parser {
 public:
  Parser() { reset(); }

  void reset();

  // Feed one received byte. Returns true when a complete, CRC-valid RC channels
  // frame has been decoded into `out`. Non-RC frame types are validated and
  // skipped (returning false); a valid LINK_STATISTICS frame is decoded into the
  // internal link-stats snapshot (see linkStats()) so mixed CRSF telemetry does
  // not wedge parsing.
  bool push(uint8_t byte, ChannelData& out);

  uint32_t framesDecoded() const { return frames_decoded_; }
  uint32_t crcErrors() const { return crc_errors_; }

  // Latest decoded link statistics. hasLinkStats() is false until the receiver
  // has sent at least one LINK_STATISTICS frame (not all ELRS configs do);
  // linkStatsCount() counts how many have been decoded.
  bool hasLinkStats() const { return link_stats_count_ > 0; }
  uint32_t linkStatsCount() const { return link_stats_count_; }
  const LinkStatistics& linkStats() const { return link_stats_; }

 private:
  enum class State : uint8_t { Sync, Length, Data };

  void unpackChannels(ChannelData& out) const;
  void unpackLinkStats();

  uint8_t buf_[kMaxFrameLen];
  uint8_t idx_;
  uint8_t frame_len_;  // CRSF length field: bytes after the length byte
  State state_;
  uint32_t frames_decoded_;
  uint32_t crc_errors_;
  uint32_t link_stats_count_;
  LinkStatistics link_stats_;
};

// Logical channel assignment (0-based) for an AETR + AUX transmitter layout.
constexpr uint8_t kChRoll = 0;      // lateral
constexpr uint8_t kChPitch = 1;     // forward
constexpr uint8_t kChThrottle = 2;  // speed
constexpr uint8_t kChYaw = 3;       // yaw
constexpr uint8_t kChArm = 4;       // AUX1
constexpr uint8_t kChKill = 5;      // AUX2
constexpr uint8_t kChGait = 6;      // AUX3
constexpr uint8_t kChAutonomy = 7;  // AUX4: grants Jetson autonomy authority
// Default time without a valid RC frame before failsafe is declared.
constexpr uint32_t kDefaultFailsafeMs = 250;

// Normalized RC status consumed by the control/safety layers.
struct RcStatus {
  uint16_t channels_us[kNumChannels];  // normalized to microseconds
  bool armed;                          // arm switch high AND not failsafe
  bool kill;                           // kill switch high OR failsafe
  uint8_t gait_index;                  // 0,1,2 from the 3-position gait switch
  bool autonomy;                       // AUX4 high: RC grants Jetson autonomy
  bool failsafe;                       // no fresh frame within the timeout
  uint32_t last_frame_ms;              // timestamp of last valid frame
  bool ever_seen;                      // any valid frame seen since boot
};

// Initialize an RcStatus to a safe failsafe state (disarmed, kill asserted).
void initRcStatus(RcStatus& rc);

// Apply a freshly decoded RC frame at host/MCU time `now_ms`.
void applyFrame(RcStatus& rc, const ChannelData& frame, uint32_t now_ms);

// Re-evaluate failsafe based on elapsed time since the last valid frame. Must
// be called periodically even when no new frame arrives. When failsafe is
// active the robot is forced disarmed with kill asserted.
void evaluateFailsafe(RcStatus& rc, uint32_t now_ms,
                      uint32_t timeout_ms = kDefaultFailsafeMs);

}  // namespace crsf
