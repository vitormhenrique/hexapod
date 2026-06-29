#pragma once

// ===========================================================================
// Controller bridge: ChannelPack CRSF remote -> high-level hexapod commands
// (oha.2). Portable, heap-free, native-testable (pio test -e native).
//
// The OpenRB-150 is the *receiver* in the ChannelPack link (see
// lib/ChannelPack/README.md). The CRSF parser (input/crsf_parser.h) already
// decodes the raw 16 x 11-bit channel frame; this bridge takes those raw ticks
// and turns the controller's physical inputs -- 4 gimbal axes, 2 pots, 2
// encoders, 6 two-position switches, 2 three-position toggles, 4 buttons, and
// 2 five-way nav clusters -- into one validated ControllerCommand snapshot the
// control layer consumes each cycle.
//
// It decodes the channel/bit layout through the vendored ChannelPack.h so there
// is a single source of truth for the wire format (the header is shared with
// the TX controller; copies must stay byte-identical -- so we include it rather
// than re-deriving the layout here).
//
// Everything is TABLE-DRIVEN and REMAPPABLE: a BindingConfig names which
// physical source feeds each logical function. setBindings() is the entry point
// for a USB host (Mac) to override the map at runtime (oha.4). defaultBindings()
// is the safe out-of-box layout documented in docs/controller_bridge.md.
//
// Control modes (the "move the core without moving the legs" requirement):
//   Walk          - gimbals drive the gait body twist (forward/strafe/yaw).
//   TranslateBody - gimbals shift the body x/y/z with the feet planted.
//   RotateBody    - gimbals tilt the body roll/pitch/yaw with the feet planted.
//
// The bridge is stateful (encoder-delta shape trims, button edge-detect +
// refractory debounce, persistent operator pose trim, failsafe hold) but uses
// only fixed-size members -- no heap, safe to own from rcTask. It never touches
// Arduino APIs and never commands a servo; it only produces intent. The control
// task applies it through the same safety gate as every other command source.
// ===========================================================================

#include <stdint.h>

#include "ChannelPack.h"  // vendored: single source of truth for the channel map

namespace controller {

// --- Logical input sources (what a binding can point at) -------------------

// Proportional / analog sources. Gimbals are bipolar (centre = 0); pots are
// unipolar (0..1); encoders are relative and integrated into a 0..1 trim.
enum class AxisSource : uint8_t {
  None = 0,
  GimbalLX,
  GimbalLY,
  GimbalRX,
  GimbalRY,
  Pot1,
  Pot2,
  Enc1,
  Enc2,
};

// Boolean (on/off) sources: the 6 two-position switches, 4 buttons, and the 10
// nav-cluster directions (NAV1/NAV2 x U/D/L/R/C).
enum class BoolSource : uint8_t {
  None = 0,
  SwA,
  SwB,
  SwC,
  SwD,
  SwG,
  SwH,
  Btn1,
  Btn2,
  Btn3,
  Btn4,
  Nav1Up,
  Nav1Down,
  Nav1Left,
  Nav1Right,
  Nav1Center,
  Nav2Up,
  Nav2Down,
  Nav2Left,
  Nav2Right,
  Nav2Center,
};

// Three-position (UP=0 / CENTER=1 / DOWN=2) sources: the two 3-pos toggles.
enum class TriSource : uint8_t {
  None = 0,
  SwE,
  SwF,
};

// --- Decoded high-level outputs --------------------------------------------

// High-level control mode selected by a 3-position toggle (or a USB override).
enum class ControlMode : uint8_t {
  Walk = 0,           // gimbals -> gait body twist
  TranslateBody = 1,  // gimbals -> body x/y/z offset, feet planted
  RotateBody = 2,     // gimbals -> body roll/pitch/yaw, feet planted
};
constexpr uint8_t kNumControlModes = 3;

// Trick / choreography id. The bridge only emits the rising-edge trigger; the
// choreography itself lives in the control-task trick engine (oha.5).
enum class TrickId : uint8_t {
  None = 0,
  StandUp,
  SitDown,
  Wave,
  CrouchToggle,
  Twirl,
  Stretch,
  LeanLook,
  DanceLoop,
};

// --- Binding configuration (remappable) ------------------------------------

// One proportional binding: which source, and whether to invert it. `deadband`
// is a fraction (0..1) of the half-span around centre that reads as exactly 0
// (kills stick/pot jitter); it is ignored for encoder sources.
struct AxisBinding {
  AxisSource source = AxisSource::None;
  bool invert = false;
  float deadband = 0.0f;
  AxisBinding() = default;
  AxisBinding(AxisSource s, bool inv = false, float db = 0.0f)
      : source(s), invert(inv), deadband(db) {}
};

// One trick binding: a boolean source whose rising edge fires a trick.
struct TrickBinding {
  BoolSource source = BoolSource::None;
  TrickId trick = TrickId::None;
  TrickBinding() = default;
  TrickBinding(BoolSource s, TrickId t) : source(s), trick(t) {}
};
constexpr uint8_t kMaxTrickBindings = 8;

// Full controller->action map. Every field is overridable (defaultBindings()
// fills the documented layout). Kept POD/trivially-copyable so a USB handler can
// stage and swap a whole config (oha.4).
struct BindingConfig {
  // Walk-mode gimbal axes.
  AxisBinding walk_forward;  // body +x forward / -x backward
  AxisBinding walk_strafe;   // body +y left / -y right
  AxisBinding walk_yaw;      // yaw CCW(+)/CW(-)
  // Translate-body-mode gimbal axes.
  AxisBinding body_x;  // forward/back body shift
  AxisBinding body_y;  // left/right body shift
  AxisBinding body_z;  // up/down body shift
  // Rotate-body-mode gimbal axes.
  AxisBinding body_roll;
  AxisBinding body_pitch;
  AxisBinding body_yaw;
  // Live shape parameters (read in every mode), normalised 0..1.
  AxisBinding speed;
  AxisBinding body_height;
  AxisBinding stride;
  AxisBinding step_height;
  // Mode + gait family selectors (3-position toggles).
  TriSource mode_select = TriSource::SwE;
  TriSource gait_select = TriSource::SwF;
  // Safety + feature switches (level, not edge).
  BoolSource arm = BoolSource::SwA;
  BoolSource estop = BoolSource::SwB;
  BoolSource feat_foot_contact = BoolSource::SwC;
  BoolSource feat_terrain_leveling = BoolSource::SwD;
  BoolSource feat_passive_pose = BoolSource::SwG;
  BoolSource host_authority = BoolSource::SwH;
  // Persistent operator pose trim (edge-nudged, fixed step per press).
  BoolSource trim_pitch_up = BoolSource::Nav1Up;
  BoolSource trim_pitch_down = BoolSource::Nav1Down;
  BoolSource trim_roll_left = BoolSource::Nav1Left;
  BoolSource trim_roll_right = BoolSource::Nav1Right;
  BoolSource trim_reset = BoolSource::Nav1Center;
  // Trick triggers (rising edge).
  TrickBinding tricks[kMaxTrickBindings];
};

// The documented out-of-box binding layout (see docs/controller_bridge.md).
BindingConfig defaultBindings();

// --- Decoded command snapshot ----------------------------------------------

// Clamp envelope for body-pose offsets (mirrors protocol::motionlim so a bridge
// command can never exceed what SET_BODY_POSE would accept over USB).
namespace poselim {
constexpr float kMaxTransMm = 50.0f;
constexpr float kMaxRotRad = 0.4363f;  // ~25 deg
}  // namespace poselim

// Encoder sensitivity: raw counts that sweep a shape-trim from 0 to 1.
constexpr int32_t kEncoderCountsFullScale = 1024;
// Operator pose-trim step applied per nav press, and its clamp.
constexpr float kTrimStepRad = 0.0174533f;  // 1 deg / press
constexpr float kTrimMaxRad = poselim::kMaxRotRad;
// Minimum gap between successive trick / trim edge fires (debounce).
constexpr uint32_t kEdgeRefractoryMs = 150;
// No fresh frame within this window -> failsafe hold.
constexpr uint32_t kDefaultFailsafeMs = 250;

// One decoded command the control layer consumes each cycle. All motion fields
// are already clamped to the safe envelope.
struct ControllerCommand {
  bool valid = false;       // a usable, link-up decode produced this snapshot
  bool failsafe = true;     // link down / stale -> safe hold asserted
  bool ever_seen = false;   // any valid frame decoded since reset
  uint32_t frame_ms = 0;    // time of the last valid frame
  bool arm_request = false;  // arm switch asserted (and not failsafe)
  bool estop = true;         // kill switch asserted OR failsafe
  bool host_authority = false;  // operator hands motion authority to USB host
  ControlMode mode = ControlMode::Walk;
  uint8_t gait_index = 0;  // 0..2 from the gait-select toggle
  // Walk-mode body twist, normalised [-1, 1].
  float twist_vx = 0.0f;
  float twist_vy = 0.0f;
  float twist_wz = 0.0f;
  // Body-pose offset (translate/rotate modes), mm + rad, clamped.
  float pose_x_mm = 0.0f;
  float pose_y_mm = 0.0f;
  float pose_z_mm = 0.0f;
  float pose_roll = 0.0f;
  float pose_pitch = 0.0f;
  float pose_yaw = 0.0f;
  // Persistent operator attitude trim (rad), added on top of pose attitude.
  float trim_roll = 0.0f;
  float trim_pitch = 0.0f;
  // Live shape parameters, normalised [0, 1].
  float speed = 0.0f;
  float body_height = 0.0f;
  float stride = 0.0f;
  float step_height = 0.0f;
  // Feature toggle request levels (switch state, applied by the control layer
  // subject to availability).
  bool feat_foot_contact = false;
  bool feat_terrain_leveling = false;
  bool feat_passive_pose = false;
  // Trick fired on this frame's rising edge (None if none).
  TrickId trick = TrickId::None;
};

// --- Bridge ----------------------------------------------------------------

class ControllerBridge {
 public:
  ControllerBridge();

  // Clear all state to a safe failsafe (disarmed, kill asserted, trim zeroed).
  void reset();

  const BindingConfig& bindings() const { return cfg_; }
  // Replace the active binding map (USB override, oha.4). Edge/trim state is
  // preserved; only the routing changes.
  void setBindings(const BindingConfig& cfg) { cfg_ = cfg; }

  // Decode one fresh CRSF channel frame (raw 11-bit ticks, 0-based: ch[0]=CH1)
  // captured at `now_ms`. `link_up` is the receiver link state (false => the
  // controller is not reachable). Returns the updated command snapshot.
  const ControllerCommand& update(const uint16_t ch[CPACK_NUM_CHANNELS],
                                  bool link_up, uint32_t now_ms);

  // Re-evaluate failsafe when no new frame has arrived. Must be called
  // periodically so a silent link drops into a safe hold even between frames.
  void evaluateFailsafe(uint32_t now_ms,
                        uint32_t timeout_ms = kDefaultFailsafeMs);

  const ControllerCommand& command() const { return cmd_; }
  // The most recent raw decoded inputs (for USB raw passthrough, oha.4).
  const ChannelPackInputs_t& rawInputs() const { return raw_; }

 private:
  // Integrate the relative encoder deltas into enc_accum_[] once per frame, so
  // the axis readers below stay pure (an encoder bound to several functions is
  // counted exactly once).
  void integrateEncoders();

  // Read helpers, normalising whatever source a binding points at.
  float readAxisBipolar(const AxisBinding& b) const;   // -> [-1, 1]
  float readAxisUnipolar(const AxisBinding& b) const;  // -> [0, 1]
  bool readBool(BoolSource s) const;                   // current level
  uint8_t readTri(TriSource s) const;                  // 0=UP,1=CENTER,2=DOWN

  // Rising-edge detector with refractory debounce, keyed by a small slot index.
  bool risingEdge(uint8_t slot, bool level, uint32_t now_ms);

  void enterFailsafe(uint32_t now_ms);

  BindingConfig cfg_;
  ControllerCommand cmd_;
  ChannelPackInputs_t raw_;

  // Encoder integration state for shape trims (Enc1, Enc2).
  int32_t enc_last_[2];
  bool enc_seen_[2];
  float enc_accum_[2];  // 0..1 integrated trim per encoder

  // Edge state. Slots 0..kMaxTrickBindings-1 = trick bindings; the next 5 are
  // the trim nudges (pitch up/down, roll left/right, reset).
  static constexpr uint8_t kTrimEdgeBase = kMaxTrickBindings;
  static constexpr uint8_t kNumEdgeSlots = kMaxTrickBindings + 5;
  bool edge_prev_[kNumEdgeSlots];
  uint32_t edge_last_ms_[kNumEdgeSlots];
};

}  // namespace controller
