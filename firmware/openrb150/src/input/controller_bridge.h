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
// Control modes (the "move the core without losing locomotion" requirement):
// the LEFT gimbal always drives planar walking (forward/strafe), and SW_F
// selects the RIGHT-gimbal behavior, applied WHILE walking:
//   Yaw           - right X turns (steers) the robot.
//   TranslateBody - right gimbal shifts the body x/y over the gait.
//   RotateBody    - right gimbal tilts the body roll/pitch over the gait.
// The walking gait family is chosen independently by SW_E (wave / ripple /
// tripod) and stays live in every mode, so the operator can walk while the
// body translates or rotates. With the left stick centred the gait holds the
// planted home stance, so the body modes still "move the core without moving
// the legs".
//
// The bridge is stateful (encoder-delta shape trims, button edge-detect +
// refractory debounce, persistent operator pose trim, failsafe hold) but uses
// only fixed-size members -- no heap, safe to own from rcTask. It never touches
// Arduino APIs and never commands a servo; it only produces intent. The control
// task applies it through the same safety gate as every other command source.
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"
#include "ChannelPack.h"  // vendored: single source of truth for the channel map
#include "rc_input_conditioner.h"

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

// --- Input profile (which decoded channel LAYOUT is on the wire) ------------
//
// Both the custom ESP32 ChannelPack controller AND a RadioMaster TX16S MK3 in
// direct ELRS/EdgeTX mode arrive as ordinary CRSF *RC channels* frames -- the
// CRSF frame TYPE is identical, so detection must NOT look at the frame type.
// Instead we classify the decoded 16-channel LAYOUT: the custom controller
// packs switches/buttons/toggles/nav as bitfields into CH9..CH11 and centres
// CH12..CH16, while the TX16S direct model sends conventional per-channel
// stick/knob/switch values. See docs/controller_bridge.md.
enum class InputProfile : uint8_t {
  Unknown = 0,
  CustomControllerChannelPack,
  Tx16sMk3Direct,
};

// --- Decoded high-level outputs --------------------------------------------

// Right-gimbal behaviour selected by SW_F (or a USB override). The left gimbal
// walks in EVERY mode and SW_E owns the gait family, so this enum never decides
// whether the robot walks -- only what the right gimbal does on top of it.
// Numeric values are on the CRSF downlink; do not renumber.
enum class ControlMode : uint8_t {
  Yaw = 0,            // right X -> turn (steer) the robot
  TranslateBody = 1,  // right gimbal -> body x/y offset over the gait
  RotateBody = 2,     // right gimbal -> body roll/pitch over the gait
};
constexpr uint8_t kNumControlModes = 3;

// Live-tunable gait shape parameters, edited from the NAV1 cluster while the
// gait-tune mode is engaged and persisted to EEPROM on an explicit save.
// The numeric values are part of the CRSF status downlink, so do not renumber.
enum class GaitTuneParam : uint8_t {
  StepHeight = 0,  // swing clearance, 0..config::kMaxGaitStepMm
  Stride = 1,      // stroke length, 0..config::kMaxGaitStrideMm
  Duty = 2,        // duty factor (persisted; Mark III APG timing is fixed)
};
constexpr uint8_t kNumGaitTuneParams = 3;

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
  SpiderAttack,
  JumpKick,
};

// --- Binding configuration (remappable) ------------------------------------

// One proportional binding: which source, whether to invert it, and an
// optional legacy deadband override. A nonzero binding deadband replaces the
// source calibration deadband; it never stacks with it.
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
// Four front-panel buttons plus all five NAV2 directions.
constexpr uint8_t kMaxTrickBindings = 9;

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
  // Live shape parameters (read in every mode), normalised 0..1. speed and
  // body_height stay on the two pots. stride / step_height / duty default to
  // AxisSource::None: they are owned by the NAV1 gait-tune editor below, and
  // an axis is read only when a host explicitly binds one.
  AxisBinding speed;
  AxisBinding body_height;
  AxisBinding stride;
  AxisBinding step_height;
  AxisBinding duty;
  // Gait family + right-gimbal mode selectors (3-position toggles). SW_E picks
  // the walking gait (wave/ripple/tripod); SW_F picks what the right gimbal
  // does (yaw / translate body / rotate body).
  TriSource gait_select = TriSource::SwE;
  TriSource mode_select = TriSource::SwF;
  // Safety + feature switches (level, not edge).
  BoolSource arm = BoolSource::SwA;
  BoolSource estop = BoolSource::SwB;
  BoolSource feat_foot_contact = BoolSource::SwC;
  BoolSource feat_terrain_leveling = BoolSource::SwD;
  BoolSource feat_passive_pose = BoolSource::None;
  BoolSource host_authority = BoolSource::SwH;
  // Persistent operator pose trim (edge-nudged, fixed step per press). These
  // are NAV1 and are active only while the gait-tune mode is disengaged.
  BoolSource trim_pitch_up = BoolSource::Nav1Up;
  BoolSource trim_pitch_down = BoolSource::Nav1Down;
  BoolSource trim_roll_left = BoolSource::Nav1Left;
  BoolSource trim_roll_right = BoolSource::Nav1Right;
  BoolSource trim_reset = BoolSource::Nav1Center;
  // Gait-tune editor. The default SW_G binding is level-controlled; while it
  // is engaged the SAME NAV1 cluster edits gait parameters instead of trim.
  BoolSource gait_tune_toggle = BoolSource::SwG;
  BoolSource gait_tune_next = BoolSource::Nav1Up;
  BoolSource gait_tune_prev = BoolSource::Nav1Down;
  BoolSource gait_tune_increase = BoolSource::Nav1Right;
  BoolSource gait_tune_decrease = BoolSource::Nav1Left;
  BoolSource gait_tune_save = BoolSource::Nav1Center;
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
constexpr int32_t kEncoderCountsFullScale = 128;
// Fraction of a gait parameter's full range applied per NAV1 press. 20 presses
// sweep the whole range, which is fine-grained enough for bench tuning while
// still reaching either end quickly.
constexpr float kGaitTuneStepFrac = 0.05f;
// Operator pose-trim step applied per nav press, and its clamp.
constexpr float kTrimStepRad = 0.0174533f;  // 1 deg / press
constexpr float kTrimMaxRad = poselim::kMaxRotRad;
// Minimum gap between successive trick / trim edge fires (debounce).
constexpr uint32_t kEdgeRefractoryMs = 150;
// No fresh frame within this window -> failsafe hold.
constexpr uint32_t kDefaultFailsafeMs = 250;
// SW_A must remain released for this long before disarming. Kill and link loss
// bypass this filter and remain immediate.
constexpr uint32_t kArmReleaseDebounceMs = 150;
// Consecutive link-up frames that must agree on a layout before the input
// profile is locked. Prefer stable detection over a first-frame lock so a
// glitchy startup frame cannot pick the wrong profile.
constexpr uint8_t kProfileDetectFrames = 3;

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
  ControlMode mode = ControlMode::Yaw;
  uint8_t gait_index = 0;  // 0=Wave, 1=Ripple, 2=Tripod; SW_E, live in all modes
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
  float duty = 0.0f;
  // Gait-tune editor state (NAV1 owns the cluster while this is engaged).
  bool gait_tune_active = false;
  GaitTuneParam gait_tune_param = GaitTuneParam::StepHeight;
  // Bumped on every NAV1 edit so downstream layers can boost the telemetry
  // rate and the leg-1 preview only while the operator is actually tuning.
  uint32_t gait_tune_edit_seq = 0;
  uint32_t gait_tune_last_edit_ms = 0;
  // Bumped once per save press. The adapter compares it against the last
  // sequence it persisted, so a press can never be lost to a task boundary and
  // never applied twice.
  uint32_t gait_tune_save_seq = 0;
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

  // Adopt a schema-validated logical input calibration profile. This is called
  // only by rcTask, which owns the bridge, after a config revision hand-off.
  // Invalid profiles are ignored so a malformed API/config hand-off cannot
  // turn into arbitrary controller motion.
  void setCalibration(const config::RcInputCalibration& calibration);
  const config::RcInputCalibration& calibration() const {
    return calibration_;
  }

  bool setInputFilterMode(InputFilterMode mode, bool diagnostic_mode_allowed);
  InputFilterMode inputFilterMode() const { return conditioner_.mode(); }

  // Seed the NAV1-edited gait parameters from the persisted gait defaults so a
  // freshly booted robot reports (and commands) exactly what is stored in
  // EEPROM. Ignored while the operator has the gait-tune mode engaged, so an
  // unrelated config revision can never yank a value out from under an edit.
  // Values are fractions of the safe range (0..1) and are clamped.
  void setGaitTuneFractions(float step_height, float stride, float duty);

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

  // Input profile detected on the first stable connection (for tests/debug).
  InputProfile detectedProfile() const { return detected_profile_; }
  bool profileLocked() const { return profile_locked_; }

 private:
  // Integrate the relative encoder deltas into enc_accum_[] once per frame, so
  // the axis readers below stay pure (an encoder bound to several functions is
  // counted exactly once). Custom-controller profile only.
  void integrateEncoders();

  // --- Input-profile detection (layout-level, not CRSF frame type) ----------
  // Tolerant "near CRSF centre" test for the reserved channels.
  static bool nearCrsfMid(uint16_t v);
  // True if the decoded layout matches a packed custom ChannelPack frame.
  static bool looksLikeCustomChannelPack(const uint16_t ch[CPACK_NUM_CHANNELS]);
  // Classify a single frame's layout (no streak/lock state).
  static InputProfile classifyFirstFrame(const uint16_t ch[CPACK_NUM_CHANNELS]);
  // Accumulate agreeing frames and lock a profile once stable.
  void attemptProfileDetection(const uint16_t ch[CPACK_NUM_CHANNELS]);

  // Decode a conventional TX16S MK3 direct ELRS/EdgeTX frame into raw_ fields.
  static void unpackTx16sMk3DirectChannels(
      const uint16_t ch[CPACK_NUM_CHANNELS], ChannelPackInputs_t* out);
  // Absolute (not relative) virtual-encoder handling for the TX16S direct
  // sliders on CH7/CH8. Drives enc_accum_[] directly, bypassing integration.
  void updateTx16sDirectVirtualEncoders(const uint16_t ch[CPACK_NUM_CHANNELS]);

  // Read helpers, normalising whatever source a binding points at.
  float readAxisBipolar(const AxisBinding& b) const;   // -> [-1, 1]
  float readAxisUnipolar(const AxisBinding& b) const;  // -> [0, 1]
  const config::RcChannelCalibration* calibrationFor(AxisSource source) const;
  bool readCalibratedBipolar(AxisSource source, int16_t raw, float& out) const;
  bool readCalibratedUnipolar(AxisSource source, int16_t raw, float& out) const;
  void updateConditionedInputs(uint32_t now_ms);
  void updateConditionedToggles(uint32_t now_ms);
  float conditionedBipolar(AxisSource source) const;
  float conditionedUnipolar(AxisSource source) const;
  float applyBindingResponse(const AxisBinding& binding, float value) const;
  bool readBool(BoolSource s) const;                   // current level
  uint8_t readTri(TriSource s) const;                  // 0=UP,1=CENTER,2=DOWN

  // Rising-edge detector with refractory debounce, keyed by a small slot index.
  bool risingEdge(uint8_t slot, bool level, uint32_t now_ms);

  // Apply the NAV1 cluster: gait-parameter editing while the tune mode is
  // engaged, otherwise the persistent operator pose trim.
  void updateGaitTune(uint32_t now_ms);
  void updatePoseTrim(uint32_t now_ms);

  void enterFailsafe(uint32_t now_ms);

  BindingConfig cfg_;
  config::RcInputCalibration calibration_;
  RcInputConditioner conditioner_;
  ControllerCommand cmd_;
  ChannelPackInputs_t raw_;
  float conditioned_gimbal_[4] = {};
  float conditioned_pot_[2] = {};
  float conditioned_encoder_[2] = {};
  uint8_t conditioned_toggles_[2] = {1, 1};
  RcTriSwitchDebouncer sw_e_debounce_;
  RcTriSwitchDebouncer sw_f_debounce_;
  uint32_t last_condition_ms_ = 0;
  bool condition_time_seen_ = false;

  // Input-profile detection state. Locked once, never auto-switches until
  // reset(); a link drop after lock keeps the same profile.
  InputProfile detected_profile_ = InputProfile::Unknown;
  bool profile_locked_ = false;
  uint8_t custom_layout_streak_ = 0;
  uint8_t tx_direct_layout_streak_ = 0;

  // Encoder integration state for shape trims (Enc1, Enc2).
  int32_t enc_last_[2];
  bool enc_seen_[2];
  float enc_accum_[2];  // 0..1 integrated trim per encoder

  bool arm_release_pending_ = false;
  uint32_t arm_release_since_ms_ = 0;

  // NAV1-edited gait shape, as fractions of the safe range. Index order is
  // GaitTuneParam (step height, stride, duty). Survives mode changes and link
  // drops; only setGaitTuneFractions() or a NAV1 edit changes it.
  float gait_tune_frac_[kNumGaitTuneParams];

  // Edge state. Slots 0..kMaxTrickBindings-1 = trick bindings; the next 5 are
  // the trim nudges (pitch up/down, roll left/right, reset); the last 6 are the
  // gait-tune editor (toggle, next, prev, increase, decrease, save).
  static constexpr uint8_t kTrimEdgeBase = kMaxTrickBindings;
  static constexpr uint8_t kGaitTuneEdgeBase = kTrimEdgeBase + 5;
  static constexpr uint8_t kNumEdgeSlots = kGaitTuneEdgeBase + 6;
  bool edge_prev_[kNumEdgeSlots];
  uint32_t edge_last_ms_[kNumEdgeSlots];
};

}  // namespace controller
