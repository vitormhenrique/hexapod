// ===========================================================================
// Controller bridge implementation (oha.2). See controller_bridge.h.
//
// Portable, heap-free, Arduino-free so it builds in the native test env. Only
// depends on the vendored ChannelPack.h for the wire layout.
// ===========================================================================

#include "controller_bridge.h"

namespace controller {

namespace {

// Clamp a float to [lo, hi].
inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Apply a symmetric centre deadband (fraction of full scale) to a [-1,1] value
// and rescale so the response is continuous from the deadband edge to +/-1.
inline float applyDeadband(float v, float db) {
  if (db <= 0.0f) return v;
  if (db >= 1.0f) return 0.0f;
  const float a = v < 0.0f ? -v : v;
  if (a <= db) return 0.0f;
  const float scaled = (a - db) / (1.0f - db);
  return v < 0.0f ? -scaled : scaled;
}

// Map a 0..1000 pot reading to a unipolar 0..1.
inline float potUnit(int16_t pot) {
  return clampf(static_cast<float>(pot) / 1000.0f, 0.0f, 1.0f);
}

// Map a -1000..1000 gimbal reading to a bipolar -1..1.
inline float gimbalUnit(int16_t g) {
  return clampf(static_cast<float>(g) / 1000.0f, -1.0f, 1.0f);
}

// --- Conventional-CRSF normalisation (TX16S MK3 direct profile) ------------
//
// TX16S direct frames carry ordinary per-channel stick/knob/switch values that
// may sit slightly outside the ChannelPack 191..1792 span, so clamp before any
// unsigned subtraction to avoid underflow.
inline uint16_t clampCrsf(uint16_t v) {
  if (v < CPACK_CRSF_MIN) return CPACK_CRSF_MIN;
  if (v > CPACK_CRSF_MAX) return CPACK_CRSF_MAX;
  return v;
}

// Conventional CRSF value -> unit 0..1.
inline float crsfUnit01(uint16_t v) {
  v = clampCrsf(v);
  return static_cast<float>(v - CPACK_CRSF_MIN) /
         static_cast<float>(CPACK_CRSF_MAX - CPACK_CRSF_MIN);
}

// Conventional CRSF value -> bipolar gimbal ticks (-1000..+1000).
inline int16_t crsfToGimbalSafe(uint16_t v) {
  return static_cast<int16_t>(crsfUnit01(v) * 2000.0f - 1000.0f);
}

// Conventional CRSF value -> unipolar pot ticks (0..1000).
inline int16_t crsfToPotSafe(uint16_t v) {
  return static_cast<int16_t>(crsfUnit01(v) * 1000.0f);
}

// Conventional CRSF value -> 3-position toggle (0=UP, 1=CENTER, 2=DOWN).
inline uint8_t crsfToTri(uint16_t v) {
  const float u = crsfUnit01(v);
  if (u < 0.3333f) return 0;
  if (u < 0.6666f) return 1;
  return 2;
}

// Conventional CRSF value -> bool (high half).
inline bool crsfToBoolHigh(uint16_t v) { return crsfUnit01(v) > 0.5f; }

// Conventional CRSF value -> one of `bins` discrete positions [0, bins-1].
inline uint8_t crsfToBins(uint16_t v, uint8_t bins) {
  if (bins == 0) return 0;
  const float u = crsfUnit01(v);
  int idx = static_cast<int>(u * static_cast<float>(bins));
  if (idx < 0) idx = 0;
  if (idx >= static_cast<int>(bins)) idx = static_cast<int>(bins) - 1;
  return static_cast<uint8_t>(idx);
}

// Reset every raw input to a neutral/false state before a TX16S direct decode
// fills only the mapped fields (never inherit last-frame contents).
inline void clearRawInputs(ChannelPackInputs_t* out) {
  for (int i = 0; i < 4; ++i) out->gimbal[i] = 0;
  for (int i = 0; i < 2; ++i) out->pot[i] = 0;
  for (int i = 0; i < 2; ++i) out->encoder[i] = 0;
  for (int i = 0; i < 8; ++i) out->switches[i] = false;
  for (int i = 0; i < 4; ++i) out->buttons[i] = false;
  for (int i = 0; i < 2; ++i) out->toggles[i] = 1;  // neutral/CENTER
  for (int n = 0; n < 2; ++n) {
    for (int d = 0; d < 5; ++d) out->nav[n][d] = false;
  }
}

}  // namespace

BindingConfig defaultBindings() {
  BindingConfig c;
  // Left gimbal walks; right gimbal does body work / strafe.
  c.walk_forward = {AxisSource::GimbalLY, false, 0.05f};
  c.walk_yaw = {AxisSource::GimbalLX, false, 0.05f};
  c.walk_strafe = {AxisSource::GimbalRX, false, 0.05f};
  // Translate-body: right gimbal shifts x/y, left-Y lifts/lowers the body.
  c.body_x = {AxisSource::GimbalRY, false, 0.05f};
  c.body_y = {AxisSource::GimbalRX, false, 0.05f};
  c.body_z = {AxisSource::GimbalLY, false, 0.05f};
  // Rotate-body: right gimbal = roll/pitch, left-X = yaw.
  c.body_roll = {AxisSource::GimbalRX, false, 0.05f};
  c.body_pitch = {AxisSource::GimbalRY, false, 0.05f};
  c.body_yaw = {AxisSource::GimbalLX, false, 0.05f};
  // Shape params: pots are absolute, encoders trim stride / step clearance.
  c.speed = {AxisSource::Pot1, false, 0.0f};
  c.body_height = {AxisSource::Pot2, false, 0.0f};
  c.stride = {AxisSource::Enc1, false, 0.0f};
  c.step_height = {AxisSource::Enc2, false, 0.0f};
  // Selectors.
  c.mode_select = TriSource::SwE;
  c.gait_select = TriSource::SwF;
  // Safety + features.
  c.arm = BoolSource::SwA;
  c.estop = BoolSource::SwB;
  c.feat_foot_contact = BoolSource::SwC;
  c.feat_terrain_leveling = BoolSource::SwD;
  c.feat_passive_pose = BoolSource::SwG;
  c.host_authority = BoolSource::SwH;
  // Operator pose trim on NAV1.
  c.trim_pitch_up = BoolSource::Nav1Up;
  c.trim_pitch_down = BoolSource::Nav1Down;
  c.trim_roll_left = BoolSource::Nav1Left;
  c.trim_roll_right = BoolSource::Nav1Right;
  c.trim_reset = BoolSource::Nav1Center;
  // Tricks: 4 buttons + NAV2 cluster.
  c.tricks[0] = {BoolSource::Btn1, TrickId::StandUp};
  c.tricks[1] = {BoolSource::Btn2, TrickId::SitDown};
  c.tricks[2] = {BoolSource::Btn3, TrickId::Wave};
  c.tricks[3] = {BoolSource::Btn4, TrickId::CrouchToggle};
  c.tricks[4] = {BoolSource::Nav2Up, TrickId::Twirl};
  c.tricks[5] = {BoolSource::Nav2Down, TrickId::Stretch};
  c.tricks[6] = {BoolSource::Nav2Left, TrickId::LeanLook};
  c.tricks[7] = {BoolSource::Nav2Center, TrickId::DanceLoop};
  return c;
}

ControllerBridge::ControllerBridge() {
  cfg_ = defaultBindings();
  reset();
}

void ControllerBridge::reset() {
  cmd_ = ControllerCommand();
  raw_ = ChannelPackInputs_t();
  detected_profile_ = InputProfile::Unknown;
  profile_locked_ = false;
  custom_layout_streak_ = 0;
  tx_direct_layout_streak_ = 0;
  for (uint8_t i = 0; i < 2; ++i) {
    enc_last_[i] = 0;
    enc_seen_[i] = false;
    enc_accum_[i] = 0.5f;  // start shape trims at mid-scale
  }
  for (uint8_t i = 0; i < kNumEdgeSlots; ++i) {
    edge_prev_[i] = false;
    // Seed one refractory window in the past (unsigned wrap) so the very first
    // press fires immediately instead of being debounced against t=0.
    edge_last_ms_[i] = 0u - kEdgeRefractoryMs;
  }
}

void ControllerBridge::integrateEncoders() {
  const int32_t enc[2] = {raw_.encoder[0], raw_.encoder[1]};
  for (uint8_t i = 0; i < 2; ++i) {
    if (!enc_seen_[i]) {
      enc_last_[i] = enc[i];
      enc_seen_[i] = true;
      continue;
    }
    // Encoders are relative and wrap at 0 <-> 2047; resolve the shortest signed
    // delta across the boundary.
    int32_t d = enc[i] - enc_last_[i];
    if (d > 1024) d -= 2048;
    if (d < -1024) d += 2048;
    enc_last_[i] = enc[i];
    enc_accum_[i] = clampf(enc_accum_[i] + static_cast<float>(d) /
                                              static_cast<float>(
                                                  kEncoderCountsFullScale),
                           0.0f, 1.0f);
  }
}

// --- Input-profile detection (decoded LAYOUT, never CRSF frame type) --------

bool ControllerBridge::nearCrsfMid(uint16_t v) {
  const int d = static_cast<int>(v) - static_cast<int>(CPACK_CRSF_MID);
  return (d < 0 ? -d : d) <= 16;
}

bool ControllerBridge::looksLikeCustomChannelPack(
    const uint16_t ch[CPACK_NUM_CHANNELS]) {
  // CH9: packed 2-pos switches. Only bits 0..5 (SwA,B,C,D,G,H) are used by the
  // current robot-side abstraction; bits 6..7 are reserved and must be clear.
  const bool switches_ok = (ch[CPACK_CH_SWITCHES] & ~uint16_t(0x003F)) == 0;

  // CH10: 4 buttons (bits 0..3) + SwE tri (bits 4..5) + SwF tri (bits 6..7).
  // A tri field of 3 is invalid, and nothing above bit 7 may be set.
  const uint16_t bt = ch[CPACK_CH_BTN_TOGGLE];
  const bool btn_toggle_ok = (bt & ~uint16_t(0x00FF)) == 0 &&
                             (((bt >> 4) & 0x03) <= 2) &&
                             (((bt >> 6) & 0x03) <= 2);

  // CH11: packed nav (Nav1 bits 0..4, Nav2 bits 5..9); nothing above bit 9.
  const bool nav_ok = (ch[CPACK_CH_NAV] & ~uint16_t(0x03FF)) == 0;

  // CH12..CH16 are reserved and packed near CRSF centre by the TX.
  bool reserved_ok = true;
  for (int i = 11; i < CPACK_NUM_CHANNELS; ++i) {
    reserved_ok = reserved_ok && nearCrsfMid(ch[i]);
  }

  return switches_ok && btn_toggle_ok && nav_ok && reserved_ok;
}

InputProfile ControllerBridge::classifyFirstFrame(
    const uint16_t ch[CPACK_NUM_CHANNELS]) {
  return looksLikeCustomChannelPack(ch) ? InputProfile::CustomControllerChannelPack
                                        : InputProfile::Tx16sMk3Direct;
}

void ControllerBridge::attemptProfileDetection(
    const uint16_t ch[CPACK_NUM_CHANNELS]) {
  if (profile_locked_) return;

  if (looksLikeCustomChannelPack(ch)) {
    if (custom_layout_streak_ < kProfileDetectFrames) ++custom_layout_streak_;
    tx_direct_layout_streak_ = 0;
    if (custom_layout_streak_ >= kProfileDetectFrames) {
      detected_profile_ = InputProfile::CustomControllerChannelPack;
      profile_locked_ = true;
    }
  } else {
    if (tx_direct_layout_streak_ < kProfileDetectFrames) {
      ++tx_direct_layout_streak_;
    }
    custom_layout_streak_ = 0;
    if (tx_direct_layout_streak_ >= kProfileDetectFrames) {
      detected_profile_ = InputProfile::Tx16sMk3Direct;
      profile_locked_ = true;
    }
  }
}

// --- TX16S MK3 direct ELRS/EdgeTX decode ------------------------------------
//
// Conventional per-channel CRSF values from the saved ROBOT_TX16S_DIRECT model
// are mapped into the SAME logical raw_ fields the custom controller produces,
// so all downstream binding/command logic is unchanged. See the channel map in
// docs/controller_bridge.md and docs/ROBOT_TX16S_channel_map.csv.
void ControllerBridge::unpackTx16sMk3DirectChannels(
    const uint16_t ch[CPACK_NUM_CHANNELS], ChannelPackInputs_t* out) {
  clearRawInputs(out);

  // CH1..CH4: Rud/Thr/Ail/Ele -> gimbals LX, LY, RX, RY.
  out->gimbal[0] = crsfToGimbalSafe(ch[0]);
  out->gimbal[1] = crsfToGimbalSafe(ch[1]);
  out->gimbal[2] = crsfToGimbalSafe(ch[2]);
  out->gimbal[3] = crsfToGimbalSafe(ch[3]);

  // CH5..CH6: S1/S2 knobs -> Pot1 (speed), Pot2 (body height).
  out->pot[0] = crsfToPotSafe(ch[4]);
  out->pot[1] = crsfToPotSafe(ch[5]);

  // CH9..CH10: SE/SD -> SwE mode select, SwF gait select (3-position).
  out->toggles[0] = crsfToTri(ch[8]);
  out->toggles[1] = crsfToTri(ch[9]);

  // CH11: SA/SF mix -> 2-bit safety mask (bit0=arm/SwA, bit1=estop/SwB).
  const uint8_t safety_mask = crsfToBins(ch[10], 4);
  out->switches[0] = (safety_mask & 0x01) != 0;  // SwA arm
  out->switches[1] = (safety_mask & 0x02) != 0;  // SwB estop

  // CH12: SB/SC/SG mix -> 4-bit feature mask. bit3/host_authority stays false
  // until the EdgeTX mix produces it, but the robot side is always 4-bit.
  const uint8_t feature_mask = crsfToBins(ch[11], 16);
  out->switches[2] = (feature_mask & 0x01) != 0;  // SwC foot contact
  out->switches[3] = (feature_mask & 0x02) != 0;  // SwD terrain leveling
  out->switches[4] = (feature_mask & 0x04) != 0;  // SwG passive pose
  out->switches[5] = (feature_mask & 0x08) != 0;  // SwH host authority

  // CH13/CH14: GV1 ACT action selector (13 positions) gated by SH action fire.
  // Selecting an action arms exactly one existing button/nav boolean so the
  // downstream rising-edge debounce keeps working. Nav2Right is deliberately
  // unmapped and must stay false.
  const uint8_t action = crsfToBins(ch[12], 13);
  const bool fire = crsfToBoolHigh(ch[13]);
  if (fire) {
    switch (action) {
      case 0:  out->nav[0][CPACK_NAV_UP] = true; break;      // trim_pitch_up
      case 1:  out->nav[0][CPACK_NAV_DOWN] = true; break;    // trim_pitch_down
      case 2:  out->nav[0][CPACK_NAV_LEFT] = true; break;    // trim_roll_left
      case 3:  out->nav[0][CPACK_NAV_RIGHT] = true; break;   // trim_roll_right
      case 4:  out->nav[0][CPACK_NAV_CENTER] = true; break;  // trim_reset
      case 5:  out->buttons[0] = true; break;                // StandUp
      case 6:  out->buttons[1] = true; break;                // SitDown
      case 7:  out->buttons[2] = true; break;                // Wave
      case 8:  out->buttons[3] = true; break;                // CrouchToggle
      case 9:  out->nav[1][CPACK_NAV_UP] = true; break;      // Twirl
      case 10: out->nav[1][CPACK_NAV_DOWN] = true; break;    // Stretch
      case 11: out->nav[1][CPACK_NAV_LEFT] = true; break;    // LeanLook
      case 12: out->nav[1][CPACK_NAV_CENTER] = true; break;  // DanceLoop
      default: break;
    }
  }
  // CH15/CH16 are EdgeTX "MAX 0" -> centred/reserved, no logical effect.
}

void ControllerBridge::updateTx16sDirectVirtualEncoders(
    const uint16_t ch[CPACK_NUM_CHANNELS]) {
  // CH7/CH8 (LS/RS sliders) are ABSOLUTE controls on the TX16S -- there is no
  // physical relative encoder, so drive enc_accum_[] straight from the value
  // instead of integrating a wrap-delta. readAxisUnipolar(Enc1/Enc2) then keeps
  // returning enc_accum_[0/1] so stride/step_height work with defaultBindings().
  const float enc0 = crsfUnit01(ch[6]);
  const float enc1 = crsfUnit01(ch[7]);

  raw_.encoder[0] = static_cast<int32_t>(enc0 * 2047.0f);
  raw_.encoder[1] = static_cast<int32_t>(enc1 * 2047.0f);

  enc_accum_[0] = clampf(enc0, 0.0f, 1.0f);
  enc_accum_[1] = clampf(enc1, 0.0f, 1.0f);

  enc_seen_[0] = true;
  enc_seen_[1] = true;
  enc_last_[0] = raw_.encoder[0];
  enc_last_[1] = raw_.encoder[1];
}

float ControllerBridge::readAxisBipolar(const AxisBinding& b) const {
  float v = 0.0f;
  switch (b.source) {
    case AxisSource::GimbalLX: v = gimbalUnit(raw_.gimbal[0]); break;
    case AxisSource::GimbalLY: v = gimbalUnit(raw_.gimbal[1]); break;
    case AxisSource::GimbalRX: v = gimbalUnit(raw_.gimbal[2]); break;
    case AxisSource::GimbalRY: v = gimbalUnit(raw_.gimbal[3]); break;
    case AxisSource::Pot1: v = potUnit(raw_.pot[0]) * 2.0f - 1.0f; break;
    case AxisSource::Pot2: v = potUnit(raw_.pot[1]) * 2.0f - 1.0f; break;
    case AxisSource::Enc1: v = enc_accum_[0] * 2.0f - 1.0f; break;
    case AxisSource::Enc2: v = enc_accum_[1] * 2.0f - 1.0f; break;
    case AxisSource::None: return 0.0f;
  }
  v = applyDeadband(v, b.deadband);
  if (b.invert) v = -v;
  return clampf(v, -1.0f, 1.0f);
}

float ControllerBridge::readAxisUnipolar(const AxisBinding& b) const {
  float v = 0.0f;
  switch (b.source) {
    case AxisSource::GimbalLX: v = (gimbalUnit(raw_.gimbal[0]) + 1.0f) * 0.5f; break;
    case AxisSource::GimbalLY: v = (gimbalUnit(raw_.gimbal[1]) + 1.0f) * 0.5f; break;
    case AxisSource::GimbalRX: v = (gimbalUnit(raw_.gimbal[2]) + 1.0f) * 0.5f; break;
    case AxisSource::GimbalRY: v = (gimbalUnit(raw_.gimbal[3]) + 1.0f) * 0.5f; break;
    case AxisSource::Pot1: v = potUnit(raw_.pot[0]); break;
    case AxisSource::Pot2: v = potUnit(raw_.pot[1]); break;
    case AxisSource::Enc1: v = enc_accum_[0]; break;
    case AxisSource::Enc2: v = enc_accum_[1]; break;
    case AxisSource::None: return 0.0f;
  }
  if (b.invert) v = 1.0f - v;
  return clampf(v, 0.0f, 1.0f);
}

bool ControllerBridge::readBool(BoolSource s) const {
  switch (s) {
    case BoolSource::None: return false;
    case BoolSource::SwA: return raw_.switches[0];
    case BoolSource::SwB: return raw_.switches[1];
    case BoolSource::SwC: return raw_.switches[2];
    case BoolSource::SwD: return raw_.switches[3];
    case BoolSource::SwG: return raw_.switches[4];
    case BoolSource::SwH: return raw_.switches[5];
    case BoolSource::Btn1: return raw_.buttons[0];
    case BoolSource::Btn2: return raw_.buttons[1];
    case BoolSource::Btn3: return raw_.buttons[2];
    case BoolSource::Btn4: return raw_.buttons[3];
    case BoolSource::Nav1Up: return raw_.nav[0][CPACK_NAV_UP];
    case BoolSource::Nav1Down: return raw_.nav[0][CPACK_NAV_DOWN];
    case BoolSource::Nav1Left: return raw_.nav[0][CPACK_NAV_LEFT];
    case BoolSource::Nav1Right: return raw_.nav[0][CPACK_NAV_RIGHT];
    case BoolSource::Nav1Center: return raw_.nav[0][CPACK_NAV_CENTER];
    case BoolSource::Nav2Up: return raw_.nav[1][CPACK_NAV_UP];
    case BoolSource::Nav2Down: return raw_.nav[1][CPACK_NAV_DOWN];
    case BoolSource::Nav2Left: return raw_.nav[1][CPACK_NAV_LEFT];
    case BoolSource::Nav2Right: return raw_.nav[1][CPACK_NAV_RIGHT];
    case BoolSource::Nav2Center: return raw_.nav[1][CPACK_NAV_CENTER];
  }
  return false;
}

uint8_t ControllerBridge::readTri(TriSource s) const {
  switch (s) {
    case TriSource::None: return 1;  // treat unmapped as CENTER
    case TriSource::SwE: return raw_.toggles[0];
    case TriSource::SwF: return raw_.toggles[1];
  }
  return 1;
}

bool ControllerBridge::risingEdge(uint8_t slot, bool level, uint32_t now_ms) {
  bool fired = false;
  if (level && !edge_prev_[slot] &&
      (now_ms - edge_last_ms_[slot]) >= kEdgeRefractoryMs) {
    fired = true;
    edge_last_ms_[slot] = now_ms;
  }
  edge_prev_[slot] = level;
  return fired;
}

void ControllerBridge::enterFailsafe(uint32_t now_ms) {
  (void)now_ms;
  // Preserve ever_seen, trim, and the last selected mode/gait, but force a safe
  // hold: no motion, disarmed, kill asserted, no trick.
  cmd_.valid = false;
  cmd_.failsafe = true;
  cmd_.arm_request = false;
  cmd_.estop = true;
  cmd_.twist_vx = cmd_.twist_vy = cmd_.twist_wz = 0.0f;
  cmd_.pose_x_mm = cmd_.pose_y_mm = cmd_.pose_z_mm = 0.0f;
  cmd_.pose_roll = cmd_.pose_pitch = cmd_.pose_yaw = 0.0f;
  cmd_.trick = TrickId::None;
}

const ControllerCommand& ControllerBridge::update(
    const uint16_t ch[CPACK_NUM_CHANNELS], bool link_up, uint32_t now_ms) {
  if (!link_up) {
    // A drop before a profile is locked clears pending streaks so stale startup
    // frames cannot combine with later frames to lock the wrong layout. Once a
    // profile is locked it survives link loss (no re-detection on return).
    if (!profile_locked_) {
      detected_profile_ = InputProfile::Unknown;
      custom_layout_streak_ = 0;
      tx_direct_layout_streak_ = 0;
    }
    enterFailsafe(now_ms);
    return cmd_;
  }

  // On the first stable connection, classify the decoded layout. Until a profile
  // stabilises, hold failsafe rather than emit commands from an unknown layout.
  if (!profile_locked_) {
    attemptProfileDetection(ch);
    if (!profile_locked_) {
      enterFailsafe(now_ms);
      return cmd_;
    }
  }

  if (detected_profile_ == InputProfile::CustomControllerChannelPack) {
    ChannelPack::unpackChannels(ch, &raw_);
    integrateEncoders();  // relative wrap-delta integration (custom only)
  } else if (detected_profile_ == InputProfile::Tx16sMk3Direct) {
    unpackTx16sMk3DirectChannels(ch, &raw_);
    updateTx16sDirectVirtualEncoders(ch);  // absolute slider passthrough
  } else {
    enterFailsafe(now_ms);
    return cmd_;
  }

  cmd_.valid = true;
  cmd_.failsafe = false;
  cmd_.ever_seen = true;
  cmd_.frame_ms = now_ms;

  // Mode + gait selectors.
  const uint8_t mode_v = readTri(cfg_.mode_select);
  cmd_.mode = static_cast<ControlMode>(mode_v < kNumControlModes ? mode_v : 0);
  cmd_.gait_index = readTri(cfg_.gait_select);

  // Safety levels.
  const bool kill = readBool(cfg_.estop);
  cmd_.estop = kill;  // not in failsafe here
  cmd_.arm_request = readBool(cfg_.arm) && !kill;
  cmd_.host_authority = readBool(cfg_.host_authority);

  // Feature toggle request levels.
  cmd_.feat_foot_contact = readBool(cfg_.feat_foot_contact);
  cmd_.feat_terrain_leveling = readBool(cfg_.feat_terrain_leveling);
  cmd_.feat_passive_pose = readBool(cfg_.feat_passive_pose);

  // Shape params (read in every mode).
  cmd_.speed = readAxisUnipolar(cfg_.speed);
  cmd_.body_height = readAxisUnipolar(cfg_.body_height);
  cmd_.stride = readAxisUnipolar(cfg_.stride);
  cmd_.step_height = readAxisUnipolar(cfg_.step_height);

  // Mode-specific motion. In a body mode the feet stay planted (twist = 0); in
  // walk mode the body pose offset is held at 0. The persistent operator trim
  // is always carried so a standing lean survives a mode change.
  cmd_.twist_vx = cmd_.twist_vy = cmd_.twist_wz = 0.0f;
  cmd_.pose_x_mm = cmd_.pose_y_mm = cmd_.pose_z_mm = 0.0f;
  cmd_.pose_roll = cmd_.pose_pitch = cmd_.pose_yaw = 0.0f;
  switch (cmd_.mode) {
    case ControlMode::Walk:
      cmd_.twist_vx = readAxisBipolar(cfg_.walk_forward);
      cmd_.twist_vy = readAxisBipolar(cfg_.walk_strafe);
      cmd_.twist_wz = readAxisBipolar(cfg_.walk_yaw);
      break;
    case ControlMode::TranslateBody:
      cmd_.pose_x_mm = readAxisBipolar(cfg_.body_x) * poselim::kMaxTransMm;
      cmd_.pose_y_mm = readAxisBipolar(cfg_.body_y) * poselim::kMaxTransMm;
      cmd_.pose_z_mm = readAxisBipolar(cfg_.body_z) * poselim::kMaxTransMm;
      break;
    case ControlMode::RotateBody:
      cmd_.pose_roll = readAxisBipolar(cfg_.body_roll) * poselim::kMaxRotRad;
      cmd_.pose_pitch = readAxisBipolar(cfg_.body_pitch) * poselim::kMaxRotRad;
      cmd_.pose_yaw = readAxisBipolar(cfg_.body_yaw) * poselim::kMaxRotRad;
      break;
  }

  // Operator pose trim (edge-nudged). Reset zeroes it.
  if (risingEdge(kTrimEdgeBase + 4, readBool(cfg_.trim_reset), now_ms)) {
    cmd_.trim_roll = 0.0f;
    cmd_.trim_pitch = 0.0f;
  }
  if (risingEdge(kTrimEdgeBase + 0, readBool(cfg_.trim_pitch_up), now_ms)) {
    cmd_.trim_pitch = clampf(cmd_.trim_pitch + kTrimStepRad, -kTrimMaxRad,
                             kTrimMaxRad);
  }
  if (risingEdge(kTrimEdgeBase + 1, readBool(cfg_.trim_pitch_down), now_ms)) {
    cmd_.trim_pitch = clampf(cmd_.trim_pitch - kTrimStepRad, -kTrimMaxRad,
                             kTrimMaxRad);
  }
  if (risingEdge(kTrimEdgeBase + 2, readBool(cfg_.trim_roll_left), now_ms)) {
    cmd_.trim_roll = clampf(cmd_.trim_roll - kTrimStepRad, -kTrimMaxRad,
                            kTrimMaxRad);
  }
  if (risingEdge(kTrimEdgeBase + 3, readBool(cfg_.trim_roll_right), now_ms)) {
    cmd_.trim_roll = clampf(cmd_.trim_roll + kTrimStepRad, -kTrimMaxRad,
                            kTrimMaxRad);
  }

  // Tricks: first binding whose source rises this frame wins (one per frame).
  cmd_.trick = TrickId::None;
  for (uint8_t i = 0; i < kMaxTrickBindings; ++i) {
    const TrickBinding& tb = cfg_.tricks[i];
    if (tb.source == BoolSource::None || tb.trick == TrickId::None) {
      // Keep the edge slot's prev state coherent even for empty bindings.
      edge_prev_[i] = false;
      continue;
    }
    if (risingEdge(i, readBool(tb.source), now_ms) &&
        cmd_.trick == TrickId::None) {
      cmd_.trick = tb.trick;
    }
  }

  return cmd_;
}

void ControllerBridge::evaluateFailsafe(uint32_t now_ms, uint32_t timeout_ms) {
  if (!cmd_.ever_seen) {
    enterFailsafe(now_ms);
    return;
  }
  if ((now_ms - cmd_.frame_ms) > timeout_ms) {
    enterFailsafe(now_ms);
  }
}

}  // namespace controller
