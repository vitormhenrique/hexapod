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

constexpr int16_t kCalibrationOutOfRangeTolerance = 64;
constexpr uint16_t kModeSwitchDebounceMs = 40;

bool rawWithinCalibrationWindow(int16_t raw,
                                const config::RcChannelCalibration& c) {
  return raw >= static_cast<int32_t>(c.min_raw) -
                    kCalibrationOutOfRangeTolerance &&
         raw <= static_cast<int32_t>(c.max_raw) +
                    kCalibrationOutOfRangeTolerance;
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
  // Left gimbal owns planar walking; right X turns the robot in Yaw mode.
  // Hardware gimbals report stick-left as negative raw; invert strafe/yaw so
  // operator left maps to command-frame left(+) and yaw CCW(+).
  c.walk_forward = {AxisSource::GimbalLY, false, 0.0f};
  c.walk_strafe = {AxisSource::GimbalLX, true, 0.0f};
  c.walk_yaw = {AxisSource::GimbalRX, true, 0.0f};
  // Translate-body overlay: right gimbal shifts the body fore/aft + lateral
  // while the left gimbal keeps walking. Z stays on the height pot.
  c.body_x = {AxisSource::GimbalRY, false, 0.0f};
  c.body_y = {AxisSource::GimbalRX, false, 0.0f};
  c.body_z = {AxisSource::None, false, 0.0f};
  // Rotate-body overlay: right gimbal = roll/pitch while the left gimbal keeps
  // planar walking. Robot yaw is intentionally disabled in this pose mode: the
  // right gimbal only has two axes and both carry the attitude overlay.
  c.body_roll = {AxisSource::GimbalRX, false, 0.0f};
  c.body_pitch = {AxisSource::GimbalRY, false, 0.0f};
  c.body_yaw = {AxisSource::None, false, 0.0f};
  // Shape params: pots are absolute; stride / step height / duty are owned by
  // the NAV1 gait-tune editor, so no axis is bound to them by default (this
  // frees ENC1/ENC2). A host may still bind an axis to override the editor.
  c.speed = {AxisSource::Pot1, false, 0.0f};
  c.body_height = {AxisSource::Pot2, false, 0.0f};
  c.stride = {AxisSource::None, false, 0.0f};
  c.step_height = {AxisSource::None, false, 0.0f};
  c.duty = {AxisSource::None, false, 0.0f};
  // Selectors: SW_E picks the walking gait, SW_F picks the right-gimbal mode.
  c.gait_select = TriSource::SwE;
  c.mode_select = TriSource::SwF;
  // Safety + features.
  c.arm = BoolSource::SwA;
  c.estop = BoolSource::SwB;
  c.feat_foot_contact = BoolSource::SwC;
  c.feat_terrain_leveling = BoolSource::SwD;
  c.feat_passive_pose = BoolSource::None;
  c.host_authority = BoolSource::SwH;
  // Operator pose trim on NAV1 (gait-tune mode borrows the same cluster).
  c.trim_pitch_up = BoolSource::Nav1Up;
  c.trim_pitch_down = BoolSource::Nav1Down;
  c.trim_roll_left = BoolSource::Nav1Left;
  c.trim_roll_right = BoolSource::Nav1Right;
  c.trim_reset = BoolSource::Nav1Center;
  // Gait-tune editor: SW_G owns the mode; NAV1 edits and saves.
  c.gait_tune_toggle = BoolSource::SwG;
  c.gait_tune_next = BoolSource::Nav1Up;
  c.gait_tune_prev = BoolSource::Nav1Down;
  c.gait_tune_increase = BoolSource::Nav1Right;
  c.gait_tune_decrease = BoolSource::Nav1Left;
  c.gait_tune_save = BoolSource::Nav1Center;
  // BTN_4 is dedicated to SD capture; the other buttons and NAV2 trigger tricks.
  c.tricks[0] = {BoolSource::Btn1, TrickId::StandUp};
  c.tricks[1] = {BoolSource::Btn2, TrickId::SitDown};
  c.tricks[2] = {BoolSource::Btn3, TrickId::Wave};
  c.tricks[4] = {BoolSource::Nav2Up, TrickId::Twirl};
  c.tricks[5] = {BoolSource::Nav2Down, TrickId::Stretch};
  c.tricks[6] = {BoolSource::Nav2Left, TrickId::JumpKick};
  c.tricks[7] = {BoolSource::Nav2Right, TrickId::SpiderAttack};
  c.tricks[8] = {BoolSource::Nav2Center, TrickId::DanceLoop};
  return c;
}

ControllerBridge::ControllerBridge() {
  cfg_ = defaultBindings();
  config::defaultRcInputCalibration(calibration_);
  conditioner_.configure(calibration_);
  reset();
}

void ControllerBridge::setCalibration(
    const config::RcInputCalibration& calibration) {
  if (config::validateRcInputCalibration(calibration)) {
    calibration_ = calibration;
    conditioner_.configure(calibration_);
  }
}

bool ControllerBridge::setInputFilterMode(
    InputFilterMode mode, bool diagnostic_mode_allowed) {
  return conditioner_.setMode(mode, diagnostic_mode_allowed);
}

void ControllerBridge::setGaitTuneFractions(float step_height, float stride,
                                            float duty) {
  // An active edit session owns the values; a config revision landing mid-edit
  // must not yank the number the operator is watching on the handset.
  if (cmd_.gait_tune_active) return;
  gait_tune_frac_[0] = clampf(step_height, 0.0f, 1.0f);
  gait_tune_frac_[1] = clampf(stride, 0.0f, 1.0f);
  gait_tune_frac_[2] = clampf(duty, 0.0f, 1.0f);
}

void ControllerBridge::reset() {
  cmd_ = ControllerCommand();
  raw_ = ChannelPackInputs_t();
#if defined(HEXAPOD_FORCE_CUSTOM_CHANNELPACK)
  detected_profile_ = InputProfile::CustomControllerChannelPack;
  profile_locked_ = true;
#else
  detected_profile_ = InputProfile::Unknown;
  profile_locked_ = false;
#endif
  custom_layout_streak_ = 0;
  tx_direct_layout_streak_ = 0;
  arm_release_pending_ = false;
  arm_release_since_ms_ = 0;
  conditioner_.reset();
  for (uint8_t index = 0; index < 4; ++index) conditioned_gimbal_[index] = 0.0f;
  for (uint8_t index = 0; index < 2; ++index) {
    conditioned_pot_[index] = 0.0f;
    conditioned_encoder_[index] = 0.5f;
    conditioned_toggles_[index] = 1;
  }
  sw_e_debounce_.reset();
  sw_f_debounce_.reset();
  last_condition_ms_ = 0;
  condition_time_seen_ = false;
  for (uint8_t i = 0; i < 2; ++i) {
    enc_last_[i] = 0;
    enc_seen_[i] = false;
    enc_accum_[i] = i == 0 ? 1.0f : 0.5f;  // full stride, medium lift
  }
  // Compiled fallbacks for the NAV1-edited gait shape; rcTask overwrites these
  // with the persisted GaitDefaults through setGaitTuneFractions() as soon as
  // a config revision is adopted.
  gait_tune_frac_[0] = 0.6f;    // 30 mm of the 50 mm step-height range
  gait_tune_frac_[1] = 0.75f;   // 60 mm of the 80 mm stride range
  gait_tune_frac_[2] = 0.625f;  // duty 159/255
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
  // CH9: packed 2-pos switches B,C,D,G,H. SW_A shares guarded low/high bands
  // with Pot 1 on full-resolution CH5 so Wide-mode quantization cannot erase
  // the safety-critical arm request.
  const uint16_t switches = ChannelPack::crsfToDiscrete(
      ch[CPACK_CH_SWITCHES], CPACK_SWITCH_FIELD_MAX);
  const bool switches_ok = (switches & ~uint16_t(0x001F)) == 0;

  // CH10: 4 buttons (bits 0..3) + SwE tri (bits 4..5) + SwF tri (bits 6..7).
  // A tri field of 3 is invalid, and nothing above bit 7 may be set.
  const uint16_t bt = ChannelPack::crsfToDiscrete(
      ch[CPACK_CH_BTN_TOGGLE], CPACK_BTN_TOGGLE_FIELD_MAX);
  const bool btn_toggle_ok = (bt & ~uint16_t(0x00FF)) == 0 &&
                             (((bt >> 4) & 0x03) <= 2) &&
                             (((bt >> 6) & 0x03) <= 2);

  // CH11: packed nav (Nav1 bits 0..4, Nav2 bits 5..9); nothing above bit 9.
  const uint16_t nav = ChannelPack::crsfToDiscrete(
      ch[CPACK_CH_NAV], CPACK_NAV_FIELD_MAX);
  const bool nav_ok = (nav & ~uint16_t(0x03FF)) == 0;

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

  // CH9..CH10: SE/SD -> SwE gait select, SwF right-gimbal mode (3-position).
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
  // instead of integrating a wrap-delta.
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

  // The TX16S has no NAV cluster, so its sliders remain the stride / step
  // height control: feed them into the same gait-tune values the custom
  // controller edits, so telemetry and the save path behave identically. An
  // active editor (reachable over USB) still takes precedence.
  if (!cmd_.gait_tune_active) {
    gait_tune_frac_[1] = enc_accum_[0];  // LS -> stride
    gait_tune_frac_[0] = enc_accum_[1];  // RS -> step height
  }
}

float ControllerBridge::readAxisBipolar(const AxisBinding& b) const {
  float v = 0.0f;
  switch (b.source) {
    case AxisSource::GimbalLX:
    case AxisSource::GimbalLY:
    case AxisSource::GimbalRX:
    case AxisSource::GimbalRY:
      v = conditionedBipolar(b.source);
      break;
    case AxisSource::Pot1:
    case AxisSource::Pot2:
    case AxisSource::Enc1:
    case AxisSource::Enc2:
      v = conditionedUnipolar(b.source) * 2.0f - 1.0f;
      break;
    case AxisSource::None: return 0.0f;
  }
  v = applyBindingResponse(b, v);
  if (b.invert) v = -v;
  return clampf(v, -1.0f, 1.0f);
}

float ControllerBridge::readAxisUnipolar(const AxisBinding& b) const {
  float v = 0.0f;
  switch (b.source) {
    case AxisSource::GimbalLX:
    case AxisSource::GimbalLY:
    case AxisSource::GimbalRX:
    case AxisSource::GimbalRY:
      v = (applyBindingResponse(b, conditionedBipolar(b.source)) + 1.0f) *
          0.5f;
      break;
    case AxisSource::Pot1:
    case AxisSource::Pot2:
    case AxisSource::Enc1:
    case AxisSource::Enc2:
      v = conditionedUnipolar(b.source);
      break;
    case AxisSource::None: return 0.0f;
  }
  if (b.invert) v = 1.0f - v;
  return clampf(v, 0.0f, 1.0f);
}

void ControllerBridge::updateConditionedInputs(uint32_t now_ms) {
  const uint32_t dt_ms = condition_time_seen_ ? now_ms - last_condition_ms_ : 0;
  condition_time_seen_ = true;
  last_condition_ms_ = now_ms;

  float normalized = 0.0f;
  for (uint8_t index = 0; index < 4; ++index) {
    const AxisSource source = static_cast<AxisSource>(index + 1u);
    const bool valid = readCalibratedBipolar(source, raw_.gimbal[index],
                                             normalized);
    conditioned_gimbal_[index] = conditioner_.update(
        static_cast<uint8_t>(source), valid ? normalized : 0.0f, dt_ms);
  }
  for (uint8_t index = 0; index < 2; ++index) {
    const AxisSource source = static_cast<AxisSource>(index + 5u);
    const bool valid = readCalibratedUnipolar(source, raw_.pot[index],
                                              normalized);
    conditioned_pot_[index] = conditioner_.update(
        static_cast<uint8_t>(source), valid ? normalized : 0.0f, dt_ms);
  }
  for (uint8_t index = 0; index < 2; ++index) {
    const AxisSource source = static_cast<AxisSource>(index + 7u);
    conditioned_encoder_[index] = conditioner_.update(
        static_cast<uint8_t>(source), enc_accum_[index], dt_ms);
  }
}

void ControllerBridge::updateConditionedToggles(uint32_t now_ms) {
  conditioned_toggles_[0] =
      sw_e_debounce_.update(raw_.toggles[0], now_ms, kModeSwitchDebounceMs);
  conditioned_toggles_[1] =
      sw_f_debounce_.update(raw_.toggles[1], now_ms, kModeSwitchDebounceMs);
}

float ControllerBridge::conditionedBipolar(AxisSource source) const {
  const uint8_t index = static_cast<uint8_t>(source);
  if (index < 1 || index > 4) return 0.0f;
  return conditioned_gimbal_[index - 1u];
}

float ControllerBridge::conditionedUnipolar(AxisSource source) const {
  const uint8_t index = static_cast<uint8_t>(source);
  if (index >= 5 && index <= 6) return conditioned_pot_[index - 5u];
  if (index >= 7 && index <= 8) return conditioned_encoder_[index - 7u];
  return 0.0f;
}

float ControllerBridge::applyBindingResponse(const AxisBinding& binding,
                                              float value) const {
  const uint8_t source = static_cast<uint8_t>(binding.source);
  if (!conditioner_.isCentered(source)) return value;
  const float deadband = binding.deadband > 0.0f
                             ? binding.deadband
                             : conditioner_.deadband(source);
  value = RcInputConditioner::applyDeadband(value, deadband);
  return RcInputConditioner::applyExpo(value, conditioner_.expo(source));
}

const config::RcChannelCalibration* ControllerBridge::calibrationFor(
    AxisSource source) const {
  const uint8_t wanted = static_cast<uint8_t>(source);
  if (wanted == 0 || wanted > config::kNumRcAnalogInputs) return nullptr;
  for (uint8_t index = 0; index < config::kNumRcAnalogInputs; ++index) {
    const config::RcChannelCalibration& c = calibration_.channels[index];
    if (c.source == wanted) return &c;
  }
  return nullptr;
}

bool ControllerBridge::readCalibratedBipolar(AxisSource source, int16_t raw,
                                             float& out) const {
  const config::RcChannelCalibration* c = calibrationFor(source);
  if (c == nullptr ||
      c->type != static_cast<uint8_t>(config::RcChannelType::CenteredAnalog) ||
      !rawWithinCalibrationWindow(raw, *c)) {
    return false;
  }
  const int16_t clamped_raw = raw < c->min_raw ? c->min_raw
                           : raw > c->max_raw ? c->max_raw
                                              : raw;
  if (clamped_raw >= c->center_raw) {
    out = static_cast<float>(clamped_raw - c->center_raw) /
          static_cast<float>(c->max_raw - c->center_raw);
  } else {
    out = static_cast<float>(clamped_raw - c->center_raw) /
          static_cast<float>(c->center_raw - c->min_raw);
  }
  if (c->reversed != 0) out = -out;
  out = clampf(out, -1.0f, 1.0f);
  return true;
}

bool ControllerBridge::readCalibratedUnipolar(AxisSource source, int16_t raw,
                                              float& out) const {
  const config::RcChannelCalibration* c = calibrationFor(source);
  if (c == nullptr ||
      c->type != static_cast<uint8_t>(config::RcChannelType::UnipolarAnalog) ||
      !rawWithinCalibrationWindow(raw, *c)) {
    return false;
  }
  const int16_t clamped_raw = raw < c->min_raw ? c->min_raw
                           : raw > c->max_raw ? c->max_raw
                                              : raw;
  out = static_cast<float>(clamped_raw - c->min_raw) /
        static_cast<float>(c->max_raw - c->min_raw);
  if (c->reversed != 0) out = 1.0f - out;
  out = clampf(out, 0.0f, 1.0f);
  return true;
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
    case TriSource::SwE: return conditioned_toggles_[0];
    case TriSource::SwF: return conditioned_toggles_[1];
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

void ControllerBridge::updatePoseTrim(uint32_t now_ms) {
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
}

void ControllerBridge::updateGaitTune(uint32_t now_ms) {
  const bool tune_switch = readBool(cfg_.gait_tune_toggle);
  // SW_G is a maintained two-position switch: its physical level is the
  // editor state, so turning it off always leaves tuning. Other bindings keep
  // the historical press-to-toggle behavior for compatibility.
  const bool tune_state_changed =
      cfg_.gait_tune_toggle == BoolSource::SwG
          ? tune_switch != cmd_.gait_tune_active
          : risingEdge(kGaitTuneEdgeBase + 0, tune_switch, now_ms);
  if (tune_state_changed) {
    if (cfg_.gait_tune_toggle == BoolSource::SwG) {
      cmd_.gait_tune_active = tune_switch;
    } else {
      cmd_.gait_tune_active = !cmd_.gait_tune_active;
    }
    cmd_.gait_tune_param = GaitTuneParam::StepHeight;
    ++cmd_.gait_tune_edit_seq;
    cmd_.gait_tune_last_edit_ms = now_ms;
  }
  if (!cmd_.gait_tune_active) {
    // Keep the borrowed NAV1 edge slots coherent so re-entering the editor does
    // not replay a press that happened while it was disengaged.
    edge_prev_[kGaitTuneEdgeBase + 1] = readBool(cfg_.gait_tune_next);
    edge_prev_[kGaitTuneEdgeBase + 2] = readBool(cfg_.gait_tune_prev);
    edge_prev_[kGaitTuneEdgeBase + 3] = readBool(cfg_.gait_tune_increase);
    edge_prev_[kGaitTuneEdgeBase + 4] = readBool(cfg_.gait_tune_decrease);
    edge_prev_[kGaitTuneEdgeBase + 5] = readBool(cfg_.gait_tune_save);
    return;
  }

  // While the editor owns NAV1 the trim edge slots must also stay coherent.
  edge_prev_[kTrimEdgeBase + 0] = readBool(cfg_.trim_pitch_up);
  edge_prev_[kTrimEdgeBase + 1] = readBool(cfg_.trim_pitch_down);
  edge_prev_[kTrimEdgeBase + 2] = readBool(cfg_.trim_roll_left);
  edge_prev_[kTrimEdgeBase + 3] = readBool(cfg_.trim_roll_right);
  edge_prev_[kTrimEdgeBase + 4] = readBool(cfg_.trim_reset);

  uint8_t index = static_cast<uint8_t>(cmd_.gait_tune_param);
  bool edited = false;
  if (risingEdge(kGaitTuneEdgeBase + 1, readBool(cfg_.gait_tune_next),
                 now_ms)) {
    index = static_cast<uint8_t>((index + 1u) % kNumGaitTuneParams);
    edited = true;
  }
  if (risingEdge(kGaitTuneEdgeBase + 2, readBool(cfg_.gait_tune_prev),
                 now_ms)) {
    index = static_cast<uint8_t>((index + kNumGaitTuneParams - 1u) %
                                 kNumGaitTuneParams);
    edited = true;
  }
  cmd_.gait_tune_param = static_cast<GaitTuneParam>(index);

  if (risingEdge(kGaitTuneEdgeBase + 3, readBool(cfg_.gait_tune_increase),
                 now_ms)) {
    gait_tune_frac_[index] =
        clampf(gait_tune_frac_[index] + kGaitTuneStepFrac, 0.0f, 1.0f);
    edited = true;
  }
  if (risingEdge(kGaitTuneEdgeBase + 4, readBool(cfg_.gait_tune_decrease),
                 now_ms)) {
    gait_tune_frac_[index] =
        clampf(gait_tune_frac_[index] - kGaitTuneStepFrac, 0.0f, 1.0f);
    edited = true;
  }
  if (risingEdge(kGaitTuneEdgeBase + 5, readBool(cfg_.gait_tune_save),
                 now_ms)) {
    // Monotonic so the request survives a control-cycle boundary; the firmware
    // still decides whether it is safe to persist.
    ++cmd_.gait_tune_save_seq;
    edited = true;
  }
  if (edited) {
    ++cmd_.gait_tune_edit_seq;
    cmd_.gait_tune_last_edit_ms = now_ms;
  }
}

void ControllerBridge::enterFailsafe(uint32_t now_ms) {
  (void)now_ms;
  // Preserve ever_seen, trim, and the last selected mode/gait, but force a safe
  // hold: no motion, disarmed, kill asserted, no trick.
  cmd_.valid = false;
  cmd_.failsafe = true;
  cmd_.arm_request = false;
  arm_release_pending_ = false;
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

  updateConditionedInputs(now_ms);
  updateConditionedToggles(now_ms);

  cmd_.valid = true;
  cmd_.failsafe = false;
  cmd_.ever_seen = true;
  cmd_.frame_ms = now_ms;

  // Gait family (SW_E) and right-gimbal mode (SW_F). The gait selection is live
  // in every mode, so the operator can keep walking while the right gimbal
  // translates or rotates the body -- body motion needs no dedicated gait.
  const uint8_t gait_v = readTri(cfg_.gait_select);
  cmd_.gait_index = gait_v < 3 ? gait_v : 0;
  const uint8_t mode_v = readTri(cfg_.mode_select);
  cmd_.mode = static_cast<ControlMode>(mode_v < kNumControlModes ? mode_v : 0);

  // Safety levels.
  const bool kill = readBool(cfg_.estop);
  cmd_.estop = kill;  // not in failsafe here
  const bool arm_level = readBool(cfg_.arm);
  if (kill) {
    cmd_.arm_request = false;
    arm_release_pending_ = false;
  } else if (arm_level) {
    cmd_.arm_request = true;
    arm_release_pending_ = false;
  } else if (cmd_.arm_request) {
    if (!arm_release_pending_) {
      arm_release_pending_ = true;
      arm_release_since_ms_ = now_ms;
    } else if ((now_ms - arm_release_since_ms_) >= kArmReleaseDebounceMs) {
      cmd_.arm_request = false;
      arm_release_pending_ = false;
    }
  } else {
    arm_release_pending_ = false;
  }
  cmd_.host_authority = readBool(cfg_.host_authority);

  // Feature toggle request levels.
  cmd_.feat_foot_contact = readBool(cfg_.feat_foot_contact);
  cmd_.feat_terrain_leveling = readBool(cfg_.feat_terrain_leveling);
  cmd_.feat_passive_pose = readBool(cfg_.feat_passive_pose);

  // Shape params (read in every mode). Speed and body height stay on the pots.
  // Stride / step height / duty come from the NAV1 gait-tune editor unless a
  // host has explicitly bound an axis to them.
  cmd_.speed = readAxisUnipolar(cfg_.speed);
  cmd_.body_height = readAxisUnipolar(cfg_.body_height);
  updateGaitTune(now_ms);
  cmd_.step_height = cfg_.step_height.source != AxisSource::None
                         ? readAxisUnipolar(cfg_.step_height)
                         : gait_tune_frac_[0];
  cmd_.stride = cfg_.stride.source != AxisSource::None
                    ? readAxisUnipolar(cfg_.stride)
                    : gait_tune_frac_[1];
  cmd_.duty = cfg_.duty.source != AxisSource::None
                  ? readAxisUnipolar(cfg_.duty)
                  : gait_tune_frac_[2];

  // Motion: the left gimbal always walks in the plane (forward + strafe), so
  // the operator never loses locomotion while adjusting the body. Right X
  // steers in Yaw mode; the other modes reuse the right gimbal for body pose.
  // Unused pose axes stay zero and operator trim survives mode changes.
  cmd_.twist_vx = cmd_.twist_vy = cmd_.twist_wz = 0.0f;
  cmd_.pose_x_mm = cmd_.pose_y_mm = cmd_.pose_z_mm = 0.0f;
  cmd_.pose_roll = cmd_.pose_pitch = cmd_.pose_yaw = 0.0f;
  cmd_.twist_vx = readAxisBipolar(cfg_.walk_forward);
  cmd_.twist_vy = readAxisBipolar(cfg_.walk_strafe);
  switch (cmd_.mode) {
    case ControlMode::Yaw:
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

  // NAV1 is dual-purpose: pose trim normally, gait-parameter editing while the
  // gait-tune mode is engaged (updateGaitTune ran above and owns the mode).
  if (!cmd_.gait_tune_active) updatePoseTrim(now_ms);

  // BTN_4 is dedicated to capture. One distinct press bumps the sequence;
  // holding the button cannot retrigger until it has been released.
  if (risingEdge(kCaptureEdge, readBool(BoolSource::Btn4), now_ms)) {
    ++cmd_.capture_toggle_seq;
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
