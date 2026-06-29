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
    enterFailsafe(now_ms);
    return cmd_;
  }

  ChannelPack::unpackChannels(ch, &raw_);
  integrateEncoders();

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
