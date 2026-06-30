#include "controller_api.h"

namespace protocol {
namespace {

// --- little-endian writers -------------------------------------------------
uint16_t putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  return 2;
}
uint16_t putI16(uint8_t* p, int16_t v) {
  return putU16(p, static_cast<uint16_t>(v));
}
uint16_t putI32(uint8_t* p, int32_t v) {
  const uint32_t u = static_cast<uint32_t>(v);
  p[0] = static_cast<uint8_t>(u & 0xFF);
  p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
  return 4;
}

// --- little-endian readers -------------------------------------------------
uint16_t readU16(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}
int16_t readI16(const uint8_t* p) { return static_cast<int16_t>(readU16(p)); }

// --- float -> fixed scalers (mirrored by protocol/python) ------------------
// Round-to-nearest with saturation to int16.
int16_t toI16(float v) {
  const float r = (v >= 0.0f) ? (v + 0.5f) : (v - 0.5f);
  if (r > 32767.0f) return 32767;
  if (r < -32768.0f) return -32768;
  return static_cast<int16_t>(r);
}
int16_t milli(float v) { return toI16(v * 1000.0f); }  // [-x..x] -> milli-units
uint8_t scalarU8(float v) {                            // [0..1] -> 0..255
  float r = v * 255.0f + 0.5f;
  if (r < 0.0f) r = 0.0f;
  if (r > 255.0f) r = 255.0f;
  return static_cast<uint8_t>(r);
}

// --- BindingConfig field codecs --------------------------------------------
uint16_t putAxis(uint8_t* p, const controller::AxisBinding& a) {
  uint16_t o = 0;
  p[o++] = static_cast<uint8_t>(a.source);
  p[o++] = a.invert ? 1 : 0;
  o += putI16(&p[o], milli(a.deadband));
  return o;  // 4 bytes
}

bool getAxis(const uint8_t* p, controller::AxisBinding* a) {
  const uint8_t src = p[0];
  if (src > static_cast<uint8_t>(controller::AxisSource::Enc2)) return false;
  a->source = static_cast<controller::AxisSource>(src);
  a->invert = (p[1] != 0);
  float db = static_cast<float>(readI16(&p[2])) / 1000.0f;
  if (db < 0.0f) db = 0.0f;
  if (db > 1.0f) db = 1.0f;
  a->deadband = db;
  return true;
}

bool validBool(uint8_t v) {
  return v <= static_cast<uint8_t>(controller::BoolSource::Nav2Center);
}
bool validTri(uint8_t v) {
  return v <= static_cast<uint8_t>(controller::TriSource::SwF);
}
bool validTrick(uint8_t v) {
  return v <= static_cast<uint8_t>(controller::TrickId::DanceLoop);
}

}  // namespace

ControllerApi::ControllerApi() { reset(); }

void ControllerApi::reset() {
  cmd_ = controller::ControllerCommand{};
  raw_ = ChannelPackInputs_t{};
  cfg_ = controller::defaultBindings();
  pending_ = controller::BindingConfig{};
  pending_valid_ = false;
}

void ControllerApi::setSnapshot(const controller::ControllerCommand& cmd,
                                const ChannelPackInputs_t& raw) {
  cmd_ = cmd;
  raw_ = raw;
}

void ControllerApi::setBindings(const controller::BindingConfig& cfg) {
  cfg_ = cfg;
}

bool ControllerApi::takePending(controller::BindingConfig* out) {
  if (!pending_valid_) return false;
  if (out != nullptr) *out = pending_;
  pending_valid_ = false;
  return true;
}

uint16_t ControllerApi::encodeState(const controller::ControllerCommand& cmd,
                                    const ChannelPackInputs_t& raw,
                                    uint8_t* out) {
  uint16_t o = 0;
  // --- decoded command (31 bytes) ---
  uint8_t f1 = 0;
  if (cmd.valid) f1 |= 0x01;
  if (cmd.failsafe) f1 |= 0x02;
  if (cmd.ever_seen) f1 |= 0x04;
  if (cmd.arm_request) f1 |= 0x08;
  if (cmd.estop) f1 |= 0x10;
  if (cmd.host_authority) f1 |= 0x20;
  if (cmd.feat_foot_contact) f1 |= 0x40;
  if (cmd.feat_terrain_leveling) f1 |= 0x80;
  out[o++] = f1;
  uint8_t f2 = 0;
  if (cmd.feat_passive_pose) f2 |= 0x01;
  out[o++] = f2;
  out[o++] = static_cast<uint8_t>(cmd.mode);
  out[o++] = cmd.gait_index;
  out[o++] = static_cast<uint8_t>(cmd.trick);
  o += putI16(&out[o], milli(cmd.twist_vx));
  o += putI16(&out[o], milli(cmd.twist_vy));
  o += putI16(&out[o], milli(cmd.twist_wz));
  o += putI16(&out[o], toI16(cmd.pose_x_mm));
  o += putI16(&out[o], toI16(cmd.pose_y_mm));
  o += putI16(&out[o], toI16(cmd.pose_z_mm));
  o += putI16(&out[o], milli(cmd.pose_roll));
  o += putI16(&out[o], milli(cmd.pose_pitch));
  o += putI16(&out[o], milli(cmd.pose_yaw));
  o += putI16(&out[o], milli(cmd.trim_roll));
  o += putI16(&out[o], milli(cmd.trim_pitch));
  out[o++] = scalarU8(cmd.speed);
  out[o++] = scalarU8(cmd.body_height);
  out[o++] = scalarU8(cmd.stride);
  out[o++] = scalarU8(cmd.step_height);
  // --- raw inputs (26 bytes) ---
  for (int i = 0; i < 4; ++i) o += putI16(&out[o], raw.gimbal[i]);
  for (int i = 0; i < 2; ++i) o += putI16(&out[o], raw.pot[i]);
  for (int i = 0; i < 2; ++i) o += putI32(&out[o], raw.encoder[i]);
  uint8_t sw = 0;
  for (int i = 0; i < 8; ++i)
    if (raw.switches[i]) sw |= static_cast<uint8_t>(1u << i);
  out[o++] = sw;
  uint8_t btn = 0;
  for (int i = 0; i < 4; ++i)
    if (raw.buttons[i]) btn |= static_cast<uint8_t>(1u << i);
  out[o++] = btn;
  out[o++] = raw.toggles[0];
  out[o++] = raw.toggles[1];
  for (int n = 0; n < 2; ++n) {
    uint8_t navb = 0;
    for (int i = 0; i < 5; ++i)
      if (raw.nav[n][i]) navb |= static_cast<uint8_t>(1u << i);
    out[o++] = navb;
  }
  return o;  // kControllerStateLen
}

uint16_t ControllerApi::encodeBindings(const controller::BindingConfig& cfg,
                                       uint8_t* out) {
  uint16_t o = 0;
  o += putAxis(&out[o], cfg.walk_forward);
  o += putAxis(&out[o], cfg.walk_strafe);
  o += putAxis(&out[o], cfg.walk_yaw);
  o += putAxis(&out[o], cfg.body_x);
  o += putAxis(&out[o], cfg.body_y);
  o += putAxis(&out[o], cfg.body_z);
  o += putAxis(&out[o], cfg.body_roll);
  o += putAxis(&out[o], cfg.body_pitch);
  o += putAxis(&out[o], cfg.body_yaw);
  o += putAxis(&out[o], cfg.speed);
  o += putAxis(&out[o], cfg.body_height);
  o += putAxis(&out[o], cfg.stride);
  o += putAxis(&out[o], cfg.step_height);
  out[o++] = static_cast<uint8_t>(cfg.mode_select);
  out[o++] = static_cast<uint8_t>(cfg.gait_select);
  out[o++] = static_cast<uint8_t>(cfg.arm);
  out[o++] = static_cast<uint8_t>(cfg.estop);
  out[o++] = static_cast<uint8_t>(cfg.feat_foot_contact);
  out[o++] = static_cast<uint8_t>(cfg.feat_terrain_leveling);
  out[o++] = static_cast<uint8_t>(cfg.feat_passive_pose);
  out[o++] = static_cast<uint8_t>(cfg.host_authority);
  out[o++] = static_cast<uint8_t>(cfg.trim_pitch_up);
  out[o++] = static_cast<uint8_t>(cfg.trim_pitch_down);
  out[o++] = static_cast<uint8_t>(cfg.trim_roll_left);
  out[o++] = static_cast<uint8_t>(cfg.trim_roll_right);
  out[o++] = static_cast<uint8_t>(cfg.trim_reset);
  for (uint8_t i = 0; i < controller::kMaxTrickBindings; ++i) {
    out[o++] = static_cast<uint8_t>(cfg.tricks[i].source);
    out[o++] = static_cast<uint8_t>(cfg.tricks[i].trick);
  }
  return o;  // kControllerBindingsLen
}

bool ControllerApi::decodeBindings(const uint8_t* in, uint16_t len,
                                   controller::BindingConfig* out) {
  if (len != kControllerBindingsLen || out == nullptr) return false;
  controller::BindingConfig c;
  uint16_t o = 0;
  controller::AxisBinding* axes[13] = {
      &c.walk_forward, &c.walk_strafe, &c.walk_yaw,    &c.body_x,
      &c.body_y,       &c.body_z,      &c.body_roll,   &c.body_pitch,
      &c.body_yaw,     &c.speed,       &c.body_height, &c.stride,
      &c.step_height};
  for (uint8_t i = 0; i < 13; ++i) {
    if (!getAxis(&in[o], axes[i])) return false;
    o += 4;
  }
  if (!validTri(in[o]) || !validTri(in[o + 1])) return false;
  c.mode_select = static_cast<controller::TriSource>(in[o++]);
  c.gait_select = static_cast<controller::TriSource>(in[o++]);
  // 6 feature/safety + 5 trim bool sources.
  controller::BoolSource* bools[11] = {
      &c.arm,
      &c.estop,
      &c.feat_foot_contact,
      &c.feat_terrain_leveling,
      &c.feat_passive_pose,
      &c.host_authority,
      &c.trim_pitch_up,
      &c.trim_pitch_down,
      &c.trim_roll_left,
      &c.trim_roll_right,
      &c.trim_reset};
  for (uint8_t i = 0; i < 11; ++i) {
    if (!validBool(in[o])) return false;
    *bools[i] = static_cast<controller::BoolSource>(in[o++]);
  }
  for (uint8_t i = 0; i < controller::kMaxTrickBindings; ++i) {
    if (!validBool(in[o]) || !validTrick(in[o + 1])) return false;
    c.tricks[i].source = static_cast<controller::BoolSource>(in[o++]);
    c.tricks[i].trick = static_cast<controller::TrickId>(in[o++]);
  }
  *out = c;
  return true;
}

bool ControllerApi::writeResult(ControllerResult r, uint8_t* out,
                                uint16_t out_cap, uint16_t* out_len,
                                uint8_t* out_flags) const {
  if (out_cap < 1) {
    *out_len = 0;
    *out_flags = 0x02;  // flag::kError
    return true;
  }
  out[0] = static_cast<uint8_t>(r);
  *out_len = 1;
  *out_flags = (r == ControllerResult::BadRequest) ? 0x02 : 0x00;
  return true;
}

bool ControllerApi::handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
                           uint8_t* out, uint16_t out_cap, uint16_t* out_len,
                           uint8_t* out_flags) {
  if (!controllermsg::isControllerMsg(msg_id)) return false;

  switch (msg_id) {
    case controllermsg::kGetState: {
      if (out_cap < kControllerStateLen) {
        return writeResult(ControllerResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      *out_len = encodeState(cmd_, raw_, out);
      *out_flags = 0;
      return true;
    }
    case controllermsg::kGetBindings: {
      if (out_cap < kControllerBindingsLen) {
        return writeResult(ControllerResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      *out_len = encodeBindings(cfg_, out);
      *out_flags = 0;
      return true;
    }
    case controllermsg::kSetBindings: {
      controller::BindingConfig c;
      if (!decodeBindings(req, req_len, &c)) {
        return writeResult(ControllerResult::BadRequest, out, out_cap, out_len,
                           out_flags);
      }
      // Stage for the bridge owner (rcTask) to adopt; reflect it immediately in
      // the api's reported config so a follow-up GET_BINDINGS round-trips.
      pending_ = c;
      pending_valid_ = true;
      cfg_ = c;
      return writeResult(ControllerResult::Ok, out, out_cap, out_len,
                         out_flags);
    }
  }
  return writeResult(ControllerResult::BadRequest, out, out_cap, out_len,
                     out_flags);
}

}  // namespace protocol
