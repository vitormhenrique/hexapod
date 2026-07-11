#include "dxl_bus.h"

namespace dxl {

using namespace ControlTableItem;

namespace {
// Protocol versions to try during a ping, most likely first. MX-28AT ships on
// the legacy Protocol 1.0 table by default, so try 1.0 before 2.0.
constexpr float kProtoTryOrder[] = {1.0f, 2.0f};
}  // namespace

DxlBus::DxlBus(HardwareSerial& port) : dxl_(port, /*dir_pin=*/-1) {}

void DxlBus::begin(uint32_t baud) {
  baud_ = baud;
  dxl_.begin(baud);
  ready_ = true;
}

const ServoProfile* DxlBus::profileById(uint8_t id) const {
  for (uint8_t i = 0; i < count_; ++i) {
    if (servos_[i].id == id) {
      return &servos_[i];
    }
  }
  return nullptr;
}

bool DxlBus::readItem(uint8_t item_idx, uint8_t id, int32_t& value) {
  value = dxl_.readControlTableItem(item_idx, id, kReadTimeoutMs);
  const uint8_t err = static_cast<uint8_t>(dxl_.getLastLibErrCode());
  if (err != 0) {
    stats_.last_error = err;
    return false;
  }
  return true;
}

bool DxlBus::ping(uint8_t id, ServoProfile& out) {
  if (!ready_ || id > kMaxServoId) {
    return false;
  }

  for (float proto : kProtoTryOrder) {
    dxl_.setPortProtocolVersion(proto);
    if (!dxl_.ping(id)) {
      continue;
    }

    const uint16_t model = dxl_.getModelNumber(id);
    int32_t fw = 0;
    readItem(FIRMWARE_VERSION, id, fw);  // best-effort

    fillProfileFromModel(out, model, static_cast<uint8_t>(fw));
    out.id = id;
    out.present = true;
    // Record the protocol actually used to reach the servo, which can differ
    // from the table-implied default (e.g. a legacy MX flashed to Protocol 2).
    out.protocol_version = (proto >= 2.0f) ? 2 : 1;
    out.torque_enabled = dxl_.getTorqueEnableStat(id);
    out.last_error = static_cast<uint8_t>(dxl_.getLastLibErrCode());

    stats_.pings_ok++;
    return true;
  }

  stats_.pings_fail++;
  stats_.last_error = static_cast<uint8_t>(dxl_.getLastLibErrCode());
  return false;
}

uint8_t DxlBus::scan(uint8_t first_id, uint8_t last_id) {
  count_ = 0;
  stats_.scans++;
  if (!ready_) {
    return 0;
  }
  if (last_id > kMaxServoId) {
    last_id = kMaxServoId;
  }

  for (uint16_t id = first_id; id <= last_id; ++id) {
    ServoProfile p;
    if (ping(static_cast<uint8_t>(id), p) && count_ < kMaxServos) {
      servos_[count_++] = p;
    }
  }
  return count_;
}

void DxlBus::beginDiscovery() {
  count_ = 0;
  status_rr_ = 0;
  stats_.scans++;
}

bool DxlBus::discoverId(uint8_t id) {
  if (!ready_ || count_ >= kMaxServos || id > kMaxServoId) return false;
  if (profileById(id) != nullptr) return true;
  ServoProfile profile;
  if (!ping(id, profile)) return false;
  servos_[count_++] = profile;
  return true;
}

bool DxlBus::readStatus(uint8_t id, ServoStatus& out) {
  out = ServoStatus{};
  out.id = id;
  if (!ready_) {
    return false;
  }

  const ServoProfile* p = profileById(id);
  const bool is_mx2 = (p != nullptr) && (p->table_kind == TableKind::Mx28V2);
  const bool is_protocol1 = (p == nullptr) || (p->protocol_version == 1);
  // Use the servo's known protocol when available, else default to 1.0 (the
  // MX-28AT factory default table).
  dxl_.setPortProtocolVersion((p != nullptr && p->protocol_version == 2) ? 2.0f
                                                                         : 1.0f);

  int32_t v = 0;
  uint8_t protocol1_status_error = 0;
  const auto readStatusItem = [&](uint8_t item_idx, int32_t& value) {
    const bool ok = readItem(item_idx, id, value);
    if (ok && is_protocol1) {
      protocol1_status_error |= dxl_.getLastStatusPacketError();
    }
    return ok;
  };

  // Present position is the key field; its success defines a usable read.
  if (readStatusItem(PRESENT_POSITION, v)) {
    out.present_position = v;
    out.ok = true;
  } else {
    stats_.reads_fail++;
    return false;
  }

  // Velocity register name differs between the two tables.
  if (readStatusItem(is_mx2 ? PRESENT_VELOCITY : PRESENT_SPEED, v)) {
    out.present_velocity = v;
  }

  // Present load/PWM proxy. Both tables expose Present Load; values are
  // sign-magnitude on legacy and signed 0.1% on MX(2.0).
  if (readStatusItem(PRESENT_LOAD, v)) {
    out.present_load = v;
  }

  // Input voltage register name differs; both report in 0.1 V units.
  if (readStatusItem(is_mx2 ? PRESENT_INPUT_VOLTAGE : PRESENT_VOLTAGE, v)) {
    out.present_voltage_mv = static_cast<uint16_t>(v * 100);
  }

  if (readStatusItem(PRESENT_TEMPERATURE, v)) {
    out.present_temperature_c = static_cast<int8_t>(v);
  }

  // MX(2.0) exposes a register; Protocol 1 reports its seven alarm bits in
  // every status packet. Preserve either representation in the same wire byte.
  if (is_mx2 && readStatusItem(HARDWARE_ERROR_STATUS, v)) {
    out.hardware_error = static_cast<uint8_t>(v);
  } else if (is_protocol1) {
    out.hardware_error = protocol1_status_error;
  }

  out.torque_enabled = dxl_.getTorqueEnableStat(id);
  stats_.reads_ok++;
  return true;
}

void DxlBus::selectProtocol(TableKind kind) {
  dxl_.setPortProtocolVersion(kind == TableKind::Mx28V2 ? 2.0f : 1.0f);
}

uint8_t DxlBus::setTorqueAll(bool on) {
  if (!ready_) {
    return 0;
  }
  uint8_t acked = 0;
  for (uint8_t i = 0; i < count_; ++i) {
    ServoProfile& p = servos_[i];
    selectProtocol(p.table_kind);
    // Dynamixel2Arduino's torqueOn/torqueOff helpers use a 100 ms default
    // timeout. Across 18 servos, enable plus failure rollback can exceed the
    // 2 s hardware watchdog before dxlTask reaches its scheduler yield.
    const bool ok = dxl_.writeControlTableItem(
        TORQUE_ENABLE, p.id, on ? 1 : 0, kWriteTimeoutMs);
    if (ok) {
      ++acked;
      p.torque_enabled = on;  // track commanded torque for allTorqueOff (lmt.6)
    } else {
      stats_.last_error = static_cast<uint8_t>(dxl_.getLastLibErrCode());
    }
  }
  return acked;
}

bool DxlBus::writeGoalPositions(const GoalTarget* targets, uint8_t count) {
  if (!ready_ || targets == nullptr || count == 0) {
    return false;
  }

  bool all_ok = true;
  // One Sync Write per control table; the library node limit (DXL_MAX_NODE)
  // caps a single instruction, so large groups are chunked.
  const TableKind kinds[] = {TableKind::Mx28Legacy, TableKind::Mx28V2};
  for (TableKind kind : kinds) {
    uint8_t i = 0;
    while (i < count) {
      sw_param_.addr = goalAddr(kind);
      sw_param_.length = goalLen(kind);
      sw_param_.id_count = 0;
      for (; i < count && sw_param_.id_count < DXL_MAX_NODE; ++i) {
        const ServoProfile* p = profileById(targets[i].id);
        if (p == nullptr || p->table_kind != kind) {
          continue;
        }
        XelInfoForSyncWriteParam_t& xel = sw_param_.xel[sw_param_.id_count];
        xel.id = targets[i].id;
        encodeGoal(kind, targets[i].tick, xel.data);
        ++sw_param_.id_count;
      }
      if (sw_param_.id_count == 0) {
        continue;  // nothing of this kind in the remaining targets
      }
      selectProtocol(kind);
      if (dxl_.syncWrite(sw_param_)) {
        stats_.writes_ok++;
      } else {
        stats_.writes_fail++;
        stats_.last_error = static_cast<uint8_t>(dxl_.getLastLibErrCode());
        all_ok = false;
      }
    }
  }
  return all_ok;
}

uint8_t DxlBus::syncReadStatus(ServoStatus* out, uint8_t out_cap) {
  if (!ready_ || out == nullptr || count_ == 0 || out_cap == 0) {
    return 0;
  }
  // Keep identity fresh for every entry, but PRESERVE the detail fields
  // (velocity/load/voltage/temperature/torque/hardware_error) populated by the
  // dxlTask round-robin readStatus (eax.6). `ok` is only rewritten for entries
  // actually attempted this call, so partial (round-robin) refreshes do not
  // wipe the validity of untouched entries.
  for (uint8_t i = 0; i < count_ && i < out_cap; ++i) {
    out[i].id = servos_[i].id;
  }

  const uint8_t readable_count = (count_ < out_cap) ? count_ : out_cap;
  if (status_rr_ >= readable_count) status_rr_ = 0;
  const uint8_t index = status_rr_++;
  const ServoProfile& profile = servos_[index];
  selectProtocol(profile.table_kind);

  uint8_t buf[4] = {0, 0, 0, 0};
  const uint8_t len = posLen(profile.table_kind);
  const int32_t received =
      dxl_.read(profile.id, posAddr(profile.table_kind), len, buf, sizeof(buf),
                kReadTimeoutMs);
  if (received >= static_cast<int32_t>(len)) {
    out[index].present_position = decodePosition(profile.table_kind, buf);
    out[index].ok = true;
    stats_.reads_ok++;
    return 1;
  }

  out[index].ok = false;
  stats_.reads_fail++;
  stats_.last_error = static_cast<uint8_t>(dxl_.getLastLibErrCode());
  return 0;
}

bool DxlBus::readRegister(uint8_t id, TableKind table, uint16_t addr,
                          uint8_t len, bool is_signed, int32_t& out) {
  if (!ready_ || (len != 1 && len != 2 && len != 4)) {
    return false;
  }
  selectProtocol(table);
  uint8_t buf[4] = {0, 0, 0, 0};
  const int32_t n = dxl_.read(id, addr, len, buf, sizeof(buf));
  if (n < static_cast<int32_t>(len)) {
    stats_.reads_fail++;
    stats_.last_error = static_cast<uint8_t>(dxl_.getLastLibErrCode());
    return false;
  }
  uint32_t raw = 0;
  for (uint8_t i = 0; i < len; ++i) {
    raw |= static_cast<uint32_t>(buf[i]) << (8 * i);
  }
  if (is_signed && len < 4) {
    // Sign-extend from the top bit of the value's width.
    const uint32_t sign_bit = 1u << (8 * len - 1);
    if (raw & sign_bit) {
      raw |= ~((1u << (8 * len)) - 1);
    }
  }
  out = static_cast<int32_t>(raw);
  stats_.reads_ok++;
  return true;
}

bool DxlBus::writeRegister(uint8_t id, TableKind table, uint16_t addr,
                           uint8_t len, int32_t value) {
  if (!ready_ || (len != 1 && len != 2 && len != 4)) {
    return false;
  }
  selectProtocol(table);
  const uint32_t raw = static_cast<uint32_t>(value);
  uint8_t buf[4];
  for (uint8_t i = 0; i < len; ++i) {
    buf[i] = static_cast<uint8_t>((raw >> (8 * i)) & 0xFF);
  }
  if (!dxl_.write(id, addr, buf, len)) {
    stats_.writes_fail++;
    stats_.last_error = static_cast<uint8_t>(dxl_.getLastLibErrCode());
    return false;
  }
  stats_.writes_ok++;
  return true;
}

bool DxlBus::setTorqueOne(uint8_t id, TableKind table, bool on) {
  if (!ready_) {
    return false;
  }
  selectProtocol(table);
  const bool ok = dxl_.writeControlTableItem(
      TORQUE_ENABLE, id, on ? 1 : 0, kWriteTimeoutMs);
  if (!ok) {
    stats_.last_error = static_cast<uint8_t>(dxl_.getLastLibErrCode());
    return false;
  }
  for (uint8_t i = 0; i < count_; ++i) {  // track commanded torque (lmt.6)
    if (servos_[i].id == id) {
      servos_[i].torque_enabled = on;
      break;
    }
  }
  return true;
}

bool DxlBus::allTorqueOff() const {
  for (uint8_t i = 0; i < count_; ++i) {
    if (servos_[i].torque_enabled) {
      return false;
    }
  }
  return true;
}

bool DxlBus::torqueState(uint8_t id, TableKind table, bool& on) {
  if (!ready_) {
    return false;
  }
  selectProtocol(table);
  on = dxl_.getTorqueEnableStat(id);
  const uint8_t err = static_cast<uint8_t>(dxl_.getLastLibErrCode());
  if (err != 0) {
    stats_.last_error = err;
    return false;
  }
  return true;
}

}  // namespace dxl