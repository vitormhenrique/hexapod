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
  value = dxl_.readControlTableItem(item_idx, id);
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

bool DxlBus::readStatus(uint8_t id, ServoStatus& out) {
  out = ServoStatus{};
  out.id = id;
  if (!ready_) {
    return false;
  }

  const ServoProfile* p = profileById(id);
  const bool is_mx2 = (p != nullptr) && (p->table_kind == TableKind::Mx28V2);
  // Use the servo's known protocol when available, else default to 1.0 (the
  // MX-28AT factory default table).
  dxl_.setPortProtocolVersion((p != nullptr && p->protocol_version == 2) ? 2.0f
                                                                         : 1.0f);

  int32_t v = 0;

  // Present position is the key field; its success defines a usable read.
  if (readItem(PRESENT_POSITION, id, v)) {
    out.present_position = v;
    out.ok = true;
  } else {
    stats_.reads_fail++;
    return false;
  }

  // Velocity register name differs between the two tables.
  if (readItem(is_mx2 ? PRESENT_VELOCITY : PRESENT_SPEED, id, v)) {
    out.present_velocity = v;
  }

  // Input voltage register name differs; both report in 0.1 V units.
  if (readItem(is_mx2 ? PRESENT_INPUT_VOLTAGE : PRESENT_VOLTAGE, id, v)) {
    out.present_voltage_mv = static_cast<uint16_t>(v * 100);
  }

  if (readItem(PRESENT_TEMPERATURE, id, v)) {
    out.present_temperature_c = static_cast<int8_t>(v);
  }

  // Hardware Error Status only exists on the MX(2.0) table.
  if (is_mx2 && readItem(HARDWARE_ERROR_STATUS, id, v)) {
    out.hardware_error = static_cast<uint8_t>(v);
  }

  out.torque_enabled = dxl_.getTorqueEnableStat(id);
  stats_.reads_ok++;
  return true;
}

}  // namespace dxl
