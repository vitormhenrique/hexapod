#pragma once

#include <stdint.h>

#include "bno085_protocol.h"
#include "i2c_bus.h"

namespace sensors {

struct ImuSample {
  int16_t pitch_cdeg = 0;
  int16_t roll_cdeg = 0;
  int16_t yaw_cdeg = 0;
  uint8_t calibration = 0;
  bool ok = false;
};

// Optional root-bus BNO085 adapter. Only i2cTask may call this class.
class Bno085 {
 public:
  // Outcome of the last begin() for diagnostics (I2C_GET_TOPOLOGY tail).
  enum class State : uint8_t {
    NotFound = 0,   // neither 0x4A nor 0x4B acknowledged
    NoResponse = 1,  // address ACKed but the SHTP reset/identity exchange failed
    Ready = 2,       // identified and rotation-vector report enabled
  };

  explicit Bno085(i2c::I2cBus& bus) : bus_(bus) {}

  // Probe 0x4A/0x4B, perform a bounded SHTP reset/identity exchange, and
  // enable a 20 Hz rotation-vector report. Boot delays occur only in i2cTask.
  bool begin();
  // Cheap address-only probe (no reset); true if a BNO085 address ACKs.
  bool probe();
  ImuSample read();

  bool present() const { return present_; }
  // Address that acknowledged during the last begin()/probe(), 0 if none.
  uint8_t address() const { return address_; }
  State state() const { return state_; }

 private:
  bool sendPacket(uint8_t channel, const uint8_t* payload,
                  uint8_t payload_len);
  bool receivePacket(uint8_t& channel, uint8_t* payload,
                     uint16_t payload_capacity, uint16_t& payload_len);
  bool enableRotationVector();

  i2c::I2cBus& bus_;
  uint8_t address_ = 0;
  uint8_t sequence_[6] = {};
  bool present_ = false;
  State state_ = State::NotFound;
};

}  // namespace sensors