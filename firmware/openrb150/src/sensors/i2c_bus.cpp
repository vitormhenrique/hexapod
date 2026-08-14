#include "i2c_bus.h"

#if defined(ARDUINO_ARCH_SAMD)
#include <wiring_private.h>
#endif

namespace i2c {
namespace {

constexpr uint8_t kErrorAddressNack = 2;
constexpr uint8_t kErrorDataNack = 3;
constexpr uint8_t kErrorBus = 4;
constexpr uint8_t kErrorTimeout = 5;

}  // namespace

#if defined(ARDUINO_ARCH_SAMD)

bool I2cBus::waitForSync(uint32_t mask) {
  const uint32_t started = micros();
  while ((SERCOM0->I2CM.SYNCBUSY.reg & mask) != 0) {
    if (static_cast<uint32_t>(micros() - started) >= transaction_timeout_us_) {
      ++stats_.timeouts;
      stats_.last_error = kErrorTimeout;
      return false;
    }
  }
  return true;
}

bool I2cBus::waitForFlag(uint8_t flags) {
  const uint32_t started = micros();
  while ((SERCOM0->I2CM.INTFLAG.reg & flags) == 0) {
    if (SERCOM0->I2CM.STATUS.bit.BUSERR ||
        SERCOM0->I2CM.STATUS.bit.ARBLOST) {
      stats_.last_error = kErrorBus;
      return false;
    }
    if (static_cast<uint32_t>(micros() - started) >= transaction_timeout_us_) {
      ++stats_.timeouts;
      stats_.last_error = kErrorTimeout;
      return false;
    }
  }
  return true;
}

bool I2cBus::initializeHardware() {
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM0_CORE) |
                      GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  const uint32_t clock_started = micros();
  while (GCLK->STATUS.bit.SYNCBUSY) {
    if (static_cast<uint32_t>(micros() - clock_started) >= transaction_timeout_us_) {
      ++stats_.timeouts;
      stats_.last_error = kErrorTimeout;
      return false;
    }
  }

  NVIC_DisableIRQ(SERCOM0_IRQn);
  SERCOM0->I2CM.CTRLA.bit.SWRST = 1;
  if (!waitForSync(SERCOM_I2CM_SYNCBUSY_SWRST) ||
      SERCOM0->I2CM.CTRLA.bit.SWRST) {
    return false;
  }
  SERCOM0->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE(I2C_MASTER_OPERATION);
  uint32_t baud = SystemCoreClock / (2u * clock_hz_);
  baud = baud > 8u ? baud - 8u : 0u;
  if (baud > 255u) baud = 255u;
  SERCOM0->I2CM.BAUD.bit.BAUD = static_cast<uint8_t>(baud);
  SERCOM0->I2CM.CTRLA.bit.ENABLE = 1;
  if (!waitForSync(SERCOM_I2CM_SYNCBUSY_ENABLE)) return false;
  SERCOM0->I2CM.STATUS.bit.BUSSTATE = WIRE_IDLE_STATE;
  if (!waitForSync(SERCOM_I2CM_SYNCBUSY_SYSOP)) return false;

  pinPeripheral(PIN_WIRE_SDA, PIO_SERCOM);
  pinPeripheral(PIN_WIRE_SCL, PIO_SERCOM);
  ready_ = true;
  stats_.last_error = 0;
  return true;
}

bool I2cBus::begin(uint32_t clock_hz) {
  clock_hz_ = clock_hz == 0 ? 100000 : clock_hz;
  ready_ = false;
  return initializeHardware();
}

// The SAMD register driver resynchronizes the SERCOM master on every
// transaction, so no pre-cleanup is needed before direct-Wire libraries.
void I2cBus::prepareForDirectWire() {}

bool I2cBus::sendStop() {
  SERCOM0->I2CM.CTRLB.bit.CMD = WIRE_MASTER_ACT_STOP;
  return waitForSync(SERCOM_I2CM_SYNCBUSY_SYSOP);
}

bool I2cBus::recover() {
  ready_ = false;
  SERCOM0->I2CM.CTRLA.bit.ENABLE = 0;
  (void)waitForSync(SERCOM_I2CM_SYNCBUSY_ENABLE);

  pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
  pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
  if (digitalRead(PIN_WIRE_SDA) == LOW) {
    for (uint8_t pulse = 0; pulse < 9; ++pulse) {
      pinMode(PIN_WIRE_SCL, OUTPUT);
      digitalWrite(PIN_WIRE_SCL, LOW);
      delayMicroseconds(5);
      pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
      delayMicroseconds(5);
    }
  }
  pinMode(PIN_WIRE_SDA, OUTPUT);
  digitalWrite(PIN_WIRE_SDA, LOW);
  delayMicroseconds(5);
  pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
  delayMicroseconds(5);

  ++stats_.recoveries;
  return initializeHardware();
}

bool I2cBus::start(uint8_t addr, bool read_operation) {
  if (!ready_ && !recover()) return false;
  const uint8_t state = SERCOM0->I2CM.STATUS.bit.BUSSTATE;
  if (state == WIRE_BUSY_STATE) return fail(kErrorBus, true);

  SERCOM0->I2CM.STATUS.reg = SERCOM_I2CM_STATUS_BUSERR |
                            SERCOM_I2CM_STATUS_ARBLOST;
  SERCOM0->I2CM.ADDR.reg =
      static_cast<uint8_t>((addr << 1) | (read_operation ? 1u : 0u));
  const uint8_t expected = read_operation
                               ? SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_MB
                               : SERCOM_I2CM_INTFLAG_MB;
  if (!waitForFlag(expected)) return fail(stats_.last_error, true);
  if (read_operation && !SERCOM0->I2CM.INTFLAG.bit.SB) {
    if (!sendStop()) return fail(kErrorTimeout, true);
    return fail(kErrorAddressNack, false);
  }
  if (SERCOM0->I2CM.STATUS.bit.RXNACK) {
    if (!sendStop()) return fail(kErrorTimeout, true);
    return fail(kErrorAddressNack, false);
  }
  return true;
}

bool I2cBus::write(uint8_t addr, const uint8_t* data, uint8_t len,
                   bool send_stop) {
  if (len > 0 && data == nullptr) return false;
  if (!start(addr, false)) return false;
  for (uint8_t index = 0; index < len; ++index) {
    SERCOM0->I2CM.DATA.reg = data[index];
    if (!waitForFlag(SERCOM_I2CM_INTFLAG_MB)) {
      return fail(stats_.last_error, true);
    }
    if (SERCOM0->I2CM.STATUS.bit.RXNACK) {
      if (!sendStop()) return fail(kErrorTimeout, true);
      return fail(kErrorDataNack, false);
    }
  }
  return !send_stop || sendStop() || fail(kErrorTimeout, true);
}

bool I2cBus::read(uint8_t addr, uint8_t* data, uint8_t len, bool send_stop) {
  if (data == nullptr || len == 0) return false;
  if (!start(addr, true)) return false;
  for (uint8_t index = 0; index < len; ++index) {
    data[index] = static_cast<uint8_t>(SERCOM0->I2CM.DATA.reg);
    if (index + 1u < len) {
      SERCOM0->I2CM.CTRLB.bit.ACKACT = 0;
      SERCOM0->I2CM.CTRLB.bit.CMD = WIRE_MASTER_ACT_READ;
      if (!waitForSync(SERCOM_I2CM_SYNCBUSY_SYSOP) ||
          !waitForFlag(SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_MB)) {
        return fail(stats_.last_error, true);
      }
    }
  }
  SERCOM0->I2CM.CTRLB.bit.ACKACT = 1;
  return !send_stop || sendStop() || fail(kErrorTimeout, true);
}

#else  // !ARDUINO_ARCH_SAMD: generic Arduino Wire implementation (Teensy 4.x)

// The Teensy 4.x Wire driver performs bounded synchronous transfers, so the
// register-level SERCOM timeout machinery above is unnecessary. The same
// public contract holds: no call blocks past the driver's internal bound, and
// errors surface through stats_/return values instead of hanging the task.

namespace {

// The Teensy core's endTransmission() NACK path (WireIMXRT.cpp) clears the
// FIFOs, queues a STOP command, and returns WITHOUT clearing MSR[NDF]. With
// NDF latched the LPI2C master freezes command processing, so that STOP stays
// stuck in the TX FIFO (observed MFSR=1 after every NACK probe). Under
// FreeRTOS the next transaction then trips MSR[FEF] (command sequence error)
// and aborts, which made every device probe after a NACK fail even though the
// bus and devices were healthy. Flush stale commands and latched error flags
// while the master is idle before starting a new transaction. Wire == LPI2C1
// on the MicroMod Teensy; this class is the sole owner of that bus.
void cleanupStaleMaster() {
  if ((IMXRT_LPI2C1.MSR & LPI2C_MSR_MBF) != 0) return;  // mid-transaction
  if ((IMXRT_LPI2C1.MFSR & 0x07) != 0 ||
      (IMXRT_LPI2C1.MSR & (LPI2C_MSR_NDF | LPI2C_MSR_FEF | LPI2C_MSR_ALF |
                           LPI2C_MSR_PLTF)) != 0) {
    IMXRT_LPI2C1.MCR |= LPI2C_MCR_RTF | LPI2C_MCR_RRF;  // flush TX/RX FIFOs
    IMXRT_LPI2C1.MSR = 0x00007F00;                      // clear latched flags
  }
}

}  // namespace

bool I2cBus::initializeHardware() {
  Wire.begin();
  Wire.setClock(clock_hz_);
  ready_ = true;
  stats_.last_error = 0;
  return true;
}

bool I2cBus::begin(uint32_t clock_hz) {
  clock_hz_ = clock_hz == 0 ? 100000 : clock_hz;
  ready_ = false;
  return initializeHardware();
}

void I2cBus::prepareForDirectWire() { cleanupStaleMaster(); }

bool I2cBus::recover() {
  ready_ = false;
  Wire.end();
  ++stats_.recoveries;
  return initializeHardware();
}

bool I2cBus::write(uint8_t addr, const uint8_t* data, uint8_t len,
                   bool send_stop) {
  if (len > 0 && data == nullptr) return false;
  if (!ready_ && !recover()) return false;
  cleanupStaleMaster();
  Wire.beginTransmission(addr);
  if (len > 0) Wire.write(data, len);
  const uint8_t err = Wire.endTransmission(send_stop);
  if (err == 0) return true;
  // Wire error codes: 2 = address NACK, 3 = data NACK, 4/5 = bus/timeout.
  if (err == 2) return fail(kErrorAddressNack, false);
  if (err == 3) return fail(kErrorDataNack, false);
  if (err == 5) {
    ++stats_.timeouts;
    return fail(kErrorTimeout, true);
  }
  return fail(kErrorBus, true);
}

bool I2cBus::read(uint8_t addr, uint8_t* data, uint8_t len, bool send_stop) {
  if (data == nullptr || len == 0) return false;
  if (!ready_ && !recover()) return false;
  cleanupStaleMaster();
  const uint8_t received = Wire.requestFrom(
      addr, len, static_cast<uint8_t>(send_stop ? 1 : 0));
  if (received != len) return fail(kErrorAddressNack, false);
  for (uint8_t index = 0; index < len; ++index) {
    const int value = Wire.read();
    if (value < 0) return fail(kErrorBus, false);
    data[index] = static_cast<uint8_t>(value);
  }
  return true;
}

#endif  // ARDUINO_ARCH_SAMD

bool I2cBus::fail(uint8_t error, bool recover_bus) {
  stats_.last_error = error;
  if (recover_bus) (void)recover();
  return false;
}

bool I2cBus::writeRead(uint8_t addr, const uint8_t* write_data,
                       uint8_t write_len, uint8_t* read_data,
                       uint8_t read_len) {
  return write(addr, write_data, write_len, false) &&
         read(addr, read_data, read_len, true);
}

bool I2cBus::probe(uint8_t addr) {
  return write(addr, nullptr, 0);
}

bool I2cBus::selectChannel(uint8_t channel) {
  if (channel >= kNumMuxChannels) {
    return false;
  }
  const uint8_t selection = static_cast<uint8_t>(1u << channel);
  if (!write(kAddrTcaMux, &selection, 1)) {
    stats_.mux_select_errors++;
    return false;
  }
  return true;
}

bool I2cBus::selectNone() {
  const uint8_t selection = 0;
  if (!write(kAddrTcaMux, &selection, 1)) {
    stats_.mux_select_errors++;
    return false;
  }
  return true;
}

void I2cBus::scanRoot(I2cTopology& topo) {
  stats_.root_scans++;
  topo.mux_present = probe(kAddrTcaMux);
  if (probe(kAddrOpenLog)) {
    topo.openlog_present = true;
    topo.openlog_address = kAddrOpenLog;
  } else if (probe(kAddrOpenLogJumper)) {
    topo.openlog_present = true;
    topo.openlog_address = kAddrOpenLogJumper;
  }
  if (probe(kAddrDebugOled)) {
    topo.oled_present = true;
    topo.oled_address = kAddrDebugOled;
  } else if (probe(kAddrDebugOledAlt)) {
    topo.oled_present = true;
    topo.oled_address = kAddrDebugOledAlt;
  }
  // Ensure no channel is left enabled from a prior scan.
  if (topo.mux_present) {
    selectNone();
  }
}

void I2cBus::scanChannels(I2cTopology& topo) {
  if (!topo.mux_present) {
    return;
  }
  stats_.channel_scans++;

  for (uint8_t ch = 0; ch < kNumFootChannels; ++ch) {
    ChannelInfo& info = topo.channels[ch];
    info = ChannelInfo{};

    if (!selectChannel(ch)) {
      // Mux write failed: leave this channel marked unscanned/missing.
      continue;
    }
    info.scanned = true;
    info.vcnl_present = probe(kAddrVcnl4040);
    info.lps_present = probe(kAddrLps25hb);
    info.device_count = static_cast<uint8_t>((info.vcnl_present ? 1 : 0) +
                                             (info.lps_present ? 1 : 0));
    updateChannelState(info);
  }

  // Always leave the bus with no channel selected so root devices are
  // addressable and no two boards can be live at once.
  selectNone();
}

void I2cBus::scanAll(I2cTopology& topo) {
  initTopology(topo);
  scanRoot(topo);
  scanChannels(topo);
}

}  // namespace i2c
