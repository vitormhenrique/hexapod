#include "i2c_bus.h"

namespace i2c {

void I2cBus::begin(uint32_t clock_hz) {
  wire_.begin();
  wire_.setClock(clock_hz);
  ready_ = true;
}

bool I2cBus::probe(uint8_t addr) {
  wire_.beginTransmission(addr);
  const uint8_t err = wire_.endTransmission();
  if (err != 0) {
    stats_.last_error = err;
  }
  return err == 0;
}

bool I2cBus::selectChannel(uint8_t channel) {
  if (channel >= kNumMuxChannels) {
    return false;
  }
  wire_.beginTransmission(kAddrTcaMux);
  wire_.write(static_cast<uint8_t>(1u << channel));  // one-hot: exclusive
  const uint8_t err = wire_.endTransmission();
  if (err != 0) {
    stats_.last_error = err;
    stats_.mux_select_errors++;
    return false;
  }
  return true;
}

bool I2cBus::selectNone() {
  wire_.beginTransmission(kAddrTcaMux);
  wire_.write(static_cast<uint8_t>(0x00));
  const uint8_t err = wire_.endTransmission();
  if (err != 0) {
    stats_.last_error = err;
    stats_.mux_select_errors++;
    return false;
  }
  return true;
}

void I2cBus::scanRoot(I2cTopology& topo) {
  stats_.root_scans++;
  topo.mux_present = probe(kAddrTcaMux);
  topo.eeprom_present = probe(kAddrEeprom);
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

  // Always leave the bus with no channel selected so the root bus (EEPROM) is
  // addressable and no two boards can be live at once.
  selectNone();
}

void I2cBus::scanAll(I2cTopology& topo) {
  initTopology(topo);
  scanRoot(topo);
  scanChannels(topo);
}

}  // namespace i2c
