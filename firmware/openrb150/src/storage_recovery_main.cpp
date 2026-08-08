#if defined(HEXAPOD_STORAGE_RECOVERY_BOOT)

#include <Arduino.h>

#include "board/board.h"
#include "config/config_bootstrap.h"
#include "config/config_schema.h"
#include "config/config_store.h"
#include "config/qwiic_openlog.h"
#include "safety/event_log.h"
#include "safety/fault_capture.h"
#include "sensors/i2c_bus.h"
#include "sensors/i2c_topology.h"
#include "protocol/crc16.h"

namespace {

i2c::I2cBus bus;
i2c::I2cTopology topology;
config::QwiicOpenLog openlog(bus);
config::QwiicConfigFile config_file(openlog);
config::ConfigStore config_store(config_file);

bool bus_ok = false;
bool openlog_ok = false;
bool crash_write_ok = false;
bool probe_write_ok = false;
bool probe_exists = false;
uint32_t probe_size_before = 0;
uint32_t probe_size_after = 0;
config::BootstrapResult bootstrap_result = config::BootstrapResult::StorageError;
uint16_t bootstrap_length = 0;
bool config_exists_before = false;
bool config_raw_size_ok = false;
uint32_t config_raw_size_before = 0;
bool config_decoded_size_ok = false;
uint32_t config_decoded_size_before = 0;
bool config_load_before = false;
bool config_commit_ok = false;
bool config_raw_size_after_ok = false;
uint32_t config_raw_size_after = 0;
bool config_exists_after = false;
bool config_load_after = false;
bool candidate_header_ok = false;
uint8_t candidate_header[config::kHeaderSize];
bool candidate_payload_ok = false;
uint16_t candidate_payload_crc = 0;
bool candidate_trailer_ok = false;
uint8_t candidate_trailer[config::kTrailerSize];
uint8_t openlog_address = 0;
uint8_t config_payload[config::kConfigPayloadSize];
char line[160];

bool appendLine(const char* name, const char* data, size_t length) {
  size_t offset = 0;
  while (offset < length) {
    const size_t remaining = length - offset;
    const uint8_t count = remaining < config::QwiicOpenLog::kMaxWriteBytes
                              ? static_cast<uint8_t>(remaining)
                              : config::QwiicOpenLog::kMaxWriteBytes;
    if (!openlog.append(name,
                        reinterpret_cast<const uint8_t*>(data + offset),
                        count)) return false;
    offset += count;
  }
  return openlog.sync();
}

void runStorageCheck() {
  fault_capture::init();
  board::init();
  bus_ok = bus.begin();
  i2c::initTopology(topology);
  if (bus_ok) bus.scanRoot(topology);
  openlog_address = topology.openlog_address;
  openlog_ok = topology.openlog_present && openlog.begin(openlog_address);
  if (!openlog_ok) return;

  const size_t crash_length = safety::formatCrashLog(
      fault_capture::lastSnapshot(), line, sizeof(line));
  crash_write_ok = crash_length == 0 ||
                   appendLine("EVENTS.LOG", line, crash_length);

  bool existed_before = false;
  (void)openlog.fileSize("PROBE.LOG", probe_size_before, existed_before);
  static const char probe[] = "OPENLOG_PROBE\n";
  probe_write_ok = appendLine("PROBE.LOG", probe, sizeof(probe) - 1u);
  probe_write_ok = probe_write_ok &&
      openlog.fileSize("PROBE.LOG", probe_size_after, probe_exists) &&
      probe_exists && probe_size_after >= probe_size_before + sizeof(probe) - 1u;

  config::RobotConfig defaults;
  config::defaultRobotConfig(defaults);
  const uint16_t defaults_length = config::serializeRobotConfig(
      defaults, config_payload, sizeof(config_payload));
    config_raw_size_ok = openlog.fileSize(
      "CONFIG.TXT", config_raw_size_before, config_exists_before);
    config_decoded_size_ok = config_file.size(config_decoded_size_before);
    candidate_header_ok = config_file.read(
        20, candidate_header, sizeof(candidate_header));
      candidate_payload_ok = config_file.read(
        20 + config::kHeaderSize, config_payload, config::kConfigPayloadSize);
      if (candidate_payload_ok) {
      candidate_payload_crc = protocol::crc16(
        config_payload, config::kConfigPayloadSize);
      }
      candidate_trailer_ok = config_file.read(
        20 + config::kHeaderSize + config::kConfigPayloadSize,
        candidate_trailer, sizeof(candidate_trailer));
    config_load_before = config_store.load(
      config_payload, sizeof(config_payload), bootstrap_length);
    if (!config_load_before) {
      (void)config::serializeRobotConfig(
        defaults, config_payload, sizeof(config_payload));
    config_commit_ok = config_store.commit(config_payload, defaults_length);
    config_raw_size_after_ok = openlog.fileSize(
      "CONFIG.TXT", config_raw_size_after, config_exists_after);
    config_load_after = config_store.load(
      config_payload, sizeof(config_payload), bootstrap_length);
    }
  bootstrap_result = config::loadOrInitializeConfig(
      config_file, config_store, config_payload, defaults_length,
      config_payload, sizeof(config_payload), bootstrap_length);
}

void printResults() {
  Serial.println("HEXAPOD OPENLOG STORAGE RECOVERY");
  Serial.print("bus_ok="); Serial.println(bus_ok ? 1 : 0);
  Serial.print("openlog_present="); Serial.print(topology.openlog_present ? 1 : 0);
  Serial.print(" address=0x"); Serial.println(openlog_address, HEX);
  Serial.print("openlog_sd_ready="); Serial.println(openlog_ok ? 1 : 0);
  Serial.print("crash_write_ok="); Serial.println(crash_write_ok ? 1 : 0);
  Serial.print("probe_write_ok="); Serial.print(probe_write_ok ? 1 : 0);
  Serial.print(" before="); Serial.print(probe_size_before);
  Serial.print(" after="); Serial.println(probe_size_after);
  Serial.print("bootstrap_result=");
  Serial.print(static_cast<uint8_t>(bootstrap_result));
  Serial.print(" length="); Serial.println(bootstrap_length);
  Serial.print("config_raw_before_ok="); Serial.print(config_raw_size_ok ? 1 : 0);
  Serial.print(" exists="); Serial.print(config_exists_before ? 1 : 0);
  Serial.print(" size="); Serial.println(config_raw_size_before);
  Serial.print("config_decoded_size_ok="); Serial.print(config_decoded_size_ok ? 1 : 0);
  Serial.print(" size="); Serial.println(config_decoded_size_before);
  Serial.print("config_load_before="); Serial.println(config_load_before ? 1 : 0);
  Serial.print("config_commit_ok="); Serial.println(config_commit_ok ? 1 : 0);
  Serial.print("config_raw_after_ok="); Serial.print(config_raw_size_after_ok ? 1 : 0);
  Serial.print(" exists="); Serial.print(config_exists_after ? 1 : 0);
  Serial.print(" size="); Serial.println(config_raw_size_after);
  Serial.print("config_load_after="); Serial.println(config_load_after ? 1 : 0);
  Serial.print("candidate_header_ok="); Serial.print(candidate_header_ok ? 1 : 0);
  Serial.print(" bytes=");
  for (uint8_t i = 0; i < sizeof(candidate_header); ++i) {
    if (candidate_header[i] < 16) Serial.print('0');
    Serial.print(candidate_header[i], HEX);
  }
  Serial.println();
  Serial.print("candidate_payload_ok="); Serial.print(candidate_payload_ok ? 1 : 0);
  Serial.print(" crc=0x"); Serial.println(candidate_payload_crc, HEX);
  Serial.print("candidate_trailer_ok="); Serial.print(candidate_trailer_ok ? 1 : 0);
  Serial.print(" bytes=");
  for (uint8_t i = 0; i < sizeof(candidate_trailer); ++i) {
    if (candidate_trailer[i] < 16) Serial.print('0');
    Serial.print(candidate_trailer[i], HEX);
  }
  Serial.println();
  Serial.println("DXL power forced OFF; normal application not started.");
}

}  // namespace

void setup() {
  runStorageCheck();
  Serial.begin(115200);
}

void loop() {
  printResults();
  delay(1000);
}

#endif