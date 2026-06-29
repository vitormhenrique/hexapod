#include "tasks.h"

#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#include <math.h>

#include "../board/board.h"
#include "../config/config_api.h"
#include "../config/eeprom_24lc32.h"
#include "../dxl/dxl_bus.h"
#include "../dxl/dxl_params.h"
#include "../dxl/servo_map.h"
#include "../gait/gait_pipeline.h"
#include "../input/crsf_parser.h"
#include "../protocol/api.h"
#include "../protocol/control_api.h"
#include "../protocol/dxl_job_api.h"
#include "../protocol/feature_api.h"
#include "../protocol/frame_reader.h"
#include "../protocol/framing.h"
#include "../protocol/maintenance_api.h"
#include "../protocol/maintenance_target_api.h"
#include "../protocol/motion_api.h"
#include "../protocol/passive_api.h"
#include "../protocol/sensor_api.h"
#include "../protocol/telemetry.h"
#include "../sensors/contact_estimator.h"
#include "../sensors/finger_sensor.h"
#include "../sensors/i2c_bus.h"
#include "../safety/command_arbiter.h"
#include "../safety/state_machine.h"
#include "../safety/system_state.h"
#include "../safety/watchdog.h"
#include "task_config.h"

namespace app {
namespace {

// Task handles, indexed by watchdog::TaskId, so the health task can read each
// task's stack high-water mark.
TaskHandle_t g_handles[watchdog::kTaskCount] = {nullptr};

// Per-task loop counters (single producer each), reported by the health task.
volatile uint32_t g_loops[watchdog::kTaskCount] = {0};

// Static description of this firmware build, reported by HELLO/GET_CAPABILITIES.
protocol::api::DeviceInfo g_deviceInfo;

// Single owner of the DYNAMIXEL TTL bus (Serial1). Only dxlTask touches this,
// satisfying the AGENTS.md rule that one task owns Dynamixel2Arduino/Serial1.
dxl::DxlBus g_dxlBus(Serial1);

// Latest per-servo status snapshot, published by dxlTask via Sync Read and read
// by telemetry/safety consumers. Single writer (dxlTask); readers take a copy.
dxl::ServoStatus g_servoStatus[dxl::DxlBus::kMaxServos];
volatile uint8_t g_servoStatusCount = 0;

// CRSF/ExpressLRS RC input state. Owned exclusively by rcTask (Serial2).
crsf::Parser g_crsfParser;
crsf::RcStatus g_rcStatus;

// Command-source arbiter (RC / Jetson / Mac). controlTask evaluates it each
// cycle from the RC snapshot; apiTask feeds Jetson heartbeats, the Mac
// maintenance lock, and host estop into it. The published authority gates the
// DXL goal-write path (motion is denied unless a source owns it).
safety::CommandArbiter g_arbiter;
volatile uint8_t g_commandSource = 0;       // safety::CommandSource value
volatile bool g_motionAuthorized = false;   // a source may drive servos
volatile bool g_killActive = true;          // RC kill / host estop asserted

// Authoritative safety state machine (AGENTS.md 5.3). controlTask advances it
// each cycle from health/RC/arbiter inputs; the result is the single source of
// truth for which states permit torque and goal writes. dxlTask enforces the
// gate at the bus level.
safety::StateMachine g_stateMachine;
volatile uint8_t g_safetyState = static_cast<uint8_t>(safety::State::Boot);
volatile uint8_t g_faultReason = 0;  // safety::FaultReason value
// True only when the current state permits motion AND a source owns authority.
volatile bool g_motionGate = false;
// True once i2cTask has finished its boot scan and seeded a (persisted or
// default) config; gates the ConfigLoad -> Disarmed transition.
volatile bool g_configReady = false;

// Telemetry subscription manager. apiTask routes the telemetry command range to
// it and walks the streams each loop, emitting due telemetry frames over USB.
protocol::SubscriptionManager g_subs;

// Host safety control intent (ESTOP/CLEAR_FAULT/SET_ARMING/SET_MODE). apiTask
// records host intent into it; controlTask folds that intent into the safety
// state machine each cycle and publishes the live state/fault back for command
// responses to echo.
protocol::ControlApi g_controlApi;

// Host high-level motion intent (SET_GAIT/SET_GAIT_PARAMS/SET_BODY_TWIST/
// SET_BODY_POSE/STOP_MOTION). apiTask validates+stores the latest clamped
// MotionIntent here; controlTask echoes the live motion gate back so the host
// knows whether the intent is being honored. The goal-generation (gait/IK)
// path consumes g_motionApi.intent() and is gated by g_motionGate.
protocol::MotionApi g_motionApi;

// Servo goal frame published by controlTask's gait/IK pipeline (lmt.1) and
// consumed by dxlTask's Sync Write (lmt.2). Single writer (controlTask) and two
// readers (dxlTask drives the bus, apiTask renders the servo_goals telemetry),
// guarded by a briefly-held mutex (the critical section is a small fixed copy,
// so no task blocks meaningfully). Stores the full per-joint result (id, tick,
// leg, joint, clamped) so the goal-write path and the clamp-flag telemetry
// (audit 22l.3) share one source of truth. g_goalValid is cleared whenever
// motion is not gated open, so neither the bus write nor the telemetry renders
// a stale frame once authority is lost.
struct GoalFrame {
  uint8_t count = 0;
  gait::PipelineJoint joints[config::kNumServos];
};
GoalFrame g_goalFrame;
SemaphoreHandle_t g_goalMutex = nullptr;  // guards g_goalFrame
volatile bool g_goalValid = false;        // a fresh, gated goal frame is ready
volatile uint32_t g_goalSeq = 0;          // bumped on each published frame
// True when the last published frame saturated any joint against the configured
// servo travel; surfaced for clamp diagnostics (lmt.2 / audit 22l.3).
volatile bool g_goalClamped = false;


// Host maintenance lock (ENTER/EXIT/HEARTBEAT). apiTask drives the lock token +
// TTL here; controlTask reads lockHeld() to feed the safety FSM's maintenance
// inputs and force-revokes the lock on E-stop / fault.
protocol::MaintenanceApi g_maintApi;

// Host maintenance leg/joint targets (SET_LEG_TARGET/SET_JOINT_TARGET). Runs the
// foot/joint request through body+leg IK and the servo map, storing clamped
// goal ticks. Only honored while in MacMaintenance with the lock held; the goal
// write path consumes the stored targets under MacMaintenance authority.
protocol::MaintTargetApi g_maintTargetApi;

// Host DXL maintenance command queue (DXL_SCAN/PING/TORQUE/PROFILE). apiTask
// enqueues a gated job here; dxlTask is the sole executor (it owns the bus) and
// writes the serialized result back for the host to poll via DXL_GET_RESULT.
protocol::DxlJobApi g_dxlJobApi;

// Host feature-flag command surface (FEATURE_GET/SET/GET_REASONS/RESET). Stores
// the host's desired enable set; controlTask publishes per-feature availability
// each cycle and consumes the desired set to drive the real toggles. Effective
// enabled = desired && available, so the firmware can auto-disable a feature
// (e.g. when its hardware disappears) without losing the host's intent.
protocol::FeatureApi g_featureApi;

// Host sensor / contact / leveling command surface (CONTACT_*/LEVELING_*/
// I2C_*/SENSOR_*). Enable/disable route through g_featureApi (single source of
// truth); CONTACT_SET_THRESHOLDS stages per-foot thresholds consumed by i2cTask
// (the contact-estimator owner). Wired to g_featureApi once at apiTask startup.
protocol::SensorApi g_sensorApi;

// Published snapshots the SensorApi reads (apiTask) and i2cTask keeps current:
// the I2C topology (refreshed on each scan) and the fused foot-contact status.
// Single-writer (i2cTask) / multi-reader PODs, like g_footState.
protocol::TopologySnapshot g_sensorTopoSnap;
protocol::StatusSnapshot g_sensorStatusSnap;

// Host passive pose streaming command surface (PASSIVE_ENTER/EXIT/
// SET_STREAM_RATE/ZERO_REFERENCE). apiTask records the host's request here;
// controlTask folds requested() into the safety FSM (only enters
// PassivePoseStream once torque is confirmed off) and force-clears it on
// E-stop / fault.
protocol::PassiveApi g_passiveApi;

// Torque-off confirmation published by dxlTask. The control task feeds this to
// the safety FSM as StateInputs.torque_off so PassivePoseStream is only entered
// when no servo is driven. Starts true (DXL power + torque are OFF at boot) and
// is set true again after dxlTask disables torque; the goal-write/torque-enable
// path (when wired) is the only place that clears it.
volatile bool g_dxlTorqueOff = true;

// CRSF runs at 420000 baud on the OpenRB-150 4-pin UART (Serial2).
constexpr uint32_t kCrsfBaud = 420000;

// Single owner of the root I2C bus (Wire): TCA9548A mux, 24LC32 EEPROM, and the
// muxed foot sensors. Only i2cTask touches this. Topology is the boot-scan
// result describing which optional devices were found.
i2c::I2cBus g_i2cBus(Wire);
i2c::I2cTopology g_i2cTopology;

// Foot contact pipeline. The reader does the Wire/mux register I/O (Arduino),
// the estimator runs the portable contact state machine, and the published
// snapshot is read by telemetry/safety consumers. All owned by i2cTask.
sensors::FingerSensorReader g_finger(Wire);
sensors::ContactEstimator g_contact;
sensors::LegContactState g_footState[sensors::kNumFeet];
volatile uint8_t g_footPresentMask = 0;
// Runtime sensor-polling toggle (feature.foot_sensor polling). Defaults on so
// present boards stream raw proximity/pressure; contact classification stays
// disabled per-foot until calibrated (FootSensorCal.enabled).
volatile bool g_sensorPollingEnabled = true;

// Persistent robot config in the 24LC32 EEPROM (root bus). When the EEPROM is
// missing or holds no valid slot the config is marked volatile and the firmware
// must run on compiled defaults and reject commits (AGENTS.md 4.3). At boot the
// i2cTask loads any valid slot and hands it to apiTask; thereafter the config
// API (apiTask) edits a RAM shadow and routes CFG_COMMIT back to i2cTask.
config::Eeprom24LC32 g_eeprom(Wire);
config::ConfigStore g_configStore(g_eeprom);
bool g_configVolatile = true;

// --- Cross-task config plumbing (AGENTS.md 5.1: only i2cTask touches Wire) ---
//
// The config API runs in apiTask (it parses USB frames), but the EEPROM commit
// is a Wire transaction that only i2cTask is allowed to perform. So apiTask
// edits/validates a RAM shadow locally, and a CFG_COMMIT hands the validated
// serialized payload to i2cTask through this mailbox and blocks (bounded) for
// the result. A separate one-shot boot-load buffer lets i2cTask pass a valid
// persisted config to apiTask so the ConfigApi shadow is still touched by only
// one task.
struct CommitMailbox {
  bool requested = false;
  bool ok = false;
  uint16_t len = 0;
  uint8_t payload[config::kConfigPayloadSize] = {0};
};
CommitMailbox g_commit;
SemaphoreHandle_t g_commitMutex = nullptr;  // guards g_commit
SemaphoreHandle_t g_commitDone = nullptr;   // i2cTask -> apiTask completion

struct BootLoad {
  bool ready = false;     // a valid persisted payload was loaded at boot
  bool consumed = false;  // apiTask has adopted it
  uint16_t len = 0;
  uint8_t payload[config::kConfigPayloadSize] = {0};
};
BootLoad g_bootLoad;

// Persistence sink used by the config API. commitPayload() is called from
// apiTask; it forwards the bytes to i2cTask and waits for the transaction.
class TaskConfigPersistence : public config::ConfigPersistence {
 public:
  bool commitPayload(const uint8_t* payload, uint16_t len) override {
    if (g_commitMutex == nullptr || g_commitDone == nullptr) return false;
    if (len > sizeof(g_commit.payload)) return false;
    xSemaphoreTake(g_commitMutex, portMAX_DELAY);
    memcpy(g_commit.payload, payload, len);
    g_commit.len = len;
    g_commit.ok = false;
    g_commit.requested = true;
    xSemaphoreGive(g_commitMutex);
    // Wait for i2cTask to run the EEPROM transaction (normally < 200 ms).
    if (xSemaphoreTake(g_commitDone, pdMS_TO_TICKS(1500)) != pdTRUE) {
      return false;  // timed out
    }
    xSemaphoreTake(g_commitMutex, portMAX_DELAY);
    const bool ok = g_commit.ok;
    xSemaphoreGive(g_commitMutex);
    return ok;
  }
  bool persistent() const override { return !g_configVolatile; }
};

TaskConfigPersistence g_configPersist;
config::ConfigApi g_configApi(g_configPersist);

// Gait -> servo goal pipeline (gait engine -> body IK -> servo map), owned by
// controlTask (lmt.1). Declared after g_configApi so its captured config
// reference is the live RAM shadow; the ServoMap stage reads the shadow by
// reference, while the gait engine + body transform are re-seeded from config
// on adopt/commit (lmt.7).
gait::GaitPipeline g_pipeline(g_configApi.config());

void initDeviceInfo() {
  g_deviceInfo.fw_major = 0;
  g_deviceInfo.fw_minor = 1;
  g_deviceInfo.fw_patch = 0;
  g_deviceInfo.feature_bits = 0;  // populated as features land in Phase 2
  const char name[] = "OpenRB150-Hex";
  size_t i = 0;
  for (; name[i] != '\0' && i < protocol::api::kDeviceNameLen; ++i) {
    g_deviceInfo.device_name[i] = name[i];
  }
  for (; i < protocol::api::kDeviceNameLen; ++i) {
    g_deviceInfo.device_name[i] = 0;
  }
}

inline void tick(watchdog::TaskId id) {
  const uint8_t i = static_cast<uint8_t>(id);
  g_loops[i]++;
  watchdog::checkIn(id);
}

// --- Telemetry payload encoding -------------------------------------------
// Little-endian writers for building telemetry frame payloads in place.
inline uint16_t put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  return 2;
}
inline uint16_t put32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
  return 4;
}

// Build the payload for telemetry `stream` into `p` (capacity kMaxPayload).
// Returns the payload length. Reads only the published cross-task snapshots, so
// it never touches a peripheral or blocks. Payloads stay within 256 bytes.
uint16_t buildTelemetry(protocol::StreamId stream, uint8_t* p,
                        uint32_t now_ms) {
  using protocol::StreamId;
  uint16_t o = 0;
  switch (stream) {
    case StreamId::Health: {
      o += put32(&p[o], now_ms);
      p[o++] = g_safetyState;
      p[o++] = g_faultReason;
      o += put32(&p[o], watchdog::missedMask());
      o += put16(&p[o], board::readBatteryMilliVolts());
      break;
    }
    case StreamId::ControlState: {
      p[o++] = g_commandSource;
      p[o++] = g_motionAuthorized ? 1 : 0;
      p[o++] = g_killActive ? 1 : 0;
      p[o++] = g_safetyState;
      p[o++] = g_faultReason;
      p[o++] = g_motionGate ? 1 : 0;
      break;
    }
    case StreamId::ServoStatus: {
      // count(1) then 14 bytes/servo: id, pos(4), vel(2), load(2), volt_mv(2),
      // temp(1), err(1), torque_enable(1). 18 servos -> 253 bytes, within the
      // 256 payload cap. Position is refreshed for all servos every cycle by the
      // all-servo Sync Read; the detail fields (vel/load/volt/temp/torque) are
      // filled by the dxlTask round-robin per-servo read (eax.6), so they are
      // valid once each servo has been polled at least once.
      const uint8_t n = g_servoStatusCount;
      p[o++] = n;
      for (uint8_t i = 0; i < n; ++i) {
        const dxl::ServoStatus& s = g_servoStatus[i];
        p[o++] = s.id;
        o += put32(&p[o], static_cast<uint32_t>(s.present_position));
        o += put16(&p[o], static_cast<uint16_t>(s.present_velocity));
        o += put16(&p[o], static_cast<uint16_t>(s.present_load));
        o += put16(&p[o], s.present_voltage_mv);
        p[o++] = static_cast<uint8_t>(s.present_temperature_c);
        p[o++] = s.hardware_error;
        p[o++] = s.torque_enabled ? 1 : 0;
      }
      break;
    }
    case StreamId::ContactState: {
      // 6 feet x 4 bytes: state(1), confidence(1), pressure_delta(2).
      for (uint8_t i = 0; i < sensors::kNumFeet; ++i) {
        const sensors::LegContactState& f = g_footState[i];
        p[o++] = static_cast<uint8_t>(f.state);
        p[o++] = f.confidence;
        o += put16(&p[o], static_cast<uint16_t>(f.pressure_delta));
      }
      break;
    }
    case StreamId::I2cSensorsRaw: {
      // 6 feet x 6 bytes: proximity(2), pressure_raw(4).
      for (uint8_t i = 0; i < sensors::kNumFeet; ++i) {
        const sensors::LegContactState& f = g_footState[i];
        o += put16(&p[o], f.proximity_raw);
        o += put32(&p[o], static_cast<uint32_t>(f.pressure_raw));
      }
      break;
    }
    case StreamId::RcInput: {
      // flags(1): bit0 armed, bit1 kill, bit2 failsafe, bit3 autonomy.
      uint8_t flags = 0;
      if (g_rcStatus.armed) flags |= 0x01;
      if (g_rcStatus.kill) flags |= 0x02;
      if (g_rcStatus.failsafe) flags |= 0x04;
      if (g_rcStatus.autonomy) flags |= 0x08;
      p[o++] = flags;
      p[o++] = g_rcStatus.gait_index;
      for (uint8_t i = 0; i < crsf::kNumChannels; ++i) {
        o += put16(&p[o], g_rcStatus.channels_us[i]);
      }
      break;
    }
    case StreamId::ApiStats: {
      // tx_backlog(4) then per-stream dropped(4) for the 7 streams.
      o += put32(&p[o], g_subs.txBacklog());
      for (uint8_t i = 0; i < protocol::kNumStreams; ++i) {
        o += put32(&p[o], g_subs.dropped(static_cast<StreamId>(i)));
      }
      break;
    }
    case StreamId::JointState: {
      // Mapped present joint angles, ready to render without the host needing
      // the servo map (eax.1). count(1) then 4 bytes/joint: leg(1), joint(1),
      // angle_centideg(int16). Angles come from the active config's servo map
      // (sign/trim/center 2048, 4096 ticks/rev), so they read correctly in both
      // active and passive (torque-off) modes from the present-position snapshot.
      // 18 joints -> 1 + 18*4 = 73 bytes, within the 256 payload cap.
      const dxl::ServoMap map(g_configApi.config());
      const uint8_t n = g_servoStatusCount;
      uint8_t* countp = &p[o++];
      uint8_t emitted = 0;
      for (uint8_t i = 0; i < n; ++i) {
        const dxl::ServoStatus& s = g_servoStatus[i];
        const config::ServoConfig* sc = map.servoForId(s.id);
        if (sc == nullptr) continue;  // skip servos not in the map
        // Single-turn present position; clamp into the device range so a
        // multi-turn or stale read cannot wrap the angle.
        int32_t raw = s.present_position;
        if (raw < 0) raw = 0;
        if (raw > config::kServoMaxTick) raw = config::kServoMaxTick;
        const float rad =
            map.tickToAngle(sc->leg, sc->joint, static_cast<uint16_t>(raw));
        long centideg = lroundf(rad * dxl::kRadToDeg * 100.0f);
        if (centideg > 32767) centideg = 32767;
        if (centideg < -32768) centideg = -32768;
        p[o++] = sc->leg;
        p[o++] = sc->joint;
        o += put16(&p[o], static_cast<uint16_t>(static_cast<int16_t>(centideg)));
        ++emitted;
      }
      *countp = emitted;
      break;
    }
    case StreamId::ServoGoals: {
      // Per-joint commanded goal after IK + servo-map clamping (eax.2). Lets the
      // host overlay commanded vs present pose in the animation. count(1) then
      // 5 bytes/joint: leg(1), joint(1), angle_centideg(int16), flags(1). flags
      // bit0 = clamped (goal saturated against configured servo travel).
      // While motion is gated open the live gait/IK goal frame (lmt.2) is the
      // authoritative source; on the bench (motion not gated) the stored
      // maintenance target set is rendered instead. Either way only joints with
      // a real command are emitted, so an idle robot yields a zero count.
      // 18 joints -> 1 + 18*5 = 91 bytes, within the 256 payload cap.
      const dxl::ServoMap map(g_configApi.config());
      uint8_t* countp = &p[o++];
      uint8_t emitted = 0;
      bool used_gait = false;
      // Live gait goals take priority while motion is authorised. Copy under a
      // zero-wait lock; if controlTask is mid-publish, fall back to the bench
      // target set for this frame rather than block the telemetry task.
      if (g_motionGate && g_goalValid && g_goalMutex != nullptr &&
          xSemaphoreTake(g_goalMutex, 0) == pdTRUE) {
        const uint8_t n = g_goalFrame.count;
        for (uint8_t i = 0; i < n; ++i) {
          const gait::PipelineJoint& jt = g_goalFrame.joints[i];
          const float rad = map.tickToAngle(jt.leg, jt.joint, jt.tick);
          long centideg = lroundf(rad * dxl::kRadToDeg * 100.0f);
          if (centideg > 32767) centideg = 32767;
          if (centideg < -32768) centideg = -32768;
          p[o++] = jt.leg;
          p[o++] = jt.joint;
          o += put16(&p[o],
                     static_cast<uint16_t>(static_cast<int16_t>(centideg)));
          p[o++] = jt.clamped ? 0x01 : 0x00;
          ++emitted;
        }
        xSemaphoreGive(g_goalMutex);
        used_gait = true;
      }
      if (!used_gait) {
        const protocol::MaintTargetSet& tgt = g_maintTargetApi.target();
        for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
          for (uint8_t j = 0; j < config::kJointsPerLeg; ++j) {
            if (!tgt.set[leg][j]) continue;
            if (map.servoFor(leg, j) == nullptr) continue;  // skip unmapped
            const float rad = map.tickToAngle(leg, j, tgt.tick[leg][j]);
            long centideg = lroundf(rad * dxl::kRadToDeg * 100.0f);
            if (centideg > 32767) centideg = 32767;
            if (centideg < -32768) centideg = -32768;
            p[o++] = leg;
            p[o++] = j;
            o += put16(&p[o],
                       static_cast<uint16_t>(static_cast<int16_t>(centideg)));
            p[o++] = tgt.clamped[leg][j] ? 0x01 : 0x00;
            ++emitted;
          }
        }
      }
      *countp = emitted;
      break;
    }
    case StreamId::LegState: {
      // Per-leg commanded foot target + IK verdict (eax.3). Lets the animation
      // draw the commanded foot positions and flag unreachable poses. count(1)
      // then 8 bytes/leg: leg(1), foot_x(i16), foot_y(i16), foot_z(i16, mm body
      // frame), flags(1). flags bit0 = reachable, bit1 = clamped (a joint hit
      // its configured travel). Only legs with a recorded SET_LEG_TARGET attempt
      // are emitted, so until a leg target is sent the payload is a zero count.
      // 6 legs -> 1 + 6*8 = 49 bytes, within the 256 payload cap.
      const protocol::MaintTargetSet& tgt = g_maintTargetApi.target();
      uint8_t* countp = &p[o++];
      uint8_t emitted = 0;
      for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
        if (!tgt.leg_target_set[leg]) continue;
        p[o++] = leg;
        o += put16(&p[o], static_cast<uint16_t>(tgt.foot_x_mm[leg]));
        o += put16(&p[o], static_cast<uint16_t>(tgt.foot_y_mm[leg]));
        o += put16(&p[o], static_cast<uint16_t>(tgt.foot_z_mm[leg]));
        uint8_t flags = 0;
        if (tgt.leg_reachable[leg]) flags |= 0x01;
        if (tgt.leg_clamped[leg]) flags |= 0x02;
        p[o++] = flags;
        ++emitted;
      }
      *countp = emitted;
      break;
    }
  }
  return o;
}

// Publish per-feature availability to the FeatureApi and drive the real runtime
// toggles from the host's desired set. Effective enabled = desired && available,
// so a feature whose hardware/state disappears is auto-disabled while the host's
// intent is preserved (AGENTS.md 1.3). Runs each control cycle.
//
// Availability today:
//   * SensorPolling   - available when the I2C mux is present; the one fully
//                        wired runtime toggle (drives g_sensorPollingEnabled).
//   * FootContact     - needs mux + >=1 present foot sensor + polling on; the
//                        gait-engine consumption lands in ubs.5, so it reports
//                        available for streaming but is not yet fed to the gait.
//   * TerrainLeveling - depends on FootContact (ubs.5); reports DependencyOff.
//   * PassivePose     - torque-off pose streaming lands in ubs.6; available
//                        once wired (the passive command group enters the
//                        torque-off PassivePoseStream state).
//   * JetsonControl   - Jetson bridge lands later; NotImplemented.
void updateFeatureFlags(uint32_t /*now_ms*/) {
  using protocol::Feature;
  using protocol::FeatureReason;

  const bool mux = g_i2cTopology.mux_present;
  const bool any_foot = g_footPresentMask != 0;

  // SensorPolling: real toggle, available whenever the mux is present.
  g_featureApi.setAvailability(
      Feature::SensorPolling, mux,
      mux ? FeatureReason::None : FeatureReason::HardwareMissing);

  // FootContact: estimator can run when sensors are present and polled.
  const bool polling = g_featureApi.effectiveEnabled(Feature::SensorPolling);
  FeatureReason contact_reason = FeatureReason::None;
  bool contact_avail = true;
  if (!mux || !any_foot) {
    contact_avail = false;
    contact_reason = FeatureReason::HardwareMissing;
  } else if (!polling) {
    contact_avail = false;
    contact_reason = FeatureReason::DependencyOff;
  }
  g_featureApi.setAvailability(Feature::FootContact, contact_avail,
                               contact_reason);

  // TerrainLeveling: needs FootContact active (gait consumption is ubs.5).
  const bool contact_on = g_featureApi.effectiveEnabled(Feature::FootContact);
  g_featureApi.setAvailability(
      Feature::TerrainLeveling, false,
      contact_on ? FeatureReason::NotImplemented : FeatureReason::DependencyOff);

  // PassivePose: torque-off present-position streaming is wired (ubs.6). The
  // passive command group + safety FSM own the actual mode; this just reports
  // the capability as available so GET_CAPABILITIES / feature_state are honest.
  g_featureApi.setAvailability(Feature::PassivePose, true, FeatureReason::None);
  // JetsonControl: not yet wired in this build.
  g_featureApi.setAvailability(Feature::JetsonControl, false,
                               FeatureReason::NotImplemented);

  g_featureApi.setLiveState(g_safetyState);

  // Drive the one real toggle: raw foot-sensor polling.
  g_sensorPollingEnabled =
      g_featureApi.effectiveEnabled(Feature::SensorPolling);
}

// --- Task bodies ----------------------------------------------------------
// Each task runs a fixed-period loop with vTaskDelayUntil so timing does not
// drift with body execution time. Bodies are stubs for the skeleton.

void controlTask(void*) {
  TickType_t next = xTaskGetTickCount();
  g_stateMachine.reset();
  for (;;) {
    tick(watchdog::TaskId::Control);

    // Arbitrate command authority from the latest RC snapshot. rcTask owns the
    // RC state; we only read a copy of the few fields the arbiter needs. The
    // arbiter is the single source of truth for who (if anyone) may move the
    // robot this cycle; the result gates the DXL goal-write path in dxlTask.
    const uint32_t now_ms =
        static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
    safety::RcInputs rc_in;
    rc_in.ever_seen = g_rcStatus.ever_seen;
    rc_in.kill = g_rcStatus.kill;
    rc_in.armed = g_rcStatus.armed;
    rc_in.autonomy_enabled = g_rcStatus.autonomy;
    const safety::ArbiterOutput& arb = g_arbiter.update(rc_in, now_ms);
    g_commandSource = static_cast<uint8_t>(arb.source);
    g_motionAuthorized = arb.motion_authorized;
    g_killActive = arb.kill_active;

    // Advance the safety state machine. It owns the boot/arm/walk/maintenance/
    // fault/estop transitions and decides whether torque and goal writes are
    // permitted this cycle. Inputs are sampled from the published snapshots so
    // the machine never blocks on a peripheral.
    const uint16_t batt_mv = board::readBatteryMilliVolts();
    safety::StateInputs si;
    si.config_loaded = g_configReady;
    si.battery_mv = batt_mv;
    si.battery_valid = batt_mv > 6000;  // below this = no pack sense (USB bench)
    si.watchdog_fault = watchdog::missedMask() != 0;
    si.dxl_hard_fault = false;  // TODO: derive from repeated bus / HW errors
    si.host_estop = g_arbiter.hostEstop() || g_controlApi.estopActive();
    si.rc_kill = g_rcStatus.kill;
    si.rc_failsafe = g_rcStatus.failsafe;
    si.rc_ever_seen = g_rcStatus.ever_seen;
    si.rc_armed = g_rcStatus.armed;
    si.arming_checks_pass = g_configReady;
    si.host_disarm = g_controlApi.disarmRequested();
    si.command_source = g_commandSource;
    si.jetson_fresh = g_arbiter.jetsonFresh(now_ms);
    si.rc_autonomy = g_rcStatus.autonomy;
    si.mac_lock_held = g_arbiter.macLockHeld(now_ms);
    // The maintenance lock is owned by the USB MaintenanceApi; a held, fresh
    // lock is both the maintenance request and the lock-held input the FSM
    // keys on to enter/hold MacMaintenance (AGENTS.md 6.4).
    const bool maint_held = g_maintApi.lockHeld(now_ms);
    si.mac_lock_held = si.mac_lock_held || maint_held;
    si.maintenance_request = maint_held;
    si.passive_request = g_passiveApi.requested();
    si.torque_off = g_dxlTorqueOff;
    si.contact_enabled = false;      // wired with the contact feature toggle
    si.contact_confident = false;
    // Honor a host CLEAR_FAULT pulse before advancing the machine.
    if (g_controlApi.consumeClearFault()) {
      g_stateMachine.requestClearFault();
    }
    const safety::State state = g_stateMachine.update(si, now_ms);
    g_safetyState = static_cast<uint8_t>(state);
    g_faultReason = static_cast<uint8_t>(g_stateMachine.faultReason());
    // Force-revoke the maintenance lock once the robot is in a fault/E-stop
    // state so a stale Mac client cannot retain bench authority across a fault.
    if (g_safetyState >= static_cast<uint8_t>(safety::State::FaultSoft)) {
      g_maintApi.revoke();
      g_passiveApi.clear();
    }
    // Publish the live state/fault so control-command responses can echo it.
    g_controlApi.setLiveState(g_safetyState, g_faultReason);
    // Echo the live safety state to the passive handler so PASSIVE_ENTER can
    // gate on a maintenance-safe state and responses echo it.
    g_passiveApi.setLiveState(g_safetyState);
    // The motion gate is the conjunction of "the state allows movement" and
    // "a command source owns authority". dxlTask uses it to keep torque off and
    // suppress goal writes unless both hold. Goal generation (gait/IK) lands in
    // a later task; this enforces the safety contract for it up front.
    g_motionGate = safety::stateAllowsMotion(state) && arb.motion_authorized;

    // Echo the live motion gate to the motion handler so SET_GAIT/twist/pose
    // responses tell the host whether the stored intent is being honored now or
    // parked until the robot is armed/authorised.
    g_motionApi.setLiveState(g_safetyState, g_motionGate);

    // --- Gait -> servo goal generation (lmt.1) ------------------------------
    // Translate the latest high-level motion intent into concrete goal ticks and
    // publish a frame for dxlTask (gait engine -> body IK -> servo map). The
    // pipeline is built once from the active config (rebuilt on config adopt/
    // commit by lmt.7). It is only advanced and published while motion is gated
    // open; the phase is reset on the rising edge of the gate so each
    // (re)authorisation starts from a clean cycle, and the frame is invalidated
    // when the gate closes so the bus-write path (lmt.2) holds position instead
    // of replaying a stale goal.
    static uint32_t applied_intent_seq = 0xFFFFFFFFu;
    static bool prev_motion_gate = false;
    if (g_motionGate) {
      const protocol::MotionIntent& intent = g_motionApi.intent();
      if (intent.seq != applied_intent_seq) {
        g_pipeline.setGait(static_cast<config::GaitId>(intent.gait));
        g_pipeline.setParams(intent.body_height_mm, intent.stride_len_mm,
                             intent.step_height_mm, intent.duty_x255,
                             intent.speed_x255);
        g_pipeline.setTwist(intent.twist_vx, intent.twist_vy, intent.twist_wz);
        applied_intent_seq = intent.seq;
      }
      if (!prev_motion_gate) {
        g_pipeline.resetPhase();
      }
      gait::PipelineOutput goals;
      g_pipeline.update(period_ms::kControl, goals);
      // Publish for dxlTask under a zero-wait lock: if dxlTask momentarily holds
      // it, skip this frame (keep the previous) rather than block controlTask.
      if (g_goalMutex != nullptr && xSemaphoreTake(g_goalMutex, 0) == pdTRUE) {
        g_goalFrame.count = goals.count;
        bool any_clamped = false;
        for (uint8_t i = 0; i < goals.count; ++i) {
          g_goalFrame.joints[i] = goals.joints[i];
          if (goals.joints[i].clamped) any_clamped = true;
        }
        g_goalClamped = any_clamped;
        ++g_goalSeq;
        g_goalValid = true;
        xSemaphoreGive(g_goalMutex);
      }
    } else {
      g_goalValid = false;
      applied_intent_seq = 0xFFFFFFFFu;  // force re-apply on next authorisation
    }
    prev_motion_gate = g_motionGate;

    // Gate maintenance leg/joint targets on the live MacMaintenance state +
    // held lock so a foot/joint nudge is only accepted on the bench.
    g_maintTargetApi.setLiveState(g_safetyState, g_maintApi.lockHeld(now_ms));
    // Same gate for the DXL maintenance command queue: scans/pings/torque/
    // profile jobs are only accepted on the bench (MacMaintenance + lock).
    g_dxlJobApi.setLiveState(g_safetyState, g_maintApi.lockHeld(now_ms));

    // --- Feature flags: publish availability and consume host intent --------
    // Reflect what the current hardware/state permits so FEATURE_SET can only
    // be honoured when it is safe (AGENTS.md 1.3). Engines that are not yet
    // wired report NotImplemented so the host gets an honest reason.
    updateFeatureFlags(now_ms);
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kControl));
  }
}

// Pack a ServoProfile's capability flags into a single byte for the wire.
uint8_t packServoCaps(const dxl::ServoProfile& p) {
  uint8_t c = 0;
  if (p.supports_sync_read) c |= 0x01;
  if (p.supports_fast_sync_read) c |= 0x02;
  if (p.supports_cw_ccw_angle_limits) c |= 0x04;
  if (p.supports_min_max_position_limits) c |= 0x08;
  if (p.supports_profile_velocity) c |= 0x10;
  if (p.supports_bus_watchdog) c |= 0x20;
  return c;
}

// Append the compact 6-byte servo record (id, model[2], fw, proto, table) used
// by the DXL_SCAN result list. Returns the new write offset.
uint8_t appendCompactServo(uint8_t* data, uint8_t off,
                           const dxl::ServoProfile& p) {
  data[off++] = p.id;
  data[off++] = static_cast<uint8_t>(p.model_number & 0xFF);
  data[off++] = static_cast<uint8_t>((p.model_number >> 8) & 0xFF);
  data[off++] = p.firmware_version;
  data[off++] = p.protocol_version;
  data[off++] = static_cast<uint8_t>(p.table_kind);
  return off;
}

// Little-endian int32 into the result buffer. Returns the new write offset.
uint8_t appendI32(uint8_t* data, uint8_t off, int32_t v) {
  const uint32_t raw = static_cast<uint32_t>(v);
  data[off++] = static_cast<uint8_t>(raw & 0xFF);
  data[off++] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  data[off++] = static_cast<uint8_t>((raw >> 16) & 0xFF);
  data[off++] = static_cast<uint8_t>((raw >> 24) & 0xFF);
  return off;
}

// Write one logical parameter to a servo with the maintenance-safe sequence:
// for an EEPROM parameter, disable torque first (and confirm), write, then read
// back and compare. RAM parameters skip the torque dance. `verified` is set
// true only when the read-back equals the written value. Returns a result code:
// NotFound (no profile), Unsupported (no descriptor on this table), BusError
// (a transaction failed), VerifyFailed (mismatch), or Ok.
protocol::dxljob::Code writeParamChecked(uint8_t id, dxl::LogicalParam param,
                                         int32_t value, int32_t& readback,
                                         bool& verified) {
  using Code = protocol::dxljob::Code;
  verified = false;
  readback = 0;
  const dxl::ServoProfile* p = g_dxlBus.profileById(id);
  if (p == nullptr) return Code::NotFound;
  dxl::ParamDescriptor d;
  if (!dxl::paramDescriptor(p->table_kind, param, d) || !d.writable) {
    return Code::Unsupported;
  }
  if (d.region == dxl::ParamRegion::Eeprom) {
    // EEPROM writes are locked while torque is on: disable and confirm.
    if (!g_dxlBus.setTorqueOne(id, p->table_kind, false)) return Code::BusError;
    bool torque_on = true;
    if (!g_dxlBus.torqueState(id, p->table_kind, torque_on) || torque_on) {
      return Code::BusError;
    }
  }
  if (!g_dxlBus.writeRegister(id, p->table_kind, d.address, d.length, value)) {
    return Code::BusError;
  }
  if (!g_dxlBus.readRegister(id, p->table_kind, d.address, d.length,
                             d.is_signed, readback)) {
    return Code::BusError;
  }
  verified = (readback == value);
  return verified ? Code::Ok : Code::VerifyFailed;
}

// Execute one queued DXL maintenance job against the bus (dxlTask context only)
// and write the serialized result back to the queue. No-op when the queue is
// empty. Bus access requires DXL power on; when it is off the job is reported
// PowerOff rather than silently returning nothing.
void runQueuedDxlJob() {
  protocol::DxlJobRequest job;
  uint8_t job_id = 0;
  if (!g_dxlJobApi.queue().claim(job, job_id)) {
    return;  // nothing pending
  }

  uint8_t data[protocol::dxljob::kMaxResult];
  uint8_t len = 0;
  protocol::dxljob::Code code = protocol::dxljob::Code::Ok;

  const bool power_on = board::dxlPowerEnabled();
  if (!power_on && job.type != protocol::dxljob::Type::None) {
    g_dxlJobApi.queue().complete(job_id, protocol::dxljob::Code::PowerOff,
                                 nullptr, 0);
    return;
  }

  switch (job.type) {
    case protocol::dxljob::Type::Scan: {
      const uint8_t found = g_dxlBus.scan(job.arg0, job.arg1);
      g_servoStatusCount = 0;  // status snapshot is stale after a rescan
      // Clear the cached detail fields: indices now map to different servos, so
      // the round-robin readStatus must repopulate them from scratch (eax.6).
      for (uint8_t i = 0; i < dxl::DxlBus::kMaxServos; ++i) {
        g_servoStatus[i] = dxl::ServoStatus{};
      }
      data[len++] = found;
      for (uint8_t i = 0; i < found; ++i) {
        if (static_cast<uint8_t>(len + 6) > protocol::dxljob::kMaxResult) break;
        len = appendCompactServo(data, len, g_dxlBus.profile(i));
      }
      break;
    }
    case protocol::dxljob::Type::Ping: {
      dxl::ServoProfile p;
      if (g_dxlBus.ping(job.arg0, p)) {
        len = appendCompactServo(data, len, p);
        data[len++] = p.present ? 1 : 0;
      } else {
        code = protocol::dxljob::Code::NotFound;
      }
      break;
    }
    case protocol::dxljob::Type::Torque: {
      const bool on = (job.arg0 != 0);
      const uint8_t acked = g_dxlBus.setTorqueAll(on);
      data[len++] = on ? 1 : 0;
      data[len++] = acked;
      break;
    }
    case protocol::dxljob::Type::GetProfile: {
      dxl::ServoProfile p;
      if (g_dxlBus.ping(job.arg0, p)) {
        len = appendCompactServo(data, len, p);
        data[len++] = p.present ? 1 : 0;
        data[len++] = packServoCaps(p);
        data[len++] = p.torque_enabled ? 1 : 0;
        data[len++] = p.last_error;
      } else {
        code = protocol::dxljob::Code::NotFound;
      }
      break;
    }
    case protocol::dxljob::Type::GetParam: {
      // [param, table_kind, length, value(i32)].
      const dxl::ServoProfile* p = g_dxlBus.profileById(job.arg0);
      if (p == nullptr) {
        code = protocol::dxljob::Code::NotFound;
        break;
      }
      if (!dxl::isLogicalParam(job.param)) {
        code = protocol::dxljob::Code::Unsupported;
        break;
      }
      const dxl::LogicalParam param =
          static_cast<dxl::LogicalParam>(job.param);
      dxl::ParamDescriptor d;
      if (!dxl::paramDescriptor(p->table_kind, param, d)) {
        code = protocol::dxljob::Code::Unsupported;
        break;
      }
      int32_t value = 0;
      if (!g_dxlBus.readRegister(job.arg0, p->table_kind, d.address, d.length,
                                 d.is_signed, value)) {
        code = protocol::dxljob::Code::BusError;
        break;
      }
      data[len++] = job.param;
      data[len++] = static_cast<uint8_t>(p->table_kind);
      data[len++] = d.length;
      len = appendI32(data, len, value);
      break;
    }
    case protocol::dxljob::Type::SetParam: {
      // [param, length, written(i32), readback(i32), verified].
      if (!dxl::isLogicalParam(job.param)) {
        code = protocol::dxljob::Code::Unsupported;
        break;
      }
      const dxl::LogicalParam param =
          static_cast<dxl::LogicalParam>(job.param);
      int32_t readback = 0;
      bool verified = false;
      code = writeParamChecked(job.arg0, param, job.val_a, readback, verified);
      if (code == protocol::dxljob::Code::NotFound ||
          code == protocol::dxljob::Code::Unsupported) {
        break;  // nothing meaningful to serialize
      }
      // Ok and VerifyFailed (and BusError after a partial write) still report
      // the written/read values so the host sees what happened.
      const dxl::ServoProfile* p = g_dxlBus.profileById(job.arg0);
      dxl::ParamDescriptor d;
      const uint8_t plen = (p != nullptr &&
                            dxl::paramDescriptor(p->table_kind, param, d))
                               ? d.length
                               : 0;
      data[len++] = job.param;
      data[len++] = plen;
      len = appendI32(data, len, job.val_a);
      len = appendI32(data, len, readback);
      data[len++] = verified ? 1 : 0;
      break;
    }
    case protocol::dxljob::Type::SetLimits: {
      // [table_kind, min(i32), max(i32), verified].
      const dxl::ServoProfile* p = g_dxlBus.profileById(job.arg0);
      if (p == nullptr) {
        code = protocol::dxljob::Code::NotFound;
        break;
      }
      dxl::LogicalParam min_param, max_param;
      if (!dxl::servoLimitParams(p->table_kind, min_param, max_param)) {
        code = protocol::dxljob::Code::Unsupported;
        break;
      }
      int32_t rb_min = 0, rb_max = 0;
      bool v_min = false, v_max = false;
      const protocol::dxljob::Code c_min =
          writeParamChecked(job.arg0, min_param, job.val_a, rb_min, v_min);
      const protocol::dxljob::Code c_max =
          writeParamChecked(job.arg0, max_param, job.val_b, rb_max, v_max);
      // Surface the worst outcome: any bus error dominates, else a verify miss.
      if (c_min == protocol::dxljob::Code::BusError ||
          c_max == protocol::dxljob::Code::BusError) {
        code = protocol::dxljob::Code::BusError;
      } else if (!v_min || !v_max) {
        code = protocol::dxljob::Code::VerifyFailed;
      }
      data[len++] = static_cast<uint8_t>(p->table_kind);
      len = appendI32(data, len, rb_min);
      len = appendI32(data, len, rb_max);
      data[len++] = (v_min && v_max) ? 1 : 0;
      break;
    }
    case protocol::dxljob::Type::ReadReg: {
      // [addr(u16), len, value(i32)]. Raw read; protocol picked from profile.
      const dxl::ServoProfile* p = g_dxlBus.profileById(job.arg0);
      if (p == nullptr) {
        code = protocol::dxljob::Code::NotFound;
        break;
      }
      const uint16_t addr = static_cast<uint16_t>(job.val_a);
      int32_t value = 0;
      if (!g_dxlBus.readRegister(job.arg0, p->table_kind, addr, job.param,
                                 false, value)) {
        code = protocol::dxljob::Code::BusError;
        break;
      }
      data[len++] = static_cast<uint8_t>(addr & 0xFF);
      data[len++] = static_cast<uint8_t>((addr >> 8) & 0xFF);
      data[len++] = job.param;
      len = appendI32(data, len, value);
      break;
    }
    case protocol::dxljob::Type::WriteReg: {
      // [addr(u16), len, written(i32), readback(i32), verified]. flags bit0 =
      // EEPROM region -> disable torque before writing.
      const dxl::ServoProfile* p = g_dxlBus.profileById(job.arg0);
      if (p == nullptr) {
        code = protocol::dxljob::Code::NotFound;
        break;
      }
      const uint16_t addr = static_cast<uint16_t>(job.val_a);
      const bool is_eeprom = (job.arg1 & 0x01) != 0;
      bool ok = true;
      if (is_eeprom) {
        bool torque_on = true;
        if (!g_dxlBus.setTorqueOne(job.arg0, p->table_kind, false) ||
            !g_dxlBus.torqueState(job.arg0, p->table_kind, torque_on) ||
            torque_on) {
          ok = false;
        }
      }
      if (ok &&
          !g_dxlBus.writeRegister(job.arg0, p->table_kind, addr, job.param,
                                  job.val_b)) {
        ok = false;
      }
      int32_t readback = 0;
      if (ok && !g_dxlBus.readRegister(job.arg0, p->table_kind, addr, job.param,
                                       false, readback)) {
        ok = false;
      }
      const bool verified = ok && (readback == job.val_b);
      if (!ok) {
        code = protocol::dxljob::Code::BusError;
      } else if (!verified) {
        code = protocol::dxljob::Code::VerifyFailed;
      }
      data[len++] = static_cast<uint8_t>(addr & 0xFF);
      data[len++] = static_cast<uint8_t>((addr >> 8) & 0xFF);
      data[len++] = job.param;
      len = appendI32(data, len, job.val_b);
      len = appendI32(data, len, readback);
      data[len++] = verified ? 1 : 0;
      break;
    }
    default:
      code = protocol::dxljob::Code::Unsupported;
      break;
  }

  g_dxlJobApi.queue().complete(job_id, code, data, len);
}

void dxlTask(void*) {
  // Bring up the DXL UART once. This only initializes Serial1; it does NOT
  // enable DXL power (board HAL owns that) or servo torque, so it is safe at
  // boot. Scanning is deferred to a maintenance command once power is on.
  g_dxlBus.begin();
  TickType_t next = xTaskGetTickCount();
  bool prev_authorized = false;
  for (;;) {
    tick(watchdog::TaskId::Dxl);

    // Execute at most one queued DXL maintenance job per cycle. dxlTask is the
    // sole owner of the bus, so all scan/ping/torque/profile work funnels here;
    // apiTask only enqueues (gated on MacMaintenance + lock) and polls the
    // result. Running one job per loop keeps the present-position streaming
    // below responsive and bounds the per-cycle bus time.
    runQueuedDxlJob();

    // Enforce the safety gate at the bus level: the instant motion is no longer
    // permitted (RC kill, host estop, disarm, fault, or a non-motion state),
    // disable torque on all discovered servos so the robot stops. Edge-
    // triggered so we do not spam the bus every cycle. No-op until a maintenance
    // scan populates servos and DXL power is on (both OFF at boot), keeping this
    // safe by default. g_motionGate already folds in the safety state machine.
    const bool authorized = g_motionGate;
    const bool have_servos = g_dxlBus.servoCount() > 0;
    if (!authorized && prev_authorized && have_servos) {
      g_dxlBus.setTorqueAll(false);
    }
    // Rising edge of authorisation: enable torque on all servos before the first
    // goal Sync-Write below so they hold and track the commanded pose rather
    // than being back-driven. Edge-triggered to avoid re-issuing torque-enable
    // every cycle.
    if (authorized && !prev_authorized && have_servos) {
      g_dxlBus.setTorqueAll(true);
    }
    prev_authorized = authorized;
    // Publish torque-off confirmation for the safety FSM (passive pose gating).
    // Torque is off whenever no servo is driven: nothing discovered yet, or
    // motion is not authorised this cycle (the goal-write/torque-enable path is
    // gated by g_motionGate, so authorized==false => torque stays off).
    g_dxlTorqueOff = !have_servos || !authorized;

    // Goal Sync-Write path (lmt.2): while motion is authorised and torque is on,
    // push the latest goal frame published by controlTask (gait -> body IK ->
    // servo map). Copy the frame out under the brief mutex so the bus is never
    // held while the lock is taken; if controlTask is mid-publish we skip this
    // cycle and the servos hold their previous goal. Writing every cycle (not
    // just on goal change) also keeps any per-servo bus watchdog fed.
    if (authorized && have_servos && g_goalValid && g_goalMutex != nullptr) {
      static dxl::GoalTarget targets[config::kNumServos];
      uint8_t count = 0;
      if (xSemaphoreTake(g_goalMutex, 0) == pdTRUE) {
        count = g_goalFrame.count;
        for (uint8_t i = 0; i < count; ++i) {
          targets[i].id = g_goalFrame.joints[i].id;
          targets[i].tick = g_goalFrame.joints[i].tick;
        }
        xSemaphoreGive(g_goalMutex);
      }
      if (count > 0) {
        g_dxlBus.writeGoalPositions(targets, count);
      }
    }

    // Publish a fresh present-position snapshot for all discovered servos in a
    // single Sync Read per control table. This is a no-op until a maintenance
    // scan populates the servo table (DXL power is OFF at boot), and never
    // enables torque or writes goals. The goal Sync-Write path (writeGoal-
    // Positions) runs above, gated by g_motionGate + a valid goal frame.
    if (g_dxlBus.servoCount() > 0) {
      const uint8_t n =
          g_dxlBus.syncReadStatus(g_servoStatus, dxl::DxlBus::kMaxServos);
      g_servoStatusCount = n;

      // Round-robin one full per-servo read per cycle (eax.6). The all-servo
      // Sync Read above carries Present Position only (the MX(2.0) 23-byte
      // status block exceeds the DXL recv buffer), so the detail fields
      // (velocity, load, voltage, temperature, torque-enable, hardware error)
      // are gathered one servo at a time. syncReadStatus preserves these fields
      // between cycles, so the servo_status stream converges over servoCount()
      // cycles. Read-only and torque-off-safe; bounded to one servo per cycle.
      static uint8_t rr_servo = 0;
      const uint8_t cnt = g_dxlBus.servoCount();
      if (rr_servo >= cnt) rr_servo = 0;
      const uint8_t rr_id = g_dxlBus.profile(rr_servo).id;
      g_dxlBus.readStatus(rr_id, g_servoStatus[rr_servo]);
      g_servoStatus[rr_servo].id = rr_id;  // readStatus does not set id
      ++rr_servo;
    }
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kDxl));
  }
}

void rcTask(void*) {
  crsf::initRcStatus(g_rcStatus);
#if defined(PIN_SERIAL2_RX)
  // Serial2 is the ExpressLRS CRSF receiver link; rcTask owns it exclusively.
  Serial2.begin(kCrsfBaud);
#endif
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Rc);
    const uint32_t now_ms =
        static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
#if defined(PIN_SERIAL2_RX)
    crsf::ChannelData frame;
    while (Serial2.available() > 0) {
      const uint8_t b = static_cast<uint8_t>(Serial2.read());
      if (g_crsfParser.push(b, frame)) {
        crsf::applyFrame(g_rcStatus, frame, now_ms);
      }
    }
#endif
    // Raise failsafe if no valid RC frame has arrived within the timeout.
    crsf::evaluateFailsafe(g_rcStatus, now_ms);
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kRc));
  }
}

void apiTask(void*) {
  static protocol::FrameReader reader;
  static uint8_t out[protocol::kMaxWireFrame];
  static uint16_t g_telemetrySeq = 0;
  // Bind the active config shadow (defaults until a persisted config is
  // adopted; the shadow object is stable, so this pointer stays valid across
  // adopt/commit) so maintenance targets can run IK + the servo map.
  g_maintTargetApi.setConfig(&g_configApi.config());
  // Route CONTACT/LEVELING enable/disable through the shared feature surface so
  // there is one desired-feature set (AGENTS.md 1.3).
  g_sensorApi.setFeatureApi(&g_featureApi);
  // Give the sensor API read access to the topology / foot-status snapshots
  // i2cTask publishes so I2C_GET_TOPOLOGY / SENSOR_GET_STATUS can encode them
  // (apiTask is a reader only; i2cTask owns Wire).
  g_sensorApi.setSnapshots(&g_sensorTopoSnap, &g_sensorStatusSnap);
#ifdef HEXAPOD_ENABLE_DXL_RAW_REGISTER
  // Expert-only: raw DXL register read/write (DXL_READ_REGISTER /
  // DXL_WRITE_REGISTER). Off by default; build with this macro defined to
  // expose it. Still maintenance-locked and torque-off gated at execution.
  g_dxlJobApi.setRawRegisterEnabled(true);
#endif
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Api);

    // Adopt a persisted config once i2cTask has loaded one at boot. Done here
    // (not at task start) because i2cTask's scan may finish after this task
    // begins; the ConfigApi shadow is thus only ever touched by apiTask.
    if (g_bootLoad.ready && !g_bootLoad.consumed) {
      g_configApi.adoptPayload(g_bootLoad.payload, g_bootLoad.len);
      g_bootLoad.consumed = true;
    }

    // Drain any received bytes, framing them into complete request bodies.
    while (Serial.available() > 0) {
      const uint8_t b = static_cast<uint8_t>(Serial.read());
      if (!reader.push(b)) {
        continue;
      }

      // Refresh the live status snapshot for this request.
      protocol::api::StatusSnapshot st;
      st.uptime_ms =
          static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
      st.state = g_safetyState;
      st.dxl_power = board::dxlPowerEnabled();
      st.dxl_power_control = board::hasDxlPowerControl();
      st.battery_mv = board::readBatteryMilliVolts();
      st.watchdog_missed = watchdog::missedMask();

      // Give the maintenance lock the current time + live state so ENTER can
      // gate on a safe state and TTL/heartbeat use the same clock as the
      // control task.
      g_maintApi.setNow(st.uptime_ms);
      g_maintApi.setLiveState(g_safetyState);

      // Refresh the passive handler's live state so PASSIVE_ENTER gates on the
      // current safety state (control task also keeps this current each cycle).
      g_passiveApi.setLiveState(g_safetyState);

      const size_t n = protocol::api::handleRequest(
          reader.body(), reader.length(), g_deviceInfo, st, out, sizeof(out),
          &g_configApi, &g_subs, &g_controlApi, &g_motionApi, &g_maintApi,
          &g_maintTargetApi, &g_dxlJobApi, &g_featureApi, &g_sensorApi,
          &g_passiveApi);
      if (n > 0) {
        Serial.write(out, n);
      }
    }

    // Emit any due telemetry frames for subscribed streams. The subscription
    // manager enforces each stream's rate and counts missed slots as dropped;
    // when the USB CDC TX buffer cannot accept a frame we count a backlog drop
    // instead of blocking the task (AGENTS.md 6.3 rate-limited subscriptions).
    const uint32_t now_ms =
        static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
    for (uint8_t i = 0; i < protocol::kNumStreams; ++i) {
      const protocol::StreamId s = static_cast<protocol::StreamId>(i);
      if (!g_subs.shouldEmit(s, now_ms)) continue;
      uint8_t payload[protocol::kMaxPayload];
      const uint16_t plen = buildTelemetry(s, payload, now_ms);
      protocol::Header h;
      h.msg_type = static_cast<uint8_t>(protocol::MsgType::Telemetry);
      h.msg_id = static_cast<uint8_t>(protocol::kTelemetryFrameMsgBase + i);
      h.seq = g_telemetrySeq++;
      h.timestamp_ms = now_ms;
      h.payload_len = plen;
      const size_t fn = protocol::encodeFrame(h, payload, out, sizeof(out));
      if (fn == 0) continue;
      if (Serial.availableForWrite() >= static_cast<int>(fn)) {
        Serial.write(out, fn);
      } else {
        g_subs.noteTxBacklog();  // TX full: drop this frame, do not block
      }
    }

    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kApi));
  }
}

// Publish the discovered I2C topology into the portable snapshot the SensorApi
// reads for I2C_GET_TOPOLOGY. Called after each scan (boot + I2C_SCAN). Only
// i2cTask writes this snapshot.
void publishTopologySnapshot() {
  protocol::TopologySnapshot& s = g_sensorTopoSnap;
  s.mux_present = g_i2cTopology.mux_present ? 1 : 0;
  s.eeprom_present = g_i2cTopology.eeprom_present ? 1 : 0;
  s.num_channels = protocol::kSensorNumChannels;
  for (uint8_t i = 0; i < protocol::kSensorNumChannels; ++i) {
    const i2c::ChannelInfo& c = g_i2cTopology.channels[i];
    s.channels[i].scanned = c.scanned ? 1 : 0;
    s.channels[i].vcnl_present = c.vcnl_present ? 1 : 0;
    s.channels[i].lps_present = c.lps_present ? 1 : 0;
    s.channels[i].device_count = c.device_count;
    s.channels[i].state = static_cast<uint8_t>(c.state);
  }
  s.valid = 1;
}

// Publish the fused per-foot contact state into the snapshot the SensorApi
// reads for SENSOR_GET_STATUS. Called each i2cTask pass after the foot copy.
void publishStatusSnapshot() {
  protocol::StatusSnapshot& s = g_sensorStatusSnap;
  s.num_feet = sensors::kNumFeet;
  s.present_mask = g_footPresentMask;
  s.polling_enabled = g_sensorPollingEnabled ? 1 : 0;
  for (uint8_t i = 0; i < sensors::kNumFeet; ++i) {
    const sensors::LegContactState& f = g_footState[i];
    s.feet[i].state = static_cast<uint8_t>(f.state);
    s.feet[i].confidence = f.confidence;
    s.feet[i].proximity = f.proximity_raw;
    s.feet[i].pressure_delta = static_cast<int16_t>(f.pressure_delta);
    uint8_t flags = 0;
    if (f.near_surface) flags |= 0x01;
    if (f.touch) flags |= 0x02;
    if (f.loaded) flags |= 0x04;
    if (f.release) flags |= 0x08;
    if (f.stale) flags |= 0x10;
    if (f.fault) flags |= 0x20;
    s.feet[i].flags = flags;
  }
  s.valid = 1;
}

void i2cTask(void*) {
  // Bring up the root I2C bus and run a one-time discovery scan so capabilities
  // (mux/EEPROM presence, per-channel foot sensors) are known early. Running it
  // here keeps the blocking probe work off the control loop.
  g_i2cBus.begin();
  g_i2cBus.scanAll(g_i2cTopology);
  g_footPresentMask = i2c::footSensorPresentMask(g_i2cTopology);
  publishTopologySnapshot();

  // Seed the contact estimator with the compiled-default foot calibration so it
  // is usable immediately (per-foot classification stays disabled until a
  // calibration enables it; raw values still stream as telemetry).
  {
    config::RobotConfig boot_cfg;
    config::defaultRobotConfig(boot_cfg);
    sensors::ContactParams params;  // conservative defaults
    g_contact.configure(boot_cfg.feet, params);
    // Seed the motion intent with the compiled-default gait parameters so the
    // first SET_BODY_TWIST has a sane baseline (host SET_GAIT_PARAMS overrides).
    g_motionApi.setDefaults(boot_cfg.gait.gait, boot_cfg.gait.body_height_mm,
                            boot_cfg.gait.stride_len_mm,
                            boot_cfg.gait.step_height_mm, boot_cfg.gait.duty_x255,
                            boot_cfg.gait.speed_x255);
  }
  // If the config EEPROM is present and holds a valid slot, load it and hand it
  // to apiTask to adopt as the active config. Otherwise stay volatile so the
  // firmware runs on compiled defaults and CFG_COMMIT is rejected (AGENTS.md
  // 4.3).
  if (g_i2cTopology.eeprom_present) {
    config::SlotStatus slots[config::kSlotCount];
    g_configStore.inspect(slots);
    if (slots[0].valid || slots[1].valid) {
      uint16_t n = 0;
      if (g_configStore.load(g_bootLoad.payload, sizeof(g_bootLoad.payload),
                             n)) {
        g_bootLoad.len = n;
        g_bootLoad.ready = true;
        g_configVolatile = false;
      } else {
        g_configVolatile = true;
      }
    } else {
      g_configVolatile = true;
    }
  } else {
    g_configVolatile = true;
  }
  // Boot discovery and config seeding are complete: a config (persisted or
  // compiled default) is now available, so the safety machine may leave
  // ConfigLoad. apiTask adopts any persisted payload independently.
  g_configReady = true;
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::I2c);

    // Service a config commit handed over by apiTask. i2cTask is the sole owner
    // of Wire/EEPROM, so the transactional store write happens here.
    bool do_commit = false;
    if (g_commitMutex != nullptr) {
      xSemaphoreTake(g_commitMutex, portMAX_DELAY);
      do_commit = g_commit.requested;
      xSemaphoreGive(g_commitMutex);
    }
    if (do_commit) {
      const bool ok = g_configStore.commit(g_commit.payload, g_commit.len);
      xSemaphoreTake(g_commitMutex, portMAX_DELAY);
      g_commit.ok = ok;
      g_commit.requested = false;
      xSemaphoreGive(g_commitMutex);
      if (ok) g_configVolatile = false;  // a valid slot now exists
      xSemaphoreGive(g_commitDone);
    }

    // Apply host-staged per-foot contact thresholds (CONTACT_SET_THRESHOLDS).
    // apiTask only stages them (it cannot touch the estimator); i2cTask owns the
    // estimator, so it applies the latest set whenever the sequence advances.
    static uint32_t applied_threshold_seq = 0;
    const uint32_t want_seq = g_sensorApi.thresholdSeq();
    if (want_seq != applied_threshold_seq) {
      const protocol::ContactThresholds& t = g_sensorApi.thresholds();
      for (uint8_t i = 0; i < sensors::kNumFeet; ++i) {
        g_contact.setThresholds(i, t.near_thresh[i], t.touch_thresh[i],
                                t.load_thresh[i]);
      }
      applied_threshold_seq = want_seq;
    }

    // Round-robin foot-sensor polling state (declared before the scan/calibrate
    // service blocks so a re-scan can force a re-power of rediscovered boards).
    static uint8_t poll_ch = 0;
    static uint8_t configured_mask = 0;

    // Service a host-requested I2C re-scan (I2C_SCAN). i2cTask is the sole Wire
    // owner, so the blocking probe runs here; the host polls I2C_GET_TOPOLOGY
    // for the refreshed result. selectNone() first so the mux is in a known
    // state before scanning the root bus + channels.
    static uint32_t applied_scan_seq = 0;
    const uint32_t want_scan = g_sensorApi.scanSeq();
    if (want_scan != applied_scan_seq) {
      g_i2cBus.selectNone();
      g_i2cBus.scanAll(g_i2cTopology);
      g_footPresentMask = i2c::footSensorPresentMask(g_i2cTopology);
      configured_mask = 0;  // force re-power of any (re)discovered boards
      publishTopologySnapshot();
      applied_scan_seq = want_scan;
    }

    // Service a host-requested baseline capture (CONTACT_CALIBRATE /
    // SENSOR_CALIBRATE). The estimator (owned here) re-zeroes the per-foot
    // pressure baseline to the latest reading for each requested foot.
    static uint32_t applied_calibrate_seq = 0;
    const uint32_t want_cal = g_sensorApi.calibrateSeq();
    if (want_cal != applied_calibrate_seq) {
      const uint8_t mask = g_sensorApi.calibrateMask();
      for (uint8_t i = 0; i < sensors::kNumFeet; ++i) {
        if ((mask & static_cast<uint8_t>(1u << i)) != 0) {
          g_contact.captureBaseline(i);
        }
      }
      applied_calibrate_seq = want_cal;
    }

    // Poll one foot sensor per iteration (round-robin) so each pass does bounded
    // Wire work and the control loop is never stalled by a slow/missing board
    // (AGENTS.md 1.1 / 5.4). The mux requires exclusive one-hot channel select;
    // we select, read, then deselect so the root bus (EEPROM) stays addressable.
    if (g_sensorPollingEnabled && g_i2cTopology.mux_present) {
      const uint8_t ch = poll_ch;
      poll_ch = static_cast<uint8_t>((poll_ch + 1) % i2c::kNumFootChannels);
      const bool present =
          (g_footPresentMask & static_cast<uint8_t>(1u << ch)) != 0;
      if (present && g_i2cBus.selectChannel(ch)) {
        const uint8_t bit = static_cast<uint8_t>(1u << ch);
        if ((configured_mask & bit) == 0) {
          // First time we touch this board: power up its sensors.
          if (g_finger.configureChannel()) {
            configured_mask = static_cast<uint8_t>(configured_mask | bit);
          }
        }
        const sensors::FootSample sample = g_finger.readFoot();
        if (!sample.ok) {
          // Force reconfigure on next visit in case the board was reseated.
          configured_mask = static_cast<uint8_t>(configured_mask & ~bit);
        }
        g_contact.update(ch, sample, millis());
        g_i2cBus.selectNone();
      }
    }
    // Decay any silent foot to STALE and republish the snapshot every pass.
    g_contact.tickStaleness(millis());
    for (uint8_t i = 0; i < sensors::kNumFeet; ++i) {
      g_footState[i] = g_contact.foot(i);
    }
    // Mirror the fused foot state into the SensorApi snapshot (SENSOR_GET_STATUS).
    publishStatusSnapshot();

    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kI2c));
  }
}

// Minimal FreeRTOS liveness test: toggle the USER LED every 100 ms. If the LED
// blinks at 5 Hz the scheduler is running and ticking correctly. This owns the
// LED so no other task should write it. Not part of the watchdog set.
void blinkTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    board::toggleUserLed();
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kBlink));
  }
}

void healthTask(void*) {
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    tick(watchdog::TaskId::Health);

    // Evaluate the software watchdog over the elapsed window. (The USER LED is
    // driven by blinkTask as the FreeRTOS liveness indicator. USB CDC is owned
    // by the api task per AGENTS.md 5.1, so the host reads health via the
    // GET_STATUS command rather than text printed here.)
    watchdog::evaluate();

    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kHealth));
  }
}

}  // namespace

void start() {
  watchdog::init();
  initDeviceInfo();

  // Sync primitives for the apiTask <-> i2cTask config commit hand-off. Created
  // here at boot (before the scheduler), not at runtime.
  g_commitMutex = xSemaphoreCreateMutex();
  g_commitDone = xSemaphoreCreateBinary();

  // Guards the controlTask -> dxlTask servo goal frame (lmt.1).
  g_goalMutex = xSemaphoreCreateMutex();

  // Route FreeRTOS fault reporting to the USER LED and USB CDC.
  vSetErrorLed(board::pinUserLed(), HIGH);
  vSetErrorSerial(&Serial);

  // Task stacks are allocated once here, at boot, before the scheduler runs;
  // there is no runtime heap churn afterward (AGENTS.md 1.2).
  xTaskCreate(controlTask, "control", stack_words::kControl, nullptr,
              priority::kControl, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Control)]);
  xTaskCreate(dxlTask, "dxl", stack_words::kDxl, nullptr,
              priority::kDxl, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Dxl)]);
  xTaskCreate(rcTask, "rc", stack_words::kRc, nullptr,
              priority::kRc, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Rc)]);
  xTaskCreate(apiTask, "api", stack_words::kApi, nullptr,
              priority::kApi, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Api)]);
  xTaskCreate(i2cTask, "i2c", stack_words::kI2c, nullptr,
              priority::kI2c, &g_handles[static_cast<uint8_t>(watchdog::TaskId::I2c)]);
  xTaskCreate(healthTask, "health", stack_words::kHealth, nullptr,
              priority::kHealth, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Health)]);

  // Standalone FreeRTOS liveness test task (USER LED @ 5 Hz). Not tracked by
  // the watchdog; handle discarded.
  xTaskCreate(blinkTask, "blink", stack_words::kBlink, nullptr,
              priority::kBlink, nullptr);

  // Hands the CPU to the scheduler; does not return.
  vTaskStartScheduler();
}

}  // namespace app
