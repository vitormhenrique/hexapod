#include "tasks.h"

#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#include <math.h>

#include "../board/board.h"
#include "../config/config_bootstrap.h"
#include "../config/config_api.h"
#include "../config/qwiic_openlog.h"
#include "../controller/controller_core.h"
#include "../dxl/dxl_bus.h"
#include "../dxl/dxl_fault_monitor.h"
#include "../dxl/hold_targets.h"
#include "../dxl/dxl_params.h"
#include "../dxl/scan_cursor.h"
#include "../dxl/servo_map.h"
#include "../gait/gait_pipeline.h"
#include "../gait/trick_engine.h"
#include "../hil/observer_api.h"
#include "../hil/output_guard.h"
#if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
#include "../hil/trace_codec.h"
#include "../hil/trace_recorder.h"
#endif
#include "../input/controller_bridge.h"
#include "../input/crsf_parser.h"
#include "../input/crsf_telemetry.h"
#include "../input/rc_frame_snapshot.h"
#include "../logging/capture_log.h"
#include "../protocol/api.h"
#include "../protocol/control_api.h"
#include "../protocol/controller_api.h"
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
#include "../protocol/telemetry_encode.h"
#include "../sensors/bno085.h"
#include "../sensors/contact_estimator.h"
#include "../sensors/finger_sensor.h"
#include "../sensors/i2c_bus.h"
#include "../sensors/qwiic_debug_oled.h"
#include "../safety/command_arbiter.h"
#include "../safety/error_journal.h"
#include "../safety/event_log.h"
#include "../safety/fault_capture.h"
#include "../safety/state_machine.h"
#include "../safety/system_state.h"
#include "../safety/watchdog.h"
#include "task_config.h"
#include "controller_time_adapter.h"
#include "status_led.h"

namespace app {
namespace {

constexpr size_t kHeapAlignmentBytes = 8;
constexpr size_t kHeapBlockHeaderBytes = 8;
constexpr size_t kHeapReserveBytes = 512;

constexpr size_t heapBlockBytes(size_t requested) {
  return (requested + kHeapBlockHeaderBytes + kHeapAlignmentBytes - 1u) &
     ~(kHeapAlignmentBytes - 1u);
}

constexpr size_t taskHeapBytes(uint16_t stack_words) {
  return heapBlockBytes(static_cast<size_t>(stack_words) * sizeof(StackType_t)) +
     heapBlockBytes(sizeof(StaticTask_t));
}

constexpr size_t kStartupTaskHeapBytes =
  taskHeapBytes(stack_words::kControl) + taskHeapBytes(stack_words::kDxl) +
  taskHeapBytes(stack_words::kRc) + taskHeapBytes(stack_words::kApi) +
  taskHeapBytes(stack_words::kI2c) + taskHeapBytes(stack_words::kHealth) +
  taskHeapBytes(configMINIMAL_STACK_SIZE);
constexpr size_t kStartupSemaphoreHeapBytes =
  3u * heapBlockBytes(sizeof(StaticSemaphore_t));

static_assert(priority::kHealth < configMAX_PRIORITIES,
        "FreeRTOS priority table must include healthTask");
static_assert(configUSE_TIMERS == 0,
        "unused FreeRTOS timer daemon must stay disabled");
static_assert(configTOTAL_HEAP_SIZE >=
          kStartupTaskHeapBytes + kStartupSemaphoreHeapBytes +
            kHeapReserveBytes + (kHeapAlignmentBytes - 1u),
        "FreeRTOS heap no longer covers boot allocations plus reserve");

// Task handles, indexed by watchdog::TaskId, so the health task can read each
// task's stack high-water mark.
TaskHandle_t g_handles[watchdog::kTaskCount] = {nullptr};

// Static description of this firmware build, reported by HELLO/GET_CAPABILITIES.
protocol::api::DeviceInfo g_deviceInfo;
uint8_t g_resetCause = 0;

// Single owner of the DYNAMIXEL TTL bus (Serial1). Only dxlTask touches this,
// satisfying the AGENTS.md rule that one task owns Dynamixel2Arduino/Serial1.
dxl::DxlBus g_dxlBus(Serial1);

// Latest per-servo status snapshot, published by dxlTask via Sync Read and read
// by telemetry/safety consumers. Single writer (dxlTask); readers take a copy.
dxl::ServoStatus g_servoStatus[dxl::DxlBus::kMaxServos];
volatile uint8_t g_servoStatusCount = 0;
volatile bool g_configuredServoCoverage = false;
volatile uint32_t g_poseKnownMask = 0;
volatile uint32_t g_servoReadinessConfigRev = 0;
constexpr uint32_t kAllServoPosesKnown =
  (1u << config::kNumServos) - 1u;
// CRSF/ExpressLRS RC input state. Owned exclusively by rcTask (Serial3).
crsf::Parser g_crsfParser;
crsf::RcStatus g_rcStatus;

// Raw RC diagnostics snapshot (a8n): the CRSF layer the parsed g_rcStatus hides
// -- raw 11-bit ticks, frame-health counters, and decoded LINK_STATISTICS. Sole
// writer is rcTask; telemetry reads a copy for the rc_diagnostics stream and the
// companion RC troubleshooting page. Plain aggregate so {} value-initializes it
// to a safe zero state (no link seen, ticks 0).
struct RcDiagSnapshot {
  uint16_t raw_ticks[crsf::kNumChannels];
  uint32_t frames_decoded;
  uint32_t crc_errors;
  uint32_t link_stats_count;
  uint32_t last_frame_ms;
  bool ever_seen;
  bool failsafe;
  bool has_link_stats;
  crsf::LinkStatistics link_stats;
};
RcDiagSnapshot g_rcDiag{};

// Controller bridge (oha.2/oha.3): decodes the ChannelPack CRSF frame into a
// high-level ControllerCommand (modes, twist, body pose, gait, shape, tricks,
// arm/estop, feature requests). rcTask is the sole writer; controlTask reads a
// copy of the published snapshot each cycle. The derived RcStatus above feeds
// the existing safety FSM / arbiter / telemetry plumbing unchanged.
controller::ControllerBridge g_bridge;
controller::ControllerCommand g_ctrlCmd;
controller::RcFrameMailbox g_rcMailbox;
controller::RcFrameSnapshot g_controlRcSnapshot;
bool g_controlRcSnapshotValid = false;
// ConfigApi is owned by apiTask, while ControllerBridge is owned by rcTask.
// controlTask already consumes a validated, revisioned config snapshot; it
// publishes this small calibration block to rcTask so the bridge never reads a
// partially replaced RobotConfig directly.
config::RcInputCalibration g_pendingRcCalibration;
volatile uint32_t g_pendingRcCalibrationRevision = 0;
volatile bool g_pendingRcCalibrationValid = false;

volatile uint8_t g_commandSource = 0;       // safety::CommandSource value
volatile bool g_motionAuthorized = false;   // a source may drive servos
volatile bool g_killActive = true;          // RC kill / host estop asserted

// Only controlTask advances this adapter clock. It translates the FreeRTOS
// scheduler timestamp into the portable ControllerTime contract without making
// the future ControllerCore depend on FreeRTOS.
controller::ControllerClock g_controlClock(period_ms::kControl);
volatile uint8_t g_safetyState = static_cast<uint8_t>(safety::State::Boot);
volatile uint8_t g_faultReason = 0;  // safety::FaultReason value
volatile uint8_t g_lastFaultReason = 0;  // safety::FaultReason value
volatile uint32_t g_lastFaultTimestampMs = 0;
// True only when the current state permits motion AND a source owns authority.
volatile bool g_motionGate = false;
// True once i2cTask has finished its boot scan and seeded a (persisted or
// default) config; gates the ConfigLoad -> Disarmed transition.
volatile bool g_configReady = false;

// Battery voltage snapshot, published by controlTask each cycle. controlTask is
// the sole analogRead() caller: the SAMD21 core's analogRead busy-waits
// unbounded on RESRDY and disables the ADC on completion, so a second task
// reading concurrently can be preempted mid-conversion and spin forever on a
// flag that never sets (observed on hardware: apiTask wedged at priority 2,
// starving healthTask's WDT pet -> hardware reset ~2 s after every host
// request). All other tasks must read this snapshot, never the ADC.
volatile uint16_t g_batteryMv = 0;
volatile bool g_batteryValid = false;
bool g_batterySampled = false;
uint32_t g_lastBatterySampleMs = 0;
constexpr uint32_t kBatterySamplePeriodMs = 100;

// Telemetry subscription manager. apiTask routes the telemetry command range to
// it and walks the streams each loop, emitting due telemetry frames over USB.
protocol::SubscriptionManager g_subs;

// USB frame reader + rx-health counters (hexapod_src-lv6). Owned exclusively by
// apiTask (which also builds api_stats telemetry, so no cross-task access):
// g_frameReader counts complete frame bodies (framesOk) and overflow-dropped
// frames; g_apiRxBad counts bodies that failed COBS/CRC/magic/length decode.
protocol::FrameReader g_frameReader;
uint32_t g_apiRxBad = 0;

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
  gait::PipelineLegTarget legs[config::kNumLegs];
  logging::AppliedMotionCapture applied;
};
GoalFrame g_goalFrame;
SemaphoreHandle_t g_goalMutex = nullptr;  // guards g_goalFrame
volatile bool g_goalValid = false;        // a fresh, gated goal frame is ready
volatile uint32_t g_goalSeq = 0;          // bumped on each published frame
// Max time dxlTask blocks to acquire g_goalMutex before falling back to the
// previously latched goal. controlTask (100 Hz, equal priority) holds the mutex
// only for a small fixed copy, so this bound is never meaningfully consumed; it
// exists purely to break the equal-priority zero-wait phase-lock that starved
// the reader for frames of >=16 joints (see dxlTask goal-write path).
constexpr uint32_t kGoalReadWaitMs = 4;
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

// A DXL scan keeps the maintenance queue slot Running while it advances one
// ping per dxlTask iteration. The queue remains single-slot, but absent IDs no
// longer turn one host request into a watchdog-starving busy-wait burst.
struct DxlScanJobState {
  bool active = false;
  uint8_t job_id = 0;
  dxl::ScanCursor cursor;
};
DxlScanJobState g_dxlScanJob;

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

// Host controller command surface (CONTROLLER_GET_STATE/GET_BINDINGS/
// SET_BINDINGS, oha.4). apiTask refreshes its reported snapshot (decoded
// command + raw inputs + active bindings) before each request and, after a
// SET_BINDINGS, hands the staged BindingConfig to rcTask via the pending
// hand-off below (rcTask owns the ControllerBridge, so apiTask never touches
// it directly). Also feeds the controller_state telemetry stream.
protocol::ControllerApi g_controllerApi;

// HIL observer state exists only in the output-disabled image. The normal
// image answers the reserved command range statelessly with NotAvailable.
#if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
hil::ObserverApi g_hilObserver;
#endif

#if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
// The recorder has one fixed producer/consumer record handoff. This separate
// ring carries only tiny capture-control messages from apiTask to controlTask,
// so a serial request never mutates recorder state directly.
enum class HilTraceRequestType : uint8_t { Capture, Abort, Marker };

struct HilTraceRequest {
  HilTraceRequestType type = HilTraceRequestType::Capture;
  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  uint32_t value = 0;
};

class HilTraceRequestRing {
 public:
  static constexpr uint8_t kCapacity = 4;

  bool hasSpace() const {
    taskENTER_CRITICAL();
    const bool available = count_ < kCapacity;
    taskEXIT_CRITICAL();
    return available;
  }

  bool push(const HilTraceRequest& request) {
    taskENTER_CRITICAL();
    if (count_ >= kCapacity) {
      taskEXIT_CRITICAL();
      return false;
    }
    entries_[write_index_] = request;
    write_index_ = static_cast<uint8_t>((write_index_ + 1u) % kCapacity);
    ++count_;
    taskEXIT_CRITICAL();
    return true;
  }

  bool pop(HilTraceRequest* request) {
    if (request == nullptr) return false;
    taskENTER_CRITICAL();
    if (count_ == 0) {
      taskEXIT_CRITICAL();
      return false;
    }
    *request = entries_[read_index_];
    read_index_ = static_cast<uint8_t>((read_index_ + 1u) % kCapacity);
    --count_;
    taskEXIT_CRITICAL();
    return true;
  }

 private:
  HilTraceRequest entries_[kCapacity] = {};
  uint8_t read_index_ = 0;
  uint8_t write_index_ = 0;
  uint8_t count_ = 0;
};

hil::trace::TraceRecorder g_hilTrace;
HilTraceRequestRing g_hilTraceRequests;
uint16_t g_hilTraceEventSeq = 0;
#endif

// Published controller snapshots (single writer rcTask, reader apiTask). Raw
// decoded ChannelPack inputs + the bridge's active binding map, copied each RC
// cycle so apiTask can report them without touching the bridge.
ChannelPackInputs_t g_ctrlRaw{};
controller::BindingConfig g_ctrlBindings;

// Lock-free bindings hand-off: apiTask stages a validated BindingConfig and
// sets the flag; rcTask adopts it into the bridge and clears the flag. Single
// producer (apiTask) / single consumer (rcTask); a SET_BINDINGS is rare and
// human-driven, and the bridge re-clamps all motion regardless, so a one-frame
// torn read is benign and self-corrected by the next adoption.
controller::BindingConfig g_ctrlPendingBindings;
volatile bool g_ctrlPendingBindingsValid = false;

// Torque-off confirmation published by dxlTask. The control task feeds this to
// the safety FSM as StateInputs.torque_off so PassivePoseStream is only entered
// when no servo is driven. Starts true (DXL power + torque are OFF at boot) and
// is set true again after dxlTask disables torque; the goal-write/torque-enable
// path (when wired) is the only place that clears it.
volatile bool g_dxlTorqueOff = true;
// Per-discovered-servo torque-off choices made through the maintenance logical
// parameter API. A released joint must stay limp even while maintenance goal
// frames continue and another torque-on path runs. dxlTask owns this mask.
uint32_t g_maintenanceReleasedMask = 0;

bool maintenanceServoReleased(uint8_t id) {
  for (uint8_t i = 0; i < g_dxlBus.servoCount(); ++i) {
    if (g_dxlBus.profile(i).id == id) {
      return (g_maintenanceReleasedMask & (uint32_t{1} << i)) != 0;
    }
  }
  return false;
}

// Hard DXL fault published by dxlTask (lmt.5). Set when a servo reports a
// hardware error bit (MX 2.0 Hardware Error Status) or the bus stops answering
// the present-status Sync Read for a sustained window. The control task feeds
// this to the safety FSM as StateInputs.dxl_hard_fault, which latches FaultHard
// until an operator CLEAR_FAULT and the underlying condition both clear. Starts
// false; only meaningful once a maintenance scan has populated the servo table.
volatile bool g_dxlHardFault = false;

// CRSF runs at 420000 baud on the OpenRB-150 D14 TX / D13 RX UART (Serial3).
constexpr uint32_t kCrsfBaud = 420000;
constexpr uint32_t kCrsfStatusPeriodMs = 200;    // 5 Hz
// While the operator is editing gait parameters from the handset the status
// frame is the feedback loop for the edit, so it is sent as fast as the RC
// cycle allows. rcTask still emits at most one telemetry frame per 10 ms cycle
// and still checks UART TX capacity first, so this can never starve RC parsing.
constexpr uint32_t kCrsfStatusBoostPeriodMs = 50;  // 20 Hz
// How long a boost lasts after the last gait-tune edit.
constexpr uint32_t kCrsfStatusBoostHoldMs = 3000;
constexpr uint32_t kCrsfAttitudePeriodMs = 100;  // 10 Hz
constexpr uint32_t kCrsfBatteryPeriodMs = 500;   // 2 Hz
constexpr uint32_t kImuFreshMs = 250;

// controlTask publishes this complete operator-facing snapshot; rcTask copies
// it atomically before adding i2cTask's IMU status and serializing a frame.
crsf::telemetry::HexapodStatus g_radioStatus;
// Set by controlTask while the handset gait-tune editor is engaged so rcTask
// raises the status-frame rate for the duration of the edit session.
volatile bool g_radioStatusBoost = false;

// Deduplicated firmware error journal (hexapod_src error reporting). Producers
// live in several tasks, so every access is wrapped in a short critical section
// by noteError()/takeError(); note() is bounded to a 12-entry table scan.
safety::ErrorJournal g_errorJournal;
safety::PersistentEventQueue g_persistentEvents;

void noteError(safety::ErrorCode code, uint8_t detail,
               safety::ErrorSeverity severity, uint32_t now_ms) {
  taskENTER_CRITICAL();
  const bool announced = g_errorJournal.note(code, detail, severity, now_ms);
  if (announced && severity != safety::ErrorSeverity::Info) {
    safety::PersistentEvent event;
    event.timestamp_ms = now_ms;
    event.code = code;
    event.detail = detail;
    event.severity = severity;
    g_persistentEvents.push(event);
  }
  taskEXIT_CRITICAL();
}


// Single owner of the root I2C bus (SERCOM0): mux, OpenLog, OLED, and muxed foot
// sensors. Only i2cTask touches this. Topology is the boot-scan
// result describing which optional devices were found.
i2c::I2cBus g_i2cBus;
i2c::I2cTopology g_i2cTopology;
sensors::Bno085 g_bno085(g_i2cBus);

struct ImuSnapshot {
  int16_t pitch_cdeg = 0;
  int16_t roll_cdeg = 0;
  int16_t yaw_cdeg = 0;
  uint8_t calibration = 0;
  bool present = false;
  bool valid = false;
  uint32_t sample_ms = 0;
};
ImuSnapshot g_imuSnapshot;

// Foot contact pipeline. The reader does bounded I2C/mux register I/O,
// the estimator runs the portable contact state machine, and the published
// snapshot is read by telemetry/safety consumers. All owned by i2cTask.
sensors::FingerSensorReader g_finger(g_i2cBus);
sensors::ContactEstimator g_contact;
sensors::LegContactState g_footState[sensors::kNumFeet];
volatile uint8_t g_footPresentMask = 0;
// Runtime sensor-polling toggle (feature.foot_sensor polling). Defaults on so
// present boards stream raw proximity/pressure; contact classification stays
// disabled per-foot until calibrated (FootSensorCal.enabled).
volatile bool g_sensorPollingEnabled = true;
volatile uint32_t g_i2cLastUpdateMs = 0;
constexpr uint32_t kI2cStaleMs = 500;

// CONFIG.TXT is a hex-encoded append journal containing the complete serialized
// RobotConfig. EVENTS.LOG is reserved for plain-text runtime diagnostics.
config::QwiicOpenLog g_openlog(g_i2cBus);
config::QwiicConfigFile g_configFile(g_openlog);
config::ConfigStore g_configStore(g_configFile);
bool g_configStorageAvailable = false;
bool g_configVolatile = true;
bool g_retainedCrashLogPending = false;
bool g_watchdogResetLogPending = false;
bool g_configBootstrapPending = false;
bool g_oledInitPending = false;
sensors::QwiicDebugOled g_debugOled(g_i2cBus);
sensors::DebugDisplayState g_debugDisplayState;

struct CaptureRuntime {
  bool recording = false;
  uint32_t handled_toggle_seq = 0;
  uint32_t handled_start_seq = 0;
  uint32_t session = 0;
  uint32_t sample = 0;
  uint32_t completed_samples = 0;
  uint32_t sample_ms = 0;
  uint32_t sample_file_size = 0;
  uint16_t sample_bytes = 0;
  uint32_t next_sample_ms = 0;
  uint8_t row = 0xFF;
  uint8_t servo_count = 0;
  uint8_t servo_index = 0;
  uint8_t goal_count = 0;
  uint8_t goal_index = 0;
  bool goal_valid = false;
  logging::RemoteCapture remote;
  logging::AppliedMotionCapture motion;
  logging::GoalCapture goals[config::kNumServos];
  gait::PipelineLegTarget legs[config::kNumLegs];
  dxl::ServoStatus servos[config::kNumServos];
  int16_t present_angle_centideg[config::kNumServos];
};

CaptureRuntime g_capture;
char g_captureLine[256];
constexpr uint32_t kCaptureIntervalMs = 500;
constexpr uint8_t kCaptureNoRow = 0xFF;
constexpr uint8_t kCaptureRowRemote = 0;
constexpr uint8_t kCaptureRowControl = 1;
constexpr uint8_t kCaptureRowLegs = 2;
constexpr uint8_t kCaptureRowGoals = 3;
constexpr uint8_t kCaptureRowServos = 4;
constexpr uint8_t kCaptureGoalsPerRow = 6;
constexpr const char* kCaptureFile = "CAPTURE.CSV";

bool openLogFileGrew(const char* file_name, uint32_t size_before,
                     uint32_t expected_growth) {
  uint32_t size_after = 0;
  bool exists_after = false;
  return g_openlog.fileSize(file_name, size_after, exists_after) &&
         exists_after && size_after >= size_before + expected_growth;
}

bool appendOpenLogLine(const char* file_name, const char* line, size_t len,
                       bool sync) {
  uint32_t size_before = 0;
  bool existed_before = false;
  if (!g_openlog.fileSize(file_name, size_before, existed_before)) return false;
  size_t offset = 0;
  while (offset < len) {
    const size_t remaining = len - offset;
    const uint8_t count = remaining < config::QwiicOpenLog::kMaxWriteBytes
                              ? static_cast<uint8_t>(remaining)
                              : config::QwiicOpenLog::kMaxWriteBytes;
    if (!g_openlog.append(file_name,
                          reinterpret_cast<const uint8_t*>(line + offset),
                          count)) {
      return false;
    }
    offset += count;
  }
  return !sync ||
         (g_openlog.sync() && openLogFileGrew(file_name, size_before,
                                               static_cast<uint32_t>(len)));
}

bool appendEventLogLine(const char* line, size_t len) {
  return appendOpenLogLine("EVENTS.LOG", line, len, true);
}

bool tryPersistRetainedCrash() {
  if (!g_retainedCrashLogPending) return true;
  const size_t length = safety::formatCrashLog(
      fault_capture::lastSnapshot(), g_captureLine, sizeof(g_captureLine));
  if (length == 0) {
    g_retainedCrashLogPending = false;
    return true;
  }
  uint32_t size_before = 0;
  bool existed_before = false;
  if (!g_openlog.fileSize("EVENTS.LOG", size_before, existed_before)) {
    return false;
  }
  if (!appendEventLogLine(g_captureLine, length)) return false;
  uint32_t size_after = 0;
  bool exists_after = false;
  if (!g_openlog.fileSize("EVENTS.LOG", size_after, exists_after) ||
      !exists_after || size_after < size_before + length) {
    return false;
  }
  fault_capture::acknowledgePersisted();
  g_retainedCrashLogPending = false;
  return true;
}

bool tryPersistWatchdogReset() {
  if (!g_watchdogResetLogPending) return true;
  const size_t length = safety::formatWatchdogResetLog(
      watchdog::lastResetMissedMask(), watchdog::lastResetProgressMarker(),
      watchdog::lastResetControlProgress(), watchdog::lastResetSafetyState(),
      g_captureLine, sizeof(g_captureLine));
  uint32_t size_before = 0;
  bool existed_before = false;
  if (length == 0 ||
      !g_openlog.fileSize("EVENTS.LOG", size_before, existed_before) ||
      !appendEventLogLine(g_captureLine, length)) {
    return false;
  }
  uint32_t size_after = 0;
  bool exists_after = false;
  if (!g_openlog.fileSize("EVENTS.LOG", size_after, exists_after) ||
      !exists_after || size_after < size_before + length) {
    return false;
  }
  g_watchdogResetLogPending = false;
  return true;
}

// --- Cross-task config plumbing (AGENTS.md 5.1: only i2cTask touches I2C) ---
//
// The config API runs in apiTask (it parses USB frames), but the storage commit
// is an I2C transaction that only i2cTask is allowed to perform. So apiTask
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

bool tryBootstrapConfig(const config::RobotConfig& defaults) {
  if (!g_configBootstrapPending) return true;
  const uint16_t defaults_len = config::serializeRobotConfig(
      defaults, g_bootLoad.payload, sizeof(g_bootLoad.payload));
  uint16_t loaded_len = 0;
  const config::BootstrapResult bootstrap =
      defaults_len == config::kConfigPayloadSize
          ? config::loadOrInitializeConfig(
                g_configFile, g_configStore, g_bootLoad.payload, defaults_len,
                g_bootLoad.payload, sizeof(g_bootLoad.payload), loaded_len)
          : config::BootstrapResult::StorageError;
  if (bootstrap == config::BootstrapResult::StorageError) {
    return false;
  }
  g_bootLoad.len = loaded_len;
  g_bootLoad.ready = true;
  g_configVolatile = false;
  g_configBootstrapPending = false;
  return true;
}

// Persistence sink used by the config API. commitPayload() is called from
// apiTask; it forwards the bytes to i2cTask and waits for the transaction.
class TaskConfigPersistence : public config::ConfigPersistence {
 public:
  bool commitPayload(const uint8_t* payload, uint16_t len) override {
    if (g_commitMutex == nullptr || g_commitDone == nullptr) return false;
    if (len > sizeof(g_commit.payload)) return false;
    if (xSemaphoreTake(g_commitMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
      return false;
    }
    memcpy(g_commit.payload, payload, len);
    g_commit.len = len;
    g_commit.ok = false;
    g_commit.requested = true;
    xSemaphoreGive(g_commitMutex);
    // Wait for i2cTask to append and sync the OpenLog transaction.
    if (xSemaphoreTake(g_commitDone, pdMS_TO_TICKS(1500)) != pdTRUE) {
      return false;  // timed out
    }
    if (xSemaphoreTake(g_commitMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
      return false;
    }
    const bool ok = g_commit.ok;
    xSemaphoreGive(g_commitMutex);
    return ok;
  }
  bool persistent() const override {
    return g_configStorageAvailable && !g_configVolatile;
  }
};

TaskConfigPersistence g_configPersist;
config::ConfigApi g_configApi(g_configPersist);

// Handset "save gait settings" hand-off. controlTask decides *whether* a save
// is allowed and stages the values; apiTask (the sole ConfigApi owner) runs the
// transaction. Guarded by short critical sections; the sequence comparison in
// controlTask makes the hand-off exactly-once.
config::GaitDefaults g_gaitSavePending;
volatile bool g_gaitSaveRequested = false;
uint32_t g_gaitSaveHandledSeq = 0;
// Battery level that starts warning the operator on the downlink. Above the
// safety FSM's cut-out, so it is advisory rather than a fault.
constexpr uint16_t kBatteryWarnMv = 10200;

// The portable controller owns algorithmic state. The snapshots below belong
// to the FreeRTOS adapter and stay static so the 384-word control-task stack
// is reserved for the existing gait/DXL call chains rather than large copies.
controller::ControllerCore g_controllerCore;
controller::RobotState g_controllerState;
controller::ControllerIntent g_controllerIntent;
controller::ControllerConfigSnapshot g_controllerConfig;
controller::RobotCommand g_controllerCommand;
uint32_t g_controllerConfigRevision = 0xFFFFFFFFu;

void initDeviceInfo() {
  g_deviceInfo.fw_major = 0;
  g_deviceInfo.fw_minor = 1;
  g_deviceInfo.fw_patch = 0;
  // Refreshed live from runtime feature availability before each request in
  // apiTask (4sa.4); this boot value just means "nothing available yet".
  g_deviceInfo.feature_bits = 0;
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

uint8_t packHilFlags(const hil::OutputGuardStatus& status) {
  uint8_t flags = 0;
  if (status.output_disabled) flags |= protocol::api::hilflag::kOutputDisabled;
  if (status.power_guard_active) {
    flags |= protocol::api::hilflag::kPowerGuardActive;
  }
  if (status.torque_guard_active) {
    flags |= protocol::api::hilflag::kTorqueGuardActive;
  }
  if (status.goal_guard_active) {
    flags |= protocol::api::hilflag::kGoalGuardActive;
  }
  if (status.write_guard_active) {
    flags |= protocol::api::hilflag::kWriteGuardActive;
  }
  return flags;
}

// Write one complete wire frame to USB CDC, reporting whether every byte was
// accepted. The stock SAMD21 core is unbuffered: Serial.availableForWrite()
// returns a constant (EPX_SIZE-1 = 63), so gating on it silently discarded
// every frame larger than one endpoint packet (servo_status is ~270 wire
// bytes and never reached the host).
//
// HIL bug (hexapod_src-2e8, plot workbench soak): a plain Serial.write(frame,
// n) tears most multi-packet frames. USBDeviceClass::send() waits for the
// previous 64-byte packet by polling the endpoint's TRCPT1 interrupt flag,
// but the core's USB ISR (USBCore.cpp ISRHandler -> epAckPendingInterrupts)
// clears that same flag first, so the poll spins the full 70 ms TX timeout
// and send() aborts mid-frame: exactly 128 bytes (two packets) hit the wire,
// the host resyncs on the next frame's 0x00 delimiter, and each torn frame
// also stalls apiTask ~70 ms (observed: >60% of servo_status frames torn even
// at 10 Hz).
//
// Fix: feed the core one sub-packet (<= 63 byte) chunk at a time and gate
// each chunk on the endpoint's BK1RDY hardware status bit, which is cleared
// by hardware when the bank drains and is never touched by the ISR. send()
// then always takes its fast path (bank idle -> copy, arm, return) and never
// enters the racy TRCPT1 wait. The inter-chunk wait yields via vTaskDelay and
// is bounded, so a wedged host (stopped draining) costs at most ~20 ms before
// the frame is dropped and counted as TX backlog by the caller.
bool txFrame(const uint8_t* frame, size_t n) {
  // CDC is the only PluggableUSB module, so its endpoints are 1 (ACM), 2
  // (OUT), 3 (IN) — see CDC.cpp CDC_ENDPOINT_IN = pluggedEndpoint + 2.
  constexpr uint8_t kCdcInEp = 3;
  constexpr uint32_t kBankDrainTimeoutMs = 20;
  size_t off = 0;
  while (off < n) {
    uint32_t waited_ms = 0;
    while (USB->DEVICE.DeviceEndpoint[kCdcInEp].EPSTATUS.bit.BK1RDY) {
      if (waited_ms >= kBankDrainTimeoutMs) return false;  // host not draining
      vTaskDelay(pdMS_TO_TICKS(1));
      ++waited_ms;
    }
    const size_t chunk = (n - off < 63) ? (n - off) : 63;
    if (Serial.write(frame + off, chunk) != chunk) return false;
    off += chunk;
  }
  return true;
}

#if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
void refreshHilObserver(uint32_t now_ms,
                        const hil::OutputGuardStatus& guard_status) {
  g_hilObserver.setNow(now_ms);
  g_hilObserver.setLiveState(g_safetyState);
  g_hilObserver.setMaintenanceLock(g_maintApi.token(),
                                    g_maintApi.lockHeld(now_ms));
  g_hilObserver.setOutputGuard(guard_status);
#if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
  g_hilObserver.setTaskRequestCapacity(g_hilTraceRequests.hasSpace());
#endif
  g_hilObserver.tick();
}

void queueHilObserverRequests() {
  // apiTask is the only producer. The observer refuses a new state-changing
  // command when this fixed ring has no space, so every successful take/push
  // pair below is nonblocking and cannot silently lose a request.
  while (g_hilTraceRequests.hasSpace()) {
    hil::CaptureRequest capture;
    if (g_hilObserver.takeCaptureRequest(&capture)) {
      HilTraceRequest request;
      request.type = HilTraceRequestType::Capture;
      request.session_id = capture.session_id;
      request.capture_id = capture.capture_id;
      request.value = capture.step_count;
      if (!g_hilTraceRequests.push(request)) return;
      continue;
    }
    hil::CaptureAbortRequest abort;
    if (g_hilObserver.takeAbortRequest(&abort)) {
      HilTraceRequest request;
      request.type = HilTraceRequestType::Abort;
      request.session_id = abort.session_id;
      request.capture_id = abort.capture_id;
      request.value = static_cast<uint8_t>(abort.reason);
      if (!g_hilTraceRequests.push(request)) return;
      continue;
    }
    hil::MarkerRequest marker;
    if (g_hilObserver.takeMarkerRequest(&marker)) {
      HilTraceRequest request;
      request.type = HilTraceRequestType::Marker;
      request.capture_id = marker.capture_id;
      request.value = marker.marker_id;
      if (!g_hilTraceRequests.push(request)) return;
      continue;
    }
    return;
  }
}

void consumeHilTraceRequests(const hil::OutputGuardStatus& guard_status) {
  HilTraceRequest request;
  while (g_hilTraceRequests.pop(&request)) {
    switch (request.type) {
      case HilTraceRequestType::Capture: {
        hil::CaptureRequest capture;
        capture.session_id = request.session_id;
        capture.capture_id = request.capture_id;
        capture.step_count = static_cast<uint8_t>(request.value);
        // The observer accepted this only after validating its token, state,
        // guard, and bounded request. begin() is a static-memory operation and
        // cannot open an alternate output or authority path.
        (void)g_hilTrace.begin(capture, guard_status, &g_controllerConfig);
        break;
      }
      case HilTraceRequestType::Abort:
        if (g_hilTrace.summary().capture_id == request.capture_id) {
          g_hilTrace.abort(
              static_cast<hil::CaptureEndReason>(request.value), guard_status);
        }
        break;
      case HilTraceRequestType::Marker:
        if (g_hilTrace.active() &&
            g_hilTrace.summary().capture_id == request.capture_id &&
            !g_hilTrace.markNext(request.value)) {
          g_hilTrace.abort(hil::CaptureEndReason::TransportOverflow,
                           guard_status);
        }
        break;
    }
  }
}

bool emitHilTraceRecord(uint32_t now_ms, uint8_t* frame_out,
                        size_t frame_out_cap) {
  hil::trace::RecordView record;
  if (!g_hilTrace.peek(&record)) return true;

  const uint16_t fragment_data_capacity =
      hil::trace::maxFragmentData(protocol::kMaxPayload);
  if (fragment_data_capacity == 0) {
    g_hilTrace.abandonCurrent(hil::CaptureEndReason::TransportOverflow,
                              hil::outputGuard().status());
    return false;
  }
  const uint8_t fragment_count = static_cast<uint8_t>(
      (record.logical_length + fragment_data_capacity - 1u) /
      fragment_data_capacity);
  for (uint8_t fragment_index = 0; fragment_index < fragment_count;
       ++fragment_index) {
    const uint16_t offset = static_cast<uint16_t>(
        static_cast<uint32_t>(fragment_index) * fragment_data_capacity);
    const uint16_t remaining =
        static_cast<uint16_t>(record.logical_length - offset);
    const uint16_t fragment_length =
        remaining < fragment_data_capacity ? remaining : fragment_data_capacity;
    if (!g_hilTrace.copySlice(
            record, offset, &frame_out[hil::trace::kFragmentPrefixBytes],
            static_cast<uint16_t>(protocol::kMaxPayload -
                                  hil::trace::kFragmentPrefixBytes),
            fragment_length)) {
      g_hilTrace.abandonCurrent(hil::CaptureEndReason::TransportOverflow,
                                hil::outputGuard().status());
      return false;
    }

    hil::trace::FragmentHeader fragment;
    fragment.session_id = record.session_id;
    fragment.capture_id = record.capture_id;
    fragment.record_seq = record.record_seq;
    fragment.record_type = record.type;
    fragment.fragment_index = fragment_index;
    fragment.fragment_count = fragment_count;
    fragment.logical_length = record.logical_length;
    fragment.logical_crc16 = record.logical_crc16;
    uint16_t payload_length = 0;
    if (!hil::trace::encodeFragmentSlice(
            fragment, &frame_out[hil::trace::kFragmentPrefixBytes],
            fragment_length, frame_out, protocol::kMaxPayload,
            &payload_length)) {
      g_hilTrace.abandonCurrent(hil::CaptureEndReason::TransportOverflow,
                                hil::outputGuard().status());
      return false;
    }

    protocol::Header header;
    header.msg_type = static_cast<uint8_t>(protocol::MsgType::Event);
    header.msg_id = record.type == hil::trace::RecordType::OutputBlocked
                        ? hil::trace::kOutputBlockedEvent
                        : hil::trace::kTraceFragmentEvent;
    header.flags = protocol::api::flag::kFragment;
    header.seq = g_hilTraceEventSeq++;
    header.timestamp_ms = now_ms;
    header.payload_len = payload_length;
    // encodeFrame first copies payload bytes into a local body, so frame_out
    // can safely serve as both the bounded fragment payload and wire output.
    const size_t frame_length = protocol::encodeFrame(
        header, frame_out, frame_out, frame_out_cap);
    if (frame_length == 0 || !txFrame(frame_out, frame_length)) {
      g_subs.noteTxBacklog();
      g_hilTrace.abandonCurrent(hil::CaptureEndReason::TransportTimeout,
                                hil::outputGuard().status());
      return false;
    }
    g_hilTrace.noteFragmentsSent(1);
  }
  g_hilTrace.acknowledge(record);

  uint32_t session_id = 0;
  uint32_t capture_id = 0;
  hil::CaptureEndReason reason = hil::CaptureEndReason::Complete;
  if (g_hilTrace.takeCompletion(&session_id, &capture_id, &reason)) {
    (void)session_id;
    (void)reason;
    g_hilObserver.noteCaptureFinished(capture_id);
  }
  return true;
}
#endif

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
      o += put16(&p[o], g_batteryMv);  // snapshot; ADC is controlTask-only
      const hil::OutputGuardStatus guard_status = hil::outputGuard().status();
      p[o++] = packHilFlags(guard_status);
      o += put32(&p[o], guard_status.blocked_power_enable);
      o += put32(&p[o], guard_status.blocked_torque_enable);
      o += put32(&p[o], guard_status.blocked_goal_write);
      o += put32(&p[o], guard_status.blocked_dxl_write);
      o += put32(&p[o], guard_status.last_goal_sequence);
      p[o++] = guard_status.last_goal_count;
      p[o++] = g_lastFaultReason;
      o += put32(&p[o], g_lastFaultTimestampMs);
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
      // valid once each servo has been polled at least once. Byte layout lives
      // in the portable encoder (lmt.12) so it is host-vector-tested.
      o = protocol::encodeServoStatus(g_servoStatus, g_servoStatusCount, p);
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
      // tx_backlog(4), per-stream dropped(4) x kNumStreams, then USB rx health
      // (hexapod_src-lv6): rx_frames(4) complete frame bodies received,
      // rx_bad(4) COBS/CRC/magic/length decode failures, rx_overflow(4) frames
      // dropped for exceeding the reader buffer. Older hosts that only parse
      // the tx fields ignore the appended tail.
      o += put32(&p[o], g_subs.txBacklog());
      for (uint8_t i = 0; i < protocol::kNumStreams; ++i) {
        o += put32(&p[o], g_subs.dropped(static_cast<StreamId>(i)));
      }
      o += put32(&p[o], g_frameReader.framesOk());
      o += put32(&p[o], g_apiRxBad);
      o += put32(&p[o], g_frameReader.overflowsDropped());
      break;
    }
    case StreamId::JointState: {
      // Mapped present joint angles, ready to render without the host needing
      // the servo map (eax.1). count(1) then 4 bytes/joint: leg(1), joint(1),
      // angle_centideg(int16). Angles come from the active config's servo map
      // (sign/trim/center 2048, 4096 ticks/rev), so they read correctly in both
      // active and passive (torque-off) modes from the present-position snapshot.
      // 18 joints -> 1 + 18*4 = 73 bytes, within the 256 payload cap. Byte
      // layout + tick->angle mapping live in the portable encoder (lmt.12).
      const dxl::ServoMap map(g_configApi.config());
      o = protocol::encodeJointState(map, g_servoStatus, g_servoStatusCount, p);
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
      // 18 joints -> 1 + 18*5 = 91 bytes, within the 256 payload cap. The byte
      // layout lives in the portable encoder (lmt.12); this case keeps only the
      // task glue (source selection + the zero-wait goal mutex).
      const dxl::ServoMap map(g_configApi.config());
      bool used_gait = false;
      // Live gait goals take priority while motion is authorised. Copy under a
      // zero-wait lock; if controlTask is mid-publish, fall back to the bench
      // target set for this frame rather than block the telemetry task.
      if (g_motionGate && g_goalValid && g_goalMutex != nullptr &&
          xSemaphoreTake(g_goalMutex, 0) == pdTRUE) {
        o = protocol::encodeServoGoals(map, g_goalFrame.joints,
                                       g_goalFrame.count, p);
        xSemaphoreGive(g_goalMutex);
        used_gait = true;
      }
      if (!used_gait) {
        o = protocol::encodeServoGoals(map, g_maintTargetApi.target(), p);
      }
      break;
    }
    case StreamId::LegState: {
      // Per-leg commanded foot target + IK verdict (eax.3). Lets the animation
      // draw the commanded foot positions and flag unreachable poses. count(1)
      // then 8 bytes/leg: leg(1), foot_x(i16), foot_y(i16), foot_z(i16, mm body
      // frame), flags(1). flags bit0 = reachable, bit1 = clamped (a joint hit
      // its configured travel). Only legs with a recorded SET_LEG_TARGET attempt
      // are emitted, so until a leg target is sent the payload is a zero count.
      // 6 legs -> 1 + 6*8 = 49 bytes, within the 256 payload cap. Byte layout
      // lives in the portable encoder (lmt.12).
      o = protocol::encodeLegState(g_maintTargetApi.target(), p);
      break;
    }
    case StreamId::ControllerState: {
      // Decoded hand-controller intent + raw ChannelPack inputs (oha.4). Mirrors
      // the CONTROLLER_GET_STATE response so a host can watch the live remote
      // (modes, twist, body pose, shape, tricks, arm/estop) and every raw
      // gimbal/pot/encoder/switch/button/nav input. 57-byte fixed layout lives
      // in the portable ControllerApi codec; rcTask publishes g_ctrlCmd/g_ctrlRaw.
      o = protocol::ControllerApi::encodeState(g_ctrlCmd, g_ctrlRaw, p);
      break;
    }
    case StreamId::RcDiagnostics: {
      // Raw CRSF layer for RC troubleshooting (a8n): the raw 11-bit ticks, frame
      // health counters, last-frame age, and decoded LINK_STATISTICS signal
      // quality. Complements the parsed RcInput stream (microsecond channels +
      // decoded arm/kill/gait/autonomy flags) so the host can show both.
      // Layout: flags(1), 16 x raw_tick(u16), frames_decoded(u32),
      // crc_errors(u32), link_stats_count(u32), last_frame_age_ms(u16), then the
      // 10-byte link-stats block = 57 bytes, within the 256 payload cap.
      // flags: bit0 ever_seen, bit1 failsafe, bit2 link_stats_valid.
      uint8_t flags = 0;
      if (g_rcDiag.ever_seen) flags |= 0x01;
      if (g_rcDiag.failsafe) flags |= 0x02;
      if (g_rcDiag.has_link_stats) flags |= 0x04;
      p[o++] = flags;
      for (uint8_t i = 0; i < crsf::kNumChannels; ++i) {
        o += put16(&p[o], g_rcDiag.raw_ticks[i]);
      }
      o += put32(&p[o], g_rcDiag.frames_decoded);
      o += put32(&p[o], g_rcDiag.crc_errors);
      o += put32(&p[o], g_rcDiag.link_stats_count);
      // Elapsed ms since the last valid RC frame, capped at 0xFFFF; 0xFFFF also
      // encodes "never seen" so the host can show a clear dropout indicator.
      uint32_t age = g_rcDiag.ever_seen ? (now_ms - g_rcDiag.last_frame_ms)
                                        : 0xFFFFu;
      if (age > 0xFFFFu) age = 0xFFFFu;
      o += put16(&p[o], static_cast<uint16_t>(age));
      const crsf::LinkStatistics& ls = g_rcDiag.link_stats;
      p[o++] = ls.up_rssi_ant1;
      p[o++] = ls.up_rssi_ant2;
      p[o++] = ls.up_link_quality;
      p[o++] = static_cast<uint8_t>(ls.up_snr);
      p[o++] = ls.active_antenna;
      p[o++] = ls.rf_mode;
      p[o++] = ls.up_tx_power;
      p[o++] = ls.down_rssi;
      p[o++] = ls.down_link_quality;
      p[o++] = static_cast<uint8_t>(ls.down_snr);
      break;
    }
    case StreamId::HilStatus: {
      const hil::OutputGuardStatus guard_status = hil::outputGuard().status();
      hil::GoalTargetRecord goals[hil::kMaxRecordedGoalTargets] = {};
      const uint8_t goal_count = hil::outputGuard().copyLastBlockedGoals(
          goals, hil::kMaxRecordedGoalTargets);
      p[o++] = packHilFlags(guard_status);
      p[o++] = goal_count;
      o += put32(&p[o], guard_status.last_goal_sequence);
      o += put32(&p[o], guard_status.blocked_power_enable);
      o += put32(&p[o], guard_status.blocked_torque_enable);
      o += put32(&p[o], guard_status.blocked_goal_write);
      o += put32(&p[o], guard_status.blocked_dxl_write);
      for (uint8_t i = 0; i < goal_count; ++i) {
        p[o++] = goals[i].id;
        o += put32(&p[o], static_cast<uint32_t>(goals[i].tick));
      }
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
//   * JetsonControl   - Jetson heartbeat -> arbiter authority is wired
//                        (lmt.13); available, defaults off, gated by RC.
// NOTE: the controlTask-loop helpers below are marked noinline deliberately.
// GCC inlines single-call-site functions, which merged all of their locals
// into controlTask's frame -- ~760 bytes that stayed live during the deep
// ControllerCore::step -> gait/IK descent and overflowed the 448-word stack.
// As real calls, each frame is popped before step() recurses deeper.
//
// Flash-resident default images. Assigning `g_x = X{}` materializes the
// temporary on the caller's stack (628 B RobotState / 320 B ControllerIntent /
// 508 B ControllerConfigSnapshot); copy-assigning from these constants does a
// flash->RAM memberwise copy with no stack temporary.
const controller::RobotState kDefaultRobotState{};
constexpr controller::ControllerIntent kDefaultControllerIntent{};
const controller::ControllerConfigSnapshot kDefaultControllerConfigSnapshot{};

__attribute__((noinline)) void updateFeatureFlags(uint32_t now_ms) {
  using protocol::Feature;
  using protocol::FeatureReason;

  const bool i2c_fresh = g_i2cLastUpdateMs != 0 &&
                         (now_ms - g_i2cLastUpdateMs) <= kI2cStaleMs;
  const bool mux = i2c_fresh && g_i2cTopology.mux_present;
  const bool any_foot = i2c_fresh && g_footPresentMask != 0;

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
  // JetsonControl: the Jetson heartbeat -> arbiter authority path is wired
  // (lmt.13). Report the capability as available so a host can opt in via
  // FEATURE_SET; it defaults off and only becomes effective authority when the
  // RC autonomy switch + RC arm + a fresh Jetson heartbeat all agree.
  g_featureApi.setAvailability(Feature::JetsonControl, true,
                               FeatureReason::None);

  g_featureApi.setLiveState(g_safetyState);

  // Drive the one real toggle: raw foot-sensor polling.
  g_sensorPollingEnabled =
      g_featureApi.effectiveEnabled(Feature::SensorPolling);
}

// --- Task bodies ----------------------------------------------------------
// Each task runs a fixed-period loop with vTaskDelayUntil so timing does not
// drift with body execution time. Bodies are stubs for the skeleton.

// Project the decoded ControllerCommand onto the legacy RcStatus the safety
// FSM / arbiter / rc_input telemetry still consume (oha.3). The bridge is the
// single decoder of truth; this keeps the rest of the safety plumbing working
// without re-plumbing it. channels_us is refreshed (for the rc_input telemetry)
// only on a fresh frame so a dropped link does not zero the last-known sticks.
void deriveRcStatus(const controller::ControllerCommand& cc,
                    const crsf::ChannelData& frame, bool fresh, uint32_t now_ms,
                    crsf::RcStatus& rc) {
  if (fresh) {
    for (uint8_t i = 0; i < crsf::kNumChannels; ++i) {
      rc.channels_us[i] = crsf::ticksToMicros(frame.channels[i]);
    }
    rc.last_frame_ms = now_ms;
  }
  rc.armed = cc.arm_request;       // SW_A (and not failsafe; bridge clears it)  // Kill requires an RC system to have existed: the bridge's failsafe hold
  // synthesises estop when no receiver has ever been seen, which must not
  // assert a kill that locks the arbiter's maintenance lock and the FSM out
  // of Disarmed on an RC-less bench (AGENTS.md mode 4). ever_seen latches, so
  // a link that drops mid-operation still kills.
  rc.kill = cc.estop && cc.ever_seen;
  rc.gait_index = cc.gait_index;   // SW_E 3-pos (wave / ripple / tripod)
  rc.autonomy = cc.host_authority; // SW_H grants host/Jetson authority
  rc.failsafe = cc.failsafe;
  rc.ever_seen = cc.ever_seen;
}

uint8_t fractionToByte(float value) {
  if (value <= 0.0f) return 0;
  if (value >= 1.0f) return 255;
  return static_cast<uint8_t>(value * 255.0f + 0.5f);
}

int16_t scaledI16(float value, float scale) {
  const float scaled = value * scale;
  if (scaled >= 32767.0f) return 32767;
  if (scaled <= -32768.0f) return -32768;
  return static_cast<int16_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

int16_t servoAngleCentideg(const dxl::ServoMap& map, uint8_t leg,
                            uint8_t joint, int32_t tick) {
  if (tick < 0 || tick > config::kServoMaxTick) return 0;
  return scaledI16(map.tickToAngle(leg, joint,
                                   static_cast<uint16_t>(tick)),
                   5729.578f);
}

logging::RemoteCapture buildRemoteCapture(
    const controller::ControllerCommand& command,
    const ChannelPackInputs_t& raw, uint32_t frame_sequence,
    uint32_t now_ms) {
  logging::RemoteCapture capture;
  capture.timestamp_ms = now_ms;
  capture.frame_sequence = frame_sequence;
  for (uint8_t i = 0; i < 4; ++i) capture.gimbal[i] = raw.gimbal[i];
  for (uint8_t i = 0; i < 2; ++i) {
    capture.pot[i] = raw.pot[i];
    capture.encoder[i] = raw.encoder[i];
    capture.toggle[i] = raw.toggles[i];
    for (uint8_t direction = 0; direction < 5; ++direction) {
      if (raw.nav[i][direction]) {
        capture.nav_mask[i] |= static_cast<uint8_t>(1u << direction);
      }
    }
  }
  for (uint8_t i = 0; i < 8; ++i) {
    if (raw.switches[i]) capture.switch_mask |= static_cast<uint8_t>(1u << i);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    if (raw.buttons[i]) capture.button_mask |= static_cast<uint8_t>(1u << i);
  }
  if (command.valid) capture.flags |= 0x01;
  if (command.failsafe) capture.flags |= 0x02;
  if (command.arm_request) capture.flags |= 0x04;
  if (command.estop) capture.flags |= 0x08;
  if (command.host_authority) capture.flags |= 0x10;
  capture.mode = static_cast<uint8_t>(command.mode);
  capture.gait = command.gait_index;
  capture.twist_milli[0] = scaledI16(command.twist_vx, 1000.0f);
  capture.twist_milli[1] = scaledI16(command.twist_vy, 1000.0f);
  capture.twist_milli[2] = scaledI16(command.twist_wz, 1000.0f);
  capture.pose[0] = scaledI16(command.pose_x_mm, 1.0f);
  capture.pose[1] = scaledI16(command.pose_y_mm, 1.0f);
  capture.pose[2] = scaledI16(command.pose_z_mm, 1.0f);
  capture.pose[3] = scaledI16(command.pose_roll, 1000.0f);
  capture.pose[4] = scaledI16(command.pose_pitch, 1000.0f);
  capture.pose[5] = scaledI16(command.pose_yaw, 1000.0f);
  capture.speed_x255 = fractionToByte(command.speed);
  capture.body_height_x255 = fractionToByte(command.body_height);
  capture.stride_x255 = fractionToByte(command.stride);
  capture.step_height_x255 = fractionToByte(command.step_height);
  capture.duty_x255 = fractionToByte(command.duty);
  capture.capture_toggle_seq = command.capture_toggle_seq;
  capture.capture_start_seq = command.capture_start_seq;
  return capture;
}

uint8_t rcGaitForTelemetry(const controller::ControllerCommand& command) {
  // SW_E selects the walking gait in every right-gimbal mode, so translate and
  // rotate no longer force a Stand pose: the operator can walk while moving
  // the body.
  return static_cast<uint8_t>(
      controller::rcGaitFromIndex(command.gait_index));
}

uint8_t batteryPercent(uint16_t millivolts) {
  constexpr uint16_t kEmptyMv = 9900;
  constexpr uint16_t kFullMv = 12600;
  if (millivolts <= kEmptyMv) return 0;
  if (millivolts >= kFullMv) return 100;
  return static_cast<uint8_t>(
      (static_cast<uint32_t>(millivolts - kEmptyMv) * 100u) /
      (kFullMv - kEmptyMv));
}

bool sendCrsfTelemetry(uint8_t type, const uint8_t* payload,
                       uint8_t payload_len) {
#if defined(PIN_SERIAL3_TX)
  uint8_t frame[crsf::kMaxFrameLen];
  const uint8_t frame_len = crsf::telemetry::buildFrame(
      type, payload, payload_len, frame, sizeof(frame));
  if (frame_len == 0 || Serial3.availableForWrite() < frame_len) return false;
  return Serial3.write(frame, frame_len) == frame_len;
#else
  (void)type;
  (void)payload;
  (void)payload_len;
  return false;
#endif
}

bool configuredServoCoverageFromBus() {
  if (g_dxlBus.servoCount() < config::kNumServos) return false;
  const config::RobotConfig& cfg = g_configApi.config();
  for (uint8_t i = 0; i < config::kNumServos; ++i) {
    if (g_dxlBus.profileById(cfg.servos[i].id) == nullptr) return false;
  }
  return true;
}

uint32_t configuredPoseMaskFromStatus() {
  uint32_t mask = 0;
  const config::RobotConfig& cfg = g_configApi.config();
  for (uint8_t i = 0; i < config::kNumServos; ++i) {
    for (uint8_t s = 0; s < g_servoStatusCount; ++s) {
      if (g_servoStatus[s].id == cfg.servos[i].id && g_servoStatus[s].ok) {
        mask |= (1u << i);
        break;
      }
    }
  }
  return mask;
}

void publishServoReadiness() {
  g_configuredServoCoverage = configuredServoCoverageFromBus();
  g_poseKnownMask = configuredPoseMaskFromStatus();
  // Publish the revision last. controlTask only consumes the readiness values
  // when this tag matches the active config, so evidence from an old servo map
  // cannot authorize a newly committed map.
  g_servoReadinessConfigRev = g_configApi.revision();
}

bool buildConfiguredHoldTargets(dxl::GoalTarget* hold) {
  if (configuredPoseMaskFromStatus() != kAllServoPosesKnown) {
    return false;
  }
  const config::RobotConfig& cfg = g_configApi.config();
  uint8_t ids[config::kNumServos];
  for (uint8_t i = 0; i < config::kNumServos; ++i) {
    ids[i] = cfg.servos[i].id;
  }
  return dxl::buildHoldTargets(ids, config::kNumServos, g_servoStatus,
                               g_servoStatusCount, hold,
                               config::kNumServos);
}

bool buildDiscoveredHoldTargets(dxl::GoalTarget* hold, uint8_t& count) {
  count = g_dxlBus.servoCount();
  if (count == 0 || count > dxl::DxlBus::kMaxServos) return false;
  uint8_t ids[dxl::DxlBus::kMaxServos];
  for (uint8_t i = 0; i < count; ++i) {
    ids[i] = g_dxlBus.profile(i).id;
  }
  return dxl::buildHoldTargets(ids, count, g_servoStatus,
                               g_servoStatusCount, hold,
                               dxl::DxlBus::kMaxServos);
}

bool refreshDiscoveredPositions() {
  const uint8_t count = g_dxlBus.servoCount();
  if (count == 0) return false;
  for (uint8_t i = 0; i < count; ++i) {
    if (g_dxlBus.syncReadStatus(g_servoStatus, dxl::DxlBus::kMaxServos) != 1) {
      return false;
    }
  }
  g_servoStatusCount = count;
  publishServoReadiness();
  return true;
}

bool verifyDiscoveredTorqueOff() {
  const uint8_t count = g_dxlBus.servoCount();
  for (uint8_t i = 0; i < count; ++i) {
    const dxl::ServoProfile& profile = g_dxlBus.profile(i);
    bool on = true;
    if (!g_dxlBus.torqueState(profile.id, profile.table_kind, on) || on) {
      return false;
    }
  }
  return true;
}

__attribute__((noinline)) bool refreshControllerConfigSnapshot() {
  const uint32_t revision = g_configApi.revision();
  if (g_controllerConfig.valid &&
      revision == g_controllerConfigRevision) {
    return true;
  }

  if (!controller::makeControllerConfigSnapshot(
          g_configApi.config(), revision, !g_configVolatile,
          g_controllerConfig)) {
    g_controllerConfig = kDefaultControllerConfigSnapshot;
    g_controllerConfig.revision = revision;
    g_controllerConfig.persistent = !g_configVolatile;
    return false;
  }
  g_controllerConfigRevision = revision;
  taskENTER_CRITICAL();
  g_pendingRcCalibration = g_controllerConfig.robot.rc_input;
  g_pendingRcCalibrationRevision = revision;
  g_pendingRcCalibrationValid = true;
  taskEXIT_CRITICAL();
  return true;
}

__attribute__((noinline)) void collectControllerState(uint32_t now_ms) {
  controller::RobotState& state = g_controllerState;
  state = kDefaultRobotState;
  state.config_ready = g_configReady;

  if (!g_batterySampled ||
      (now_ms - g_lastBatterySampleMs) >= kBatterySamplePeriodMs) {
    uint16_t battery_mv = 0;
    g_batteryValid = board::readBatteryMilliVolts(battery_mv) &&
                     battery_mv > 6000;
    g_batteryMv = battery_mv;
    g_lastBatterySampleMs = now_ms;
    g_batterySampled = true;
  }
  state.battery.millivolts = g_batteryMv;
  state.battery.valid = g_batteryValid;
  state.battery.validity = state.battery.valid
                               ? controller::SnapshotValidity::Fresh
                               : controller::SnapshotValidity::Unknown;

  uint8_t servo_count = g_servoStatusCount;
  if (servo_count > config::kNumServos) servo_count = config::kNumServos;
  state.dxl.servo_count = servo_count;
  state.dxl.validity = controller::SnapshotValidity::Fresh;
  state.dxl.configured_servo_coverage = g_configuredServoCoverage;
  state.dxl.pose_known_mask = g_poseKnownMask;
  state.dxl.config_revision = g_servoReadinessConfigRev;
  state.dxl.torque_off = g_dxlTorqueOff;
  state.dxl.hard_fault = g_dxlHardFault;
  for (uint8_t index = 0; index < servo_count; ++index) {
    state.dxl.servos[index] = g_servoStatus[index];
  }

  state.contact.present_mask = g_footPresentMask;
  state.contact.validity = controller::SnapshotValidity::Fresh;
  for (uint8_t leg = 0; leg < sensors::kNumFeet; ++leg) {
    state.contact.feet[leg] = g_footState[leg];
  }
  state.watchdog_fault = watchdog::criticalStalled();
}

__attribute__((noinline)) void collectControllerIntent(bool maintenance_held,
                                                        bool host_disarm) {
  controller::ControllerIntent& intent = g_controllerIntent;
  intent = kDefaultControllerIntent;
  controller::RcFrameSnapshot latest_rc;
  taskENTER_CRITICAL();
  const bool rc_snapshot_available = g_rcMailbox.copy(latest_rc);
  taskEXIT_CRITICAL();
  if (rc_snapshot_available) {
    g_controlRcSnapshot = latest_rc;
    g_controlRcSnapshotValid = true;
  }
  if (g_controlRcSnapshotValid) {
    const controller::RcFrameSnapshot& rc = g_controlRcSnapshot;
    intent.rc.command = rc.command;
    intent.rc.ever_seen = rc.status.ever_seen;
    intent.rc.kill = rc.status.kill;
    intent.rc.armed = rc.status.armed;
    intent.rc.failsafe = rc.status.failsafe;
    intent.rc.autonomy_enabled = rc.status.autonomy;
  }
  intent.motion = g_motionApi.intent();
  intent.maintenance.lock_held = maintenance_held;
  intent.maintenance.lock_token = g_maintApi.token();
  intent.maintenance.control_mode = g_maintTargetApi.controlMode();
  intent.maintenance.targets = g_maintTargetApi.target();
  intent.features.foot_contact_enabled =
      g_featureApi.effectiveEnabled(protocol::Feature::FootContact);
  intent.features.terrain_leveling_enabled =
      g_featureApi.effectiveEnabled(protocol::Feature::TerrainLeveling);
  intent.features.sensor_polling_enabled =
      g_featureApi.effectiveEnabled(protocol::Feature::SensorPolling);
  intent.features.jetson_control_enabled =
      g_featureApi.effectiveEnabled(protocol::Feature::JetsonControl);
  intent.features.passive_pose_enabled =
      g_featureApi.effectiveEnabled(protocol::Feature::PassivePose);
  intent.host_estop = g_controlApi.estopActive();
  intent.host_disarm = host_disarm;
  intent.clear_fault_requested = g_controlApi.consumeClearFault();
  intent.passive_requested = g_passiveApi.requested();
  intent.jetson_heartbeat_received = g_controlApi.consumeJetsonHeartbeat();
}

__attribute__((noinline)) void publishControllerCommand(uint32_t now_ms) {
  const controller::RobotCommand& command = g_controllerCommand;
  g_commandSource = static_cast<uint8_t>(command.command_source);
  g_motionAuthorized = command.motion_authorized;
  g_killActive = g_controllerIntent.rc.kill || g_controllerIntent.host_estop;
  g_safetyState = static_cast<uint8_t>(command.safety_state);
  g_faultReason = static_cast<uint8_t>(command.fault_reason);
  g_lastFaultReason =
      static_cast<uint8_t>(g_controllerCore.lastFaultReason());
  g_lastFaultTimestampMs = g_controllerCore.lastFaultTimestampMs();
  watchdog::markSafetyState(g_safetyState);

  if (command.diagnostics.clear_maintenance_lock) g_maintApi.revoke();
  if (command.diagnostics.clear_passive_request) g_passiveApi.clear();
  if (command.diagnostics.clear_maintenance_targets) {
    g_maintTargetApi.clearTargets();
    g_maintTargetApi.resetControlMode();
  }

  g_motionGate = command.motion_gate;
  g_controlApi.setLiveState(g_safetyState, g_faultReason);
  g_passiveApi.setLiveState(g_safetyState);
  g_motionApi.setLiveState(g_safetyState, g_motionGate);

  crsf::telemetry::HexapodStatus radio;
  radio.safety_state = g_safetyState;
  radio.command_source = g_commandSource;
  radio.fault_reason = g_faultReason;
  radio.battery_mv = g_controllerState.battery.valid
                         ? g_controllerState.battery.millivolts
                         : 0;
  if (g_controllerIntent.rc.armed) {
    radio.flags |= crsf::telemetry::flag::kArmed;
  }
  if (command.motion_gate) {
    radio.flags |= crsf::telemetry::flag::kMotionGate;
  }
  if (g_controllerIntent.rc.kill || g_controllerIntent.host_estop) {
    radio.flags |= crsf::telemetry::flag::kKill;
  }
  if (g_controllerIntent.rc.failsafe) {
    radio.flags |= crsf::telemetry::flag::kFailsafe;
  }
  if (g_controllerState.battery.valid) {
    radio.flags |= crsf::telemetry::flag::kBatteryValid;
  }
  if (g_faultReason != static_cast<uint8_t>(safety::FaultReason::None)) {
    radio.flags |= crsf::telemetry::flag::kFault;
  }

  const bool report_rc_selection =
      command.command_source != safety::CommandSource::Jetson &&
      command.command_source != safety::CommandSource::MacMaintenance &&
      g_controllerIntent.rc.command.ever_seen;
  if (report_rc_selection) {
    const controller::ControllerCommand& rc = g_controllerIntent.rc.command;
    radio.gait = rcGaitForTelemetry(rc);
    radio.control_mode = static_cast<uint8_t>(rc.mode);
    radio.body_height_mm = command.diagnostics.applied_body_height_mm;
    radio.stride_mm = command.diagnostics.applied_stride_mm;
    radio.step_height_mm = command.diagnostics.applied_step_height_mm;
    radio.speed_x255 = command.diagnostics.applied_speed_x255;
    radio.duty_x255 = command.diagnostics.applied_duty_x255;
  } else if (command.command_source == safety::CommandSource::Jetson) {
    const protocol::MotionIntent& motion = g_controllerIntent.motion;
    radio.gait = motion.gait;
    radio.control_mode = static_cast<uint8_t>(controller::ControlMode::Yaw);
    radio.body_height_mm = motion.body_height_mm;
    radio.stride_mm = motion.stride_len_mm;
    radio.step_height_mm = motion.step_height_mm;
    radio.speed_x255 = motion.speed_x255;
    radio.duty_x255 = motion.duty_x255;
  } else {
    radio.gait = static_cast<uint8_t>(config::GaitId::Stand);
    radio.control_mode = static_cast<uint8_t>(controller::ControlMode::Yaw);
    radio.body_height_mm = g_controllerConfig.robot.gait.body_height_mm;
    radio.stride_mm = g_controllerConfig.robot.gait.stride_len_mm;
    radio.step_height_mm = g_controllerConfig.robot.gait.step_height_mm;
    radio.speed_x255 = g_controllerConfig.robot.gait.speed_x255;
    radio.duty_x255 = g_controllerConfig.robot.gait.duty_x255;
  }

  // --- Deduplicated error reporting -----------------------------------------
  // Every producer here is level-triggered and runs at 100 Hz. The journal
  // collapses repeats into one announcement plus a running count, so a stuck
  // fault costs one downlink frame per kRepeatIntervalMs instead of 500.
  if (command.fault_reason != safety::FaultReason::None) {
    noteError(safety::ErrorCode::SafetyFault,
              static_cast<uint8_t>(command.fault_reason),
              command.safety_state >= safety::State::FaultHard
                  ? safety::ErrorSeverity::Critical
                  : safety::ErrorSeverity::Error,
              now_ms);
  }
  if (g_controllerState.dxl.hard_fault) {
    noteError(safety::ErrorCode::DxlHardwareError, 0,
              safety::ErrorSeverity::Critical, now_ms);
  }
  if (g_controllerState.battery.valid &&
      g_controllerState.battery.millivolts < kBatteryWarnMv) {
    noteError(safety::ErrorCode::BatteryLow,
              static_cast<uint8_t>(g_controllerState.battery.millivolts / 100u),
              safety::ErrorSeverity::Warning, now_ms);
  }
  if (g_controllerIntent.rc.failsafe && g_controllerIntent.rc.ever_seen) {
    noteError(safety::ErrorCode::RcFailsafe, 0,
              safety::ErrorSeverity::Error, now_ms);
  }
  if (g_configVolatile) {
    noteError(safety::ErrorCode::ConfigVolatile, 0,
              safety::ErrorSeverity::Warning, now_ms);
  }
  if (command.diagnostics.any_goal_unreachable) {
    noteError(safety::ErrorCode::GoalUnreachable, 0,
              safety::ErrorSeverity::Warning, now_ms);
  }
  if (command.diagnostics.any_goal_clamped) {
    noteError(safety::ErrorCode::GoalClamped, 0,
              safety::ErrorSeverity::Info, now_ms);
  }
  if (g_controllerState.watchdog_fault) {
    noteError(safety::ErrorCode::WatchdogStall, 0,
              safety::ErrorSeverity::Critical, now_ms);
  }

  // --- Handset gait-tune editor --------------------------------------------
  const controller::ControllerDiagnostics& diag = command.diagnostics;
  uint8_t tune_flags = 0;
  if (diag.gait_tune_active) {
    tune_flags |= crsf::telemetry::tuneflag::kTuneActive;
  }
  if (diag.gait_tune_preview) {
    tune_flags |= crsf::telemetry::tuneflag::kPreviewActive;
  }
  if (g_configVolatile) {
    tune_flags |= crsf::telemetry::tuneflag::kConfigVolatile;
  }
  tune_flags |= static_cast<uint8_t>(
      (diag.gait_tune_param & 0x03u)
      << crsf::telemetry::tuneflag::kParamShift);

  // A save press is handed to apiTask, the sole owner of ConfigApi. The
  // sequence comparison makes this exactly-once even though the two tasks run
  // at different rates.
  if (diag.gait_save_seq != g_gaitSaveHandledSeq) {
    if (diag.gait_save_requested) {
      g_gaitSaveHandledSeq = diag.gait_save_seq;
      config::GaitDefaults pending = g_controllerConfig.robot.gait;
      pending.body_height_mm = diag.applied_body_height_mm;
      pending.stride_len_mm = diag.applied_stride_mm;
      pending.step_height_mm = diag.applied_step_height_mm;
      pending.duty_x255 = diag.applied_duty_x255;
      pending.speed_x255 = diag.applied_speed_x255;
      taskENTER_CRITICAL();
      g_gaitSavePending = pending;
      g_gaitSaveRequested = true;
      taskEXIT_CRITICAL();
    }
    // Not safe to persist right now (robot moving / RC not in authority):
    // leave the sequence unhandled so the operator can simply stop and press
    // again, and tell them why exactly once.
    else if (diag.gait_tune_active) {
      noteError(safety::ErrorCode::GaitSaveRejected,
                static_cast<uint8_t>(safety::GaitSaveReject::Busy),
                safety::ErrorSeverity::Warning, now_ms);
    }
  }
  if (g_gaitSaveRequested) {
    tune_flags |= crsf::telemetry::tuneflag::kSavePending;
  }
  radio.tune_flags = tune_flags;

  safety::ErrorEntry latest;
  bool has_error = false;
  uint32_t suppressed = 0;
  // Advance to the next announced incident at most once per downlink period.
  // controlTask runs at 100 Hz; draining every cycle would burn through a burst
  // of pending entries between two 5 Hz status frames and only the last one
  // would ever reach the operator.
  static uint32_t error_drain_ms = 0;
  static bool error_drain_seen = false;
  const bool drain_due = !error_drain_seen ||
                         (now_ms - error_drain_ms) >= kCrsfStatusBoostPeriodMs;
  taskENTER_CRITICAL();
  if (drain_due) g_errorJournal.takePending(nullptr);
  has_error = g_errorJournal.hasLatest();
  if (has_error) latest = g_errorJournal.latest();
  suppressed = g_errorJournal.suppressed();
  taskEXIT_CRITICAL();
  if (drain_due) {
    error_drain_ms = now_ms;
    error_drain_seen = true;
  }
  if (has_error) {
    radio.error_code = static_cast<uint8_t>(latest.code);
    radio.error_detail = latest.detail;
    radio.error_sequence = latest.sequence;
    radio.error_count = latest.count;
    radio.tune_flags = static_cast<uint8_t>(
        radio.tune_flags |
        ((static_cast<uint8_t>(latest.severity) & 0x03u)
         << crsf::telemetry::tuneflag::kSeverityShift));
  }
  radio.error_suppressed =
      suppressed > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(suppressed);

  g_radioStatusBoost =
      diag.gait_tune_active &&
      (now_ms - g_controllerIntent.rc.command.gait_tune_last_edit_ms) <
          kCrsfStatusBoostHoldMs;

  // Staged at file scope, not on the controlTask stack: this frame sits on the
  // same stack as the armed gait/IK path, and the extra ~52 words pushed the
  // 448-word stack to a 43-word high-water margin (overflow on arming).
  // Only controlTask writes this staging copy, so no lock is needed here.
  static sensors::DebugDisplayState display;
  const controller::ControllerCommand& rc = g_controllerIntent.rc.command;
  display.body_height_mm = diag.applied_body_height_mm;
  display.stride_mm = diag.applied_stride_mm;
  display.step_height_mm = diag.applied_step_height_mm;
  display.duty_x255 = diag.applied_duty_x255;
  display.speed_x255 = diag.applied_speed_x255;
  display.gait = rc.gait_index == 0 ? static_cast<uint8_t>(config::GaitId::Wave)
         : rc.gait_index == 1
           ? static_cast<uint8_t>(config::GaitId::Ripple)
           : static_cast<uint8_t>(config::GaitId::Tripod);
  display.safety_state = g_safetyState;
  display.fault_reason = g_faultReason;
  display.battery_mv = g_controllerState.battery.millivolts;
  display.battery_valid = g_controllerState.battery.valid;
  display.rc_seen = rc.ever_seen;
  display.rc_failsafe = rc.failsafe;
  display.rc_armed = rc.arm_request;
  display.tune_param = diag.gait_tune_param;
  display.tune_active = diag.gait_tune_active;
  display.trim_roll_cdeg = static_cast<int16_t>(rc.trim_roll * 5729.578f);
  display.trim_pitch_cdeg = static_cast<int16_t>(rc.trim_pitch * 5729.578f);

  taskENTER_CRITICAL();
  g_radioStatus = radio;
  g_debugDisplayState = display;
  taskEXIT_CRITICAL();

  if (command.goal_valid) {
    // Preserve the existing bounded publication behavior: never block the
    // real-time controller if dxlTask is copying the prior goal frame.
    if (g_goalMutex != nullptr && xSemaphoreTake(g_goalMutex, 0) == pdTRUE) {
      g_goalFrame.count = command.goals.count;
      for (uint8_t index = 0; index < command.goals.count; ++index) {
        g_goalFrame.joints[index] = command.goals.joints[index];
      }
      for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
        g_goalFrame.legs[leg] = command.goals.legs[leg];
      }
      const uint32_t goal_sequence = g_goalSeq + 1u;
      logging::AppliedMotionCapture& applied = g_goalFrame.applied;
      applied.goal_sequence = goal_sequence;
      applied.body_height_mm = command.diagnostics.applied_body_height_mm;
      applied.stride_mm = command.diagnostics.applied_stride_mm;
      applied.step_height_mm = command.diagnostics.applied_step_height_mm;
      applied.command_source = static_cast<uint8_t>(command.command_source);
      applied.safety_state = static_cast<uint8_t>(command.safety_state);
      applied.gait = radio.gait;
      applied.duty_x255 = command.diagnostics.applied_duty_x255;
      applied.speed_x255 = command.diagnostics.applied_speed_x255;
      applied.flags = logging::kAppliedMotionFlagMotionGate |
                      logging::kAppliedMotionFlagGoalValid;
      if (command.diagnostics.any_goal_clamped) {
        applied.flags |= logging::kAppliedMotionFlagGoalClamped;
      }
      if (command.diagnostics.any_goal_unreachable) {
        applied.flags |= logging::kAppliedMotionFlagGoalUnreachable;
      }
      if (command.diagnostics.any_goal_reach_limited) {
        applied.flags |= logging::kAppliedMotionFlagGoalReachLimited;
      }
      g_goalClamped = command.diagnostics.any_goal_clamped;
      g_goalSeq = goal_sequence;
      g_goalValid = true;
      xSemaphoreGive(g_goalMutex);
    }
  } else {
    // A closed gate or maintenance-entry edge must never leave a stale frame
    // eligible for the Sync Write path.
    g_goalValid = false;
  }

  const bool maintenance_held = g_maintApi.lockHeld(now_ms);
  g_maintTargetApi.setLiveState(g_safetyState, maintenance_held);
  g_dxlJobApi.setLiveState(g_safetyState, maintenance_held);
}

void controlTask(void*) {
  fault_capture::markStartupStage(fault_capture::StartupStage::TasksRunning);
  TickType_t next = xTaskGetTickCount();
  g_controllerCore.reset();
  g_controlClock.reset();
  g_controllerConfig = controller::ControllerConfigSnapshot{};
  g_controllerConfigRevision = 0xFFFFFFFFu;

  for (;;) {
    tick(watchdog::TaskId::Control);

    const controller::ControllerTime cycle_time =
        controllerTimeFromFreeRtos(g_controlClock);
    const uint32_t now_ms = cycle_time.now_ms;

    // A host force-disarm is latched until SET_ARMING(arm) releases it. Revoke
    // any bench-mode intents at their owners so they cannot re-enter
    // maintenance/passive from Disarmed on the next control cycle.
    const bool host_disarm = g_controlApi.disarmRequested();
    if (host_disarm) {
      g_maintApi.revoke();
      g_passiveApi.clear();
    }
    // Mirror the live USB MaintenanceApi lock (the real ENTER/EXIT/HEARTBEAT
    // owner) into the arbiter so a held bench lock actually grants
    // MacMaintenance motion authority (hexapod_src-0y9: previously only the
    // arbiter's own never-acquired token counted, so the motion gate stayed
    // closed in maintenance and accepted joint targets were never actuated).
    //
    // TTL hold while a DXL job is in flight: a scan/ping burst busy-waits the
    // bus inside one dxlTask cycle (priority 3), starving apiTask (priority 2)
    // for up to ~1.2 s, so the host's 0.25 s heartbeats are processed too late
    // and the 1 s lock TTL lapsed mid-scan -- silently dropping MacMaintenance,
    // force-cutting DXL power, and rejecting every subsequent bench command.
    // The host provably just asked for this work, so keep the lock fed until
    // the job completes; normal TTL expiry resumes one window afterwards.
    {
      const protocol::dxljob::Slot js = g_dxlJobApi.queue().slotState();
      if (js == protocol::dxljob::Slot::Pending ||
          js == protocol::dxljob::Slot::Running) {
        g_maintApi.feedTtl(now_ms);
      }
    }
    const bool maint_held = g_maintApi.lockHeld(now_ms);

    refreshControllerConfigSnapshot();
  #if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
    // HIL observer requests arrive through the API-owned fixed ring and are
    // consumed here, after the firmware-owned config snapshot is current but
    // before the next ControllerCore boundary.
    consumeHilTraceRequests(hil::outputGuard().status());
  #endif
    collectControllerState(now_ms);
    collectControllerIntent(maint_held, host_disarm);

    watchdog::markControlProgress(80);
    g_controllerCore.step(g_controllerState, g_controllerIntent,
                          g_controllerConfig, cycle_time, g_controllerCommand);
    watchdog::markControlProgress(81);
  #if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
    // This is the parity capture point: the portable core has produced final
    // calibrated tick goals, while adapter-owned power/torque/goal effects
    // have not yet been published to the DXL boundary.
    g_hilTrace.captureStep(g_controllerState, g_controllerIntent,
                 g_controllerConfig, cycle_time, g_controllerCommand,
                 hil::outputGuard().status());
  #endif
    publishControllerCommand(now_ms);

    // --- Feature flags: publish availability and consume host intent --------
    // Reflect what the current hardware/state permits so FEATURE_SET can only
    // be honoured when it is safe (AGENTS.md 1.3). Engines that are not yet
    // wired report NotImplemented so the host gets an honest reason.
    updateFeatureFlags(now_ms);
    watchdog::markControlProgress(0);

    // If gait/IK work exceeded the 10 ms period, rebase before delaying.
    // Without this guard vTaskDelayUntil() returns immediately forever once
    // `next` falls behind, leaving priority-3 control continuously runnable
    // and starving lower-priority tasks.
    const TickType_t now_ticks = xTaskGetTickCount();
    if (static_cast<int32_t>(now_ticks - next) >=
        static_cast<int32_t>(pdMS_TO_TICKS(period_ms::kControl))) {
      next = now_ticks;
    }
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
  if (param == dxl::LogicalParam::TorqueEnable) {
    if (value != 0 && value != 1) return Code::VerifyFailed;
    if (!g_dxlBus.setTorqueOne(id, p->table_kind, value != 0)) {
      return Code::BusError;
    }
    bool torque_on = false;
    if (!g_dxlBus.torqueState(id, p->table_kind, torque_on)) {
      return Code::BusError;
    }
    readback = torque_on ? 1 : 0;
    verified = (readback == value);
    if (verified) {
      for (uint8_t i = 0; i < g_dxlBus.servoCount(); ++i) {
        if (g_dxlBus.profile(i).id == id) {
          const uint32_t bit = (uint32_t{1} << i);
          if (value == 0) {
            g_maintenanceReleasedMask |= bit;
          } else {
            g_maintenanceReleasedMask &= ~bit;
          }
          break;
        }
      }
    }
    return verified ? Code::Ok : Code::VerifyFailed;
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

void clearServoStatusSnapshot();

void completeDxlScanJob(uint8_t job_id) {
  uint8_t data[protocol::dxljob::kMaxResult];
  uint8_t len = 0;
  const uint8_t found = g_dxlBus.servoCount();
  data[len++] = found;
  for (uint8_t i = 0; i < found; ++i) {
    if (static_cast<uint8_t>(len + 6) > protocol::dxljob::kMaxResult) break;
    len = appendCompactServo(data, len, g_dxlBus.profile(i));
  }
  g_dxlJobApi.queue().complete(job_id, protocol::dxljob::Code::Ok, data, len);
}

// Advance a running maintenance scan by exactly one ID. Returning true means
// the queue slot was owned by a scan (whether it completed this cycle), so the
// caller must not claim another job.
bool advanceDxlScanJob() {
  if (!g_dxlScanJob.active) return false;

  if (g_dxlJobApi.queue().slotState() != protocol::dxljob::Slot::Running ||
      g_dxlJobApi.queue().currentJobId() != g_dxlScanJob.job_id) {
    g_dxlScanJob.active = false;
    g_dxlScanJob.cursor.reset();
    return false;
  }

  if (!board::dxlPowerEnabled()) {
    g_dxlJobApi.queue().complete(g_dxlScanJob.job_id,
                                 protocol::dxljob::Code::PowerOff, nullptr, 0);
    g_dxlScanJob.active = false;
    g_dxlScanJob.cursor.reset();
    return true;
  }

  watchdog::markProgress(21);
  g_dxlBus.discoverId(g_dxlScanJob.cursor.currentId());
  if (!g_dxlScanJob.cursor.advanceAfterPing()) {
    completeDxlScanJob(g_dxlScanJob.job_id);
    g_dxlScanJob.active = false;
  }
  return true;
}

// Execute one bounded DXL maintenance operation against the bus (dxlTask
// context only) and write the serialized result back to the queue. No-op when
// the queue is empty. A scan remains Running while advanceDxlScanJob() processes
// one ID per task cycle. Bus access requires DXL power on; when it is off the
// job is reported PowerOff rather than silently returning nothing.
void runQueuedDxlJob() {
  if (advanceDxlScanJob()) return;

  protocol::DxlJobRequest job;
  uint8_t job_id = 0;
  if (!g_dxlJobApi.queue().claim(job, job_id)) {
    return;  // nothing pending
  }

  uint8_t data[protocol::dxljob::kMaxResult];
  uint8_t len = 0;
  protocol::dxljob::Code code = protocol::dxljob::Code::Ok;

  // The output-disabled image rejects every DXL state-changing maintenance
  // request before a bus transaction. Count the logical operation that would
  // have reached the lower guard so GET_STATUS/health remains auditable.
  if (hil::outputDisabled()) {
    switch (job.type) {
      case protocol::dxljob::Type::Power:
        if (job.arg0 != 0) {
          board::setDxlPower(true);
          data[len++] = 0;
          data[len++] = board::hasDxlPowerControl() ? 1 : 0;
          g_dxlJobApi.queue().complete(
              job_id, protocol::dxljob::Code::OutputDisabled, data, len);
          return;
        }
        break;
      case protocol::dxljob::Type::Torque:
        if (job.arg0 != 0) {
          hil::outputGuard().allowGoalWrite();
          hil::outputGuard().allowTorque(true);
          data[len++] = 1;
          data[len++] = 0;
          g_dxlJobApi.queue().complete(
              job_id, protocol::dxljob::Code::OutputDisabled, data, len);
          return;
        }
        // The unpowered HIL image is already torque-off. Preserve the normal
        // safety-reducing operation as an explicit successful no-op.
        data[len++] = 0;
        data[len++] = 0;
        g_dxlJobApi.queue().complete(job_id, protocol::dxljob::Code::Ok, data,
                                     len);
        return;
      case protocol::dxljob::Type::SetParam:
      case protocol::dxljob::Type::SetLimits:
      case protocol::dxljob::Type::WriteReg:
        hil::outputGuard().allowDxlWrite();
        g_dxlJobApi.queue().complete(
            job_id, protocol::dxljob::Code::OutputDisabled, nullptr, 0);
        return;
      default:
        break;
    }
  }

  // The DXL power toggle is the one job allowed to run with power off: it owns
  // the power FET (board:: HAL). Handle it before the power-on gate below.
  // dxlTask is the sole owner of the board power line, so the toggle must run
  // here, never in apiTask. Result payload: [power_on(1), has_control(1)].
  if (job.type == protocol::dxljob::Type::Power) {
    if (!board::hasDxlPowerControl()) {
      // No software-controlled FET on this build (e.g. mkrzero fallback).
      g_dxlJobApi.queue().complete(job_id, protocol::dxljob::Code::Unsupported,
                                   nullptr, 0);
      return;
    }
    board::setDxlPower(job.arg0 != 0);
    uint8_t pd[2];
    pd[0] = board::dxlPowerEnabled() ? 1 : 0;
    pd[1] = board::hasDxlPowerControl() ? 1 : 0;
    g_dxlJobApi.queue().complete(job_id, protocol::dxljob::Code::Ok, pd, 2);
    return;
  }

  const bool power_on = board::dxlPowerEnabled();
  if (!power_on && job.type != protocol::dxljob::Type::None) {
    g_dxlJobApi.queue().complete(job_id, protocol::dxljob::Code::PowerOff,
                                 nullptr, 0);
    return;
  }

  switch (job.type) {
    case protocol::dxljob::Type::Scan: {
      g_dxlBus.beginDiscovery();
      clearServoStatusSnapshot();
      g_dxlScanJob.job_id = job_id;
      g_dxlScanJob.active = g_dxlScanJob.cursor.begin(job.arg0, job.arg1);
      if (!g_dxlScanJob.active) {
        g_dxlJobApi.queue().complete(job_id, protocol::dxljob::Code::Unsupported,
                                     nullptr, 0);
      }
      return;
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
      const uint8_t servo_count = g_dxlBus.servoCount();
      uint8_t acked = 0;
      if (on) {
        // Explicit all-servo arm (used by Center All) supersedes individual
        // maintenance releases.
        g_maintenanceReleasedMask = 0;
        // Torque-on is all-or-nothing: every discovered servo must have a
        // fresh, valid present position and receive Goal := Present before any
        // servo is energised. A partial snapshot must never enable the rest
        // against stale Goal Position registers.
        dxl::GoalTarget hold[dxl::DxlBus::kMaxServos];
        uint8_t hold_count = 0;
        if (!refreshDiscoveredPositions() ||
          !buildDiscoveredHoldTargets(hold, hold_count) ||
            !g_dxlBus.writeGoalPositions(hold, hold_count)) {
          code = protocol::dxljob::Code::BusError;
          data[len++] = 1;
          data[len++] = 0;
          break;
        }
        acked = g_dxlBus.setTorqueAll(true);
        if (acked != servo_count) {
          g_dxlBus.setTorqueAll(false);
          code = protocol::dxljob::Code::BusError;
        }
      } else {
        acked = g_dxlBus.setTorqueAll(false);
        if (acked != servo_count || !verifyDiscoveredTorqueOff()) {
          code = protocol::dxljob::Code::BusError;
        }
      }
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

void clearServoStatusSnapshot() {
  g_servoStatusCount = 0;
  g_poseKnownMask = 0;
  for (uint8_t i = 0; i < dxl::DxlBus::kMaxServos; ++i) {
    g_servoStatus[i] = dxl::ServoStatus{};
  }
}

void dxlTask(void*) {
  // Bring up the DXL UART once. This only initializes Serial1; it does NOT
  // enable DXL power (board HAL owns that) or servo torque, so it is safe at
  // boot. Scanning is deferred to a maintenance command once power is on.
  g_dxlBus.begin();
  TickType_t next = xTaskGetTickCount();
  bool prev_authorized = false;
  bool arming_discovery_started = false;
  bool arming_discovery_finished = false;
  bool arming_event_noted = false;
  uint8_t arming_discovery_index = 0;
  uint32_t arming_power_on_ms = 0;
  constexpr uint16_t kServoBootDelayMs = 500;
  // Sustained dead-bus window before declaring a hard bus fault (lmt.5).
  constexpr uint16_t kDxlBusFailLimit = 50;  // ~1 s at the 50 Hz dxl period
  dxl::FaultMonitor servo_fault_monitor;
  for (;;) {
    tick(watchdog::TaskId::Dxl);

    const uint32_t now_ms =
        static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
    const safety::State live_state =
        static_cast<safety::State>(g_safetyState);
    const bool arming = live_state == safety::State::ArmingChecks;

    // RC arming owns a bounded torque-off bring-up sequence. Power the bus,
    // wait for the servos to boot, discover exactly the configured IDs once,
    // then let the normal status reader populate fresh present positions. The
    // safety FSM remains in ArmingChecks until all 18 IDs and poses are known.
    // The arming event is queued for best-effort persistence (i2cTask drains
    // g_persistentEvents); arming NEVER waits on storage. Written straight to
    // the persistence queue at Info severity so it is not announced as an
    // operator-facing error.
    if (arming && !arming_event_noted) {
      arming_event_noted = true;
      safety::PersistentEvent arming_event;
      arming_event.timestamp_ms = now_ms;
      arming_event.code = safety::ErrorCode::ArmingStarted;
      arming_event.detail = g_commandSource;
      arming_event.severity = safety::ErrorSeverity::Info;
      taskENTER_CRITICAL();
      g_persistentEvents.push(arming_event);
      taskEXIT_CRITICAL();
    }
    if (arming) {
      if (!board::dxlPowerEnabled()) {
        watchdog::markProgress(10);
        board::setDxlPower(true);
        arming_power_on_ms = now_ms;
        arming_discovery_started = false;
        arming_discovery_finished = false;
        arming_discovery_index = 0;
        clearServoStatusSnapshot();
      }
      if ((now_ms - arming_power_on_ms) >= kServoBootDelayMs) {
        if (!arming_discovery_started) {
          if (configuredServoCoverageFromBus()) {
            arming_discovery_finished = true;
          } else {
            g_dxlBus.beginDiscovery();
            g_configuredServoCoverage = false;
            arming_discovery_index = 0;
            arming_discovery_started = true;
            arming_discovery_finished = false;
            clearServoStatusSnapshot();
          }
        }
        if (arming_discovery_started && !arming_discovery_finished) {
          const config::RobotConfig& cfg = g_configApi.config();
          watchdog::markProgress(11);
          const uint8_t discover_id = cfg.servos[arming_discovery_index].id;
          if (!g_dxlBus.discoverId(discover_id)) {
            noteError(safety::ErrorCode::DxlDiscoveryIncomplete, discover_id,
                      safety::ErrorSeverity::Error, now_ms);
          }
          ++arming_discovery_index;
          if (arming_discovery_index >= config::kNumServos) {
            arming_discovery_finished = true;
          }
          publishServoReadiness();
        }
        if (!arming_discovery_started) {
          publishServoReadiness();
        }
      }
    } else {
      arming_discovery_started = false;
      arming_discovery_finished = false;
      arming_discovery_index = 0;
      arming_event_noted = false;
    }

    // Execute at most one bounded DXL maintenance operation per cycle. A scan
    // stays in the queue's Running state and advances one ping per loop, so
    // absent IDs cannot starve the watchdog or lower-priority tasks. dxlTask
    // remains the sole bus owner; apiTask only enqueues (gated on
    // MacMaintenance + lock) and polls the result.
    watchdog::markProgress(20);
    runQueuedDxlJob();
    publishServoReadiness();

    // Safety force-off (4sa.1): retain DXL power only while the state policy
    // allows it. ArmingChecks uses a powered torque-off bus for discovery;
    // StandReady and motion states retain power; maintenance/passive keep their
    // existing behavior. Disarm, RC kill / host estop, or a fault cuts power.
    // board:: caches the FET state, so this is at most one digitalWrite per
    // transition; it is a no-op on builds without a software power FET
    // (board::dxlPowerEnabled() stays false there).
    if (board::dxlPowerEnabled() &&
        !safety::stateAllowsDxlPower(live_state)) {
      board::setDxlPower(false);
      clearServoStatusSnapshot();
    }

    // Enforce the safety gate at the bus level: the instant motion is no longer
    // permitted (RC kill, host estop, disarm, fault, or a non-motion state),
    // disable torque on all discovered servos so the robot stops. Edge-
    // triggered so we do not spam the bus every cycle. No-op until a maintenance
    // scan populates servos and DXL power is on (both OFF at boot), keeping this
    // safe by default. g_motionGate already folds in the safety state machine.
    const bool authorized = g_motionGate;
    const bool have_servos = g_dxlBus.servoCount() > 0;
    static bool torque_seed_pending = false;
    static bool torque_enable_fault = false;
    if (!authorized && prev_authorized && have_servos) {
      watchdog::markProgress(40);
      g_dxlBus.setTorqueAll(false);
    }
    if (!authorized) {
      torque_seed_pending = false;  // disarm any pending seed-then-enable
    }
    // Rising edge of authorisation: arm a seed-then-enable sequence. Before
    // torque is enabled we latch each servo's measured present position as its
    // goal (lmt.4 safety) so no servo snaps to a stale Goal Position register
    // when torque turns on. Present positions are read continuously below, so
    // the snapshot is normally already fresh on this edge; if it is not (cold
    // start right after a scan) we keep torque off and retry next cycle rather
    // than enable torque against an unknown goal.
    //
    // Bench exception (hexapod_src-0y9): when MacMaintenance owns motion the
    // operator controls torque explicitly via the DXL_TORQUE job (which does
    // its own seed-then-enable), so entering maintenance must NOT silently
    // stiffen the robot.
    const bool maint_authority =
        g_commandSource ==
        static_cast<uint8_t>(safety::CommandSource::MacMaintenance);
    if (authorized && !prev_authorized && have_servos && !maint_authority) {
      torque_seed_pending = true;
    }
    if (authorized && torque_seed_pending && have_servos) {
      static dxl::GoalTarget hold[config::kNumServos];
      watchdog::markProgress(50);
      if (buildConfiguredHoldTargets(hold) &&
          g_dxlBus.writeGoalPositions(hold, config::kNumServos)) {
        watchdog::markProgress(51);
        const uint8_t acked = g_dxlBus.setTorqueAll(true);
        if (acked == config::kNumServos) {
          torque_seed_pending = false;
          torque_enable_fault = false;
        } else {
          watchdog::markProgress(52);
          g_dxlBus.setTorqueAll(false);
          torque_enable_fault = true;
          noteError(safety::ErrorCode::DxlWriteFailed, 0,
                    safety::ErrorSeverity::Critical, now_ms);
        }
      } else {
        noteError(safety::ErrorCode::DxlWriteFailed, 0,
                  safety::ErrorSeverity::Error, now_ms);
      }
    }
    if (!board::dxlPowerEnabled()) {
      g_maintenanceReleasedMask = 0;
    } else if (live_state == safety::State::MacMaintenance &&
               g_maintenanceReleasedMask != 0) {
      // Reassert only when another firmware path changed the cached torque
      // state back to ON; this avoids writing torque-off every 20 ms.
      for (uint8_t i = 0; i < g_dxlBus.servoCount(); ++i) {
        const uint32_t bit = (uint32_t{1} << i);
        const dxl::ServoProfile& profile = g_dxlBus.profile(i);
        if ((g_maintenanceReleasedMask & bit) != 0 &&
            profile.torque_enabled) {
          g_dxlBus.setTorqueOne(profile.id, profile.table_kind, false);
        }
      }
    }
    prev_authorized = authorized;
    // Publish torque-off confirmation for the safety FSM (passive pose gating).
    // Power-off is inherently safe. While powered, require every configured
    // servo to be discovered as well as every discovered torque cache to be
    // OFF; an incomplete scan cannot prove that an unlisted powered servo is
    // limp. A maintenance DXL_TORQUE-on while motion is unauthorised therefore
    // never falsely opens passive entry.
    g_dxlTorqueOff = dxl::torqueOffConfirmed(
      board::dxlPowerEnabled(), g_configuredServoCoverage,
      have_servos && g_dxlBus.allTorqueOff());

    // Goal Sync-Write path (lmt.2): while motion is authorised and torque is on,
    // push the latest goal frame published by controlTask (gait -> body IK ->
    // servo map). Copy the frame out under a briefly-blocking mutex so the bus
    // is never held while the lock is taken; if the copy times out (controlTask
    // holding it longer than kGoalReadWaitMs, which never happens in practice)
    // the servos hold their previous goal. Writing every cycle (not just on
    // goal change) also keeps any per-servo bus watchdog fed.
    if (authorized && !torque_seed_pending && have_servos && g_goalValid &&
        g_goalMutex != nullptr) {
      static dxl::GoalTarget targets[config::kNumServos];
      uint8_t count = 0;
      // Bounded wait (not zero) to acquire the frame: controlTask publishes at
      // 100 Hz and holds this mutex only for a tiny fixed copy, but with both
      // tasks at equal priority a zero-wait take can phase-lock and miss the
      // hold window every cycle once the frame grows past ~15 joints (HIL:
      // 16-18 servo goal frames never actuated while 1-15 did). A short bounded
      // wait guarantees the reader observes each published frame; the hold is
      // microseconds, so this never approaches the 20 ms dxl period budget.
      if (xSemaphoreTake(g_goalMutex, pdMS_TO_TICKS(kGoalReadWaitMs)) ==
          pdTRUE) {
        const uint8_t frame_count = g_goalFrame.count;
        for (uint8_t i = 0; i < frame_count; ++i) {
          const uint8_t id = g_goalFrame.joints[i].id;
          if (maint_authority && maintenanceServoReleased(id)) {
            continue;
          }
          targets[count].id = id;
          targets[count].tick = g_goalFrame.joints[i].tick;
          ++count;
        }
        xSemaphoreGive(g_goalMutex);
      }
      if (count > 0) {
        watchdog::markProgress(60);
        if (!g_dxlBus.writeGoalPositions(targets, count)) {
          noteError(safety::ErrorCode::DxlWriteFailed, 0,
                    safety::ErrorSeverity::Error, now_ms);
        }
      }
    }

    // Publish a fresh present-position snapshot for all discovered servos.
    // Gated on BOTH a populated servo table (scan) and DXL power: reading an
    // unpowered bus is pure timeout busy-wait inside the library, and a
    // sustained busy-wait at dxlTask priority starves the health task's
    // hardware WDT pet and hard-resets the MCU (found on HIL when the
    // FaultHard power force-cut raced these reads, hexapod_src-2e8). Never
    // enables torque or writes goals; the goal Sync-Write path runs above.
    static uint16_t consec_zero_reads = 0;  // dead-bus counter (lmt.5)
    const bool arming_reads_ready =
        !arming || (arming_discovery_finished && g_configuredServoCoverage);
    if (!g_dxlScanJob.active && g_dxlBus.servoCount() > 0 && board::dxlPowerEnabled() &&
        arming_reads_ready) {
      watchdog::markProgress(70);
      const uint8_t n =
          g_dxlBus.syncReadStatus(g_servoStatus, dxl::DxlBus::kMaxServos);
      const uint8_t cnt = g_dxlBus.servoCount();
      // The snapshot covers the whole servo table (per-entry .ok marks
      // validity); syncReadStatus refreshes exactly one servo round-robin.
      // Any fresh read publishes the full table to the telemetry encoders.
      if (n > 0) {
        g_servoStatusCount = cnt;
      }

      // Round-robin one full per-servo read per cycle (eax.6). The all-servo
      // position refresh above carries Present Position only, so the detail
      // fields (velocity, load, voltage, temperature, torque-enable, hardware
      // error) are gathered one servo at a time. syncReadStatus preserves
      // these fields between cycles, so the servo_status stream converges over
      // servoCount() cycles. Read-only and torque-off-safe; bounded to one
      // servo per cycle and to DxlBus::kReadTimeoutMs per register read.
      // Skipped when this cycle's position transaction failed (n == 0): it
      // would only add more timeout work on a potentially dead bus.
      static uint8_t rr_servo = 0;
      if (n > 0 && !arming) {
        if (rr_servo >= cnt) rr_servo = 0;
        const dxl::ServoProfile& rr_profile = g_dxlBus.profile(rr_servo);
        const uint8_t rr_id = rr_profile.id;
        watchdog::markProgress(71);
        const bool detail_ok =
            g_dxlBus.readStatus(rr_id, g_servoStatus[rr_servo]);
        g_servoStatus[rr_servo].id = rr_id;  // readStatus does not set id
        if (detail_ok) {
          servo_fault_monitor.observe(
              rr_servo, rr_profile.table_kind,
              g_servoStatus[rr_servo].hardware_error);
        } else {
          noteError(safety::ErrorCode::DxlReadFailed, rr_id,
                    safety::ErrorSeverity::Warning, now_ms);
        }
        ++rr_servo;
      }

      // Hard-fault detection (lmt.5): publish g_dxlHardFault for the safety FSM.
      // A servo hardware-error bit (MX 2.0 Hardware Error Status; converges via
      // the detail read) is an immediate hard fault. A powered bus that fails
      // kDxlBusFailLimit consecutive bounded position transactions is a hard
      // bus failure; intermittent per-servo failures keep arming checks from
      // passing but do not falsely classify the entire bus as dead.
      if (n == 0) {
        if (consec_zero_reads < 0xFFFF) ++consec_zero_reads;
        if (consec_zero_reads == kDxlBusFailLimit) {
          noteError(safety::ErrorCode::DxlBusSilent, 5,
                    safety::ErrorSeverity::Critical, now_ms);
        }
      } else {
        consec_zero_reads = 0;
      }
      g_dxlHardFault = torque_enable_fault || servo_fault_monitor.faulted() ||
                       (consec_zero_reads >= kDxlBusFailLimit);
    } else {
      // No scanned servos or DXL power is off: there is no powered bus to
      // fault on (FaultHard itself force-cuts power above, so keeping the
      // fault asserted here would deadlock CLEAR_FAULT recovery).
      consec_zero_reads = 0;
      if (!board::dxlPowerEnabled()) {
        torque_enable_fault = false;
        servo_fault_monitor.reset();
      }
      g_dxlHardFault = torque_enable_fault;
    }
    publishServoReadiness();
    watchdog::markProgress(0);

    // Overrun guard: bus reads on a powered-but-unresponsive bus busy-wait
    // their timeouts inside the library, and a cycle that overruns the period
    // makes vTaskDelayUntil() return immediately -- dxlTask (priority 3) then
    // runs back-to-back and starves the lower-priority health task, whose
    // withheld hardware-WDT pet resets the MCU (hexapod_src-2e8 HIL). Resync
    // the schedule after an overrun so this task always truly blocks for one
    // period, guaranteeing lower-priority tasks CPU time.
    {
      const TickType_t now_ticks = xTaskGetTickCount();
      if (static_cast<int32_t>(now_ticks - next) >=
          static_cast<int32_t>(pdMS_TO_TICKS(period_ms::kDxl))) {
        next = now_ticks;  // schedule slipped a full period; rebase
      }
    }
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kDxl));
  }
}

void rcTask(void*) {
  crsf::RcStatus rc_status;
  crsf::initRcStatus(rc_status);
  crsf::initRcStatus(g_rcStatus);
  taskENTER_CRITICAL();
  g_rcMailbox.reset();
  taskEXIT_CRITICAL();
  g_bridge.reset();
#if defined(PIN_SERIAL3_RX)
  // Serial3 is the ExpressLRS CRSF receiver link; rcTask owns it exclusively.
  Serial3.begin(kCrsfBaud);
#endif
  TickType_t next = xTaskGetTickCount();
  constexpr uint16_t kMaxBytesPerCycle = 128;
  uint32_t applied_calibration_revision = 0xFFFFFFFFu;
  uint32_t applied_gait_tune_revision = 0xFFFFFFFFu;
  uint32_t last_status_tx_ms = 0;
  uint32_t last_attitude_tx_ms = 0;
  uint32_t last_battery_tx_ms = 0;
  for (;;) {
    tick(watchdog::TaskId::Rc);
    const uint32_t now_ms =
        static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
    crsf::ChannelData frame;
    bool fresh = false;
#if defined(PIN_SERIAL3_RX)
    uint16_t bytes_processed = 0;
    while (Serial3.available() > 0 &&
           bytes_processed < kMaxBytesPerCycle) {
      const uint8_t b = static_cast<uint8_t>(Serial3.read());
      ++bytes_processed;
      if (g_crsfParser.push(b, frame)) {
        // Feed the raw 11-bit ChannelPack ticks to the bridge decoder.
        g_bridge.update(frame.channels, /*link_up=*/true, now_ms);
        fresh = true;
      }
    }
#endif
    // Trip failsafe if no fresh frame arrived within the timeout (or the link
    // has never been seen) -- forces a safe disarmed/estop hold.
    g_bridge.evaluateFailsafe(now_ms);
    config::RcInputCalibration pending_calibration;
    uint32_t pending_calibration_revision = 0;
    bool calibration_pending = false;
    taskENTER_CRITICAL();
    calibration_pending = g_pendingRcCalibrationValid;
    if (calibration_pending) {
      pending_calibration = g_pendingRcCalibration;
      pending_calibration_revision = g_pendingRcCalibrationRevision;
    }
    taskEXIT_CRITICAL();
    if (calibration_pending &&
        pending_calibration_revision != applied_calibration_revision) {
      g_bridge.setCalibration(pending_calibration);
      applied_calibration_revision = pending_calibration_revision;
    }
    // Adopt a host-staged binding map before publishing this cycle's command so
    // a SET_BINDINGS takes effect immediately (rcTask is the sole writer of the
    // bridge; apiTask only stages).
    if (g_ctrlPendingBindingsValid) {
      g_bridge.setBindings(g_ctrlPendingBindings);
      g_ctrlPendingBindingsValid = false;
    }
    // Seed the NAV1-edited gait shape from the persisted defaults whenever the
    // known-good config changes (boot adopt, host commit, handset save), so the
    // handset always starts from what is actually stored. The bridge ignores
    // this while the operator has the editor open.
    {
      const uint32_t gait_rev = g_configApi.revision();
      if (gait_rev != applied_gait_tune_revision) {
        const config::GaitDefaults& d = g_configApi.config().gait;
        g_bridge.setGaitTuneFractions(
            static_cast<float>(d.step_height_mm) /
                static_cast<float>(config::kMaxGaitStepMm),
            static_cast<float>(d.stride_len_mm) /
                static_cast<float>(config::kMaxGaitStrideMm),
            static_cast<float>(d.duty_x255) / 255.0f);
        applied_gait_tune_revision = gait_rev;
      }
    }
    // Build one complete RC snapshot before publishing it. controlTask either
    // copies this whole frame or retains its prior coherent frame for a cycle.
    const controller::ControllerCommand command = g_bridge.command();
    deriveRcStatus(command, frame, fresh, now_ms, rc_status);
    controller::RcFrameSnapshot rc_snapshot;
    rc_snapshot.command = command;
    rc_snapshot.status = rc_status;
    rc_snapshot.frame_sequence = g_crsfParser.framesDecoded();
    rc_snapshot.published_ms = now_ms;
    taskENTER_CRITICAL();
    g_rcMailbox.publish(rc_snapshot);
    taskEXIT_CRITICAL();

    // Legacy telemetry and USB-controller views retain their existing latest
    // snapshots. The control path above never reads these independently.
    const ChannelPackInputs_t raw_inputs = g_bridge.rawInputs();
    g_ctrlCmd = command;
    g_rcStatus = rc_status;
    // Publish the raw inputs + active bindings for the controller USB API /
    // controller_state telemetry (apiTask is a reader only).
    g_ctrlRaw = raw_inputs;
    g_ctrlBindings = g_bridge.bindings();
    // Publish the raw RC diagnostics snapshot (a8n). Raw ticks refresh only on a
    // fresh frame (so a dropout freezes the last-known sticks); the parser's
    // frame-health counters and link statistics are always current.
    if (fresh) {
      for (uint8_t i = 0; i < crsf::kNumChannels; ++i) {
        g_rcDiag.raw_ticks[i] = frame.channels[i];
      }
    }
    g_rcDiag.frames_decoded = g_crsfParser.framesDecoded();
    g_rcDiag.crc_errors = g_crsfParser.crcErrors();
    g_rcDiag.link_stats_count = g_crsfParser.linkStatsCount();
    g_rcDiag.has_link_stats = g_crsfParser.hasLinkStats();
    if (g_rcDiag.has_link_stats) {
      g_rcDiag.link_stats = g_crsfParser.linkStats();
    }
    g_rcDiag.last_frame_ms = rc_status.last_frame_ms;
    g_rcDiag.ever_seen = rc_status.ever_seen;
    g_rcDiag.failsafe = rc_status.failsafe;

    // Emit at most one frame per RC cycle. UART buffer capacity is checked by
    // sendCrsfTelemetry(), so downlink telemetry never blocks RC parsing.
    if (rc_status.ever_seen && !rc_status.failsafe) {
      crsf::telemetry::HexapodStatus radio;
      ImuSnapshot imu;
      taskENTER_CRITICAL();
      radio = g_radioStatus;
      imu = g_imuSnapshot;
      taskEXIT_CRITICAL();
      const bool imu_fresh = imu.present && imu.valid &&
                             (now_ms - imu.sample_ms) <= kImuFreshMs;
      if (imu.present) radio.flags |= crsf::telemetry::flag::kImuPresent;
      if (imu_fresh) radio.flags |= crsf::telemetry::flag::kImuFresh;
      radio.imu_calibration = imu.calibration;

      uint8_t payload[crsf::telemetry::kHexapodPayloadBytes];
      // While the operator edits gait parameters the status frame IS the UI
      // feedback loop, so send it at 20 Hz instead of 5 Hz for the duration.
      const uint32_t status_period_ms = g_radioStatusBoost
                                            ? kCrsfStatusBoostPeriodMs
                                            : kCrsfStatusPeriodMs;
      if ((now_ms - last_status_tx_ms) >= status_period_ms) {
        crsf::telemetry::encodeHexapodStatus(radio, payload);
        if (sendCrsfTelemetry(crsf::telemetry::kFrameTypeHexapodStatus,
                              payload,
                              crsf::telemetry::kHexapodPayloadBytes)) {
          last_status_tx_ms = now_ms;
        }
      } else if (imu_fresh &&
                 (now_ms - last_attitude_tx_ms) >= kCrsfAttitudePeriodMs) {
        crsf::telemetry::encodeAttitude(
            imu.pitch_cdeg, imu.roll_cdeg, imu.yaw_cdeg, payload);
        if (sendCrsfTelemetry(crsf::telemetry::kFrameTypeAttitude, payload,
                              crsf::telemetry::kAttitudePayloadBytes)) {
          last_attitude_tx_ms = now_ms;
        }
      } else if ((radio.flags & crsf::telemetry::flag::kBatteryValid) != 0 &&
                 (now_ms - last_battery_tx_ms) >= kCrsfBatteryPeriodMs) {
        crsf::telemetry::encodeBattery(
            radio.battery_mv, batteryPercent(radio.battery_mv), payload);
        if (sendCrsfTelemetry(crsf::telemetry::kFrameTypeBattery, payload,
                              crsf::telemetry::kBatteryPayloadBytes)) {
          last_battery_tx_ms = now_ms;
        }
      }
    }
    const TickType_t now_ticks = xTaskGetTickCount();
    if (static_cast<int32_t>(now_ticks - next) >=
        static_cast<int32_t>(pdMS_TO_TICKS(period_ms::kRc))) {
      next = now_ticks;
    }
    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kRc));
  }
}

void apiTask(void*) {
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
  constexpr uint8_t kMaxFramesPerCycle = 4;
  constexpr uint16_t kMaxBytesPerCycle = protocol::kMaxWireFrame;
  for (;;) {
    tick(watchdog::TaskId::Api);

    const uint32_t api_loop_now_ms =
      static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
  #if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
    refreshHilObserver(api_loop_now_ms, hil::outputGuard().status());
    queueHilObserverRequests();
  #endif

    // Adopt a persisted config once i2cTask has loaded one at boot. Done here
    // (not at task start) because i2cTask's scan may finish after this task
    // begins; the ConfigApi shadow is thus only ever touched by apiTask.
    if (g_bootLoad.ready && !g_bootLoad.consumed) {
      g_configApi.adoptPayload(g_bootLoad.payload, g_bootLoad.len);
      g_bootLoad.consumed = true;
    }

    // Handset "save gait settings". controlTask already checked that RC holds
    // authority and the robot is not moving; here we only run the transaction
    // and report the outcome through the deduplicated error journal so the
    // handset sees a rejection reason instead of a silent no-op.
    if (g_gaitSaveRequested) {
      config::GaitDefaults pending;
      taskENTER_CRITICAL();
      pending = g_gaitSavePending;
      taskEXIT_CRITICAL();
      const config::CfgResult result = g_configApi.saveGaitDefaults(pending);
      if (result != config::CfgResult::Ok) {
        safety::GaitSaveReject reason = safety::GaitSaveReject::StoreFailed;
        if (result == config::CfgResult::Volatile) {
          reason = safety::GaitSaveReject::NotPersistent;
        } else if (result == config::CfgResult::ValidationFailed) {
          reason = safety::GaitSaveReject::Invalid;
        }
        noteError(safety::ErrorCode::GaitSaveRejected,
                  static_cast<uint8_t>(reason),
                  safety::ErrorSeverity::Warning, api_loop_now_ms);
      }
      taskENTER_CRITICAL();
      g_gaitSaveRequested = false;
      taskEXIT_CRITICAL();
    }

    // Drain any received bytes, framing them into complete request bodies.
    uint8_t frames_processed = 0;
    uint16_t bytes_processed = 0;
    while (Serial.available() > 0 &&
           frames_processed < kMaxFramesPerCycle &&
           bytes_processed < kMaxBytesPerCycle) {
      const uint8_t b = static_cast<uint8_t>(Serial.read());
      ++bytes_processed;
      if (!g_frameReader.push(b)) {
        continue;
      }
      ++frames_processed;

      // Refresh the live status snapshot for this request.
      protocol::api::StatusSnapshot st;
      st.uptime_ms =
          static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
      st.state = g_safetyState;
      st.dxl_power = board::dxlPowerEnabled();
      st.dxl_power_control = board::hasDxlPowerControl();
      st.battery_mv = g_batteryMv;  // snapshot; ADC is controlTask-only
      st.watchdog_missed = watchdog::missedMask();
      st.reset_cause = g_resetCause;
      st.last_reset_watchdog_missed = watchdog::lastResetMissedMask();
      st.last_reset_progress_marker = watchdog::lastResetProgressMarker();
      st.control_stack_free_words = static_cast<uint16_t>(
          uxTaskGetStackHighWaterMark(
              g_handles[static_cast<uint8_t>(watchdog::TaskId::Control)]));
      st.dxl_stack_free_words = static_cast<uint16_t>(
          uxTaskGetStackHighWaterMark(
              g_handles[static_cast<uint8_t>(watchdog::TaskId::Dxl)]));
      st.last_reset_control_progress = watchdog::lastResetControlProgress();
      st.last_reset_safety_state = watchdog::lastResetSafetyState();
      st.dxl_power_transitions = board::dxlPowerTransitions();
      const hil::OutputGuardStatus guard_status = hil::outputGuard().status();
      st.hil_flags = packHilFlags(guard_status);
      st.blocked_power_enable = guard_status.blocked_power_enable;
      st.blocked_torque_enable = guard_status.blocked_torque_enable;
      st.blocked_goal_write = guard_status.blocked_goal_write;
      st.blocked_dxl_write = guard_status.blocked_dxl_write;
      st.last_goal_sequence = guard_status.last_goal_sequence;
      st.last_goal_count = guard_status.last_goal_count;
      st.last_fault_reason = g_lastFaultReason;
      st.last_fault_timestamp_ms = g_lastFaultTimestampMs;
            const fault_capture::Snapshot& fatal = fault_capture::lastSnapshot();
            st.last_fatal_reason = static_cast<uint8_t>(fatal.reason);
            st.last_fatal_stage = static_cast<uint8_t>(fatal.stage);
            memcpy(st.last_fatal_task_name, fatal.task_name,
              sizeof(st.last_fatal_task_name));
            st.last_fault_stack_pointer = fatal.stack_pointer;
            st.last_fault_exception_return = fatal.exception_return;
            st.last_fault_registers[0] = fatal.r0;
            st.last_fault_registers[1] = fatal.r1;
            st.last_fault_registers[2] = fatal.r2;
            st.last_fault_registers[3] = fatal.r3;
            st.last_fault_registers[4] = fatal.r12;
            st.last_fault_registers[5] = fatal.lr;
            st.last_fault_registers[6] = fatal.pc;
            st.last_fault_registers[7] = fatal.xpsr;

      // Give the maintenance lock the current time + live state so ENTER can
      // gate on a safe state and TTL/heartbeat use the same clock as the
      // control task.
      g_maintApi.setNow(st.uptime_ms);
      g_maintApi.setLiveState(g_safetyState);

      const bool config_lock =
          g_safetyState == static_cast<uint8_t>(safety::State::MacMaintenance) &&
          g_maintApi.lockHeld(st.uptime_ms);
#if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
      // A trace references one immutable config revision. Captures are short
      // and configuration remains readable, but staging/commit is rejected
      // until the terminal trace record has been delivered.
      const bool trace_locks_config =
          g_hilObserver.captureActive() || g_hilTrace.active() ||
          g_hilTrace.terminalPending();
#else
      const bool trace_locks_config = false;
#endif
      g_configApi.setMutationPolicy(config_lock && !trace_locks_config,
                                    config_lock && !trace_locks_config &&
                                        g_dxlTorqueOff);

      // Refresh the passive handler's live state so PASSIVE_ENTER gates on the
      // current safety state (control task also keeps this current each cycle).
      g_passiveApi.setLiveState(g_safetyState);

      // Refresh the controller API's reported snapshot from rcTask's published
      // decoded command + raw inputs + active bindings (oha.4). Read-only
      // copies; the bridge itself stays owned by rcTask.
      g_controllerApi.setSnapshot(g_ctrlCmd, g_ctrlRaw);
      g_controllerApi.setBindings(g_ctrlBindings);

      // Report honest runtime capabilities: feature_bits mirrors the per-
      // feature availability the control task publishes each cycle (bit index
      // == Feature enum order), so GET_CAPABILITIES is no longer a zero stub
      // (4sa.4). g_deviceInfo is only touched by this task.
      g_deviceInfo.feature_bits = g_featureApi.availableMask();
      g_deviceInfo.build_flags =
          (st.hil_flags & protocol::api::hilflag::kOutputDisabled) != 0
          ? protocol::api::capabilityflag::kHilOutputDisabled
          : 0;

      // Update the observer immediately before dispatching a HIL command so
      // its independent TTL and the maintenance token are evaluated from this
      // exact request-time snapshot. Generic/API activity never refreshes
      // either maintenance or Jetson authority.
    #if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
      refreshHilObserver(st.uptime_ms, guard_status);
      queueHilObserverRequests();
    #endif

      protocol::DecodeStatus decode_st = protocol::DecodeStatus::Ok;
      const size_t n = protocol::api::handleRequest(
          g_frameReader.body(), g_frameReader.length(), g_deviceInfo, st, out,
          sizeof(out), &g_configApi, &g_subs, &g_controlApi, &g_motionApi,
          &g_maintApi, &g_maintTargetApi, &g_dxlJobApi, &g_featureApi,
          &g_sensorApi, &g_passiveApi, &g_controllerApi, &decode_st,
#if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
          &g_hilObserver);
#else
          nullptr);
#endif
    #if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
      queueHilObserverRequests();
    #endif
      if (decode_st != protocol::DecodeStatus::Ok) {
        ++g_apiRxBad;  // corrupt frame: count it so api_stats makes it visible
      }
      if (n > 0) {
        if (!txFrame(out, n)) {
          g_subs.noteTxBacklog();  // USB TX could not take the response frame
        }
      }

      // If a CONTROLLER_SET_BINDINGS just validated a new map, stage it for
      // rcTask to adopt (it owns the bridge). Drop if a previous stage is still
      // pending to keep the single-slot hand-off lock-free.
      if (!g_ctrlPendingBindingsValid) {
        controller::BindingConfig nb;
        if (g_controllerApi.takePending(&nb)) {
          g_ctrlPendingBindings = nb;
          g_ctrlPendingBindingsValid = true;
        }
      }
    }

    // Emit any due telemetry frames for subscribed streams. The subscription
    // manager enforces each stream's rate and counts missed slots as dropped;
    // when the USB CDC TX buffer cannot accept a frame we count a backlog drop
    // instead of blocking the task (AGENTS.md 6.3 rate-limited subscriptions).
    const uint32_t now_ms = api_loop_now_ms;
  #if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
    // The trace transport has priority and never runs from controlTask. A
    // failed fragment turns into an explicit terminal failure; it never causes
    // a controller step to be delayed, decimated, or silently dropped.
    (void)emitHilTraceRecord(now_ms, out, sizeof(out));
    const bool suppress_optional_telemetry =
      g_hilObserver.captureActive() || g_hilTrace.active() ||
      g_hilTrace.terminalPending();
  #endif
    for (uint8_t i = 0; i < protocol::kNumStreams; ++i) {
      const protocol::StreamId s = static_cast<protocol::StreamId>(i);
  #if defined(HEXAPOD_HIL_OUTPUT_DISABLED)
      if (suppress_optional_telemetry && s != protocol::StreamId::Health &&
        s != protocol::StreamId::HilStatus) {
      continue;
      }
  #endif
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
      if (!txFrame(out, fn)) {
        // Endpoint is wedged (host stopped draining): count the drop and stop
        // emitting for this cycle rather than stalling on every stream.
        g_subs.noteTxBacklog();
        break;
      }
    }

    const TickType_t now_ticks = xTaskGetTickCount();
    if (static_cast<int32_t>(now_ticks - next) >=
        static_cast<int32_t>(pdMS_TO_TICKS(period_ms::kApi))) {
      next = now_ticks;
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
  // Keep the wire position stable; this legacy field now reports whether the
  // replacement config storage (Qwiic OpenLog) is present and SD-ready.
  s.eeprom_present = g_i2cTopology.openlog_present ? 1 : 0;
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
  // fault_capture::init() copied the retained previous-reset record before any
  // peripheral or RTOS work. Check it before discovery so the first possible
  // SD operation is an attempt to preserve that crash.
  g_retainedCrashLogPending =
      fault_capture::lastSnapshot().reason != fault_capture::FatalReason::None;
  g_watchdogResetLogPending = (g_resetCause & 0x20u) != 0;

  // Bring up only the root bus first. Channel scans, config bootstrap, sensors,
  // and OLED initialization are lower priority than retained crash persistence.
  g_i2cBus.begin();
  i2c::initTopology(g_i2cTopology);
  g_i2cBus.scanRoot(g_i2cTopology);
  if (g_i2cTopology.openlog_present) {
    g_configStorageAvailable =
        g_openlog.begin(g_i2cTopology.openlog_address);
    g_i2cTopology.openlog_present = g_configStorageAvailable;
    if (g_configStorageAvailable && g_retainedCrashLogPending) {
      (void)tryPersistRetainedCrash();
    }
    if (g_configStorageAvailable && !g_retainedCrashLogPending &&
        g_watchdogResetLogPending) {
      (void)tryPersistWatchdogReset();
    }
  }

  g_configBootstrapPending = g_configStorageAvailable;
  if (!g_retainedCrashLogPending && !g_watchdogResetLogPending &&
      g_configBootstrapPending) {
    (void)tryBootstrapConfig(g_configApi.config());
  }

  // Only after the crash/config attempts, discover muxed devices. OLED begin
  // stays deferred until the steady-state loop so its graphics call chain can
  // never prevent the first diagnostic/config file writes.
  g_i2cBus.scanChannels(g_i2cTopology);
  g_oledInitPending = g_i2cTopology.oled_present;
  g_footPresentMask = i2c::footSensorPresentMask(g_i2cTopology);
  const bool imu_present = g_bno085.begin();
  taskENTER_CRITICAL();
  g_imuSnapshot.present = imu_present;
  g_imuSnapshot.valid = false;
  g_imuSnapshot.sample_ms = 0;
  taskEXIT_CRITICAL();
  publishTopologySnapshot();
  g_i2cLastUpdateMs = static_cast<uint32_t>(xTaskGetTickCount()) *
                      portTICK_PERIOD_MS;

  // Seed the contact estimator with the compiled-default foot calibration so it
  // is usable immediately (per-foot classification stays disabled until a
  // calibration enables it; raw values still stream as telemetry).
  {
    // Kept off this task's stack (it is ~360 bytes) and fully repopulated by
    // ConfigApi construction before the scheduler, so use the existing stable
    // default shadow instead of allocating another RobotConfig copy.
    const config::RobotConfig& boot_config = g_configApi.config();
    sensors::ContactParams params;  // conservative defaults
    g_contact.configure(boot_config.feet, params);
    // Seed the motion intent with the compiled-default gait parameters so the
    // first SET_BODY_TWIST has a sane baseline (host SET_GAIT_PARAMS overrides).
    g_motionApi.setDefaults(
        boot_config.gait.gait, boot_config.gait.body_height_mm,
        boot_config.gait.stride_len_mm, boot_config.gait.step_height_mm,
        boot_config.gait.duty_x255, boot_config.gait.speed_x255);
  }
  // Boot discovery and config seeding are complete: a config (persisted or
  // compiled default) is now available, so the safety machine may leave
  // ConfigLoad. apiTask adopts any persisted payload independently.
  g_configReady = true;
  TickType_t next = xTaskGetTickCount();
  // Loop period, runtime-tunable via SENSOR_SET_RATE (lmt.9). Starts at the
  // nominal 50 Hz; a host rate request derives and clamps a new period below.
  uint32_t i2c_period_ms = period_ms::kI2c;
  for (;;) {
    tick(watchdog::TaskId::I2c);
    bool storage_io_this_pass = false;

    static uint32_t next_crash_attempt_ms = 0;
    static uint32_t next_watchdog_attempt_ms = 0;
    static uint32_t next_bootstrap_attempt_ms = 0;
    const uint32_t boot_storage_now_ms = millis();
    if (g_retainedCrashLogPending && g_configStorageAvailable &&
        static_cast<int32_t>(boot_storage_now_ms - next_crash_attempt_ms) >= 0) {
      storage_io_this_pass = true;
      if (!tryPersistRetainedCrash()) {
        next_crash_attempt_ms = boot_storage_now_ms + 1000u;
      }
    }
    if (!g_retainedCrashLogPending && g_watchdogResetLogPending &&
        g_configStorageAvailable && !storage_io_this_pass &&
        static_cast<int32_t>(boot_storage_now_ms - next_watchdog_attempt_ms) >= 0) {
      storage_io_this_pass = true;
      if (!tryPersistWatchdogReset()) {
        next_watchdog_attempt_ms = boot_storage_now_ms + 1000u;
      }
    }
    if (!g_retainedCrashLogPending && !g_watchdogResetLogPending &&
        g_configBootstrapPending &&
        !storage_io_this_pass &&
        static_cast<int32_t>(boot_storage_now_ms - next_bootstrap_attempt_ms) >= 0) {
      storage_io_this_pass = true;
      if (!tryBootstrapConfig(g_configApi.config())) {
        next_bootstrap_attempt_ms = boot_storage_now_ms + 1000u;
      }
    }
    const bool boot_storage_pending = g_configStorageAvailable &&
      (g_retainedCrashLogPending || g_watchdogResetLogPending ||
       g_configBootstrapPending);

    if (!boot_storage_pending && g_oledInitPending && !storage_io_this_pass) {
      if (g_i2cTopology.mux_present) g_i2cBus.selectNone();
      if (!g_debugOled.begin(g_i2cTopology.oled_address)) {
        g_i2cTopology.oled_present = false;
      }
      g_oledInitPending = false;
      storage_io_this_pass = true;
    }

    // Service a config commit handed over by apiTask. i2cTask is the sole owner
    // of OpenLog, so the append-and-sync transaction happens here.
    bool do_commit = false;
    if (g_commitMutex != nullptr) {
      if (xSemaphoreTake(g_commitMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        do_commit = g_commit.requested;
        xSemaphoreGive(g_commitMutex);
      }
    }
    if (do_commit && !boot_storage_pending) {
      storage_io_this_pass = true;
      const bool ok = g_configStore.commit(g_commit.payload, g_commit.len);
      if (xSemaphoreTake(g_commitMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_commit.ok = ok;
        g_commit.requested = false;
        xSemaphoreGive(g_commitMutex);
        if (ok) g_configVolatile = false;  // a valid slot now exists
        xSemaphoreGive(g_commitDone);
      }
    }

    // BTN_4 toggles capture; BTN_1 + BTN_2 held for three seconds sends a
    // separate start-only sequence. Both are monotonic so they survive task
    // boundaries and packet coalescing.
    logging::RemoteCapture capture_remote = buildRemoteCapture(
      g_ctrlCmd, g_ctrlRaw, g_rcDiag.frames_decoded, g_ctrlCmd.frame_ms);
    const uint32_t toggle_delta =
        capture_remote.capture_toggle_seq - g_capture.handled_toggle_seq;
    const uint32_t start_delta =
        capture_remote.capture_start_seq - g_capture.handled_start_seq;
    if (!boot_storage_pending && !storage_io_this_pass &&
        (toggle_delta != 0 || start_delta != 0)) {
      g_capture.handled_toggle_seq = capture_remote.capture_toggle_seq;
      g_capture.handled_start_seq = capture_remote.capture_start_seq;
      const bool start_requested = start_delta != 0 && !g_capture.recording;
      const bool toggle_requested = (toggle_delta & 1u) != 0;
      if (start_requested || toggle_requested) {
        const uint32_t capture_now_ms = millis();
        if (!g_capture.recording) {
          if (!g_configStorageAvailable) {
            noteError(safety::ErrorCode::CaptureUnavailable, 0,
                      safety::ErrorSeverity::Warning, capture_now_ms);
          } else {
            g_capture.session = capture_now_ms;
            g_capture.sample = 0;
            g_capture.completed_samples = 0;
            g_capture.sample_file_size = 0;
            g_capture.sample_bytes = 0;
            g_capture.row = kCaptureNoRow;
            const size_t len = logging::formatCaptureMarker(
                true, g_capture.session, capture_now_ms, 0, g_captureLine,
                sizeof(g_captureLine));
            const bool ok = len > 0 && appendOpenLogLine(
                kCaptureFile, g_captureLine, len, true);
            storage_io_this_pass = true;
            if (ok) {
              g_capture.recording = true;
              g_capture.next_sample_ms = capture_now_ms;
            } else {
              noteError(safety::ErrorCode::CaptureWriteFailed, 1,
                        safety::ErrorSeverity::Error, capture_now_ms);
            }
          }
        } else {
          const size_t len = logging::formatCaptureMarker(
              false, g_capture.session, capture_now_ms,
              g_capture.completed_samples, g_captureLine,
              sizeof(g_captureLine));
          const bool ok = len > 0 && appendOpenLogLine(
              kCaptureFile, g_captureLine, len, true);
          storage_io_this_pass = true;
          g_capture.recording = false;
          g_capture.row = kCaptureNoRow;
          if (!ok) {
            noteError(safety::ErrorCode::CaptureWriteFailed, 4,
                      safety::ErrorSeverity::Error, capture_now_ms);
          }
        }
      }
    }

    if (!boot_storage_pending && g_capture.recording && !storage_io_this_pass) {
      const uint32_t capture_now_ms = millis();
      bool row_ok = true;
      if (g_capture.row == kCaptureNoRow &&
          static_cast<int32_t>(capture_now_ms - g_capture.next_sample_ms) >= 0) {
        bool exists = false;
        if (g_openlog.fileSize(kCaptureFile, g_capture.sample_file_size,
                               exists) && exists) {
          ++g_capture.sample;
          g_capture.sample_ms = capture_now_ms;
          g_capture.sample_bytes = 0;
          g_capture.remote = buildRemoteCapture(
              g_ctrlCmd, g_ctrlRaw, g_rcDiag.frames_decoded, g_ctrlCmd.frame_ms);
          g_capture.servo_count = g_servoStatusCount > config::kNumServos
                                      ? config::kNumServos
                                      : g_servoStatusCount;
          g_capture.servo_index = 0;
          const dxl::ServoMap map(g_configApi.config());
          for (uint8_t index = 0; index < g_capture.servo_count; ++index) {
            g_capture.servos[index] = g_servoStatus[index];
            const config::ServoConfig* servo =
                map.servoForId(g_capture.servos[index].id);
            g_capture.present_angle_centideg[index] = servo == nullptr
                ? 0
                : servoAngleCentideg(map, servo->leg, servo->joint,
                                     g_capture.servos[index].present_position);
          }

          g_capture.goal_valid = false;
          g_capture.goal_count = 0;
          g_capture.goal_index = 0;
          logging::AppliedMotionCapture& motion = g_capture.motion;
          motion.goal_sequence = g_goalSeq;
          motion.body_height_mm = 0;
          motion.stride_mm = 0;
          motion.step_height_mm = 0;
          motion.command_source = g_commandSource;
          motion.safety_state = g_safetyState;
          motion.gait = 0;
          motion.duty_x255 = 0;
          motion.speed_x255 = 0;
          motion.flags = 0;
          if (g_goalMutex != nullptr &&
              xSemaphoreTake(g_goalMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            if (g_goalValid) {
              g_capture.goal_valid = true;
              g_capture.goal_count = g_goalFrame.count > config::kNumServos
                  ? config::kNumServos
                  : g_goalFrame.count;
              g_capture.motion = g_goalFrame.applied;
              for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
                g_capture.legs[leg] = g_goalFrame.legs[leg];
              }
              for (uint8_t index = 0; index < g_capture.goal_count; ++index) {
                const gait::PipelineJoint& joint = g_goalFrame.joints[index];
                logging::GoalCapture& goal = g_capture.goals[index];
                goal.id = joint.id;
                goal.leg = joint.leg;
                goal.joint = joint.joint;
                goal.goal_tick = joint.tick;
                goal.goal_angle_centideg =
                    servoAngleCentideg(map, joint.leg, joint.joint, joint.tick);
                goal.flags = joint.clamped ? logging::kGoalCaptureFlagClamped : 0;
              }
            }
            xSemaphoreGive(g_goalMutex);
          }
          g_capture.row = kCaptureRowRemote;
        } else {
          row_ok = false;
        }
      }

      if (row_ok && g_capture.row == kCaptureRowRemote) {
        const size_t len = logging::formatRemoteCaptureRow(
            g_capture.session, g_capture.sample, g_capture.remote, g_captureLine,
            sizeof(g_captureLine));
        row_ok = len > 0 && appendOpenLogLine(
            kCaptureFile, g_captureLine, len, false);
        storage_io_this_pass = true;
        if (row_ok) g_capture.sample_bytes += static_cast<uint16_t>(len);
        if (row_ok) g_capture.row = kCaptureRowControl;
      } else if (row_ok && g_capture.row == kCaptureRowControl) {
        const size_t len = logging::formatAppliedMotionCaptureRow(
            g_capture.session, g_capture.sample, g_capture.sample_ms,
            g_capture.motion, g_captureLine, sizeof(g_captureLine));
        row_ok = len > 0 && appendOpenLogLine(
            kCaptureFile, g_captureLine, len, false);
        storage_io_this_pass = true;
        if (row_ok) {
          g_capture.sample_bytes += static_cast<uint16_t>(len);
          g_capture.row = g_capture.goal_valid ? kCaptureRowLegs
                                                : kCaptureRowServos;
        }
      } else if (row_ok && g_capture.row == kCaptureRowLegs) {
        const size_t len = logging::formatLegCaptureRow(
            g_capture.session, g_capture.sample, g_capture.sample_ms,
            g_capture.motion.goal_sequence, g_capture.legs, config::kNumLegs,
            g_captureLine, sizeof(g_captureLine));
        row_ok = len > 0 && appendOpenLogLine(
            kCaptureFile, g_captureLine, len, false);
        storage_io_this_pass = true;
        if (row_ok) {
          g_capture.sample_bytes += static_cast<uint16_t>(len);
          g_capture.row = kCaptureRowGoals;
        }
      } else if (row_ok && g_capture.row == kCaptureRowGoals) {
        const uint8_t remaining =
            static_cast<uint8_t>(g_capture.goal_count - g_capture.goal_index);
        const uint8_t count = remaining < kCaptureGoalsPerRow
                                  ? remaining
                                  : kCaptureGoalsPerRow;
        const size_t len = logging::formatGoalCaptureRow(
            g_capture.session, g_capture.sample, g_capture.sample_ms,
            g_capture.motion.goal_sequence,
            &g_capture.goals[g_capture.goal_index], count, g_captureLine,
            sizeof(g_captureLine));
        row_ok = len > 0 && appendOpenLogLine(
            kCaptureFile, g_captureLine, len, false);
        storage_io_this_pass = true;
        if (row_ok) {
          g_capture.sample_bytes += static_cast<uint16_t>(len);
          g_capture.goal_index =
              static_cast<uint8_t>(g_capture.goal_index + count);
          if (g_capture.goal_index >= g_capture.goal_count) {
            g_capture.row = kCaptureRowServos;
          }
        }
      } else if (row_ok && g_capture.row == kCaptureRowServos &&
                 g_capture.servo_index < g_capture.servo_count) {
        const dxl::ServoStatus& servo =
            g_capture.servos[g_capture.servo_index];
        const size_t len = logging::formatServoCaptureRow(
            g_capture.session, g_capture.sample, g_capture.sample_ms, servo,
            g_capture.present_angle_centideg[g_capture.servo_index],
            g_captureLine, sizeof(g_captureLine));
        row_ok = len > 0 && appendOpenLogLine(
            kCaptureFile, g_captureLine, len, false);
        storage_io_this_pass = true;
        if (row_ok) {
          g_capture.sample_bytes += static_cast<uint16_t>(len);
          ++g_capture.servo_index;
        }
      }

      if (row_ok && g_capture.row == kCaptureRowServos &&
          g_capture.servo_index >= g_capture.servo_count) {
        row_ok = g_openlog.sync() && openLogFileGrew(
            kCaptureFile, g_capture.sample_file_size, g_capture.sample_bytes);
        storage_io_this_pass = true;
        if (row_ok) {
          ++g_capture.completed_samples;
          g_capture.row = kCaptureNoRow;
          g_capture.next_sample_ms = g_capture.sample_ms + kCaptureIntervalMs;
        }
      }
      if (!row_ok) {
        g_capture.recording = false;
        g_capture.row = kCaptureNoRow;
        noteError(safety::ErrorCode::CaptureWriteFailed, 2,
                  safety::ErrorSeverity::Error, capture_now_ms);
      }
    }

    // Persist one deduplicated warning/error at a time. Keep the queue head on
    // failure and retry slowly so missing/failed media cannot starve sensors.
    if (!boot_storage_pending && g_configStorageAvailable &&
      !storage_io_this_pass) {
      static uint32_t next_log_attempt_ms = 0;
      const uint32_t log_now_ms = millis();
      if (static_cast<int32_t>(log_now_ms - next_log_attempt_ms) >= 0) {
        safety::PersistentEvent event;
        bool has_event = false;
        taskENTER_CRITICAL();
        has_event = g_persistentEvents.peek(event);
        taskEXIT_CRITICAL();
        if (has_event) {
          storage_io_this_pass = true;
          char line[96];
          const size_t len = safety::formatEventLog(event, line, sizeof(line));
          const bool written = len > 0 && appendEventLogLine(line, len);
          if (written) {
            taskENTER_CRITICAL();
            g_persistentEvents.pop();
            taskEXIT_CRITICAL();
            next_log_attempt_ms = log_now_ms;
          } else {
            next_log_attempt_ms = log_now_ms + 1000u;
          }
        }
      }
    }

    // Re-apply the active config to i2c-owned consumers when it changes (boot
    // adopt / CFG_COMMIT, lmt.7). apiTask updates the config shadow; we re-seed
    // the contact estimator foot calibration and the motion-intent gait
    // defaults from the new known-good config, and push the persisted feature
    // default set into the feature api. i2cTask owns these consumers' boot
    // seeding, so it owns the refresh too (the gait pipeline + body IK are
    // refreshed in controlTask). Watched by revision so it runs once per change.
    static uint32_t applied_cfg_rev = 0xFFFFFFFFu;
    const uint32_t cfg_rev = g_configApi.revision();
    if (cfg_rev != applied_cfg_rev) {
      const config::RobotConfig& cfg = g_configApi.config();
      sensors::ContactParams params;  // conservative defaults
      g_contact.configure(cfg.feet, params);
      g_motionApi.setDefaults(cfg.gait.gait, cfg.gait.body_height_mm,
                              cfg.gait.stride_len_mm, cfg.gait.step_height_mm,
                              cfg.gait.duty_x255, cfg.gait.speed_x255);
      g_featureApi.applyDefaults(cfg.feature_defaults);
      applied_cfg_rev = cfg_rev;
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
    static uint32_t last_imu_sample_ms = 0;

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
          // Calibrating a foot also activates it: a freshly-baselined foot with
          // a usable threshold set should start classifying contact instead of
          // staying AIR (lmt.9 / audit 22l.7). setEnabled refuses feet without a
          // usable calibration, so an unconfigured foot stays disabled.
          g_contact.setEnabled(i, true);
        }
      }
      applied_calibrate_seq = want_cal;
    }

    // Apply a host-requested sensor poll rate (SENSOR_SET_RATE). apiTask stages
    // the rate; i2cTask owns the loop timing, so it derives the loop period from
    // the requested Hz, clamped to a safe window so a host cannot starve the
    // loop (which also services config commits) or spin it pointlessly fast
    // (lmt.9 / audit 22l.7). Unset (rate 0) keeps the nominal period.
    static uint32_t applied_rate_seq = 0;
    const uint32_t want_rate_seq = g_sensorApi.rateSeq();
    if (want_rate_seq != applied_rate_seq) {
      const uint16_t hz = g_sensorApi.sensorRateHz();
      if (hz > 0) {
        uint32_t ms = 1000u / hz;
        if (ms < period_ms::kI2cMinMs) ms = period_ms::kI2cMinMs;
        if (ms > period_ms::kI2cMaxMs) ms = period_ms::kI2cMaxMs;
        i2c_period_ms = ms;
      }
      applied_rate_seq = want_rate_seq;
    }

    // Poll one foot sensor per iteration (round-robin) so each pass does bounded
    // Wire work and the control loop is never stalled by a slow/missing board
    // (AGENTS.md 1.1 / 5.4). The mux requires exclusive one-hot channel select;
    // we select, read, then deselect so root devices stay addressable.
    if (g_sensorPollingEnabled && g_i2cTopology.mux_present) {
      const uint8_t ch = poll_ch;
      poll_ch = static_cast<uint8_t>((poll_ch + 1) % i2c::kNumFootChannels);
      const bool present =
          (g_footPresentMask & static_cast<uint8_t>(1u << ch)) != 0;
      if (present) {
        if (g_i2cBus.selectChannel(ch)) {
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
        } else {
          // Mux channel select failed: feed a failed sample so the foot's fault
          // counter advances and telemetry reflects the I2C problem promptly,
          // instead of waiting only for the staleness timeout (lmt.9 / audit
          // 22l.7). Drop the cached power-up so the board is reconfigured if the
          // mux recovers.
          const uint8_t bit = static_cast<uint8_t>(1u << ch);
          configured_mask = static_cast<uint8_t>(configured_mask & ~bit);
          g_contact.update(ch, sensors::FootSample{}, millis());
          g_i2cBus.selectNone();  // best-effort: leave the mux in a known state
        }
      }
    }

    const uint32_t sensor_now_ms = millis();
    if (g_bno085.present() &&
        (sensor_now_ms - last_imu_sample_ms) >= 50) {
      last_imu_sample_ms = sensor_now_ms;
      if (g_i2cTopology.mux_present) g_i2cBus.selectNone();
      const sensors::ImuSample sample = g_bno085.read();
      taskENTER_CRITICAL();
      g_imuSnapshot.present = true;
      g_imuSnapshot.valid = sample.ok;
      if (sample.ok) {
        g_imuSnapshot.pitch_cdeg = sample.pitch_cdeg;
        g_imuSnapshot.roll_cdeg = sample.roll_cdeg;
        g_imuSnapshot.yaw_cdeg = sample.yaw_cdeg;
        g_imuSnapshot.calibration = sample.calibration;
        g_imuSnapshot.sample_ms = sensor_now_ms;
      }
      taskEXIT_CRITICAL();
    }
    // Decay any silent foot to STALE and republish the snapshot every pass.
    g_contact.tickStaleness(millis());
    for (uint8_t i = 0; i < sensors::kNumFeet; ++i) {
      g_footState[i] = g_contact.foot(i);
    }
    // Mirror the fused foot state into the SensorApi snapshot (SENSOR_GET_STATUS).
    publishStatusSnapshot();
    g_i2cLastUpdateMs = static_cast<uint32_t>(xTaskGetTickCount()) *
              portTICK_PERIOD_MS;

    // A changed snapshot schedules all eight pages. At most one page is sent
    // per pass so the optional display cannot monopolize the shared bus.
    if (!boot_storage_pending && g_i2cTopology.oled_present &&
      g_debugOled.ready()) {
      static sensors::DebugDisplayState displayed;
      static sensors::DebugDisplayState pending;
      static uint8_t next_page = sensors::QwiicDebugOled::kPageCount;
      static uint32_t last_refresh_ms = 0;
      sensors::DebugDisplayState latest;
      taskENTER_CRITICAL();
      latest = g_debugDisplayState;
      taskEXIT_CRITICAL();
      latest.servo_count = g_servoStatusCount;
      latest.foot_present_mask = g_footPresentMask;
      latest.i2c_error = g_i2cBus.stats().last_error;
      latest.dxl_power = board::dxlPowerEnabled();
      latest.dxl_hard_fault = g_dxlHardFault;
      latest.config_storage = g_configStorageAvailable && !g_configVolatile;
      latest.mux_present = g_i2cTopology.mux_present;
      latest.capture_recording = g_capture.recording;
      latest.capture_samples = g_capture.completed_samples;
      latest.view = static_cast<uint8_t>((millis() / 10000u) %
                     sensors::QwiicDebugOled::kViewCount);
      const uint32_t display_now_ms = millis();
      const bool display_allowed =
          g_safetyState != static_cast<uint8_t>(safety::State::ArmingChecks);
      if (display_allowed &&
          next_page >= sensors::QwiicDebugOled::kPageCount &&
          (last_refresh_ms == 0 ||
           (display_now_ms - last_refresh_ms) >= 1000u) &&
          memcmp(&latest, &displayed, sizeof(latest)) != 0) {
        pending = latest;
        next_page = 0;
      }
        if (display_allowed && !storage_io_this_pass &&
          next_page < sensors::QwiicDebugOled::kPageCount) {
        if (g_debugOled.drawPage(next_page, pending)) {
          ++next_page;
          if (next_page == sensors::QwiicDebugOled::kPageCount) {
            displayed = pending;
            last_refresh_ms = display_now_ms;
          }
        } else {
          g_i2cTopology.oled_present = false;
        }
      }
    }

    vTaskDelayUntil(&next, pdMS_TO_TICKS(i2c_period_ms));
  }
}

void healthTask(void*) {
  TickType_t next = xTaskGetTickCount();
  uint32_t last_watchdog_eval_ms = 0;
  for (;;) {
    tick(watchdog::TaskId::Health);

    const uint32_t now_ms =
        static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
    if (last_watchdog_eval_ms == 0 ||
        (now_ms - last_watchdog_eval_ms) >= 500u) {
      watchdog::evaluate();
      last_watchdog_eval_ms = now_ms;
    }
    status_led::Inputs led;
    led.state = static_cast<safety::State>(g_safetyState);
    led.fault = static_cast<safety::FaultReason>(g_faultReason);
    led.configured_servo_coverage = g_configuredServoCoverage;
    led.all_servo_poses_known = g_poseKnownMask == kAllServoPosesKnown;
    led.watchdog_stalled = watchdog::criticalStalled();
    board::setUserLed(status_led::ledOn(led, now_ms));

    vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms::kHealth));
  }
}

}  // namespace

void setResetCause(uint8_t cause) { g_resetCause = cause; }

void start() {
  watchdog::init(g_resetCause);
  initDeviceInfo();

  // Sync primitives for the apiTask <-> i2cTask config commit hand-off. Created
  // here at boot (before the scheduler), not at runtime.
  g_commitMutex = xSemaphoreCreateMutex();
  g_commitDone = xSemaphoreCreateBinary();

  // Guards the controlTask -> dxlTask servo goal frame (lmt.1).
  g_goalMutex = xSemaphoreCreateMutex();

  // Fatal hooks run with scheduler/peripheral state unknown. Never let them
  // block forever flushing USB; the library's LED code remains deterministic.
  vSetErrorLed(board::pinUserLed(), HIGH);
  vSetErrorSerial(nullptr);

  // Task stacks are allocated once here, at boot, before the scheduler runs;
  // there is no runtime heap churn afterward (AGENTS.md 1.2).
  const BaseType_t control_created =
  xTaskCreate(controlTask, "control", stack_words::kControl, nullptr,
              priority::kControl, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Control)]);
  const BaseType_t dxl_created =
  xTaskCreate(dxlTask, "dxl", stack_words::kDxl, nullptr,
              priority::kDxl, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Dxl)]);
  const BaseType_t rc_created =
  xTaskCreate(rcTask, "rc", stack_words::kRc, nullptr,
              priority::kRc, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Rc)]);
  const BaseType_t api_created =
  xTaskCreate(apiTask, "api", stack_words::kApi, nullptr,
              priority::kApi, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Api)]);
  const BaseType_t i2c_created =
  xTaskCreate(i2cTask, "i2c", stack_words::kI2c, nullptr,
              priority::kI2c, &g_handles[static_cast<uint8_t>(watchdog::TaskId::I2c)]);
  const BaseType_t health_created =
  xTaskCreate(healthTask, "health", stack_words::kHealth, nullptr,
              priority::kHealth, &g_handles[static_cast<uint8_t>(watchdog::TaskId::Health)]);

  const bool allocations_ok = g_commitMutex != nullptr &&
                              g_commitDone != nullptr &&
                              g_goalMutex != nullptr &&
                              control_created == pdPASS && dxl_created == pdPASS &&
                              rc_created == pdPASS && api_created == pdPASS &&
                              i2c_created == pdPASS && health_created == pdPASS;
  if (!allocations_ok) {
    board::setDxlPower(false);
    for (;;) {
      board::toggleUserLed();
      delay(100);
    }
  }

  // Hands the CPU to the scheduler; does not return.
  fault_capture::markStartupStage(
      fault_capture::StartupStage::SchedulerStarting);
  vTaskStartScheduler();
}

}  // namespace app
