#pragma once

// ===========================================================================
// Portable telemetry payload encoders (no Arduino / FreeRTOS / peripheral deps).
//
// The per-stream payload byte layouts for the four "kinematic" telemetry
// streams used to live inline in app/tasks.cpp::buildTelemetry, which is
// target-only code (it reads global cross-task snapshots and takes the goal
// mutex). That made the concrete servo_status / joint_state / servo_goals /
// leg_state wire layouts impossible to unit-test on the host (audit 22l.9 /
// lmt.12).
//
// These free functions take plain snapshot inputs and write the exact same
// little-endian payloads buildTelemetry produced, so:
//   * app/tasks.cpp keeps only the task glue (snapshot selection + the goal
//     mutex) and calls these encoders, and
//   * test/test_telemetry_encode verifies the byte layouts against the wire
//     schema (and the Python decoders in protocol/python/.../telemetry.py).
//
// Every encoder returns the number of payload bytes written and never exceeds
// the documented per-stream maximum (all <= the 256-byte payload cap).
// ===========================================================================

#include <stdint.h>

#include "dxl/dxl_status.h"
#include "dxl/servo_map.h"
#include "gait/gait_pipeline.h"
#include "protocol/maintenance_target_api.h"

namespace protocol {

// servo_status: count(1) then 14 bytes/servo: id(1), present_position(u32),
// present_velocity(i16), present_load(i16), present_voltage_mv(u16),
// present_temperature_c(i8), hardware_error(1), torque_enabled(1).
// 18 servos -> 1 + 18*14 = 253 bytes.
uint16_t encodeServoStatus(const dxl::ServoStatus* servos, uint8_t count,
                           uint8_t* out);

// joint_state: count(1) then 4 bytes/joint: leg(1), joint(1),
// angle_centideg(i16). Present ticks are mapped through `map` (sign/trim/center)
// to URDF-zero-relative angles; servos whose id is not in the map are skipped,
// so the emitted count may be < `count`. 18 joints -> 1 + 18*4 = 73 bytes.
uint16_t encodeJointState(const dxl::ServoMap& map,
                          const dxl::ServoStatus* servos, uint8_t count,
                          uint8_t* out);

// servo_goals from a live gait/IK pipeline frame: count(1) then 5 bytes/joint:
// leg(1), joint(1), angle_centideg(i16), flags(1) where bit0 = clamped. Takes
// the resolved joint array + count directly (works for any goal-frame holder).
// 18 joints -> 1 + 18*5 = 91 bytes.
uint16_t encodeServoGoals(const dxl::ServoMap& map,
                          const gait::PipelineJoint* joints, uint8_t count,
                          uint8_t* out);

// servo_goals from the maintenance target set (bench fallback): same 5-byte
// layout. Only joints with a stored command are emitted; unmapped joints are
// skipped.
uint16_t encodeServoGoals(const dxl::ServoMap& map,
                          const MaintTargetSet& targets, uint8_t* out);

// leg_state: count(1) then 8 bytes/leg: leg(1), foot_x(i16), foot_y(i16),
// foot_z(i16, mm body frame), flags(1) where bit0 = reachable, bit1 = clamped.
// Only legs with a recorded SET_LEG_TARGET attempt are emitted.
// 6 legs -> 1 + 6*8 = 49 bytes.
uint16_t encodeLegState(const MaintTargetSet& targets, uint8_t* out);

}  // namespace protocol
