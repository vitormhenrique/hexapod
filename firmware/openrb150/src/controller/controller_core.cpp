#include "controller_core.h"

#include <math.h>

#include "command_frame.h"

namespace controller {
namespace {

constexpr uint32_t kAllServoPosesKnown =
    (1u << config::kNumServos) - 1u;
constexpr uint8_t kContactConfidenceMinimum = 128;
constexpr uint8_t kMinimumConfidentFeet = 4;
// Leg used to demonstrate a gait-parameter change from the handset. Index 0 is
// "leg 1" in the config/servo numbering (DXL ids 1/7/13).
constexpr int8_t kGaitTunePreviewLeg = 0;

config::GaitId gaitFromWire(uint8_t gait) {
  if (gait > static_cast<uint8_t>(config::GaitId::Crawl)) {
    return config::GaitId::Stand;
  }
  return static_cast<config::GaitId>(gait);
}

bool rcCommandIsActive(const ControllerCommand& command) {
  return command.valid &&
         (fabsf(command.twist_vx) > 0.05f ||
          fabsf(command.twist_vy) > 0.05f ||
          fabsf(command.twist_wz) > 0.05f ||
          fabsf(command.pose_x_mm) > 2.0f ||
          fabsf(command.pose_y_mm) > 2.0f ||
          fabsf(command.pose_z_mm) > 2.0f ||
          fabsf(command.pose_roll) > 0.02f ||
          fabsf(command.pose_pitch) > 0.02f ||
          fabsf(command.pose_yaw) > 0.02f ||
          command.trick != TrickId::None);
}

bool sticksAreActive(const ControllerCommand& command) {
  return fabsf(command.twist_vx) > 0.12f ||
         fabsf(command.twist_vy) > 0.12f ||
         fabsf(command.twist_wz) > 0.12f ||
         fabsf(command.pose_x_mm) > 5.0f ||
         fabsf(command.pose_y_mm) > 5.0f ||
         fabsf(command.pose_z_mm) > 5.0f ||
         fabsf(command.pose_roll) > 0.05f ||
         fabsf(command.pose_pitch) > 0.05f ||
         fabsf(command.pose_yaw) > 0.05f;
}

gait::BodyPose poseFromMotion(const protocol::MotionIntent& motion) {
  gait::BodyPose pose;
  pose.x_mm = motion.pose_x_mm;
  pose.y_mm = motion.pose_y_mm;
  pose.z_mm = motion.pose_z_mm;
  pose.roll = motion.pose_roll;
  pose.pitch = motion.pose_pitch;
  pose.yaw = motion.pose_yaw;
  return pose;
}

// Rotate an operator command-frame pose (x forward, y left, roll about
// forward, pitch about left) into URDF body frame B for the body IK.
gait::BodyPose commandPoseToBody(const gait::BodyPose& command) {
  gait::BodyPose body = command;
  commandPlanarToBody(command.x_mm, command.y_mm, body.x_mm, body.y_mm);
  commandAttitudeToBody(command.roll, command.pitch, body.roll, body.pitch);
  return body;
}

}  // namespace

config::GaitId rcGaitFromIndex(uint8_t gait_index) {
  static const config::GaitId kGaits[3] = {
      config::GaitId::Wave,
      config::GaitId::Ripple,
      config::GaitId::Tripod,
  };
  if (gait_index > 2) gait_index = 2;
  return kGaits[gait_index];
}

ControllerCore::ControllerCore()
    : pipeline_(config_cache_.snapshot().robot),
      servo_map_(config_cache_.snapshot().robot) {
  body_command_shaper_.configure(
      config_cache_.snapshot().robot.body_command);
  reset();
}

void ControllerCore::reset() {
  state_machine_.reset();
  arbiter_.reset();
  trick_engine_.reset();
  pipeline_.reconfigure();
  pipeline_.resetPhase();
  body_command_shaper_.configure(config_cache_.snapshot().robot.body_command);
  body_command_shaper_.reset(
      static_cast<float>(config_cache_.snapshot().robot.gait.body_height_mm));
  applied_intent_sequence_ = 0xFFFFFFFFu;
  applied_gait_ = 0xFF;
  idle_seen_intent_sequence_ = 0xFFFFFFFFu;
  previous_rc_trick_ = TrickId::None;
  previous_motion_gate_ = false;
  previous_maintenance_authority_ = false;
}

void ControllerCore::step(const RobotState& state,
                          const ControllerIntent& intent,
                          const ControllerConfigSnapshot& config,
                          const ControllerTime& time,
                          RobotCommand& command) {
  command = RobotCommand{};
  command.diagnostics.intent_sequence = intent.motion.seq;
  command.diagnostics.config_revision = config_cache_.snapshot().revision;

  // A controller cannot infer elapsed time. Fail closed until an adapter
  // supplies a valid monotonic sample, without advancing any controller state.
  if (!time.valid) {
    command.safety_state = state_machine_.state();
    command.fault_reason = state_machine_.faultReason();
    return;
  }

  bool config_usable = false;
  if (config.valid) {
    const ConfigSnapshotUpdate update = config_cache_.apply(
        config.robot, config.revision, config.persistent);
    config_usable = update != ConfigSnapshotUpdate::Rejected;
    if (update == ConfigSnapshotUpdate::Updated) {
      pipeline_.reconfigure();
      const ControllerConfigSnapshot& updated_config = config_cache_.snapshot();
      body_command_shaper_.configure(updated_config.robot.body_command);
      body_command_shaper_.reset(
          static_cast<float>(updated_config.robot.gait.body_height_mm));
      applied_intent_sequence_ = 0xFFFFFFFFu;
      applied_gait_ = 0xFF;
      command.diagnostics.config_reapplied = true;
    }
  }
  config_usable = config_usable && state.config_ready;
  const ControllerConfigSnapshot& active_config = config_cache_.snapshot();
  command.diagnostics.config_revision = active_config.revision;

  if (intent.jetson_heartbeat_received &&
      intent.features.jetson_control_enabled) {
    arbiter_.jetsonHeartbeat(time.now_ms);
  }
  arbiter_.setHostEstop(intent.host_estop);
  arbiter_.setExternalMacLock(intent.maintenance.lock_held,
                              intent.maintenance.lock_token);

  safety::RcInputs rc_inputs;
  rc_inputs.ever_seen = intent.rc.ever_seen;
  rc_inputs.kill = intent.rc.kill;
  rc_inputs.armed = intent.rc.armed;
  rc_inputs.autonomy_enabled = intent.rc.autonomy_enabled;
  const safety::ArbiterOutput& arbiter =
      arbiter_.update(rc_inputs, time.now_ms);

  uint8_t confident_feet = 0;
  for (uint8_t leg = 0; leg < sensors::kNumFeet; ++leg) {
    const sensors::LegContactState& foot = state.contact.feet[leg];
    if (!foot.stale && !foot.fault &&
        foot.confidence >= kContactConfidenceMinimum) {
      ++confident_feet;
    }
  }
  command.diagnostics.confident_contact_feet = confident_feet;

  const bool readiness_current =
      state.dxl.config_revision == active_config.revision;
  const bool servo_coverage =
      readiness_current && state.dxl.configured_servo_coverage;
  const bool pose_known =
      readiness_current && state.dxl.pose_known_mask == kAllServoPosesKnown;

  safety::StateInputs safety_inputs;
  safety_inputs.config_loaded = config_usable;
  safety_inputs.battery_mv = state.battery.millivolts;
  safety_inputs.battery_valid = state.battery.valid;
  safety_inputs.watchdog_fault = state.watchdog_fault;
  safety_inputs.dxl_hard_fault = state.dxl.hard_fault;
  safety_inputs.host_estop = intent.host_estop;
  safety_inputs.rc_kill = intent.rc.kill;
  safety_inputs.rc_failsafe = intent.rc.failsafe;
  safety_inputs.rc_ever_seen = intent.rc.ever_seen;
  safety_inputs.rc_armed = intent.rc.armed;
  safety_inputs.arming_checks_pass =
      config_usable && state.battery.valid && servo_coverage && pose_known &&
      !state.dxl.hard_fault;
  safety_inputs.host_disarm = intent.host_disarm;
  safety_inputs.command_source = static_cast<uint8_t>(arbiter.source);
  safety_inputs.jetson_fresh = arbiter_.jetsonFresh(time.now_ms);
  safety_inputs.rc_autonomy = intent.rc.autonomy_enabled;
  safety_inputs.mac_lock_held =
      arbiter_.macLockHeld(time.now_ms) || intent.maintenance.lock_held;
  safety_inputs.maintenance_request = intent.maintenance.lock_held;
  safety_inputs.passive_request = intent.passive_requested;
  safety_inputs.torque_off = state.dxl.torque_off;
  safety_inputs.contact_enabled = intent.features.foot_contact_enabled;
  safety_inputs.contact_confident =
      intent.features.foot_contact_enabled &&
      confident_feet >= kMinimumConfidentFeet;
  const bool host_motion_changed =
      intent.motion.seq != idle_seen_intent_sequence_;
  idle_seen_intent_sequence_ = intent.motion.seq;
  safety_inputs.motion_active = rcCommandIsActive(intent.rc.command) ||
                                host_motion_changed || trick_engine_.active();

  if (intent.clear_fault_requested) state_machine_.requestClearFault();
  const safety::State safety_state =
      state_machine_.update(safety_inputs, time.now_ms);

  command.safety_state = safety_state;
  command.fault_reason = state_machine_.faultReason();
  command.command_source = arbiter.source;
  command.motion_authorized = arbiter.motion_authorized;
  command.allow_dxl_power =
      config_usable && safety::stateAllowsDxlPower(safety_state);
  command.allow_torque =
      config_usable && safety::stateAllowsTorque(safety_state);
  command.motion_gate =
      config_usable && safety::stateAllowsMotion(safety_state) &&
      arbiter.motion_authorized;
  command.diagnostics.motion_gate_rising =
      command.motion_gate && !previous_motion_gate_;
  command.diagnostics.motion_gate_falling =
      !command.motion_gate && previous_motion_gate_;

  // RC gait-tune editor state is reported (and its save request evaluated)
  // outside the motion branch: the safest moment to write EEPROM is while the
  // robot is disarmed, and the handset must keep showing the live values then.
  {
    const ControllerCommand& rc_cmd = intent.rc.command;
    const bool rc_authority = arbiter.source == safety::CommandSource::Rc;
    command.diagnostics.gait_tune_active =
        rc_authority && rc_cmd.gait_tune_active;
    command.diagnostics.gait_tune_param =
        static_cast<uint8_t>(rc_cmd.gait_tune_param);
    // Never persist while the robot is moving (AGENTS.md 4.3: no config commits
    // during active walking). The adapter applies the final store checks.
    command.diagnostics.gait_save_seq = rc_cmd.gait_tune_save_seq;
    command.diagnostics.gait_save_requested =
        rc_authority && !rcCommandIsActive(rc_cmd) && !trick_engine_.active();
  }

  if (intent.host_disarm ||
      safety_state >= safety::State::FaultSoft) {
    command.diagnostics.clear_maintenance_lock = true;
    command.diagnostics.clear_passive_request = true;
  }

  const bool maintenance_authority =
      command.motion_gate &&
      arbiter.source == safety::CommandSource::MacMaintenance;
  if (maintenance_authority && !previous_maintenance_authority_) {
    // The adapter owns the persisted target API, so it performs the clear on
    // this explicit edge before a subsequent step can honor new targets.
    command.diagnostics.maintenance_session_started = true;
    command.diagnostics.clear_maintenance_targets = true;
    trick_engine_.cancel();
    applied_intent_sequence_ = 0xFFFFFFFFu;
    applied_gait_ = 0xFF;
  } else if (maintenance_authority &&
             intent.maintenance.control_mode ==
                 protocol::MaintControlMode::JointTargets) {
    const protocol::MaintTargetSet& targets = intent.maintenance.targets;
    for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
      for (uint8_t joint = 0; joint < config::kJointsPerLeg; ++joint) {
        if (!targets.set[leg][joint]) continue;
        const config::ServoConfig* servo = servo_map_.servoFor(leg, joint);
        if (servo == nullptr) continue;
        gait::PipelineJoint& goal =
            command.goals.joints[command.goals.count++];
        goal.id = servo->id;
        goal.tick = targets.tick[leg][joint];
        goal.leg = leg;
        goal.joint = joint;
        goal.clamped = targets.clamped[leg][joint];
        if (goal.clamped) command.diagnostics.any_goal_clamped = true;
      }
    }
    command.goal_valid = command.goals.count > 0;
    trick_engine_.cancel();
    applied_intent_sequence_ = 0xFFFFFFFFu;
    applied_gait_ = 0xFF;
  } else if (command.motion_gate) {
    const bool rc_drives = arbiter.source == safety::CommandSource::Rc;
    uint8_t effective_gait = intent.motion.gait;
    float effective_vx = intent.motion.twist_vx;
    float effective_vy = intent.motion.twist_vy;
    float effective_wz = intent.motion.twist_wz;
    gait::BodyPose body_pose = poseFromMotion(intent.motion);
    uint16_t requested_body_height = intent.motion.body_height_mm;
    uint16_t requested_stride = intent.motion.stride_len_mm;
    uint16_t requested_step_height = intent.motion.step_height_mm;
    uint8_t requested_speed = intent.motion.speed_x255;
    uint8_t requested_duty = intent.motion.duty_x255;
    float trick_height_rate_override_mm_per_s = 0.0f;

    if (rc_drives) {
      const ControllerCommand& rc = intent.rc.command;
      if (rc.trick != TrickId::None && previous_rc_trick_ == TrickId::None) {
        trick_engine_.trigger(rc.trick, rc.body_height, time.now_ms);
      }
      previous_rc_trick_ = rc.trick;
      const gait::TrickOutput& trick =
          trick_engine_.update(time.dt_ms, sticksAreActive(rc));

      float height_fraction = rc.body_height;
      if (trick.active) {
        effective_gait = static_cast<uint8_t>(trick.gait);
        effective_vx = trick.twist_vx;
        effective_vy = trick.twist_vy;
        effective_wz = trick.twist_wz;
        body_pose = trick.pose;
        if (trick.override_height) height_fraction = trick.body_height_frac;
        trick_height_rate_override_mm_per_s = trick.height_rate_mm_per_s;
        if (trick.override_gait_shape) {
          requested_stride = trick.stride_mm;
          requested_step_height = trick.step_height_mm;
          requested_speed = trick.speed_x255;
        }
      } else {
        // Simultaneous walk + body-pose (Phoenix-style): the bridge always
        // maps the left gimbal to twist and the mode-selected right-gimbal
        // overlay to either strafe (twist_vy) or a body-pose offset. Both are
        // applied together, so the operator can translate/tilt the body while
        // walking; with the sticks centred the gait holds the planted home
        // stance and only the pose moves the core.
        effective_gait = static_cast<uint8_t>(rcGaitFromIndex(rc.gait_index));
        effective_vx = rc.twist_vx;
        effective_vy = rc.twist_vy;
        effective_wz = rc.twist_wz;
        body_pose.x_mm = rc.pose_x_mm;
        body_pose.y_mm = rc.pose_y_mm;
        body_pose.z_mm = rc.pose_z_mm;
        body_pose.roll = rc.pose_roll;
        body_pose.pitch = rc.pose_pitch;
        body_pose.yaw = rc.pose_yaw;
      }
      if (!trick.active) {
        body_pose.roll += rc.trim_roll;
        body_pose.pitch += rc.trim_pitch;
      }
      const uint16_t body_height = static_cast<uint16_t>(
          gait::rcBodyHeightMm(height_fraction) + 0.5f);
        requested_body_height = body_height;
        requested_stride = static_cast<uint16_t>(
          rc.stride * config::kMaxGaitStrideMm);
        requested_step_height = static_cast<uint16_t>(
          rc.step_height * config::kMaxGaitStepMm);
        requested_speed = static_cast<uint8_t>(rc.speed * 255.0f);
        requested_duty = static_cast<uint8_t>(rc.duty * 255.0f);
      applied_intent_sequence_ = 0xFFFFFFFFu;
    } else if (intent.motion.seq != applied_intent_sequence_) {
      trick_engine_.cancel();
      previous_rc_trick_ = TrickId::None;
      applied_intent_sequence_ = intent.motion.seq;
    } else {
      trick_engine_.cancel();
      previous_rc_trick_ = TrickId::None;
    }

      BodyCommand desired_body;
      desired_body.vx = effective_vx;
      desired_body.vy = effective_vy;
      desired_body.wz = effective_wz;
      desired_body.body_height_mm = static_cast<float>(requested_body_height);
        desired_body.height_rate_override_mm_per_s =
          trick_height_rate_override_mm_per_s;
      desired_body.pose = body_pose;
      const BodyCommand& shaped_body =
        body_command_shaper_.update(desired_body, time.dt_ms);

      effective_vx = shaped_body.vx;
      effective_vy = shaped_body.vy;
      effective_wz = shaped_body.wz;
      body_pose = shaped_body.pose;
      const uint16_t shaped_height = static_cast<uint16_t>(
        shaped_body.body_height_mm + 0.5f);
      pipeline_.setParams(shaped_height, requested_stride, requested_step_height,
                requested_duty, requested_speed);
      pipeline_.setBodyPose(commandPoseToBody(body_pose));
      command.diagnostics.applied_body_height_mm = shaped_height;
      command.diagnostics.applied_stride_mm = requested_stride;
      command.diagnostics.applied_step_height_mm = requested_step_height;
      command.diagnostics.applied_duty_x255 = requested_duty;
      command.diagnostics.applied_speed_x255 = requested_speed;
    const config::GaitId desired_gait = gaitFromWire(effective_gait);
    if (effective_gait != applied_gait_) {
      pipeline_.setGait(desired_gait);
      applied_gait_ = effective_gait;
    }
    // Twist and pose are operator command-frame (forward/left); the gait
    // pipeline works in URDF body frame B (+Y front, +X right).
    float body_vx = 0.0f;
    float body_vy = 0.0f;
    commandPlanarToBody(effective_vx, effective_vy, body_vx, body_vy);
    pipeline_.setTwist(body_vx, body_vy, effective_wz);

    // Gait-tune preview: leg 1 (index kGaitTunePreviewLeg) traces the live
    // stride / step height so the operator can see a parameter change on the
    // robot itself. Only while RC drives, no trick is running, and the walk
    // command is neutral -- a real walking command always wins.
    const bool preview =
        rc_drives && intent.rc.command.gait_tune_active &&
        !trick_engine_.active() && !sticksAreActive(intent.rc.command);
    pipeline_.setPreviewLeg(preview ? kGaitTunePreviewLeg : -1);
    command.diagnostics.gait_tune_preview = preview;

    if (!previous_motion_gate_) {
      pipeline_.resetPhase();
      for (uint8_t index = 0; index < state.dxl.servo_count; ++index) {
        const dxl::ServoStatus& servo = state.dxl.servos[index];
        if (!servo.ok || servo.present_position < 0 ||
            servo.present_position > config::kServoMaxTick) {
          continue;
        }
        pipeline_.seedGoal(servo.id,
                           static_cast<uint16_t>(servo.present_position));
      }
    }

    pipeline_.update(time.dt_ms, command.goals);
    command.goal_valid = true;
    command.diagnostics.any_goal_clamped = false;
    for (uint8_t index = 0; index < command.goals.count; ++index) {
      if (command.goals.joints[index].clamped) {
        command.diagnostics.any_goal_clamped = true;
        break;
      }
    }
    command.diagnostics.any_goal_unreachable =
        command.goals.any_unreachable;
    command.diagnostics.any_goal_reach_limited =
        command.goals.any_reach_limited;
  } else {
    trick_engine_.cancel();
    body_command_shaper_.reset(
        static_cast<float>(active_config.robot.gait.body_height_mm));
    previous_rc_trick_ = TrickId::None;
    applied_intent_sequence_ = 0xFFFFFFFFu;
    applied_gait_ = 0xFF;
    pipeline_.setPreviewLeg(-1);
    // With motion gated the handset must still see what a save would store, so
    // report the RC-selected shape (or the persisted defaults when RC is not
    // the source) rather than leaving the fields at zero.
    const ControllerCommand& rc_cmd = intent.rc.command;
    const bool rc_shape =
        arbiter.source == safety::CommandSource::Rc && rc_cmd.ever_seen;
    command.diagnostics.applied_body_height_mm =
        rc_shape ? static_cast<uint16_t>(
                       gait::rcBodyHeightMm(rc_cmd.body_height) + 0.5f)
                 : active_config.robot.gait.body_height_mm;
    command.diagnostics.applied_stride_mm =
        rc_shape ? static_cast<uint16_t>(rc_cmd.stride *
                                         config::kMaxGaitStrideMm)
                 : active_config.robot.gait.stride_len_mm;
    command.diagnostics.applied_step_height_mm =
        rc_shape ? static_cast<uint16_t>(rc_cmd.step_height *
                                         config::kMaxGaitStepMm)
                 : active_config.robot.gait.step_height_mm;
    command.diagnostics.applied_duty_x255 =
        rc_shape ? static_cast<uint8_t>(rc_cmd.duty * 255.0f)
                 : active_config.robot.gait.duty_x255;
    command.diagnostics.applied_speed_x255 =
        rc_shape ? static_cast<uint8_t>(rc_cmd.speed * 255.0f)
                 : active_config.robot.gait.speed_x255;
  }

  previous_motion_gate_ = command.motion_gate;
  previous_maintenance_authority_ = maintenance_authority;
}

}  // namespace controller