#include "hexapod_controller_ros/ros2_control_sil_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "config/config_schema.h"
#include "dxl/servo_map.h"
#include "protocol/motion_api.h"

namespace hexapod_controller_ros {
namespace {

constexpr uint32_t kConfigRevision = 1;
constexpr uint32_t kMaximumMotionValidityMs = 1000;
constexpr uint8_t kAllFeetMask =
  static_cast<uint8_t>((1u << sensors::kNumFeet) - 1u);

// Approximate steady-gait body-speed model used only for the visualization
// odometry (odom -> base_footprint). Mirrors the gait engine's cadence range
// and per-gait duty factors: during stance a foot sweeps `stride * twist`
// over `duty / f` seconds, so the body advances at stride*twist*f/duty.
constexpr double kOdomMinFreqHz = 0.25;
constexpr double kOdomMaxFreqHz = 1.20;
constexpr double kOdomYawRadiusM = 0.237;  // mean home-foot stance radius
constexpr double kOdomMaxStepS = 0.1;

double odomDutyForGait(uint8_t gait, uint8_t duty_x255) {
  double duty = 1.0;
  switch (gait) {
    case protocol::motiongait::kTripod: duty = 0.5; break;
    case protocol::motiongait::kRipple: duty = 0.667; break;
    case protocol::motiongait::kWave:
    case protocol::motiongait::kCrawl: duty = 0.833; break;
    default: return 1.0;
  }
  const double requested = static_cast<double>(duty_x255) / 255.0;
  return std::clamp(requested > duty ? requested : duty, duty, 0.95);
}
constexpr std::array<const char*, config::kNumServos> kJointNames = {
    "leg_1_coxa_joint", "leg_1_femur_joint", "leg_1_tibia_joint",
    "leg_2_coxa_joint", "leg_2_femur_joint", "leg_2_tibia_joint",
    "leg_3_coxa_joint", "leg_3_femur_joint", "leg_3_tibia_joint",
    "leg_4_coxa_joint", "leg_4_femur_joint", "leg_4_tibia_joint",
    "leg_5_coxa_joint", "leg_5_femur_joint", "leg_5_tibia_joint",
    "leg_6_coxa_joint", "leg_6_femur_joint", "leg_6_tibia_joint",
};

float clampUnit(float value) {
  return std::clamp(value, 0.0f, 1.0f);
}

float clampSignedUnit(float value) {
  return std::clamp(value, -1.0f, 1.0f);
}

uint16_t metersToMillimeters(float value, uint16_t maximum) {
  const float millimeters = std::clamp(
      value * 1000.0f, 0.0f, static_cast<float>(maximum));
  return static_cast<uint16_t>(std::lround(millimeters));
}

float metersToBoundedMillimeters(double value, float maximum) {
  return static_cast<float>(std::clamp(
      value * 1000.0, -static_cast<double>(maximum),
      static_cast<double>(maximum)));
}

float boundedRadians(double value) {
  return static_cast<float>(std::clamp(
      value, -static_cast<double>(protocol::motionlim::kMaxPoseRotRad),
      static_cast<double>(protocol::motionlim::kMaxPoseRotRad)));
}

controller::SnapshotValidity contactValidityFromWire(uint8_t validity) {
  switch (validity) {
    case hexapod_msgs::msg::FootContactArray::VALIDITY_FRESH:
      return controller::SnapshotValidity::Fresh;
    case hexapod_msgs::msg::FootContactArray::VALIDITY_STALE:
      return controller::SnapshotValidity::Stale;
    case hexapod_msgs::msg::FootContactArray::VALIDITY_FAULT:
      return controller::SnapshotValidity::Fault;
    default:
      return controller::SnapshotValidity::Unknown;
  }
}

bool contactValidityIsKnown(uint8_t validity) {
  return validity <= hexapod_msgs::msg::FootContactArray::VALIDITY_FAULT;
}

bool contactStateIsKnown(uint8_t state) {
  return state <= hexapod_msgs::msg::FootContact::STATE_FAULT;
}

void setContactState(sensors::LegContactState& destination, uint8_t state) {
  destination.state = static_cast<sensors::ContactState>(state);
  destination.near_surface =
      state == hexapod_msgs::msg::FootContact::STATE_NEAR;
  destination.touch = state == hexapod_msgs::msg::FootContact::STATE_TOUCH ||
                      state == hexapod_msgs::msg::FootContact::STATE_LOADED;
  destination.loaded = state == hexapod_msgs::msg::FootContact::STATE_LOADED;
  destination.release = state == hexapod_msgs::msg::FootContact::STATE_RELEASE;
  destination.stale = state == hexapod_msgs::msg::FootContact::STATE_STALE;
  destination.fault = state == hexapod_msgs::msg::FootContact::STATE_FAULT;
  if (destination.stale || destination.fault) destination.confidence = 0;
}

void markContactUnavailable(sensors::LegContactState& destination,
                            bool fault) {
  setContactState(destination, fault
                                   ? hexapod_msgs::msg::FootContact::STATE_FAULT
                                   : hexapod_msgs::msg::FootContact::STATE_STALE);
  destination.confidence = 0;
}

void invalidateContacts(controller::ContactSnapshot& snapshot,
                        controller::SnapshotValidity validity) {
  snapshot.validity = validity;
  const bool fault = validity == controller::SnapshotValidity::Fault;
  for (uint8_t leg = 0; leg < sensors::kNumFeet; ++leg) {
    markContactUnavailable(snapshot.feet[leg], fault);
  }
}

bool convertFootContacts(const hexapod_msgs::msg::FootContactArray& message,
                         controller::ContactSnapshot& snapshot) {
  if (!contactValidityIsKnown(message.validity) ||
      (message.present_mask & static_cast<uint8_t>(~kAllFeetMask)) != 0u) {
    return false;
  }

  snapshot = controller::ContactSnapshot{};
  snapshot.present_mask = message.present_mask;
  snapshot.validity = contactValidityFromWire(message.validity);
  std::array<bool, sensors::kNumFeet> found = {};
  for (const hexapod_msgs::msg::FootContact& source : message.feet) {
    if (source.leg_index >= sensors::kNumFeet || found[source.leg_index] ||
        !contactStateIsKnown(source.state)) {
      return false;
    }

    found[source.leg_index] = true;
    sensors::LegContactState& destination = snapshot.feet[source.leg_index];
    destination.timestamp_ms = message.source_time_ms;
    destination.proximity_raw = source.proximity_raw;
    destination.pressure_raw = source.pressure_raw;
    destination.pressure_baseline = source.pressure_baseline;
    destination.pressure_delta = source.pressure_delta;
    destination.confidence = source.confidence;
    setContactState(destination, source.state);
  }
  for (bool present : found) {
    if (!present) return false;
  }

  if (snapshot.validity != controller::SnapshotValidity::Fresh) {
    invalidateContacts(snapshot, snapshot.validity);
    return true;
  }

  for (uint8_t leg = 0; leg < sensors::kNumFeet; ++leg) {
    if ((snapshot.present_mask & (1u << leg)) == 0u) {
      markContactUnavailable(snapshot.feet[leg], false);
    }
  }
  return true;
}

}  // namespace

Ros2ControlSilAdapter::Ros2ControlSilAdapter(
    rclcpp_lifecycle::LifecycleNode& node)
    : node_(node), servo_map_(config_) {
  config::defaultRobotConfig(config_);
  config_valid_ = config::validateRobotConfig(config_);

  const std::string feedback_topic = node_.declare_parameter<std::string>(
      "feedback_topic", "/joint_states");
  const std::string motion_topic = node_.declare_parameter<std::string>(
      "motion_topic", "~/motion_command");
  const std::string safety_topic = node_.declare_parameter<std::string>(
      "sil_safety_topic", "~/sil_safety");
    const std::string foot_contact_topic = node_.declare_parameter<std::string>(
      "sil_foot_contact_topic", "~/sil_foot_contacts");
  const std::string simulation_command_topic =
      node_.declare_parameter<std::string>(
          "simulation_command_topic", "/position_controller/commands");
  const int feedback_timeout_ms = node_.declare_parameter<int>(
      "feedback_timeout_ms", static_cast<int>(feedback_timeout_ms_));
  if (feedback_timeout_ms > 0) {
    feedback_timeout_ms_ = static_cast<uint32_t>(feedback_timeout_ms);
  }
  const int contact_timeout_ms = node_.declare_parameter<int>(
      "contact_timeout_ms", static_cast<int>(contact_timeout_ms_));
  if (contact_timeout_ms > 0) {
    contact_timeout_ms_ = static_cast<uint32_t>(contact_timeout_ms);
  }

  joint_state_subscription_ =
      node_.create_subscription<sensor_msgs::msg::JointState>(
          feedback_topic, rclcpp::QoS(10),
          [this](const sensor_msgs::msg::JointState::SharedPtr message) {
            if (!acceptJointState(*message)) {
              RCLCPP_WARN(node_.get_logger(),
                          "Rejected incomplete or invalid joint-state feedback");
            }
          });
  motion_subscription_ =
      node_.create_subscription<hexapod_msgs::msg::MotionCommand>(
          motion_topic, rclcpp::QoS(10),
          [this](const hexapod_msgs::msg::MotionCommand::SharedPtr message) {
            if (!acceptMotionCommand(*message)) {
              RCLCPP_WARN(node_.get_logger(),
                          "Rejected invalid high-level motion command");
            }
          });
  safety_subscription_ =
      node_.create_subscription<hexapod_msgs::msg::SilSafetyInput>(
          safety_topic, rclcpp::QoS(10),
          [this](const hexapod_msgs::msg::SilSafetyInput::SharedPtr message) {
            acceptSafetyInput(*message);
          });
  foot_contact_subscription_ =
      node_.create_subscription<hexapod_msgs::msg::FootContactArray>(
          foot_contact_topic, rclcpp::QoS(10),
          [this](const hexapod_msgs::msg::FootContactArray::SharedPtr message) {
            if (!acceptFootContacts(*message)) {
              RCLCPP_WARN(node_.get_logger(),
                          "Rejected invalid simulated foot-contact snapshot");
            }
          });

  simulation_command_publisher_ =
      node_.create_publisher<std_msgs::msg::Float64MultiArray>(
          simulation_command_topic, rclcpp::QoS(10));
  joint_command_publisher_ =
      node_.create_publisher<hexapod_msgs::msg::JointCommand>(
          "~/joint_command", rclcpp::QoS(10));
  status_publisher_ =
      node_.create_publisher<hexapod_msgs::msg::ControllerStatus>(
          "~/controller_status", rclcpp::QoS(10));
  odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

int Ros2ControlSilAdapter::jointIndexForName(const std::string& name) {
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    if (name == kJointNames[index]) return static_cast<int>(index);
  }
  return -1;
}

const char* Ros2ControlSilAdapter::jointNameForIndex(uint8_t index) {
  return index < config::kNumServos ? kJointNames[index] : "";
}

bool Ros2ControlSilAdapter::isFinite(
    const hexapod_msgs::msg::MotionCommand& message) {
  return std::isfinite(message.body_height_m) &&
         std::isfinite(message.stride_length_m) &&
         std::isfinite(message.step_height_m) &&
         std::isfinite(message.duty_factor) &&
         std::isfinite(message.speed_scale) &&
         std::isfinite(message.normalized_vx) &&
         std::isfinite(message.normalized_vy) &&
         std::isfinite(message.normalized_wz) &&
         std::isfinite(message.body_translation_m.x) &&
         std::isfinite(message.body_translation_m.y) &&
         std::isfinite(message.body_translation_m.z) &&
         std::isfinite(message.body_rpy_rad.x) &&
         std::isfinite(message.body_rpy_rad.y) &&
         std::isfinite(message.body_rpy_rad.z);
}

bool Ros2ControlSilAdapter::convertMotionCommand(
    const hexapod_msgs::msg::MotionCommand& message,
    protocol::MotionIntent& intent, std::chrono::milliseconds& validity) {
  if (message.gait >= protocol::motiongait::kCount || !isFinite(message)) {
    return false;
  }

  const int64_t requested_ms =
      static_cast<int64_t>(message.valid_for.sec) * 1000 +
      static_cast<int64_t>(message.valid_for.nanosec / 1000000u);
  if (requested_ms <= 0) return false;

  validity = std::chrono::milliseconds(std::min<int64_t>(
      requested_ms, static_cast<int64_t>(kMaximumMotionValidityMs)));
  intent = protocol::MotionIntent{};
  intent.seq = message.sequence;
  intent.gait = message.gait;
  intent.body_height_mm = metersToMillimeters(
      message.body_height_m, protocol::motionlim::kMaxBodyHeightMm);
  intent.stride_len_mm = metersToMillimeters(
      message.stride_length_m, protocol::motionlim::kMaxStrideMm);
  intent.step_height_mm = metersToMillimeters(
      message.step_height_m, protocol::motionlim::kMaxStepMm);
  intent.duty_x255 = static_cast<uint8_t>(std::lround(
      clampUnit(message.duty_factor) * 255.0f));
  intent.speed_x255 = static_cast<uint8_t>(std::lround(
      clampUnit(message.speed_scale) * 255.0f));
  intent.twist_vx = clampSignedUnit(message.normalized_vx);
  intent.twist_vy = clampSignedUnit(message.normalized_vy);
  intent.twist_wz = clampSignedUnit(message.normalized_wz);
  intent.pose_x_mm = metersToBoundedMillimeters(
      message.body_translation_m.x, protocol::motionlim::kMaxPoseTransMm);
  intent.pose_y_mm = metersToBoundedMillimeters(
      message.body_translation_m.y, protocol::motionlim::kMaxPoseTransMm);
  intent.pose_z_mm = metersToBoundedMillimeters(
      message.body_translation_m.z, protocol::motionlim::kMaxPoseTransMm);
  intent.pose_roll = boundedRadians(message.body_rpy_rad.x);
  intent.pose_pitch = boundedRadians(message.body_rpy_rad.y);
  intent.pose_yaw = boundedRadians(message.body_rpy_rad.z);
  return true;
}

uint32_t Ros2ControlSilAdapter::elapsedMilliseconds(
    const SteadyClock::time_point& now) const {
  const uint64_t elapsed = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - epoch_)
          .count());
  return static_cast<uint32_t>(elapsed);
}

bool Ros2ControlSilAdapter::acceptJointState(
    const sensor_msgs::msg::JointState& message) {
  if (message.name.size() != message.position.size()) return false;

  std::array<double, config::kNumServos> positions = {};
  std::array<bool, config::kNumServos> found = {};
  for (size_t index = 0; index < message.name.size(); ++index) {
    const int joint_index = jointIndexForName(message.name[index]);
    if (joint_index < 0 || found[static_cast<size_t>(joint_index)] ||
        !std::isfinite(message.position[index])) {
      return false;
    }
    positions[static_cast<size_t>(joint_index)] = message.position[index];
    found[static_cast<size_t>(joint_index)] = true;
  }
  for (bool present : found) {
    if (!present) return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  feedback_positions_rad_ = positions;
  feedback_received_ = SteadyClock::now();
  feedback_valid_ = true;
  return true;
}

bool Ros2ControlSilAdapter::acceptMotionCommand(
    const hexapod_msgs::msg::MotionCommand& message) {
  protocol::MotionIntent motion;
  std::chrono::milliseconds validity{0};
  if (!convertMotionCommand(message, motion, validity)) return false;

  std::lock_guard<std::mutex> lock(mutex_);
  motion_ = motion;
  motion_expires_ = SteadyClock::now() + validity;
  motion_valid_ = true;
  return true;
}

void Ros2ControlSilAdapter::acceptSafetyInput(
    const hexapod_msgs::msg::SilSafetyInput& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  safety_.rc_seen = message.rc_seen;
  safety_.rc_kill = message.rc_kill;
  safety_.rc_armed = message.rc_armed;
  safety_.rc_autonomy_enabled = message.rc_autonomy_enabled;
  safety_.host_estop = message.host_estop;
  safety_.battery_valid = message.battery_valid;
  safety_.battery_millivolts = message.battery_millivolts;
  safety_.watchdog_fault = message.watchdog_fault;
  safety_.dxl_hard_fault = message.dxl_hard_fault;
  safety_.foot_contact_enabled = message.foot_contact_enabled;
  safety_.terrain_leveling_enabled = message.terrain_leveling_enabled;
}

bool Ros2ControlSilAdapter::acceptFootContacts(
    const hexapod_msgs::msg::FootContactArray& message) {
  controller::ContactSnapshot snapshot;
  if (!convertFootContacts(message, snapshot)) return false;

  std::lock_guard<std::mutex> lock(mutex_);
  contacts_.snapshot = snapshot;
  contacts_.received = SteadyClock::now();
  contacts_.received_valid = true;
  return true;
}

bool Ros2ControlSilAdapter::sample(controller::ControllerStepInput& input) {
  std::lock_guard<std::mutex> lock(mutex_);
  const SteadyClock::time_point now = SteadyClock::now();
  if (!feedback_valid_ ||
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now - feedback_received_)
              .count() > feedback_timeout_ms_) {
    return false;
  }

  const uint32_t now_ms = elapsedMilliseconds(now);
  if (sampled_once_ && now_ms - last_sample_ms_ > 1000u) {
    sampled_once_ = false;
    return false;
  }
  const uint32_t dt_ms = sampled_once_
                             ? std::max<uint32_t>(1u, now_ms - last_sample_ms_)
                             : 10u;
  last_sample_ms_ = now_ms;
  sampled_once_ = true;

  input = controller::ControllerStepInput{};
  input.config.robot = config_;
  input.config.revision = kConfigRevision;
  input.config.valid = config_valid_;
  input.config.persistent = false;
  input.state.config_ready = config_valid_;
  input.state.battery.millivolts = safety_.battery_millivolts;
  input.state.battery.valid = safety_.battery_valid;
  input.state.battery.validity = safety_.battery_valid
                                     ? controller::SnapshotValidity::Fresh
                                     : controller::SnapshotValidity::Unknown;
  input.state.watchdog_fault = safety_.watchdog_fault;

  if (!contacts_.received_valid) {
    invalidateContacts(input.state.contact, controller::SnapshotValidity::Unknown);
  } else {
    input.state.contact = contacts_.snapshot;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            now - contacts_.received)
            .count() > contact_timeout_ms_) {
      invalidateContacts(input.state.contact,
                         controller::SnapshotValidity::Stale);
    }
  }

  input.state.dxl.servo_count = config::kNumServos;
  input.state.dxl.validity = controller::SnapshotValidity::Fresh;
  input.state.dxl.configured_servo_coverage = true;
  input.state.dxl.pose_known_mask = (1u << config::kNumServos) - 1u;
  input.state.dxl.config_revision = kConfigRevision;
  input.state.dxl.torque_off = !safety_.rc_armed;
  input.state.dxl.hard_fault = safety_.dxl_hard_fault;
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    const config::ServoConfig& servo = config_.servos[index];
    const uint8_t joint_index = static_cast<uint8_t>(
        servo.leg * config::kJointsPerLeg + servo.joint);
    const dxl::JointCommand converted = servo_map_.angleToTick(
        servo.leg, servo.joint, feedback_positions_rad_[joint_index]);
    if (converted.unmapped) return false;

    dxl::ServoStatus& status = input.state.dxl.servos[index];
    status.id = servo.id;
    status.present_position = converted.tick;
    status.torque_enabled = safety_.rc_armed;
    status.ok = true;
  }

  input.intent.rc.ever_seen = safety_.rc_seen;
  input.intent.rc.kill = safety_.rc_kill;
  input.intent.rc.armed = safety_.rc_armed;
  input.intent.rc.failsafe = safety_.rc_seen && safety_.rc_kill;
  input.intent.rc.autonomy_enabled = safety_.rc_autonomy_enabled;
  input.intent.host_estop = safety_.host_estop;
    input.intent.features.foot_contact_enabled =
      safety_.foot_contact_enabled;
    input.intent.features.terrain_leveling_enabled =
      safety_.terrain_leveling_enabled;
  input.intent.features.jetson_control_enabled = true;
  if (motion_valid_ && now <= motion_expires_) {
    input.intent.motion = motion_;
    input.intent.jetson_heartbeat_received = true;
  }

  input.time.now_ms = now_ms;
  input.time.dt_ms = dt_ms;
  input.time.valid = true;
  return true;
}

void Ros2ControlSilAdapter::publish(
    const controller::ControllerStepInput& input,
    const controller::RobotCommand& command) {
  std::array<double, config::kNumServos> positions_rad = {};
  std::array<uint16_t, config::kNumServos> goal_ticks = {};
  std::array<bool, config::kNumServos> clamped = {};
  std::array<bool, config::kNumServos> found = {};
  bool goals_are_complete = command.goal_valid &&
                            command.goals.count == config::kNumServos;
  dxl::ServoMap map(input.config.robot);

  for (uint8_t goal_index = 0; goal_index < command.goals.count;
       ++goal_index) {
    const gait::PipelineJoint& goal = command.goals.joints[goal_index];
    if (goal.leg >= config::kNumLegs ||
        goal.joint >= config::kJointsPerLeg) {
      goals_are_complete = false;
      continue;
    }
    const uint8_t joint_index = static_cast<uint8_t>(
        goal.leg * config::kJointsPerLeg + goal.joint);
    if (found[joint_index]) {
      goals_are_complete = false;
      continue;
    }
    found[joint_index] = true;
    goal_ticks[joint_index] = goal.tick;
    positions_rad[joint_index] = map.tickToAngle(
        goal.leg, goal.joint, goal.tick);
    clamped[joint_index] = goal.clamped;
  }
  for (bool present : found) {
    if (!present) goals_are_complete = false;
  }

  hexapod_msgs::msg::JointCommand joint_command;
  joint_command.header.stamp = node_.get_clock()->now();
  joint_command.source_time_ms = input.time.now_ms;
  joint_command.config_revision = input.config.revision;
  joint_command.safety_state = static_cast<uint8_t>(command.safety_state);
  joint_command.fault_reason = static_cast<uint8_t>(command.fault_reason);
  joint_command.command_source = static_cast<uint8_t>(command.command_source);
  joint_command.motion_gate = command.motion_gate;
  joint_command.goal_valid = command.goal_valid;
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    if (!found[index]) continue;
    joint_command.name.emplace_back(jointNameForIndex(index));
    joint_command.goal_tick.emplace_back(goal_ticks[index]);
    joint_command.position_rad.emplace_back(positions_rad[index]);
    joint_command.clamped.emplace_back(clamped[index]);
  }
  joint_command_publisher_->publish(joint_command);

  const bool send_simulation_command =
      command.motion_gate && command.goal_valid && goals_are_complete;
  if (send_simulation_command) {
    std_msgs::msg::Float64MultiArray simulation_command;
    simulation_command.data.assign(positions_rad.begin(), positions_rad.end());
    simulation_command_publisher_->publish(simulation_command);
    std::lock_guard<std::mutex> lock(mutex_);
    last_simulation_command_ = positions_rad;
    ++simulation_output_count_;
  }

  hexapod_msgs::msg::ControllerStatus status;
  status.header.stamp = joint_command.header.stamp;
  status.source_time_ms = input.time.now_ms;
  status.config_revision = input.config.revision;
  status.safety_state = static_cast<uint8_t>(command.safety_state);
  status.fault_reason = static_cast<uint8_t>(command.fault_reason);
  status.command_source = static_cast<uint8_t>(command.command_source);
  status.motion_authorized = command.motion_authorized;
  status.motion_gate = command.motion_gate;
  status.allow_dxl_power = command.allow_dxl_power;
  status.allow_torque = command.allow_torque;
  status.goal_valid = command.goal_valid;
  status.simulation_output_published = send_simulation_command;
  status_publisher_->publish(status);

  // Integrate the commanded twist into a planar visualization odometry and
  // broadcast odom -> base_footprint so walking translates the RViz model.
  const SteadyClock::time_point now = SteadyClock::now();
  double dt_s = 0.0;
  if (odom_time_valid_) {
    dt_s = std::chrono::duration<double>(now - odom_last_update_).count();
    dt_s = std::clamp(dt_s, 0.0, kOdomMaxStepS);
  }
  odom_last_update_ = now;
  odom_time_valid_ = true;

  const protocol::MotionIntent& motion = input.intent.motion;
  const bool walking_gait =
      motion.gait >= protocol::motiongait::kTripod &&
      motion.gait <= protocol::motiongait::kCrawl;
  if (send_simulation_command && walking_gait && dt_s > 0.0) {
    const double speed = static_cast<double>(motion.speed_x255) / 255.0;
    const double freq =
        kOdomMinFreqHz + (kOdomMaxFreqHz - kOdomMinFreqHz) * speed;
    const double duty = odomDutyForGait(motion.gait, motion.duty_x255);
    const double stride_m =
        static_cast<double>(motion.stride_len_mm) / 1000.0;
    const double gain = stride_m * freq / duty;
    // Twist is operator command-frame: forward/left/yaw-CCW. Body frame B
    // has +Y forward and +X right (command_frame.h).
    const double v_forward = gain * static_cast<double>(motion.twist_vx);
    const double v_left = gain * static_cast<double>(motion.twist_vy);
    const double body_vx = -v_left;
    const double body_vy = v_forward;
    const double omega =
        gain * static_cast<double>(motion.twist_wz) / kOdomYawRadiusM;
    const double c = std::cos(odom_yaw_rad_);
    const double s = std::sin(odom_yaw_rad_);
    odom_x_m_ += (body_vx * c - body_vy * s) * dt_s;
    odom_y_m_ += (body_vx * s + body_vy * c) * dt_s;
    odom_yaw_rad_ += omega * dt_s;
  }

  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = joint_command.header.stamp;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom_x_m_;
  odom_tf.transform.translation.y = odom_y_m_;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation.z = std::sin(odom_yaw_rad_ / 2.0);
  odom_tf.transform.rotation.w = std::cos(odom_yaw_rad_ / 2.0);
  odom_broadcaster_->sendTransform(odom_tf);
}

uint32_t Ros2ControlSilAdapter::simulationOutputCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return simulation_output_count_;
}

std::array<double, config::kNumServos>
Ros2ControlSilAdapter::lastSimulationCommand() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_simulation_command_;
}

}  // namespace hexapod_controller_ros