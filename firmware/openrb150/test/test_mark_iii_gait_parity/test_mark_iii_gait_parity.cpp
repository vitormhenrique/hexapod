#include <math.h>
#include <unity.h>

#include "config/config_schema.h"
#include "gait/gait_pipeline.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kTicksPerDeg = 4096.0f / 360.0f;
constexpr float kStrideMm = 2.0f;
constexpr float kLiftMm = 5.0f;
constexpr float kBodyHeightMm = 40.0f;
constexpr float kYawTravelRad = 32.0f * kDegToRad;
constexpr float kCoxaMm = 56.08f;
constexpr float kFemurMm = 66.51f;
constexpr float kTibiaMm = 24.86f;
constexpr float kHomeRadiusMm = 127.0f;
constexpr float kHomeLocalZMm = -44.55f;
constexpr float kCoxaFrameZMm = 4.5f;

constexpr float kHome[config::kNumLegs][2] = {
  {-155.4f, -205.4f}, {155.4f, -205.4f}, {196.8f, 0.0f},
  {155.4f, 205.4f},   {-155.4f, 205.4f}, {-196.8f, 0.0f},
};

constexpr float kMount[config::kNumLegs][3] = {
  {-65.6f, -115.6f, 135.0f}, {65.6f, -115.6f, -135.0f},
  {69.8f, 0.0f, -90.0f},     {65.6f, 115.6f, -45.0f},
  {-65.6f, 115.6f, 45.0f},   {-69.8f, 0.0f, 90.0f},
};

constexpr uint8_t kServoIds[config::kNumLegs][config::kJointsPerLeg] = {
  {1, 2, 3},   {4, 5, 6},   {7, 8, 9},
  {10, 11, 12}, {13, 14, 15}, {16, 17, 18},
};

struct ReferenceGait {
  config::GaitId id;
  uint8_t steps;
  uint8_t travel_divisor;
  uint8_t origins[config::kNumLegs];
};

constexpr ReferenceGait kGaits[] = {
    {config::GaitId::Tripod, 8, 4, {5, 1, 5, 1, 5, 1}},
    {config::GaitId::Ripple, 12, 8, {1, 7, 11, 3, 9, 5}},
    {config::GaitId::Wave, 24, 20, {1, 13, 17, 21, 9, 5}},
};

struct ReferenceCommand {
  float body_x;
  float body_y;
  float yaw;
};

constexpr ReferenceCommand kCommands[] = {
    {0.0f, 1.0f, 0.0f},  // forward
    {1.0f, 0.0f, 0.0f},  // strafe right in body frame
  {0.0f, 0.0f, 0.04f},  // small counter-clockwise yaw above motion deadband
};

float clampUnit(float value) {
  if (value < -1.0f) return -1.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

void referenceKeyframe(const ReferenceGait& gait, uint8_t leg,
                       uint8_t gait_step, float& longitudinal, float& lift) {
  const uint8_t relative = static_cast<uint8_t>(
      (gait_step + gait.steps - gait.origins[leg]) % gait.steps);
  lift = 0.0f;
  if (relative == 0) {
    longitudinal = 0.0f;
    lift = 1.0f;
  } else if (relative == 1) {
    longitudinal = 0.5f;
    lift = 0.5f;
  } else if (relative == 2) {
    longitudinal = 0.5f;
  } else if (relative == gait.steps - 1) {
    longitudinal = -0.5f;
    lift = 0.5f;
  } else {
    longitudinal =
        0.5f - static_cast<float>(relative - 2) /
                   static_cast<float>(gait.travel_divisor);
  }
}

void referenceFoot(const ReferenceGait& gait, const ReferenceCommand& command,
                   uint8_t leg, uint32_t elapsed_ms, float& x, float& y,
                   float& z) {
  const float step_phase =
      fmodf(static_cast<float>(elapsed_ms) / 50.0f,
            static_cast<float>(gait.steps));
  const uint8_t current_step =
      static_cast<uint8_t>(floorf(step_phase)) + 1;
  const uint8_t next_step = current_step == gait.steps
                                ? 1
                                : static_cast<uint8_t>(current_step + 1);
  const float interpolation = step_phase - floorf(step_phase);
  float current_longitudinal, current_lift;
  float next_longitudinal, next_lift;
  referenceKeyframe(gait, leg, current_step, current_longitudinal,
                    current_lift);
  referenceKeyframe(gait, leg, next_step, next_longitudinal, next_lift);
  const float longitudinal =
      current_longitudinal +
      (next_longitudinal - current_longitudinal) * interpolation;
  const float lift =
      current_lift + (next_lift - current_lift) * interpolation;
  const float yaw = command.yaw * kYawTravelRad * longitudinal;
  const float cosine = cosf(yaw);
  const float sine = sinf(yaw);
  x = kHome[leg][0] * cosine - kHome[leg][1] * sine +
      command.body_x * kStrideMm * longitudinal;
  y = kHome[leg][0] * sine + kHome[leg][1] * cosine +
      command.body_y * kStrideMm * longitudinal;
  const float command_scale = fmaxf(
      fabsf(command.body_x),
      fmaxf(fabsf(command.body_y), fabsf(command.yaw)));
  const float lift_scale = fminf(1.0f, command_scale * 4.0f);
  z = -kBodyHeightMm + kLiftMm * lift_scale * lift;
}

void rawPlanar(float radius, float z, float& femur, float& tibia) {
  const float planar = radius - kCoxaMm;
  const float distance = sqrtf(planar * planar + z * z);
  const float knee_cosine = clampUnit(
      (distance * distance - kFemurMm * kFemurMm - kTibiaMm * kTibiaMm) /
      (2.0f * kFemurMm * kTibiaMm));
  tibia = acosf(knee_cosine);
  const float line = atan2f(z, planar);
  const float shoulder = atan2f(
      kTibiaMm * sinf(tibia), kFemurMm + kTibiaMm * cosf(tibia));
  femur = line - shoulder;
}

uint16_t referenceServoTick(uint8_t leg, uint8_t joint, float body_x,
                            float body_y, float body_z, bool& clamped) {
  const float transform = -(kMount[leg][2] + 90.0f) * kDegToRad;
  const float delta_x = body_x - kMount[leg][0];
  const float delta_y = body_y - kMount[leg][1];
  const float local_x = cosf(transform) * delta_x -
                        sinf(transform) * delta_y;
  const float local_y = sinf(transform) * delta_x +
                        cosf(transform) * delta_y;
  const float radius = sqrtf(local_x * local_x + local_y * local_y);
  const float local_z = body_z - kCoxaFrameZMm;
  float home_femur, home_tibia;
  rawPlanar(kHomeRadiusMm, kHomeLocalZMm, home_femur, home_tibia);
  float target_femur, target_tibia;
  rawPlanar(radius, local_z, target_femur, target_tibia);

  float servo_degrees = 0.0f;
  if (joint == static_cast<uint8_t>(config::JointRole::Coxa)) {
    servo_degrees = atan2f(local_y, local_x) * kRadToDeg;
  } else if (joint == static_cast<uint8_t>(config::JointRole::Femur)) {
    servo_degrees = (target_femur - home_femur) * kRadToDeg;
  } else {
    servo_degrees = (target_tibia - home_tibia) * kRadToDeg;
  }

  const float minimum = -90.0f;
  const float maximum = 90.0f;
  clamped = servo_degrees < minimum || servo_degrees > maximum;
  if (servo_degrees < minimum) servo_degrees = minimum;
  if (servo_degrees > maximum) servo_degrees = maximum;
  long tick = lroundf(config::kServoCenterTick + servo_degrees * kTicksPerDeg);
  if (tick < 0) tick = 0;
  if (tick > config::kServoMaxTick) tick = config::kServoMaxTick;
  return static_cast<uint16_t>(tick);
}

const gait::PipelineJoint* jointById(const gait::PipelineOutput& output,
                                     uint8_t id) {
  for (uint8_t index = 0; index < output.count; ++index) {
    if (output.joints[index].id == id) return &output.joints[index];
  }
  return nullptr;
}

void assertTimeSeriesParity(const ReferenceGait& gait,
                            const ReferenceCommand& command) {
  config::RobotConfig config;
  config::defaultRobotConfig(config);
  config.gait.body_height_mm = 40;
  config.gait.stride_len_mm = 2;
  config.gait.step_height_mm = 5;
  config.gait.duty_x255 = 159;
  config.gait.speed_x255 = 128;
  gait::GaitPipeline pipeline(config);
  pipeline.setGait(gait.id);
  pipeline.setTwist(command.body_x, command.body_y, command.yaw);

  gait::PipelineOutput output;
  pipeline.update(0, output);
  const uint32_t duration_ms =
      static_cast<uint32_t>(gait.steps) * 50u * 2u;
  // Phoenix defines joint endpoints once per NomGaitSpeed interval. Compare
  // those authoritative 50 ms keyframes over two complete cycles; production
  // uses smooth Cartesian interpolation between the same endpoints.
  for (uint32_t elapsed_ms = 0; elapsed_ms < duration_ms;
       elapsed_ms += 50u) {
    if (elapsed_ms != 0) pipeline.update(50, output);
    TEST_ASSERT_EQUAL_UINT8(config::kNumServos, output.count);
    TEST_ASSERT_FALSE(output.any_unreachable);
    TEST_ASSERT_FALSE(output.any_reach_limited);
    for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
      float foot_x, foot_y, foot_z;
      referenceFoot(gait, command, leg, elapsed_ms, foot_x, foot_y, foot_z);
      for (uint8_t joint = 0; joint < config::kJointsPerLeg; ++joint) {
        const gait::PipelineJoint* actual =
            jointById(output, kServoIds[leg][joint]);
        TEST_ASSERT_NOT_NULL(actual);
        bool expected_clamped = false;
        const uint16_t expected = referenceServoTick(
          leg, joint, foot_x, foot_y, foot_z, expected_clamped);
        TEST_ASSERT_INT_WITHIN(3, expected, actual->tick);
        TEST_ASSERT_EQUAL(expected_clamped, actual->clamped);
      }
    }
  }
}

void test_all_leg_angles_follow_mark_iii_keyframes_through_time() {
  for (const ReferenceGait& gait : kGaits) {
    for (const ReferenceCommand& command : kCommands) {
      assertTimeSeriesParity(gait, command);
    }
  }
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_all_leg_angles_follow_mark_iii_keyframes_through_time);
  return UNITY_END();
}