// Host probe: MacMaintenance + GaitPipeline (host-motion) path at crouch.
// Mirrors tools/analysis/serial_crouch_test.py, which froze at h=65 on HW.
#include <math.h>
#include <stdio.h>

#include "controller_sim_adapter.h"

using namespace controller;
using namespace config;

static void walkHost(uint16_t body_height_mm) {
  sim::ControllerSimAdapter adapter;
  adapter.configureDefault();
  adapter.setReadyDxl();
  adapter.state().dxl.torque_off = false;
  adapter.setTime(1000, 10);

  ControllerIntent& intent = adapter.intent();
  intent.rc.ever_seen = false;
  intent.rc.armed = false;
  intent.rc.kill = false;
  intent.rc.failsafe = true;

  // Host maintenance lock + gait-pipeline control mode.
  intent.maintenance.lock_held = true;
  intent.maintenance.lock_token = 7;
  intent.maintenance.control_mode = protocol::MaintControlMode::GaitPipeline;

  protocol::MotionIntent& motion = intent.motion;
  motion.gait = 2;  // tripod (wire id used by SET_GAIT)
  motion.body_height_mm = body_height_mm;
  motion.stride_len_mm = 60;
  motion.step_height_mm = 30;
  motion.duty_x255 = 159;
  motion.speed_x255 = 128;
  motion.twist_vx = 0.0f;
  motion.twist_vy = 0.6f;
  motion.twist_wz = 0.0f;
  motion.seq = 1;

  // Settle at height with zero twist first.
  motion.twist_vy = 0.0f;
  for (int i = 0; i < 800; ++i) adapter.advance(10);
  motion.twist_vy = 0.6f;
  motion.seq = 2;

  uint16_t femur_min = 65535, femur_max = 0;
  uint16_t coxa_min = 65535, coxa_max = 0;
  bool gate = false;
  for (int i = 0; i < 800; ++i) {
    const RobotCommand& cmd = adapter.advance(10);
    gate = cmd.motion_gate;
    if (!cmd.goal_valid) continue;
    for (uint8_t k = 0; k < cmd.goals.count; ++k) {
      const auto& j = cmd.goals.joints[k];
      if (j.leg == 0 && j.joint == 1) {
        if (j.tick < femur_min) femur_min = j.tick;
        if (j.tick > femur_max) femur_max = j.tick;
      }
      if (j.leg == 0 && j.joint == 0) {
        if (j.tick < coxa_min) coxa_min = j.tick;
        if (j.tick > coxa_max) coxa_max = j.tick;
      }
    }
  }
  printf("host-motion h=%3u gate=%d  leg1 coxa[%u..%u] amp=%d  femur[%u..%u] amp=%d\n",
         body_height_mm, gate, coxa_min, coxa_max, coxa_max - coxa_min,
         femur_min, femur_max, femur_max - femur_min);
}

int main() {
  walkHost(132);
  walkHost(100);
  walkHost(80);
  walkHost(70);
  walkHost(65);
  return 0;
}
