// Dump 18 servo ticks at 8 evenly spaced phases of one steady tripod cycle
// (h=132, stride 60, step 30, duty 159, speed 128, forward 0.6). Output JSON
// on stdout for the hardware apex test.
#include <stdio.h>

#include "controller_sim_adapter.h"

using namespace controller;
using namespace config;

int main() {
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
  intent.maintenance.lock_held = true;
  intent.maintenance.lock_token = 7;
  intent.maintenance.control_mode = protocol::MaintControlMode::GaitPipeline;

  protocol::MotionIntent& motion = intent.motion;
  motion.gait = 2;
  motion.body_height_mm = 132;
  motion.stride_len_mm = 60;
  motion.step_height_mm = 30;
  motion.duty_x255 = 159;
  motion.speed_x255 = 128;
  motion.seq = 1;
  for (int i = 0; i < 800; ++i) adapter.advance(10);
  motion.twist_vx = 0.6f;  // forward
  motion.seq = 2;
  for (int i = 0; i < 800; ++i) adapter.advance(10);  // steady state

  // Tripod cycle: 8 steps x 80 ms = 640 ms = 64 sim steps of 10 ms.
  // Sample every 8 sim steps -> 8 phases.
  printf("[\n");
  for (int phase = 0; phase < 8; ++phase) {
    const RobotCommand& cmd = adapter.command();
    printf("  {\"phase\": %d, \"ticks\": {", phase);
    bool first = true;
    for (uint8_t k = 0; k < cmd.goals.count; ++k) {
      const auto& j = cmd.goals.joints[k];
      printf("%s\"%u\": [%u, %u, %u]", first ? "" : ", ", j.id, j.tick,
             j.leg, j.joint);
      first = false;
    }
    printf("}}%s\n", phase == 7 ? "" : ",");
    for (int i = 0; i < 8; ++i) adapter.advance(10);
  }
  printf("]\n");
  return 0;
}
