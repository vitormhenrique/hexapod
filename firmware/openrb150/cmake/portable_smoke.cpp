#include "config/config_schema.h"
#include "controller/controller_contract.h"
#include "gait/gait_pipeline.h"

int main() {
  config::RobotConfig config;
  config::defaultRobotConfig(config);

  gait::GaitPipeline pipeline(config);
  pipeline.setGait(config::GaitId::Stand);
  gait::PipelineOutput goals;
  pipeline.update(10, goals);
  if (goals.count != config::kNumServos) return 1;

  controller::ControllerStepInput input;
  input.time.now_ms = 10;
  input.time.dt_ms = 10;
  input.time.valid = true;
  input.config.robot = config;
  input.config.revision = 1;
  input.config.valid = true;

  return input.time.valid && input.config.valid ? 0 : 1;
}