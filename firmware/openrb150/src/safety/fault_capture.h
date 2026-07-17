#pragma once

#include <stdint.h>

namespace fault_capture {

enum class FatalReason : uint8_t {
  None = 0,
  HardFault = 1,
  StackOverflow = 2,
  MallocFailed = 3,
};

enum class StartupStage : uint8_t {
  Reset = 0,
  SetupEntered = 1,
  BoardInitialized = 2,
  SerialStarted = 3,
  AppStarting = 4,
  SchedulerStarting = 5,
  TasksRunning = 6,
};

struct Snapshot {
  FatalReason reason = FatalReason::None;
  StartupStage stage = StartupStage::Reset;
  char task_name[16] = {0};
  uint32_t stack_pointer = 0;
  uint32_t exception_return = 0;
  uint32_t r0 = 0;
  uint32_t r1 = 0;
  uint32_t r2 = 0;
  uint32_t r3 = 0;
  uint32_t r12 = 0;
  uint32_t lr = 0;
  uint32_t pc = 0;
  uint32_t xpsr = 0;
};

// Adopt and clear a validated record left by the previous reset.
void init();
void markStartupStage(StartupStage stage);
const Snapshot& lastSnapshot();

}  // namespace fault_capture