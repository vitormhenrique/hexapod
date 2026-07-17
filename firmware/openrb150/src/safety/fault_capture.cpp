#include "fault_capture.h"

#if defined(ARDUINO_ARCH_SAMD)
#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#endif

namespace fault_capture {
namespace {

Snapshot g_lastSnapshot;
volatile uint8_t g_currentStage =
    static_cast<uint8_t>(StartupStage::Reset);

#if defined(ARDUINO_ARCH_SAMD)
struct RetainedFaultRecord {
  uint32_t magic;
  uint32_t magic_inverse;
  uint32_t version_reason_stage;
  uint32_t task_name[4];
  uint32_t stack_pointer;
  uint32_t exception_return;
  uint32_t registers[8];
  uint32_t checksum;
  uint32_t checksum_inverse;
};

constexpr uint32_t kRetainedMagic = 0x464C5431u;  // "FLT1"
constexpr uint8_t kRecordVersion = 1;
constexpr uintptr_t kSramStart = 0x20000000u;
constexpr uintptr_t kSramEnd = 0x20008000u;

__attribute__((section(".noinit")))
volatile RetainedFaultRecord g_retainedFault;

uint32_t recordChecksum(const volatile RetainedFaultRecord& record) {
  uint32_t checksum = 0xA5C35A3Cu;
  checksum ^= record.version_reason_stage;
  for (uint8_t i = 0; i < 4; ++i) checksum ^= record.task_name[i];
  checksum ^= record.stack_pointer;
  checksum ^= record.exception_return;
  for (uint8_t i = 0; i < 8; ++i) checksum ^= record.registers[i];
  return checksum;
}

bool retainedValid() {
  return g_retainedFault.magic == kRetainedMagic &&
         g_retainedFault.magic_inverse == ~kRetainedMagic &&
         g_retainedFault.checksum_inverse == ~g_retainedFault.checksum &&
         recordChecksum(g_retainedFault) == g_retainedFault.checksum;
}

void forceDxlPowerOff() {
  // OpenRB-150 DXL power enable is PA28. Direct register writes remain valid
  // even when Arduino/RTOS state or the active stack is damaged.
  PORT->Group[0].OUTCLR.reg = (1ul << 28);
  PORT->Group[0].DIRSET.reg = (1ul << 28);
}

[[noreturn]] void resetAfterCapture() {
  forceDxlPowerOff();
  __DSB();
  SCB->AIRCR = (0x5FAul << SCB_AIRCR_VECTKEY_Pos) |
               SCB_AIRCR_SYSRESETREQ_Msk;
  __DSB();
  for (;;) __NOP();
}

void writeRecord(FatalReason reason, const char* task_name,
                 const uint32_t* frame, uint32_t stack_pointer,
                 uint32_t exception_return) {
  g_retainedFault.magic = 0;
  g_retainedFault.magic_inverse = 0;
  g_retainedFault.version_reason_stage =
      static_cast<uint32_t>(kRecordVersion) |
      (static_cast<uint32_t>(reason) << 8) |
      (static_cast<uint32_t>(g_currentStage) << 16);

  for (uint8_t i = 0; i < 4; ++i) g_retainedFault.task_name[i] = 0;
  if (task_name != nullptr) {
    volatile char* destination =
        reinterpret_cast<volatile char*>(g_retainedFault.task_name);
    for (uint8_t i = 0; i < 15 && task_name[i] != '\0'; ++i) {
      destination[i] = task_name[i];
    }
  }

  g_retainedFault.stack_pointer = stack_pointer;
  g_retainedFault.exception_return = exception_return;
  for (uint8_t i = 0; i < 8; ++i) {
    g_retainedFault.registers[i] = frame == nullptr ? 0 : frame[i];
  }
  g_retainedFault.checksum = recordChecksum(g_retainedFault);
  g_retainedFault.checksum_inverse = ~g_retainedFault.checksum;
  g_retainedFault.magic_inverse = ~kRetainedMagic;
  __DSB();
  g_retainedFault.magic = kRetainedMagic;
  __DSB();
}
#endif

}  // namespace

void init() {
  g_lastSnapshot = Snapshot{};
#if defined(ARDUINO_ARCH_SAMD)
  if (retainedValid()) {
    g_lastSnapshot.reason = static_cast<FatalReason>(
        (g_retainedFault.version_reason_stage >> 8) & 0xFFu);
    g_lastSnapshot.stage = static_cast<StartupStage>(
        (g_retainedFault.version_reason_stage >> 16) & 0xFFu);
    const volatile char* source =
        reinterpret_cast<const volatile char*>(g_retainedFault.task_name);
    for (uint8_t i = 0; i < 15; ++i) {
      g_lastSnapshot.task_name[i] = source[i];
    }
    g_lastSnapshot.stack_pointer = g_retainedFault.stack_pointer;
    g_lastSnapshot.exception_return = g_retainedFault.exception_return;
    g_lastSnapshot.r0 = g_retainedFault.registers[0];
    g_lastSnapshot.r1 = g_retainedFault.registers[1];
    g_lastSnapshot.r2 = g_retainedFault.registers[2];
    g_lastSnapshot.r3 = g_retainedFault.registers[3];
    g_lastSnapshot.r12 = g_retainedFault.registers[4];
    g_lastSnapshot.lr = g_retainedFault.registers[5];
    g_lastSnapshot.pc = g_retainedFault.registers[6];
    g_lastSnapshot.xpsr = g_retainedFault.registers[7];
  }
  g_retainedFault.magic = 0;
  g_retainedFault.magic_inverse = 0;
#endif
  g_currentStage = static_cast<uint8_t>(StartupStage::SetupEntered);
}

void markStartupStage(StartupStage stage) {
  g_currentStage = static_cast<uint8_t>(stage);
}

const Snapshot& lastSnapshot() { return g_lastSnapshot; }

}  // namespace fault_capture

namespace fault_capture {
#if defined(ARDUINO_ARCH_SAMD)
extern "C" [[noreturn]] void faultCaptureHardFaultC(uint32_t* frame,
                                                     uint32_t exc_return) {
  const uintptr_t stack_pointer = reinterpret_cast<uintptr_t>(frame);
  const bool frame_valid = (stack_pointer & 0x3u) == 0 &&
                           stack_pointer >= kSramStart &&
                           stack_pointer <= kSramEnd - 8u * sizeof(uint32_t);
  writeRecord(fault_capture::FatalReason::HardFault, nullptr,
              frame_valid ? frame : nullptr,
              static_cast<uint32_t>(stack_pointer), exc_return);
  resetAfterCapture();
}

extern "C" __attribute__((naked)) void HardFault_Handler(void) {
  __asm volatile(
      "mov r1, lr\n"
      "movs r2, #4\n"
      "tst r1, r2\n"
      "beq 1f\n"
      "mrs r0, psp\n"
      "b 2f\n"
      "1: mrs r0, msp\n"
      "2: b faultCaptureHardFaultC\n");
}

extern "C" [[noreturn]] void __wrap_vApplicationStackOverflowHook(
    TaskHandle_t, char* task_name) {
  uint32_t stack_pointer = 0;
  __asm volatile("mrs %0, psp" : "=r"(stack_pointer));
  writeRecord(fault_capture::FatalReason::StackOverflow, task_name, nullptr,
              stack_pointer, 0);
  resetAfterCapture();
}

extern "C" [[noreturn]] void __wrap_vApplicationMallocFailedHook(void) {
  uint32_t stack_pointer = 0;
  __asm volatile("mrs %0, psp" : "=r"(stack_pointer));
  writeRecord(fault_capture::FatalReason::MallocFailed, nullptr, nullptr,
              stack_pointer, 0);
  resetAfterCapture();
}
#endif
}  // namespace fault_capture