#pragma once

// ===========================================================================
// RTOS task system.
//
// Creates the six Phase 1 tasks (control, dxl, rc, api, i2c, health) with
// bounded stacks and starts the FreeRTOS scheduler. For the skeleton each task
// only runs a periodic loop, checks in with the software watchdog, and bumps a
// loop counter; the health task reports stack high-water marks, loop counts,
// and the watchdog missed-task mask over USB CDC.
//
// The task bodies live together here while they are stubs. As each subsystem is
// implemented (USB API rbg.5, DXL bus rbg.6, I2C rbg.7, CRSF rbg.9, control/
// gait in Phase 2) its body moves into a dedicated app/*_task module.
// ===========================================================================

namespace app {

// Create all tasks and start the scheduler. Call once at the end of setup().
// Does NOT return on success (the scheduler takes over the CPU).
void start();

}  // namespace app
