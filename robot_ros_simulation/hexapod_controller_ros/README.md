# Hexapod Controller ROS Wrapper

`hexapod_controller_ros` is a ROS 2 Jazzy lifecycle shell around the portable
firmware `ControllerCore`. It links the Arduino-free CMake target from
`firmware/openrb150`; it does not link Arduino, FreeRTOS, DYNAMIXEL, `Wire`, or
the MCU task source.

## Lifecycle

The node starts unconfigured. Configure it, then activate it with a ROS 2
lifecycle client before it can step the core. Its `period_ms` parameter defaults
to 10 ms. Deactivation cancels the timer; cleanup resets core state and destroys
the timer.

The timer callback is the only code path that invokes `ControllerCore::step()`.
`ControllerInputAdapter::sample()` must return a complete copied
`ControllerStepInput`; `ControllerCommandAdapter::publish()` receives the
result from that exact input. Future ROS subscriptions may stage their data in a
separate adapter-owned mailbox, but must never mutate ControllerCore state from
a callback.

The adapter supplies `ControllerTime`. The ROS wall timer schedules sampling but
does not manufacture, clamp, or replace controller time. This keeps native,
SIL, and future MCU parity traces comparable.

## Current Scope

The executable installs a `Ros2ControlSilAdapter` before lifecycle
configuration. Its timer is still the only caller of `ControllerCore::step()`.
ROS callbacks only stage copied mailbox values, protected by the adapter's
mutex; they never mutate core state.

The adapter accepts the following typed, high-level inputs:

- `~/motion_command` (`hexapod_msgs/msg/MotionCommand`) supplies a bounded
  gait/body request and acts as the SIL Jetson heartbeat while its bounded
  `valid_for` interval remains fresh.
- `~/sil_safety` (`hexapod_msgs/msg/SilSafetyInput`) supplies SIL-only RC
  authority, health snapshots, and the requested simulated contact mode. It is
  not a hardware or serial API.
- `~/sil_foot_contacts` (`hexapod_msgs/msg/FootContactArray`) supplies one
  fused six-foot snapshot for SIL. Each observation is keyed by its leg index,
  retains the source controller time, and may represent fresh, stale, faulted,
  low-confidence, or missing contact data. Invalid, absent, or receipt-expired
  snapshots are converted to zero-confidence unavailable feet before the core
  runs.
- `joint_states` supplies exactly the 18 canonical named state observations.
  Missing, duplicate, unknown, non-finite, or stale feedback is rejected.

`ServoMap` is used in both directions: named radian feedback becomes a
calibrated DYNAMIXEL tick snapshot for the core, and only complete,
motion-gated core output becomes a radian `Float64MultiArray` on the mock
`position_controller/commands` topic. The adapter also publishes traceable
`~/joint_command` and `~/controller_status` messages. It never subscribes to
raw joint or DXL target commands, and it does not bridge to physical hardware.

`FootContactArray.header.stamp` is ROS ordering time and
`source_time_ms` is the preserved controller-time snapshot timestamp. Contact
data contains no coordinates, so the fixed leg index, rather than a frame ID,
identifies each sensor attachment. Mailbox expiry is measured from receipt time
because a simulated publisher may have a different clock epoch. No `/imu`
source is added: `ControllerCore` currently has no IMU contract field or
consumer, and publishing synthetic zero measurements would misrepresent the
simulation.

## SIL Launch

Build the workspace, then launch the isolated mock path:

```sh
just build
just sil-launch
```

`sil.launch.py` starts `robot_state_publisher`, mock `ros2_control`, the joint
state broadcaster, the position controller, and the active ControllerCore
node. It deliberately leaves the GUI, RViz, and velocity controller out of
this path. The launch assumes the unprefixed canonical 18-joint map; it is not
valid for a renamed/prefixed URDF.

For an explicit demonstration, first publish a safe unarmed SIL snapshot and
observe `/hexapod_controller_core/controller_status`. After it reports
`Disarmed`, publish an armed, autonomy-enabled SIL snapshot plus a fresh
`MotionCommand`. Removing `rc_armed` or asserting `host_estop` stops raw
position-controller output on the next controller step.

## Companion Simulation

`companion_sim.launch.py` adds a local simulated-firmware endpoint to the SIL
graph. It speaks the same framed protocol as the companion application, then
maps only high-level gait, gait-parameter, body-twist, and body-pose requests
to `/hexapod_controller_core/motion_command`. The bridge continuously republishes
the latest request with a bounded validity interval and mirrors the simulated
arming state on `/hexapod_controller_core/sil_safety`. The simulated firmware
boots disarmed; starting the Gait Lab session (or `SET_ARMING(arm)`) arms it,
which gives ControllerCore the arm edge it requires before allowing motion.
The SIL controller runs with the idle auto-disarm timeout disabled because no
physical servos hold torque in simulation.

Run the complete workflow from this directory:

```sh
just companion-sim
```

Or run it from the monorepo root:

```sh
just sim-companion
```

The app opens with the loopback endpoint selected and connects to:

```text
tcp://127.0.0.1:5560?token=hexapod-sim
```

The combined launch also starts RViz2 by default so the simulated robot is
visible while commands are issued from the companion. The joint slider GUI
remains disabled to avoid competing with companion motion commands. For a
headless simulation, pass `false` as the fourth positional argument:
`just sim-companion 127.0.0.1 5560 hexapod-sim false` from the monorepo root,
or `just companion-sim 127.0.0.1 5560 hexapod-sim false` from this workspace.

Use a different loopback port or token when another local simulation is already
active. The optional arguments are positional: `host`, `port`, then `token`.

```sh
just companion-sim 127.0.0.1 5561 my-local-token
```

The endpoint is intentionally loopback-only. It provides `HELLO`, status,
capabilities, high-level motion, and health/control/RC telemetry. DYNAMIXEL
maintenance, servo tuning, passive pose, I2C/contact sensing, terrain
leveling, and Jetson authority are unavailable and appear disabled in the
companion. It is a SIL control surface, not a replacement for firmware or a
hardware safety test.

Build and test from `robot_ros_simulation`:

```sh
just build
pixi run -e jazzy --manifest-path pixi.toml \
  colcon test --packages-select hexapod_controller_ros
pixi run -e jazzy --manifest-path pixi.toml \
  colcon test-result --all --verbose
```