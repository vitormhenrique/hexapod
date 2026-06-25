# Jetson Autonomy Plan for Hexapod with Gemini 336-Class Depth Camera

## Design principle

The Jetson is an optional autonomy advisor. It perceives the world, estimates pose, builds a local map, classifies terrain, and proposes safe body-level commands. The OpenRB-150 firmware remains the final authority for arming, failsafe, RC override, gait timing, IK, joint limits, DYNAMIXEL safety, and servo commands.

The Jetson must never stream raw servo ticks in normal walking mode.

## Recommended software stack

- ROS 2 on Jetson for camera, TF, state estimation, mapping, visualization, and integration.
- Orbbec ROS 2 wrapper for Gemini 336 camera topics.
- Isaac ROS Visual SLAM or an equivalent VIO/VO source for odometry.
- Isaac ROS Nvblox for real-time local 3D reconstruction and costmap support.
- Nav2 where useful for costmap-based route-around behavior.
- A custom Python-first `hexapod_autonomy` package for legged terrain reasoning, RC/autonomy blending, heading lock, and climb/around decisions.
- A shared serial protocol client so the Mac companion and Jetson speak the same MCU API.

## Runtime architecture

```text
Gemini 336 camera
  -> depth/RGB/point cloud/IMU
  -> visual odometry / state fusion
  -> local 3D map + costmap
  -> robot-centric traversability grid
  -> obstacle classifier
  -> local route planner
  -> RC/autonomy mixer
  -> Jetson serial bridge
  -> OpenRB-150 firmware
  -> gait/IK/DYNAMIXEL safety
```

## Features to develop

### 1. Camera bring-up and recording

Goal: prove the Gemini 336 pipeline before doing autonomy.

Tasks:

- Launch camera in ROS 2.
- Verify depth, RGB, camera info, point cloud, and IMU topics.
- Record bags under varied lighting: indoor, sunlight, reflective tile, shadows, near obstacles.
- Add a repeatable `just jetson-camera` and `just jetson-record-bag` workflow.

Acceptance:

- A recorded bag can be replayed on the Jetson and on a dev machine.
- Camera timestamps are stable enough for mapping.
- Frame rate and dropped-frame statistics are visible.

### 2. TF, URDF, and camera calibration

Goal: make all geometry explicit.

Tasks:

- Publish `map -> odom -> base_link -> camera_link`.
- Add camera pose to `robot_description` and Jetson config.
- Visualize the camera frustum, body, legs, and local map together.

Acceptance:

- RViz shows the robot model, camera frame, point cloud, and map aligned.
- Camera pose can be edited and committed through config.

### 3. Jetson-to-MCU bridge

Goal: let ROS 2 interact with the MCU safely.

Tasks:

- Publish MCU telemetry as ROS 2 topics.
- Accept safe high-level commands only: body twist, desired heading, gait hint, clearance hint, speed scale, stop, heartbeat.
- Add TTL to every Jetson motion command.

Acceptance:

- Jetson can command low-speed flat-floor motion.
- Stale Jetson commands expire.
- RC kill and disarm always override Jetson.

### 4. State estimation

Goal: know how the robot is moving in the world.

Tasks:

- Run visual odometry or VSLAM from camera data.
- Fuse camera odometry, camera IMU, and MCU body estimates when available.
- Publish `odom -> base_link` consistently.

Acceptance:

- Short flat-floor walking produces plausible odometry.
- Pose drift is logged and visible.
- Mapping continues without large frame jumps during slow walking.

### 5. Local 3D map

Goal: represent obstacles around the robot.

Tasks:

- Build a local 3D map from depth plus pose.
- Export an obstacle/cost layer usable by the local route planner.
- Mark unknown areas as risky, not free.

Acceptance:

- Boxes, chair legs, walls, ramps, and drop-offs appear in the map.
- Low-confidence depth areas are visible.

### 6. Traversability grid

Goal: build legged-specific terrain layers, not just wheeled-robot occupancy.

Maintain a robot-centric 2.5D grid with layers:

- height
- height variance
- slope
- surface normal
- roughness
- obstacle height
- drop depth
- confidence
- foothold quality
- time since observed
- contact-confirmed cells

Acceptance:

- The grid updates from replayed bags.
- Each cell has an explainable reason for being safe, risky, or blocked.

### 7. Climbable-vs-bypass classifier

Goal: decide whether to go over or go around.

Classify objects as:

- `CLIMBABLE`
- `BYPASS`
- `UNKNOWN_STOP`
- `LOW_CONFIDENCE_SLOW`

Initial deterministic checks:

- obstacle height <= configured safe step height
- slope <= configured safe slope
- top surface wide enough for footholds
- roughness within configured limit
- no detected drop-off immediately behind obstacle
- enough confident cells for at least a stable support pattern
- object is not moving
- surface is not low-confidence reflective/transparent depth

Acceptance:

- Classifier works from recorded scenes.
- Every decision includes logged reasons.

### 8. Heading-preserving local route planner

Goal: go around obstacles without permanently changing the robot's heading.

Candidate actions:

- `STOP`
- `STRAIGHT_OVER_SLOW`
- `STRAFE_LEFT_AROUND`
- `STRAFE_RIGHT_AROUND`
- `ARC_LEFT_AROUND`
- `ARC_RIGHT_AROUND`
- `BACK_UP_SMALL`

Cost terms:

- collision cost
- step cost
- slope cost
- roughness cost
- foothold cost
- unknown cost
- heading deviation cost
- RC intent deviation cost

Acceptance:

- Robot can detour left/right while body yaw remains near the original mission heading.
- Planner chooses stop when climb and bypass are both unsafe.

### 9. RC/autonomy mixer

Goal: autonomy assists the pilot rather than replacing them abruptly.

Rules:

- RC kill/disarm always wins.
- RC autonomy switch is required for Jetson control.
- RC speed knob scales Jetson velocity.
- RC yaw stick overrides heading lock.
- RC translation stick can nudge or bias the selected route.
- Jetson confidence reduces speed or triggers stop.

Acceptance:

- Pilot can take over instantly.
- Manual nudges bias autonomy without disabling it.
- Low confidence slows/stops the robot.

### 10. Gait adaptation hints

Goal: adapt walking style to terrain without moving gait safety out of firmware.

Jetson can request:

- lower speed scale
- shorter stride
- increased foot clearance
- crawl/wave gait for careful stepping
- body height adjustment within firmware limits

Acceptance:

- MCU clamps all hints.
- Known low-step tests use slower, higher-clearance movement.

### 11. Contact-feedback terrain probing

Goal: use servo and foot/finger data as a validation layer.

Inputs:

- servo present position vs goal
- velocity/load/current or available load proxy
- DYNAMIXEL error bits
- foot/finger contact, pressure, proximity, or touch state

Use it to detect:

- early collision
- missed foothold
- soft terrain
- slip
- unexpected obstruction

Acceptance:

- Contact outcomes annotate the traversability grid and logs.
- Unexpected contact triggers slow/stop/recover behavior.

## Command contract additions

```text
JETSON_HEARTBEAT(seq, monotonic_ms, autonomy_state, confidence, ttl_ms)
JETSON_SET_BODY_TWIST(vx_mm_s, vy_mm_s, yaw_rate_mrad_s, ttl_ms)
JETSON_SET_NAV_INTENT(mode, desired_heading_mrad, speed_scale, gait_hint, clearance_hint_mm, confidence, ttl_ms)
JETSON_STOP(reason_code)
AUTONOMY_STATUS(state, selected_action, risk_score, heading_error, rc_blend, map_confidence)
```

## Development order

1. Camera bring-up and recording.
2. Jetson bridge telemetry only.
3. Safe flat-floor body-twist commands.
4. TF/URDF/camera calibration.
5. Visual odometry and state estimation.
6. Local 3D map.
7. Traversability grid.
8. Around-only obstacle avoidance with heading lock.
9. Low-step climb detection with RC confirmation.
10. Slow automatic climb over known test obstacles.
11. Contact feedback and recovery behaviors.
12. Staged field tests.

## Safety test ladder

1. Replay tests only.
2. Bench tests with no servo torque.
3. Suspended robot with torque and no ground contact.
4. One-leg/one-obstacle test.
5. Tethered flat-floor ground test.
6. Around-only obstacle test.
7. Known low-step climb test.
8. Mixed obstacle test with pilot ready to kill.

Every test produces a camera bag, MCU session log, config snapshot, and short operator notes.

## Firmware contact-mode and passive-stream integration updates

The Jetson autonomy stack must treat MCU contact features as optional capabilities, not assumptions. At startup the Jetson bridge should read firmware capabilities and feature states:

```text
GET_CAPABILITIES
GET_FEATURES
I2C_GET_TOPOLOGY
DXL_GET_SERVO_PROFILE
SUBSCRIBE contact_state, leveling_state, servo_status, passive_joint_state, feature_state
```

Rules:

- If `foot_contact_available` is false, autonomy must not request contact-dependent climbing behavior.
- If `terrain_leveling_enabled` is false or auto-disabled, autonomy must lower speed or choose bypass/stop for uncertain terrain.
- If contact data becomes stale while crossing an obstacle, autonomy must send a conservative stop or recovery intent.
- The Jetson may request contact-aware gait or terrain leveling through API feature toggles, but the MCU may reject or auto-disable them.
- Contact telemetry should annotate the traversability grid as positive/negative ground-truth feedback.

Passive pose streaming is useful before full autonomy:

- The Jetson/ROS 2 bridge should consume `passive_joint_state` when the MCU is in torque-off passive mode.
- ROS 2 joint states can drive `robot_state_publisher`, RViz, Foxglove, and calibration tooling while a user manually moves the robot.
- In passive mode, the Jetson must not send motion commands. It may only monitor, log, visualize, and publish model state.

Additional bridge topics to expose:

```text
/hexapod/features
/hexapod/i2c_topology
/hexapod/contact_state
/hexapod/leveling_state
/hexapod/passive_joint_state
/hexapod/dxl_profiles
/hexapod/dxl_param_events
```

