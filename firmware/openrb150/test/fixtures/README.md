# Controller Replay Fixtures

Controller replay fixtures are fixed-size C++ records used only by native
tests. They contain decoded portable controller contracts, never raw USB,
CRSF, DXL, I2C, Arduino, ROS, or wall-clock data.

## Format V1

`controller::replay::Fixture` has a `FixtureHeader` and up to 16
`ReplayFrame` values. The header must contain `format_version == 1` and
`time_unit == Milliseconds`.

Each frame supplies a full `ControllerTime`, `RobotState`, `ControllerIntent`,
and `ControllerConfigSnapshot`, followed by an `ExpectedOutput` semantic
oracle. `now_ms` and `dt_ms` are unsigned monotonic milliseconds. Values use
the units already defined by the portable contracts: millivolts, DYNAMIXEL
ticks, radians, millimeters, and normalized body twist.

The replay runner checks state, fault, source, configuration revision, motion
gate, torque policy, goal validity/count, gate edges, and requested semantic
goal changes. It intentionally avoids platform-specific floating-point golden
values. A future decoded firmware log may be normalized into the same contract
records before being added as another fixture.

`controller_arm_walk_estop_replay.h` is the initial synthetic fixture. It
covers the arm-to-stand path, RC walking gait, authority loss, and host E-stop.