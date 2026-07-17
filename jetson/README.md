# Hexapod Jetson Bridge

This package is the pure-Python, non-ROS portion of the Jetson integration. It
uses the same `transport.ProtocolClient` as the companion application to send
the firmware's high-level motion commands and receive typed telemetry.

The bridge only exposes body twist, body pose, gait selection, gait parameters,
stop, and telemetry subscription APIs. It does not expose arming, torque,
DYNAMIXEL, raw-register, or raw-joint controls.

The OpenRB-150 remains the authority for all motion and safety decisions. A
fresh Jetson heartbeat alone is insufficient: firmware also requires the
Jetson feature flag, RC arming, and RC autonomy approval before it accepts
Jetson motion authority.

## Development

From the monorepo root:

```bash
just jetson-sync
just jetson-test
```

## Basic Use

```python
from hexapod_jetson_bridge import JetsonBridge

bridge = JetsonBridge.connect("/dev/ttyACM0")
if bridge is None:
    raise RuntimeError("OpenRB-150 handshake failed")

try:
    bridge.set_gait(3)  # ripple
    bridge.set_body_twist(0.2, 0.0, 0.0)
finally:
    bridge.stop_motion()
    bridge.close()
```

`JetsonBridge.start()` subscribes to health and control-state telemetry and
maintains a Jetson-specific heartbeat at 10 Hz. Applications can add further
rate-limited subscriptions through `subscribe()`. ROS 2 publishers and
subscribers use the optional adapter described below.

## Optional ROS 2 Adapter

The ROS 2 adapter runs from the RoboStack Jazzy workspace. It subscribes to
`hexapod_msgs/msg/MotionCommand` and forwards only high-level gait, gait
parameters, twist, and body-pose requests through `JetsonBridge`.

From the monorepo root:

```bash
cd robot_ros_simulation
just install
just build
just jetson-ros /dev/cu.usbmodem2101
```

The node subscribes to `motion_command` by default. Remap it with ROS arguments
passed after the serial port, for example:

```bash
just jetson-ros /dev/cu.usbmodem2101 --remap motion_command:=autonomy/motion_command
```

`MotionCommand.valid_for` is required and is capped at one second. The adapter
issues one `STOP_MOTION` request when that local TTL expires; its heartbeat
continues independently. The mapping rejects unknown gaits and non-finite
values, clamps the request to the firmware motion envelope, converts metres to
millimetres and radians to degrees, and still relies on firmware for final
validation.

The ROS adapter requires a real OpenRB-150 USB connection. It is not a
simulation controller and it does not grant motion authority: the firmware
still requires its Jetson feature enabled, RC arming, RC autonomy approval,
and a fresh heartbeat before motion is accepted.

## Mac TCP Relay

When the Jetson owns the OpenRB-150 USB device, it can expose that same link to
one Mac companion client over TCP. The relay is byte-transparent after a
token-authentication preamble, so the companion retains its existing framed
protocol, request sequence correlation, telemetry subscriptions, and all
firmware safety checks.

On the Jetson, start the relay with the OpenRB-150 serial device. It listens on
loopback by default; use a Jetson LAN address or `0.0.0.0` only on a trusted
private network:

```bash
cd <clone>/hexapod
just jetson-relay --serial-port /dev/ttyACM0 --host 0.0.0.0
```

Without `--token`, the relay generates and prints a new token. Copy the printed
endpoint to the Mac, replacing `<jetson-host>` with the Jetson hostname or IP:

```bash
uv run --project companion hexapod-cli status \
    --port 'tcp://<jetson-host>:5555?token=<printed-token>'
```

The companion Connect page also accepts this endpoint directly in its editable
connection field. A relay restart generates a different token unless an
explicit `--token` is supplied.

The relay deliberately owns the USB stream exclusively and rejects concurrent
Mac clients. Do not run it against the same MCU serial device as a
`JetsonBridge` autonomy process: two readers would race for MCU responses. The
relay is not TLS-encrypted, so use it only on a trusted network or through an
SSH tunnel. It never rewrites commands, manufactures a Jetson heartbeat, or
bypasses firmware arming, RC approval, maintenance locks, or emergency-stop
rules.