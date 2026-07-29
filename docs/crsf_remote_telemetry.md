# CRSF Remote Telemetry

The OpenRB-150 sends a bounded telemetry downlink to the ESP32-S3 remote over
the same CRSF connection used for RC commands. `rcTask` remains the only
`Serial3` owner. It reads receiver frames first and queues at most one telemetry
frame per 10 ms cycle after checking UART TX capacity.

## Wiring

| OpenRB-150 | ELRS receiver |
| --- | --- |
| D13 / `Serial3 RX` | CRSF TX |
| D14 / `Serial3 TX` | CRSF RX |
| GND | GND |

A one-way receiver connection still controls the robot but cannot return robot
telemetry to the handset.

## Streams

| CRSF type | Rate | Payload |
| --- | ---: | --- |
| `0x08` Battery | 2 Hz | Standard CRSF voltage and estimated 3S remaining percent |
| `0x1E` Attitude | 10 Hz when fresh | Standard CRSF pitch, roll, yaw in radians x 10000 |
| `0x80` Hexapod status | 5 Hz | Versioned robot mode and gait snapshot below |

The optional root-bus BNO085 is probed at `0x4A`, then `0x4B`. `i2cTask` is the
only task that configures or reads it. A bounded SHTP adapter enables a 20 Hz
rotation-vector report and converts its Q14 quaternion to Euler telemetry.
Missing or stale IMU data clears the corresponding status flags and suppresses
attitude frames; it never blocks or disables walking.

## Hexapod Status V1

All multi-byte fields are big-endian. The payload is exactly 20 bytes.

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | Magic `HX` (`0x48 0x58`) |
| 2 | 1 | Version (`1`) |
| 3 | 1 | Flags: armed, motion gate, kill, failsafe, IMU present/fresh, fault, battery valid |
| 4 | 1 | Safety state (`safety::State`) |
| 5 | 1 | Command source (`safety::CommandSource`) |
| 6 | 1 | Gait (`config::GaitId`) |
| 7 | 1 | Controller mode (walk/translate/rotate) |
| 8 | 1 | Fault reason (`safety::FaultReason`) |
| 9 | 1 | Speed, 0-255 |
| 10 | 1 | Duty factor, 0-255 |
| 11 | 1 | BNO085 rotation-vector quality (0-3 repeated in each 2-bit field) |
| 12 | 2 | Battery, mV |
| 14 | 2 | Body height, mm |
| 16 | 2 | Stride, mm |
| 18 | 2 | Step height, mm |

The controller accepts only exact-length, magic-matching version 1 payloads.
It keeps the last decoded values normally colored for two seconds, then retains
them in orange with a stale indicator until a new valid frame arrives. Any
incompatible layout change requires a new version and compatibility coverage
in both golden vector tests:

- Robot: `firmware/openrb150/test/test_crsf_telemetry/`
- Controller: `simulator/test_hexapod_telemetry.c`
