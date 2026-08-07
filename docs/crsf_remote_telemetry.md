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
| `0x80` Hexapod status | 5 Hz, 20 Hz while tuning | Versioned robot mode, gait, tuning, and error snapshot below |

The status frame is normally sent at 5 Hz. While the operator has the handset
gait-tune editor engaged it is the feedback loop for the edit, so `controlTask`
raises the rate to 20 Hz for 3 s after every NAV1 change. `rcTask` still emits
at most one telemetry frame per 10 ms cycle and still checks UART TX capacity
first, so the boost can never starve RC parsing.

The optional root-bus BNO085 is probed at `0x4A`, then `0x4B`. `i2cTask` is the
only task that configures or reads it. A bounded SHTP adapter enables a 20 Hz
rotation-vector report and converts its Q14 quaternion to Euler telemetry.
Missing or stale IMU data clears the corresponding status flags and suppresses
attitude frames; it never blocks or disables walking.

## Hexapod Status V2

All multi-byte fields are big-endian. The payload is exactly 28 bytes.

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | Magic `HX` (`0x48 0x58`) |
| 2 | 1 | Version (`2`) |
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
| 20 | 1 | Tune flags (below) |
| 21 | 1 | Error code (`safety::ErrorCode`), 0 = none |
| 22 | 1 | Error detail (servo id, foot index, fault reason, ... ) |
| 23 | 1 | Error sequence, bumped per announced incident, 0 = never announced |
| 24 | 2 | Occurrences in the current incident (saturating) |
| 26 | 2 | Duplicate occurrences the journal suppressed (saturating) |

Tune flag byte at offset 20:

| Bits | Field |
| --- | --- |
| 0 | Gait-tune editor engaged |
| 1 | Leg-1 preview running |
| 2 | A save is queued for the EEPROM transaction |
| 3 | Config store is volatile (a save would be rejected) |
| 4-5 | Selected parameter: 0 step height, 1 stride, 2 duty |
| 6-7 | Error severity: 0 info, 1 warning, 2 error, 3 critical |

Body height / stride / step height / speed / duty always report the values the
controller **actually applied** this cycle, so the handset readout, the gait
engine, and a subsequent save can never disagree.

### Error deduplication

Firmware error producers are level-triggered and run at 100 Hz, so they are fed
through `safety::ErrorJournal` (`firmware/openrb150/src/safety/error_journal.h`)
before reaching this frame. The journal keys a fixed 12-entry table on
`(code, detail)` and announces an entry only when it is new information:

- the first occurrence of a key,
- a repeat after 5 s of continued failure, carrying the running count,
- the first occurrence after the key has been quiet for 15 s.

Everything else is counted in `error_suppressed`. A stuck fault therefore costs
one downlink update every 5 s instead of 500. The remote uses
`error_sequence` to log a new incident once, then suppresses status-frame
duplicates and the 5 s still-failing heartbeat. It logs the same key again only
after the firmware has observed a quiet interval and restarts its occurrence
count, or when its severity escalates. The status page continues to show the
latest code, severity, and count live. Draining is highest-severity-first, and
a full table never evicts an unsent, at-least-as-severe entry, so a chatty
warning cannot hide a critical fault.

The controller accepts only exact-length, magic-matching version 2 payloads.
It keeps the last decoded values normally colored for two seconds, then retains
them in orange with a stale indicator until a new valid frame arrives. Any
incompatible layout change requires a new version and compatibility coverage
in both golden vector tests:

- Robot: `firmware/openrb150/test/test_crsf_telemetry/`
- Controller: `simulator/test_hexapod_telemetry.c`
