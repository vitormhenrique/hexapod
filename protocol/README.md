# Hexapod wire protocol

Binary protocol shared by the firmware, the Mac companion app, the Jetson
bridge, and test tools. One implementation per language, pinned to the same
golden vectors so they agree byte-for-byte.

## Frame format (v0.3)

```
0x00  COBS( header || payload || crc16 )  0x00
```

- **Delimiter:** `0x00` brackets every frame. COBS guarantees the encoded body
  contains no `0x00`.
- **CRC:** CRC-16/CCITT-FALSE (`poly=0x1021`, `init=0xFFFF`, no reflection,
  `xorout=0x0000`), computed over `header || payload`, appended little-endian.
- **Header (14 bytes, little-endian):**

  | field         | size | notes                         |
  | ------------- | ---: | ----------------------------- |
  | magic         |    1 | `0xA5`                        |
  | version_major |    1 | incompatible schema changes   |
  | version_minor |    1 | compatible additions          |
  | msg_type      |    1 | command/response/telemetry/event |
  | msg_id        |    1 | command or telemetry id       |
  | flags         |    1 | ack/error/fragment bits       |
  | seq           |    2 | per-sender sequence           |
  | timestamp_ms  |    4 | sender uptime                 |
  | payload_len   |    2 | bytes of payload              |

## Implementations

- **Firmware (C++, portable):** `firmware/openrb150/src/protocol/`
  (`crc16`, `cobs`, `framing`). No heap/STL; caller owns all buffers.
- **Host (Python):** `protocol/python/hexapod_protocol/`.

## Golden vectors

`tests/vectors/frames.json` is the single source of truth, generated from the
Python reference:

```bash
python protocol/tests/gen_vectors.py
```

Both test suites assert against these exact bytes.

## Running tests

Host (Python):

```bash
uv venv .venv && uv pip install --python .venv pytest
PYTHONPATH=protocol/python .venv/bin/python -m pytest protocol/tests -q
```

Firmware (native Unity, no hardware needed):

```bash
cd firmware/openrb150 && pio test -e native
```

Both suites cover CRC16 check values, COBS round-trips (including the 0xFF
block boundary at 254/255 bytes), full-frame encode/decode against the golden
vectors, and rejection of corrupt frames (bit flips, bad magic, truncation,
malformed COBS).

## Output-Disabled HIL Surface

Protocol v0.3 includes the append-only output-disabled HIL observation surface:

- `GET_CAPABILITIES` appends `build_flags` at byte 25; bit 0 means this is an
  immutable output-disabled image.
- `GET_STATUS` appends guard flags, blocked power/torque/goal/raw-write
  counters, the last blocked goal sequence, its target count, and the most
  recently latched fault reason/timestamp after the original 28-byte payload.
- `GET_STATUS` then appends a retained fatal record: fatal reason, startup
  stage, task name, stack pointer, exception return, and the Cortex-M0+ stacked
  `r0-r3`, `r12`, `lr`, `pc`, and `xPSR`. Older payload lengths decode these
  fields as zero.
- `health` appends the same guard summary and retained fault fields after its
  original 12-byte payload.
- Telemetry stream 12, `hil_status` (maximum 10 Hz), carries
  `flags(1), goal_count(1), last_goal_sequence(u32), blocked_power(u32),
  blocked_torque(u32), blocked_goal(u32), blocked_raw_write(u32)`, followed by
  `goal_count` exact `id(u8), tick(i32)` records from the last blocked Sync
  Write.

Older decoders retain the original fields. The HIL fields are software
evidence of a rejected output attempt, not proof that a physical DYNAMIXEL bus
remained silent; retain board-level rail and packet observations for HIL proof.
