# hexapod-protocol

Host-side Python implementation of the OpenRB-150 hexapod USB serial protocol.
UI-independent: shared by the companion CLI, the PySide6 app, and tests.

Mirrors the firmware under `firmware/openrb150/src/protocol/`.

## Contents

- `framing` — 14-byte header, COBS + CRC16 frame encode/decode
- `cobs`, `crc16` — wire primitives (golden-vector tested against firmware)
- `api` — command builders and response parsers (hello, status, capabilities,
  subscribe / set-rate / unsubscribe, stream stats)
- `telemetry` — per-stream decoders (health, control_state, servo_status,
  contact_state, i2c_sensors_raw, rc_input, api_stats)

## Wire format

```
0x00  COBS( header(14) || payload || crc16(2) )  0x00
```

Little-endian header: `magic(1) ver_major(1) ver_minor(1) msg_type(1) msg_id(1)
flags(1) seq(2) timestamp_ms(4) payload_len(2)`.

## Install / test

```bash
uv pip install -e .[dev]
pytest
```
