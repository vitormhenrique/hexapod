"""``hexapod-cli`` — scriptable companion CLI (no Qt required).

Connects to the robot over USB and exposes status, capabilities, telemetry
streaming, and session logging. Designed to run from a clean environment so it
works before the GUI is installed.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Optional

import typer

from hexapod_protocol import telemetry as tlm

from .transport import list_serial_ports, open_serial
from .transport.protocol_client import ProtocolClient
from .data import SessionLogger

app = typer.Typer(add_completion=False, help="Hexapod companion CLI.")


def _err(msg: str) -> None:
    typer.secho(msg, fg=typer.colors.RED, err=True)


def _connect(port: Optional[str], baud: int) -> ProtocolClient:
    if port is None:
        ports = list_serial_ports()
        usb = [p for p in ports if "usbmodem" in p.device or "ACM" in p.device]
        chosen = (usb or ports)
        if not chosen:
            _err("No serial ports found. Pass --port explicitly.")
            raise typer.Exit(code=2)
        port = chosen[0].device
        typer.secho(f"Using port {port}", fg=typer.colors.BLUE)
    link = open_serial(port, baud=baud)
    if link is None:
        _err(f"Could not open {port}.")
        raise typer.Exit(code=2)
    client = ProtocolClient(link)
    client.start()
    return client


@app.command()
def ports() -> None:
    """List available serial ports."""
    found = list_serial_ports()
    if not found:
        typer.echo("No serial ports found.")
        return
    for p in found:
        typer.echo(f"{p.device:24}  {p.description}")


@app.command()
def status(
    port: Optional[str] = typer.Option(None, "--port", "-p"),
    baud: int = typer.Option(115200, "--baud"),
) -> None:
    """Connect, handshake, and print firmware status + capabilities."""
    client = _connect(port, baud)
    try:
        hello = client.hello()
        if hello is None:
            _err("No HELLO response (timeout).")
            raise typer.Exit(code=1)
        typer.secho(
            f"{hello.device_name}  fw {hello.fw_major}.{hello.fw_minor}.{hello.fw_patch}"
            f"  proto {hello.proto_major}.{hello.proto_minor}",
            fg=typer.colors.GREEN,
        )
        st = client.get_status()
        if st:
            typer.echo(
                f"  state={tlm.SAFETY_STATE_NAMES.get(st.state, st.state)} "
                f"uptime={st.uptime_ms} ms  dxl_power={st.dxl_power} "
                f"battery={st.battery_mv} mV  watchdog_missed=0x{st.watchdog_missed:X}"
            )
        caps = client.get_capabilities()
        if caps:
            typer.echo(f"  feature_bits=0x{caps.feature_bits:08X}")
    finally:
        client.stop()


@app.command()
def stream(
    streams: str = typer.Argument(
        ..., help="Comma-separated stream names, e.g. health,servo_status"
    ),
    rate: int = typer.Option(10, "--rate", "-r", help="Requested rate (Hz)."),
    seconds: float = typer.Option(5.0, "--seconds", "-s"),
    port: Optional[str] = typer.Option(None, "--port", "-p"),
    baud: int = typer.Option(115200, "--baud"),
) -> None:
    """Subscribe to telemetry streams and print decoded records live."""
    names = [s for s in streams.split(",") if s.strip()]
    try:
        ids = [tlm.stream_id_from_name(n) for n in names]
    except ValueError as exc:
        _err(str(exc))
        raise typer.Exit(code=2)

    client = _connect(port, baud)
    count = {"n": 0}

    def on_tel(stream_id: int, record: object, header) -> None:
        count["n"] += 1
        name = tlm.STREAM_NAMES.get(tlm.StreamId(stream_id), str(stream_id))
        typer.echo(f"[{header.timestamp_ms:>8} ms] {name}: {record}")

    client.on_telemetry(on_tel)
    try:
        for sid in ids:
            res = client.subscribe(int(sid), rate)
            if res and res.ok:
                typer.secho(
                    f"subscribed {tlm.STREAM_NAMES[tlm.StreamId(int(sid))]} "
                    f"@ {res.effective_rate_hz} Hz",
                    fg=typer.colors.GREEN,
                )
            else:
                _err(f"subscribe failed for stream {sid}: {res}")
        time.sleep(seconds)
        typer.secho(f"\n{count['n']} telemetry frames received.", fg=typer.colors.BLUE)
    finally:
        for sid in ids:
            client.unsubscribe(int(sid))
        client.stop()


@app.command()
def log(
    streams: str = typer.Option(
        "health,servo_status,contact_state,i2c_sensors_raw,rc_input",
        "--streams",
    ),
    rate: int = typer.Option(20, "--rate", "-r"),
    seconds: float = typer.Option(10.0, "--seconds", "-s"),
    out: Path = typer.Option(Path("data/sessions"), "--out"),
    name: str = typer.Option("hexapod", "--name"),
    port: Optional[str] = typer.Option(None, "--port", "-p"),
    baud: int = typer.Option(115200, "--baud"),
) -> None:
    """Record a telemetry session (raw frames + decoded JSONL)."""
    names = [s for s in streams.split(",") if s.strip()]
    try:
        ids = [tlm.stream_id_from_name(n) for n in names]
    except ValueError as exc:
        _err(str(exc))
        raise typer.Exit(code=2)

    client = _connect(port, baud)
    hello = client.hello()
    fw = (
        {
            "device": hello.device_name,
            "fw": f"{hello.fw_major}.{hello.fw_minor}.{hello.fw_patch}",
        }
        if hello
        else None
    )
    logger = SessionLogger(out_dir=out, robot_name=name, firmware=fw)
    typer.secho(f"recording -> {logger.dir}", fg=typer.colors.BLUE)

    def on_tel(stream_id: int, record: object, header) -> None:
        logger.log_record(
            tlm.STREAM_NAMES.get(tlm.StreamId(stream_id), str(stream_id)),
            record,
            robot_time_ms=header.timestamp_ms,
        )

    client.on_telemetry(on_tel)
    try:
        for sid in ids:
            client.subscribe(int(sid), rate)
        time.sleep(seconds)
    finally:
        for sid in ids:
            client.unsubscribe(int(sid))
        client.stop()
        logger.close()
    typer.secho(
        f"done: {logger._meta.record_count} records, "  # noqa: SLF001
        f"{logger._meta.frame_count} raw frames.",
        fg=typer.colors.GREEN,
    )


@app.command("stream-stats")
def stream_stats(
    port: Optional[str] = typer.Option(None, "--port", "-p"),
    baud: int = typer.Option(115200, "--baud"),
) -> None:
    """Print the firmware's per-stream emit/drop counters."""
    client = _connect(port, baud)
    try:
        stats = client.get_stream_stats()
        if stats is None:
            _err("No GET_STREAM_STATS response.")
            raise typer.Exit(code=1)
        typer.echo(f"tx_backlog={stats.tx_backlog}")
        for s in stats.streams:
            name = tlm.STREAM_NAMES.get(tlm.StreamId(s.stream_id), str(s.stream_id))
            typer.echo(
                f"  {name:18} enabled={s.enabled} rate={s.rate_hz:>3} Hz "
                f"emitted={s.emitted} dropped={s.dropped}"
            )
    finally:
        client.stop()


@app.command()
def gui() -> None:
    """Launch the PySide6 companion app (same as ``hexapod-companion``)."""
    from .app import main as gui_main

    gui_main()


def main() -> None:
    app()


if __name__ == "__main__":
    sys.exit(app())
