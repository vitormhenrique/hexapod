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

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm

from transport import list_serial_ports, open_serial
from transport.protocol_client import ProtocolClient
from data import SessionLogger

app = typer.Typer(add_completion=False, help="Hexapod companion CLI.")


def _err(msg: str) -> None:
    typer.secho(msg, fg=typer.colors.RED, err=True)


def _connect(port: Optional[str], baud: int) -> ProtocolClient:
    if port is None:
        ports = list_serial_ports()
        usb = [p for p in ports if "usbmodem" in p.device or "ACM" in p.device]
        chosen = usb or ports
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
    from app import main as gui_main

    gui_main()


# ----------------------------------------------------------------------------
# Control + tuning subcommands (kj8.3). Each connects, sends one safe command,
# prints the firmware response, and disconnects. The MCU remains the final
# safety gate, so motion commands echo the firmware's accept/reject verdict.
# ----------------------------------------------------------------------------

_PORT = typer.Option(None, "--port", "-p")
_BAUD = typer.Option(115200, "--baud")


def _state_name(state: int) -> str:
    return tlm.SAFETY_STATE_NAMES.get(state, f"0x{state:02X}")


def _show_control(res) -> None:
    if res is None:
        _err("no response (timeout or rejected).")
        raise typer.Exit(code=1)
    verdict = "ok" if res.ok else "REJECTED"
    color = typer.colors.GREEN if res.ok else typer.colors.YELLOW
    typer.secho(
        f"{verdict}  state={_state_name(res.state)} fault={res.fault}", fg=color
    )


def _show_motion(res) -> None:
    if res is None:
        _err("no response (timeout or rejected).")
        raise typer.Exit(code=1)
    verdict = "ok" if res.ok else "REJECTED"
    color = typer.colors.GREEN if res.ok else typer.colors.YELLOW
    typer.secho(
        f"{verdict}  state={_state_name(res.state)} "
        f"motion_allowed={res.motion_allowed}",
        fg=color,
    )


def _show_sensor_feature(res) -> None:
    if res is None:
        _err("no response.")
        raise typer.Exit(code=1)
    color = typer.colors.GREEN if res.ok else typer.colors.YELLOW
    typer.secho(
        f"available={res.available} enabled={res.enabled} reason={res.reason}",
        fg=color,
    )


def _show_passive(res) -> None:
    if res is None:
        _err("no response.")
        raise typer.Exit(code=1)
    color = typer.colors.GREEN if res.ok else typer.colors.YELLOW
    typer.secho(f"result={res.result} state={_state_name(res.state)}", fg=color)


def _require_done(res):
    if res is None or not res.done:
        _err("DXL job did not complete (rejected or timed out).")
        raise typer.Exit(code=1)
    return res


# --- safety ---------------------------------------------------------------

safety_app = typer.Typer(help="Arming, e-stop, fault, and mode control.")
app.add_typer(safety_app, name="safety")


@safety_app.command("arm")
def safety_arm(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Release the host disarm latch (RC arm switch still required to walk)."""
    client = _connect(port, baud)
    try:
        _show_control(client.set_arming(True))
    finally:
        client.stop()


@safety_app.command("disarm")
def safety_disarm(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Latch the robot disarmed."""
    client = _connect(port, baud)
    try:
        _show_control(client.set_arming(False))
    finally:
        client.stop()


@safety_app.command("estop")
def safety_estop(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Trigger an emergency stop."""
    client = _connect(port, baud)
    try:
        _show_control(client.estop())
    finally:
        client.stop()


@safety_app.command("clear-fault")
def safety_clear_fault(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Clear a soft fault latch."""
    client = _connect(port, baud)
    try:
        _show_control(client.clear_fault())
    finally:
        client.stop()


@safety_app.command("mode")
def safety_mode(
    mode: int = typer.Argument(..., help="Safety-reducing mode id (firmware-defined)."),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Request a safety-reducing mode (firmware only honors safe transitions)."""
    client = _connect(port, baud)
    try:
        _show_control(client.set_mode(mode))
    finally:
        client.stop()


# --- gait / motion --------------------------------------------------------

gait_app = typer.Typer(help="Gait selection, parameters, and body twist.")
app.add_typer(gait_app, name="gait")

_GAITS = {
    "stand": api.GAIT_STAND,
    "sit": api.GAIT_SIT,
    "tripod": api.GAIT_TRIPOD,
    "ripple": api.GAIT_RIPPLE,
    "wave": api.GAIT_WAVE,
    "crawl": api.GAIT_CRAWL,
}


@gait_app.command("set")
def gait_set(
    gait: str = typer.Argument(..., help="stand|sit|tripod|ripple|wave|crawl"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Select a gait."""
    key = gait.strip().lower()
    if key not in _GAITS:
        _err(f"unknown gait '{gait}'. Choices: {', '.join(_GAITS)}")
        raise typer.Exit(code=2)
    client = _connect(port, baud)
    try:
        _show_motion(client.set_gait(_GAITS[key]))
    finally:
        client.stop()


@gait_app.command("params")
def gait_params(
    body_height: int = typer.Option(..., "--body-height", help="mm"),
    stride: int = typer.Option(..., "--stride", help="mm"),
    step_height: int = typer.Option(..., "--step-height", help="mm"),
    duty: int = typer.Option(128, "--duty", help="0..255"),
    speed: int = typer.Option(128, "--speed", help="0..255"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Set gait parameters (lengths in mm; duty/speed 0..255)."""
    client = _connect(port, baud)
    try:
        _show_motion(
            client.set_gait_params(body_height, stride, step_height, duty, speed)
        )
    finally:
        client.stop()


@gait_app.command("twist")
def gait_twist(
    vx: float = typer.Option(0.0, "--vx", help="forward, -1..1"),
    vy: float = typer.Option(0.0, "--vy", help="lateral, -1..1"),
    wz: float = typer.Option(0.0, "--wz", help="yaw, -1..1"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Command a body twist (normalized velocities)."""
    client = _connect(port, baud)
    try:
        _show_motion(client.set_body_twist(vx, vy, wz))
    finally:
        client.stop()


@gait_app.command("stop")
def gait_stop(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Stop all motion."""
    client = _connect(port, baud)
    try:
        _show_motion(client.stop_motion())
    finally:
        client.stop()


# --- features -------------------------------------------------------------

feature_app = typer.Typer(help="Feature flag get/set.")
app.add_typer(feature_app, name="feature")

_FEATURES = {
    "foot_contact": api.FEATURE_FOOT_CONTACT,
    "terrain_leveling": api.FEATURE_TERRAIN_LEVELING,
    "sensor_polling": api.FEATURE_SENSOR_POLLING,
    "jetson_control": api.FEATURE_JETSON_CONTROL,
    "passive_pose": api.FEATURE_PASSIVE_POSE,
}
_FEATURE_NAMES = {v: k for k, v in _FEATURES.items()}


@feature_app.command("get")
def feature_get(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """List every feature's available/enabled state and reason."""
    client = _connect(port, baud)
    try:
        fl = client.feature_get()
        if fl is None:
            _err("no FEATURE_GET response.")
            raise typer.Exit(code=1)
        for f in fl.features:
            name = _FEATURE_NAMES.get(f.feature, str(f.feature))
            typer.echo(
                f"  {name:18} available={f.available} enabled={f.enabled} "
                f"reason={f.reason}"
            )
    finally:
        client.stop()


@feature_app.command("set")
def feature_set(
    feature: str = typer.Argument(..., help="|".join(_FEATURES)),
    enable: bool = typer.Argument(..., help="true|false"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Enable or disable a feature (firmware may reject with a reason)."""
    key = feature.strip().lower()
    if key not in _FEATURES:
        _err(f"unknown feature '{feature}'. Choices: {', '.join(_FEATURES)}")
        raise typer.Exit(code=2)
    client = _connect(port, baud)
    try:
        res = client.feature_set(_FEATURES[key], enable)
        if res is None:
            _err("no FEATURE_SET response.")
            raise typer.Exit(code=1)
        color = typer.colors.GREEN if res.ok else typer.colors.YELLOW
        typer.secho(
            f"available={res.available} enabled={res.enabled} reason={res.reason}",
            fg=color,
        )
    finally:
        client.stop()


# --- contact / leveling ---------------------------------------------------

contact_app = typer.Typer(help="Foot-contact enable/disable/calibrate.")
app.add_typer(contact_app, name="contact")


@contact_app.command("enable")
def contact_enable(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Enable foot-contact detection."""
    client = _connect(port, baud)
    try:
        _show_sensor_feature(client.contact_enable(True))
    finally:
        client.stop()


@contact_app.command("disable")
def contact_disable(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Disable foot-contact detection."""
    client = _connect(port, baud)
    try:
        _show_sensor_feature(client.contact_enable(False))
    finally:
        client.stop()


@contact_app.command("calibrate")
def contact_calibrate(
    foot: int = typer.Option(
        api.SENSOR_CALIBRATE_ALL, "--foot", help="0..5 or 255 for all"
    ),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Re-zero the pressure baseline (feet must be at rest)."""
    client = _connect(port, baud)
    try:
        res = client.contact_calibrate(foot)
        if res is None:
            _err("no CONTACT_CALIBRATE response.")
            raise typer.Exit(code=1)
        color = typer.colors.GREEN if res.ok else typer.colors.YELLOW
        typer.secho(f"result={res.result} mask=0x{res.mask:02X}", fg=color)
    finally:
        client.stop()


# --- passive pose ---------------------------------------------------------

passive_app = typer.Typer(help="Torque-off passive pose streaming.")
app.add_typer(passive_app, name="passive")


@passive_app.command("enter")
def passive_enter(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Enter torque-off passive pose mode (robot goes limp)."""
    client = _connect(port, baud)
    try:
        _show_passive(client.passive_enter())
    finally:
        client.stop()


@passive_app.command("exit")
def passive_exit(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Exit passive pose mode."""
    client = _connect(port, baud)
    try:
        _show_passive(client.passive_exit())
    finally:
        client.stop()


# --- DXL maintenance ------------------------------------------------------

dxl_app = typer.Typer(help="DXL scan/ping/torque and logical parameter access.")
app.add_typer(dxl_app, name="dxl")


def _resolve_param(name: str) -> int:
    """Map a logical param name (e.g. ``torque_limit``) to its DXL_PARAM_* id."""
    const = f"DXL_PARAM_{name.strip().upper()}"
    pid = getattr(api, const, None)
    if pid is None or not isinstance(pid, int) or name.upper() == "COUNT":
        names = [
            n[len("DXL_PARAM_") :].lower()
            for n in dir(api)
            if n.startswith("DXL_PARAM_") and n != "DXL_PARAM_COUNT"
        ]
        _err(f"unknown param '{name}'. Choices: {', '.join(sorted(names))}")
        raise typer.Exit(code=2)
    return pid


@dxl_app.command("scan")
def dxl_scan(
    first: int = typer.Option(1, "--first"),
    last: int = typer.Option(252, "--last"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Scan the DXL bus for servos."""
    client = _connect(port, baud)
    try:
        res = _require_done(client.dxl_scan(first, last))
        servos = res.servos()
        typer.secho(f"{len(servos)} servo(s) found.", fg=typer.colors.GREEN)
        for s in servos:
            typer.echo(
                f"  id={s.id} model={s.model} fw={s.firmware} "
                f"proto={s.protocol} table={s.table_kind}"
            )
    finally:
        client.stop()


@dxl_app.command("ping")
def dxl_ping(
    servo_id: int = typer.Argument(...),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Ping a single servo."""
    client = _connect(port, baud)
    try:
        res = _require_done(client.dxl_ping(servo_id))
        typer.secho(f"code={res.code} data={res.data.hex()}", fg=typer.colors.GREEN)
    finally:
        client.stop()


@dxl_app.command("torque")
def dxl_torque(
    on: bool = typer.Argument(..., help="true|false"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Enable or disable torque on all servos (maintenance only)."""
    client = _connect(port, baud)
    try:
        res = _require_done(client.dxl_torque(on))
        typer.secho(f"code={res.code} data={res.data.hex()}", fg=typer.colors.GREEN)
    finally:
        client.stop()


@dxl_app.command("power")
def dxl_power(
    on: bool = typer.Argument(..., help="true|false"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Enable or disable the DYNAMIXEL power FET (maintenance only).

    Firmware only accepts this while in MacMaintenance with the bench lock held,
    and force-cuts power on any exit from maintenance (disarm, estop, fault).
    """
    client = _connect(port, baud)
    try:
        res = _require_done(client.dxl_power(on))
        pr = res.power()
        if pr is not None:
            typer.secho(
                f"power_on={pr.power_on} has_control={pr.has_control}",
                fg=typer.colors.GREEN,
            )
        else:
            typer.secho(f"code={res.code} data={res.data.hex()}", fg=typer.colors.GREEN)
    finally:
        client.stop()


@dxl_app.command("get")
def dxl_get(
    servo_id: int = typer.Argument(...),
    param: str = typer.Argument(..., help="logical param name, e.g. torque_limit"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Read a logical servo parameter."""
    pid = _resolve_param(param)
    client = _connect(port, baud)
    try:
        res = _require_done(client.dxl_get_param(servo_id, pid))
        pv = res.param()
        if pv is None:
            _err(f"read failed (code {res.code}).")
            raise typer.Exit(code=1)
        typer.secho(
            f"{param} = {pv.value}  (table {pv.table_kind})", fg=typer.colors.GREEN
        )
    finally:
        client.stop()


@dxl_app.command("set")
def dxl_set(
    servo_id: int = typer.Argument(...),
    param: str = typer.Argument(..., help="logical param name"),
    value: int = typer.Argument(...),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Write a logical servo parameter (EEPROM params force torque-off)."""
    pid = _resolve_param(param)
    client = _connect(port, baud)
    try:
        res = _require_done(client.dxl_set_param(servo_id, pid, value))
        sp = res.set_param()
        if sp is None:
            _err(f"write failed (code {res.code}).")
            raise typer.Exit(code=1)
        color = typer.colors.GREEN if sp.verified else typer.colors.YELLOW
        typer.secho(
            f"wrote {sp.written}, read-back {sp.readback}, verified={sp.verified}",
            fg=color,
        )
    finally:
        client.stop()


@dxl_app.command("limits")
def dxl_limits(
    servo_id: int = typer.Argument(...),
    min_tick: int = typer.Argument(...),
    max_tick: int = typer.Argument(...),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Write servo position limits (legacy CW/CCW or MX2.0 min/max)."""
    if min_tick >= max_tick:
        _err("min must be < max.")
        raise typer.Exit(code=2)
    client = _connect(port, baud)
    try:
        res = _require_done(client.dxl_set_servo_limits(servo_id, min_tick, max_tick))
        sl = res.servo_limits()
        if sl is None:
            _err(f"write failed (code {res.code}).")
            raise typer.Exit(code=1)
        color = typer.colors.GREEN if sl.verified else typer.colors.YELLOW
        typer.secho(
            f"table {sl.table_kind}: [{sl.min_tick}, {sl.max_tick}] "
            f"verified={sl.verified}",
            fg=color,
        )
    finally:
        client.stop()


def main() -> None:
    app()


if __name__ == "__main__":
    sys.exit(app())
