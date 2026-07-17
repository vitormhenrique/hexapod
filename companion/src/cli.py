"""``hexapod-cli`` — scriptable companion CLI (no Qt required).

Connects to the robot over USB and exposes status, capabilities, telemetry
streaming, and session logging. Designed to run from a clean environment so it
works before the GUI is installed.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import typer

from hexapod_protocol import api
from hexapod_protocol import hil
from hexapod_protocol import telemetry as tlm

from transport import list_serial_ports, open_transport
from transport.protocol_client import ProtocolClient
from data import SessionLogger, SessionReplay, export_selected_csv, write_session_summary
from data.plot_signals import build_signal_registry, registry_by_key

app = typer.Typer(add_completion=False, help="Hexapod companion CLI.")

CONNECT_ATTEMPTS = 5
CONNECT_RETRY_DELAY_S = 2.5
REQUEST_ATTEMPTS = 3
REQUEST_RETRY_DELAY_S = 1.0

RESET_CAUSE_NAMES = {
    0x01: "POR",
    0x02: "BOD12",
    0x04: "BOD33",
    0x10: "EXT",
    0x20: "WDT",
    0x40: "SYST",
}

FATAL_REASON_NAMES = {1: "HardFault", 2: "StackOverflow", 3: "MallocFailed"}
STARTUP_STAGE_NAMES = {
    0: "Reset",
    1: "SetupEntered",
    2: "BoardInitialized",
    3: "SerialStarted",
    4: "AppStarting",
    5: "SchedulerStarting",
    6: "TasksRunning",
}


def _reset_cause(value: int) -> str:
    names = [name for bit, name in RESET_CAUSE_NAMES.items() if value & bit]
    return "+".join(names) if names else "unknown"


def _err(msg: str) -> None:
    typer.secho(msg, fg=typer.colors.RED, err=True)


def _port_score(port_info) -> int:
    haystack = f"{port_info.device} {port_info.description} {port_info.hwid}".lower()
    if "bluetooth" in haystack:
        return -100
    score = 0
    if "openrb" in haystack:
        score += 100
    if "usbmodem" in haystack or "acm" in haystack:
        score += 50
    if str(port_info.device).startswith("/dev/cu."):
        score += 20
    return score


def _connect(port: Optional[str], baud: int) -> ProtocolClient:
    if port is None:
        ports = list_serial_ports()
        chosen = sorted(ports, key=_port_score, reverse=True)
        if not chosen:
            _err("No serial ports found. Pass --port explicitly.")
            raise typer.Exit(code=2)
        port = chosen[0].device
        typer.secho(f"Using port {port}", fg=typer.colors.BLUE)
    link = open_transport(port, baud=baud)
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
    last_error = "timeout"
    for attempt in range(1, CONNECT_ATTEMPTS + 1):
        client = _connect(port, baud)
        try:
            hello = _retry_request(client, "HELLO", client.hello)
            st = _retry_request(client, "STATUS", client.get_status)
            caps = _retry_request(client, "CAPABILITIES", client.get_capabilities)

            typer.secho(
                f"{hello.device_name}  fw {hello.fw_major}.{hello.fw_minor}.{hello.fw_patch}"
                f"  proto {hello.proto_major}.{hello.proto_minor}",
                fg=typer.colors.GREEN,
            )
            typer.echo(
                f"  state={tlm.SAFETY_STATE_NAMES.get(st.state, st.state)} "
                f"uptime={st.uptime_ms} ms  dxl_power={st.dxl_power} "
                f"battery={st.battery_mv} mV  watchdog_missed=0x{st.watchdog_missed:X} "
                f"reset_cause=0x{st.reset_cause:02X}({_reset_cause(st.reset_cause)}) "
                f"last_reset_watchdog_missed=0x{st.last_reset_watchdog_missed:X} "
                f"last_reset_dxl_progress={st.last_reset_progress_marker} "
                f"last_reset_control_progress={st.last_reset_control_progress} "
                f"last_reset_state={st.last_reset_safety_state} "
                f"dxl_power_transitions={st.dxl_power_transitions} "
                f"last_fault={tlm.FAULT_REASON_NAMES.get(st.last_fault_reason, st.last_fault_reason)} "
                f"last_fault_time_ms={st.last_fault_timestamp_ms}"
            )
            typer.echo(
                f"  stack_free_words: control={st.control_stack_free_words} "
                f"dxl={st.dxl_stack_free_words}"
            )
            if st.last_fatal_reason:
                typer.echo(
                    "  retained_fatal: "
                    f"reason={FATAL_REASON_NAMES.get(st.last_fatal_reason, st.last_fatal_reason)} "
                    f"stage={STARTUP_STAGE_NAMES.get(st.last_fatal_stage, st.last_fatal_stage)} "
                    f"task={st.last_fatal_task_name or '(none)'} "
                    f"sp=0x{st.last_fault_stack_pointer:08X} "
                    f"exc_return=0x{st.last_fault_exception_return:08X}"
                )
                typer.echo(
                    "  retained_frame: "
                    f"r0=0x{st.last_fault_r0:08X} r1=0x{st.last_fault_r1:08X} "
                    f"r2=0x{st.last_fault_r2:08X} r3=0x{st.last_fault_r3:08X} "
                    f"r12=0x{st.last_fault_r12:08X} lr=0x{st.last_fault_lr:08X} "
                    f"pc=0x{st.last_fault_pc:08X} xpsr=0x{st.last_fault_xpsr:08X}"
                )
            typer.echo(f"  feature_bits=0x{caps.feature_bits:08X}")
            avail = [
                api.FEATURE_NAMES[i] for i in api.capability_features(caps.feature_bits)
            ]
            typer.echo(
                f"  features_available={', '.join(avail) if avail else '(none)'}"
            )
            return
        except Exception as exc:
            last_error = str(exc)
            if attempt >= CONNECT_ATTEMPTS:
                break
            typer.secho(
                f"Retrying serial status ({attempt + 1}/{CONNECT_ATTEMPTS})...",
                fg=typer.colors.YELLOW,
                err=True,
            )
            time.sleep(CONNECT_RETRY_DELAY_S)
        finally:
            client.stop()

    _err(last_error)
    raise typer.Exit(code=1)


def _retry_request(client: ProtocolClient, name: str, call):
    last_error = f"No {name} response (timeout)."
    for attempt in range(1, REQUEST_ATTEMPTS + 1):
        result = call()
        if result is not None:
            return result
        if not client.connected:
            raise RuntimeError(last_error)
        if attempt < REQUEST_ATTEMPTS:
            time.sleep(REQUEST_RETRY_DELAY_S)
    raise RuntimeError(last_error)


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
    client.on_raw_frame(logger.log_raw_frame)
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


@app.command("export-csv")
def export_csv(
    session_dir: Path = typer.Argument(..., exists=True, file_okay=False, dir_okay=True),
    signals: str = typer.Option(
        ...,
        "--signals",
        "-s",
        help="Comma-separated signal keys, e.g. health.battery_mv,servo.1.position.",
    ),
    out: Optional[Path] = typer.Option(None, "--out", help="CSV output path."),
) -> None:
    """Export selected recorded telemetry signals to a timestamped CSV."""
    requested = [key.strip() for key in signals.split(",") if key.strip()]
    registry = registry_by_key(build_signal_registry())
    unknown = [key for key in requested if key not in registry]
    if not requested:
        _err("Select at least one signal with --signals.")
        raise typer.Exit(code=2)
    if unknown:
        _err(f"Unknown signal key(s): {', '.join(unknown)}")
        raise typer.Exit(code=2)

    target = out or session_dir / "selected_signals.csv"
    try:
        rows = export_selected_csv(
            SessionReplay(session_dir), [registry[key] for key in requested], target
        )
    except (OSError, ValueError) as exc:
        _err(f"CSV export failed: {exc}")
        raise typer.Exit(code=1)
    typer.secho(f"exported {rows} CSV row(s) -> {target}", fg=typer.colors.GREEN)


@app.command("export-report")
def export_report(
    session_dir: Path = typer.Argument(..., exists=True, file_okay=False, dir_okay=True),
    out: Optional[Path] = typer.Option(None, "--out", help="Text report output path."),
) -> None:
    """Write a human-readable health, stream, and event summary for a session."""
    target = out or session_dir / "session_summary.txt"
    try:
        summary = write_session_summary(SessionReplay(session_dir), target)
    except (OSError, ValueError) as exc:
        _err(f"Report export failed: {exc}")
        raise typer.Exit(code=1)
    typer.secho(
        f"wrote report for {summary['session_id']} -> {target}", fg=typer.colors.GREEN
    )


@app.command("hil-decode")
def hil_decode(
    session_dir: Path = typer.Argument(..., exists=True, file_okay=False, dir_okay=True),
    out: Optional[Path] = typer.Option(None, "--out", help="Decoded JSON artifact path."),
    session_id: Optional[int] = typer.Option(
        None, "--session-id", help="Require this nonzero HIL observer session ID."
    ),
) -> None:
    """Decode retained raw HIL trace events into an offline parity artifact."""
    try:
        replay = SessionReplay(session_dir)
        trace = hil.decode_trace_frames(
            (frame for _, frame in replay.iter_raw_frames()),
            expected_session_id=session_id,
        )
    except (OSError, ValueError, hil.TraceDecodeError) as exc:
        _err(f"Could not decode HIL trace: {exc}")
        raise typer.Exit(code=1)

    output = out or session_dir / "hil_trace.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(hil.trace_to_artifact(trace), indent=2) + "\n",
        encoding="utf-8",
    )
    typer.secho(
        f"decoded {len(trace.steps)} controller step(s) -> {output}",
        fg=typer.colors.GREEN,
    )
    if trace.end.reason is not hil.CaptureEndReason.COMPLETE:
        _err(
            f"Trace ended with {trace.end.reason.name}; artifact is diagnostic only "
            "and cannot be used for controller parity."
        )
        raise typer.Exit(code=1)


@app.command("hil-capture")
def hil_capture(
    steps: int = typer.Option(1, "--steps", min=1, max=32),
    out: Path = typer.Option(Path("data/sessions"), "--out"),
    name: str = typer.Option("hil", "--name"),
    timeout: float = typer.Option(5.0, "--timeout", min=1.0),
    port: Optional[str] = typer.Option(None, "--port", "-p"),
    baud: int = typer.Option(115200, "--baud"),
) -> None:
    """Capture output-disabled HIL trace records into a lossless session."""
    client = _connect(port, baud)
    logger: Optional[SessionLogger] = None
    maintenance_token = 0
    session_token = 0
    result_error: Optional[str] = None
    completed = None

    try:
        capabilities = _retry_request(client, "GET_CAPABILITIES", client.get_capabilities)
        if not capabilities.hil_output_disabled:
            raise RuntimeError("firmware image is not output-disabled HIL")
        observer_capability = _retry_request(
            client, "HIL_GET_CAPABILITY", client.hil_get_capability
        )
        if (
            not observer_capability.ok
            or not observer_capability.available
            or not observer_capability.output_disabled
        ):
            raise RuntimeError("HIL observer capability is unavailable")
        if observer_capability.trace_schema_version != hil.TRACE_SCHEMA_VERSION:
            raise RuntimeError(
                "firmware HIL trace schema does not match this host decoder"
            )
        status = _retry_request(client, "GET_STATUS", client.get_status)
        if not status.hil_output_disabled or status.dxl_power:
            raise RuntimeError("DXL power is enabled; refusing HIL capture")

        maintenance = _retry_request(
            client, "ENTER_MAINTENANCE", client.enter_maintenance
        )
        if not maintenance.ok or maintenance.token == 0:
            raise RuntimeError(f"maintenance lock rejected: result={maintenance.result}")
        maintenance_token = maintenance.token

        opened = _retry_request(
            client,
            "HIL_OPEN_SESSION",
            lambda: client.hil_open_session(maintenance_token),
        )
        if not opened.ok or opened.session_token == 0:
            raise RuntimeError(f"HIL observer session rejected: result={opened.result}")
        session_token = opened.session_token

        logger = SessionLogger(
            out_dir=out,
            robot_name=name,
            firmware={
                "device": capabilities.device_name,
                "protocol": f"{capabilities.proto_major}.{capabilities.proto_minor}",
                "hil_session_id": opened.session_id,
                "trace_schema_version": opened.trace_schema_version,
            },
        )
        client.on_raw_frame(logger.log_raw_frame)
        logger.mark_event(
            "hil_session_opened",
            session_id=opened.session_id,
            trace_schema_version=opened.trace_schema_version,
            requested_steps=steps,
        )
        assembler = hil.TraceAssembler(expected_session_id=opened.session_id)
        capture_done = threading.Event()
        decode_error: list[str] = []

        def on_event(event_id: int, payload: bytes, _header) -> None:
            try:
                parsed = assembler.accept_event(event_id, payload)
                if parsed is not None:
                    logger.log_record("hil_trace", parsed)
                    if isinstance(parsed, hil.TraceEnd):
                        capture_done.set()
            except hil.TraceDecodeError as exc:
                decode_error.append(str(exc))
                capture_done.set()

        client.on_event(on_event)
        try:
            capture = _retry_request(
                client,
                "HIL_CAPTURE",
                lambda: client.hil_capture(session_token, steps),
            )
            if not capture.ok or capture.capture_id == 0:
                raise RuntimeError(f"HIL capture rejected: result={capture.result}")
            logger.mark_event(
                "hil_capture_requested",
                capture_id=capture.capture_id,
                requested_steps=steps,
            )

            deadline = time.monotonic() + timeout
            next_heartbeat = time.monotonic() + 0.2
            while not capture_done.wait(timeout=0.02):
                now = time.monotonic()
                if now >= deadline:
                    raise RuntimeError("timed out waiting for HIL TraceEnd")
                if now >= next_heartbeat:
                    heartbeat = client.hil_heartbeat(session_token)
                    if heartbeat is None or not heartbeat.ok:
                        raise RuntimeError("HIL observer heartbeat failed")
                    maintenance_heartbeat = client.maint_heartbeat(maintenance_token)
                    if maintenance_heartbeat is None or not maintenance_heartbeat.ok:
                        raise RuntimeError("maintenance heartbeat failed")
                    next_heartbeat = now + 0.2
            if decode_error:
                raise RuntimeError(f"HIL trace decode failed: {decode_error[0]}")
            completed = assembler.finalize()
            if completed.end.reason is not hil.CaptureEndReason.COMPLETE:
                raise RuntimeError(
                    f"HIL capture ended {completed.end.reason.name}; retained session is diagnostic only"
                )
            logger.mark_event(
                "hil_capture_complete",
                capture_id=capture.capture_id,
                recorded_steps=len(completed.steps),
                emitted_fragment_count=completed.end.emitted_fragment_count,
            )
        finally:
            client.remove_event(on_event)
    except Exception as exc:  # noqa: BLE001
        result_error = str(exc)
        if logger is not None:
            logger.mark_event("hil_capture_error", result_error)
    finally:
        if session_token:
            client.hil_close_session(session_token)
        if maintenance_token:
            client.exit_maintenance(maintenance_token)
        if logger is not None:
            logger.close()
        client.stop()

    if result_error is not None:
        _err(result_error)
        raise typer.Exit(code=1)
    assert logger is not None and completed is not None
    artifact = logger.dir / "hil_trace.json"
    artifact.write_text(
        json.dumps(hil.trace_to_artifact(completed), indent=2) + "\n",
        encoding="utf-8",
    )
    typer.secho(
        f"captured {len(completed.steps)} controller step(s) -> {logger.dir}",
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


# --- rc / ExpressLRS link diagnostics -------------------------------------

rc_app = typer.Typer(help="RC / ExpressLRS link diagnostics (read-only).")
app.add_typer(rc_app, name="rc")

# CRSF mid tick (992 ~ 1500 us). CH12-16 are intentionally parked here by the
# ChannelPack, so they are excluded from the "frozen channel" check.
_RC_CRSF_MID = 992
# A channel whose tick range stays within this many ticks over the whole
# capture is treated as "not moving" (i.e. not being transmitted / stuck).
_RC_FROZEN_SPAN = 4

# ChannelPack wire layout (1-based CH numbers). Mirrors ChannelPack.h so the
# raw-tick view is self-describing.
_RC_CHANNEL_LABELS = [
    "LX gimbal",
    "LY gimbal",
    "RX gimbal",
    "RY gimbal",
    "Pot1 + SW_A (arm)",
    "Pot2",
    "Encoder1",
    "Encoder2",
    "SW_B/C/D/G/H",
    "Buttons + SW_E/F",
    "NAV1 + NAV2",
    "reserved",
    "reserved",
    "reserved",
    "reserved",
    "reserved",
]
# CH1-11 (0-based 0..10) carry live controller payload; CH12-16 are parked.
_RC_PAYLOAD_CHANNELS = list(range(11))
# CH9-11 only arrive over the air in a 16-channel switch mode. In "8ch" mode
# the ES24TX drops them, so they freeze at their last/centre value.
_RC_UPPER_CHANNELS = (8, 9, 10)


def _rc_switch_summary(ctrl) -> str:
    """One-line decoded switch/button/nav view from controller_state.raw."""
    raw = ctrl.raw
    sw_names = ["A", "B", "C", "D", "G", "H"]
    on = [n for n, v in zip(sw_names, raw.switches[:6]) if v]
    btns = [str(i + 1) for i, v in enumerate(raw.buttons) if v]
    tog = f"E={raw.toggles[0]} F={raw.toggles[1]}"
    return (
        f"switches[{','.join(on) if on else '-'}] "
        f"buttons[{','.join(btns) if btns else '-'}] {tog}"
    )


@rc_app.command("channels")
def rc_channels(
    seconds: float = typer.Option(
        3.0, "--seconds", "-s", help="Capture window; toggle every control during it."
    ),
    rate: int = typer.Option(20, "--rate", "-r", help="Requested telemetry rate (Hz)."),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Stream raw CRSF ticks + decoded switches to find dead/clamped channels.

    Toggle every switch, button, and stick on the controller during the capture
    window. Any payload channel (CH1-11) that never moves is not reaching the
    robot. The usual cause is the ES24TX being in "8ch" switch mode, which only
    transmits CH1-8 and freezes CH9-16 (SW_B/C/D/G/H, buttons, toggles, nav).
    Fix: set Switch Mode to "16ch Rate/2" (use the "Full Res" variant when
    the transmitter offers it) and keep Packet Rate at "100Hz Full".
    """
    client = _connect(port, baud)
    latest: dict[str, object] = {"diag": None, "ctrl": None}
    seen = {"diag": 0, "ctrl": 0}
    mins: list[Optional[int]] = [None] * 16
    maxs: list[Optional[int]] = [None] * 16
    diag_id = int(tlm.StreamId.RC_DIAGNOSTICS)
    ctrl_id = int(tlm.StreamId.CONTROLLER_STATE)

    def on_tel(stream_id: int, record: object, header) -> None:
        if stream_id == diag_id:
            latest["diag"] = record
            seen["diag"] += 1
            for i, t in enumerate(record.raw_ticks[:16]):
                mins[i] = t if mins[i] is None else min(mins[i], t)
                maxs[i] = t if maxs[i] is None else max(maxs[i], t)
        elif stream_id == ctrl_id:
            latest["ctrl"] = record
            seen["ctrl"] += 1

    client.on_telemetry(on_tel)
    try:
        for sid in (diag_id, ctrl_id):
            res = client.subscribe(sid, rate)
            if not (res and res.ok):
                _err(f"subscribe failed for {tlm.STREAM_NAMES[tlm.StreamId(sid)]}: {res}")
        typer.secho(
            f"Capturing {seconds:g}s — move every stick/switch now...",
            fg=typer.colors.BLUE,
        )
        time.sleep(seconds)
    finally:
        for sid in (diag_id, ctrl_id):
            client.unsubscribe(sid)
        client.stop()

    diag = latest["diag"]
    if diag is None or seen["diag"] == 0:
        _err(
            "No rc_diagnostics frames received. Check the USB link and that the "
            "firmware publishes stream 11 (rc_diagnostics)."
        )
        raise typer.Exit(code=1)

    if diag.failsafe or not diag.ever_seen:
        typer.secho(
            "RC link is in failsafe (no valid CRSF frames). Power on the "
            "controller and check the ELRS bind before reading channels.",
            fg=typer.colors.YELLOW,
        )

    typer.echo(f"raw_ticks over {seconds:g}s (frames={seen['diag']}):")
    typer.echo("  ch   tick    span  moved  meaning")
    frozen_payload: list[int] = []
    for i in range(16):
        lo, hi = mins[i], maxs[i]
        tick = diag.raw_ticks[i] if i < len(diag.raw_ticks) else 0
        span = (hi - lo) if lo is not None and hi is not None else 0
        moved = span > _RC_FROZEN_SPAN
        if i in _RC_PAYLOAD_CHANNELS and not moved:
            frozen_payload.append(i)
        mark = "yes" if moved else "  -"
        color = None if moved or i not in _RC_PAYLOAD_CHANNELS else typer.colors.YELLOW
        typer.secho(
            f"  CH{i + 1:<2} {tick:>5}  {span:>6}   {mark}   {_RC_CHANNEL_LABELS[i]}",
            fg=color,
        )

    ctrl = latest["ctrl"]
    if ctrl is not None:
        typer.echo(f"decoded: {_rc_switch_summary(ctrl)}")

    upper_frozen = [i for i in frozen_payload if i in _RC_UPPER_CHANNELS]
    lower_moved = any(
        i not in _RC_UPPER_CHANNELS
        and mins[i] is not None
        and (maxs[i] - mins[i]) > _RC_FROZEN_SPAN
        for i in _RC_PAYLOAD_CHANNELS
    )
    if len(upper_frozen) == len(_RC_UPPER_CHANNELS) and lower_moved:
        typer.secho(
            "\nDiagnosis: CH9-11 are frozen while lower channels move — the TX is "
            "dropping the upper channels. Set the ES24TX Switch Mode to "
            '"16ch Rate/2" (or "16ch Rate/2 Full Res" when available) with '
            'Packet Rate "100Hz Full".',
            fg=typer.colors.RED,
        )
    elif frozen_payload:
        names = ", ".join(f"CH{i + 1}" for i in frozen_payload)
        typer.secho(
            f"\n{names} did not move during capture — either not toggled, or not "
            "being transmitted. Re-run and exercise those inputs to confirm.",
            fg=typer.colors.YELLOW,
        )
    else:
        typer.secho(
            "\nAll payload channels (CH1-11) moved — the link is streaming every "
            "channel.",
            fg=typer.colors.GREEN,
        )


@rc_app.command("link")
def rc_link(
    seconds: float = typer.Option(2.0, "--seconds", "-s"),
    rate: int = typer.Option(10, "--rate", "-r"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Print CRSF link statistics + frame health (RSSI, LQ, SNR, CRC errors)."""
    client = _connect(port, baud)
    latest: dict[str, object] = {"diag": None}
    diag_id = int(tlm.StreamId.RC_DIAGNOSTICS)

    def on_tel(stream_id: int, record: object, header) -> None:
        if stream_id == diag_id:
            latest["diag"] = record

    client.on_telemetry(on_tel)
    try:
        res = client.subscribe(diag_id, rate)
        if not (res and res.ok):
            _err(f"subscribe failed for rc_diagnostics: {res}")
            raise typer.Exit(code=1)
        time.sleep(seconds)
    finally:
        client.unsubscribe(diag_id)
        client.stop()

    diag = latest["diag"]
    if diag is None:
        _err("No rc_diagnostics frames received.")
        raise typer.Exit(code=1)

    age = "never" if diag.last_frame_age_ms == 0xFFFF else f"{diag.last_frame_age_ms} ms"
    typer.echo(
        f"ever_seen={diag.ever_seen} failsafe={diag.failsafe} "
        f"last_frame_age={age}"
    )
    typer.echo(
        f"frames_decoded={diag.frames_decoded} crc_errors={diag.crc_errors} "
        f"link_stats_count={diag.link_stats_count}"
    )
    if diag.link_stats_valid:
        ls = diag.link_stats
        typer.echo(
            f"uplink   rssi={ls.up_rssi_dbm} dBm lq={ls.up_link_quality}% "
            f"snr={ls.up_snr} dB ant={ls.active_antenna} rf_mode={ls.rf_mode} "
            f"tx_power_idx={ls.up_tx_power}"
        )
        typer.echo(
            f"downlink rssi={ls.down_rssi_dbm} dBm lq={ls.down_link_quality}% "
            f"snr={ls.down_snr} dB"
        )
    else:
        typer.secho("no LINK_STATISTICS frames yet.", fg=typer.colors.YELLOW)


def main() -> None:
    app()


if __name__ == "__main__":
    sys.exit(app())
