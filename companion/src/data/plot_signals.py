"""Plottable-signal registry and replay extraction (nxi.1).

Pure, UI-independent helpers shared by the Plot Workbench page. A
:class:`PlotSignal` names one scalar time-series that can be pulled out of a
decoded telemetry record; the registry enumerates every field the workbench can
plot from the servo, leg, control, RC, and sensor streams. ``extract_series``
turns a recorded session (any iterable of decoded frames) into per-signal
``(xs, ys)`` arrays for replay plotting -- no Qt, no serial port.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Optional

from hexapod_protocol import telemetry as tlm

# Number of DYNAMIXEL servos on the hexapod (3 joints x 6 legs).
NUM_SERVOS = 18


@dataclass(frozen=True)
class PlotSignal:
    """One selectable scalar time-series.

    ``extract`` returns the value for a decoded record of the matching stream, or
    ``None`` when the record does not carry it (e.g. a servo id not present).
    """

    key: str  # unique id, e.g. "servo.1.position"
    group: str  # picker group / stream family, e.g. "Servo"
    label: str  # human label, e.g. "Servo #1 position"
    stream_id: int  # tlm.StreamId value the signal is decoded from
    unit: str
    extract: Callable[[object], Optional[float]]


# --- extractor factories ---------------------------------------------------


def _attr(name: str, scale: float = 1.0) -> Callable[[object], Optional[float]]:
    def f(rec: object) -> Optional[float]:
        v = getattr(rec, name, None)
        return None if v is None else float(v) * scale

    return f


def _bool_attr(name: str) -> Callable[[object], Optional[float]]:
    def f(rec: object) -> Optional[float]:
        v = getattr(rec, name, None)
        return None if v is None else (1.0 if v else 0.0)

    return f


def _servo_field(servo_id: int, attr: str) -> Callable[[object], Optional[float]]:
    def f(rec: object) -> Optional[float]:
        for s in getattr(rec, "servos", []):
            if s.id == servo_id:
                return float(getattr(s, attr))
        return None

    return f


def _list_field(list_attr: str, index: int, attr: str, is_bool: bool = False):
    def f(rec: object) -> Optional[float]:
        items = getattr(rec, list_attr, [])
        if index >= len(items):
            return None
        v = getattr(items[index], attr)
        return (1.0 if v else 0.0) if is_bool else float(v)

    return f


def _channel(index: int) -> Callable[[object], Optional[float]]:
    def f(rec: object) -> Optional[float]:
        ch = getattr(rec, "channels_us", [])
        return float(ch[index]) if index < len(ch) else None

    return f


# --- registry --------------------------------------------------------------


def build_signal_registry() -> list[PlotSignal]:
    """Return every plottable signal, grouped by stream family."""
    sigs: list[PlotSignal] = []
    H = int(tlm.StreamId.HEALTH)
    C = int(tlm.StreamId.CONTROL_STATE)
    S = int(tlm.StreamId.SERVO_STATUS)
    CT = int(tlm.StreamId.CONTACT_STATE)
    I2 = int(tlm.StreamId.I2C_SENSORS_RAW)
    R = int(tlm.StreamId.RC_INPUT)
    L = int(tlm.StreamId.LEG_STATE)

    # Health.
    sigs.append(PlotSignal("health.battery_mv", "Health", "Battery", H, "mV",
                           _attr("battery_mv")))
    sigs.append(PlotSignal("health.watchdog_missed", "Health", "Watchdog missed",
                           H, "count", _attr("watchdog_missed")))

    # Control.
    sigs.append(PlotSignal("control.command_source", "Control", "Command source",
                           C, "id", _attr("command_source")))
    sigs.append(PlotSignal("control.motion_gate", "Control", "Motion gate",
                           C, "0/1", _bool_attr("motion_gate")))
    sigs.append(PlotSignal("control.motion_authorized", "Control",
                           "Motion authorized", C, "0/1",
                           _bool_attr("motion_authorized")))
    sigs.append(PlotSignal("control.kill", "Control", "Kill active", C, "0/1",
                           _bool_attr("kill_active")))

    # Servo (per id): measured position and temperature.
    for sid in range(1, NUM_SERVOS + 1):
        sigs.append(PlotSignal(f"servo.{sid}.position", "Servo",
                               f"Servo #{sid} position", S, "ticks",
                               _servo_field(sid, "position")))
        sigs.append(PlotSignal(f"servo.{sid}.temperature_c", "Servo",
                               f"Servo #{sid} temperature", S, "\u00b0C",
                               _servo_field(sid, "temperature_c")))

    # Sensor: per-foot fused contact + raw proximity/pressure.
    for n in range(tlm.NUM_FEET):
        sigs.append(PlotSignal(f"contact.{n}.confidence", "Sensor",
                               f"Foot {n} confidence", CT, "0-255",
                               _list_field("feet", n, "confidence")))
        sigs.append(PlotSignal(f"contact.{n}.state", "Sensor",
                               f"Foot {n} contact state", CT, "enum",
                               _list_field("feet", n, "state")))
        sigs.append(PlotSignal(f"i2c.{n}.proximity", "Sensor",
                               f"Foot {n} proximity", I2, "raw",
                               _list_field("feet", n, "proximity")))
        sigs.append(PlotSignal(f"i2c.{n}.pressure", "Sensor",
                               f"Foot {n} pressure", I2, "raw",
                               _list_field("feet", n, "pressure_raw")))

    # RC input: gait selector, arm, and the first proportional channels.
    sigs.append(PlotSignal("rc.gait_index", "RC", "Gait index", R, "id",
                           _attr("gait_index")))
    sigs.append(PlotSignal("rc.armed", "RC", "Armed", R, "0/1",
                           _bool_attr("armed")))
    for ch in range(4):
        sigs.append(PlotSignal(f"rc.ch{ch + 1}", "RC", f"Channel {ch + 1}", R,
                               "\u00b5s", _channel(ch)))

    # Leg targets: commanded foot height + reachability.
    for n in range(tlm.NUM_FEET):
        sigs.append(PlotSignal(f"leg.{n}.foot_z_mm", "Leg", f"Leg {n} foot Z", L,
                               "mm", _list_field("legs", n, "foot_z_mm")))
        sigs.append(PlotSignal(f"leg.{n}.reachable", "Leg", f"Leg {n} reachable",
                               L, "0/1",
                               _list_field("legs", n, "reachable", is_bool=True)))

    return sigs


def registry_by_key(registry: Iterable[PlotSignal]) -> dict[str, PlotSignal]:
    return {s.key: s for s in registry}


def streams_for(signals: Iterable[PlotSignal]) -> set[int]:
    """The set of stream ids that must be subscribed to feed ``signals``."""
    return {s.stream_id for s in signals}


# --- replay extraction -----------------------------------------------------


def extract_series(
    frames: Iterable,
    signals: Iterable[PlotSignal],
    t0_ns: Optional[int] = None,
) -> dict[str, tuple[list[float], list[float]]]:
    """Pull per-signal ``(xs, ys)`` series from decoded replay frames.

    ``frames`` is any iterable of objects exposing ``stream`` (name), ``record``,
    and ``timestamp_ms`` -- i.e. ``SessionReplay.iter_decoded_frames()``. When
    ``t0_ns`` is given, the x axis is host wall-clock seconds relative to that
    reference (``host_time_ns`` per frame), so telemetry aligns with event
    markers on the same clock; otherwise x is the frame's robot timestamp in
    seconds. Frames whose stream/record do not match a signal are skipped.
    """
    sigs = list(signals)
    by_stream: dict[int, list[PlotSignal]] = {}
    for s in sigs:
        by_stream.setdefault(s.stream_id, []).append(s)
    out: dict[str, tuple[list[float], list[float]]] = {
        s.key: ([], []) for s in sigs
    }
    for df in frames:
        record = getattr(df, "record", None)
        stream_name = getattr(df, "stream", None)
        if record is None or stream_name is None:
            continue
        try:
            sid = int(tlm.stream_id_from_name(stream_name))
        except ValueError:
            continue
        matches = by_stream.get(sid)
        if not matches:
            continue
        if t0_ns is not None:
            x = (getattr(df, "host_time_ns", 0) - t0_ns) / 1e9
        else:
            x = getattr(df, "timestamp_ms", 0) / 1000.0
        for s in matches:
            y = s.extract(record)
            if y is None:
                continue
            xs, ys = out[s.key]
            xs.append(x)
            ys.append(y)
    return out
