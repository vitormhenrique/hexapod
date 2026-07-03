"""``hexapod-hil`` — hardware-in-loop test harness for the companion features.

One subcommand per companion UI page (hexapod_src-2e8). Each command connects
to the real robot over USB, exercises the same protocol calls the page makes,
and prints PASS/FAIL per check. No Qt required — this reuses the transport,
protocol client, data-logging, and URDF layers directly, so a green run means
the page's backend contract is verified against real firmware.

Usage:
    uv run hexapod-hil connect
    uv run hexapod-hil overview --seconds 5
    uv run hexapod-hil all --port /dev/cu.usbmodem2101

Exit code 0 = all checks passed, 1 = at least one FAIL, 2 = setup error.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import typer

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm

from transport import list_serial_ports, open_serial
from transport.protocol_client import ProtocolClient

hil = typer.Typer(
    add_completion=False,
    help="Hardware-in-loop page-by-page tests against the real robot.",
)

_PORT = typer.Option(None, "--port", "-p", help="Serial port (auto-detect if omitted).")
_BAUD = typer.Option(115200, "--baud")

EXPECTED_DEVICE_NAME = "OpenRB150-Hex"
BATTERY_MIN_MV = 6000
BATTERY_MAX_MV = 20000


# ---------------------------------------------------------------------------
# Check runner
# ---------------------------------------------------------------------------


@dataclass
class Check:
    name: str
    status: str  # PASS | FAIL | SKIP
    detail: str


class Runner:
    """Collects per-page check results and renders a summary."""

    def __init__(self, page: str) -> None:
        self.page = page
        self.checks: list[Check] = []
        typer.secho(f"\n=== {page} ===", fg=typer.colors.CYAN, bold=True)

    def _add(self, name: str, status: str, detail: str) -> None:
        self.checks.append(Check(name, status, detail))
        color = {
            "PASS": typer.colors.GREEN,
            "FAIL": typer.colors.RED,
            "SKIP": typer.colors.YELLOW,
        }[status]
        typer.secho(f"  [{status}] {name}: {detail}", fg=color)

    def ok(self, name: str, detail: str = "") -> None:
        self._add(name, "PASS", detail)

    def fail(self, name: str, detail: str = "") -> None:
        self._add(name, "FAIL", detail)

    def skip(self, name: str, detail: str = "") -> None:
        self._add(name, "SKIP", detail)

    def check(self, name: str, cond: bool, detail: str = "") -> bool:
        self._add(name, "PASS" if cond else "FAIL", detail)
        return cond

    def info(self, msg: str) -> None:
        typer.secho(f"  ....  {msg}", fg=typer.colors.BLUE)

    @property
    def failed(self) -> int:
        return sum(1 for c in self.checks if c.status == "FAIL")

    def summary(self) -> bool:
        """Print the page summary; returns True when nothing failed."""
        n_pass = sum(1 for c in self.checks if c.status == "PASS")
        n_skip = sum(1 for c in self.checks if c.status == "SKIP")
        color = typer.colors.GREEN if self.failed == 0 else typer.colors.RED
        typer.secho(
            f"  -- {self.page}: {n_pass} passed, {self.failed} failed, "
            f"{n_skip} skipped",
            fg=color,
            bold=True,
        )
        return self.failed == 0

    def finish(self) -> None:
        """Summarize and exit with the page verdict (single-page mode)."""
        ok = self.summary()
        raise typer.Exit(code=0 if ok else 1)


# ---------------------------------------------------------------------------
# Connection + telemetry helpers
# ---------------------------------------------------------------------------


def _pick_port(port: Optional[str]) -> str:
    if port:
        return port
    ports = list_serial_ports()

    def score(p) -> int:
        hay = f"{p.device} {p.description} {p.hwid}".lower()
        if "bluetooth" in hay:
            return -100
        s = 0
        if "openrb" in hay:
            s += 100
        if "usbmodem" in hay or "acm" in hay:
            s += 50
        if str(p.device).startswith("/dev/cu."):
            s += 20
        return s

    ranked = sorted(ports, key=score, reverse=True)
    if not ranked or score(ranked[0]) <= 0:
        typer.secho("No candidate serial port found; pass --port.", fg=typer.colors.RED)
        raise typer.Exit(code=2)
    return ranked[0].device


def _connect(port: Optional[str], baud: int) -> ProtocolClient:
    device = _pick_port(port)
    link = open_serial(device, baud=baud)
    if link is None:
        typer.secho(f"Could not open {device}.", fg=typer.colors.RED)
        raise typer.Exit(code=2)
    client = ProtocolClient(link)
    client.start()
    return client


def _retry(call: Callable, attempts: int = 3, delay: float = 0.5):
    """Call ``call`` until it returns non-None or attempts are exhausted."""
    result = None
    for i in range(attempts):
        result = call()
        if result is not None:
            return result
        if i < attempts - 1:
            time.sleep(delay)
    return result


@dataclass
class Collector:
    """Buffers decoded telemetry records per stream name (thread-safe)."""

    records: dict = field(default_factory=dict)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def callback(self, stream_id: int, record: object, header) -> None:
        try:
            name = tlm.STREAM_NAMES.get(tlm.StreamId(stream_id), str(stream_id))
        except ValueError:
            name = str(stream_id)
        with self._lock:
            self.records.setdefault(name, []).append((header.timestamp_ms, record))

    def count(self, name: str) -> int:
        with self._lock:
            return len(self.records.get(name, []))

    def get(self, name: str) -> list:
        with self._lock:
            return list(self.records.get(name, []))

    def clear(self) -> None:
        with self._lock:
            self.records.clear()


def _collect(
    client: ProtocolClient,
    r: Runner,
    streams: dict[str, int],
    seconds: float,
) -> Collector:
    """Subscribe to ``streams`` ({name: rate_hz}), collect for ``seconds``,
    unsubscribe, and return the collector. Records subscribe verdicts."""
    col = Collector()
    client.on_telemetry(col.callback)
    sids = []
    for name, rate in streams.items():
        sid = int(tlm.stream_id_from_name(name))
        res = client.subscribe(sid, rate)
        if res is not None and res.ok:
            sids.append(sid)
            r.ok(f"subscribe {name}", f"effective {res.effective_rate_hz} Hz")
        else:
            r.fail(f"subscribe {name}", f"result={res}")
    time.sleep(seconds)
    for sid in sids:
        client.unsubscribe(sid)
    time.sleep(0.2)  # let in-flight frames drain
    return col


def _state_name(state: int) -> str:
    return tlm.SAFETY_STATE_NAMES.get(state, f"0x{state:02X}")


def _get_state(client: ProtocolClient) -> Optional[int]:
    st = _retry(client.get_status)
    return st.state if st else None


def _wait_state(
    client: ProtocolClient,
    predicate: Callable[[int], bool],
    timeout: float = 2.0,
) -> Optional[int]:
    """Poll GET_STATUS until the FSM state satisfies ``predicate``.

    Command responses echo the state *before* the control task ticks, so page
    tests must settle on the post-transition state rather than trusting the
    immediate reply. Returns the last observed state (matching or not).
    """
    deadline = time.monotonic() + timeout
    state = _get_state(client)
    while time.monotonic() < deadline:
        if state is not None and predicate(state):
            return state
        time.sleep(0.1)
        state = _get_state(client)
    return state


STATE_DISARMED = 2
STATE_MAC_MAINTENANCE = 8
STATE_PASSIVE = 9
STATE_ESTOP = 12


def _scan_servos(client: ProtocolClient, attempts: int = 4, delay: float = 1.0):
    """Scan IDs 1-30, retrying while the freshly powered servos boot.

    MX-28s take over a second after DXL power-on before they answer pings, so
    a single scan right after power-up regularly finds nothing (HIL 2e8).
    Returns the (possibly empty) servo list from the last scan.
    """
    servos = []
    for _ in range(attempts):
        time.sleep(delay)
        scan = client.dxl_scan(1, 30)
        servos = scan.servos() if scan and scan.done else []
        if servos:
            break
    return servos


class _MaintLock:
    """Holds the maintenance lock with a background MAINT_HEARTBEAT thread.

    The firmware lock TTL is 1 s without heartbeats; page tests routinely take
    longer (DXL power-up, scans, staged writes), so a bare ENTER would lapse
    mid-page and gated commands would start failing with BadToken/Rejected.
    """

    def __init__(self, client: ProtocolClient):
        self._client = client
        self.token = 0
        self.result: Optional[api.MaintResultMsg] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def acquire(self) -> bool:
        self.result = _retry(self._client.enter_maintenance)
        if self.result is None or not self.result.ok or self.result.token == 0:
            return False
        self.token = self.result.token
        self._thread = threading.Thread(target=self._beat, daemon=True)
        self._thread.start()
        return True

    def _beat(self) -> None:
        # 0.25 s beat against the 1 s firmware TTL: a single request lost to a
        # busy serial link (e.g. during a DXL scan burst) must not lapse the
        # lock -- three consecutive losses are needed before expiry.
        while not self._stop.wait(0.25):
            try:
                self._client.maint_heartbeat(self.token)
            except Exception:
                return

    def release(self) -> Optional[api.MaintResultMsg]:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.token == 0:
            return None
        try:
            return self._client.exit_maintenance(self.token)
        finally:
            self.token = 0


# ---------------------------------------------------------------------------
# Page: Connect / Setup
# ---------------------------------------------------------------------------


@hil.command()
def connect(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Connect page: discovery, handshake, capabilities, reconnect cycle."""
    r = Runner("Connect / Setup")

    ports = list_serial_ports()
    r.check("port discovery", len(ports) > 0, f"{len(ports)} port(s) found")

    client = _connect(port, baud)
    device = _pick_port(port)
    try:
        hello = _retry(client.hello)
        if hello is None:
            r.fail("HELLO", "no response")
            r.finish()
        r.check(
            "HELLO identity",
            hello.device_name == EXPECTED_DEVICE_NAME,
            f"{hello.device_name} fw {hello.fw_major}.{hello.fw_minor}."
            f"{hello.fw_patch} proto {hello.proto_major}.{hello.proto_minor}",
        )

        st = _retry(client.get_status)
        if st is None:
            r.fail("GET_STATUS", "no response")
        else:
            r.ok(
                "GET_STATUS",
                f"state={_state_name(st.state)} battery={st.battery_mv} mV "
                f"dxl_power={st.dxl_power}",
            )
            r.check(
                "battery plausible",
                BATTERY_MIN_MV < st.battery_mv < BATTERY_MAX_MV,
                f"{st.battery_mv} mV",
            )

        caps = _retry(client.get_capabilities)
        if caps is None:
            r.fail("GET_CAPABILITIES", "no response")
        else:
            avail = [
                api.FEATURE_NAMES[i] for i in api.capability_features(caps.feature_bits)
            ]
            r.ok(
                "GET_CAPABILITIES",
                f"feature_bits=0x{caps.feature_bits:08X} "
                f"available={', '.join(avail) or '(none)'}",
            )

        # Uptime advances across two status reads (scheduler alive).
        st1 = _retry(client.get_status)
        time.sleep(0.5)
        st2 = _retry(client.get_status)
        if st1 and st2:
            r.check(
                "uptime advances",
                st2.uptime_ms > st1.uptime_ms,
                f"{st1.uptime_ms} -> {st2.uptime_ms} ms",
            )
        else:
            r.fail("uptime advances", "status read failed")
    finally:
        client.stop()

    # Disconnect / reconnect cycle.
    client2 = _connect(device, baud)
    try:
        hello2 = _retry(client2.hello)
        r.check("reconnect HELLO", hello2 is not None, "second session handshake")
    finally:
        client2.stop()

    r.finish()


# ---------------------------------------------------------------------------
# Page: Overview
# ---------------------------------------------------------------------------


@hil.command()
def overview(
    seconds: float = typer.Option(4.0, "--seconds", "-s"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Overview page: live health/control_state updates, plausible values."""
    r = Runner("Overview")
    client = _connect(port, baud)
    try:
        col = _collect(
            client, r, {"health": 5, "control_state": 10, "leg_state": 10}, seconds
        )

        health = col.get("health")
        if r.check("health stream flows", len(health) >= 2, f"{len(health)} records"):
            uptimes = [rec.uptime_ms for _, rec in health]
            r.check(
                "uptime monotonic",
                all(b >= a for a, b in zip(uptimes, uptimes[1:])),
                f"{uptimes[0]} -> {uptimes[-1]} ms",
            )
            last = health[-1][1]
            r.check(
                "state known",
                last.state in tlm.SAFETY_STATE_NAMES,
                f"state={last.state_name} fault={last.fault_name}",
            )
            r.check(
                "battery plausible",
                BATTERY_MIN_MV < last.battery_mv < BATTERY_MAX_MV,
                f"{last.battery_mv} mV",
            )
            r.check(
                "no watchdog misses",
                last.watchdog_missed == 0,
                f"0x{last.watchdog_missed:X}",
            )

        cs = col.get("control_state")
        if r.check("control_state flows", len(cs) >= 2, f"{len(cs)} records"):
            last_cs = cs[-1][1]
            r.ok(
                "command source",
                f"source={last_cs.source_name} gate={last_cs.motion_gate} "
                f"kill={last_cs.kill_active}",
            )

        legs = col.get("leg_state")
        r.check(
            "leg_state flows (mini-map)",
            len(legs) >= 1,
            f"{len(legs)} records",
        )
    finally:
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Mode & Safety Center
# ---------------------------------------------------------------------------


@hil.command("mode-safety")
def mode_safety(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Mode & Safety page: estop + clear fault, maintenance lock, features."""
    r = Runner("Mode & Safety Center")
    client = _connect(port, baud)
    try:
        state0 = _get_state(client)
        r.info(f"initial state: {_state_name(state0) if state0 is not None else '?'}")

        # Estop -> clear fault round trip. The FSM transitions on the next
        # control tick, so settle on GET_STATUS rather than the reply echo.
        res = _retry(client.estop)
        if res is None:
            r.fail("ESTOP", "no response")
        else:
            state = _wait_state(client, lambda s: s == STATE_ESTOP)
            r.check(
                "ESTOP enters ESTOP state",
                state == STATE_ESTOP,
                f"state={_state_name(state) if state is not None else '?'}",
            )
        res = _retry(client.clear_fault)
        if res is None:
            r.fail("CLEAR_FAULT", "no response")
        else:
            state = _wait_state(client, lambda s: s != STATE_ESTOP)
            r.check(
                "clear fault recovers",
                state is not None and state != STATE_ESTOP,
                f"state={_state_name(state) if state is not None else '?'}",
            )

        # Maintenance lock: acquire, heartbeat, second-acquire busy, release.
        lock = _MaintLock(client)
        if not lock.acquire():
            r.fail(
                "maintenance lock acquire",
                f"result={getattr(lock.result, 'result', None)} "
                f"state={_state_name(getattr(lock.result, 'state', 255))}",
            )
        else:
            r.ok(
                "maintenance lock acquire",
                f"token=0x{lock.token:08X} state={_state_name(lock.result.state)}",
            )
            state = _wait_state(client, lambda s: s == STATE_MAC_MAINTENANCE)
            r.check(
                "FSM enters MAC_MAINTENANCE",
                state == STATE_MAC_MAINTENANCE,
                f"state={_state_name(state) if state is not None else '?'}",
            )
            again = client.enter_maintenance()
            r.check(
                "second acquire rejected/busy",
                again is None or not again.ok,
                f"result={getattr(again, 'result', None)}",
            )
            rel = lock.release()
            state = _wait_state(client, lambda s: s != STATE_MAC_MAINTENANCE)
            r.check(
                "maintenance lock release",
                rel is not None and state != STATE_MAC_MAINTENANCE,
                f"state={_state_name(state) if state is not None else '?'}",
            )

        # Feature toggles reflect firmware reject reasons.
        fl = _retry(client.feature_get)
        if fl is None:
            r.fail("FEATURE_GET", "no response")
        else:
            lines = [
                f"{api.FEATURE_NAMES[f.feature]}:a={int(f.available)},"
                f"e={int(f.enabled)},r={f.reason}"
                for f in fl.features
                if f.feature < len(api.FEATURE_NAMES)
            ]
            r.ok("FEATURE_GET", " ".join(lines))
            unavailable = [f for f in fl.features if not f.available]
            if unavailable:
                target = unavailable[0]
                res = client.feature_set(target.feature, True)
                name = api.FEATURE_NAMES[target.feature]
                r.check(
                    f"enable unavailable '{name}' rejected with reason",
                    res is not None and not res.ok and res.reason != 0,
                    f"ok={getattr(res, 'ok', None)} reason={getattr(res, 'reason', None)}",
                )
            else:
                r.skip("reject-reason check", "all features available")
    finally:
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Passive Pose
# ---------------------------------------------------------------------------


@hil.command("passive-pose")
def passive_pose(
    seconds: float = typer.Option(4.0, "--seconds", "-s"),
    interactive: bool = typer.Option(
        False, "--interactive", help="Prompt to move a joint by hand and verify."
    ),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Passive Pose page: enter/exit, torque off, joint positions stream."""
    r = Runner("Passive Pose")
    client = _connect(port, baud)
    lock = _MaintLock(client)
    powered = False
    try:
        # Bench flow (hexapod_src-nkb): DXL power + scan happen under the
        # maintenance lock, then passive entry hands off directly from
        # MAC_MAINTENANCE so the bus stays powered for present-position reads.
        if not lock.acquire():
            r.fail(
                "enter maintenance",
                f"result={getattr(lock.result, 'result', None)}",
            )
            r.finish()
        r.ok("enter maintenance", f"token=0x{lock.token:08X}")
        _wait_state(client, lambda s: s == STATE_MAC_MAINTENANCE)

        pw = client.dxl_power(True)
        pr = pw.power() if pw and pw.done else None
        if pr is None or not pr.power_on:
            r.fail("DXL power on", f"result={pw}")
            r.finish()
        powered = True
        r.ok("DXL power on", f"has_control={pr.has_control}")

        servos = _scan_servos(client)
        if not r.check(
            "DXL scan finds servos", len(servos) > 0, f"{len(servos)} found"
        ):
            r.finish()

        res = _retry(client.passive_enter)
        if res is None or not res.ok:
            r.fail(
                "passive enter",
                f"result={getattr(res, 'result', None)} "
                f"state={_state_name(getattr(res, 'state', 255))}",
            )
            r.finish()
        state = _wait_state(client, lambda s: s == STATE_PASSIVE)
        r.check(
            "passive enter -> PASSIVE_POSE_STREAM",
            state == STATE_PASSIVE,
            f"state={_state_name(state) if state is not None else '?'}",
        )

        col = _collect(client, r, {"joint_state": 20, "servo_status": 10}, seconds)

        joints = col.get("joint_state")
        if r.check("joint_state flows", len(joints) >= 2, f"{len(joints)} records"):
            last = joints[-1][1]
            r.check(
                "joints reported for all scanned servos",
                len(last.joints) >= len(servos),
                f"{len(last.joints)} joints / {len(servos)} scanned",
            )

        servo = col.get("servo_status")
        if servo:
            last_sv = servo[-1][1]
            torqued = [s.id for s in last_sv.servos if s.torque_enabled]
            r.check(
                "all torque off",
                not torqued,
                f"torque-on ids={torqued or 'none'} ({len(last_sv.servos)} servos)",
            )
        else:
            r.skip("torque-off check", "no servo_status records (DXL power off?)")

        if interactive and joints:
            before = {(j.leg, j.joint): j.angle_centideg for j in joints[-1][1].joints}
            typer.confirm(
                "  Move any leg by hand, then press enter",
                default=True,
                prompt_suffix=" ",
            )
            col2 = _collect(client, r, {"joint_state": 20}, 2.0)
            j2 = col2.get("joint_state")
            if j2:
                after = {(j.leg, j.joint): j.angle_centideg for j in j2[-1][1].joints}
                moved = [
                    k
                    for k in before
                    if k in after and abs(after[k] - before[k]) > 200  # >2 deg
                ]
                r.check(
                    "manual movement tracked",
                    len(moved) > 0,
                    f"{len(moved)} joint(s) moved >2 deg",
                )
            else:
                r.fail("manual movement tracked", "no joint_state after prompt")

        res = _retry(client.passive_exit)
        state = _wait_state(client, lambda s: s != STATE_PASSIVE)
        r.check(
            "passive exit",
            res is not None and res.ok and state != STATE_PASSIVE,
            f"state={_state_name(state) if state is not None else '?'}",
        )
    finally:
        if powered:
            # Back in maintenance (lock still heartbeating) -> power off there.
            _wait_state(client, lambda s: s == STATE_MAC_MAINTENANCE)
            off = client.dxl_power(False)
            opr = off.power() if off and off.done else None
            r.check(
                "DXL power off",
                opr is not None and not opr.power_on,
                f"power_on={getattr(opr, 'power_on', '?')}",
            )
        if lock.token:
            lock.release()
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Servo Map & DXL Tuning
# ---------------------------------------------------------------------------


@hil.command("servo-tuning")
def servo_tuning(
    limits_servo: Optional[int] = typer.Option(
        None, "--limits-servo", help="Servo id for the limit write test."
    ),
    skip_write: bool = typer.Option(
        False, "--skip-write", help="Skip the EEPROM limit write/read-back."
    ),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Servo tuning page: DXL power, scan, profile, staged limit write."""
    r = Runner("Servo Map & DXL Tuning")
    client = _connect(port, baud)
    lock = _MaintLock(client)
    powered = False
    try:
        if not lock.acquire():
            r.fail(
                "enter maintenance",
                f"result={getattr(lock.result, 'result', None)}",
            )
            r.finish()
        r.ok("enter maintenance", f"token=0x{lock.token:08X}")
        _wait_state(client, lambda s: s == STATE_MAC_MAINTENANCE)

        pw = client.dxl_power(True)
        pr = pw.power() if pw and pw.done else None
        if pr is None or not pr.power_on:
            r.fail("DXL power on", f"result={pw}")
            r.finish()
        powered = True
        r.ok("DXL power on", f"has_control={pr.has_control}")

        servos = _scan_servos(client)
        if not r.check(
            "DXL scan finds servos", len(servos) > 0, f"{len(servos)} found"
        ):
            r.finish()
        r.info(
            "ids: " + ", ".join(f"{s.id}(m{s.model},t{s.table_kind})" for s in servos)
        )

        # TX-gate regression: scanned servos must appear in servo_status.
        col = _collect(client, r, {"servo_status": 10}, 2.5)
        status_recs = col.get("servo_status")
        seen_ids = set()
        for _, rec in status_recs:
            seen_ids.update(s.id for s in rec.servos)
        scanned_ids = {s.id for s in servos}
        r.check(
            "scanned servos in servo_status stream",
            scanned_ids <= seen_ids,
            f"scanned={sorted(scanned_ids)} streamed={sorted(seen_ids)}",
        )

        target = limits_servo if limits_servo is not None else servos[0].id
        prof = client.dxl_get_servo_profile(target)
        r.check(
            "servo profile read",
            prof is not None and prof.done and len(prof.data) > 0,
            f"id={target} data={prof.data.hex() if prof else '?'}",
        )

        # Read current limits via logical params, then (optionally) write the
        # same values back and verify the read-back path.
        # TableKind wire values (dxl_model.h): 0=Unknown, 1=Mx28Legacy, 2=Mx28V2.
        legacy = servos[0].table_kind == 1
        lo_param = (
            api.DXL_PARAM_CW_ANGLE_LIMIT if legacy else api.DXL_PARAM_MIN_POSITION_LIMIT
        )
        hi_param = (
            api.DXL_PARAM_CCW_ANGLE_LIMIT
            if legacy
            else api.DXL_PARAM_MAX_POSITION_LIMIT
        )
        lo = client.dxl_get_param(target, lo_param)
        hi = client.dxl_get_param(target, hi_param)
        lo_v = lo.param() if lo and lo.done else None
        hi_v = hi.param() if hi and hi.done else None
        if not r.check(
            "limit params read",
            lo_v is not None and hi_v is not None,
            f"id={target} min={getattr(lo_v, 'value', '?')} "
            f"max={getattr(hi_v, 'value', '?')}",
        ):
            r.finish()

        if skip_write:
            r.skip("staged limit write", "--skip-write")
        else:
            res = client.dxl_set_servo_limits(target, lo_v.value, hi_v.value)
            sl = res.servo_limits() if res else None
            r.check(
                "limit write + read-back verified",
                sl is not None and sl.verified,
                f"id={target} [{getattr(sl, 'min_tick', '?')}, "
                f"{getattr(sl, 'max_tick', '?')}] table={getattr(sl, 'table_kind', '?')}",
            )
    finally:
        if powered:
            off = client.dxl_power(False)
            opr = off.power() if off and off.done else None
            r.check(
                "DXL power off",
                opr is not None and not opr.power_on,
                f"power_on={getattr(opr, 'power_on', '?')}",
            )
        if lock.token:
            rel = lock.release()
            r.check("exit maintenance", rel is not None and rel.ok, "")
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Leg Lab
# ---------------------------------------------------------------------------


@hil.command("leg-lab")
def leg_lab(
    leg: int = typer.Option(0, "--leg"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Leg Lab page: maintenance-gated foot target, clamp/reachability."""
    # Home-stance foot target in the body frame for each leg (HexNav defaults,
    # firmware config_schema.cpp kLegSeeds + gait home stance): the coxa-frame
    # home foot (127, 0, -44.55) mapped through hip_xy + Rz(mount_yaw + 90deg).
    # The old hardcoded (120, 0, -80) sat outside leg 0's (rear-left) workspace
    # and made the "reachable" check fail against real geometry.
    seeds = [  # (mount_x_mm, mount_y_mm, mount_z_mm, yaw_deg)
        (-65.6, -115.6, -16.5, 135.0),  # leg 1 rear-left
        (65.6, -115.6, -16.5, -135.0),  # leg 2 rear-right
        (69.8, 0.0, -16.5, -90.0),  # leg 3 mid-right
        (65.6, 115.6, -16.5, -45.0),  # leg 4 front-right
        (-65.6, 115.6, -16.5, 45.0),  # leg 5 front-left
        (-69.8, 0.0, -16.5, 90.0),  # leg 6 mid-left
    ]
    mx, my, mz, yaw = seeds[leg % 6]
    rad = math.radians(yaw + 90.0)
    home_x = int(round(mx + 127.0 * math.cos(rad)))
    home_y = int(round(my + 127.0 * math.sin(rad)))
    home_z = int(round(mz + 21.0 - 44.55))
    r = Runner("Leg Lab")
    client = _connect(port, baud)
    lock = _MaintLock(client)
    try:
        # Without the lock the command must be rejected.
        res = client.set_leg_target(leg, home_x, home_y, home_z)
        r.check(
            "leg target rejected without lock",
            res is None or res.result != 0,
            f"result={getattr(res, 'result', 'timeout')}",
        )

        if not lock.acquire():
            r.fail(
                "enter maintenance",
                f"result={getattr(lock.result, 'result', None)}",
            )
            r.finish()
        r.ok("enter maintenance", f"token=0x{lock.token:08X}")
        _wait_state(client, lambda s: s == STATE_MAC_MAINTENANCE)

        # Reachable target (home stance): IK verdict + ticks come back.
        res = client.set_leg_target(leg, home_x, home_y, home_z)
        if res is None:
            r.fail("reachable foot target", "no response")
        else:
            r.check(
                "reachable foot target accepted",
                res.result == 0 and res.reachable,
                f"reachable={res.reachable} clamps=({res.clamp_low:03b},"
                f"{res.clamp_high:03b}) ticks={res.ticks}",
            )

        # Absurd target: must be flagged unreachable/clamped, not crash.
        res = client.set_leg_target(leg, 500, 0, 200)
        if res is None:
            r.fail("unreachable target feedback", "no response")
        else:
            flagged = (not res.reachable) or res.clamp_low or res.clamp_high
            r.check(
                "unreachable target flagged",
                bool(flagged),
                f"reachable={res.reachable} clamps=({res.clamp_low:03b},"
                f"{res.clamp_high:03b})",
            )

        # Joint target path.
        jres = client.set_joint_target(leg, 1, 1500)  # femur +15 deg
        r.check(
            "joint target accepted",
            jres is not None and jres.result == 0,
            f"result={getattr(jres, 'result', 'timeout')}",
        )
    finally:
        if lock.token:
            rel = lock.release()
            r.check("exit maintenance", rel is not None and rel.ok, "")
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Gait Lab
# ---------------------------------------------------------------------------


@hil.command("gait-lab")
def gait_lab(
    expect_motion: bool = typer.Option(
        False,
        "--expect-motion",
        help="Robot is armed & suspended: expect commands to be accepted.",
    ),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Gait Lab page: command staging; firmware verdict must match state."""
    r = Runner("Gait Lab")
    client = _connect(port, baud)
    try:
        st = _retry(client.get_status)
        state = st.state if st else None
        r.info(f"state: {_state_name(state) if state is not None else '?'}")

        def verdict(name: str, res) -> None:
            if res is None:
                r.fail(name, "no response")
                return
            if expect_motion:
                r.check(
                    f"{name} accepted",
                    res.ok and res.motion_allowed,
                    f"ok={res.ok} motion_allowed={res.motion_allowed} "
                    f"state={_state_name(res.state)}",
                )
            else:
                # Bench-safe: firmware must answer and must NOT authorize motion.
                r.check(
                    f"{name} answered, motion gated",
                    not res.motion_allowed,
                    f"ok={res.ok} motion_allowed={res.motion_allowed} "
                    f"state={_state_name(res.state)}",
                )

        verdict("SET_GAIT stand", client.set_gait(api.GAIT_STAND))
        verdict(
            "SET_GAIT_PARAMS",
            client.set_gait_params(90, 40, 30, 128, 100),
        )
        verdict("SET_BODY_TWIST zero", client.set_body_twist(0.0, 0.0, 0.0))
        verdict("STOP_MOTION", client.stop_motion())
    finally:
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Sensor Dashboard / I2C Explorer
# ---------------------------------------------------------------------------


@hil.command("sensor-dashboard")
def sensor_dashboard(
    seconds: float = typer.Option(3.0, "--seconds", "-s"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Sensor page: I2C scan/topology, sensor status, raw stream."""
    r = Runner("Sensor Dashboard / I2C Explorer")
    client = _connect(port, baud)
    try:
        scan = _retry(client.i2c_scan)
        r.check(
            "I2C_SCAN",
            scan is not None and scan.ok,
            f"scan_seq={getattr(scan, 'scan_seq', '?')}",
        )

        topo = _retry(client.i2c_get_topology)
        if topo is None:
            r.fail("I2C_GET_TOPOLOGY", "no response")
        else:
            r.check("EEPROM 0x50 present", topo.eeprom_present, "")
            r.check("TCA mux 0x70 present", topo.mux_present, "")
            present = [i for i, ch in enumerate(topo.channels) if ch.state == 1]
            r.ok(
                "channel presence",
                f"{len(topo.channels)} channels, present={present or 'none'}",
            )

        st = _retry(client.sensor_get_status)
        if st is None:
            r.fail("SENSOR_GET_STATUS", "no response")
        else:
            r.ok(
                "SENSOR_GET_STATUS",
                f"present_mask=0x{st.present_mask:02X} "
                f"polling={st.polling_enabled} feet={len(st.feet)}",
            )

        col = _collect(client, r, {"i2c_sensors_raw": 10}, seconds)
        raw = col.get("i2c_sensors_raw")
        if st is not None and st.present_mask == 0:
            r.skip("live finger sensor values", "no sensors present")
        else:
            r.check(
                "live finger sensor values",
                len(raw) >= 1,
                f"{len(raw)} records",
            )
    finally:
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Foot Contact & Leveling
# ---------------------------------------------------------------------------


@hil.command("foot-contact")
def foot_contact(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Foot contact page: enable verdict must match sensor availability."""
    r = Runner("Foot Contact & Leveling")
    client = _connect(port, baud)
    try:
        sensors = _retry(client.sensor_get_status)
        have_sensors = sensors is not None and sensors.present_mask != 0
        r.info(
            f"sensors present_mask=" f"0x{sensors.present_mask:02X}"
            if sensors
            else "sensor status unknown"
        )

        res = _retry(lambda: client.contact_enable(True))
        if res is None:
            r.fail("CONTACT_ENABLE", "no response")
        elif have_sensors:
            r.check(
                "contact enable with sensors",
                res.ok and res.enabled,
                f"available={res.available} enabled={res.enabled} reason={res.reason}",
            )
        else:
            r.check(
                "contact enable rejected with clear reason",
                not res.enabled and res.reason != 0,
                f"available={res.available} enabled={res.enabled} reason={res.reason}",
            )

        res = _retry(lambda: client.contact_enable(False))
        r.check(
            "contact disable",
            res is not None and not res.enabled,
            f"enabled={getattr(res, 'enabled', '?')}",
        )

        res = _retry(lambda: client.leveling_enable(True))
        if res is None:
            r.fail("LEVELING_ENABLE", "no response")
        elif have_sensors:
            r.ok(
                "leveling enable verdict",
                f"available={res.available} enabled={res.enabled} reason={res.reason}",
            )
        else:
            r.check(
                "leveling enable rejected with clear reason",
                not res.enabled and res.reason != 0,
                f"available={res.available} enabled={res.enabled} reason={res.reason}",
            )
        res = _retry(lambda: client.leveling_enable(False))
        r.check(
            "leveling disable",
            res is not None and not res.enabled,
            f"enabled={getattr(res, 'enabled', '?')}",
        )
    finally:
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Plot Workbench (streaming soak)
# ---------------------------------------------------------------------------


@hil.command("plot-workbench")
def plot_workbench(
    seconds: float = typer.Option(60.0, "--seconds", "-s"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Plot page: sustained multi-stream soak, no decode errors/backlog."""
    r = Runner("Plot Workbench (soak)")
    client = _connect(port, baud)
    try:
        # Firmware drop/backlog counters are cumulative since boot, so take a
        # baseline before the soak and assert on the deltas afterwards.
        base_snap = client.diagnostics_snapshot()
        base_fw = client.get_stream_stats()
        base_dropped = (
            {s.stream_id: s.dropped for s in base_fw.streams} if base_fw else {}
        )

        r.info(f"soaking for {seconds:.0f} s ...")
        col = _collect(
            client,
            r,
            {"health": 5, "servo_status": 20, "api_stats": 2},
            seconds,
        )

        n_servo = col.count("servo_status")
        n_health = col.count("health")
        r.check(
            "servo_status volume",
            n_servo >= seconds * 20 * 0.5,
            f"{n_servo} records (>= 50% of nominal)",
        )
        r.check("health volume", n_health >= seconds * 5 * 0.5, f"{n_health} records")

        snap = client.diagnostics_snapshot()
        derr = snap.decode_errors - base_snap.decode_errors
        r.check(
            "no host decode errors during soak",
            derr == 0,
            f"decode_errors +{derr} rx={snap.rx_frames}",
        )

        stats = col.get("api_stats")
        if r.check("api_stats flows", len(stats) >= 2, f"{len(stats)} records"):
            first, last = stats[0][1], stats[-1][1]
            r.check(
                "tx backlog not growing",
                last.tx_backlog <= max(first.tx_backlog, 8),
                f"{first.tx_backlog} -> {last.tx_backlog}",
            )
            r.check(
                "no firmware rx corruption",
                last.rx_bad == 0 and last.rx_overflow == 0,
                f"rx_bad={last.rx_bad} rx_overflow={last.rx_overflow}",
            )

        fw_stats = client.get_stream_stats()
        if fw_stats is not None:
            dropped = {
                tlm.STREAM_NAMES.get(tlm.StreamId(s.stream_id), s.stream_id): delta
                for s in fw_stats.streams
                if (delta := s.dropped - base_dropped.get(s.stream_id, 0))
            }
            r.check(
                "no firmware stream drops during soak",
                not dropped,
                f"dropped={dropped or 'none'}",
            )
        else:
            r.fail("GET_STREAM_STATS", "no response")
    finally:
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Session Browser (record + replay)
# ---------------------------------------------------------------------------


@hil.command("session-browser")
def session_browser(
    seconds: float = typer.Option(5.0, "--seconds", "-s"),
    out: Path = typer.Option(Path("data/sessions"), "--out"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Session page: record a live session, then replay and verify it."""
    from data import SessionLogger, SessionReplay

    r = Runner("Session Browser")
    client = _connect(port, baud)
    logger = None
    try:
        hello = _retry(client.hello)
        fw = (
            {
                "device": hello.device_name,
                "fw": f"{hello.fw_major}.{hello.fw_minor}.{hello.fw_patch}",
            }
            if hello
            else None
        )
        logger = SessionLogger(out_dir=out, robot_name="hil", firmware=fw)
        r.info(f"recording -> {logger.dir}")

        def on_tel(stream_id: int, record: object, header) -> None:
            logger.log_record(
                tlm.STREAM_NAMES.get(tlm.StreamId(stream_id), str(stream_id)),
                record,
                robot_time_ms=header.timestamp_ms,
            )

        client.on_telemetry(on_tel)
        client.on_raw_frame(logger.log_raw_frame)
        for name, rate in {"health": 5, "servo_status": 10}.items():
            client.subscribe(int(tlm.stream_id_from_name(name)), rate)
        time.sleep(seconds)
        for name in ("health", "servo_status"):
            client.unsubscribe(int(tlm.stream_id_from_name(name)))
        client.remove_raw_frame(logger.log_raw_frame)
        logger.close()
        n_rec = logger._meta.record_count  # noqa: SLF001
        r.check("session recorded", n_rec > 0, f"{n_rec} records")

        replay = SessionReplay(logger.dir)
        meta = replay.meta
        r.check(
            "manifest readable",
            meta.get("record_count", 0) == n_rec,
            f"session_id={meta.get('session_id', '?')}",
        )
        rows = sum(1 for _ in replay.iter_records())
        r.check("telemetry rows replay", rows == n_rec, f"{rows} rows")
        decoded = sum(1 for _ in replay.iter_decoded_frames())
        r.check("raw frames re-decode", decoded > 0, f"{decoded} frames")
    finally:
        if logger is not None:
            logger.close()
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: URDF Viewer
# ---------------------------------------------------------------------------


@hil.command("urdf-viewer")
def urdf_viewer(
    seconds: float = typer.Option(3.0, "--seconds", "-s"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """URDF page: live passive joint telemetry drives forward kinematics."""
    from data.urdf_model import load_hexnav
    from data.urdf_fk import UrdfForwardKinematics, joint_state_to_urdf_angles

    r = Runner("URDF Viewer")
    try:
        model = load_hexnav()
        fk = UrdfForwardKinematics(model)
        r.ok("URDF model loads", f"{len(model.joints)} joints")
    except Exception as exc:  # noqa: BLE001
        r.fail("URDF model loads", str(exc))
        r.finish()

    client = _connect(port, baud)
    lock = _MaintLock(client)
    entered = False
    powered = False
    try:
        # Same bench flow as passive-pose: power + scan under the maintenance
        # lock so present positions exist, then hand off to passive streaming.
        if not lock.acquire():
            r.fail(
                "enter maintenance",
                f"result={getattr(lock.result, 'result', None)}",
            )
            r.finish()
        _wait_state(client, lambda s: s == STATE_MAC_MAINTENANCE)
        pw = client.dxl_power(True)
        pr = pw.power() if pw and pw.done else None
        if pr is None or not pr.power_on:
            r.fail("DXL power on", f"result={pw}")
            r.finish()
        powered = True
        servos = _scan_servos(client)
        if not r.check(
            "DXL scan finds servos", len(servos) > 0, f"{len(servos)} found"
        ):
            r.finish()

        res = _retry(client.passive_enter)
        entered = res is not None and res.ok
        state = _wait_state(client, lambda s: s == STATE_PASSIVE)
        r.check(
            "passive enter for live overlay",
            entered and state == STATE_PASSIVE,
            f"state={_state_name(state) if state is not None else '?'}",
        )

        col = _collect(client, r, {"joint_state": 20}, seconds)
        joints = col.get("joint_state")
        if not r.check("joint_state flows", len(joints) >= 1, f"{len(joints)} records"):
            r.finish()

        angles = joint_state_to_urdf_angles(joints[-1][1])
        r.check(
            "telemetry maps to URDF joints",
            len(angles) == 18,
            f"{len(angles)} mapped joints",
        )
        feet = fk.foot_positions(angles)
        r.check(
            "FK computes foot positions",
            len(feet) == 6,
            f"{len(feet)} feet",
        )
    finally:
        if entered:
            client.passive_exit()
        if powered:
            _wait_state(client, lambda s: s == STATE_MAC_MAINTENANCE)
            client.dxl_power(False)
        if lock.token:
            lock.release()
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: Diagnostics
# ---------------------------------------------------------------------------


@hil.command()
def diagnostics(
    seconds: float = typer.Option(5.0, "--seconds", "-s"),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Diagnostics page: frame stats, CRC/decode counters, stream stats."""
    r = Runner("Diagnostics")
    client = _connect(port, baud)
    try:
        client.set_raw_capture(True)
        col = _collect(client, r, {"health": 10, "api_stats": 2}, seconds)

        snap = client.diagnostics_snapshot()
        r.check(
            "host frames received",
            snap.rx_frames > 0,
            f"rx={snap.rx_frames} tx={snap.tx_frames}",
        )
        r.check("host decode errors zero", snap.decode_errors == 0, "")
        raws = client.drain_raw_frames()
        r.check(
            "raw frame inspector captures",
            len(raws) > 0 and all(f.ok for f in raws),
            f"{len(raws)} frames, all decoded",
        )

        stats = col.get("api_stats")
        if stats:
            last = stats[-1][1]
            r.check(
                "firmware CRC/decode counters zero",
                last.rx_bad == 0 and last.rx_overflow == 0,
                f"rx_frames={last.rx_frames} rx_bad={last.rx_bad} "
                f"rx_overflow={last.rx_overflow}",
            )
        else:
            r.fail("api_stats telemetry", "no records")

        fw_stats = client.get_stream_stats()
        r.check("GET_STREAM_STATS", fw_stats is not None, "")

        # DXL error visibility: servo_status carries hardware_error bits.
        col2 = _collect(client, r, {"servo_status": 10}, 1.5)
        sv = col2.get("servo_status")
        if sv:
            errs = {
                s.id: s.hardware_error
                for _, rec in sv
                for s in rec.servos
                if s.hardware_error
            }
            r.check("DXL error counters clean", not errs, f"errors={errs or 'none'}")
        else:
            r.skip("DXL error counters", "no servo_status (DXL power off)")
    finally:
        client.set_raw_capture(False)
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Page: RC Troubleshooting
# ---------------------------------------------------------------------------


@hil.command("rc-troubleshooting")
def rc_troubleshooting(
    seconds: float = typer.Option(3.0, "--seconds", "-s"),
    expect_receiver: bool = typer.Option(
        False, "--expect-receiver", help="An RC receiver is connected and linked."
    ),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """RC page: rc_input + rc_diagnostics reflect the actual link state."""
    r = Runner("RC Troubleshooting")
    client = _connect(port, baud)
    try:
        col = _collect(client, r, {"rc_input": 10, "rc_diagnostics": 5}, seconds)

        rc = col.get("rc_input")
        diag = col.get("rc_diagnostics")
        r.check("rc_input flows", len(rc) >= 1, f"{len(rc)} records")
        if not r.check("rc_diagnostics flows", len(diag) >= 1, f"{len(diag)} records"):
            r.finish()

        last = diag[-1][1]
        if expect_receiver:
            r.check(
                "receiver linked",
                last.ever_seen and not last.failsafe,
                f"ever_seen={last.ever_seen} failsafe={last.failsafe} "
                f"frames={last.frames_decoded} crc_err={last.crc_errors}",
            )
        else:
            r.check(
                "no-receiver state correct",
                (not last.ever_seen) or last.failsafe,
                f"ever_seen={last.ever_seen} failsafe={last.failsafe} "
                f"age={last.last_frame_age_ms} ms",
            )
        if rc:
            last_rc = rc[-1][1]
            r.check(
                "rc_input failsafe coherent",
                last_rc.failsafe == last.failsafe,
                f"rc_input.failsafe={last_rc.failsafe} "
                f"rc_diag.failsafe={last.failsafe}",
            )
    finally:
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# Global estop reachability (Settings page has no firmware surface).
# ---------------------------------------------------------------------------


@hil.command()
def estop(port: Optional[str] = _PORT, baud: int = _BAUD) -> None:
    """Global estop: trigger + recover (usable standalone from any page)."""
    r = Runner("Global E-stop")
    client = _connect(port, baud)
    try:
        res = _retry(client.estop)
        state = _wait_state(client, lambda s: s == STATE_ESTOP)
        r.check(
            "estop honored",
            res is not None and state == STATE_ESTOP,
            f"state={_state_name(state) if state is not None else '?'}",
        )
        res = _retry(client.clear_fault)
        state = _wait_state(client, lambda s: s != STATE_ESTOP)
        r.check(
            "recovery",
            res is not None and state is not None and state != STATE_ESTOP,
            f"state={_state_name(state) if state is not None else '?'}",
        )
    finally:
        client.stop()
    r.finish()


# ---------------------------------------------------------------------------
# All pages
# ---------------------------------------------------------------------------

_ALL_PAGES: list[tuple[str, Callable]] = []


@hil.command("all")
def run_all(
    soak_seconds: float = typer.Option(
        60.0, "--soak-seconds", help="Duration for the plot-workbench soak."
    ),
    skip_write: bool = typer.Option(
        False, "--skip-write", help="Skip the servo EEPROM limit write."
    ),
    port: Optional[str] = _PORT,
    baud: int = _BAUD,
) -> None:
    """Run every page test in checklist order; aggregate the verdicts."""
    pages: list[tuple[str, Callable[[], None]]] = [
        ("connect", lambda: connect(port=port, baud=baud)),
        ("overview", lambda: overview(seconds=4.0, port=port, baud=baud)),
        ("mode-safety", lambda: mode_safety(port=port, baud=baud)),
        (
            "passive-pose",
            lambda: passive_pose(seconds=4.0, interactive=False, port=port, baud=baud),
        ),
        (
            "servo-tuning",
            lambda: servo_tuning(
                limits_servo=None, skip_write=skip_write, port=port, baud=baud
            ),
        ),
        ("leg-lab", lambda: leg_lab(leg=0, port=port, baud=baud)),
        ("gait-lab", lambda: gait_lab(expect_motion=False, port=port, baud=baud)),
        (
            "sensor-dashboard",
            lambda: sensor_dashboard(seconds=3.0, port=port, baud=baud),
        ),
        ("foot-contact", lambda: foot_contact(port=port, baud=baud)),
        (
            "plot-workbench",
            lambda: plot_workbench(seconds=soak_seconds, port=port, baud=baud),
        ),
        (
            "session-browser",
            lambda: session_browser(
                seconds=5.0, out=Path("data/sessions"), port=port, baud=baud
            ),
        ),
        ("urdf-viewer", lambda: urdf_viewer(seconds=3.0, port=port, baud=baud)),
        ("diagnostics", lambda: diagnostics(seconds=5.0, port=port, baud=baud)),
        (
            "rc-troubleshooting",
            lambda: rc_troubleshooting(
                seconds=3.0, expect_receiver=False, port=port, baud=baud
            ),
        ),
        ("estop", lambda: estop(port=port, baud=baud)),
    ]
    verdicts: dict[str, bool] = {}
    for name, fn in pages:
        try:
            fn()
            verdicts[name] = True  # typer.Exit not raised (unreachable normally)
        except typer.Exit as exc:
            verdicts[name] = exc.exit_code == 0
        except Exception as exc:  # noqa: BLE001
            typer.secho(f"  [FAIL] {name}: crashed: {exc}", fg=typer.colors.RED)
            verdicts[name] = False
        time.sleep(0.5)  # let the port settle between sessions

    typer.secho("\n=== Summary ===", fg=typer.colors.CYAN, bold=True)
    for name, ok in verdicts.items():
        typer.secho(
            f"  {'PASS' if ok else 'FAIL'}  {name}",
            fg=typer.colors.GREEN if ok else typer.colors.RED,
        )
    raise typer.Exit(code=0 if all(verdicts.values()) else 1)


def main() -> None:
    hil()


if __name__ == "__main__":
    sys.exit(hil())
