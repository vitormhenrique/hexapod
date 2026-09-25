"""Qt bridge over the threaded :class:`ProtocolClient`.

Lives between the UI and the transport. Telemetry/connection callbacks fire on
the reader thread; this object re-emits them as Qt signals (queued to the GUI
thread) so widgets never touch the serial thread directly. The UI thread is
never blocked: connect/handshake run in a worker thread.
"""

from __future__ import annotations

import threading
import time

from hexapod_protocol import api
from hexapod_protocol import config as cfg
from hexapod_protocol import telemetry as tlm
from hexapod_protocol.framing import VERSION_MAJOR, VERSION_MINOR, version_compatible
from PySide6.QtCore import QObject, QTimer, Signal

from diagnostics import print_exception
from transport import (
    is_tcp_proxy_endpoint,
    list_serial_ports,
    open_serial,
    open_tcp_proxy,
)
from transport.protocol_client import ProtocolClient

_STATE_DISARMED = tlm.SafetyState.DISARMED
_STATE_MAC_MAINTENANCE = tlm.SafetyState.MAC_MAINTENANCE
_STATE_PASSIVE = tlm.SafetyState.PASSIVE_POSE_STREAM
_SIMULATION_DEVICE_NAME = "HexNav ROS SIM"
_SIMULATION_STREAMS = frozenset(
    (
        int(tlm.StreamId.HEALTH),
        int(tlm.StreamId.CONTROL_STATE),
        int(tlm.StreamId.RC_INPUT),
    )
)


class ConnectionService(QObject):
    connected = Signal(bool)
    connecting = Signal(bool)  # True while a connect/handshake is in progress
    hello_received = Signal(object)  # api.HelloInfo
    status_received = Signal(object)  # api.StatusInfo
    capabilities_received = Signal(object)  # api.Capabilities
    telemetry = Signal(int, object)  # stream_id, decoded record
    event = Signal(str, str)  # kind, detail
    error = Signal(str)
    control_result = Signal(str, object)  # kind, api.ControlResult
    motion_result = Signal(str, object)  # kind, api.MotionResultMsg
    feature_result = Signal(object)  # api.FeatureSetResult
    feature_list = Signal(object)  # api.FeatureList
    maint_result = Signal(object)  # api.MaintResultMsg
    leg_target_result = Signal(object)  # api.LegTargetResult (None on failure)
    joint_target_result = Signal(object)  # api.JointTargetResult (None on failure)
    servo_torque_changed = Signal(int, bool, bool)  # id, enabled, verified
    config_loaded = Signal(object)  # config.RobotConfig (None on failure)
    config_summary = Signal(object)  # config.ConfigSummary (None on failure)
    config_staged = Signal(bool)  # True if every CFG_SET_BLOCK was acked
    config_result = Signal(str, object)  # kind (validate/commit/reset), api.CfgResult
    config_save_changed = Signal(bool, str, object)  # busy, phase/detail, readback
    dxl_result = Signal(str, object)  # kind, api.DxlJobResult (None on failure)
    sensor_feature_result = Signal(str, object)  # kind, api.SensorFeatureResult
    contact_threshold_result = Signal(object)  # api.ContactThresholdResult
    sensor_calibrate_result = Signal(object)  # api.SensorCalibrateResult
    sensor_rate_result = Signal(object)  # api.SensorRateResult
    i2c_topology = Signal(object)  # api.I2cTopologyResult (None on failure)
    sensor_status = Signal(object)  # api.SensorStatusResult (None on failure)
    passive_result = Signal(str, object)  # kind, api.PassiveResult
    passive_rate_result = Signal(object)  # api.PassiveRateResult
    passive_setup_changed = Signal(bool, str)  # busy, phase/detail
    maint_lock_changed = Signal(bool, int)  # held, token
    maintenance_setup_changed = Signal(bool, bool, str)  # busy, ready, detail
    gait_test_changed = Signal(bool, str)  # active, phase/detail
    gait_test_busy_changed = Signal(bool)
    state_changed = Signal(int)  # latest safety state (-1 = unknown/disconnected)
    simulation_mode_changed = Signal(bool)

    def __init__(self) -> None:
        super().__init__()
        self._client: ProtocolClient | None = None
        self._poll = QTimer(self)
        self._poll.setInterval(1000)
        self._poll.timeout.connect(self._poll_status)
        # Latest firmware safety state (-1 = unknown). Tracked on the GUI
        # thread via our own status_received signal so pages can gate
        # controls without each re-decoding StatusInfo.
        self._robot_state: int = -1
        self._simulation_mode = False
        self.status_received.connect(self._track_state)
        # Status-poll lifecycle: QTimer.start()/stop() must run on the thread
        # that owns the timer (this GUI-thread object). connect_to()'s worker
        # and the client's link-lost callback both run on other threads, so
        # they only emit `connected`; the queued delivery of this connection
        # starts/stops the poll safely on the GUI thread. (Calling
        # QTimer.singleShot(0, ...) from the worker never fired: the worker
        # thread has no Qt event loop.)
        self.connected.connect(self._on_connected_changed)
        self._maint_token: int = 0
        self._maint_setup_busy = False
        self._maint_setup_ready = False
        self._config_save_busy = False
        # Maintenance-lock keepalive: the firmware lock TTL is 1 s without a
        # MAINT_HEARTBEAT (AGENTS.md 6.4), so a held lock must be beaten from a
        # background thread or MacMaintenance silently lapses to Disarmed.
        self._maint_hb_stop: threading.Event | None = None
        self._gait_test_active = False
        self._gait_test_busy = False
        self._gait_test_owns_lock = False
        self._gait_test_cancel = threading.Event()
        # True while a passive-pose flow owns the maintenance lock + DXL power
        # it set up, so passive_exit only unwinds what it created (a user who
        # was already doing bench maintenance is left in maintenance).
        self._passive_owns_setup = False
        self._passive_busy = False
        self._connecting = False
        self._connect_max_attempts = 5
        self._connect_retry_delay_s = 2.5
        self._hello_attempts_per_link = 3
        self._stream_leases: dict[object, dict[int, int]] = {}
        self._effective_stream_rates: dict[int, int] = {}
        self._inactive_stream_owners: set[object] = set()

    # --- discovery --------------------------------------------------------

    def available_ports(self) -> list:
        return list_serial_ports()

    # --- lifecycle --------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._client.connected

    @property
    def robot_state(self) -> int:
        """Latest safety state from status polling (-1 when unknown)."""
        return self._robot_state

    @property
    def simulation_mode(self) -> bool:
        """Whether the current endpoint is the local ROS simulated firmware."""
        return self._simulation_mode

    def _track_state(self, st) -> None:
        if st.state != self._robot_state:
            self._robot_state = st.state
            self.state_changed.emit(st.state)

    def _reset_state(self) -> None:
        if self._robot_state != -1:
            self._robot_state = -1
            self.state_changed.emit(-1)

    def _set_simulation_mode(self, enabled: bool) -> None:
        if enabled != self._simulation_mode:
            self._simulation_mode = enabled
            self.simulation_mode_changed.emit(enabled)

    def connect_to(self, port: str, baud: int = 115200) -> None:
        """Open a serial or proxy endpoint and handshake without blocking the UI."""
        if self.is_connected:
            self.disconnect()

        self._connecting = True
        self.connecting.emit(True)

        def worker() -> None:
            attempt_port = port
            last_error: BaseException | None = None
            for attempt in range(1, self._connect_max_attempts + 1):
                if not self._connecting:
                    return
                client: ProtocolClient | None = None
                try:
                    if is_tcp_proxy_endpoint(attempt_port):
                        link = open_tcp_proxy(attempt_port)
                    else:
                        link = open_serial(attempt_port, baud=baud)
                    if link is None:
                        raise RuntimeError(f"could not open {attempt_port}")
                    client = ProtocolClient(link)
                    client.on_telemetry(self._on_telemetry)
                    client.on_connection(
                        lambda value, c=client: self._on_connection(value, c)
                    )
                    self._client = client
                    client.start()
                    # Retry HELLO on the same open link before tearing the port
                    # down: the OpenRB-150 CDC stack dislikes open/close churn,
                    # and a board mid-reboot (watchdog reset re-enumeration)
                    # answers on the second or third try. Only retry while the
                    # link is still alive; a dead link falls through to the
                    # outer reopen/re-resolve loop.
                    hello = None
                    for _ in range(self._hello_attempts_per_link):
                        if not self._connecting:
                            client.stop()
                            return
                        hello = client.hello()
                        if hello is not None or not client.connected:
                            break
                    if hello is None:
                        raise RuntimeError(f"no HELLO response from {attempt_port}")
                    self._set_simulation_mode(
                        hello.device_name == _SIMULATION_DEVICE_NAME
                    )
                    self.hello_received.emit(hello)
                    # Surface a protocol version mismatch as a diagnostic rather
                    # than silently talking to incompatible firmware (4sa.5). The
                    # frame layer carries but does not reject the version; the
                    # handshake is where the host can compare and warn.
                    if not version_compatible(hello.proto_major, hello.proto_minor):
                        self.event.emit(
                            "version",
                            f"protocol mismatch: firmware v{hello.proto_major}."
                            f"{hello.proto_minor} vs host v{VERSION_MAJOR}."
                            f"{VERSION_MINOR}",
                        )
                    self.event.emit(
                        "connect",
                        f"{hello.device_name} fw "
                        f"{hello.fw_major}.{hello.fw_minor}.{hello.fw_patch}",
                    )
                    caps = client.get_capabilities()
                    if caps is None:
                        raise RuntimeError(
                            f"no CAPABILITIES response from {attempt_port}"
                        )
                    self.capabilities_received.emit(caps)
                    if client.connected and self._client is client:
                        self._connecting = False
                        self.connecting.emit(False)
                        # The queued connected -> _on_connected_changed slot
                        # starts the status poll on the GUI thread.
                        self.connected.emit(True)
                        return
                    reason = f"connection to {attempt_port} was lost during handshake"
                    if client.last_error is not None:
                        reason = f"{reason}: {client.last_error}"
                    last_error = RuntimeError(reason)
                    raise last_error
                except Exception as exc:  # noqa: BLE001 - transport boundary
                    last_error = last_error or exc
                    print_exception(
                        f"connect attempt {attempt}/{self._connect_max_attempts} "
                        f"failed for {attempt_port}",
                        exc,
                    )
                    if client is not None:
                        client.stop()
                    if attempt >= self._connect_max_attempts:
                        break
                    if not self._connecting:
                        return
                    self.event.emit(
                        "connect",
                        f"retrying connection ({attempt + 1}/"
                        f"{self._connect_max_attempts})",
                    )
                    time.sleep(self._connect_retry_delay_s)
                    if not self._connecting:
                        return
                    attempt_port = self._retry_port_for(port, attempt_port)

            self._connecting = False
            self.connecting.emit(False)
            self.connected.emit(False)
            detail = f": {last_error}" if last_error is not None else ""
            self.error.emit(f"Connection to {port} failed{detail}")

        threading.Thread(target=worker, name="hexapod-connect", daemon=True).start()

    def _retry_port_for(self, requested_port: str, last_port: str) -> str:
        if is_tcp_proxy_endpoint(requested_port):
            return requested_port
        ports = list_serial_ports()
        if any(p.device == last_port for p in ports):
            return last_port
        if any(p.device == requested_port for p in ports):
            return requested_port
        best_port = requested_port
        best_score = 0
        for p in ports:
            haystack = f"{p.device} {p.description} {p.hwid}".lower()
            if "bluetooth" in haystack:
                continue
            score = 0
            if "openrb" in haystack:
                score += 100
            if "usbmodem" in haystack:
                score += 50
            if str(p.device).startswith("/dev/cu."):
                score += 20
            if score > best_score:
                best_score = score
                best_port = p.device
        return best_port

    def disconnect(self) -> None:
        self._connecting = False
        self._drop_maint_lock(notify=False)
        if self._client is not None:
            self._client.stop()
            self._client = None
        self._reset_state()
        self._set_simulation_mode(False)
        self.connected.emit(False)
        self.event.emit("disconnect", "link closed")

    # --- commands (safe no-ops when disconnected) ------------------------

    def acquire_stream(self, owner: object, stream_id: int, rate_hz: int) -> None:
        """Request a stream for one UI owner using max-rate arbitration."""
        if self._simulation_mode and stream_id not in _SIMULATION_STREAMS:
            return
        requested = self._stream_leases.setdefault(owner, {})
        rate_hz = max(1, int(rate_hz))
        if requested.get(stream_id) == rate_hz:
            return
        requested[stream_id] = rate_hz
        self._apply_stream_rate(stream_id)

    def release_streams(self, owner: object) -> None:
        """Release every stream lease held by one UI owner."""
        requested = self._stream_leases.pop(owner, {})
        for stream_id in requested:
            self._apply_stream_rate(stream_id)

    def release_stream(self, owner: object, stream_id: int) -> None:
        """Release one stream lease while preserving the owner's other feeds."""
        requested = self._stream_leases.get(owner)
        if requested is None or stream_id not in requested:
            return
        requested.pop(stream_id)
        if not requested:
            self._stream_leases.pop(owner, None)
        self._apply_stream_rate(stream_id)

    def set_stream_owner_active(self, owner: object, active: bool) -> None:
        """Include or exclude one owner's leases from effective requests."""
        was_active = owner not in self._inactive_stream_owners
        if active == was_active:
            return
        if active:
            self._inactive_stream_owners.discard(owner)
        else:
            self._inactive_stream_owners.add(owner)
        for stream_id in self._stream_leases.get(owner, {}):
            self._apply_stream_rate(stream_id)

    def _apply_stream_rate(self, stream_id: int) -> None:
        desired = max(
            (
                rates[stream_id]
                for owner, rates in self._stream_leases.items()
                if owner not in self._inactive_stream_owners
                if stream_id in rates
            ),
            default=0,
        )
        current = self._effective_stream_rates.get(stream_id, 0)
        if desired == current:
            return
        if desired:
            self._effective_stream_rates[stream_id] = desired
            self._send_subscribe(stream_id, desired)
        else:
            self._effective_stream_rates.pop(stream_id, None)
            self._send_unsubscribe(stream_id)

    def _restore_stream_leases(self) -> None:
        self._effective_stream_rates.clear()
        stream_ids = {
            stream_id
            for rates in self._stream_leases.values()
            for stream_id in rates
        }
        for stream_id in stream_ids:
            self._apply_stream_rate(stream_id)

    def subscribe(self, stream_id: int, rate_hz: int) -> None:
        """Issue an unowned stream request for CLI and legacy callers."""
        if self._simulation_mode and stream_id not in _SIMULATION_STREAMS:
            return
        self._send_subscribe(stream_id, rate_hz)

    def _send_subscribe(self, stream_id: int, rate_hz: int) -> None:
        if self._client:
            threading.Thread(
                target=lambda: self._client
                and self._client.subscribe(stream_id, rate_hz),
                daemon=True,
            ).start()

    def unsubscribe(self, stream_id: int) -> None:
        """Issue an unowned stream cancellation for legacy callers."""
        self._send_unsubscribe(stream_id)

    def _send_unsubscribe(self, stream_id: int) -> None:
        if self._client:
            threading.Thread(
                target=lambda: self._client and self._client.unsubscribe(stream_id),
                daemon=True,
            ).start()

    def mark_note(self, text: str) -> None:
        """Emit an operator note event (annotates plots + session logs)."""
        note = (text or "").strip()
        if note:
            self.event.emit("note", note)

    # --- diagnostics (safe no-ops when disconnected) ---------------------

    def set_raw_capture(self, enabled: bool) -> None:
        """Toggle bounded raw-frame capture on the reader thread."""
        if self._client:
            self._client.set_raw_capture(enabled)

    def drain_raw_frames(self) -> list:
        """Return and clear captured raw frames (empty when disconnected)."""
        if self._client:
            return self._client.drain_raw_frames()
        return []

    def diagnostics_snapshot(self):
        """Protocol counters snapshot, or ``None`` when disconnected."""
        if self._client:
            return self._client.diagnostics_snapshot()
        return None

    def emergency_stop(self) -> None:
        """Send a real ESTOP command on a worker thread and report the ack."""
        self.event.emit("estop", "operator pressed EMERGENCY STOP")
        client = self._client
        if client is None:
            self.error.emit("ESTOP: not connected")
            return

        def worker() -> None:
            res = client.estop()
            if res is None:
                self.error.emit("ESTOP: no response from firmware")
            else:
                self.control_result.emit("estop", res)
                self.event.emit("estop", f"firmware state={res.state}")

        threading.Thread(target=worker, name="hexapod-estop", daemon=True).start()

    def clear_fault(self) -> None:
        self._run_control("clear_fault", lambda c: c.clear_fault())

    def set_arming(self, arm: bool) -> None:
        self._run_control("arm" if arm else "disarm", lambda c: c.set_arming(arm))

    def set_mode(self, mode: int) -> None:
        self._run_control("set_mode", lambda c: c.set_mode(mode))

    def set_gait(self, gait: int) -> None:
        self._run_motion("set_gait", lambda c: c.set_gait(gait))

    def set_gait_params(
        self,
        body_height_mm: int,
        stride_len_mm: int,
        step_height_mm: int,
        duty_x255: int,
        speed_x255: int,
    ) -> None:
        self._run_motion(
            "set_gait_params",
            lambda c: c.set_gait_params(
                body_height_mm, stride_len_mm, step_height_mm, duty_x255, speed_x255
            ),
        )

    def set_body_twist(self, vx: float, vy: float, wz: float) -> None:
        self._run_motion("set_body_twist", lambda c: c.set_body_twist(vx, vy, wz))

    def set_body_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> None:
        self._run_motion(
            "set_body_pose",
            lambda c: c.set_body_pose(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg),
        )

    def stop_motion(self) -> None:
        self._run_motion("stop_motion", lambda c: c.stop_motion())

    def command_walk(self, gait: int, vx: float, vy: float, wz: float) -> None:
        """Select a walking gait, then apply twist in one ordered transaction."""
        self._run_motion_sequence(
            (
                ("set_gait", lambda c: c.set_gait(gait)),
                ("set_body_twist", lambda c: c.set_body_twist(vx, vy, wz)),
            )
        )

    def hold_stand(self) -> None:
        """Stop motion and return body commands to neutral in firmware order."""
        self._run_motion_sequence(
            (
                ("set_body_twist", lambda c: c.set_body_twist(0.0, 0.0, 0.0)),
                (
                    "set_body_pose",
                    lambda c: c.set_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                ),
                ("stop_motion", lambda c: c.stop_motion()),
            )
        )

    def command_standing_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> None:
        """Enter Stand before applying a body-pose command."""
        self._run_motion_sequence(
            (
                ("set_gait", lambda c: c.set_gait(api.GAIT_STAND)),
                (
                    "set_body_pose",
                    lambda c: c.set_body_pose(
                        x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg
                    ),
                ),
            )
        )

    def set_feature(self, feature: int, enable: bool) -> None:
        """Enable/disable a feature flag; emit the firmware's reflected state."""
        client = self._client
        if client is None:
            self.error.emit("feature: not connected")
            return

        def worker() -> None:
            res = client.feature_set(feature, enable)
            if res is None:
                self.error.emit("feature: no response")
            else:
                self.feature_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    def passive_enter(self) -> None:
        """Enter passive streaming with the DXL bus powered so joint data flows.

        Passive mode reports servo *present positions*, which the firmware only
        reads when the bus is powered and at least one servo has been scanned
        (tasks.cpp gates the present-position read on
        ``servoCount() > 0 && dxlPowerEnabled()``). Entering passive straight
        from Disarmed leaves the bus unpowered, so the state changes but nothing
        streams. This runs the documented bench flow: acquire the maintenance
        lock, power the bus, scan, confirm torque off, request passive, then
        subscribe to joint_state so the Passive Pose page updates live.
        """
        client = self._client
        if client is None:
            self.error.emit("passive enter: not connected")
            return
        if self._passive_busy:
            return
        self._passive_busy = True
        self.passive_setup_changed.emit(True, "preparing passive mode")

        def worker() -> None:
            owns_setup = False
            power_on = False
            success = False
            detail = "passive setup failed"
            try:
                # PASSIVE_ENTER is idempotent. Reconfirm an already-active
                # stream without acquiring a maintenance lock.
                if self._robot_state == _STATE_PASSIVE:
                    self.passive_setup_changed.emit(True, "refreshing passive mode")
                    res = client.passive_enter()
                    if res is None or not res.ok:
                        detail = "firmware rejected passive refresh"
                        self.error.emit(f"passive enter: {detail}")
                        if res is not None:
                            self.passive_result.emit("enter", res)
                        return
                    self.passive_result.emit("enter", res)
                    success = True
                    detail = "passive streaming active"
                    self.event.emit("commit", "passive streaming already active")
                    return

                had_lock = bool(self._maint_token)
                self.passive_setup_changed.emit(True, "acquiring maintenance")
                if not self._acquire_maint_lock(client):
                    detail = "maintenance lock rejected"
                    self.error.emit(f"passive enter: {detail}")
                    return
                owns_setup = not had_lock
                self._passive_owns_setup = owns_setup
                if not self._wait_state(
                    client, _STATE_MAC_MAINTENANCE, timeout=2.0
                ):
                    detail = "did not reach maintenance"
                    self.error.emit(f"passive enter: {detail}")
                    return

                self.passive_setup_changed.emit(True, "powering DXL bus")
                pw = client.dxl_power(True)
                pr = pw.power() if pw and pw.done else None
                if pr is None or not pr.power_on:
                    detail = "DXL power on failed"
                    self.error.emit(f"passive enter: {detail}")
                    return
                power_on = True

                self.passive_setup_changed.emit(True, "discovering 18 servos")
                servos: list = []
                for _ in range(4):
                    time.sleep(1.0)
                    if self._client is not client:
                        detail = "connection lost"
                        return
                    scan = client.dxl_scan(1, 18)
                    servos = scan.servos() if scan and scan.done else []
                    if len(servos) == 18:
                        break
                if len(servos) != 18:
                    detail = f"expected 18 servos, found {len(servos)}"
                    self.error.emit(f"passive enter: {detail}")
                    return

                self.passive_setup_changed.emit(True, "confirming torque off")
                torque = client.dxl_torque(False)
                torque_off = (
                    torque is not None
                    and torque.done
                    and torque.code == api.DXL_CODE_OK
                    and len(torque.data) >= 2
                    and torque.data[0] == 0
                    and torque.data[1] == 18
                )
                if not torque_off:
                    detail = "torque off not confirmed by all servos"
                    self.error.emit(f"passive enter: {detail}")
                    return

                self.passive_setup_changed.emit(True, "entering passive state")
                res = client.passive_enter()
                if res is None or not res.ok:
                    detail = "firmware rejected passive mode"
                    self.error.emit(f"passive enter: {detail}")
                    if res is not None:
                        self.passive_result.emit("enter", res)
                    return
                if not self._wait_state(client, _STATE_PASSIVE, timeout=2.0):
                    detail = "did not reach passive state"
                    self.error.emit(f"passive enter: {detail}")
                    return
                self.passive_result.emit("enter", res)
                success = True
                detail = "passive streaming active"
                self.event.emit("commit", detail)
            except Exception as exc:  # noqa: BLE001 - worker boundary
                detail = f"setup failed: {exc}"
                self.error.emit(f"passive enter: {detail}")
            finally:
                if not success and owns_setup:
                    try:
                        if power_on:
                            client.dxl_torque(False)
                            client.dxl_power(False)
                        self._release_maint_lock(client)
                    except Exception as cleanup_error:  # noqa: BLE001
                        print_exception("passive setup cleanup failed", cleanup_error)
                    self._passive_owns_setup = False
                self._passive_busy = False
                self.passive_setup_changed.emit(False, detail)

        threading.Thread(
            target=worker, name="hexapod-passive-enter", daemon=True
        ).start()

    def passive_exit(self) -> None:
        """Exit passive streaming and unwind the bench setup passive_enter did.

        With the maintenance lock still held the FSM returns to MacMaintenance
        on exit, so power the bus back down there and release the lock, then
        drop the joint_state subscription.
        """
        client = self._client
        if client is None:
            self.error.emit("passive exit: not connected")
            return
        if self._passive_busy:
            return
        self._passive_busy = True
        self.passive_setup_changed.emit(True, "exiting passive mode")

        def worker() -> None:
            detail = "passive exit failed"
            try:
                res = client.passive_exit()
                if res is None:
                    self.error.emit("passive exit: no response")
                    return
                self.passive_result.emit("exit", res)
                if not res.ok:
                    self.error.emit("passive exit: firmware rejected")
                    return
                if self._passive_owns_setup and self._maint_token:
                    self._wait_state(client, _STATE_MAC_MAINTENANCE, timeout=2.0)
                    client.dxl_power(False)
                    self._release_maint_lock(client)
                self._passive_owns_setup = False
                detail = "passive mode exited"
            except Exception as exc:  # noqa: BLE001 - worker boundary
                detail = f"exit failed: {exc}"
                self.error.emit(f"passive exit: {detail}")
            finally:
                self._passive_busy = False
                self.passive_setup_changed.emit(False, detail)

        threading.Thread(
            target=worker, name="hexapod-passive-exit", daemon=True
        ).start()

    def passive_zero_reference(self) -> None:
        self._run_passive("zero", lambda c: c.passive_zero_reference())

    # --- passive-pose orchestration helpers ------------------------------

    def _wait_state(
        self, client: ProtocolClient, target: int, timeout: float = 3.0
    ) -> bool:
        """Poll GET_STATUS until the firmware reports ``target`` (or timeout).

        Command replies echo the state *before* the control task ticks, so the
        bench flow settles on the post-transition state by polling directly
        rather than trusting the immediate ack (mirrors the HIL helper)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._client is not client:
                return False
            st = client.get_status()
            if st is not None and st.state == target:
                self.status_received.emit(st)
                return True
            time.sleep(0.1)
        return False

    def _acquire_maint_lock(self, client: ProtocolClient) -> bool:
        """Acquire (or reuse) the maintenance lock and start its heartbeat."""
        if self._maint_token:
            return True
        res = client.enter_maintenance()
        if res is None or not res.ok or not res.token:
            return False
        self._maint_token = res.token
        self._start_maint_heartbeat(client, res.token)
        self.maint_lock_changed.emit(True, res.token)
        self.maint_result.emit(res)
        self.event.emit("commit", f"maintenance lock acquired (token {res.token})")
        return True

    def _release_maint_lock(self, client: ProtocolClient) -> None:
        """Stop the heartbeat and release the held maintenance lock."""
        token = self._maint_token
        if not token:
            return
        self._stop_maint_heartbeat()
        res = client.exit_maintenance(token)
        self._maint_token = 0
        self.maint_lock_changed.emit(False, 0)
        if res is not None:
            self.maint_result.emit(res)

    def passive_set_stream_rate(self, rate_hz: int) -> None:
        client = self._client
        if client is None:
            self.error.emit("passive: not connected")
            return

        def worker() -> None:
            res = client.passive_set_stream_rate(rate_hz)
            if res is None:
                self.error.emit("passive rate: no response")
            else:
                self.passive_rate_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    @property
    def passive_busy(self) -> bool:
        return self._passive_busy

    def _run_passive(self, kind: str, call) -> None:
        client = self._client
        if client is None:
            self.error.emit(f"passive {kind}: not connected")
            return

        def worker() -> None:
            res = call(client)
            if res is None:
                self.error.emit(f"passive {kind}: no response")
            else:
                self.passive_result.emit(kind, res)

        threading.Thread(target=worker, daemon=True).start()

    # --- contact / leveling / sensor calibration -------------------------

    def set_contact(self, enable: bool) -> None:
        self._run_sensor_feature("contact", lambda c: c.contact_enable(enable))

    def set_leveling(self, enable: bool) -> None:
        self._run_sensor_feature("leveling", lambda c: c.leveling_enable(enable))

    def _run_sensor_feature(self, kind: str, call) -> None:
        client = self._client
        if client is None:
            self.error.emit(f"{kind}: not connected")
            return

        def worker() -> None:
            res = call(client)
            if res is None:
                self.error.emit(f"{kind}: no response")
            else:
                self.sensor_feature_result.emit(kind, res)

        threading.Thread(target=worker, daemon=True).start()

    def set_contact_thresholds(
        self, foot: int, near: int, touch: int, load: int
    ) -> None:
        client = self._client
        if client is None:
            self.error.emit("thresholds: not connected")
            return

        def worker() -> None:
            res = client.contact_set_thresholds(foot, near, touch, load)
            if res is None:
                self.error.emit("thresholds: no response")
            else:
                self.contact_threshold_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    def calibrate_contact(self, foot: int = api.SENSOR_CALIBRATE_ALL) -> None:
        client = self._client
        if client is None:
            self.error.emit("calibrate: not connected")
            return

        def worker() -> None:
            res = client.contact_calibrate(foot)
            if res is None:
                self.error.emit("calibrate: no response")
            else:
                self.sensor_calibrate_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    def set_sensor_rate(self, rate_hz: int) -> None:
        """Stage the foot-sensor poll rate; emit ``sensor_rate_result``."""
        client = self._client
        if client is None:
            self.error.emit("sensor rate: not connected")
            return

        def worker() -> None:
            res = client.sensor_set_rate(rate_hz)
            if res is None:
                self.error.emit("sensor rate: no response")
            else:
                self.sensor_rate_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    def refresh_i2c_topology(self, rescan: bool = False) -> None:
        """Read the I2C topology (optionally re-running discovery first).

        Emits ``i2c_topology`` with the result (``None`` on failure).
        """
        if self._simulation_mode:
            self.i2c_topology.emit(None)
            return
        client = self._client
        if client is None:
            self.error.emit("i2c topology: not connected")
            return

        def worker() -> None:
            if rescan:
                client.i2c_scan()
            res = client.i2c_get_topology()
            if res is None:
                self.error.emit("i2c topology: no response")
            self.i2c_topology.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    def refresh_sensor_status(self) -> None:
        """Read the fused per-foot sensor status; emit ``sensor_status``."""
        if self._simulation_mode:
            self.sensor_status.emit(None)
            return
        client = self._client
        if client is None:
            self.error.emit("sensor status: not connected")
            return

        def worker() -> None:
            res = client.sensor_get_status()
            if res is None:
                self.error.emit("sensor status: no response")
            self.sensor_status.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    def refresh_features(self) -> None:
        """Request the live feature-flag table; emit it via ``feature_list``."""
        client = self._client
        if client is None:
            return

        def worker() -> None:
            fl = client.feature_get()
            if fl is not None:
                self.feature_list.emit(fl)

        threading.Thread(target=worker, daemon=True).start()

    def enter_maintenance(self) -> None:
        """Acquire the maintenance lock, then power and scan the DXL bus.

        Bench work always needs a powered, enumerated bus, so this orchestrates
        the full setup on a worker thread: acquire the lock, wait for
        MacMaintenance, power the DYNAMIXEL rail, then scan so present positions
        and servo profiles are available. Power/scan results are emitted on
        ``dxl_result`` so the servo pages update. A power/scan failure leaves
        the lock held (the user can retry) rather than tearing everything down.
        """
        client = self._client
        if client is None:
            self.error.emit("maintenance: not connected")
            return
        if self._maint_setup_busy:
            return
        self._maint_setup_busy = True
        self._maint_setup_ready = False
        self.maintenance_setup_changed.emit(True, False, "acquiring maintenance")

        def worker() -> None:
            ready = False
            detail = "maintenance setup failed"
            try:
                if not self._acquire_maint_lock(client):
                    detail = "maintenance lock rejected"
                    self.error.emit(f"maintenance: {detail}")
                    return
                self.maintenance_setup_changed.emit(
                    True, False, "waiting for MacMaintenance"
                )
                if not self._wait_state(
                    client, _STATE_MAC_MAINTENANCE, timeout=2.0
                ):
                    detail = "did not reach MacMaintenance"
                    self.error.emit(f"maintenance: {detail}")
                    return

                # GET_STATUS can observe MacMaintenance one control cycle before
                # the DXL API's copied gate state. Retry the cheap power job so
                # one operator click carries through that handoff.
                self.maintenance_setup_changed.emit(
                    True, False, "powering DXL bus"
                )
                pw = None
                pr = None
                for _ in range(5):
                    pw = client.dxl_power(True)
                    pr = pw.power() if pw and pw.done else None
                    if pr is not None and pr.power_on:
                        break
                    time.sleep(0.1)
                self.dxl_result.emit("power", pw)
                if pr is None or not pr.power_on:
                    detail = "DXL power on failed"
                    self.error.emit(f"maintenance: {detail}")
                    return
                self.event.emit("commit", "maintenance: DXL power on")

                self.maintenance_setup_changed.emit(
                    True, False, "scanning servos"
                )
                scan = None
                servos: list = []
                for _ in range(4):
                    time.sleep(1.0)
                    if self._client is not client:
                        detail = "connection lost"
                        return
                    scan = client.dxl_scan(1, 18)
                    servos = scan.servos() if scan and scan.done else []
                    if servos:
                        break
                self.dxl_result.emit("scan", scan)
                if not servos:
                    detail = "no servos found on scan"
                    self.error.emit(f"maintenance: {detail}")
                    return
                ready = True
                detail = f"ready: {len(servos)} servos scanned"
                self.event.emit("commit", f"maintenance: {len(servos)} servos scanned")
            finally:
                self._maint_setup_busy = False
                self._maint_setup_ready = ready
                self.maintenance_setup_changed.emit(False, ready, detail)

        threading.Thread(target=worker, name="hexapod-enter-maint", daemon=True).start()

    def exit_maintenance(self) -> None:
        """Release the cached maintenance lock token."""
        client = self._client
        if client is None:
            self.error.emit("maintenance: not connected")
            return
        token = self._maint_token

        def worker() -> None:
            self._stop_maint_heartbeat()
            res = client.exit_maintenance(token)
            if res is None:
                if token and token == self._maint_token and self._client is client:
                    self._start_maint_heartbeat(client, token)
                self.error.emit("maintenance: no response")
                return
            if res.ok:
                self._maint_token = 0
                self._maint_setup_busy = False
                self._maint_setup_ready = False
                self.maint_lock_changed.emit(False, 0)
                self.maintenance_setup_changed.emit(
                    False, False, "maintenance exited"
                )
                self.event.emit("commit", "maintenance lock released")
            else:
                # A rejected token is no longer authoritative. Clear it rather
                # than leaving the UI enabled with a stopped heartbeat.
                self._maint_token = 0
                self._maint_setup_busy = False
                self._maint_setup_ready = False
                self.maint_lock_changed.emit(False, 0)
                self.maintenance_setup_changed.emit(
                    False, False, "maintenance lock was rejected"
                )
                self.error.emit("maintenance: lock release rejected")
            self.maint_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    @property
    def maint_lock_held(self) -> bool:
        return self._maint_token != 0

    @property
    def maintenance_setup_busy(self) -> bool:
        """True while enter_maintenance's lock/power/scan worker is running."""
        return self._maint_setup_busy

    @property
    def maintenance_setup_ready(self) -> bool:
        """True once maintenance is entered with a powered, scanned DXL bus."""
        return self._maint_setup_ready

    def _start_maint_heartbeat(self, client: ProtocolClient, token: int) -> None:
        """Beat the maintenance lock every 0.25 s (firmware TTL is 1 s).

        Three consecutive lost beats are needed before the lock lapses, so a
        single request lost to a busy serial link (e.g. a DXL scan burst)
        cannot drop MacMaintenance. Mirrors the proven HIL ``_MaintLock``.
        """
        self._stop_maint_heartbeat()
        stop = threading.Event()
        self._maint_hb_stop = stop

        def beat() -> None:
            misses = 0
            while not stop.wait(0.25):
                if self._client is not client or token != self._maint_token:
                    return
                try:
                    res = client.maint_heartbeat(token)
                except Exception:  # noqa: BLE001 - heartbeat transport boundary
                    return
                if res is None:
                    misses += 1
                    if misses < 3:
                        continue
                elif res.ok:
                    misses = 0
                    continue
                # Firmware says the lock is gone (expired/revoked) or the link
                # dropped three beats: reflect reality in the UI.
                if token == self._maint_token:
                    self._maint_token = 0
                    self.maint_lock_changed.emit(False, 0)
                    self.event.emit("error", "maintenance lock lost")
                return

        threading.Thread(target=beat, name="hexapod-maint-hb", daemon=True).start()

    def _stop_maint_heartbeat(self) -> None:
        if self._maint_hb_stop is not None:
            self._maint_hb_stop.set()
            self._maint_hb_stop = None

    def _drop_maint_lock(self, notify: bool = True) -> None:
        """Clear local lock state (link is going away; no EXIT possible)."""
        self._stop_maint_heartbeat()
        self._maint_setup_busy = False
        self._maint_setup_ready = False
        self.maintenance_setup_changed.emit(False, False, "connection lost")
        if self._maint_token:
            self._maint_token = 0
            self.maint_lock_changed.emit(False, 0)
            if notify:
                self.event.emit("error", "maintenance lock lost (link down)")
        if self._gait_test_active or self._gait_test_busy:
            self._gait_test_active = False
            self._gait_test_busy = False
            self._gait_test_owns_lock = False
            self._gait_test_cancel.set()
            self.gait_test_busy_changed.emit(False)
            self.gait_test_changed.emit(False, "connection lost")

    @property
    def gait_test_active(self) -> bool:
        return self._gait_test_active

    @property
    def gait_test_busy(self) -> bool:
        return self._gait_test_busy

    def start_gait_test(
        self,
        body_height_mm: int,
        stride_len_mm: int,
        step_height_mm: int,
        duty_x255: int,
        speed_x255: int,
    ) -> None:
        """Start high-level gait control under MacMaintenance, without RC."""
        client = self._client
        if client is None:
            self.error.emit("gait test: not connected")
            return
        if self._gait_test_active:
            self.gait_test_changed.emit(True, "running")
            return
        if self._gait_test_busy:
            return

        if self._simulation_mode:
            self._start_simulated_gait_test(
                client,
                body_height_mm,
                stride_len_mm,
                step_height_mm,
                duty_x255,
                speed_x255,
            )
            return

        self._gait_test_busy = True
        self._gait_test_cancel.clear()
        self.gait_test_busy_changed.emit(True)
        self.gait_test_changed.emit(False, "starting")
        acquired_here = not bool(self._maint_token)

        def fail(detail: str, acquired_here: bool) -> None:
            try:
                client.stop_motion()
                client.dxl_torque(False)
                client.set_maint_control_mode(api.MAINT_CONTROL_JOINT_TARGETS)
                if acquired_here:
                    client.dxl_power(False)
                    self._release_maint_lock(client)
            except Exception as cleanup_error:  # noqa: BLE001
                print_exception("gait setup cleanup failed", cleanup_error)
            self._gait_test_active = False
            self._gait_test_busy = False
            self._gait_test_owns_lock = False
            self.gait_test_busy_changed.emit(False)
            self.gait_test_changed.emit(False, detail)
            if detail != "start cancelled":
                self.error.emit(f"gait test: {detail}")

        def cancelled(acquired_here: bool) -> bool:
            if not self._gait_test_cancel.is_set():
                return False
            fail("start cancelled", acquired_here)
            return True

        def worker() -> None:
            self.gait_test_changed.emit(False, "acquiring maintenance")
            if not self._acquire_maint_lock(client):
                fail("maintenance lock rejected", acquired_here)
                return
            if cancelled(acquired_here):
                return
            if not self._wait_state(client, _STATE_MAC_MAINTENANCE, timeout=2.0):
                fail("did not reach MacMaintenance", acquired_here)
                return
            if cancelled(acquired_here):
                return

            self.gait_test_changed.emit(False, "powering DXL bus")
            power = client.dxl_power(True)
            power_state = power.power() if power and power.done else None
            if power_state is None or not power_state.power_on:
                fail("DXL power on failed", acquired_here)
                return
            if cancelled(acquired_here):
                return

            self.gait_test_changed.emit(False, "discovering 18 servos")
            servos = []
            for _ in range(4):
                time.sleep(1.0)
                if self._client is not client:
                    return
                scan = client.dxl_scan(1, 18)
                servos = scan.servos() if scan and scan.done else []
                if len(servos) == 18:
                    break
                if cancelled(acquired_here):
                    return
            if len(servos) != 18:
                fail(f"expected 18 servos, found {len(servos)}", acquired_here)
                return
            if cancelled(acquired_here):
                return

            mode = client.set_maint_control_mode(
                api.MAINT_CONTROL_GAIT_PIPELINE
            )
            if mode is None or not mode.ok:
                fail("firmware rejected gait-pipeline mode", acquired_here)
                return
            if cancelled(acquired_here):
                return

            # Seed a neutral, static intent before torque is enabled. The DXL
            # torque job separately seeds every Goal Position from its measured
            # Present Position, so this transition cannot replay stale goals.
            client.set_gait_params(
                body_height_mm,
                stride_len_mm,
                step_height_mm,
                duty_x255,
                speed_x255,
            )
            client.set_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            client.set_body_twist(0.0, 0.0, 0.0)
            client.set_gait(api.GAIT_STAND)

            self.gait_test_changed.emit(False, "enabling torque")
            torque = client.dxl_torque(True)
            torque_ok = (
                torque is not None
                and torque.done
                and torque.code == api.DXL_CODE_OK
                and len(torque.data) >= 2
                and torque.data[0] == 1
                and torque.data[1] == 18
            )
            if not torque_ok:
                fail("not all 18 servos enabled torque", acquired_here)
                return

            self._gait_test_active = True
            self._gait_test_busy = False
            self._gait_test_owns_lock = acquired_here
            self.gait_test_busy_changed.emit(False)
            self.gait_test_changed.emit(True, "running in MacMaintenance")
            self.event.emit("commit", "gait test running without RC")

        def guarded_worker() -> None:
            try:
                worker()
            except Exception as exc:  # noqa: BLE001 - worker boundary
                fail(f"setup failed: {exc}", acquired_here)

        threading.Thread(
            target=guarded_worker, name="hexapod-gait-start", daemon=True
        ).start()

    def _start_simulated_gait_test(
        self,
        client: ProtocolClient,
        body_height_mm: int,
        stride_len_mm: int,
        step_height_mm: int,
        duty_x255: int,
        speed_x255: int,
    ) -> None:
        """Start Gait Lab in ROS SIL without any DXL maintenance operations."""
        self._gait_test_busy = True
        self._gait_test_cancel.clear()
        self.gait_test_busy_changed.emit(True)
        self.gait_test_changed.emit(False, "starting ROS simulation")

        def worker() -> None:
            try:
                arming = client.set_arming(True)
                results = (
                    client.set_gait_params(
                        body_height_mm,
                        stride_len_mm,
                        step_height_mm,
                        duty_x255,
                        speed_x255,
                    ),
                    client.set_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    client.set_body_twist(0.0, 0.0, 0.0),
                    client.set_gait(api.GAIT_STAND),
                )
            except Exception as exc:  # noqa: BLE001 - worker boundary
                arming = None
                results = ()
                detail = f"simulation setup failed: {exc}"
            else:
                detail = (
                    "simulation arming rejected"
                    if arming is None or not arming.ok
                    else "simulation command rejected"
                )

            if self._gait_test_cancel.is_set():
                self._gait_test_busy = False
                self.gait_test_busy_changed.emit(False)
                self.gait_test_changed.emit(False, "start cancelled")
                return
            if (
                arming is None
                or not arming.ok
                or not results
                or any(result is None or not result.ok for result in results)
            ):
                self._gait_test_busy = False
                self.gait_test_busy_changed.emit(False)
                self.gait_test_changed.emit(False, detail)
                self.error.emit(f"gait test: {detail}")
                return

            self._gait_test_active = True
            self._gait_test_busy = False
            self._gait_test_owns_lock = False
            self.gait_test_busy_changed.emit(False)
            self.gait_test_changed.emit(True, "running in ROS simulation")
            self.event.emit("commit", "gait controls running in ROS simulation")

        threading.Thread(
            target=worker, name="hexapod-sim-gait-start", daemon=True
        ).start()

    def stop_gait_test(self) -> None:
        """Stop motion, disable torque, and unwind this page's bench session."""
        client = self._client
        if client is None:
            self._gait_test_active = False
            self._gait_test_busy = False
            self.gait_test_busy_changed.emit(False)
            self.gait_test_changed.emit(False, "disconnected")
            return
        if self._gait_test_busy and not self._gait_test_active:
            self._gait_test_cancel.set()
            self.gait_test_changed.emit(False, "stopping setup")
            return
        if not self._gait_test_active:
            return

        if self._simulation_mode:
            self._stop_simulated_gait_test(client)
            return

        self._gait_test_busy = True
        self.gait_test_busy_changed.emit(True)
        self.gait_test_changed.emit(True, "stopping")

        def worker() -> None:
            try:
                client.set_body_twist(0.0, 0.0, 0.0)
                client.set_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                client.stop_motion()
                time.sleep(0.35)
                torque = client.dxl_torque(False)
            except Exception as exc:  # noqa: BLE001 - worker boundary
                self._gait_test_busy = False
                self.gait_test_busy_changed.emit(False)
                detail = f"stop failed: {exc}; retry Stop or use ESTOP"
                self.gait_test_changed.emit(True, detail)
                self.error.emit(f"gait test: {detail}")
                return

            torque_off = (
                torque is not None
                and torque.done
                and torque.code == api.DXL_CODE_OK
                and len(torque.data) >= 2
                and torque.data[0] == 0
                and torque.data[1] == 18
            )
            if not torque_off:
                self._gait_test_busy = False
                self.gait_test_busy_changed.emit(False)
                detail = "torque off not confirmed; retry Stop or use ESTOP"
                self.gait_test_changed.emit(True, detail)
                self.error.emit(f"gait test: {detail}")
                return

            client.set_maint_control_mode(api.MAINT_CONTROL_JOINT_TARGETS)
            if self._gait_test_owns_lock:
                client.dxl_power(False)
                self._release_maint_lock(client)
            self._gait_test_active = False
            self._gait_test_busy = False
            self._gait_test_owns_lock = False
            self.gait_test_busy_changed.emit(False)
            self.gait_test_changed.emit(False, "stopped; torque off")
            self.event.emit("commit", "gait test stopped")

        threading.Thread(target=worker, name="hexapod-gait-stop", daemon=True).start()

    def _stop_simulated_gait_test(self, client: ProtocolClient) -> None:
        """Stop ROS SIL motion without issuing hardware torque commands."""
        self._gait_test_busy = True
        self.gait_test_busy_changed.emit(True)
        self.gait_test_changed.emit(True, "stopping ROS simulation")

        def worker() -> None:
            try:
                results = (
                    client.set_body_twist(0.0, 0.0, 0.0),
                    client.set_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    client.stop_motion(),
                )
                client.set_arming(False)
            except Exception as exc:  # noqa: BLE001 - worker boundary
                results = ()
                detail = f"simulation stop failed: {exc}"
            else:
                detail = "simulation stop rejected"

            self._gait_test_busy = False
            self.gait_test_busy_changed.emit(False)
            if not results or any(result is None or not result.ok for result in results):
                self.gait_test_changed.emit(True, detail)
                self.error.emit(f"gait test: {detail}")
                return
            self._gait_test_active = False
            self.gait_test_changed.emit(False, "stopped in ROS simulation")
            self.event.emit("commit", "ROS simulation gait controls stopped")

        threading.Thread(
            target=worker, name="hexapod-sim-gait-stop", daemon=True
        ).start()

    def set_leg_target(self, leg: int, x_mm: int, y_mm: int, z_mm: int) -> None:
        """Command one leg's foot to (x, y, z) mm (body frame); emit IK verdict.

        Honored only in MacMaintenance with the lock held; the firmware reports
        reachability, per-joint clamp masks, and resulting servo ticks.
        """
        client = self._client
        if client is None:
            self.error.emit("leg target: not connected")
            return

        def worker() -> None:
            res = client.set_leg_target(leg, x_mm, y_mm, z_mm)
            if res is None:
                self.error.emit("leg target: no response")
            self.leg_target_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    def set_joint_target(self, leg: int, joint: int, angle_cdeg: int) -> None:
        """Command one joint (leg, joint) to ``angle_cdeg`` centidegrees.

        Honored only in MacMaintenance with the lock held; the firmware reports
        the travel-limit clamp flags and resulting servo tick.
        """
        client = self._client
        if client is None:
            self.error.emit("joint target: not connected")
            return

        def worker() -> None:
            res = client.set_joint_target(leg, joint, angle_cdeg)
            if res is None:
                self.error.emit("joint target: no response")
            self.joint_target_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    def set_joint_target_with_torque(
        self, servo_id: int, leg: int, joint: int, angle_cdeg: int
    ) -> None:
        """Enable one servo with read-back verification, then command its joint."""
        client = self._client
        if client is None:
            self.error.emit("joint target: not connected")
            return

        def worker() -> None:
            res = client.set_joint_target(leg, joint, angle_cdeg)
            self.joint_target_result.emit(res)
            if res is None or not res.ok:
                self.error.emit("joint target: rejected before torque enable")
                return
            # While this servo is torque-off, let controlTask publish and
            # dxlTask write the new Goal Position before making it stiff.
            time.sleep(0.08)
            torque = client.dxl_set_param(
                servo_id, api.DXL_PARAM_TORQUE_ENABLE, 1
            )
            verified = self._torque_write_verified(torque, True)
            self.servo_torque_changed.emit(servo_id, True, verified)
            if not verified:
                self.error.emit(f"joint target: servo {servo_id} torque enable failed")

        threading.Thread(target=worker, daemon=True).start()

    # --- OpenLog-backed robot config -------------------------------------

    def load_config(self) -> None:
        """Read the staged robot config (windowed) and emit ``config_loaded``."""
        if self._simulation_mode:
            self.config_summary.emit(None)
            self.config_loaded.emit(None)
            return
        client = self._client
        if client is None:
            self.error.emit("config: not connected")
            return

        def worker() -> None:
            summary = client.cfg_get_summary()
            self.config_summary.emit(summary)
            cfg = client.read_config()
            if cfg is None:
                self.error.emit("config: read failed")
            self.config_loaded.emit(cfg)

        threading.Thread(target=worker, daemon=True).start()

    def stage_config(self, config) -> None:
        """Push a full config to the staging buffer; emit ``config_staged``."""
        client = self._client
        if client is None:
            self.error.emit("config: not connected")
            return

        def worker() -> None:
            ok = client.write_config(config)
            if not ok:
                self.error.emit("config: stage failed")
            self.config_staged.emit(ok)

        threading.Thread(target=worker, daemon=True).start()

    def validate_config(self) -> None:
        self._run_cfg("validate", lambda c: c.cfg_validate())

    def commit_config(self) -> None:
        self._run_cfg("commit", lambda c: c.cfg_commit())

    def save_config(self, config) -> None:
        """Stage, validate, torque-off, commit, and verify one RobotConfig."""
        client = self._client
        if client is None:
            self.error.emit("config save: not connected")
            return
        if self._config_save_busy:
            self.error.emit("config save: another save is already running")
            return
        self._config_save_busy = True
        self.config_save_changed.emit(True, "staging config", None)

        def fail(detail: str) -> None:
            self.event.emit("error", f"config save failed: {detail}")
            self.error.emit(f"config save: {detail}")

        def worker() -> None:
            readback = None
            detail = "config save failed"
            try:
                expected = cfg.encode_robot_config(config)
                if not client.write_config(config):
                    detail = "staging failed"
                    fail(detail)
                    return

                self.config_save_changed.emit(True, "validating config", None)
                validation = client.cfg_validate()
                if validation is None or not validation.ok:
                    code = getattr(validation, "result", "no response")
                    detail = f"validation failed ({code})"
                    fail(detail)
                    return

                self.config_save_changed.emit(True, "disabling servo torque", None)
                torque = client.dxl_torque(False)
                torque_off = (
                    torque is not None
                    and torque.done
                    and torque.code == api.DXL_CODE_OK
                    and len(torque.data) >= 2
                    and torque.data[0] == 0
                    and torque.data[1] == len(config.servos)
                )
                if not torque_off:
                    detail = "torque off was not verified for every servo"
                    fail(detail)
                    return

                time.sleep(0.15)
                self.config_save_changed.emit(True, "committing to OpenLog", None)
                committed = client.cfg_commit()
                if committed is None or not committed.ok:
                    code = getattr(committed, "result", "no response")
                    detail = f"commit failed ({code})"
                    fail(detail)
                    return

                self.config_save_changed.emit(True, "verifying read-back", None)
                readback = client.read_config()
                if readback is None:
                    detail = "read-back failed"
                    fail(detail)
                    return
                if cfg.encode_robot_config(readback) != expected:
                    detail = "read-back does not match the saved config"
                    readback = None
                    fail(detail)
                    return

                detail = "saved and verified"
                self.config_loaded.emit(readback)
                self.event.emit("commit", "config saved and read-back verified")
            except Exception as exc:  # noqa: BLE001 - transaction boundary
                detail = f"save failed: {exc}"
                fail(detail)
            finally:
                self._config_save_busy = False
                self.config_save_changed.emit(False, detail, readback)

        threading.Thread(
            target=worker, name="hexapod-config-save", daemon=True
        ).start()

    @property
    def config_save_busy(self) -> bool:
        return self._config_save_busy

    def reset_config_defaults(self) -> None:
        self._run_cfg("reset", lambda c: c.cfg_reset_defaults())

    def _run_cfg(self, kind: str, call) -> None:
        client = self._client
        if client is None:
            self.error.emit(f"config {kind}: not connected")
            return

        def worker() -> None:
            res = call(client)
            if res is None:
                self.error.emit(f"config {kind}: no response")
            else:
                if res.ok:
                    self.event.emit("commit", f"config {kind} ok")
                else:
                    self.event.emit("error", f"config {kind} failed ({res.result})")
            self.config_result.emit(kind, res)

        threading.Thread(target=worker, daemon=True).start()

    @property
    def client(self) -> ProtocolClient | None:
        return self._client

    # --- DXL maintenance jobs (submit+poll on a worker thread) -----------

    def _run_dxl(self, kind: str, call) -> None:
        """Run a blocking DXL job (submit + poll) off the GUI thread."""
        client = self._client
        if client is None:
            self.error.emit(f"{kind}: not connected")
            return

        def worker() -> None:
            res = call(client)
            if res is None:
                self.error.emit(f"{kind}: no result (rejected or timed out)")
            self.dxl_result.emit(kind, res)

        threading.Thread(target=worker, name=f"hexapod-dxl-{kind}", daemon=True).start()

    def dxl_power(self, on: bool) -> None:
        self._run_dxl("power", lambda c: c.dxl_power(on))

    def dxl_torque(self, on: bool) -> None:
        self._run_dxl("torque", lambda c: c.dxl_torque(on))

    def dxl_scan(self) -> None:
        # Scan IDs 1-30 (the config space) with a generous job timeout: freshly
        # powered MX-28s take >1 s to answer pings (HIL 2e8), and the full
        # sweep exceeds the 2 s dxl_run default.
        self._run_dxl(
            "scan", lambda c: c.dxl_run(api.build_dxl_scan(1, 30), timeout=8.0)
        )

    def center_all_joints(self) -> None:
        """Safely arm every servo, then center all joints atomically.

        Read the active servo map and invert raw tick 2048 for each joint. A
        The CAD default maps zero relative angle and raw tick 2048 to the same
        pose, while calibrated configs may carry per-servo trims/signs. The 18
        mapped angles are sent atomically after DXL_TORQUE-on has seeded every
        Goal Position from Present Position. Firmware still clamps every target
        to configured travel and requires MacMaintenance + lock.
        """
        client = self._client
        if client is None:
            self.error.emit("center all: not connected")
            return

        def worker() -> None:
            robot_config = client.read_config()
            if robot_config is None:
                self.event.emit("error", "center all: failed to read active config")
                self.joint_target_result.emit(None)
                return
            # A previous Gait Lab session can leave the maintenance controller
            # in GaitPipeline mode, where joint targets are silently ignored.
            mode = client.set_maint_control_mode(api.MAINT_CONTROL_JOINT_TARGETS)
            if mode is None or not mode.ok:
                self.event.emit(
                    "error", "center all: failed to select joint-target mode"
                )
                self.joint_target_result.emit(None)
                return
            servo_map = cfg.ServoMap(robot_config)
            angles_cdeg: list[int] = []
            for leg in range(cfg.NUM_LEGS):
                for joint in range(cfg.JOINTS_PER_LEG):
                    angle = servo_map.tick_to_angle(
                        leg, joint, cfg.SERVO_CENTER_TICK
                    )
                    angles_cdeg.append(round(angle * cfg.RAD_TO_DEG * 100.0))
            torque = client.dxl_torque(True)
            torque_ok = (
                torque is not None
                and torque.done
                and torque.code == api.DXL_CODE_OK
                and len(torque.data) >= 2
                and torque.data[0] == 1
                and torque.data[1] == len(robot_config.servos)
            )
            if not torque_ok:
                self.event.emit("error", "center all: failed to arm every servo")
                self.joint_target_result.emit(None)
                return
            for servo in robot_config.servos:
                self.servo_torque_changed.emit(servo.id, True, True)
            res = client.set_all_joint_targets(angles_cdeg)
            if res is None:
                self.event.emit("error", "center all: rejected or timed out")
            elif res.ok:
                self.event.emit(
                    "commit", f"center all: {res.stored_count}/18 joints accepted"
                )
            else:
                self.event.emit("error", f"center all: rejected (result {res.result})")
            self.joint_target_result.emit(res)

        threading.Thread(target=worker, name="hexapod-center-all", daemon=True).start()

    def set_all_joint_targets_with_torque(self, angles_cdeg: list[int]) -> None:
        """Enable all configured servos, then atomically apply 18 joint targets."""
        client = self._client
        if client is None:
            self.error.emit("all joint targets: not connected")
            return
        expected_count = cfg.NUM_LEGS * cfg.JOINTS_PER_LEG
        if len(angles_cdeg) != expected_count:
            self.error.emit(
                f"all joint targets: expected {expected_count}, got {len(angles_cdeg)}"
            )
            return
        targets = list(angles_cdeg)

        def worker() -> None:
            robot_config = client.read_config()
            if robot_config is None:
                self.error.emit("all joint targets: failed to read active config")
                self.joint_target_result.emit(None)
                return
            mode = client.set_maint_control_mode(api.MAINT_CONTROL_JOINT_TARGETS)
            if mode is None or not mode.ok:
                self.error.emit("all joint targets: failed to select joint-target mode")
                self.joint_target_result.emit(None)
                return
            torque = client.dxl_torque(True)
            torque_ok = (
                torque is not None
                and torque.done
                and torque.code == api.DXL_CODE_OK
                and len(torque.data) >= 2
                and torque.data[0] == 1
                and torque.data[1] == len(robot_config.servos)
            )
            if not torque_ok:
                self.error.emit("all joint targets: failed to enable every servo")
                self.joint_target_result.emit(None)
                return
            for servo in robot_config.servos:
                self.servo_torque_changed.emit(servo.id, True, True)
            result = client.set_all_joint_targets(targets)
            if result is None:
                self.error.emit("all joint targets: rejected or timed out")
            elif result.ok:
                self.event.emit(
                    "commit",
                    f"all joint targets: {result.stored_count}/{expected_count} accepted",
                )
            else:
                self.error.emit(
                    f"all joint targets: rejected (result {result.result})"
                )
            self.joint_target_result.emit(result)

        threading.Thread(
            target=worker, name="hexapod-all-joint-targets", daemon=True
        ).start()

    def set_joint_targets_raw_ticks(
        self,
        targets: list[tuple[int, int, int]],
        robot_config=None,
        ensure_setup: bool = True,
    ) -> None:
        """Command joints to exact physical raw ticks.

        The firmware maps ``SET_JOINT_TARGET`` angles through the CONFIGURED
        servo map (sign/trim), so each requested tick is inverted through the
        same map (``tick_to_angle``) before sending -- the round trip lands on
        the requested physical tick regardless of the persisted calibration.
        ``robot_config`` avoids a slow windowed config read when the caller
        already holds the active config. ``ensure_setup=False`` skips the
        control-mode/torque handshake for fast nudges after an initial
        commanded pose. Each entry is ``(leg, joint, tick)``. Emits
        ``joint_target_result``.
        """
        client = self._client
        if client is None:
            self.error.emit("calib ticks: not connected")
            self.joint_target_result.emit(None)
            return

        def worker() -> None:
            config = robot_config
            if config is None:
                config = client.read_config()
                if config is None:
                    self.error.emit("calib ticks: failed to read active config")
                    self.joint_target_result.emit(None)
                    return
            servo_map = cfg.ServoMap(config)
            if ensure_setup:
                # Joint-target control mode so the gait pipeline cannot
                # overwrite, then torque on (seeds goals from present pose).
                mode = client.set_maint_control_mode(
                    api.MAINT_CONTROL_JOINT_TARGETS
                )
                if mode is None or not mode.ok:
                    self.error.emit(
                        "calib ticks: failed to select joint-target mode"
                    )
                    self.joint_target_result.emit(None)
                    return
                torque = client.dxl_torque(True)
                torque_ok = (
                    torque is not None
                    and torque.done
                    and torque.code == api.DXL_CODE_OK
                )
                if not torque_ok:
                    self.error.emit("calib ticks: failed to enable torque")
                    self.joint_target_result.emit(None)
                    return
            last = None
            for leg, joint, tick in targets:
                t = max(0, min(cfg.SERVO_MAX_TICK, int(tick)))
                servo = servo_map.servo_for(leg, joint)
                if servo is None:
                    self.error.emit(
                        f"calib ticks: leg {leg + 1} joint {joint} unmapped"
                    )
                    self.joint_target_result.emit(None)
                    return
                angle_cdeg = round(
                    cfg.tick_to_angle(servo, t) * cfg.RAD_TO_DEG * 100.0
                )
                last = client.set_joint_target(leg, joint, angle_cdeg)
                if last is None or not last.ok:
                    self.error.emit(
                        f"calib ticks: leg {leg + 1} joint {joint} rejected"
                    )
                    self.joint_target_result.emit(last)
                    return
            self.event.emit(
                "commit", f"calib ticks: commanded {len(targets)} joint(s)"
            )
            self.joint_target_result.emit(last)

        threading.Thread(
            target=worker, name="hexapod-calib-ticks", daemon=True
        ).start()

    def set_leg_fixture_pose_raw(
        self,
        leg: int,
        coxa_tick: int,
        femur_tick: int,
        tibia_tick: int,
        robot_config=None,
    ) -> None:
        """Command one leg's three joints to exact physical raw ticks."""
        self.set_joint_targets_raw_ticks(
            [
                (leg, 0, coxa_tick),
                (leg, 1, femur_tick),
                (leg, 2, tibia_tick),
            ],
            robot_config=robot_config,
        )

    def dxl_get_param(self, servo_id: int, param: int) -> None:
        self._run_dxl("get_param", lambda c: c.dxl_get_param(servo_id, param))

    def dxl_set_param(self, servo_id: int, param: int, value: int) -> None:
        self._run_dxl("set_param", lambda c: c.dxl_set_param(servo_id, param, value))

    def set_servo_torque(self, servo_id: int, enabled: bool) -> None:
        """Set and read-back verify torque for one servo."""
        client = self._client
        if client is None:
            self.error.emit("servo torque: not connected")
            return

        def worker() -> None:
            res = client.dxl_set_param(
                servo_id, api.DXL_PARAM_TORQUE_ENABLE, int(enabled)
            )
            verified = self._torque_write_verified(res, enabled)
            if not verified:
                self.error.emit(f"servo torque: servo {servo_id} write failed")
            self.servo_torque_changed.emit(servo_id, enabled, verified)

        threading.Thread(target=worker, daemon=True).start()

    def set_dxl_positions_with_torque(
        self, targets: list[tuple[int, int]]
    ) -> None:
        """Write verified raw goal ticks, then enable each targeted servo.

        This path is intentionally independent of the configured leg/joint map.
        Each tuple is ``(servo_id, goal_tick)`` from the latest DXL scan. Goal
        writes complete before any torque enable, so staged multi-servo commands
        cannot energize against stale register values.
        """
        client = self._client
        if client is None:
            self.error.emit("DXL positions: not connected")
            return
        staged = [(int(servo_id), int(tick)) for servo_id, tick in targets]
        if not staged:
            return

        def worker() -> None:
            for servo_id, tick in staged:
                result = client.dxl_set_param(
                    servo_id, api.DXL_PARAM_GOAL_POSITION, tick
                )
                value = result.set_param() if result is not None else None
                if (
                    result is None
                    or not result.done
                    or result.code != api.DXL_CODE_OK
                    or value is None
                    or not value.verified
                ):
                    detail = f"servo {servo_id} goal write failed"
                    self.error.emit(f"DXL positions: {detail}")
                    self.dxl_result.emit(
                        "goal_positions", {"ok": False, "detail": detail}
                    )
                    return
            for servo_id, _tick in staged:
                torque = client.dxl_set_param(
                    servo_id, api.DXL_PARAM_TORQUE_ENABLE, 1
                )
                verified = self._torque_write_verified(torque, True)
                self.servo_torque_changed.emit(servo_id, True, verified)
                if not verified:
                    detail = f"servo {servo_id} torque enable failed"
                    self.error.emit(f"DXL positions: {detail}")
                    self.dxl_result.emit(
                        "goal_positions", {"ok": False, "detail": detail}
                    )
                    return
            detail = f"commanded {len(staged)} servo(s)"
            self.event.emit("commit", f"DXL positions: {detail}")
            self.dxl_result.emit(
                "goal_positions", {"ok": True, "detail": detail}
            )

        threading.Thread(
            target=worker, name="hexapod-dxl-positions", daemon=True
        ).start()

    @staticmethod
    def _torque_write_verified(result, enabled: bool) -> bool:
        param = result.set_param() if result is not None else None
        return bool(
            result is not None
            and result.done
            and result.code == api.DXL_CODE_OK
            and param is not None
            and param.param == api.DXL_PARAM_TORQUE_ENABLE
            and param.verified
            and param.readback == int(enabled)
        )

    def dxl_set_servo_limits(self, servo_id: int, min_tick: int, max_tick: int) -> None:
        self._run_dxl(
            "set_limits", lambda c: c.dxl_set_servo_limits(servo_id, min_tick, max_tick)
        )

    def dxl_read_register(self, servo_id: int, address: int, length: int) -> None:
        self._run_dxl(
            "read_register", lambda c: c.dxl_read_register(servo_id, address, length)
        )

    def dxl_write_register(
        self, servo_id: int, address: int, length: int, value: int, is_eeprom: bool
    ) -> None:
        self._run_dxl(
            "write_register",
            lambda c: c.dxl_write_register(
                servo_id, address, length, value, is_eeprom=is_eeprom
            ),
        )

    # --- command plumbing -------------------------------------------------

    def _run_control(self, kind: str, call) -> None:
        client = self._client
        if client is None:
            self.error.emit(f"{kind}: not connected")
            return

        def worker() -> None:
            res = call(client)
            if res is None:
                self.error.emit(f"{kind}: no response")
            else:
                self.control_result.emit(kind, res)

        threading.Thread(target=worker, daemon=True).start()

    def _run_motion(self, kind: str, call) -> None:
        client = self._client
        if client is None:
            self.error.emit(f"{kind}: not connected")
            return

        def worker() -> None:
            res = call(client)
            if res is None:
                self.error.emit(f"{kind}: no response")
            else:
                self.motion_result.emit(kind, res)

        threading.Thread(target=worker, daemon=True).start()

    def _run_motion_sequence(self, commands) -> None:
        """Run dependent motion commands serially and stop on first rejection."""
        client = self._client
        if client is None:
            self.error.emit("motion: not connected")
            return

        def worker() -> None:
            for kind, call in commands:
                res = call(client)
                if res is None:
                    self.error.emit(f"{kind}: no response")
                    return
                self.motion_result.emit(kind, res)
                if not res.ok:
                    return

        threading.Thread(
            target=worker, name="hexapod-motion-sequence", daemon=True
        ).start()

    def _run_in_worker(self, call) -> None:
        client = self._client
        if client is None:
            return
        threading.Thread(target=lambda: client and call(client), daemon=True).start()

    # --- internal ---------------------------------------------------------

    def _on_connected_changed(self, up: bool) -> None:
        """Start/stop the status poll (GUI thread; see __init__ note)."""
        if up:
            self._poll.start()
            self._poll_status()  # first poll immediately, not after 1 s
            self._restore_stream_leases()
        else:
            self._poll.stop()
            self._effective_stream_rates.clear()

    def _poll_status(self) -> None:
        client = self._client
        if client is None:
            return

        def worker() -> None:
            st = client.get_status()
            if st is not None:
                self.status_received.emit(st)

        threading.Thread(target=worker, daemon=True).start()

    def _on_telemetry(self, stream_id: int, record: object, header) -> None:
        self.telemetry.emit(stream_id, record)

    def _on_connection(
        self, value: bool, client: ProtocolClient | None = None
    ) -> None:
        if value:
            return
        if client is not None and self._client is not client:
            return
        self._drop_maint_lock()
        self.connecting.emit(False)
        self._client = None
        self._reset_state()
        self._set_simulation_mode(False)
        if not self._connecting:
            self.connected.emit(False)
