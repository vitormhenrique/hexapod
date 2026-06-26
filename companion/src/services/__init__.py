"""Qt bridge over the threaded :class:`ProtocolClient`.

Lives between the UI and the transport. Telemetry/connection callbacks fire on
the reader thread; this object re-emits them as Qt signals (queued to the GUI
thread) so widgets never touch the serial thread directly. The UI thread is
never blocked: connect/handshake run in a worker thread.
"""

from __future__ import annotations

import threading
from typing import Optional

from PySide6.QtCore import QObject, QTimer, Signal

from hexapod_protocol import api, telemetry as tlm

from transport import list_serial_ports, open_serial
from transport.protocol_client import ProtocolClient


class ConnectionService(QObject):
    connected = Signal(bool)
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
    dxl_result = Signal(str, object)  # kind, api.DxlJobResult (None on failure)
    sensor_feature_result = Signal(str, object)  # kind, api.SensorFeatureResult
    contact_threshold_result = Signal(object)  # api.ContactThresholdResult
    sensor_calibrate_result = Signal(object)  # api.SensorCalibrateResult
    passive_result = Signal(str, object)  # kind, api.PassiveResult
    passive_rate_result = Signal(object)  # api.PassiveRateResult

    def __init__(self) -> None:
        super().__init__()
        self._client: Optional[ProtocolClient] = None
        self._poll = QTimer(self)
        self._poll.setInterval(1000)
        self._poll.timeout.connect(self._poll_status)
        self._last_state: Optional[int] = None
        self._maint_token: int = 0

    # --- discovery --------------------------------------------------------

    def available_ports(self) -> list:
        return list_serial_ports()

    # --- lifecycle --------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._client.connected

    def connect_to(self, port: str, baud: int = 115200) -> None:
        """Open the port and handshake in a worker thread (non-blocking)."""
        if self.is_connected:
            self.disconnect()

        def worker() -> None:
            link = open_serial(port, baud=baud)
            if link is None:
                self.error.emit(f"Could not open {port}")
                self.connected.emit(False)
                return
            client = ProtocolClient(link)
            client.on_telemetry(self._on_telemetry)
            client.on_connection(self._on_connection)
            client.start()
            self._client = client
            hello = client.hello()
            if hello is not None:
                self.hello_received.emit(hello)
                self.event.emit(
                    "connect",
                    f"{hello.device_name} fw "
                    f"{hello.fw_major}.{hello.fw_minor}.{hello.fw_patch}",
                )
            caps = client.get_capabilities()
            if caps is not None:
                self.capabilities_received.emit(caps)
            self.connected.emit(True)
            # Status polling must be started on the GUI thread (QTimer affinity).
            QTimer.singleShot(0, self._poll.start)

        threading.Thread(target=worker, name="hexapod-connect", daemon=True).start()

    def disconnect(self) -> None:
        self._poll.stop()
        if self._client is not None:
            self._client.stop()
            self._client = None
        self.connected.emit(False)
        self.event.emit("disconnect", "link closed")

    # --- commands (safe no-ops when disconnected) ------------------------

    def subscribe(self, stream_id: int, rate_hz: int) -> None:
        if self._client:
            threading.Thread(
                target=lambda: self._client
                and self._client.subscribe(stream_id, rate_hz),
                daemon=True,
            ).start()

    def unsubscribe(self, stream_id: int) -> None:
        if self._client:
            threading.Thread(
                target=lambda: self._client and self._client.unsubscribe(stream_id),
                daemon=True,
            ).start()

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
        self._run_control(
            "arm" if arm else "disarm", lambda c: c.set_arming(arm)
        )

    def set_mode(self, mode: int) -> None:
        self._run_control("set_mode", lambda c: c.set_mode(mode))

    def set_gait(self, gait: int) -> None:
        self._run_motion("set_gait", lambda c: c.set_gait(gait))

    def stop_motion(self) -> None:
        self._run_motion("stop_motion", lambda c: c.stop_motion())

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
        self._run_passive("enter", lambda c: c.passive_enter())

    def passive_exit(self) -> None:
        self._run_passive("exit", lambda c: c.passive_exit())

    def passive_zero_reference(self) -> None:
        self._run_passive("zero", lambda c: c.passive_zero_reference())

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
        """Acquire the maintenance lock; cache the returned token on success."""
        client = self._client
        if client is None:
            self.error.emit("maintenance: not connected")
            return

        def worker() -> None:
            res = client.enter_maintenance()
            if res is None:
                self.error.emit("maintenance: no response")
                return
            if res.ok:
                self._maint_token = res.token
                self.event.emit(
                    "commit", f"maintenance lock acquired (token {res.token})"
                )
            else:
                self.event.emit("error", "maintenance lock rejected")
            self.maint_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    def exit_maintenance(self) -> None:
        """Release the cached maintenance lock token."""
        client = self._client
        if client is None:
            self.error.emit("maintenance: not connected")
            return
        token = self._maint_token

        def worker() -> None:
            res = client.exit_maintenance(token)
            if res is None:
                self.error.emit("maintenance: no response")
                return
            if res.ok:
                self._maint_token = 0
                self.event.emit("commit", "maintenance lock released")
            self.maint_result.emit(res)

        threading.Thread(target=worker, daemon=True).start()

    @property
    def client(self) -> Optional[ProtocolClient]:
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

    def dxl_get_param(self, servo_id: int, param: int) -> None:
        self._run_dxl("get_param", lambda c: c.dxl_get_param(servo_id, param))

    def dxl_set_param(self, servo_id: int, param: int, value: int) -> None:
        self._run_dxl("set_param", lambda c: c.dxl_set_param(servo_id, param, value))

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

    def _run_in_worker(self, call) -> None:
        client = self._client
        if client is None:
            return
        threading.Thread(
            target=lambda: client and call(client), daemon=True
        ).start()

    # --- internal ---------------------------------------------------------

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

    def _on_connection(self, value: bool) -> None:
        if not value:
            self.connected.emit(False)
