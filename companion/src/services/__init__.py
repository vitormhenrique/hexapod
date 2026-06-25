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

from ..transport import list_serial_ports, open_serial
from ..transport.protocol_client import ProtocolClient


class ConnectionService(QObject):
    connected = Signal(bool)
    hello_received = Signal(object)  # api.HelloInfo
    status_received = Signal(object)  # api.StatusInfo
    capabilities_received = Signal(object)  # api.Capabilities
    telemetry = Signal(int, object)  # stream_id, decoded record
    event = Signal(str, str)  # kind, detail
    error = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._client: Optional[ProtocolClient] = None
        self._poll = QTimer(self)
        self._poll.setInterval(1000)
        self._poll.timeout.connect(self._poll_status)
        self._last_state: Optional[int] = None

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
                self.event.emit("connect", f"{hello.device_name} fw "
                                f"{hello.fw_major}.{hello.fw_minor}.{hello.fw_patch}")
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
                target=lambda: self._client and self._client.subscribe(stream_id, rate_hz),
                daemon=True,
            ).start()

    def unsubscribe(self, stream_id: int) -> None:
        if self._client:
            threading.Thread(
                target=lambda: self._client and self._client.unsubscribe(stream_id),
                daemon=True,
            ).start()

    def emergency_stop(self) -> None:
        """Best-effort estop. Wired to a real ESTOP msg-id when firmware adds it."""
        self.event.emit("estop", "operator pressed EMERGENCY STOP")
        # TODO: send api.MSG_ESTOP once defined in the firmware protocol.

    @property
    def client(self) -> Optional[ProtocolClient]:
        return self._client

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
