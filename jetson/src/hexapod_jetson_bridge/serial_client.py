"""Jetson-facing high-level motion bridge backed by the shared protocol client.

The bridge never grants authority itself. It periodically refreshes the
firmware's distinct Jetson liveness signal and forwards only high-level motion
intent. The OpenRB-150 still validates feature availability, RC approval,
safety state, gait timing, IK, and every final actuator command.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Mapping, Optional

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm

from transport import open_serial
from transport.protocol_client import ProtocolClient, TelemetryCallback


@dataclass(frozen=True)
class JetsonHeartbeat:
    """The firmware state echoed by one successful Jetson heartbeat."""

    uptime_ms: int
    state: int


class JetsonBridge:
    """Forward safe Jetson autonomy intent through :class:`ProtocolClient`.

    A local freshness check prevents this process from continuing to send
    motion after its heartbeat worker is stale. It is an extra guard only;
    firmware remains the source of truth for all authority and actuator safety.
    """

    DEFAULT_HEARTBEAT_INTERVAL_S = 0.10
    DEFAULT_HEARTBEAT_TIMEOUT_S = 0.25
    DEFAULT_TELEMETRY_RATES = {
        int(tlm.StreamId.HEALTH): 1,
        int(tlm.StreamId.CONTROL_STATE): 10,
    }

    def __init__(
        self,
        protocol_client: ProtocolClient,
        *,
        heartbeat_interval_s: float = DEFAULT_HEARTBEAT_INTERVAL_S,
        heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S,
        telemetry_rates: Optional[Mapping[int, int]] = None,
    ) -> None:
        if heartbeat_interval_s <= 0:
            raise ValueError("heartbeat_interval_s must be positive")
        if heartbeat_timeout_s <= heartbeat_interval_s:
            raise ValueError(
                "heartbeat_timeout_s must be longer than heartbeat_interval_s"
            )

        rates = self.DEFAULT_TELEMETRY_RATES if telemetry_rates is None else telemetry_rates
        if any(rate_hz <= 0 for rate_hz in rates.values()):
            raise ValueError("telemetry subscription rates must be positive")

        self._client = protocol_client
        self._heartbeat_interval_s = float(heartbeat_interval_s)
        self._heartbeat_timeout_s = float(heartbeat_timeout_s)
        self._telemetry_rates = dict(rates)
        self._heartbeat_stop = threading.Event()
        self._heartbeat_worker: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._hello: Optional[api.HelloInfo] = None
        self._last_heartbeat: Optional[JetsonHeartbeat] = None
        self._last_heartbeat_at: Optional[float] = None
        self._started = False
        self._closed = False

    @classmethod
    def connect(
        cls,
        port: str,
        *,
        baud: int = 115200,
        response_timeout: float = 1.0,
        heartbeat_interval_s: float = DEFAULT_HEARTBEAT_INTERVAL_S,
        heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S,
        telemetry_rates: Optional[Mapping[int, int]] = None,
    ) -> Optional["JetsonBridge"]:
        """Open a serial link, perform the bridge handshake, or return ``None``."""
        stream = open_serial(port, baud)
        if stream is None:
            return None
        bridge = cls(
            ProtocolClient(stream, response_timeout=response_timeout),
            heartbeat_interval_s=heartbeat_interval_s,
            heartbeat_timeout_s=heartbeat_timeout_s,
            telemetry_rates=telemetry_rates,
        )
        if bridge.start() is not None:
            return bridge
        bridge.close()
        return None

    @property
    def connected(self) -> bool:
        """Whether the shared protocol client currently has a live transport."""
        return self._client.connected

    @property
    def hello_info(self) -> Optional[api.HelloInfo]:
        """The successful startup handshake, if one has completed."""
        with self._lock:
            return self._hello

    @property
    def last_heartbeat(self) -> Optional[JetsonHeartbeat]:
        """The latest successful Jetson liveness response."""
        with self._lock:
            return self._last_heartbeat

    @property
    def heartbeat_fresh(self) -> bool:
        """Whether a bridge heartbeat succeeded within the local guard window."""
        with self._lock:
            last = self._last_heartbeat_at
        return (
            self._client.connected
            and last is not None
            and time.monotonic() - last <= self._heartbeat_timeout_s
        )

    def start(self) -> Optional[api.HelloInfo]:
        """Handshake, subscribe to baseline telemetry, and start liveness I/O."""
        with self._lock:
            if self._closed:
                raise RuntimeError("cannot start a closed JetsonBridge")
            if self._started:
                return self._hello

        self._client.start()
        hello = self._client.hello()
        if hello is None or self.heartbeat() is None:
            return None

        # The first successful subscription starts ProtocolClient's background
        # reader after its serial CDC handshake path has completed.
        for stream_id, rate_hz in self._telemetry_rates.items():
            result = self._client.subscribe(stream_id, rate_hz)
            if result is None or not result.ok:
                return None

        with self._lock:
            self._hello = hello
            self._started = True
            self._heartbeat_stop.clear()
            worker = threading.Thread(
                target=self._heartbeat_loop,
                name="hexapod-jetson-heartbeat",
                daemon=True,
            )
            self._heartbeat_worker = worker
            worker.start()
        return hello

    def close(self) -> None:
        """Stop liveness work and close the shared serial transport."""
        with self._lock:
            worker = self._heartbeat_worker
            self._heartbeat_worker = None
            self._started = False
            self._closed = True
            self._heartbeat_stop.set()
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=1.0)
        self._client.stop()

    def heartbeat(self) -> Optional[JetsonHeartbeat]:
        """Refresh firmware's Jetson-only liveness timestamp once."""
        response = self._client.jetson_heartbeat()
        if response is None:
            return None
        heartbeat = JetsonHeartbeat(uptime_ms=response[0], state=response[1])
        with self._lock:
            self._last_heartbeat = heartbeat
            self._last_heartbeat_at = time.monotonic()
        return heartbeat

    def on_telemetry(self, callback: TelemetryCallback) -> None:
        """Register a callback for decoded firmware telemetry records."""
        self._client.on_telemetry(callback)

    def subscribe(self, stream_id: int, rate_hz: int) -> Optional[api.SubscribeResult]:
        """Request one additional firmware telemetry stream at a bounded rate."""
        return self._client.subscribe(stream_id, rate_hz)

    def unsubscribe(self, stream_id: int) -> Optional[api.SubscribeResult]:
        """Stop one firmware telemetry stream."""
        return self._client.unsubscribe(stream_id)

    def set_gait(self, gait: int) -> Optional[api.MotionResultMsg]:
        """Forward a high-level gait selection while liveness remains fresh."""
        if not self._motion_ready():
            return None
        return self._client.set_gait(gait)

    def set_gait_params(
        self,
        body_height_mm: int,
        stride_len_mm: int,
        step_height_mm: int,
        duty_x255: int,
        speed_x255: int,
    ) -> Optional[api.MotionResultMsg]:
        """Forward bounded gait hints while liveness remains fresh."""
        if not self._motion_ready():
            return None
        return self._client.set_gait_params(
            body_height_mm,
            stride_len_mm,
            step_height_mm,
            duty_x255,
            speed_x255,
        )

    def set_body_twist(
        self, vx: float, vy: float, wz: float
    ) -> Optional[api.MotionResultMsg]:
        """Forward normalized forward, lateral, and yaw motion intent."""
        if not self._motion_ready():
            return None
        return self._client.set_body_twist(vx, vy, wz)

    def set_body_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> Optional[api.MotionResultMsg]:
        """Forward a bounded body-pose request while liveness remains fresh."""
        if not self._motion_ready():
            return None
        return self._client.set_body_pose(
            x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg
        )

    def stop_motion(self) -> Optional[api.MotionResultMsg]:
        """Forward a stop even if the bridge's liveness guard is stale."""
        return self._client.stop_motion()

    def _heartbeat_loop(self) -> None:
        while not self._heartbeat_stop.wait(self._heartbeat_interval_s):
            self.heartbeat()

    def _motion_ready(self) -> bool:
        with self._lock:
            started = self._started
        return started and self.heartbeat_fresh