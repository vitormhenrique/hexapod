"""Single-client, authenticated TCP relay for a Jetson-owned MCU USB link."""

from __future__ import annotations

import argparse
import hmac
import secrets
import socket
import threading
from typing import Optional, Sequence

from transport import ByteStream, open_serial
from transport.tcp_proxy import (
    RELAY_BUSY,
    RELAY_OK,
    RELAY_UNAUTHORIZED,
    parse_relay_auth,
    receive_relay_line,
    validate_relay_token,
)


class TcpRelayServer:
    """Own one USB stream and expose it to one authenticated Mac TCP client.

    After its authentication line, the relay copies bytes in both directions
    without decoding or modifying the MCU protocol. This intentionally cannot
    share a USB stream with ``JetsonBridge``: only one reader may own MCU
    responses at a time.
    """

    def __init__(
        self,
        stream: ByteStream,
        token: str,
        *,
        host: str = "127.0.0.1",
        port: int = 5555,
        client_timeout_s: float = 0.1,
    ) -> None:
        validate_relay_token(token)
        if not 0 <= port <= 65535:
            raise ValueError("relay port must be in range 0..65535")
        if client_timeout_s <= 0:
            raise ValueError("client_timeout_s must be positive")

        self._stream = stream
        self._token = token
        self._host = host
        self._port = port
        self._client_timeout_s = client_timeout_s
        self._stop = threading.Event()
        self._state_lock = threading.Lock()
        self._listener: Optional[socket.socket] = None
        self._client: Optional[socket.socket] = None
        self._accept_worker: Optional[threading.Thread] = None
        self._session_worker: Optional[threading.Thread] = None

    @property
    def port(self) -> int:
        """The active listening port, including an OS-selected port zero value."""
        with self._state_lock:
            listener = self._listener
        if listener is None:
            return self._port
        return int(listener.getsockname()[1])

    @property
    def client_connected(self) -> bool:
        """Whether an authenticated Mac connection currently owns the relay."""
        with self._state_lock:
            return self._client is not None

    def start(self) -> None:
        """Bind the TCP listener and begin accepting a single proxy client."""
        with self._state_lock:
            if self._listener is not None:
                return
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((self._host, self._port))
            listener.listen(2)
            listener.settimeout(0.1)
            self._listener = listener
            self._stop.clear()
            self._accept_worker = threading.Thread(
                target=self._accept_loop,
                name="hexapod-jetson-relay-accept",
                daemon=True,
            )
            self._accept_worker.start()

    def close(self) -> None:
        """Stop the listener, disconnect the Mac, and release the owned USB stream."""
        self._stop.set()
        with self._state_lock:
            listener = self._listener
            client = self._client
            accept_worker = self._accept_worker
            session_worker = self._session_worker
            self._listener = None
            self._client = None
        for connection in (listener, client):
            if connection is None:
                continue
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        if accept_worker is not None and accept_worker is not threading.current_thread():
            accept_worker.join(timeout=1.0)
        if session_worker is not None and session_worker is not threading.current_thread():
            session_worker.join(timeout=1.0)
        self._stream.close()

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            with self._state_lock:
                listener = self._listener
            if listener is None:
                return
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self._authenticate_or_reject(connection)

    def _authenticate_or_reject(self, connection: socket.socket) -> None:
        try:
            connection.settimeout(1.0)
            line, initial_data = receive_relay_line(connection)
            if not hmac.compare_digest(parse_relay_auth(line), self._token):
                connection.sendall(RELAY_UNAUTHORIZED + b"\n")
                connection.close()
                return
            with self._state_lock:
                if self._client is not None:
                    connection.sendall(RELAY_BUSY + b"\n")
                    connection.close()
                    return
                self._client = connection
            connection.sendall(RELAY_OK + b"\n")
            connection.settimeout(self._client_timeout_s)
            worker = threading.Thread(
                target=self._serve_client,
                args=(connection, initial_data),
                name="hexapod-jetson-relay-client",
                daemon=True,
            )
            with self._state_lock:
                self._session_worker = worker
            worker.start()
        except (ConnectionError, OSError, ValueError, socket.timeout):
            try:
                connection.close()
            except OSError:
                pass

    def _serve_client(self, connection: socket.socket, initial_data: bytes) -> None:
        finished = threading.Event()

        def client_to_stream() -> None:
            try:
                if initial_data:
                    self._stream.write(initial_data)
                while not self._stop.is_set() and not finished.is_set():
                    try:
                        chunk = connection.recv(512)
                    except socket.timeout:
                        continue
                    if not chunk:
                        return
                    self._stream.write(chunk)
            except OSError:
                pass
            finally:
                finished.set()

        inbound_worker = threading.Thread(
            target=client_to_stream,
            name="hexapod-jetson-relay-inbound",
            daemon=True,
        )
        inbound_worker.start()
        try:
            while not self._stop.is_set() and not finished.is_set():
                waiting = getattr(self._stream, "in_waiting", 0) or 0
                chunk = self._stream.read(max(1, min(waiting, 512)))
                if chunk:
                    connection.sendall(chunk)
                else:
                    finished.wait(0.005)
        except OSError:
            pass
        finally:
            finished.set()
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
            inbound_worker.join(timeout=1.0)
            with self._state_lock:
                if self._client is connection:
                    self._client = None
                    self._session_worker = None


def run_relay(
    serial_port: str,
    *,
    baud: int = 115200,
    host: str = "127.0.0.1",
    port: int = 5555,
    token: str | None = None,
) -> None:
    """Open the MCU USB port and serve it until the operator interrupts us."""
    stream = open_serial(serial_port, baud=baud)
    if stream is None:
        raise RuntimeError(f"could not open OpenRB-150 serial port {serial_port}")
    relay_token = token or secrets.token_urlsafe(32)
    server = TcpRelayServer(stream, relay_token, host=host, port=port)
    server.start()
    print(
        f"Jetson relay listening on {host}:{server.port}; "
        f"connect with tcp://<jetson-host>:{server.port}?token={relay_token}",
        flush=True,
    )
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


def main(argv: Sequence[str] | None = None) -> None:
    """Run the Jetson relay command-line entry point."""
    parser = argparse.ArgumentParser(
        description="Expose a Jetson-owned OpenRB-150 USB link to one Mac client."
    )
    parser.add_argument("--serial-port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--token")
    args = parser.parse_args(argv)
    run_relay(
        args.serial_port,
        baud=args.baud,
        host=args.host,
        port=args.port,
        token=args.token,
    )


if __name__ == "__main__":
    main()