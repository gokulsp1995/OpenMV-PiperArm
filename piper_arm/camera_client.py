#!/usr/bin/env python3
"""TCP client for communicating with the OpenMV AE3 camera server.

Connects to the camera over WiFi and sends JSON commands / receives JSON
responses.  Used by the arm controller to trigger scans and read results.
"""

import json
import socket
import time
import logging

logger = logging.getLogger(__name__)


class CameraClient:
    """TCP client that connects to the OpenMV camera server."""

    def __init__(self, host: str, port: int = 8470, timeout: float = 5.0):
        """
        Args:
            host: IP address of the OpenMV camera on the WiFi network.
            port: TCP port the camera is listening on.
            timeout: Socket timeout in seconds.
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: socket.socket | None = None

    # ── Connection Management ─────────────────────────────────────────────────

    def connect(self) -> bool:
        """Establish a TCP connection to the camera.  Returns True on success."""
        self.disconnect()
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(self.timeout)
            self._sock.connect((self.host, self.port))
            logger.info("Connected to camera at %s:%d", self.host, self.port)
            return True
        except Exception as e:
            logger.error("Camera connection failed: %s", e)
            self._sock = None
            return False

    def disconnect(self):
        """Close the TCP connection."""
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    # ── Commands ──────────────────────────────────────────────────────────────

    def scan(self, panel: str | None = None) -> dict | None:
        """Send a SCAN command and return the detection result dict.

        Args:
            panel: Optional panel layout name ("panel1" or "panel2").
                   When provided, the camera uses panel geometry for
                   position-based button identification as a fallback.

        Returns None on communication failure.
        """
        cmd = {"cmd": "SCAN"}
        if panel is not None:
            cmd["panel"] = panel
        return self._send_command(cmd)

    def status(self) -> dict | None:
        """Send a STATUS command and return the status dict."""
        return self._send_command({"cmd": "STATUS"})

    # ── Internal ──────────────────────────────────────────────────────────────

    def _send_command(self, cmd: dict) -> dict | None:
        """Send a JSON command and receive the JSON response."""
        if not self.connected:
            if not self.connect():
                return None

        try:
            # Send
            data = json.dumps(cmd) + "\n"
            self._sock.sendall(data.encode("utf-8"))

            # Receive (newline-delimited JSON)
            response = self._recv_line()
            if response is None:
                logger.warning("No response from camera")
                return None

            return json.loads(response)

        except json.JSONDecodeError as e:
            logger.error("Invalid JSON from camera: %s", e)
            return None
        except socket.timeout:
            logger.warning("Camera response timeout")
            return None
        except Exception as e:
            logger.error("Camera communication error: %s", e)
            self.disconnect()
            return None

    def _recv_line(self) -> str | None:
        """Receive a newline-terminated line from the socket."""
        buf = b""
        while True:
            try:
                ch = self._sock.recv(1)
                if not ch:
                    return None  # disconnected
                if ch == b"\n":
                    break
                buf += ch
                if len(buf) > 8192:  # safety limit
                    break
            except socket.timeout:
                return None
        return buf.decode("utf-8").strip()

    # ── Context Manager ───────────────────────────────────────────────────────

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
