#!/usr/bin/env python3
"""Serial client for communicating with the OpenMV AE3 camera server.

Connects to the camera over USB serial channel and sends JSON commands / receives JSON
responses.  Used by the arm controller to trigger scans and read results.

connect, disconnect, connected, scan, status is
unchanged from the WiFi version
"""

import json
import glob
import time
import logging

import serial 
logger = logging.getLogger(__name__)


class CameraClient:
    """Serial client that connects to the OpenMV camera server."""

    def __init__(self, port_hint: str = "OpenMV", baudrate: int = 115200,
                 timeout: float = 5.0):
        """
        Args:
            port_hint: substring to match against /dev/serial/by-id/
                       entries to find the camera. Replaces host/port from
                       the WiFi version -- there's no IP/port to configure
                       for a wired link, but there IS a device to find,
                       since /dev/ttyACM* numbering isn't stable across
                       reboots/replugs.
            baudrate: ignored by USB CDC in practice, kept because
                      pyserial's API still requires a value.
            timeout: read timeout in seconds, passed to pyserial.
        """
        self.port_hint = port_hint
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: serial.Serial | None = None

    # ── Connection Management ─────────────────────────────────────────────────

    def connect(self) -> bool:
        """Open the serial port to the camera. Returns True on success."""
        self.disconnect()
        try:
            device = self._resolve_port()
            self._ser = serial.Serial(device, self.baudrate,
                                    timeout=self.timeout, dsrdtr=False)
            self._ser.reset_input_buffer()
            logger.info("Connected to camera at %s", device)
            return True
        except Exception as e:
            logger.error("Camera connection failed: %s", e)
            self._ser = None
            return False

    def disconnect(self):
        """Close the serial port."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def _resolve_port(self) -> str:
        """Find the camera's device path via /dev/serial/by-id/."""
        candidates = glob.glob("/dev/serial/by-id/*")
        for path in candidates:
            if self.port_hint.lower() in path.lower():
                return path
        raise RuntimeError(
            f"No /dev/serial/by-id/ entry matched '{self.port_hint}'. "
            f"Found: {candidates}"
        )


    

    # ── Commands ──────────────────────────────────────────────────────────────

    def scan(self, panel: str | None = None) -> dict | None:
        
        cmd = {"cmd": "SCAN"}
        if panel is not None:
            cmd["panel"] = panel
        return self._send_command(cmd)

    def status(self) -> dict | None:
        return self._send_command({"cmd": "STATUS"})

    # ── Internal ──────────────────────────────────────────────────────────────

    def _send_command(self, cmd: dict) -> dict | None:
        """Send a JSON command and receive the JSON response."""
        if not self.connected:
            if not self.connect():
                return None

        try:
            data = json.dumps(cmd) + "\n"
            self._ser.write(data.encode("utf-8"))

            # Receive (newline-delimited JSON)
            response = self._recv_line()
            if response is None:
                logger.warning("No response from camera")
                return None

            return json.loads(response)

        except json.JSONDecodeError as e:
            logger.error("Invalid JSON from camera: %s", e)
            return None
        except Exception as e:
            logger.error("Camera communication error: %s", e)
            self.disconnect()
            return None

    def _recv_line(self) -> str | None:
        """Receive a newline-terminated line from the serial port."""
        buf = b""
        deadline = time.time() + self.timeout
        while time.time() < deadline:
                ch = self._ser.read(1)
                if not ch:
                    continue
                if ch == b"\n":
                    return buf.decode("utf-8").strip()
                buf += ch
                if len(buf) > 8192:  # safety limit
                    break
        return None

    # ── Context Manager ───────────────────────────────────────────────────────

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
