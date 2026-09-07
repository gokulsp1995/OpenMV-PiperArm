#!/usr/bin/env python3
"""OpenMV RL Data Stream Receiver + Browser Server.

Receives JPEG video (UDP) and sensor data (TCP) from the OpenMV AE3 camera,
then serves a browser dashboard via HTTP + WebSocket for live visualisation.

Usage:
    python receiver.py [--camera-ip AUTO] [--record]

Architecture:
    UDP 8471 ← chunked JPEG frames from camera (broadcast)
    TCP 8472 → sensor data stream from camera + control commands to camera
    HTTP 8080 → serves index.html (browser dashboard)
    WS   8080 → pushes frames + sensor data to browser in real-time
"""

import asyncio
import argparse
import json
import logging
import os
import struct
import sys
import time
from pathlib import Path
from datetime import datetime

try:
    import aiohttp
    from aiohttp import web
except ImportError:
    print("ERROR: aiohttp not installed. Run: pip install aiohttp")
    sys.exit(1)

try:
    import websockets
except ImportError:
    websockets = None  # We'll use aiohttp's built-in WebSocket support instead

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
logger = logging.getLogger("receiver")

# ── Constants ─────────────────────────────────────────────────────────────────

UDP_VIDEO_PORT = 8471
TCP_SENSOR_PORT = 8472
HTTP_PORT = 8080

UDP_MAGIC = b"\x4f\x4d\x56\x53"  # "OMVS"
HEADER_SIZE = 16  # 4 + 4 + 4 + 2 + 2

STATIC_DIR = Path(__file__).parent


# ── Frame Reassembler ─────────────────────────────────────────────────────────

class FrameAssembler:
    """Reassembles chunked UDP packets into complete JPEG frames."""

    def __init__(self):
        self._pending = {}  # frame_id → {chunks: {idx: data}, count: int, ts: int}
        self._last_complete_id = -1

    def add_chunk(self, data: bytes):
        """Process a raw UDP packet. Returns (frame_id, timestamp_ms, jpeg_bytes)
        when a frame is complete, or None otherwise."""
        if len(data) < HEADER_SIZE:
            return None

        magic = data[:4]
        if magic != UDP_MAGIC:
            return None

        frame_id, timestamp_ms, chunk_idx, chunk_count = struct.unpack(
            "<IIhh", data[4:HEADER_SIZE]
        )
        payload = data[HEADER_SIZE:]

        # Skip frames older than the last completed one
        if frame_id <= self._last_complete_id:
            return None

        # Store chunk
        if frame_id not in self._pending:
            self._pending[frame_id] = {
                "chunks": {},
                "count": chunk_count,
                "ts": timestamp_ms,
            }

        entry = self._pending[frame_id]
        entry["chunks"][chunk_idx] = payload

        # Check if complete
        if len(entry["chunks"]) >= entry["count"]:
            # Assemble in order
            jpeg = b"".join(entry["chunks"][i] for i in range(entry["count"]))
            self._last_complete_id = frame_id

            # Clean up old pending frames
            stale = [fid for fid in self._pending if fid <= frame_id]
            for fid in stale:
                del self._pending[fid]

            return (frame_id, entry["ts"], jpeg)

        return None


# ── Recorder ──────────────────────────────────────────────────────────────────

class DataRecorder:
    """Records JPEG frames + sensor metadata to disk for RL training."""

    def __init__(self, output_dir: str = None):
        self._active = False
        self._dir = None
        self._meta_file = None
        self._frame_count = 0

    def start(self, output_dir: str = None):
        if self._active:
            return
        if output_dir is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = str(Path.cwd() / "recordings" / f"session_{ts}")
        self._dir = Path(output_dir)
        (self._dir / "frames").mkdir(parents=True, exist_ok=True)
        self._meta_file = open(self._dir / "metadata.jsonl", "w")
        self._frame_count = 0
        self._active = True
        logger.info("Recording started → %s", self._dir)

    def stop(self):
        if not self._active:
            return
        self._active = False
        if self._meta_file:
            self._meta_file.close()
            self._meta_file = None
        logger.info("Recording stopped (%d frames saved)", self._frame_count)

    def save_frame(self, frame_id: int, jpeg: bytes, sensor_data: dict):
        if not self._active:
            return
        # Save JPEG
        fname = f"frame_{frame_id:06d}.jpg"
        with open(self._dir / "frames" / fname, "wb") as f:
            f.write(jpeg)

        # Save metadata line
        meta = {
            "frame_id": frame_id,
            "filename": fname,
            "timestamp_ms": sensor_data.get("timestamp_ms", 0),
            "imu": sensor_data.get("imu", {}),
            "tof": sensor_data.get("tof", []),
            "apriltags": sensor_data.get("apriltags", []),
        }
        self._meta_file.write(json.dumps(meta) + "\n")
        self._meta_file.flush()
        self._frame_count += 1

    @property
    def active(self):
        return self._active

    @property
    def frame_count(self):
        return self._frame_count


# ── Server State ──────────────────────────────────────────────────────────────

class StreamState:
    """Shared state between all server components."""

    def __init__(self):
        self.latest_jpeg: bytes = b""
        self.latest_sensor: dict = {}
        self.camera_ip: str = ""
        self.connected: bool = False
        self.fps: float = 0.0
        self.frame_count: int = 0
        self.ws_clients: set = set()
        self.recorder = DataRecorder()
        self.tcp_writer: asyncio.StreamWriter = None

        # FPS tracking
        self._fps_frames = 0
        self._fps_time = time.monotonic()


state = StreamState()


# ── UDP Video Receiver ────────────────────────────────────────────────────────

class UDPVideoProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.assembler = FrameAssembler()

    def datagram_received(self, data, addr):
        result = self.assembler.add_chunk(data)
        if result is None:
            return

        frame_id, timestamp_ms, jpeg = result

        # Update state
        state.latest_jpeg = jpeg
        state.frame_count = frame_id
        state.camera_ip = addr[0]
        state.connected = True

        # Track FPS
        state._fps_frames += 1
        now = time.monotonic()
        elapsed = now - state._fps_time
        if elapsed >= 2.0:
            state.fps = round(state._fps_frames / elapsed, 1)
            state._fps_frames = 0
            state._fps_time = now

        # Push to WebSocket clients
        asyncio.ensure_future(_broadcast_frame(jpeg))

        # Record if active
        if state.recorder.active:
            state.recorder.save_frame(frame_id, jpeg, state.latest_sensor)


async def _broadcast_frame(jpeg: bytes):
    """Send JPEG frame to all connected WebSocket clients as binary."""
    dead = set()
    for ws in state.ws_clients:
        try:
            await ws.send_bytes(jpeg)
        except Exception:
            dead.add(ws)
    state.ws_clients -= dead


# ── TCP Sensor Receiver ──────────────────────────────────────────────────────

async def tcp_sensor_client(camera_ip: str):
    """Connect to the camera's TCP sensor port and receive JSON data."""
    while True:
        try:
            logger.info("Connecting to camera TCP %s:%d ...", camera_ip, TCP_SENSOR_PORT)
            reader, writer = await asyncio.open_connection(camera_ip, TCP_SENSOR_PORT)
            state.tcp_writer = writer
            logger.info("TCP sensor stream connected")

            buffer = b""
            while True:
                data = await reader.read(4096)
                if not data:
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8"))
                        if msg.get("type") == "sensor":
                            state.latest_sensor = msg
                            # Push sensor data to WS clients as JSON text
                            await _broadcast_sensor(msg)
                    except (json.JSONDecodeError, UnicodeDecodeError):
                        pass

        except (ConnectionRefusedError, OSError) as e:
            logger.warning("TCP connection failed: %s — retrying in 2s", e)
        except asyncio.CancelledError:
            break
        finally:
            state.tcp_writer = None

        await asyncio.sleep(2)


async def _broadcast_sensor(sensor_data: dict):
    """Send sensor JSON to all WS clients as text."""
    text = json.dumps(sensor_data)
    dead = set()
    for ws in state.ws_clients:
        try:
            await ws.send_str(text)
        except Exception:
            dead.add(ws)
    state.ws_clients -= dead


async def send_camera_command(cmd: dict):
    """Send a command to the camera over TCP."""
    if state.tcp_writer is None:
        return False
    try:
        data = json.dumps(cmd) + "\n"
        state.tcp_writer.write(data.encode("utf-8"))
        await state.tcp_writer.drain()
        return True
    except Exception as e:
        logger.error("Failed to send command: %s", e)
        return False


# ── HTTP + WebSocket Server ───────────────────────────────────────────────────

async def handle_index(request):
    """Serve the main dashboard HTML."""
    return web.FileResponse(STATIC_DIR / "index.html")


async def handle_ws(request):
    """WebSocket endpoint for browser clients."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    state.ws_clients.add(ws)
    logger.info("Browser client connected (total: %d)", len(state.ws_clients))

    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                try:
                    cmd = json.loads(msg.data)
                    await _handle_ws_command(cmd, ws)
                except json.JSONDecodeError:
                    pass
            elif msg.type == aiohttp.WSMsgType.ERROR:
                break
    finally:
        state.ws_clients.discard(ws)
        logger.info("Browser client disconnected (total: %d)", len(state.ws_clients))

    return ws


async def _handle_ws_command(cmd: dict, ws):
    """Process commands from the browser UI."""
    action = cmd.get("action", "")

    if action == "apriltag_on":
        await send_camera_command({"cmd": "APRILTAG_ON"})
    elif action == "apriltag_off":
        await send_camera_command({"cmd": "APRILTAG_OFF"})
    elif action == "set_quality":
        await send_camera_command({"cmd": "SET_QUALITY", "value": cmd.get("value", 50)})
    elif action == "record_start":
        state.recorder.start()
        await ws.send_str(json.dumps({"type": "record_status", "active": True}))
    elif action == "record_stop":
        state.recorder.stop()
        await ws.send_str(json.dumps({
            "type": "record_status",
            "active": False,
            "frames_saved": state.recorder.frame_count,
        }))
    elif action == "status":
        await ws.send_str(json.dumps({
            "type": "client_status",
            "camera_ip": state.camera_ip,
            "connected": state.connected,
            "fps": state.fps,
            "frame_count": state.frame_count,
            "recording": state.recorder.active,
            "recording_frames": state.recorder.frame_count,
        }))


async def handle_status_api(request):
    """REST endpoint for status checks."""
    return web.json_response({
        "camera_ip": state.camera_ip,
        "connected": state.connected,
        "fps": state.fps,
        "frame_count": state.frame_count,
        "recording": state.recorder.active,
    })


# ── Auto-Discovery ───────────────────────────────────────────────────────────

async def auto_discover_camera(timeout: float = 30.0):
    """Wait for the first UDP frame to arrive and return the camera IP."""
    logger.info("Auto-discovering camera on UDP port %d (timeout=%ds)...",
                UDP_VIDEO_PORT, timeout)
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        if state.camera_ip:
            logger.info("Camera discovered at %s", state.camera_ip)
            return state.camera_ip
        await asyncio.sleep(0.5)
    logger.warning("Camera auto-discovery timed out")
    return None


# ── Main ──────────────────────────────────────────────────────────────────────

async def main(args):
    loop = asyncio.get_event_loop()

    # Start UDP receiver
    transport, _ = await loop.create_datagram_endpoint(
        UDPVideoProtocol,
        local_addr=("0.0.0.0", UDP_VIDEO_PORT),
    )
    logger.info("UDP video receiver listening on port %d", UDP_VIDEO_PORT)

    # Determine camera IP
    camera_ip = args.camera_ip
    if camera_ip.upper() == "AUTO":
        camera_ip = await auto_discover_camera()
        if camera_ip is None:
            logger.error("Could not discover camera. Specify --camera-ip manually.")
            transport.close()
            return
    else:
        state.camera_ip = camera_ip

    # Start TCP sensor client
    tcp_task = asyncio.create_task(tcp_sensor_client(camera_ip))

    # Start recording if requested
    if args.record:
        state.recorder.start()

    # Set up HTTP + WebSocket server
    app = web.Application()
    app.router.add_get("/", handle_index)
    app.router.add_get("/ws", handle_ws)
    app.router.add_get("/api/status", handle_status_api)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", HTTP_PORT)
    await site.start()

    logger.info("=" * 60)
    logger.info("  Dashboard: http://localhost:%d", HTTP_PORT)
    logger.info("  Camera IP: %s", camera_ip)
    logger.info("  Recording: %s", "ON" if args.record else "OFF (use dashboard to start)")
    logger.info("=" * 60)

    # Run forever
    try:
        while True:
            await asyncio.sleep(3600)
    except asyncio.CancelledError:
        pass
    finally:
        tcp_task.cancel()
        transport.close()
        await runner.cleanup()
        if state.recorder.active:
            state.recorder.stop()


def parse_args():
    parser = argparse.ArgumentParser(description="OpenMV RL Data Stream Receiver")
    parser.add_argument(
        "--camera-ip", default="AUTO",
        help="Camera IP address, or AUTO to discover via UDP broadcast (default: AUTO)",
    )
    parser.add_argument(
        "--record", action="store_true",
        help="Start recording immediately on launch",
    )
    parser.add_argument(
        "--port", type=int, default=HTTP_PORT,
        help="HTTP/WebSocket server port (default: 8080)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    HTTP_PORT = args.port
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        logger.info("Shutting down...")
