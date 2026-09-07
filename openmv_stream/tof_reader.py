# OpenMV AE3 — VL53L8CX 8×8 Time-of-Flight Reader
# =================================================
# Wraps the on-board VL53L8CX sensor via the OpenMV `vl53l8cx` driver.
# Provides distance readings as a flat 64-element list (row-major 8×8).

from machine import I2C
import time

try:
    from vl53l8cx import VL53L8CX
except ImportError:
    # Fallback: the driver may be named differently on some firmware builds.
    VL53L8CX = None

import config


class ToFReader:
    """Reads the VL53L8CX 8×8 ToF sensor and exposes the distance grid."""

    def __init__(self):
        self._sensor = None
        self._data = None  # latest 8×8 distance grid (mm), row-major
        self._valid = False
        self._init_sensor()

    # ── Initialisation ────────────────────────────────────────────────────────

    def _init_sensor(self):
        """Initialise the VL53L8CX over I²C."""
        if VL53L8CX is None:
            print("[ToF] VL53L8CX driver not available — distance will be 0")
            return

        try:
            bus = I2C(config.TOF_I2C_BUS)
            self._sensor = VL53L8CX(bus)
            self._sensor.resolution = 64  # 8×8
            self._sensor.ranging_freq = config.TOF_RANGING_FREQ
            self._sensor.start_ranging()
            self._valid = True
            print("[ToF] VL53L8CX initialised (8×8, %d Hz)" % config.TOF_RANGING_FREQ)
        except Exception as e:
            print("[ToF] Init failed:", e)
            self._valid = False

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self):
        """Poll the sensor for a new reading.  Call once per frame."""
        if not self._valid or self._sensor is None:
            return
        try:
            if self._sensor.data_ready():
                results = self._sensor.get_ranging_data()
                # results.distance_mm is a flat list of 64 values (row-major 8×8)
                self._data = results.distance_mm
        except Exception as e:
            print("[ToF] Read error:", e)

    def get_full_grid(self):
        """Return the full 64-element distance list (row-major), or None."""
        return self._data

    def get_distance_at_pixel(self, px, py):
        """Return the distance (mm) at image pixel (px, py).

        Maps the pixel coordinate to the closest 8×8 zone.
        Returns 0 if the sensor is unavailable or no data yet.
        """
        if self._data is None:
            return 0

        grid = config.TOF_GRID_SIZE
        zx = int(px * grid / config.CAMERA_WIDTH)
        zy = int(py * grid / config.CAMERA_HEIGHT)
        zx = max(0, min(grid - 1, zx))
        zy = max(0, min(grid - 1, zy))

        idx = zy * grid + zx
        if idx < len(self._data):
            return self._data[idx]
        return 0

    @property
    def available(self):
        return self._valid
