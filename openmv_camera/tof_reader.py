# OpenMV AE3 — VL53L8CX 8x8 Time-of-Flight Reader
# ================================================
# Uses the board's built-in `tof` module. read_depth() returns
# (grid, min, max) -- a flat 64-element row-major list plus the extremes.
#
# TWO FIXES over the naive single-zone lookup:
#
# 1. FOV SCALE. The old mapping assumed the 8x8 ToF grid spans exactly the
#    camera's field of view. The camera lens and the ToF emitter are
#    separate parts with their own FOVs, so that's an assumption, not a
#    fact. If the ToF sees wider than the camera, a pixel near the frame
#    edge maps to a zone aimed well outside what the camera sees -- which
#    is how a button at 175mm reads as 270mm (the wall behind it).
#    TOF_FOV_SCALE compresses the camera's frame into the middle of the
#    grid. 1.0 = identical FOVs; 0.6 = ToF sees ~1.7x wider.
#
# 2. PATCH SAMPLING. One zone is a narrow cone and the button is small, so
#    a zone straddling the button edge returns the background. Sampling
#    every zone the detection's bounding box touches and taking the
#    NEAREST valid reading is far more robust: the button is closer to the
#    camera than anything behind it, so the minimum is the button.

import tof
import config


class ToFReader:
    """Reads the VL53L8CX 8x8 ToF sensor and maps pixels to depth."""

    def __init__(self):
        self._data = None    # latest 64-element grid, row-major
        self._valid = False
        self._init_sensor()

    # -- Initialisation ---------------------------------------------------------

    def _init_sensor(self):
        try:
            tof.init()
            self._valid = True
            print("[ToF] initialised (%dx%d)" % (tof.width(), tof.height()))
        except Exception as e:
            print("[ToF] Init failed:", e)
            self._valid = False

    # -- Reading ----------------------------------------------------------------

    def update(self):
        """Read a new frame. Call once per scan."""
        if not self._valid:
            return
        try:
            grid, _lo, _hi = tof.read_depth()
            self._data = grid
        except Exception as e:
            print("[ToF] Read error:", e)

    @staticmethod
    def _validate(mm):
        """Reject sentinel and out-of-range values.

        Zones with no valid target return status codes rather than
        distances -- observed -127 and implausible highs in the thousands.
        Returning 0 lets callers treat it as "no reading".
        """
        if mm is None:
            return 0
        mm = mm - config.TOF_OFFSET_MM
        if mm < config.TOF_MIN_VALID_MM or mm > config.TOF_MAX_VALID_MM:
            return 0
        return mm

    # -- Pixel to zone ----------------------------------------------------------

    def _pixel_to_zone(self, px, py):
        """Map an image pixel to a ToF zone index, honouring the FOV scale.

        With scale s, the camera's frame occupies the central fraction s of
        the ToF grid, so a pixel at the frame edge maps inward rather than
        to the outermost zone.
        """
        g = config.TOF_GRID_SIZE
        s = config.TOF_FOV_SCALE

        # Normalise pixel to 0..1 across the frame
        fx = px / config.CAMERA_WIDTH
        fy = py / config.CAMERA_HEIGHT

        # Compress into the central fraction s of the grid
        fx = 0.5 + (fx - 0.5) * s
        fy = 0.5 + (fy - 0.5) * s

        zx = int(fx * g)
        zy = int(fy * g)
        if config.TOF_FLIP_X:
            zx = (g - 1) - zx
        if config.TOF_FLIP_Y:
            zy = (g - 1) - zy
        zx = max(0, min(g - 1, zx))
        zy = max(0, min(g - 1, zy))
        return zx, zy

    # -- Public API -------------------------------------------------------------

    def get_distance_at_pixel(self, px, py):
        """Distance at a single pixel's zone. Returns 0 if unavailable."""
        if self._data is None:
            return 0
        g = config.TOF_GRID_SIZE
        zx, zy = self._pixel_to_zone(px, py)
        idx = zy * g + zx
        if idx < len(self._data):
            return self._validate(self._data[idx])
        return 0

    def get_distance_in_rect(self, x, y, w, h):
        """Nearest valid distance across every zone the rect touches.

        Preferred over get_distance_at_pixel() for a detection: the button
        is nearer than its surroundings, so the minimum valid reading in
        the region is the button rather than the background behind it.

        Returns 0 if no zone in the rect produced a valid reading.
        """
        if self._data is None:
            return 0

        g = config.TOF_GRID_SIZE
        zx0, zy0 = self._pixel_to_zone(x, y)
        zx1, zy1 = self._pixel_to_zone(x + w, y + h)

        best = 0
        for zy in range(min(zy0, zy1), max(zy0, zy1) + 1):
            for zx in range(min(zx0, zx1), max(zx0, zx1) + 1):
                idx = zy * g + zx
                if idx >= len(self._data):
                    continue
                d = self._validate(self._data[idx])
                if d > 0 and (best == 0 or d < best):
                    best = d
        return best

    def get_distance_at_zone(self, zx, zy):
        """Distance at a zone directly (0-indexed), no pixel mapping."""
        if self._data is None:
            return 0
        g = config.TOF_GRID_SIZE
        zx = max(0, min(g - 1, zx))
        zy = max(0, min(g - 1, zy))
        idx = zy * g + zx
        if idx < len(self._data):
            return self._validate(self._data[idx])
        return 0

    def get_full_grid(self):
        """The raw 64-element grid, unvalidated. For diagnostics."""
        return self._data

    @property
    def available(self):
        return self._valid