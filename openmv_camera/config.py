# OpenMV AE3 — Lift Button Detector Configuration
# ================================================
# All constants for WiFi, camera, ToF, and detection parameters.

# ── WiFi ──────────────────────────────────────────────────────────────────────
WIFI_SSID = "MeiPiehChi"
WIFI_PASSWORD = "robot2robot"
WIFI_TIMEOUT_MS = 15000  # max time to wait for association

# ── TCP Server ────────────────────────────────────────────────────────────────
# The camera runs a TCP *server* so the robot computer can connect at will.
TCP_PORT = 8470
TCP_BACKLOG = 1  # only one client (the robot) at a time
TCP_RECV_BUF = 512

# ── Camera ────────────────────────────────────────────────────────────────────
# PAG7936 1 MP global-shutter sensor — nominal intrinsics at VGA (640×480).
# The PAG7936 has a 1/4" optical format (~3.6 × 2.7 mm active area).
# With the standard ~2.8 mm M12 lens: fx ≈ 2.8 * 640 / 3.6 ≈ 498 px.
# Principal point assumed at image centre.  Refine after calibration.
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FOCAL_LENGTH_PX = 498.0   # PAG7936 nominal focal length in pixels at VGA
PRINCIPAL_X = 320.0        # principal point X (pixels)
PRINCIPAL_Y = 240.0        # principal point Y (pixels)

# ── VL53L8CX Time-of-Flight ──────────────────────────────────────────────────
TOF_GRID_SIZE = 8          # 8×8 zones
TOF_I2C_BUS = 0            # on-board I2C bus index for the VL53L8CX
TOF_I2C_ADDR = 0x29        # default I2C address
TOF_RANGING_FREQ = 15      # Hz

# ── Button Detection — LAB Colour Thresholds ─────────────────────────────────
# Each tuple is (L_min, L_max, A_min, A_max, B_min, B_max) in LAB colour space.
# Tuned for typical stainless-steel lift panels under corridor lighting.
# Adjust after testing on the actual lift.

# White illuminated button (high luminance, near-neutral chrominance)
WHITE_THRESH = (70, 100, -15, 15, -15, 15)

# Green illuminated button (moderate L, strong negative A = green)
GREEN_THRESH = (30, 80, -60, -10, -10, 40)

# Dark / unlit button (low luminance)
DARK_THRESH_L_MAX = 35  # anything below this L is "dark"

# ── Blob Detection Parameters ────────────────────────────────────────────────
# Thresholds for finding metallic / plastic button blobs (initial blob search).
# These are permissive — classification happens after.
BLOB_THRESH = [(15, 100, -40, 40, -40, 40)]  # broad LAB range
BLOB_MIN_PIXELS = 200
BLOB_MAX_PIXELS = 8000
BLOB_MIN_CIRCULARITY = 0.5   # buttons are roughly circular
BLOB_MERGE_DISTANCE = 10
BLOB_MARGIN = 5               # pixel margin around blobs to avoid edge effects

# ── Template Matching ─────────────────────────────────────────────────────────
TEMPLATE_DIR = "/templates"
TEMPLATE_THRESHOLD = 0.60     # NCC threshold (0–1) for a positive match
TEMPLATE_SCALE_RANGE = (0.8, 1.2)  # search at 80%–120% of template size
TEMPLATE_SCALE_STEP = 0.1

# Known button identifiers and their corresponding template filenames.
# Arrow buttons use arrow-up.pgm / arrow-down.pgm.
BUTTON_TEMPLATES = {
    "1": "btn_1.pgm",
    "2": "btn_2.pgm",
    "3": "btn_3.pgm",
    "up": "arrow_up.pgm",
    "down": "arrow_down.pgm",
}

# ── Panel Layouts ─────────────────────────────────────────────────────────────
# Describes the physical button arrangement on each lift panel.
# Used as a fallback when templates are unavailable: detected blobs are
# assigned IDs based on their relative positions within the image.
#
# Panel 1 — horizontal single row:
#   (1)  (2)  (3)  [door-open]  [alarm]
#   Buttons are sorted left-to-right by pixel X.
#
# Panel 2 — 3 rows × 2 columns:
#   Row 1:  (3)      [other]
#   Row 2:  (1)      (2)
#   Row 3:  [other]  [other]
#   Buttons are sorted top-to-bottom then left-to-right.
#
# "panel" is sent in the SCAN command so the detector knows which layout.
PANEL_LAYOUTS = {
    "panel1": {
        # Ordered left→right.  Only the first 3 are relevant lift buttons.
        "order": ["1", "2", "3"],
        "arrangement": "horizontal",
    },
    "panel2": {
        # Grid layout: (row, col) → button id.
        "grid": {
            (0, 0): "3",
            (1, 0): "1",
            (1, 1): "2",
        },
        "arrangement": "grid",
        "rows": 3,
        "cols": 2,
    },
}

# ── LED Feedback ──────────────────────────────────────────────────────────────
# Use the on-board RGB LED to signal state.
LED_IDLE = (0, 0, 1)      # blue = idle / waiting
LED_SCANNING = (0, 1, 0)  # green = scanning
LED_ERROR = (1, 0, 0)     # red = error
LED_CONNECTED = (0, 1, 1) # cyan = WiFi connected
