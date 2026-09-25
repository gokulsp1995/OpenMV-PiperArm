# OpenMV AE3 — Lift Button Detector Configuration
# ================================================
# Rebuilt after flash corruption. WiFi/TCP constants removed (serial now),
# blob-detection constants removed (template-only pipeline), and the
# duplicated TEMPLATE_STEP / TEMPLATE_ROI definitions resolved.

# -- Camera --------------------------------------------------------------------
# PAG7936 1MP global shutter. NOTE: this sensor outputs 16:10, not 4:3 --
# QVGA is 320x200 and VGA is 640x400, confirmed via img.height(). The old
# values of 240/120 were wrong and skewed both the ToF row mapping and the
# vertical back-projection.
#
# QVGA is the only usable size: VGA raises MemoryError in find_template()
# with SEARCH_EX, and every smaller size (QQVGA, QQQVGA, HQVGA, QCIF...)
# is rejected by the sensor with "Sensor control failed".
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 200
PRINCIPAL_X = 160.0        # image centre
PRINCIPAL_Y = 100.0
FOCAL_LENGTH_PX = 249.0    # nominal, halved from the 498 VGA figure.
                            # Placeholder until calibrated -- see the
                            # invariance test before trusting a press.

# -- VL53L8CX Time-of-Flight ----------------------------------------------------
# The driver is the built-in `tof` module (tof.init(), tof.read_depth()),
# NOT a vl53l8cx package -- that module doesn't exist on this firmware.
# read_depth() returns (grid, min, max), not a bare list.
TOF_GRID_SIZE = 8          # 8x8 zones

# Validity band. Zones with no valid return come back as sentinel values --
# observed -127 and implausible highs in the thousands. Anything outside
# this range is treated as "no reading" (0) rather than acted on.
TOF_MIN_VALID_MM = 50
TOF_MAX_VALID_MM = 300
TOF_FOV_SCALE = 1.0
TOF_FLIP_X = True
TOF_FLIP_Y = True
TOF_OFFSET_MM = 25

# -- Template Matching ----------------------------------------------------------
TEMPLATE_DIR = "/flash/templates"
TEMPLATE_THRESHOLD = 0.80  # NCC threshold. 0.60 matched up and down at the
                            # same pixel; 0.85 separated them.
TEMPLATE_STEP = 3          # search stride. ~81ms per match at QVGA.

# Reject matches whose centre falls outside this (x, y, w, h) box. Filtered
# in button_detector.detect() rather than passed as find_template(roi=),
# because on this firmware the roi argument crops the image in place and
# the returned frame comes back cropped too.
# None = search the whole frame.
TEMPLATE_ROI = None

# CRITICAL: templates must be captured at the SAME resolution and distance
# the detector runs at. find_template does no scale search. The original
# 30x30 template was cropped from an HD frame, so the button appeared at a
# completely different scale than in the live QVGA frame -- which is why it
# matched masking tape and background instead of the arrow.
BUTTON_TEMPLATES = {
    "up": "up_red.pgm",
    # "down": "arrow_down.pgm",   # re-enable once "up" is reliable
}

# -- Colour Classification ------------------------------------------------------
# NOTE: init_camera() captures GRAYSCALE, so the a/b chrominance channels
# don't exist and everything classifies as "dark". These thresholds are
# kept for when colour capture is viable; press_button()'s white_lit
# verification is non-functional until then.
WHITE_THRESH = (70, 100, -15, 15, -15, 15)
GREEN_THRESH = (30, 80, -60, -10, -10, 40)
DARK_THRESH_L_MAX = 35

# Pixel margin inset when sampling colour inside a matched rectangle,
# to avoid edge effects.
BLOB_MARGIN = 5

# -- LED Feedback ---------------------------------------------------------------
LED_IDLE = (0, 0, 1)       # blue  = idle, all sensors healthy
LED_SCANNING = (0, 1, 0)   # green = scan in progress
LED_ERROR = (1, 0, 0)      # red   = operation failed
LED_DEGRADED = (0, 1, 1)   # cyan  = idle, but ToF failed to initialise