# OpenMV AE3 — RL Data Streaming Configuration
# ==============================================
# All constants for WiFi, camera, ToF, IMU, streaming, and AprilTag detection.

# ── WiFi ──────────────────────────────────────────────────────────────────────
WIFI_SSID = "Autodiscovery"
WIFI_PASSWORD = "robot2robot"
WIFI_TIMEOUT_MS = 15000  # max time to wait for association

# ── Camera ────────────────────────────────────────────────────────────────────
# PAG7936 1 MP global-shutter sensor — native 1280×800.
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 800
JPEG_QUALITY = 50          # JPEG compression quality (10–95, lower = smaller)
TARGET_FPS = 30            # target streaming frame rate

# Camera intrinsics at 1280×800 (nominal, refine after calibration)
# PAG7936 ~3.6 mm active width, ~2.8 mm lens: fx ≈ 2.8 * 1280 / 3.6 ≈ 996
FOCAL_LENGTH_PX = 996.0
PRINCIPAL_X = 640.0
PRINCIPAL_Y = 400.0

# ── VL53L8CX Time-of-Flight ──────────────────────────────────────────────────
TOF_GRID_SIZE = 8          # 8×8 zones
TOF_I2C_BUS = 0            # on-board I2C bus index for the VL53L8CX
TOF_I2C_ADDR = 0x29        # default I2C address
TOF_RANGING_FREQ = 15      # Hz

# ── LSM6DSM IMU ──────────────────────────────────────────────────────────────
IMU_I2C_BUS = 0            # on-board I2C bus index
IMU_I2C_ADDR = 0x6A        # LSM6DSM default address (SA0 = 0; 0x6B if SA0 = 1)
IMU_SAMPLE_HZ = 30         # read at frame rate

# Accelerometer configuration
IMU_ACCEL_RANGE = 4        # ±4 g
IMU_ACCEL_ODR = 104        # Hz (output data rate)

# Gyroscope configuration
IMU_GYRO_RANGE = 500       # ±500 dps
IMU_GYRO_ODR = 104         # Hz

# ── Network — Streaming ──────────────────────────────────────────────────────
UDP_VIDEO_PORT = 8471      # UDP port for JPEG frame broadcast
TCP_SENSOR_PORT = 8472     # TCP port for sensor data + control commands
TCP_BACKLOG = 2            # allow 2 simultaneous clients
TCP_RECV_BUF = 512         # max incoming command size
UDP_CHUNK_SIZE = 1400      # max payload per UDP datagram (MTU-safe)
UDP_BROADCAST_ADDR = "255.255.255.255"

# Packet magic for UDP video frames
UDP_MAGIC = b"\x4f\x4d\x56\x53"  # "OMVS" — OpenMV Stream

# ── AprilTag Detection ───────────────────────────────────────────────────────
APRILTAG_ENABLED = False   # default off — toggled via TCP command
APRILTAG_FAMILY = 0        # TAG36H11 (image.TAG36H11 = 0 in OpenMV)
APRILTAG_MAX_TAGS = 10     # max tags to detect per frame

# Downscale factor for AprilTag detection (detection on smaller image = faster)
# The detection runs on a grayscale copy at CAMERA_WIDTH/APRILTAG_SCALE × …
APRILTAG_SCALE = 2         # 1280/2 = 640 wide for detection

# Tag size in metres (physical tag dimension, needed for pose estimation)
APRILTAG_TAG_SIZE_M = 0.05  # 50 mm default — adjust to your actual tags

# ── LED Feedback ──────────────────────────────────────────────────────────────
LED_IDLE = (0, 0, 1)       # blue = idle / waiting
LED_STREAMING = (0, 1, 0)  # green = streaming
LED_ERROR = (1, 0, 0)      # red = error
LED_CONNECTED = (0, 1, 1)  # cyan = WiFi connected
