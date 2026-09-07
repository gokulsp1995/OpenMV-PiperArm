# OpenMV AE3 — LSM6DSM IMU Reader
# ================================
# Direct I²C register access to the on-board LSM6DSM 6-axis IMU.
# Provides accelerometer (g) and gyroscope (dps) readings at configurable ODR.
#
# Reference: STMicroelectronics LSM6DSM datasheet (DocID027906 Rev 9)

from machine import I2C
import struct
import config


# ── LSM6DSM Register Map (subset) ────────────────────────────────────────────
_WHO_AM_I = 0x0F       # should return 0x6A
_CTRL1_XL = 0x10       # accelerometer control
_CTRL2_G = 0x11        # gyroscope control
_CTRL3_C = 0x12        # control register 3 (BDU, IF_INC)
_STATUS_REG = 0x1E     # data-ready bits
_OUTX_L_G = 0x22       # gyroscope output start (6 bytes: GX, GY, GZ)
_OUTX_L_XL = 0x28      # accelerometer output start (6 bytes: AX, AY, AZ)

# ODR bits for CTRL1_XL and CTRL2_G [7:4]
_ODR_MAP = {
    0: 0x00,       # power-down
    12: 0x10,      # 12.5 Hz
    26: 0x20,      # 26 Hz
    52: 0x30,      # 52 Hz
    104: 0x40,     # 104 Hz
    208: 0x50,     # 208 Hz
    416: 0x60,     # 416 Hz
    833: 0x70,     # 833 Hz
    1660: 0x80,    # 1.66 kHz
}

# Accelerometer full-scale bits for CTRL1_XL [3:2]
_ACCEL_FS_MAP = {
    2: 0x00,   # ±2 g
    4: 0x08,   # ±4 g
    8: 0x0C,   # ±8 g
    16: 0x04,  # ±16 g
}

# Gyroscope full-scale bits for CTRL2_G [3:2]  (bit 1 = FS_125)
_GYRO_FS_MAP = {
    125: 0x02,     # ±125 dps
    250: 0x00,     # ±250 dps
    500: 0x04,     # ±500 dps
    1000: 0x08,    # ±1000 dps
    2000: 0x0C,    # ±2000 dps
}

# Sensitivity (LSB → physical units)
_ACCEL_SENS = {
    2: 0.000061,    # g/LSB
    4: 0.000122,
    8: 0.000244,
    16: 0.000488,
}

_GYRO_SENS = {
    125: 0.004375,  # dps/LSB
    250: 0.00875,
    500: 0.0175,
    1000: 0.035,
    2000: 0.070,
}


class IMUReader:
    """Reads the LSM6DSM 6-axis IMU (accel + gyro) over I²C."""

    def __init__(self):
        self._bus = None
        self._addr = config.IMU_I2C_ADDR
        self._valid = False
        self._accel_sens = _ACCEL_SENS.get(config.IMU_ACCEL_RANGE, 0.000122)
        self._gyro_sens = _GYRO_SENS.get(config.IMU_GYRO_RANGE, 0.0175)
        self._last = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # cached reading
        self._init_sensor()

    # ── Initialisation ────────────────────────────────────────────────────────

    def _init_sensor(self):
        """Configure the LSM6DSM registers."""
        try:
            self._bus = I2C(config.IMU_I2C_BUS)

            # Verify WHO_AM_I
            who = self._read_reg(_WHO_AM_I)
            if who != 0x6A:
                print("[IMU] WHO_AM_I mismatch: 0x%02X (expected 0x6A)" % who)
                self._valid = False
                return

            # CTRL3_C: enable BDU (block data update) + IF_INC (auto-increment)
            self._write_reg(_CTRL3_C, 0x44)

            # CTRL1_XL: set accelerometer ODR + full-scale
            odr_bits = _ODR_MAP.get(config.IMU_ACCEL_ODR, 0x40)
            fs_bits = _ACCEL_FS_MAP.get(config.IMU_ACCEL_RANGE, 0x08)
            self._write_reg(_CTRL1_XL, odr_bits | fs_bits)

            # CTRL2_G: set gyroscope ODR + full-scale
            odr_bits = _ODR_MAP.get(config.IMU_GYRO_ODR, 0x40)
            fs_bits = _GYRO_FS_MAP.get(config.IMU_GYRO_RANGE, 0x04)
            self._write_reg(_CTRL2_G, odr_bits | fs_bits)

            self._valid = True
            print("[IMU] LSM6DSM initialised (accel ±%dg, gyro ±%ddps, ODR %dHz)"
                  % (config.IMU_ACCEL_RANGE, config.IMU_GYRO_RANGE, config.IMU_ACCEL_ODR))

        except Exception as e:
            print("[IMU] Init failed:", e)
            self._valid = False

    # ── Register I/O ──────────────────────────────────────────────────────────

    def _read_reg(self, reg):
        """Read a single register byte."""
        data = self._bus.readfrom_mem(self._addr, reg, 1)
        return data[0]

    def _read_regs(self, reg, length):
        """Read multiple consecutive registers."""
        return self._bus.readfrom_mem(self._addr, reg, length)

    def _write_reg(self, reg, value):
        """Write a single register byte."""
        self._bus.writeto_mem(self._addr, reg, bytes([value]))

    # ── Public API ────────────────────────────────────────────────────────────

    def read(self):
        """Read accelerometer and gyroscope.

        Returns (ax, ay, az, gx, gy, gz) where:
            ax, ay, az — acceleration in g
            gx, gy, gz — angular rate in degrees per second
        """
        if not self._valid:
            return self._last

        try:
            # Read 12 bytes: gyro (6) + accel (6) — they are contiguous
            # OUTX_L_G = 0x22 .. 0x27 (gyro XYZ)
            # OUTX_L_XL = 0x28 .. 0x2D (accel XYZ)
            raw = self._read_regs(_OUTX_L_G, 12)
            gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw = struct.unpack("<hhhhhh", raw)

            ax = ax_raw * self._accel_sens
            ay = ay_raw * self._accel_sens
            az = az_raw * self._accel_sens
            gx = gx_raw * self._gyro_sens
            gy = gy_raw * self._gyro_sens
            gz = gz_raw * self._gyro_sens

            self._last = (ax, ay, az, gx, gy, gz)
            return self._last

        except Exception as e:
            print("[IMU] Read error:", e)
            return self._last

    @property
    def available(self):
        return self._valid
