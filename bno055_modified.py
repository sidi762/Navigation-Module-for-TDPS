import struct
import pyb

CONFIG_MODE = 0x00
ACCONLY_MODE = 0x01
MAGONLY_MODE = 0x02
GYRONLY_MODE = 0x03
ACCMAG_MODE = 0x04
ACCGYRO_MODE = 0x05
MAGGYRO_MODE = 0x06
AMG_MODE = 0x07
IMUPLUS_MODE = 0x08
COMPASS_MODE = 0x09
M4G_MODE = 0x0A
NDOF_FMC_OFF_MODE = 0x0B
NDOF_MODE = 0x0C

AXIS_P0 = bytes([0x21, 0x04])
AXIS_P1 = bytes([0x24, 0x00])
AXIS_P2 = bytes([0x24, 0x06])
AXIS_P3 = bytes([0x21, 0x02])
AXIS_P4 = bytes([0x24, 0x03])
AXIS_P5 = bytes([0x21, 0x01])
AXIS_P6 = bytes([0x21, 0x07])
AXIS_P7 = bytes([0x24, 0x05])

_MODE_REGISTER = 0x3D
_POWER_REGISTER = 0x3E
_AXIS_MAP_CONFIG = 0x41
_CALIB_STAT = 0x35


class BNO055:
    def __init__(self, i2c, address=0x28, mode=NDOF_MODE, axis=AXIS_P4):
        self.i2c = i2c
        self.address = address
        if self.read_id() != bytes([0xA0, 0xFB, 0x32, 0x0F]):
            raise RuntimeError("Failed to find expected ID register values. Check wiring!")
        self.operation_mode(CONFIG_MODE)
        self.system_trigger(0x20)  # reset
        pyb.delay(700)
        self.power_mode(0x00)  # POWER_NORMAL
        self.axis(axis)
        self.page(0)
        pyb.delay(10)
        self.operation_mode(mode)
        self.system_trigger(0x80)  # external oscillator
        pyb.delay(200)

    def read_registers(self, register, size=1):
        return self.i2c.readfrom_mem(self.address, register, size)

    def write_registers(self, register, data):
        self.i2c.writeto_mem(self.address, register, data)

    def operation_mode(self, mode=None):
        if mode:
            self.write_registers(_MODE_REGISTER, bytes([mode]))
        else:
            return self.read_registers(_MODE_REGISTER, 1)[0]

    def system_trigger(self, data):
        self.write_registers(0x3F, bytes([data]))

    def power_mode(self, mode=None):
        if mode:
            self.write_registers(_POWER_REGISTER, bytes([mode]))
        else:
            return self.read_registers(_POWER_REGISTER, 1)

    def calibration_status(self):
        calibration_data = struct.unpack("<h", self.read_registers(_CALIB_STAT, 8))[0]
        sys = (calibration_data >> 6) & 0x03
        gyro = (calibration_data >> 4) & 0x03
        accel = (calibration_data >> 2) & 0x03
        mag = calibration_data & 0x03
        return sys, gyro, accel, mag

    def calibrated(self):
        sys, gyro, accel, mag = self.calibration_status()
        return sys == gyro == accel == mag == 0x03

    def read_mag_offsets(self):
        mode = self.operation_mode()
        self.operation_mode(CONFIG_MODE)
        ﻿pyb.delay(700)
        MAG_OFFSET_X_LSB = self.read_registers(0x5B, 8)
        MAG_OFFSET_X_MSB = self.read_registers(0x5C, 8)
        MAG_OFFSET_Y_LSB = self.read_registers(0x5D, 8)
        MAG_OFFSET_Y_MSB = self.read_registers(0x5E, 8)
        MAG_OFFSET_Z_LSB = self.read_registers(0x5F, 8)
        MAG_OFFSET_Z_MSB = self.read_registers(0x60, 8)
        MAG_RADIUS_LSB = self.read_registers(0x69, 8)
        MAG_RADIUS_MSB = self.read_registers(0x6A, 8)
        return [MAG_OFFSET_X_LSB, MAG_OFFSET_X_MSB, \
                MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_MSB, \
                MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_MSB, \
                MAG_RADIUS_LSB, MAG_RADIUS_MSB]
        self.operation_mode(mode)

    def write_mag_offsets(self, offsets):
        mode = self.operation_mode()
        self.operation_mode(CONFIG_MODE)
        self.write_registers(0x5B, offsets[0])
        self.write_registers(0x5C, offsets[1])
        self.write_registers(0x5D, offsets[2])
        self.write_registers(0x5E, offsets[3])
        self.write_registers(0x5F, offsets[4])
        self.write_registers(0x60, offsets[5])
        self.write_registers(0x69, offsets[6])
        self.write_registers(0x6A, offsets[7])
        self.operation_mode(mode)

    def page(self, num=None):
        if num:
            self.write_registers(0x3F, bytes([num]))
        else:
            self.read_registers(0x3F)

    def temperature(self):
        return self.read_registers(0x34, 1)[0]

    def read_id(self):
        return self.read_registers(0x00, 4)

    def axis(self, placement=None):
        if placement:
            self.write_registers(_AXIS_MAP_CONFIG, placement)
        else:
            return self.read_registers(_AXIS_MAP_CONFIG, 2)

    def quaternion(self):
        data = struct.unpack("<hhhh", self.read_registers(0x20, 8))
        return [d / (1 << 14) for d in data]  # [w, x, y, z]

    def euler(self):
        data = struct.unpack("<hhh", self.read_registers(0x1A, 6))
        return [d / 16 for d in data]  # [yaw, roll, pitch]

    def accelerometer(self):
        data = struct.unpack("<hhh", self.read_registers(0x08, 6))
        return [d / 100 for d in data]  # [x, y, z]

    def magnetometer(self):
        data = struct.unpack("<hhh", self.read_registers(0x0E, 6))
        return [d / 16 for d in data]  # [x, y, z]

    def gyroscope(self):
        data = struct.unpack("<hhh", self.read_registers(0x14, 6))
        return [d / 900 for d in data]  # [x, y, z]

    def linear_acceleration(self):
        data = struct.unpack("<hhh", self.read_registers(0x28, 6))
        return [d / 100 for d in data]  # [x, y, z]

    def gravity(self):
        data = struct.unpack("<hhh", self.read_registers(0x2E, 6))
        return [d / 100 for d in data]  # [x, y, z]
