'''
Sidi Liang, 2023
Calibrate BNO055 imu and save calibration data to file
'''
import sensor, image, time, pyb
from machine import Pin, I2C
from bno055_third_party import *
from HCSR04 import HCSR04
from time import sleep_ms
from pyb import Timer

RED_LED_PIN = 1
GREEN_LED_PIN = 2
BLUE_LED_PIN = 3

# I2C
i2c = I2C(2, freq=400000)
imu = None
while imu == None:
    # Wait for IMU to be ready
    try:
        imu = BNO055(i2c)
    except:
        pyb.LED(RED_LED_PIN).on()

pyb.LED(RED_LED_PIN).off()
# Configure the imu
COMPASS_MODE = 0x09
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

heading, roll, pitch = 0, 0, 0
# Calibrate the imu
offsets = b'\x00\x00\x00\x00\x00\x00\x17\x00\xb8\x01\xfd\xff\x00\x00\x00\x00\xff\xff\xe8\x03l\x02'
imu.set_offsets(offsets)
while True:
    time.sleep(1)


    print('Temperature {}°C'.format(imu.temperature()))
    print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
    print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
    print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
    print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
    print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
    print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
    heading, roll, pitch = imu.euler()
    heading += 90
    if heading > 360:
        heading -= 360
    print(heading)

