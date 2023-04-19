#
# OpenMV Software for TDPS self-driving vehicle project
# Main.py
# Sidi Liang, 2023
#

import sensor, image, time, pyb
from machine import Pin, I2C
from bno055 import BNO055, AXIS_P7
from HCSR04 import HCSR04
from LineTracking import LineTracking, dead_reckoning
from time import sleep_ms
from pyb import Timer
import uasyncio
from messaging import readwrite

'''
GPIO Pin usage:
    - P0: UART 1 RX - Comm. with master control
    - P1: UART 1 TX - Comm. with master control
    - P2: HCSR04 1 trigger (not tested yet)
    - P3: HCSR04 1 echo (not tested yet)
    - P4: I2C 2 SCL - BNO055 IMU
    - P5: I2C 2 SDA - BNO055 IMU
    - P6: Reserved Button - NOT 5V tolerant in ADC/DAC mode
    - P7: Reserved fot possible second ultrasonic - I2C 4 avilable (SCL)- Servo 1 avilable
    - P8: Reserved fot possible second ultrasonic - I2C 4 avilable (SDA) - Servo 2 avilable
    - P9: Mode Switch
'''

RED_LED_PIN = 1
BLUE_LED_PIN = 3

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

status_data = {'Info_Task': "1",
               'Info_Patio': "1",
               'Info_Stage': "1",
               'Control_Command': 0,
               'Control_Angle': "0",
               'Control_Velocity': "0"}

master_is_ready = 0

# UART using uart 1 and baud rate of 115200
uart = pyb.UART(1, 9600)

# I2C
i2c = I2C(2, freq=400000)
imu = None
try:
    imu = BNO055(i2c)
except:
    pyb.LED(RED_LED_PIN).on()

dr = dead_reckoning.DeadReckoning()

line_tracking = LineTracking(sensor, draw=True)
#line_tracking.start()

async def start_patio_1():
    line_tracking.start()
    status_data['Info_Patio'] = 1
    print("In Patio 1")
    current_task = status_data['Info_Task']
    patio1_task1_stop_signal = 0
    patio1_task2_stop_signal = 0
    patio1_task3_stop_signal = 0
    if current_task == 1:
        status_data['Info_Task'] = 1
        # Line following
        status_data['Info_Stage'] = 1
        print("Performing task 1")
        while True:
            velocity = 100
            control = line_tracking.calculate()
            line = line_tracking.get_line()
            theta_err = line_tracking.get_theta_err()
            status_data['Control_Command'] = control
            status_data['Control_Angle'] = theta_err
            status_data['Control_Velocity'] = velocity
            if imu:
                dr.dead_reckoning(imu)
                # print("Velocity m/s: ", dr.velocity_x, dr.velocity_y, dr.velocity_z)
                # print("Position m: ", dr.position_x, dr.position_y, dr.position_z)

            if patio1_task1_stop_signal:
                velocity = 0
                current_task = 2
                line_tracking.end()
                break
            await uasyncio.sleep_ms(1)

    elif current_task == 2:
        status_data['Info_Task'] = current_task

        #Turn right 90 degress
        status_data['Info_Stage'] = 1

        # Crossing the bridge
        status_data['Info_Stage'] = 2
        while True:
            velocity = 50
            #check_task_done()
            if patio1_task2_stop_signal:
                current_task = 3
                break


    elif current_task == 3:
        status_data['Info_Task'] = current_task

        #Turn left 90 degress
        status_data['Info_Stage'] = 1

        # Passing the Door
        status_data['Info_Stage'] = 2
        while True:
            velocity = 100
            if patio1_task3_stop_signal:
                #Patio 1 done
                current_patio = 0
                current_task = 0
                current_stage = 0
                break

    return 0


async def start_patio_2():
    status_data['Info_Patio'] = 2
    print("In Patio 2")
    current_task = status_data['Info_Task']
    patio1_task1_stop_signal = 0
    patio1_task2_stop_signal = 0
    patio1_task3_stop_signal = 0
    if current_task == 1:
        # Arrow detection
        status_data['Info_Task'] = 1
        print("Performing task 1")

        ###Moveforward by ultrasonic

        ###arrowdetection
        arrow=arrow_detection();
                #Arrow.py

        ###turn angle
        if arrow=="left":
            pass
            #imu.angle();
        elif arrow=="forward":
            pass
            #imu
        elif arrow=="right":
            pass
            #imu

        ###moveforward by ultrasonic

    elif current_task == 2:
        #drop the ball
        status_data['Info_Task'] = 2
        ###find fence

        ###distance to 0
             #ultra
        ###drop the ball



    elif current_task == 3:
        # To planter
        status_data['Info_Task'] = 3
        ###imu turn left

        ###go for apriltag



    print("Patio 2 is progressing....can't wait to see:)...")
    return 0


async def main():
    '''
        Main coroutine
    '''
    while True:
        clock.tick()
        ret = 1
        current_patio = int(status_data['Info_Patio'])
        if current_patio == 1:
            ret = await start_patio_1()
            if ret == 0:
                continue
        elif current_patio == 2:
            ret = await start_patio_2()
            if ret == 0:
                continue


loop = uasyncio.get_event_loop()
loop.create_task(readwrite())
loop.create_task(main())
loop.run_forever()
#Todo: Soft-reset (Or maybe hard-reset?)
