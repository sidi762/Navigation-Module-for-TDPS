#
# OpenMV Software for TDPS self-driving vehicle project
# test_navigator.py
# Sidi Liang, 2023
#
'''
    Code for testing the Navigator class.
    Connect the IMU to I2C 2 and Master Cont. to UART 1
    before running this test.
'''
import sensor, image, time, pyb, uasyncio
from machine import Pin, I2C
from bno055 import BNO055, AXIS_P7
from Navigation import Navigator, LineTracking, DeadReckoning, Odometer
from messaging import OpenMV_MessageHandler

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

master_is_ready = 0

RED_LED_PIN = 1
BLUE_LED_PIN = 3


status_data = {'Info_Task': 1,
               'Info_Patio': 1,
               'Info_Stage': 1,
               'Control_PID': 0,
               'Control_Angle': 0,
               'Control_Velocity': 0,
               'Control_Cam_Pitch': 0, #0/1/2 0 for patio 1
               'Control_Ball': 0, # 1 for releasing the ball
               'Control_Comm': 0 # 1 for starting wireless communication
               }


# UART using uart 1 and baud rate of 115200
uart = pyb.UART(1, baudrate=9600, read_buf_len=512)
messaging = OpenMV_MessageHandler(uart, status_data)

imu = None

# Test if navigator will fail when imu is not valid, e.g. when the cable falls off
#navigator_invalid = Navigator(imu, status_data) #fail line 46
#navigator_invalid.set_target_heading(100)
#print('target heading: ', navigator_invalid.get_target_heading())
#print('current heading: ', navigator_invalid.get_current_heading())
#print('control output: ', navigator_invalid.get_control_output())

i2c = I2C(2, freq=400000)
imu = BNO055(i2c)

try:
    imu = BNO055(i2c)
except:
    pyb.LED(RED_LED_PIN).on()

navigator = Navigator(imu, status_data, turn_pid_p = 0.6, turn_pid_i = 0.05, turn_pid_d = 0.01, turn_pid_imax = 1)
navigator.set_target_heading(180)
print('target heading: ', navigator.get_target_heading())
print('current heading: ', navigator.get_current_heading())
print('control output: ', navigator.get_control_output())


async def main():
    '''
        Main coroutine
    '''
    while messaging.master_is_ready() == 0:
        await uasyncio.sleep_ms(1)
        pass
    # await navigator.turn_to_heading(0)
    # await uasyncio.sleep(0)
    # await navigator.turn_degrees(90, 1) # Turn right 90
    # await uasyncio.sleep(0)
    # await navigator.turn_degrees(90, -1) # Turn left 90
    # await uasyncio.sleep(0)
    # await navigator.turn_right_90()
    # await uasyncio.sleep(0)
    # await navigator.turn_left_90()
    # await uasyncio.sleep(0)
    # await navigator.turn_to_heading(0) # Should turn left 90
    # await uasyncio.sleep(0)
    # await navigator.turn_to_heading(90) # Should turn right 90
    # await uasyncio.sleep(0)

    navigator.start_async()
    print("async started")
    while True:
        print("target heading: ", navigator.get_target_heading())
        print("current heading: ", navigator.get_current_heading())


        navigator.set_target_heading(270)
        print("target heading: ", navigator.get_target_heading())
        print("current heading: ", navigator.get_current_heading())
        await uasyncio.sleep_ms(10000)
        navigator.set_target_heading(180)
        print("target heading: ", navigator.get_target_heading())
        print("current heading: ", navigator.get_current_heading())
        await uasyncio.sleep_ms(10000)
        navigator.set_target_heading(90)
        print("target heading: ", navigator.get_target_heading())
        print("current heading: ", navigator.get_current_heading())
        await uasyncio.sleep_ms(10000)
        navigator.set_target_heading(0)
        print("target heading: ", navigator.get_target_heading())
        print("current heading: ", navigator.get_current_heading())
        await uasyncio.sleep_ms(10000)
        navigator.set_target_heading(90)
        print("target heading: ", navigator.get_target_heading())
        print("current heading: ", navigator.get_current_heading())
        await uasyncio.sleep_ms(10000)
        navigator.set_target_heading(180)
        print("target heading: ", navigator.get_target_heading())
        print("current heading: ", navigator.get_current_heading())
        await uasyncio.sleep_ms(10000)



    navigator.end_async()




    print("All tests passed!")

try:
    uasyncio.run(main())
except KeyboardInterrupt:  # Trapping this is optional
    print('Interrupted')  # or pass
finally:
    uasyncio.new_event_loop()  # Clear retained state
