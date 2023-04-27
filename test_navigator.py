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
from Navigation import Navigator, LineTracking, DeadReckoning, Odometer
from messaging import OpenMV_MessageHandler

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

master_is_ready = 0

# UART using uart 1 and baud rate of 115200
uart = pyb.UART(1, baudrate=9600, read_buf_len=512)
messaging = OpenMV_MessageHandler(uart, status_data)

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

imu = None

# Test if navigator will fail when imu is not valid, e.g. when the cable falls off
navigator_invalid = Navigator(imu)
navigator_invalid.set_target_heading(100)
print('target heading: ', navigator_invalid.get_target_heading())
print('current heading: ', navigator_invalid.get_current_heading())
print('control output: ', navigator_invalid.get_control_output())

i2c = I2C(2, freq=400000)

try:
    imu = BNO055(i2c)
except:
    pyb.LED(RED_LED_PIN).on()

navigator = Navigator(imu)
navigator.set_target_heading(180)
print('target heading: ', navigator.get_target_heading())
print('current heading: ', navigator.get_current_heading())
print('control output: ', navigator.get_control_output())

navigator.turn_to_heading(0)
navigator.turn_degrees(90) # Turn right 90
navigator.turn_right_degrees(90)
navigator.turn_left_degrees(90)
navigator.turn_right_90()
navigator.turn_left_90()
navigator.turn_to_heading(0) # Should turn left 90
navigator.turn_to_heading(90) # Should turn right 90

async def main():
    '''
        Main coroutine
    '''
    navigator.start_async()
    navigator.set_target_heading(270)
    uasyncio.sleep_ms(2)
    navigator.set_target_heading(30)
    uasyncio.sleep_ms(2)
    navigator.set_target_heading(0)
    uasyncio.sleep_ms(2)
    navigator.end_async()
    print("All tests passed!")

try:
    uasyncio.run(main())
except KeyboardInterrupt:  # Trapping this is optional
    print('Interrupted')  # or pass
finally:
    uasyncio.new_event_loop()  # Clear retained state
