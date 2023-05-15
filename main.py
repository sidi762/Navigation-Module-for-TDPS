#
# OpenMV Software for TDPS self-driving vehicle project
# Main.py
# Sidi Liang, 2023
#

import sensor, image, time, pyb
from machine import Pin, I2C
from bno055 import BNO055, AXIS_P7
from HCSR04 import HCSR04
from Navigation import Navigator, LineTracking, DeadReckoning, Odometer
from time import sleep_ms
from pyb import Timer
import uasyncio
from messaging import OpenMV_MessageHandler
from Arrow import arrow_detection

'''
GPIO Pin usage:
    - P0: UART 1 RX - Comm. with master control (Master TX is C6)
    - P1: UART 1 TX - Comm. with master control (Master RX is C7)
    - P2: HCSR04 1 trigger (not tested yet)
    - P3: HCSR04 1 echo (not tested yet)
    - P4: I2C 2 SCL - BNO055 IMU
    - P5: I2C 2 SDA - BNO055 IMU
    - P6: Reserved Button - NOT 5V tolerant in ADC/DAC mode
    - P7: Reserved fot possible second ultrasonic - I2C 4 avilable (SCL)- Servo 1 avilable
    - P8: Reserved fot possible second ultrasonic - I2C 4 avilable (SDA) - Servo 2 avilable
    - P9: Mode Switch
'''
mode_switch = Pin('P9', Pin.IN, Pin.PULL_DOWN)
RED_LED_PIN = 1
BLUE_LED_PIN = 3

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

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

master_is_ready = 0

# HCSR04
ultrasonic = HCSR04(trig=Pin('P2', Pin.OUT_PP), echo=Pin('P3', Pin.IN, Pin.PULL_DOWN))
ultrasonic_right = HCSR04(trig=Pin('P7', Pin.OUT_PP), echo=Pin('P8', Pin.IN, Pin.PULL_DOWN))

# UART using uart 1 and baud rate of 115200
uart = pyb.UART(1, baudrate=9600, read_buf_len=512)
messaging = OpenMV_MessageHandler(uart, status_data, 1)

# I2C
i2c = I2C(2, freq=400000)
imu = None
try:
    imu = BNO055(i2c)
except:
    pyb.LED(RED_LED_PIN).on()

dr = DeadReckoning()

line_tracking = LineTracking(sensor, draw=True)
navigator = Navigator(imu, status_data, turn_pid_p = 0.6,
                      turn_pid_i = 0.001, turn_pid_d = 0.001,
                      turn_pid_imax = 2)
odometer = Odometer()
#line_tracking.start()

async def start_patio_1():
    last_odo = 0
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
            #velocity = 100
            control, velocity = line_tracking.calculate()
            line = line_tracking.get_line()
            theta_err = line_tracking.get_theta_err()
            status_data['Control_PID'] = control
            status_data['Control_Angle'] = theta_err
            status_data['Control_Velocity'] = velocity
            encoder_data = messaging.get_encoder_data()
            await uasyncio.sleep(0)
            odometer.update_with_encoder_data(float(encoder_data['Info_Encoder_A']),\
                                              float(encoder_data['Info_Encoder_B']))
            await uasyncio.sleep(0)
            odo = odometer.get_odometer()
            if odo > last_odo:
                #print(odo)
                last_odo = odo
            print(ultrasonic.getDistance())
            print(ultrasonic_right.getDistance())
            if imu:
                dr.dead_reckoning(imu)
                #print("Velocity m/s: ", dr.velocity_x, dr.velocity_y, dr.velocity_z)
                #print("Position m: ", dr.position_x, dr.position_y, dr.position_z)

            if patio1_task1_stop_signal:
                velocity = 0
                status_data['Control_Velocity'] = velocity
                current_task = 2
                line_tracking.end()
                break
            await uasyncio.sleep_ms(1)

    elif current_task == 2:
        status_data['Info_Task'] = current_task

        #Turn right 90 degress
        status_data['Info_Stage'] = 1
        navigator.turn_right_90()


        # Crossing the bridge
        status_data['Info_Stage'] = 2
        while True:
            velocity = 50
            status_data['Control_Velocity'] = velocity
            #check_task_done()
            if patio1_task2_stop_signal:
                velocity = 0
                status_data['Control_Velocity'] = velocity
                current_task = 3
                break


    elif current_task == 3:
        status_data['Info_Task'] = current_task

        #Turn left 90 degress
        status_data['Info_Stage'] = 1
        navigator.turn_left_90()

        # Passing the Door
        status_data['Info_Stage'] = 2
        while True:
            velocity = 100
            status_data['Control_Velocity'] = velocity
            if patio1_task3_stop_signal:
                velocity = 0
                status_data['Control_Velocity'] = velocity
                #Patio 1 done
                current_patio = 0
                current_task = 0
                current_stage = 0
                break

    return 0


nav = Navigator()
get_distance = HCSR04().getDistance()
#get_right_distance =

def move_forward_until_hit():
    while True:
        distance = get_distance()
        if distance == 0:
            time.sleep_ms(50)  # wait for the car to collide with the board
            distance = get_distance()
            if distance > 100:
                # board has been hit
                break
        else:
            #move forward
            status_data['Control_Velocity'] = 100

async def start_patio_2():

    #差：##1.右超声波 ##2. 调整角度

    status_data['Info_Patio'] = 2
    print("In Patio 2")
    current_task = status_data['Info_Task']
    patio2_task1_stop_signal = 0
    patio2_task2_stop_signal = 0
    patio2_task3_stop_signal = 0


    if current_task == 1:
        # Arrow detection
        status_data['Info_Task'] = 1
        print("Performing task 1")

        distance = get_distance()
        ###Moveforward by ultrasonic
        if distance >20:
            status_data['Control_Velocity']=100
        status_data['Control_Velocity']=0

        ###arrowdetection
        arrow_direction = arrow_detection();
                #Arrow.py
        turning_angles = {"left": -135, "up":-90,"right": -45}

        # turn left or right based on the arrow direction
        if arrow_direction in turning_angles:
            angle = turning_angles[arrow_direction]
            status_data['Control_Angle']= -angle  # negative angle to turn left
            move_forward_until_hit()
            status_data['Control_Angle']= angle


        while True:
            if patio2_task1_stop_signal:
                velocity = 0
                current_task = 2
                break
        await uasyncio.sleep_ms(1)

    elif current_task == 2:
        #navigate to task2
        status_data['Info_Task'] = current_task
        status_data['Control_Angle'] = nav.turn_degrees(-90)
        stages = {
            "forward": {"turn_degrees": 0, "velocity": 100},
            "turn_left": {"turn_degrees": -90, "velocity": 0},
            "forward2": {"turn_degrees": 0, "velocity": 100},
            "turn_right": {"turn_degrees": 90, "velocity": 0},
            "forward3": {"turn_degrees": 0, "velocity": 100},
            "turn_left2": {"turn_degrees": -90, "velocity": 0},
            "forward4": {"turn_degrees": 0, "velocity": 100},
            "turn_right2": {"turn_degrees": 90, "velocity": 0},
            "forward5": {"turn_degrees": 0, "velocity": 100},
            "turn_left3": {"turn_degrees": -90, "velocity": 0},
            "forward6": {"turn_degrees": 0, "velocity": 0}
        }
        current_stage = "forward"
        current_stage_index = 0

        while True:
            distance = get_distance()
            right_distance = get_right_distance()
            if distance < 10 or right_distance > 30:
                current_stage_index += 1
                current_stage = list(stages.keys())[current_stage_index]
            if current_stage_index == 10:
                status_data['Control_Velocity'] = 0
                break
            stage = stages[current_stage]
            status_data['Control_Angle'] = stage["turn_degrees"]
            status_data['Control_Velocity'] = stage["velocity"]
            await uasyncio.sleep_ms(1000)

        #drop the ball
        status_data['Control_Ball'] = 1
        while True:

            #check_task_done()
            if patio2_task2_stop_signal:
                current_task = 3
                break


    elif current_task == 3:
        velocity = 100
        status_data['Info_Task'] = current_task
        ###nav to communication spot
        status_data['Control_Angle'] = nav.turn_degrees(-90)
        stages = {
            "forward": {"turn_degrees": 0, "velocity": 100},
            "turn_left": {"turn_degrees": -90, "velocity": 0},
            "backward": {"turn_degrees": 0, "velocity": -100},
            "turn_right": {"turn_degrees": 90, "velocity": 0},
            "stop": {"turn_degrees": 0, "velocity": 0}
        }
        current_stage = "forward"
        current_stage_index = 0
        start_time = 0  # Define start_time with a default value
        while True:
            distance = get_distance()
            right_distance = get_right_distance()

            if current_stage == "forward" and distance < 10:
                current_stage = "turn_left"
            elif current_stage == "turn_left":
                current_stage = "backward"
            elif current_stage == "backward" and right_distance > 50:
                current_stage = "turn_right"
                status_data['Control_Velocity'] = velocity
                start_time = pyb.millis()
            elif current_stage == "turn_right" and pyb.elapsed_millis(start_time) > 5000:
                current_stage = "stop"
            elif current_stage == "stop":
                break

            stage = stages[current_stage]
            status_data['Control_Angle'] = stage["turn_degrees"]
            status_data['Control_Velocity'] = stage["velocity"]
            break

        #communication
        status_data['Control_Comm']=1

        while True:
            if patio2_task3_stop_signal:
                # Patio 2 done
                current_patio = 0
                current_task = 0
                current_stage = 0
                break

    return 0


def move_forward(duration):
    velocity = 100
    start_time = time.time()
    while time.time() - start_time < duration:
        status_data['Control_Velocity'] = velocity
    status_data['Control_Velocity'] = 0


async def main():
    '''
        Main coroutine
    '''
    while True:
        clock.tick()
        ret = 1
        current_patio = mode_switch.value() + 1 # 1 for patio 1, 2 for patio 2
        status_data['Info_Patio'] = current_patio
        print("Current Patio: ", current_patio)
        if current_patio == 1:
            ret = await start_patio_1()
            if ret == 0:
                continue
        elif current_patio == 2:
            ret = await start_patio_2()
            if ret == 0:
                continue


#Todo: Soft-reset (Or maybe hard-reset?)
try:
    uasyncio.run(main())
except KeyboardInterrupt:  # Trapping this is optional
    print('Interrupted')  # or pass
finally:
    uasyncio.new_event_loop()  # Clear retained state
