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
from pid import PID

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
GREEN_LED_PIN = 2
BLUE_LED_PIN = 3

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(True)
sensor.set_hmirror(True)
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
messaging = OpenMV_MessageHandler(uart, status_data, 0)

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

dr = DeadReckoning()

line_tracking = LineTracking(sensor, draw=True)
line_tracking_white = LineTracking(sensor, draw=True)
navigator = Navigator(imu, status_data, turn_pid_p = 0.8,
                      turn_pid_i = 0.1, turn_pid_d = 0.008,
                      turn_pid_imax = 10)
odometer = Odometer()
#line_tracking.start()

async def start_patio_1():
    last_odo = 0
    line_tracking.start()
    status_data['Info_Patio'] = 1
    print("In Patio 1")
    current_task = status_data['Info_Task']
    current_task = 1
    status_data['Control_Cam_Pitch'] = 0
    patio1_task1_stop_signal = 0
    patio1_task2_stop_signal = 0
    patio1_task3_stop_signal = 0
    yaw, roll, pitch = imu.euler()
    init_heading = yaw
    print("Patio 1 Initial Heading: ", init_heading)

    while True:
        await uasyncio.sleep_ms(1)
        if current_task == 1:
            status_data['Info_Task'] = 1
            # Line following
            status_data['Info_Stage'] = 1
            print("Performing task 1")
            while True:
                # velocity = 100
                # First corner is at about 247.4 odometer
                control, velocity = line_tracking.calculate()
                line = line_tracking.get_line()
                theta_err = line_tracking.get_theta_err()
                status_data['Control_PID'] = control
                #status_data['Control_Angle'] = theta_err
                status_data['Control_Angle'] = 0
                encoder_data = messaging.get_encoder_data()
                await uasyncio.sleep(0)
                odometer.update_with_encoder_data(float(encoder_data['Info_Encoder_A']),\
                                                  float(encoder_data['Info_Encoder_B']))
                odo = odometer.get_odometer()
                if odo > last_odo:
                    print("odo: ", odo)
                    last_odo = odo

                status_data['Control_Velocity'] = velocity
                # if odo < 1200:
                #     status_data['Control_Velocity'] = velocity
                # elif odo < 2000:
                #     status_data['Control_Velocity'] = velocity * 3
                # else:
                #     status_data['Control_Velocity'] = velocity

                #print(ultrasonic.get_distance())
                #print(ultrasonic_right.get_distance())
                if imu:
                    dr.dead_reckoning(imu)
                    yaw, roll, pitch = imu.euler()
                    #print("Heading: ", yaw)
                    #print("Velocity m/s: ", dr.velocity_x, dr.velocity_y, dr.velocity_z)
                    #print("Position m: ", dr.position_x, dr.position_y, dr.position_z)
                front_distance = ultrasonic.get_distance()
                print("front: ", front_distance)
                if front_distance == 0:
                    front_distance = 300
                if front_distance < 20:
                    # Arrived at the bridge
                    # May need beacon, remove odo?
                    velocity = 0
                    status_data['Control_Velocity'] = velocity
                    current_task = 2
                    line_tracking.end()
                    break
                await uasyncio.sleep_ms(1)

        elif current_task == 2:
            print("Performing task 2")
            status_data['Info_Task'] = current_task
            #Turn right 90 degress
            status_data['Info_Stage'] = 1
            await uasyncio.sleep_ms(1)
            navigator.turn_right_90()
            await uasyncio.sleep_ms(10000)

            odo = odometer.get_odometer()
            task_2_odo_start = odo
            print("task 2 initial odo: ", odo)
            last_odo = odo
            # Crossing the bridge
            status_data['Info_Stage'] = 2
            print("Crossing the bridge")
            while True:
                await uasyncio.sleep_ms(1)
                encoder_data = messaging.get_encoder_data()
                odometer.update_with_encoder_data(float(encoder_data['Info_Encoder_A']),\
                                                  float(encoder_data['Info_Encoder_B']))
                odo = odometer.get_odometer()
                print("odo: ", odo)
                task_2_odo = odo - task_2_odo_start
                if odo > last_odo:
                    print("task_2_odo: ", task_2_odo)
                    last_odo = odo

                #if task_2_odo < 150:
                #    velocity = 100
                #    status_data['Control_Velocity'] = velocity
                #else:
                #    velocity = 60
                #    status_data['Control_Velocity'] = velocity


                velocity = 100
                status_data['Control_Velocity'] = velocity

                status_data['Control_Velocity'] = velocity
                #check_task_done()
                front_distance = ultrasonic.get_distance()
                if front_distance < 15:
                    # odo?
                    velocity = 0
                    status_data['Control_Velocity'] = 0
                    current_task = 3
                    break


        elif current_task == 3:
            print("Performing task 3")
            status_data['Info_Task'] = current_task
            await uasyncio.sleep_ms(3000) # Wait for 3s
            #Turn left 90 degress
            status_data['Info_Stage'] = 1
            navigator.turn_left_90()
            await uasyncio.sleep_ms(10000)
            task_3_target_heading = init_heading
            navigator.set_target_heading(task_3_target_heading)
            navigator.start_async()

            task_3_odo_start = odometer.get_odometer()
            print("task 3 initial odo: ", odo)
            last_odo = task_3_odo_start

            # Passing the Door
            status_data['Info_Stage'] = 2
            while True:
                await uasyncio.sleep_ms(1)
                encoder_data = messaging.get_encoder_data()
                odometer.update_with_encoder_data(float(encoder_data['Info_Encoder_A']),\
                                                  float(encoder_data['Info_Encoder_B']))
                odo = odometer.get_odometer()
                task_3_odo = odo - task_3_odo_start
                if odo > last_odo:
                    print("task_3_odo: ", task_3_odo)
                    last_odo = odo
                velocity = 100
                status_data['Control_Velocity'] = velocity
                if task_3_odo > 300:
                    # need tuning
                    navigator.end_async()
                    velocity = 0
                    status_data['Control_Velocity'] = velocity
                    #Patio 1 done
                    current_patio = 0
                    current_task = 0
                    current_stage = 0
                    break

    return 0


async def move_forward_until_hit():
    while True:
        print("trying to hit board")
        distance = ultrasonic.get_distance()
        if distance < 5:
            await uasyncio.sleep_ms(1000)  # wait for the car to collide with the board
            distance = ultrasonic.get_distance()
            if distance > 100:
                # board has been hit
                break
        else:
            #move forward
            status_data['Control_Velocity'] = 100
        await uasyncio.sleep_ms(1)

task2_pid = PID(p=1, i=0, d=0)
async def patio_2_task_1():
    # Arrow detection
    status_data['Info_Task'] = 1
    print("Performing task 1")
    status_data['Control_Velocity'] = 0
    while messaging.master_is_ready() == 0:
        await uasyncio.sleep_ms(1)
        pyb.LED(BLUE_LED_PIN).on()
        pyb.LED(RED_LED_PIN).on()
        pass

    ###Moveforward by ultrasonic
    while True:
        status_data['Control_Velocity'] = 100
        distance = ultrasonic.get_distance()
        print("distance is", distance)
        if distance < 20:
            status_data['Control_Velocity'] = 0
            break
        await uasyncio.sleep_ms(1)

    ###arrowdetection
    while True:
        arrow_direction = arrow_detection();
        if arrow_direction:
            print("arrow detected as:",arrow_direction)
            break
        await uasyncio.sleep_ms(1)

    turning_angles = {"left": -135, "up":-90,"right": -45}

    # turn left or right based on the arrow direction
    if arrow_direction in turning_angles:
        angle = turning_angles[arrow_direction]
        navigator.turn_degrees(-angle, -1) # negative angle to turn left
        await move_forward_until_hit()
        navigator.turn_degrees(-angle, -1)

    status_data['Control_Velocity']=0
    return 0

async def patio_2_task_2():
    print("Performing task 2")
    status_data['Info_Task'] = 2
    status_data['Control_Velocity'] = 0
    yaw, roll, pitch = imu.euler()
    task2_init_heading = yaw
    task2_target_heading = task2_init_heading

    while True:
        # task 2 stage 1: move forward until reaching
        # the fence
        await uasyncio.sleep_ms(5)
        status_data['Control_Velocity'] = 150
        distance = ultrasonic.get_distance()
        print("distance is", distance)
        await uasyncio.sleep_ms(5)
        if distance < 30:
            status_data['Control_Velocity'] = 0
            navigator.turn_left_90()
            task2_target_heading = task2_init_heading - 90
            await uasyncio.sleep_ms(5000)
            break

    current_stage = "forward1"
    print(current_stage)

    status_data['Control_Velocity'] = 150
    await uasyncio.sleep_ms(3000)

    # stage: "forward1" Todo: add some control on heading
    navigator.set_target_heading(task2_target_heading)
    while current_stage =="forward1":
        status_data['Control_Velocity'] = 150
        right_distance = ultrasonic_right.get_distance()
        steering_pid = task2_pid.get_pid(right_distance - 20, 1)
        print("The right distance is", right_distance)
        if right_distance > 80 and right_distance != 300:
            current_stage = "turn_right1"
            print(current_stage)
            status_data['Control_Velocity'] = 0
            break
        await uasyncio.sleep_ms(1)

    # stage: "turn_right1"
    navigator.turn_degrees(86) #？
    task2_target_heading += 90
    await uasyncio.sleep_ms(5000)
    current_stage = "forward2"
    print(current_stage)


    while current_stage == "forward2":
        status_data['Control_Velocity'] = 150
        distance = ultrasonic.get_distance()
        print("The distance is", distance)
        if distance < 35 and distance != 0:
            current_stage = "turn_left1"
            print(current_stage)
            status_data['Control_Velocity'] = 0
            break
        await uasyncio.sleep_ms(1)

    # stage: "turn_left1"
    navigator.turn_left_90()
    task2_target_heading -= 90
    await uasyncio.sleep_ms(5000)
    current_stage = "forward3"
    print(current_stage)

    navigator.set_target_heading(task2_target_heading)
    status_data['Control_Velocity'] = 150
    await uasyncio.sleep_ms(3000)
    while current_stage == "forward3":
        status_data['Control_Velocity'] = 150
        right_distance = ultrasonic_right.get_distance()
        steering_pid = task2_pid.get_pid(right_distance - 20, 1)
        print("The right distance is", right_distance)
        if right_distance > 80 and right_distance != 300:
            current_stage = "turn_right2"
            print(current_stage)
            status_data['Control_Velocity'] = 0
            break
        await uasyncio.sleep_ms(1)

    # stage: "turn_right2"
    navigator.turn_degrees(86) #?
    task2_target_heading += 90
    await uasyncio.sleep_ms(5000)
    current_stage = "forward4"
    print(current_stage)

    navigator.set_target_heading(task2_target_heading)
    while current_stage == "forward4":
        status_data['Control_Velocity'] = 100
        distance = ultrasonic.get_distance()
        right_distance = ultrasonic_right.get_distance()
        steering_pid = task2_pid.get_pid(right_distance - 20, 1)
        status_data['Control_PID'] = steering_pid
        print("The distance is", distance)
        if distance < 35 and distance != 0:
            current_stage = "turn_left2"
            print(current_stage)
            status_data['Control_Velocity'] = 0
            break
        await uasyncio.sleep_ms(1)

    # stage: "turn_left2"
    navigator.turn_left_90()
    task2_target_heading -= 90
    await uasyncio.sleep_ms(5000)
    current_stage = "forward5"
    print(current_stage)

    # stage: "forward5" Todo: add some control on heading
    while current_stage == "forward5":
        status_data['Control_Velocity'] = 100
        distance = ultrasonic.get_distance()
        right_distance = ultrasonic_right.get_distance()
        steering_pid = task2_pid.get_pid(right_distance - 20, 1)
        print("The distance is", distance)
        if distance < 20 :
            print("Bucket Detected!")
            status_data['Control_Velocity'] = 0
            status_data['Control_PID'] = 0
            break
        await uasyncio.sleep_ms(1)

    #drop the ball
    status_data['Control_Ball'] = 1
    while True:
        # Wait until the ball is dropped and the claw is retracted
        await uasyncio.sleep_ms(1)
        feedback = messaging.get_feedback_data()
        print(feedback['Info_Ball'])
        if feedback['Info_Ball'] == "1":
            break

    return 0 #task 2 done

async def patio_2_task_3():
    status_data['Info_Task'] = 3
    print("Performing task 3")
    current_stage = "turn_left"
    ###nav to communication spot

    # Turn left 90 degrees
    navigator.turn_left_90()
    await uasyncio.sleep_ms(5000)
    current_stage = "forward"
    print(current_stage)

    while current_stage == "forward":
        await uasyncio.sleep_ms(1)
        status_data['Control_Velocity'] = 150
        distance = ultrasonic.get_distance()
        print("Front distance: ", distance)
        if distance < 30 :
            status_data['Control_Velocity'] = 0
            current_stage = "turn_left"
            print(current_stage)
            break

    navigator.turn_left_90()
    await uasyncio.sleep_ms(5000)
    current_stage = "backward"
    print(current_stage)

    while current_stage=="backward":
        await uasyncio.sleep_ms(1)
        print(current_stage)
        status_data['Control_Velocity'] = -140
        right_distance = ultrasonic_right.get_distance()
        if right_distance > 80 and right_distance != 300:
            #await uasyncio.sleep_ms(2000)
            status_data['Control_Velocity'] = 0
            current_stage = "turn_right"
            print(current_stage)
            break

    navigator.turn_right_90()
    await uasyncio.sleep_ms(5000)
    current_stage = "final to communication"
    print(current_stage)

    # Drive to communication spot
    status_data['Control_Velocity'] = 100
    await uasyncio.sleep_ms(5000)
    status_data['Control_Velocity'] = 0
    print("communication spot arrived!")

    # Communication
    status_data['Control_Comm'] = 1
    await uasyncio.sleep_ms(5000)
    print("Driving to finishing line!")

    # Drive to finish line
    status_data['Control_Velocity'] = 100
    await uasyncio.sleep_ms(6000)
    status_data['Control_Velocity'] = 0
    print("finishing line arrived!")


async def start_patio_2():
    #差：调整角度

    status_data['Info_Patio'] = 2
    print("In Patio 2")
    #current_task = status_data['Info_Task']
    current_task = 2

    while(True):
        #await uasyncio.sleep(0)
        if current_task == 1:
            await patio_2_task_1()
            current_task = 2
        elif current_task == 2:
            #navigate to task2
            await patio_2_task_2()
            current_task = 3
        elif current_task == 3:
            await patio_2_task_3()
            current_task = 300
        elif current_task == 300:
            break # all tasks are done

    return 0

async def main():
    '''
        Main coroutine
    '''
    while True:
        clock.tick()
        ret = 1
        print("Waiting for master to be ready")
        while messaging.master_is_ready() == 0:
            await uasyncio.sleep_ms(1)
            pyb.LED(BLUE_LED_PIN).on()
            pyb.LED(GREEN_LED_PIN).on()
            pass
        pyb.LED(GREEN_LED_PIN).off()
        pyb.LED(BLUE_LED_PIN).off()
        current_patio = 2 - mode_switch.value() # 1 for patio 1, 2 for patio 2
        status_data['Info_Patio'] = current_patio
        print("Current Patio: ", current_patio)
        if current_patio == 1:
            ret = await start_patio_1()
            if ret == 0:
                break
        elif current_patio == 2:
            ret = await start_patio_2()
            if ret == 0:
                break
    return 0


#Todo: Soft-reset (Or maybe hard-reset?)
try:
    uasyncio.run(main())
except KeyboardInterrupt:  # Trapping this is optional
    print('Interrupted')  # or pass
finally:
    uasyncio.new_event_loop()  # Clear retained state
