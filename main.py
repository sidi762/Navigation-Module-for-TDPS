#
# OpenMV Software for TDPS self-driving vehicle project
# Main.py
# Sidi Liang, 2023
#

import sensor, image, time, pyb, json
from machine import Pin, I2C
from bno055 import BNO055, AXIS_P7
from HCSR04 import HCSR04
from LineTracking import LineTracking, dead_reckoning
from time import sleep_ms
from pyb import Timer
import uasyncio

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
               'Control_Command': 0,
               'Control_Angle': 0,
               'Control_Velocity': 0}

encoder_data = {'Info_Encoder_A': 0.0,
                'Info_Encoder_B': 0.0,
                'Info_Encoder_C': 0.0,
                'Info_Encoder_D': 0.0}

master_is_ready = 0

# UART using uart 1 and baud rate of 115200
uart = pyb.UART(1, 115200)

async def uart_messaging_json(data):
    data_json = json.dumps([status_data])
    uart_send_buffer = b'\xaa\x55' + len(data_json).to_bytes(1, 'big') + bytes(data_json, 'utf-8')
    print("UART sent: ", uart_send_buffer)
    print("UART len: ", len(uart_send_buffer))
    last_message = uart_send_buffer
    return uart_send_buffer

async def update_encoder_data(data):
    try:
        encoder_data['Info_Encoder_A'] = data['Info_Encoder_A']
        encoder_data['Info_Encoder_B'] = data['Info_Encoder_B']
        encoder_data['Info_Encoder_C'] = data['Info_Encoder_C']
        encoder_data['Info_Encoder_D'] = data['Info_Encoder_D']
    except:
        return False
    return True


async def uart_recv_json(rcv_buffer):
    data = ''
    print("Processing rcvbuffer: ", rcv_buffer)
    try:
        data = json.loads(rcv_buffer)
    except ValueError as err:
        print("ValueError! Received json string invalid")
        print("Message causing error: ", rcv_buffer)
        return False

    if await update_encoder_data(data):
        return True
    else:
        print("Encoder_data failed to update, check data")
        print("Message causing error: ", data)
        return False



async def readwrite():
    '''
        Coroutine handling UART communication with the master control.
    '''
    swriter = uasyncio.StreamWriter(uart, {})
    sreader = uasyncio.StreamReader(uart)
    last_message = 'nil'

    while True:
        print('Waiting for incoming message...')
        rcvbuf = ''
        rcv = await sreader.read(1)
        if rcv:
            print('Received: ', rcv)
            buf = last_message
            if rcv == b'\xcc':
                # 0xcc: last message correctly received,
                # send next message
                buf = await uart_messaging_json(status_data)
                last_message = buf
            elif rcv == b'\xdd':
                # 0xdd: error in the last transmission,
                # resend message
                buf = last_message
            elif rcv == b'\xaa':
                # 0xaa 0x55: Incoming message
                rcv = await sreader.read(1)
                print('Received: ', rcv)
                if rcv == b'\x55':
                    # while rcv != b'\xbb':
                    #     rcv = await sreader.read(1)
                    #     rcvbuf += rcv
                    rcvlen = await sreader.read(1)
                    rcvlen = int.from_bytes(rcvlen, 'big')
                    print("rcvlen: ", rcvlen)
                    if rcvlen == 1:
                        print("Master control ready")
                        master_is_ready = 1
                        continue
                    rcvbuf = await sreader.read(rcvlen)
                    if await uart_recv_json(rcvbuf):
                        buf = b'\xcc' # Message correctly received
                    else:
                        buf = b'\xdd'
                else:
                    buf = b'\xdd'

            else:
                buf = b'\xdd'

            await swriter.awrite(buf)
            print('Sent: ', buf)
            await uasyncio.sleep_ms(1)

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
        status_data['Info_Task'] = 2

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
        status_data['Info_Task'] = 3

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
            #imu.angle();
        elif arrow=="forward":
            #imu
        elif arrow=="right":
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
        current_patio = status_data['Info_Patio']
        if current_patio == 1:
            ret = await start_patio_1()
            if ret == 0:
                current_patio = 2
        elif current_patio == 2:
            ret = await start_patio_2()
            if ret == 0: break


loop = uasyncio.get_event_loop()
loop.create_task(readwrite())
loop.create_task(main())
loop.run_forever()
