#
# OpenMV Software for TDPS self-driving vehicle project
# The Line Tracking Module
# UART Messaging
# Sidi Liang, 2023
#

import json, uasyncio

encoder_data = {'Info_Encoder_A': "0",
                'Info_Encoder_B': "0",
                'Info_Encoder_C': "0",
                'Info_Encoder_D': "0"}

async def uart_messaging_json(data, status_data):
    data_json = json.dumps(status_data)
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
            if rcv == b'\xcd':
                # 0xcc: last message correctly received,
                # send next message
                buf = await uart_messaging_json(status_data)
                last_message = buf
            elif rcv == b'\xcc':
                await uasyncio.sleep_ms(1)
                continue
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
                        await uasyncio.sleep_ms(1)
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
