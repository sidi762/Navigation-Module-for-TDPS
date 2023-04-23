#
# OpenMV Software for TDPS self-driving vehicle project
# UART Messaging
# Sidi Liang, 2023
#

import json, uasyncio

class OpenMV_MessageHandler:
    def __init__(self, uart, status_data_ref):
        self._uart = uart
        self.last_message = 'nil'
        self.status_data_ref = status_data_ref
        self.swriter = uasyncio.StreamWriter(uart, {})
        self.sreader = uasyncio.StreamReader(uart)
        self._task = uasyncio.create_task(self.readwrite())


    _encoder_data = {'Info_Encoder_A': "0",
                     'Info_Encoder_B': "0",
                     'Info_Encoder_C': "0",
                     'Info_Encoder_D': "0"}

    async def _uart_messaging_json(self, status_data):
        data_json = json.dumps(status_data)
        uart_send_buffer = b'\xaa\x55' \
                           + len(data_json).to_bytes(1, 'big') \
                           + bytes(data_json, 'utf-8')
        print("UART sent: ", uart_send_buffer)
        print("UART len: ", len(uart_send_buffer))
        last_message = uart_send_buffer
        return uart_send_buffer

    async def _update_encoder_data(self, data):
        try:
            self._encoder_data['Info_Encoder_A'] = data['Info_Encoder_A']
            self._encoder_data['Info_Encoder_B'] = data['Info_Encoder_B']
            self._encoder_data['Info_Encoder_C'] = data['Info_Encoder_C']
            self._encoder_data['Info_Encoder_D'] = data['Info_Encoder_D']
        except:
            return False
        return True

    async def _uart_recv_json(self, rcv_buffer):
        data = ''
        print("Processing rcvbuffer: ", rcv_buffer)
        try:
            data = json.loads(rcv_buffer)
        except ValueError as err:
            print("ValueError! Received json string invalid")
            print("Message causing error: ", rcv_buffer)
            return False

        if await self._update_encoder_data(data):
            return True
        else:
            print("Encoder_data failed to update, check data")
            print("Message causing error: ", data)
            return False

    def get_encoder_data(self):
        '''
        Returns a copy of the encoder data
        '''
        return self._encoder_data.copy()

    def cancel(self):
        self._task.cancel()

    async def readwrite(self):
        '''
            Coroutine handling UART communication with the master control.
        '''
        swriter = self.swriter
        sreader = self.sreader

        try:
            while True:
                last_message = self.last_message
                status_data = self.status_data_ref
                print('Waiting for incoming message...')
                rcvbuf = ''
                rcv = await sreader.read(1)
                #rcv = self._uart.read(1)
                if rcv:
                    print('Received: ', rcv)
                    buf = last_message
                    if rcv == b'\xcd':
                        # 0xcd: last message correctly received,
                        #       openMV should send next message
                        buf = await self._uart_messaging_json(status_data)
                        self.last_message = buf
                    elif rcv == b'\xcc':
                        # 0xcd: last message correctly received
                        await uasyncio.sleep_ms(1)
                        continue
                    elif rcv == b'\xdd':
                        # 0xdd: error in the last transmission,
                        #       resend message
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
                            if await self._uart_recv_json(rcvbuf):
                                buf = b'\xcc' # Message correctly parsed
                            else:
                                # Message cannot be parsed correctly, possible
                                # corruption in transmission
                                buf = b'\xdd'
                        else:
                            # Does not match the protocol, possible
                            # corruption in transmission
                            buf = b'\xdd'

                    else:
                        # Unspecified in protocol, possible
                        # corruption in transmission
                        buf = b'\xdd'
                        await uasyncio.sleep_ms(1)

                    await swriter.awrite(buf)
                    await swriter.drain()
                    await sreader.drain()
                    print('Sent: ', buf)
                    await uasyncio.sleep_ms(5)
                #End If
            #End While
        except asyncio.CancelledError:
            print('Comm. task cancelled.')
