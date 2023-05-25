#
# OpenMV Software for TDPS self-driving vehicle project
# UART Messaging
# Sidi Liang, 2023
#

import json, uasyncio

class OpenMV_MessageHandler:
    def __init__(self, uart, status_data_ref, print_debug=0):
        self._uart = uart
        self._last_message = 'nil'
        self._status_data_ref = status_data_ref
        self._swriter = uasyncio.StreamWriter(uart, {})
        self._sreader = uasyncio.StreamReader(uart)
        self._task = uasyncio.create_task(self.readwrite())
        self._master_is_ready = 0
        self._print_debug = print_debug


    _encoder_data = {'Info_Encoder_A': "0",
                     'Info_Encoder_B': "0"}

    _feedback_data = {'Info_Cam_Pitch': 0,
                      'Info_Ball': 0,
                      'Info_Turning': 0,
                      'Info_Comm': 0}

    def _debug_print(self, *args, **kwargs):
        '''
        Prints the debug log if self._print_debug is set to 1
        '''
        if self._print_debug == 1:
            print(*args, **kwargs)

    async def _uart_messaging_json(self, status_data):
        data_json = json.dumps(status_data)
        uart_send_buffer = b'\xaa\x55' \
                           + len(data_json).to_bytes(1, 'big') \
                           + bytes(data_json, 'utf-8')
        self._debug_print("UART sent: ", uart_send_buffer)
        self._debug_print("UART len: ", len(uart_send_buffer))
        last_message = uart_send_buffer
        return uart_send_buffer

    async def _update_encoder_data(self, data):
        try:
            self._encoder_data['Info_Encoder_A'] = data['Info_Encoder_A']
            self._encoder_data['Info_Encoder_B'] = data['Info_Encoder_B']
        except:
            return False
        return True

    async def _update_feedback_data(self, data):
        try:
            self._feedback_data['Info_Cam_Pitch'] = data['Info_Cam_Pitch']
            self._feedback_data['Info_Ball'] = data['Info_Ball']
            self._feedback_data['Info_Comm'] = data['Info_Comm']
            self._feedback_data['Info_Turning'] = data['Info_Turning']
        except:
            return False
        return True

    async def _uart_recv_json(self, rcv_buffer):
        data = ''
        self._debug_print("Processing rcvbuffer: ", rcv_buffer)
        success = True
        try:
            data = json.loads(rcv_buffer)
        except ValueError as err:
            self._debug_print("ValueError! Received json string invalid")
            self._debug_print("Message causing error: ", rcv_buffer)
            return False

        ret = await self._update_encoder_data(data)
        if not ret:
            self._debug_print("encoder data failed to update, check data")
            self._debug_print("Message causing error: ", data)
            success = False

        ret = await self._update_feedback_data(data)
        if not ret:
            self._debug_print("feedback data failed to update, check data")
            self._debug_print("Message causing error: ", data)
            success = False

        return success


    def get_encoder_data(self):
        '''
            Returns a copy of the encoder data
        '''
        return self._encoder_data.copy()

    def get_feedback_data(self):
        '''
            Returns a copy of the encoder data
        '''
        return self._feedback_data.copy()

    def master_is_ready(self):
        '''
            Returns True if the master is ready to send data
        '''
        return self._last_message != 'nil'


    def cancel(self):
        '''
            Cancels the coroutine
        '''
        self._task.cancel()

    async def readwrite(self):
        '''
            Coroutine handling UART communication with the master control.
        '''
        swriter = self._swriter
        sreader = self._sreader

        try:
            while True:
                last_message = self._last_message
                status_data = self._status_data_ref
                self._debug_print('Waiting for incoming message...')
                rcvbuf = ''
                rcv = await sreader.read(1)
                if rcv:
                    self._debug_print('Received: ', rcv)
                    buf = last_message
                    if rcv == b'\xcd':
                        # 0xcd: last message correctly received,
                        #       openMV should send next message
                        buf = await self._uart_messaging_json(status_data)
                        self._last_message = buf
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
                        self._debug_print('Received: ', rcv)
                        if rcv == b'\x55':
                            rcvlen = await sreader.read(1)
                            rcvlen = int.from_bytes(rcvlen, 'big')
                            self._debug_print("rcvlen: ", rcvlen)
                            if rcvlen == 1:
                                self._debug_print("Master control ready")
                                self._master_is_ready = 1
                                await uasyncio.sleep_ms(1)
                                continue
                            rcvbuf = await sreader.read(rcvlen)
                            if await self._uart_recv_json(rcvbuf):
                                buf = b'\xcc' # Message correctly parsed
                            else:
                                # Message cannot be parsed correctly, possible
                                # corruption in transmission
                                buf = b'\xdd'
                                await uasyncio.sleep_ms(1)
                        else:
                            # Does not match the protocol, possible
                            # corruption in transmission
                            buf = b'\xdd'
                            await uasyncio.sleep_ms(1)

                    else:
                        # Unspecified in protocol, possible
                        # corruption in transmission
                        buf = b'\xdd'
                        await uasyncio.sleep_ms(1)

                    await swriter.awrite(buf)
                    await swriter.drain()
                    await sreader.drain()
                    self._debug_print('Sent: ', buf)
                    await uasyncio.sleep_ms(5)
                #End If
            #End While
        except uasyncio.CancelledError:
            print('Comm. task cancelled.')
