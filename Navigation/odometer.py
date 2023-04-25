#
# OpenMV Software for TDPS self-driving vehicle project
# The Line Tracking Module
# Odometer
# Sidi Liang, 2023
#



import time

class Odometer:

    def __init__(self):
        self._time_last_update = 0
        self._encoder_A = 0
        self._encoder_B = 0
        self._odometer = 0


    def update_with_encoder_data(self, encoder_A, encoder_B):
        self._encoder_A = encoder_A
        self._encoder_B = encoder_B
        self._update()

    def _update(self):
        time_now = time.time_ns() / 1000000 #ms
        interval = (time_now - self._time_last_update) / 1000 #seconds
        v_dx = self._encoder_A # Placeholder
        self._odometer += v_dx * interval
        self._time_last_update = time_now

    def get_odometer(self):
        return self._odometer
