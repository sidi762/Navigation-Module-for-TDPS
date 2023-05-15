#
# OpenMV Software for TDPS self-driving vehicle project
# HCSR04 Ultrasonic Sensor Module
# Sidi Liang, 2023
#

import pyb

class HCSR04:

    def __init__(self, trig, echo):
        """
            Usage: sensor = HCSR04(trig = Pin('P2', Pin.OUT_PP), echo = Pin('P1', Pin.IN, Pin.PULL_DOWN))
        """
        self.trig = trig
        self.echo = echo

    def get_distance(self):
        """
            Usage: distance = sensor.getDistance()
            returns the distance in CM
        """
        pulse_start, pulse_end, pulse_dur = 0, 0, 0
        self.trig.value(1)
        pyb.udelay(5)
        self.trig.value(0)
        pyb.udelay(10)
        self.trig.value(1)
        limit = 20000 # Limit to prevent lag
        timer = pyb.micros()
        while self.echo.value() == 0 and timer+limit > pyb.micros():
            pulse_start = pyb.micros()
        timer = pyb.micros()
        while self.echo.value() == 1 and timer+limit > pyb.micros():
            pulse_end = pyb.elapsed_micros(pulse_start)
        pulse_dur = float(pulse_end)
        distance = (pulse_dur) / 50  # Todo: calibration
        if distance > 300: # Max distance is 300
            distance = 300
        return distance
