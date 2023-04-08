#
# OpenMV Software for TDPS self-driving vehicle project
# The Line Tracking Module
# Sidi Liang, 2023
#

#Notes (to be removed):
#kernel_size = 1 # kernel width = (size*2)+1, kernel height = (size*2)+1
#kernel = [-9, +9, +11,\
#          -23, +40, +13,\
#          -21, -19, -1]

#kernel = [-3, +0, +1,\
#          -4, +8, +2,\
#          -3, -2, +1]
#thresholds = [(100, 255)]

import image, time, pid

class LineTracking:

    def __init__(self, sensor, kernal_size = 1, kernal = [-3, +0, +1, -4, +8, +2, -3, -2, +1], thresholds = [(100, 255)], draw = False):
        """
            Class for line tracking
            Arguments:
                sensor: openMV sensor
                kernal_size: The kernal size for filtering,
                             calculated by kernel width = (size*2)+1, kernel height = (size*2)+1,
                             defaults to 1 (3x3 kernal)
                kernal: The kernal for filtering, defaults to [-3, +0, +1, -4, +8, +2, -3, -2, +1]
                thresholds: The thresholds for binary filtering, defaults to [(100, 255)]
                draw: Set to True to draw the resulting line on the captured image, defaults to False

            Usage:
                line_tracking = LineTracking(sensor)
                line_tracking.start()
                while(True):
                    line = line_tracking.get_line()
                line_tracking.stop()
        """
        self.sensor = sensor
        self.kernal_size = kernal_size
        self.kernal = kernal
        self.thresholds = thresholds
        self.sensorSettings = {'pixformat': 0, 'framesize': 0}
        self.isStarted = False
        self.draw = draw


    def start(self):
        """
            Stores the current state of the sensor and sets the sensor for line tracking
        """
        self._store_sensor_settings()
        self._set_sensor_for_line_detection()
        img = self.sensor.snapshot()
        self.center_coord = img.width() / 2
        self.isStarted = True

    def end(self):
        """
            Resumes sensor to the stored settings
        """
        # Resume sensor settings
        sens = self.sensor
        sens.set_pixformat(self.sensorSettings.pixformat)
        sens.set_framesize(self.sensorSettings.framesize)
        sens.skip_frames(time = 100)     # Wait for settings take effect.
        self.isStarted = False

    def _set_sensor_for_line_detection(self):
        sens = self.sensor
        sens.set_pixformat(self.sensor.GRAYSCALE) # Set pixel format to GRAYSCALE
        sens.set_framesize(self.sensor.HQVGA)
        # if (self.sensor.get_id() == self.sensor.OV7725): # Set the sharpness/edge register for OV7725
        #    print("Using OV7725")
        #    sens.__write_reg(0xAC, 0xDF)
        #    sens.__write_reg(0x8F, 0xFF)
        sens.skip_frames(time = 100)     # Wait for settings take effect.

    def _store_sensor_settings(self):
        self.sensorSettings['pixformat'] = self.sensor.get_pixformat()
        self.sensorSettings['framesize'] = self.sensor.get_framesize()


    def _apply_filter(self, img, threshold=True):
        img.morph(self.kernal_size, self.kernal, threshold=threshold, invert=True)
        img.binary(self.thresholds)
        img.erode(1, threshold = 2)
        return img

    def _capture_filter_and_calculate_line(self):
        if not self.isStarted:
            print("Error: Set the camera first by calling LineTracking.start()")
            return 0
        img = self.sensor.snapshot()
        img = self._apply_filter(img)
        line = img.get_regression([(255, 255)], robust=True)
        self.img = img
        if line is not None:
            if self.draw:
                img.draw_line(line.line(), color=(255,255,0))
            self.calculatedLine = line
            return line

    def _line_error(self, line):
        if line:
            rho_err = abs(line.rho())-self.center_coord
            if line.theta()>90:
                theta_err = line.theta()-180
            else:
                theta_err = line.theta()
            self.rho_err, self.theta_err = rho_err, theta_err
            return rho_err, theta_err
        else
            return 0

    def calculate(self):
        line = self._capture_filter_and_calculate_line()
        rho_err, theta_err = self._line_error(line)
        return rho_err, theta_err

    def get_line(self):
        """
            Returns the detected line as a Line object. Returns None if no line is detected.
        """
        if self.calculatedLine:
            return self.calculatedLine
        else return None
