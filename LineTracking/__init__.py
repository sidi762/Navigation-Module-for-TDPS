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

import image, time

class LineTracking:

    def __init__(self, sensor, kernal_size = 1, kernal = [-3, +0, +1, -4, +8, +2, -3, -2, +1], thresholds = [(100, 255)]):
        self.sensor = sensor
        self.kernal_size = kernal_size
        self.kernal = kernal
        self.thresholds = thresholds
        self.sensorSettings = {'pixformat': 0, 'framesize': 0}
        self.isStarted = False


    def start(self):
        self._store_sensor_settings()
        self._set_sensor_for_line_detection()
        self.isStarted = True

    def end(self):
        # Resume sensor settings
        self.sensor.set_pixformat(self.sensorSettings.pixformat)
        self.sensor.set_framesize(self.sensorSettings.framesize)
        self.sensor.skip_frames(time = 100)     # Wait for settings take effect.
        self.isStarted = False

    def _set_sensor_for_line_detection(self):
        self.sensor.set_pixformat(self.sensor.GRAYSCALE) # Set pixel format to GRAYSCALE
        self.sensor.set_framesize(self.sensor.HQVGA)
        # if (self.sensor.get_id() == self.sensor.OV7725): # Set the sharpness/edge register for OV7725
        #    print("Using OV7725")
        #    self.sensor.__write_reg(0xAC, 0xDF)
        #    self.sensor.__write_reg(0x8F, 0xFF)
        self.sensor.skip_frames(time = 100)     # Wait for settings take effect.

    def _store_sensor_settings(self):
        self.sensorSettings['pixformat'] = self.sensor.get_pixformat()
        self.sensorSettings['framesize'] = self.sensor.get_framesize()


    def _apply_filter(self, img, threshold=True):
        img.morph(self.kernal_size, self.kernal, threshold=threshold, invert=True)
        img.binary(self.thresholds)
        img.erode(1, threshold = 2)
        return img


    def get_line(self):
        if not self.isStarted:
            print("Error: Set the camera first by calling LineTracking.start()")
            return 0
        img = self.sensor.snapshot()
        img = self._apply_filter(img)
        line = img.get_regression([(255, 255)], robust=True)

        if line is not None:
            img.draw_line(line.line(), color=(255,255,0))

        return line
