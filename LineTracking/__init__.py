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

import sensor, image, time

#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.skip_frames(time = 2000)

#clock = time.clock()




class LineTracking:

    def __init__(self, kernal_size = 1, kernal = [-3, +0, +1, -4, +8, +2, -3, -2, +1], thresholds = [(100, 255)]):
        self.kernal_size = kernal_size
        self.kernal = kernal
        self.thresholds = thresholds
        return 0


    def setSensorForLineDetection(sensor):
        sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to GRAYSCALE
        sensor.set_framesize(sensor.HQVGA)
        if (sensor.get_id() == sensor.OV7725): # Set the sharpness/edge register for OV7725
            print("Using OV7725")
            sensor.__write_reg(0xAC, 0xDF)
            sensor.__write_reg(0x8F, 0xFF)
        sensor.skip_frames(time = 100)     # Wait for settings take effect.

    def __applyFilter__(img, threshold=True):
        img.morph(self.kernel_size, self.kernel, threshold=threshold, invert=True)
        img.binary(thresholds)
        img.erode(1, threshold = 2)
        return img


    def getLine(img):
        #Todo: check if sensor is set
        img = self.__applyFilter__(img)
        line = img.get_regression([(255, 255)], robust=True)
        return line
