# AprilTag test
#

import sensor, image, time, math

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.HQVGA) # we run out of memory if the resolution is much bigger...
sensor.skip_frames(30)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
sensor.set_vflip(True)
sensor.set_hmirror(True)
clock = time.clock()


while(True):
    clock.tick()
    img = sensor.snapshot()
    for tag in img.find_apriltags(): # defaults to TAG36H11 without "families".
        img.draw_rectangle(tag.rect(), color = (255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
        degress = 180 * tag.rotation() / math.pi
        center_coord = img.width() / 2
        cx = tag.cx() - center_coord
        print(tag.id(), degress, cx)
