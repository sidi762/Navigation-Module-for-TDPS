import time, sensor, image
from image import SEARCH_EX, SEARCH_DS

def arrow_detection():
    # Reset sensor
    sensor.reset()

    # Set sensor settings
    sensor.set_contrast(1)
    sensor.set_gainceiling(16)
    # Max resolution for template matching with SEARCH_EX is QQVGA
    sensor.set_framesize(sensor.QQVGA)
    # You can set windowing to reduce the search image.
    #sensor.set_windowing(((640-80)//2, (480-60)//2, 80, 60))
    sensor.set_pixformat(sensor.GRAYSCALE)

    # Load template.
    # Template should be a small (eg. 32x32 pixels) grayscale image.
    templates = ["/0.pgm", "/1.pgm", "/2.pgm", "/6.pgm"] #保存多个模板

    clock = time.clock()

    # Run template matching
    while (True):
        clock.tick()
        img = sensor.snapshot()

        # find_template(template, threshold, [roi, step, search])
        # ROI: The region of interest tuple (x, y, w, h).
        # Step: The loop step used (y+=step, x+=step) use a bigger step to make it faster.
        # Search is either image.SEARCH_EX for exhaustive search or image.SEARCH_DS for diamond search
        #
        # Note1: ROI has to be smaller than the image and bigger than the template.
        # Note2: In diamond search, step and ROI are both ignored.
        for t in templates:
            template = image.Image(t)
            #对每个模板遍历进行模板匹配
            r = img.find_template(template, 0.85, step=4, search=SEARCH_EX) #, roi=(10, 0, 60, 60))
            #find_template(template, threshold, [roi, step, search]),threshold中
            #的0.85是相似度阈值,roi是进行匹配的区域（左上顶点为（10，0），长80宽60的矩形），
            #注意roi的大小要比模板图片大，比frambuffer小。
            #把匹配到的图像标记出来
            if r:
                img.draw_rectangle(r)
                return t
