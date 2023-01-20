# Multi Color Blob Tracking

import sensor, image, time, math,pyb
from pyb import UART

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green things. You may wish to tune them...

thresholds = [(0, 42, 23, 61, 20, 52), # generic_red_thresholds
              (0, 100, 14, 127, -9, 13), # generic_pink_thresholds (GAMEBALL COLOR)
              (3, 66, -24, 32, -76, -28)] # generic_blue_thresholds

# You may pass up to 16 thresholds above. However, it's not really possible to segment any
# scene with 16 thresholds before color thresholds start to overlap heavily.


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.set_auto_exposure(False)
sensor.set_saturation(2)
sensor.set_brightness(-3)


led1 = pyb.LED(1)
led2 = pyb.LED(2)
led3 = pyb.LED(3)
led4 = pyb.LED(4)

uart = UART(3, 115200, timeout_char=1000)                         # init with given baudrate
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000) # init with given parameters

adc = pyb.ADC(pyb.Pin('P6'))

#led.off()
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. Don't set "merge=True" becuase that will merge blobs which we don't want here.

while(True):
    clock.tick()
    img = sensor.snapshot()

    #bloblist object that contains all the blobs
    blobList = img.find_blobs(thresholds, pixels_threshold=50, area_threshold=500)  #Threshhold number for resolution(distance)
    #the red,green,blue arrays to hold the index of the biggest blob and the area of the blobs
    #for comparison
    redBlob = [1000,1000,0]
    pinkBlob = [1000,1000,0] #Game ball
    blueBlob =[1000,1000,0]


   #loop over the list, compare all the pixels and return the biggest one
    for blob in blobList:
        if blob.elongation() < 0.8:
            if blob.density() > 0.3:
    #find the biggest pixel area for each color, and center the camera based on the color we pick
        #code = 1 is red
        #code = 2 is pink
        #code = 4 is blue
        #catogerize color
                if blob.code() == 1:
                    if blob.pixels() > redBlob[2]:
                        redBlob = [blob.cxf(), blob.cyf(), blob.pixels()]
                elif blob.code() == 2:
                    if blob.pixels()> pinkBlob[2]:
                        pinkBlob = [blob.cxf(), blob.cyf(), blob.pixels()]
                elif blob.code() == 4:
                    if blob.pixels()> blueBlob[2]:
                        blueBlob = [blob.cxf(), blob.cyf(), blob.pixels()]

                # These values depend on the blob not being circular - otherwise they will be shaky.
                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255,0,0))
                    img.draw_line(blob.major_axis_line(), color=(0,255,0))
                    img.draw_line(blob.minor_axis_line(), color=(0,0,255))

                # These values are stable all the time.
                img.draw_rectangle(blob.rect())
                img.draw_cross(blob.cx(), blob.cy())
                # Note - the blob rotation is unique to 0-180 only.
                img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

                #print(blobList)
                #print(clock.fps())
    uart.write("%f"%redBlob[0])
    uart.write(',')
    uart.write("%f"%redBlob[1])
    uart.write(',')
    uart.write('rr,')
    uart.write(';')
    uart.write("%f"%pinkBlob[0])
    uart.write(',')
    uart.write("%f"%pinkBlob[1])
    uart.write(',')
    uart.write('gg,')
    uart.write(';')
    uart.write("%f"%blueBlob[0])
    uart.write(',')
    uart.write("%f"%blueBlob[1])
    uart.write(',')
    uart.write('bb,')
    uart.write(';')

    uart.write(str(adc.read()/8.2758))
    uart.write(';!\n')

    #print("%f\n"%redBlob[0], end='')
    #print("%f\n"%redBlob[1], end='')
    #print("r")
    #print("%f\n"%pinkBlob[0], end='')
    #print("%f\n"%pinkBlob[1], end='')
    #print("p")
    #print("%f\n"%blueBlob[0], end='')
    #print("%f\n"%blueBlob[1], end='')
    #print("b")

    print((adc.read()/8.2758))



    #LED indicator
    #blobs = len(blobList)
    #if blobs>1:
    led1.on()
    led2.on()
    led3.on()
    #led4.on()
    #else:
       #led.off()






