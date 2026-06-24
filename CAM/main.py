# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Multi Color Code Tracking Example
#
# This example shows off multi color code tracking using the OpenMV Cam.
#
# A color code is a blob composed of two or more colors. The example below will
# only track colored objects which have two or more the colors below in them.
import sensor, time, math, ustruct
from pyb import UART, LED

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)
ir_led = LED(4)

red_led.off()
green_led.off()
blue_led.off()
ir_led.off()

uart = UART(3, 460800)
uart.init(460800, bits=8, parity=None, stop=1, timeout_char=1400)
start_bit = 254
stop_bit = 255

# # Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# Blue_thresholds = [(19, 51, -21, 11, -65, -10)]   #old
# Yellow_thresholds = [(27, 87, -5, 35, 73, 12)]    #old
# Yellow_thresholds = [(27, 87, -2, 60, 75, 12)]    #old

# Blue_thresholds = [(19, 45, -21, 15, -65, -8)]   #new
# Blue_thresholds = [(32, 62, -41, 38, -36, -17)]     #kantou
# Blue_thresholds = [(20, 100, -21, 40, -100, -26)]  #house
# Yellow_thresholds = [(27, 87, -5, 35, 73, 12)]    # new
# Yellow_thresholds = [(51, 100, -18, 61, 70, 13)]    #new
# Yellow_thresholds = [(51, 100, -80, 56, 10, 70)]    #kantou
# Yellow_thresholds = [(87, 100, -18, 61, 70, 13)]    #house

# Blue_thresholds = [(16, 88, -68, 95, -72, -15)] #274
# Yellow_thresholds = [(51, 100, -80, 52, 86, 25)] #274

Blue_thresholds = [(16, 88, -68, 95, -72, -15)] #110
Yellow_thresholds = [(83, 98, -64, 52, 14, 92)] #110

# Yellow_thresholds = [(34, 100, -77, 48, 94, 39)] #honban
# Blue_thresholds = [(72, 92, -60, 24, -75, -12)] #honban
# Blue_thresholds = [(38, 83, -33, 127, -128, -20)] #honban2

# Blue_thresholds = [(38, 100, -49, 127, -98, -18)] #Acoat
# Yellow_thresholds = [   (61, 100, -76, 48, 105, 22)] #Acoat

# Blue_thresholds = [(36, 100, -49, 127, -97, -16)] #AcoatB
# Yellow_thresholds = [(61, 100, -76, 48, 105, 24)] #ACcoat

# Blue_thresholds = [(28, 100, -49, 127, -96, -18)] #AcoatB
# Yellow_thresholds = [(61, 100, -76, 48, 105, 24)] #ACcoat


# Blue_thresholds = [(23, 100, -33, 127, -73, -18)] #fcoatb
# Yellow_thresholds = [(61, 100, -72, 47, 86, 29)] #fcoatb


# Blue_thresholds = [(33, 100, -36, 127, -66, -20)] #bcoat
# Blue_thresholds = [(17, 100, -3, 127, -52, -18)] #bcoat
# Yellow_thresholds = [(62, 100, -74, 47, 105, 34)] #bcoat

# Blue_thresholds = [(21, 100, -33, 127, -73, -20)] #bcoatb
# Yellow_thresholds = [(61, 100, -74, 48, 105, 23)] #bcoatb


# Blue_thresholds = [(26, 100, -21, 127, -73, -22)] #bcoat
# Yellow_thresholds = [(61, 100, -74, 48, 105, 29)] #bcoat

# Blue_thresholds = [(21, 100, -33, 127, -73, -20)] #bcoatb
# Yellow_thresholds = [(61, 100, -74, 48, 105, 23)] #bcoatb

# Blue_thresholds = [(48, 100, -50, 127, -99, -17)]
# Yellow_thresholds = [   (61, 100, -76, 48, 105, 22)] #default

# Yellow_thresholds = [(95, 71, -42, 0, 94, 48)] #hotel
# Blue_thresholds = [(57, 40, -56, 48, -46, 0)] #hotel

# Blue_thresholds = [(24, 61, -20, 28, -56, -15)] #2025nest temae
# Yellow_thresholds = [(57, 100, -44, 66, 23, 78)] #2025nest temae

# Blue_thresholds = [(23, 48, -25, 17, -49, -7)] #2025nest oku
# Yellow_thresholds = [(56, 100, -44, 67, 24, 83)] #2025nest oku

# Blue_thresholds = [(40, 62, -32, 41, -66, -14)] #2026 node
# Yellow_thresholds = [(56, 100, -42, 63, 24, 87)] #2026 node

# Blue_thresholds = [(29, 53, -21, 50, -73, -23)] #expo
# Yellow_thresholds = [(67, 97, -44, 50, 23, 70)] #expo

# Blue_thresholds = [(23, 47, -70, 127, -52, -17)] #2026 japan G
# Yellow_thresholds = [(61, 100, -76, 48, 105, 22)] #2026 japan G


# Yellow_thresholds = [(66, 100, -83, 64, 113, 33)] #kanyuu
# Blue_thresholds = [(39, 100, -49, 53, -81, -10)] #kanyuu



IMG_CX = 163        #QVGA
IMG_CY = 120       #QVGA

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# sensor.set_auto_gain(False)  # must be turned off for color tracking
# sensor.set_auto_whitebal(False)  # must be turned off for color tracking
# sensor.set_auto_exposure(True)

sensor.set_auto_gain(False)         # 色追跡では基本OFF
sensor.set_auto_whitebal(False)     # 色追跡では基本OFF
sensor.set_auto_exposure(False, exposure_us=10000)  # ←ここ
sensor.set_brightness(1)            # 必要なら追加
sensor.skip_frames(time=1000)       # 設定反映待ち
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    img.draw_cross(IMG_CX, IMG_CY)

    Blue = img.find_blobs(Blue_thresholds, pixels_threshold=60, area_threshold=60)
    Yellow = img.find_blobs(Yellow_thresholds, pixels_threshold=60, area_threshold=60)

    if Blue:
        for blob in Blue:
            # img.draw_rectangle(blob.rect())
            # img.draw_cross(blob.cx(), blob.cy())

            Blue_M = max(Blue, key=lambda b: b.pixels())    #get laregest area

            img.draw_rectangle(Blue_M.rect(), color=(255, 0, 0))
            img.draw_cross(Blue_M.cx(), Blue_M.cy(), color=(255, 0, 0))
            img.draw_circle(Blue_M.cx(), Blue_M.cy() ,10, color=(255, 0, 0), thickness=1)

            Blue_width = Blue_M.w()
            Blue_angle = math.atan2(IMG_CX - Blue_M.cx(), IMG_CY - Blue_M.cy()) * 180 / math.pi + 180   #angle

            bx, by, bw, bh = Blue_M.rect()
            corners = [
                (bx, by),
                (bx + bw, by),
                (bx, by + bh),
                (bx + bw, by + bh),
            ]
            corners.sort(key=lambda p: (p[0] - IMG_CX) ** 2 + (p[1] - IMG_CY) ** 2)
            selected = corners[1:3]
            corner_angles = []
            for cx, cy in selected:
                angle = math.atan2(IMG_CX - cx, IMG_CY - cy) * 180 / math.pi + 180
                corner_angles.append(angle)
                img.draw_rectangle(cx - 2, cy - 2, 5, 5, color=(0, 255, 0))
            if len(corner_angles) == 2:
                Blue_angle_range = abs(corner_angles[1] - corner_angles[0])
                if Blue_angle_range > 180:
                    Blue_angle_range = 360 - Blue_angle_range
                dx = Blue_M.cx() - IMG_CX
                dy = Blue_M.cy() - IMG_CY
                Blue_radius = math.sqrt(dx * dx + dy * dy)

                # if radius != 0:
                #     half_rad = math.radians(Blue_angle_range / 2)
                #     for sign in (1, -1):
                #         cos_t = math.cos(sign * half_rad)
                #         sin_t = math.sin(sign * half_rad)
                #         rx = dx * cos_t - dy * sin_t
                #         ry = dx * sin_t + dy * cos_t
                #         px = int(IMG_CX + rx)
                #         py = int(IMG_CY + ry)
                #         img.draw_rectangle(px - 2, py - 2, 5, 5, color=(255, 0, 255))
    else:
        Blue_angle = 180
        Blue_width = 0
        Blue_radius = 250
        Blue_angle_range = 0

    if Yellow:
        for blob in Yellow:
            # img.draw_rectangle(blob.rect())
            # img.draw_cross(blob.cx(), blob.cy())

            Yellow_M = max(Yellow, key=lambda b: b.pixels())    #get laregest area

            img.draw_rectangle(Yellow_M.rect(), color=(255, 0, 0))
            img.draw_cross(Yellow_M.cx(), Yellow_M.cy(), color=(255, 0, 0))
            img.draw_circle(Yellow_M.cx(), Yellow_M.cy() ,10, color=(255, 0, 0), thickness=1)

            Yellow_width = Yellow_M.w()
            Yellow_angle = math.atan2(IMG_CX - Yellow_M.cx(), IMG_CY - Yellow_M.cy()) * 180 / math.pi + 180    #angle

            yx, yy, yw, yh = Yellow_M.rect()
            ycorners = [
                (yx, yy),
                (yx + yw, yy),
                (yx, yy + yh),
                (yx + yw, yy + yh),
            ]
            ycorners.sort(key=lambda p: (p[0] - IMG_CX) ** 2 + (p[1] - IMG_CY) ** 2)
            yselected = ycorners[1:3]
            ycorner_angles = []
            for cx, cy in yselected:
                angle = math.atan2(IMG_CX - cx, IMG_CY - cy) * 180 / math.pi + 180
                ycorner_angles.append(angle)
                img.draw_rectangle(cx - 2, cy - 2, 5, 5, color=(0, 255, 0))
            if len(ycorner_angles) == 2:
                Yellow_angle_range = abs(ycorner_angles[1] - ycorner_angles[0])
                if Yellow_angle_range > 180:
                    Yellow_angle_range = 360 - Yellow_angle_range
                dx = Yellow_M.cx() - IMG_CX
                dy = Yellow_M.cy() - IMG_CY
                Yellow_radius = math.sqrt(dx * dx + dy * dy)

                # if radius != 0:
                #     half_rad = math.radians(Yellow_angle_range / 2)
                #     for sign in (1, -1):
                #         cos_t = math.cos(sign * half_rad)
                #         sin_t = math.sin(sign * half_rad)
                #         rx = dx * cos_t - dy * sin_t
                #         ry = dx * sin_t + dy * cos_t
                #         px = int(IMG_CX + rx)
                #         py = int(IMG_CY + ry)
                #         img.draw_rectangle(px - 2, py - 2, 5, 5, color=(255, 0, 255))
    else:
        Yellow_angle = 180
        Yellow_width = 0
        Yellow_radius = 250
        Yellow_angle_range = 0

    # if Blue:
        # print("width:", Blue_width, "angle:", int(Blue_angle*(start_bit-1)/360))
        print("width:", Blue_width,int(Blue_radius),int(Blue_angle_range), "angle:", int(Blue_angle))

    # if Yellow:
        # print("Yellow max area position x,y:", Yellow_M.cx(), Yellow_M.cy(), "width:", Yellow_width, "angle:", Yellow_angle)

    try:
        uart.write(ustruct.pack('B',start_bit))
        uart.write(ustruct.pack('B',int(Blue_angle*start_bit/360)))
        uart.write(ustruct.pack('B',Blue_width))
        uart.write(ustruct.pack('B',int(Yellow_angle*start_bit/360)))
        uart.write(ustruct.pack('B',Yellow_width))
        uart.write(ustruct.pack('B',int(Blue_angle_range)))
        uart.write(ustruct.pack('B',int(Blue_radius)))
        uart.write(ustruct.pack('B',int(Yellow_angle_range)))
        uart.write(ustruct.pack('B',int(Yellow_radius)))
        uart.write(ustruct.pack('B',stop_bit))
    except OSError as error:
        pass

#    print(clock.fps())
