# OpenMV ball detection template.
# Calibrate BALL_THRESHOLD, ORIGIN_X_PX, and PX_PER_CM on the real beam.

import sensor
import time
from pyb import UART

UART_PORT = 3
UART_BAUD = 115200

BALL_THRESHOLD = (0, 100, -20, 20, -20, 20)  # TODO: calibrate.
ORIGIN_X_PX = 160
PX_PER_CM = 12.0
MIN_PIXELS = 30

uart = UART(UART_PORT, UART_BAUD, timeout_char=1000)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

def send_ball(timestamp_ms, x_cm, quality, valid):
    x100 = int(x_cm * 100)
    uart.write("<B,{},{},{},{}>\n".format(timestamp_ms, x100, quality, 1 if valid else 0))

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([BALL_THRESHOLD], pixels_threshold=MIN_PIXELS, area_threshold=MIN_PIXELS)

    best = None
    for blob in blobs:
        if best is None or blob.pixels() > best.pixels():
            best = blob

    now = time.ticks_ms()
    if best is None:
        send_ball(now, 0.0, 0, False)
        continue

    img.draw_rectangle(best.rect(), color=(255, 0, 0))
    img.draw_cross(best.cx(), best.cy(), color=(0, 255, 0))

    x_cm = (best.cx() - ORIGIN_X_PX) / PX_PER_CM
    quality = min(100, best.pixels())
    send_ball(now, x_cm, quality, True)

