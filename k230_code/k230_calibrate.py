"""
K230 calibration tool WITH WiFi streaming + false-detection filtering.

Run this on K230 to calibrate pixel-to-cm mapping.
Streams video to PC so you can SEE the ball position,
and prints YOLO cx coordinates for known positions.

Usage:
  1. Start PC receiver:  python pc_receiver/pc_receiver.py
  2. Mark the pipe with a ruler at -10, -5, 0, +5, +10 cm
  3. Run this script from CanMV IDE
  4. Place ball at -10 cm → read cx from IDE output or PC overlay
  5. Repeat for -5, 0, +5, +10 cm
  6. Update PX_PER_CM and ZERO_X_PX in k230_yolo.py

Wireless: K230 hotspot "test" / "90z5M92#", PC at 192.168.137.1:8888
"""
import sys as _sys
_sys.path.insert(0, '/sdcard/app')

from libs.YOLO import YOLO11
from libs.Utils import *
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA
import image, time, network, socket, gc, os

# ---- Clean up ----
try:    Display.deinit()
except: pass
try:    MediaManager.deinit()
except: pass

# ============ Config ============
MODEL_PATH       = "/sdcard/kmodel/yolo11n_det_320.kmodel"
MODEL_LABELS     = {0: 'steel'}
MODEL_INPUT_SIZE = [320, 320]
RGB888P_SIZE     = [640, 480]
DISPLAY_SIZE     = [640, 480]
CONF_THRESH      = 0.45      # higher for calibration — filter noise
NMS_THRESH       = 0.45
MAX_BOXES        = 10

# WiFi
WIFI_SSID = "test"
WIFI_PASS = "90z5M92#"
PC_IP     = "192.168.137.1"
PC_PORT   = 8888

# Streaming: 640x240 pipe crop, JPEG Q50, ~6fps
PIPE_CROP = (0, 120, 640, 240)
JPEG_Q    = 50
VIDEO_FPS = 6

# Detection ROI: only accept detections in the pipe band
# (filters out edge-of-frame false positives)
ROI_Y_MIN = 100
ROI_Y_MAX = 380
ROI_X_MIN = 20
ROI_X_MAX = 620

# K23V protocol
K23V_MAGIC   = b'K23V'
K23V_VERSION = 1
K23V_JPEG    = 1

print("=" * 50)
print("  K230 CALIBRATION + VIDEO STREAMING")
print("=" * 50)
print("PC receiver must be running first!")
print("Place ball at known marks: -10 -5 0 +5 +10 cm")
print("")

# ---- Camera (same init order as k230_yolo.py) ----
print("[CAL] Init camera...")
sensor = Sensor(id=2, width=640, height=480, fps=60)
sensor.reset()
sensor.set_framesize(w=DISPLAY_SIZE[0], h=DISPLAY_SIZE[1], chn=CAM_CHN_ID_0)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
sensor.set_framesize(w=RGB888P_SIZE[0], h=RGB888P_SIZE[1], chn=CAM_CHN_ID_2)
sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
Display.init(Display.VIRT, DISPLAY_SIZE[0], DISPLAY_SIZE[1], to_ide=True)
MediaManager.init()
print("[CAL]   Camera OK")

# ---- Load model ----
print("[CAL] Loading YOLO model...")
yolo = YOLO11(
    task_type="detect", mode="video",
    kmodel_path=MODEL_PATH, labels=MODEL_LABELS,
    rgb888p_size=RGB888P_SIZE, model_input_size=MODEL_INPUT_SIZE,
    display_size=DISPLAY_SIZE,
    conf_thresh=CONF_THRESH, nms_thresh=NMS_THRESH,
    max_boxes_num=MAX_BOXES, debug_mode=0
)
yolo.config_preprocess()
print("[CAL]   Model OK")

# ---- WiFi ----
print("[CAL] Connecting WiFi to {}...".format(WIFI_SSID))
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASS)
deadline = time.ticks_ms() + 10000
while not wlan.isconnected():
    if time.ticks_diff(time.ticks_ms(), deadline) > 0:
        print("[CAL]   WiFi FAILED — continuing without streaming")
        pc_sock = None
        break
    time.sleep_ms(100)
else:
    print("[CAL]   WiFi OK: {}".format(wlan.ifconfig()[0]))
    # Connect to PC
    try:
        addr = socket.getaddrinfo(PC_IP, PC_PORT)[0][-1]
        pc_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        pc_sock.settimeout(0.5)
        pc_sock.connect(addr)
        pc_sock.setblocking(False)
        print("[CAL]   PC connected at {}:{}".format(PC_IP, PC_PORT))
    except Exception as e:
        print("[CAL]   PC connect FAILED: {}".format(e))
        pc_sock = None

# ---- Start sensor ----
sensor.run()
time.sleep_ms(600)
for _ in range(8):
    try: sensor.snapshot(chn=CAM_CHN_ID_2)
    except: time.sleep_ms(100)

print("[CAL] ========================================")
print("[CAL] READY. Watch PC for pipe view.")
print("[CAL] Place ball at each mark, record cx:")
print("[CAL]   -10cm → cx=??")
print("[CAL]    -5cm → cx=??")
print("[CAL]     0cm → cx=??")
print("[CAL]    +5cm → cx=??")
print("[CAL]   +10cm → cx=??")
print("[CAL] ========================================")
print("[CAL] cx(px) | cy(px) | conf | in_roi")
print("-" * 45)

clock = time.clock()
frame_idx = 0
last_video_ms = 0
video_interval_ms = 1000 // VIDEO_FPS
video_buf = None

while True:
    clock.tick()
    now_ms = time.ticks_ms()

    # 1. YOLO detection from CHN_2
    ai_img = sensor.snapshot(chn=CAM_CHN_ID_2)
    ai_np = ai_img.to_numpy_ref()

    best_cx = 0.0
    best_cy = 0.0
    best_conf = 0.0
    best_w = 0
    best_h = 0
    found = False

    try:
        results = yolo.run(ai_np)
    except:
        results = ([], [], [])

    if results and results[0]:
        for i in range(len(results[0])):
            score = float(results[2][i])
            x, y, w, h = results[0][i]
            cx = float(x) + float(w) / 2.0
            cy = float(y) + float(h) / 2.0

            # ROI filter: only accept detections inside the pipe zone
            in_roi = (ROI_X_MIN <= cx <= ROI_X_MAX and
                      ROI_Y_MIN <= cy <= ROI_Y_MAX)

            if score > best_conf and in_roi:
                best_conf = score
                best_cx = cx
                best_cy = cy
                best_w = int(w)
                best_h = int(h)
                found = True

    # 2. Print detection (every 0.5s)
    frame_idx += 1
    if frame_idx % 15 == 0:
        if found:
            print("[CAL] cx={:.1f}  cy={:.1f}  conf={:.2f}  OK".format(
                best_cx, best_cy, best_conf))
        else:
            print("[CAL] NO BALL (conf>{:.0f} or out of pipe ROI)".format(CONF_THRESH))

    # 3. Video streaming to PC (from CHN_0)
    if pc_sock is not None and time.ticks_diff(now_ms, last_video_ms) >= video_interval_ms:
        try:
            stream_img = sensor.snapshot()  # CHN_0
            pipe_view = stream_img.copy(roi=PIPE_CROP)

            # Draw detection overlay if found
            if found:
                bx = int(best_cx - best_w // 2) - PIPE_CROP[0]
                by = int(best_cy - best_h // 2) - PIPE_CROP[1]
                pipe_view.draw_rectangle(bx, by, best_w, best_h,
                                         color=(255, 0, 0), thickness=2)
                pipe_view.draw_cross(int(best_cx - PIPE_CROP[0]),
                                     int(best_cy - PIPE_CROP[1]),
                                     color=(0, 255, 0), size=12, thickness=2)

            jpeg = bytes(pipe_view.compress(quality=JPEG_Q))

            # K23V header + payload
            length = len(jpeg)
            header = bytearray(10)
            header[0:4] = K23V_MAGIC
            header[4]   = K23V_VERSION
            header[5]   = K23V_JPEG
            header[6]   = (length >> 24) & 0xFF
            header[7]   = (length >> 16) & 0xFF
            header[8]   = (length >> 8) & 0xFF
            header[9]   = length & 0xFF
            video_buf = header + jpeg
            sent = 0
            while sent < len(video_buf):
                chunk = min(1400, len(video_buf) - sent)
                n = pc_sock.send(video_buf[sent:sent + chunk])
                if n <= 0:
                    raise OSError("send failed")
                sent += n
            last_video_ms = now_ms
        except Exception:
            print("[CAL] Video disconnected — continuing detection only")
            try: pc_sock.close()
            except: pass
            pc_sock = None

    # 4. GC
    if frame_idx % 60 == 0:
        try: gc.collect()
        except: pass
