# K230 钢球检测 + WiFi 图传
# UART 发坐标给主控, WiFi 发标注画面到电脑录像
#
# 每次修改后拔 USB 断电重启 K230!

from media.sensor import *
from media.display import *
from machine import UART
import image
import time
import network
import socket
import gc

# ---- 0. 清理 ----
try: Display.deinit()
except: pass
try: MediaManager.deinit()
except: pass

# ========== 配置 ==========
# WiFi 图传
WIFI_SSID     = "test"
WIFI_PASS     = "90z5M92#"
PC_IP         = "192.168.137.1"
PC_PORT       = 8888
FRAME_MAGIC   = b'\xA5\x5A\xA5\x5A'
JPEG_QUALITY  = 25

# 钢球检测参数 —— 需要现场标定!
BALL_THRESHOLD = (0, 80, -40, 40, -40, 40)   # LAB 颜色阈值  TODO: 标定
MIN_PIXELS     = 20                            # 最小像素数
ORIGIN_X_PX    = 320                           # 图像中心 X (640/2) TODO: 标定
PX_PER_CM      = 15.0                          # 像素/厘米比例   TODO: 标定

# UART (发给主控)
UART_NUM       = 2
UART_BAUD      = 115200
# ==============================

# ---- 1. WiFi ----
print("[K230] WiFi...")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASS)
for i in range(30):
    if wlan.isconnected(): break
    time.sleep_ms(500)
if not wlan.isconnected():
    wlan.disconnect()
    time.sleep_ms(500)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    for i in range(30):
        if wlan.isconnected(): break
        time.sleep_ms(500)
if not wlan.isconnected():
    raise Exception("WiFi failed")
print("[K230] IP:", wlan.ifconfig()[0])

# ---- 2. 摄像头 ----
print("[K230] 摄像头...")
sensor = Sensor(id=2, width=1920, height=1080, fps=30)
sensor.reset()
sensor.set_pixformat(Sensor.RGB565)
try: sensor.set_framesize(Sensor.FHD)
except: pass
Display.init(Display.VIRT, sensor.width(), sensor.height(), to_ide=True)
MediaManager.init()
sensor.run()
time.sleep_ms(500)
for _ in range(5):
    try:
        sensor.snapshot()
        print("[K230] 就绪 {}x{}".format(sensor.width(), sensor.height()))
        break
    except:
        time.sleep_ms(300)

# ---- 3. UART (发给主控) ----
try:
    uart = UART(UART_NUM, UART_BAUD)
    print("[K230] UART{} 就绪".format(UART_NUM))
except Exception as e:
    print("[K230] UART 失败:", e)
    uart = None

# ---- 4. 连接 PC ----
print("[K230] 连接 PC...")
def connect_pc():
    for _ in range(999):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((PC_IP, PC_PORT))
            print("[K230] TCP OK")
            return s
        except:
            time.sleep_ms(2000)

sock = connect_pc()

def send_all(data):
    mv = memoryview(data)
    off = 0
    while off < len(mv):
        n = sock.send(mv[off:])
        if n <= 0: raise OSError()
        off += n

# ---- 5. 主循环: 检测 + 推流 ----
print("[K230] 检测+推流...")
clock = time.clock()
n = 0

X_CM = 640 // 2  # 图像中心 X
Y_CROP_START = (1080 - 480) // 2  # 垂直居中裁剪

while True:
    clock.tick()

    # 采图
    img = sensor.snapshot()

    # 中心裁剪到 640x480 (减少计算量)
    img = img.copy(roi=(X_CM - 320, Y_CROP_START, 640, 480))

    # ---- 钢球检测 ----
    blobs = img.find_blobs([BALL_THRESHOLD], pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_PIXELS, merge=True)

    best = None
    for b in blobs:
        if best is None or b.pixels() > best.pixels():
            best = b

    if best is not None:
        x_cm = (best.cx() - ORIGIN_X_PX) / PX_PER_CM
        quality = min(100, best.pixels())
        valid = True

        # 画面标注
        img.draw_rectangle(best.rect(), color=(255, 0, 0), thickness=2)
        img.draw_cross(best.cx(), best.cy(), color=(0, 255, 0), size=10, thickness=2)
        img.draw_string(2, 2, "X={:.1f}cm".format(x_cm), color=(255, 255, 255), scale=2)

        # 发 UART 给主控 (按 控制接口骨架.md 协议)
        if uart is not None:
            try:
                msg = "<B,{},{},{},{:d}>\n".format(
                    time.ticks_ms(), int(x_cm * 100), quality, 1)
                uart.write(msg)
            except:
                pass
    else:
        valid = False
        x_cm = 0.0
        quality = 0
        if uart is not None:
            try:
                uart.write("<B,{},{},{},0>\n".format(time.ticks_ms(), 0, 0))
            except:
                pass

    # ---- WiFi 推流 ----
    jpeg = bytes(img.compress(quality=JPEG_QUALITY))
    try:
        send_all(FRAME_MAGIC)
        send_all(len(jpeg).to_bytes(4, 'big'))
        send_all(jpeg)
    except OSError:
        print("[K230] 断线重连...")
        try: sock.close()
        except: pass
        sock = connect_pc()

    n += 1
    if n % 30 == 0:
        ball_info = "X={:.1f}".format(x_cm) if valid else "NO BALL"
        print("[K230] FPS:{:.1f}  {}B  {}".format(clock.fps(), len(jpeg), ball_info))
        gc.collect()
