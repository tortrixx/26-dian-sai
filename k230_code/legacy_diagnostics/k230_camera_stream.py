# K230 CanMV 摄像头图传 —— WiFi 推流到电脑
# 适用: CanMV v1.4.3 + GC2093 (亚博版 K230)
#
# 每次修改后需拔 USB 断电重启 K230!

from media.sensor import *
from media.display import *
import image
import time
import network
import socket
import gc

# ---- 0. 清理残留 ----
try: Display.deinit()
except: pass
try: MediaManager.deinit()
except: pass

# ========== 配置 ==========
WIFI_SSID     = "test"
WIFI_PASS     = "YOUR_HOTSPOT_PASSWORD"
PC_IP         = "192.168.137.1"
PC_PORT       = 8888
FRAME_MAGIC   = b'\xA5\x5A\xA5\x5A'   # 帧同步魔数
JPEG_QUALITY  = 25                     # 画质 (10-50)
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
    # 重试一次
    print("[K230] 重试 WiFi...")
    wlan.disconnect()
    time.sleep_ms(500)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    for i in range(30):
        if wlan.isconnected(): break
        time.sleep_ms(500)
if not wlan.isconnected():
    raise Exception("WiFi failed")
print("[K230] IP:", wlan.ifconfig()[0])

# ---- 2. 摄像头 (1080p, 等待颜色球检测功能完成后再优化分辨率) ----
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
else:
    raise Exception("snapshot failed")

# ---- 3. TCP ----
print("[K230] 连接 PC...")
def connect_pc():
    for attempt in range(999):  # 永不放弃
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((PC_IP, PC_PORT))
            print("[K230] TCP OK")
            return s
        except:
            if attempt == 0:
                print("[K230] 等待 PC 端...")
            time.sleep_ms(2000)
    raise Exception("TCP failed")

sock = connect_pc()

def send_all(data):
    mv = memoryview(data)
    off = 0
    while off < len(mv):
        n = sock.send(mv[off:])
        if n <= 0:
            raise OSError()
        off += n

# ---- 4. 推流 ----
print("[K230] 推流...")
clock = time.clock()
n = 0

while True:
    clock.tick()

    img = sensor.snapshot()

    # 中心裁剪到 640x480 再压缩 (大幅减少 JPEG 计算量)
    crop_w, crop_h = 640, 480
    x = (img.width() - crop_w) // 2
    y = (img.height() - crop_h) // 2
    try:
        img = img.copy(roi=(x, y, crop_w, crop_h))
    except:
        # 如果 copy(roi) 不支持，用全图
        pass

    jpeg = bytes(img.compress(quality=JPEG_QUALITY))

    try:
        send_all(FRAME_MAGIC)
        send_all(len(jpeg).to_bytes(4, 'big'))
        send_all(jpeg)
    except OSError:
        print("[K230] 断线重连...")
        try: sock.close()
        except: pass
        sock = connect_pc()  # 永不放弃，一直等到 PC 重新运行

    n += 1
    if n % 30 == 0:
        print("[K230] FPS:{:.1f}  {}B".format(clock.fps(), len(jpeg)))
        gc.collect()
