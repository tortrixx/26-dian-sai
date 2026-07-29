"""
K230 YOLOv8 钢球检测 + WiFi 图传 — 整合版
基于队友的 PipeLine/YOLO/UART 代码, 增加 WiFi 推流到 PC 录像

部署: 把此文件复制到 /sdcard/steel_ball/ 目录下运行
依赖: libs/PipeLine.py, libs/YOLO.py, libs/Utils.py, steel_ball_yolov8n_320.kmodel
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from libs.Utils import ScopedTiming
import gc, os, sys, time

# ---- MSPM0 UART 协议 (队友代码) ----
from machine import UART, FPIOA

sys.path.append("/sdcard/app")
sys.path.append("/sdcard/steel_ball")

try:
    from ybUtils.YbUart import YbUart
except Exception:
    YbUart = None

try:
    from k230_mspm0_uart_protocol import Mspm0UartProtocol
except Exception:
    PROTO_HEAD_0 = 0xAA
    PROTO_HEAD_1 = 0x55
    MSG_VISION_TARGET = 0x01
    VISION_FLAG_FOUND = 0x01

    def _clamp_i16(v):
        v = int(v)
        return max(-32768, min(32767, v))

    def _put_i16_le(buf, off, v):
        v = _clamp_i16(v) & 0xFFFF
        buf[off] = v & 0xFF
        buf[off + 1] = (v >> 8) & 0xFF

    def _chk(frame, start, end):
        c = 0
        for i in range(start, end):
            c = (c + frame[i]) & 0xFF
        return c

    class Mspm0UartProtocol:
        def __init__(self, uart):
            self.uart = uart
            self.seq = 0

        def _write(self, frame):
            try:
                return self.uart.write(frame)
            except Exception:
                return self.uart.send(frame)

        def deinit(self):
            try: return self.uart.deinit()
            except Exception: return None

        def send_vision(self, found, err_x, err_y, quality=0):
            payload = bytearray(6)
            payload[0] = VISION_FLAG_FOUND if found else 0
            _put_i16_le(payload, 1, err_x)
            _put_i16_le(payload, 3, err_y)
            payload[5] = max(0, min(255, int(quality)))
            length = len(payload) + 2
            frame = bytearray(2 + 1 + length + 1)
            frame[0] = PROTO_HEAD_0
            frame[1] = PROTO_HEAD_1
            frame[2] = length
            frame[3] = MSG_VISION_TARGET
            frame[4] = self.seq
            for i in range(len(payload)):
                frame[5 + i] = payload[i]
            frame[len(frame) - 1] = _chk(frame, 2, len(frame) - 1)
            self.seq = (self.seq + 1) & 0xFF
            return self._write(frame)

# ---- 配置 ----
MODEL_PATHS = [
    "/sdcard/steel_ball/steel_ball_yolov8n_320.kmodel",
    "/sdcard/app/steel_ball_yolov8n_320.kmodel",
    "/sdcard/steel_ball_yolov8n_320.kmodel",
]
LABELS = ["steel_ball"]
MODEL_INPUT_SIZE = [320, 320]
SENSOR_ID = 2
DISPLAY_MODE = "lcd"
DISPLAY_SIZE = [640, 480]
RGB888P_SIZE = [640, 360]
H_MIRROR = False; V_FLIP = False
CONFIDENCE_THRESHOLD = 0.22
FILTER_CONFIDENCE_THRESHOLD = 0.35
TOUCHING_CONFIDENCE_THRESHOLD = 0.22
MIN_BOX_AREA_RATIO = 0.004; MAX_BOX_AREA_RATIO = 0.18
MIN_ASPECT_RATIO = 0.55; MAX_ASPECT_RATIO = 1.80
TOUCHING_DISTANCE_RATIO = 1.35
NMS_THRESHOLD = 0.45; MAX_BOXES = 50
UART_BAUDRATE = 115200; UART_TX_PIN = 9; UART_RX_PIN = 10
UART_SEND_PERIOD_MS = 80

# WiFi 图传配置
WIFI_ENABLE    = True
WIFI_SSID      = "test"
WIFI_PASS      = "YOUR_HOTSPOT_PASSWORD"
PC_IP          = "192.168.137.1"
PC_PORT        = 8888
FRAME_MAGIC    = b'\xA5\x5A\xA5\x5A'
JPEG_QUALITY   = 25      # 640x480 JPEG 约 7-10KB
WIFI_JPEG_SIZE = (320, 240)  # 缩小后发送, 省带宽不卡检测

# ---- 辅助函数 ----
def find_model_path():
    for p in MODEL_PATHS:
        try: os.stat(p); print("[YOLO+WiFi] model:", p); return p
        except OSError: pass
    raise OSError("kmodel not found")

def enable_uart_pin(fpioa, pin, func, is_tx):
    try: fpioa.set_function(pin, func, ie=0 if is_tx else 1, oe=1 if is_tx else 0); return
    except: pass
    fpioa.set_function(pin, func)
    try: fpioa.set_function(pin, set_ie=0 if is_tx else 1, set_oe=1 if is_tx else 0)
    except: pass

def init_uart():
    try:
        fpioa = FPIOA()
        enable_uart_pin(fpioa, UART_TX_PIN, fpioa.UART1_TXD, True)
        enable_uart_pin(fpioa, UART_RX_PIN, fpioa.UART1_RXD, False)
        u = UART(UART.UART1, baudrate=UART_BAUDRATE, bits=UART.EIGHTBITS,
                 parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        print("[YOLO+WiFi] UART1 IO{}/IO{} {}baud".format(UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE))
        return u
    except Exception as e:
        print("[YOLO+WiFi] UART1 failed:", e)
    if YbUart is not None:
        try: print("[YOLO+WiFi] YbUart fallback"); return YbUart(baudrate=UART_BAUDRATE)
        except: pass
    return None

def is_reasonable_ball(box, ds):
    x, y, w, h = box
    if w <= 0 or h <= 0: return False
    asp = w / h
    ar = w * h / (ds[0] * ds[1])
    return MIN_ASPECT_RATIO <= asp <= MAX_ASPECT_RATIO and MIN_BOX_AREA_RATIO <= ar <= MAX_BOX_AREA_RATIO

def is_near(box, other):
    cx = box[0] + box[2] / 2; cy = box[1] + box[3] / 2
    ocx = other[0] + other[2] / 2; ocy = other[1] + other[3] / 2
    d2 = (cx - ocx)**2 + (cy - ocy)**2
    nd = max(box[2], box[3], other[2], other[3]) * TOUCHING_DISTANCE_RATIO
    return d2 <= nd * nd

def filter_result(result, ds):
    if not result or len(result[0]) == 0: return result
    candidates, strong = [], []
    for i in range(len(result[0])):
        b, s = result[0][i], float(result[2][i])
        if not is_reasonable_ball(b, ds): continue
        candidates.append((i, b, s))
        if s >= FILTER_CONFIDENCE_THRESHOLD: strong.append(b)
    boxes, cids, scores = [], [], []
    for i, b, s in candidates:
        keep = s >= FILTER_CONFIDENCE_THRESHOLD
        if not keep and s >= TOUCHING_CONFIDENCE_THRESHOLD:
            for sb in strong:
                if is_near(b, sb): keep = True; break
        if keep: boxes.append(b); cids.append(result[1][i]); scores.append(s)
    return [boxes, cids, scores]

def get_best(result):
    best_s, best_ctr, best_box = -1.0, None, None
    if result and len(result[0]) > 0:
        for i in range(len(result[0])):
            x, y, w, h = result[0][i]; s = float(result[2][i])
            if s > best_s:
                best_s = s; best_ctr = (int(x + w / 2), int(y + h / 2)); best_box = (x, y, w, h)
    return len(result[0]) if result else 0, best_ctr, best_s, best_box

# ---- WiFi ----
def init_wifi():
    import network
    print("[YOLO+WiFi] WiFi connecting...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    for _ in range(30):
        if wlan.isconnected(): break
        time.sleep_ms(500)
    if not wlan.isconnected():
        wlan.disconnect(); time.sleep_ms(500)
        wlan.connect(WIFI_SSID, WIFI_PASS)
        for _ in range(30):
            if wlan.isconnected(): break
            time.sleep_ms(500)
    if not wlan.isconnected():
        print("[YOLO+WiFi] WiFi FAILED — 仅本地检测")
        return None, None
    print("[YOLO+WiFi] WiFi OK:", wlan.ifconfig()[0])

    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((PC_IP, PC_PORT))
        print("[YOLO+WiFi] PC connected")
        return wlan, sock
    except:
        print("[YOLO+WiFi] PC not ready")
        return wlan, None

def wifi_send(sock, img):
    if sock is None: return
    import socket as _s
    # 缩小到传输分辨率
    try: img = img.copy(roi=(0, 0, img.width(), img.height()))
    except: pass
    jpeg = bytes(img.compress(quality=JPEG_QUALITY))
    try:
        sock.send(FRAME_MAGIC)
        sock.send(len(jpeg).to_bytes(4, 'big'))
        sock.send(jpeg)
    except Exception:
        pass  # WiFi 失败不影响检测

def wifi_reconnect(wlan, old_sock):
    if wlan is None: return None
    import socket as _s
    try: old_sock.close()
    except: pass
    try:
        sock = _s.socket(_s.AF_INET, _s.SOCK_STREAM)
        sock.connect((PC_IP, PC_PORT))
        return sock
    except: return None

# ---- 主程序 ----
def main():
    wlan, wifi_sock = None, None
    if WIFI_ENABLE:
        wlan, wifi_sock = init_wifi()

    model_path = find_model_path()
    pipeline = PipeLine(rgb888p_size=RGB888P_SIZE, display_mode=DISPLAY_MODE, display_size=DISPLAY_SIZE)
    pipeline.create(sensor_id=SENSOR_ID, hmirror=H_MIRROR, vflip=V_FLIP)
    ds = pipeline.get_display_size()
    ic = (ds[0] // 2, ds[1] // 2)

    uart_raw = init_uart()
    uart = Mspm0UartProtocol(uart_raw) if uart_raw else None
    if uart:
        print("[YOLO+WiFi] UART MSPM0 ready")
        for _ in range(20):
            uart.send_vision(False, 0, 0, 1)
            time.sleep_ms(200)

    detector = YOLOv8(task_type="detect", mode="video", kmodel_path=model_path,
                      labels=LABELS, rgb888p_size=RGB888P_SIZE, model_input_size=MODEL_INPUT_SIZE,
                      display_size=ds, conf_thresh=CONFIDENCE_THRESHOLD,
                      nms_thresh=NMS_THRESHOLD, max_boxes_num=MAX_BOXES, debug_mode=0)
    detector.config_preprocess()

    fi = 0
    last_uart = time.ticks_ms() - UART_SEND_PERIOD_MS
    wifi_n = 0
    print("[YOLO+WiFi] 运行中...")
    clock = time.clock()

    while True:
        clock.tick()
        with ScopedTiming("total", 1):
            frame = pipeline.get_frame()
            result = detector.run(frame)
            result = filter_result(result, ds)
            detector.draw_result(result, pipeline.osd_img)
            # 叠加信息
            cnt, best_ctr, best_sc, best_box = get_best(result)
            found = best_ctr is not None
            if found:
                err_x = best_ctr[0] - ic[0]
                err_y = best_ctr[1] - ic[1]
                quality = max(0, min(100, int(best_sc * 100)))
            else:
                err_x, err_y, quality = 0, 0, 0

            now = time.ticks_ms()
            if time.ticks_diff(now, last_uart) >= UART_SEND_PERIOD_MS:
                uart.send_vision(found, err_x, err_y, quality)
                last_uart = now

            pipeline.show_image()

            # WiFi 推流 (非阻塞, 失败不卡主循环)
            if wifi_sock is not None and fi % 2 == 0:  # 隔帧发送，降低 CPU 压力
                try:
                    wifi_send(wifi_sock, pipeline.osd_img)
                    wifi_n += 1
                except:
                    wifi_sock = wifi_reconnect(wlan, wifi_sock)

            fi += 1
            if fi % 30 == 0:
                fps = clock.fps()
                status = "FOUND" if found else "NO BALL"
                print("[YOLO+WiFi] FPS:{:.1f}  {}  err=({},{})  WiFi:{}".format(
                    fps, status, err_x, err_y, wifi_n))
                gc.collect()

if __name__ == "__main__":
    main()
