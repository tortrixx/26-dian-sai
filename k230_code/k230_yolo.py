"""K230 YOLO11 NPU steel-ball detector — PipeLine edition.

Based on Laoguigui2/K230- reference code.  Uses PipeLine for sensor management,
which avoids the Yahboom v1.4.3 CHN_2 buf_init bug by calling set_framesize
BEFORE set_pixformat.

Architecture:
  PipeLine CHN_2 (RGBP888 640x360) → AI2D → YOLO11 NPU → ball detection
  PipeLine CHN_0 (YUV420SP) → virtual display (IDE preview)
  UART → MSPM0 ball position
  JPEG (from CHN_2 → RGB565) → TCP WiFi → PC receiver

Usage:
  Copy to /sdcard/app/k230_yolo.py, run from CanMV IDE.

Requires:
  /sdcard/kmodel/yolo11n_det_320.kmodel   # steel-ball model (Laoguigui2)
  /sdcard/app/libs/{AIBase,AI2D,PipeLine,Utils,YOLO}.py
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

# ---- 0. Clean up IDE media pipeline ----
try:
    Display.deinit()
except Exception:
    pass
try:
    MediaManager.deinit()
except Exception:
    pass

# ============ Configuration ============

# ---- YOLO / NPU (Laoguigui2 model — 1 class: steel ball) ----
MODEL_PATH = "/sdcard/kmodel/yolo11n_det_320.kmodel"
MODEL_LABELS = {0: 'steel'}       # dict format — 1 class
MODEL_INPUT_SIZE = [320, 320]     # kmodel input
RGB888P_SIZE = [640, 480]         # AI channel = full frame (no vertical crop)
DISPLAY_SIZE = [640, 480]         # virtual display / JPEG frame size
CONF_THRESH = 0.35                # lower threshold for more consistent detection
NMS_THRESH = 0.45
MAX_BOXES = 10

# ---- Vision / tracking ----
# Pixel-to-cm calibration.  Calibrated 2026-07-31 with real steel ball.
ZERO_X_PX = 345.0
PX_PER_CM = 20.1
PIPE_ROI = (0, 120, 640, 240)

# Alpha-beta tracker
TRACK_HALF_WIDTH_PX = 100
MAX_TRACK_SPEED_PX_S = 700.0
TRACK_MISS_LIMIT = 10
ALPHA = 0.65
BETA = 0.12

# ---- UART ----
UART_BAUD = 115200
UART_TX = 9
UART_RX = 10

# ---- Wire protocol ----
PROTO_HEAD_0 = 0xAA
PROTO_HEAD_1 = 0x55
MSG_VISION_TARGET = 0x01
VISION_FLAG_VALID = 0x01
VISION_FLAG_TRACKED = 0x02

# ---- WiFi streaming ----
WIFI_SSID = "test"
WIFI_PASS = "90z5M92#"
PC_IP = "192.168.137.1"
PC_PORT = 8888
MAGIC = b'\xA5\x5A\xA5\x5A'
VIDEO_CONNECT_TIMEOUT_S = 0.50
VIDEO_STALL_TIMEOUT_MS = 3000
WIFI_RETRY_MS = 8000
PC_RETRY_MS = 3000

# Streaming (JPEG over TCP)
STREAM_PROFILE = "pipe_detail"
STREAM_PROFILES = {
    "control":     (320, 240, 8, 70),
    "pipe_detail": (640, 240, 6, 50),
}
VIDEO_W, VIDEO_H, VIDEO_TARGET_FPS, JPEG_Q = STREAM_PROFILES[STREAM_PROFILE]
VIDEO_INTERVAL_MS = max(1, 1000 // VIDEO_TARGET_FPS)
PIPE_VIDEO_CROP = (0, 120, 640, 240)

# K23V protocol
STREAM_MAGIC = b'K23V'
STREAM_VERSION = 1
STREAM_CODEC_JPEG = 1

STREAM_OVERLAY_ENABLE = True
STREAM_OVERLAY_BOX_COLOR = (255, 0, 0)
STREAM_OVERLAY_CROSS_COLOR = (0, 255, 0)
STREAM_OVERLAY_THICKNESS = 2
STREAM_OVERLAY_CROSS_SIZE = 12

PERFORMANCE_LOG = True
VISION_ENABLE = True


# ============ Protocol helpers ============

def _checksum(buf, start, end):
    value = 0
    for index in range(start, end):
        value = (value + buf[index]) & 0xFF
    return value

def _clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value

def _put_i16_le(buf, offset, value):
    value = int(value)
    value = max(-32768, min(32767, value)) & 0xFFFF
    buf[offset] = value & 0xFF
    buf[offset + 1] = (value >> 8) & 0xFF

def _ticks_diff(now, then):
    try:
        return time.ticks_diff(now, then)
    except Exception:
        return now - then

def build_vision_frame(seq, valid, x_cm_x100, y_offset_px, quality=0, tracked=False):
    payload = bytearray(6)
    flags = 0
    if valid:
        flags |= VISION_FLAG_VALID
    if tracked:
        flags |= VISION_FLAG_TRACKED
    payload[0] = flags
    _put_i16_le(payload, 1, x_cm_x100)
    _put_i16_le(payload, 3, y_offset_px)
    payload[5] = _clamp(quality, 0, 255)
    length = len(payload) + 2
    frame = bytearray(2 + 1 + length + 1)
    frame[0] = PROTO_HEAD_0
    frame[1] = PROTO_HEAD_1
    frame[2] = length
    frame[3] = MSG_VISION_TARGET
    frame[4] = seq & 0xFF
    for index in range(len(payload)):
        frame[5 + index] = payload[index]
    frame[-1] = _checksum(frame, 2, len(frame) - 1)
    return frame

def pixel_to_cm(px_x):
    return (px_x - ZERO_X_PX) / PX_PER_CM


# ============ Alpha-Beta Tracker ============

class AlphaBetaTracker:
    def __init__(self):
        self.ready = False
        self.x = ZERO_X_PX
        self.v = 0.0
        self.last_ms = 0
        self.misses = 0

    def predicted_x(self, now_ms):
        if not self.ready:
            return ZERO_X_PX
        dt_s = _clamp(_ticks_diff(now_ms, self.last_ms) / 1000.0, 0.0, 0.25)
        return self.x + self.v * dt_s

    def update(self, measured_x, now_ms):
        measured_x = float(measured_x)
        if not self.ready:
            self.ready = True
            self.x = measured_x
            self.v = 0.0
            self.last_ms = now_ms
            self.misses = 0
            return self.x
        dt_s = _clamp(_ticks_diff(now_ms, self.last_ms) / 1000.0, 0.001, 0.25)
        prediction = self.x + self.v * dt_s
        residual = measured_x - prediction
        self.x = prediction + ALPHA * residual
        self.v = _clamp(self.v + BETA * residual / dt_s,
                        -MAX_TRACK_SPEED_PX_S, MAX_TRACK_SPEED_PX_S)
        self.last_ms = now_ms
        self.misses = 0
        return self.x

    def miss(self):
        self.misses += 1

    def use_local_roi(self):
        return self.ready and self.misses < TRACK_MISS_LIMIT


# ============ Video sender ============

class VideoSender:
    def __init__(self):
        self._pending = None
        self._sent = 0
        self._stall_count = 0
        self._start_ms = 0

    def enqueue_payload(self, codec, jpeg_bytes):
        self._pending = (codec, jpeg_bytes)
        self._sent = 0

    def pending(self):
        return self._pending is not None

    def reset(self):
        self._pending = None
        self._sent = 0
        self._stall_count = 0

    def stall_count(self):
        return self._stall_count

    def flush(self, sock, now_ms):
        if self._pending is None:
            return 0, False
        codec, payload = self._pending
        length = len(payload)
        if self._sent == 0:
            self._start_ms = now_ms
            header = bytearray(10)
            header[0:4] = STREAM_MAGIC
            header[4] = STREAM_VERSION
            header[5] = codec
            header[6] = (length >> 24) & 0xFF
            header[7] = (length >> 16) & 0xFF
            header[8] = (length >> 8) & 0xFF
            header[9] = length & 0xFF
            self._buf = header + payload
            self._buf_len = len(self._buf)
        total_sent = 0
        attempts = 0
        max_attempts = 4  # don't spin forever in one flush call
        try:
            while self._sent < self._buf_len and attempts < max_attempts:
                chunk = min(512, self._buf_len - self._sent)
                sent = sock.send(self._buf[self._sent:self._sent + chunk])
                if sent > 0:
                    self._sent += sent
                    total_sent += sent
                    self._stall_count = 0  # reset on progress
                    attempts += 1
                elif sent == 0:
                    # Non-blocking socket buffer full — try again next tick
                    self._stall_count += 1
                    break
                else:
                    # sent < 0 — socket error
                    self._stall_count += 1
                    break
        except OSError:
            # EAGAIN / EWOULDBLOCK — buffer full, try next tick
            self._stall_count += 1
        except Exception:
            self._stall_count += 5  # hard error
        if self._sent >= self._buf_len:
            self._pending = None
            self._sent = 0
            self._stall_count = 0
            return total_sent, False
        # Stall threshold: only report as dead after ~3 seconds of failures
        if self._stall_count > 60:
            return total_sent, True
        return total_sent, False


# ============ WiFi helpers ============

def wifi_connect_once():
    try:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if not wlan.isconnected():
            wlan.connect(WIFI_SSID, WIFI_PASS)
            deadline = time.ticks_ms() + 8000
            while not wlan.isconnected():
                if _ticks_diff(time.ticks_ms(), deadline) > 0:
                    break
                time.sleep_ms(50)
        return wlan
    except Exception:
        return None


def wifi_ready(wlan, now_ms, last_attempt_ms):
    if wlan is None:
        return False, last_attempt_ms
    if wlan.isconnected():
        return True, last_attempt_ms
    if _ticks_diff(now_ms, last_attempt_ms) < WIFI_RETRY_MS:
        return False, last_attempt_ms
    try:
        wlan.connect(WIFI_SSID, WIFI_PASS)
    except Exception:
        pass
    return wlan.isconnected(), now_ms


def pc_connect_once():
    try:
        addr = socket.getaddrinfo(PC_IP, PC_PORT)[0][-1]
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(VIDEO_CONNECT_TIMEOUT_S)
        sock.connect(addr)
        sock.setblocking(False)
        print("[K230] PC video receiver connected")
        return sock
    except Exception:
        return None


# ============ UART sender ============

uart_seq = 0

def send_ball(valid, x_cm_x100, y_offset_px, quality, tracked):
    global uart_seq
    try:
        frame = build_vision_frame(uart_seq, valid, x_cm_x100,
                                   y_offset_px, quality, tracked)
        uart.write(bytes(frame))
        uart_seq = (uart_seq + 1) & 0xFF
    except Exception:
        pass


# ============ Streaming overlay ============

def _draw_stream_marker(canvas, marker, crop_offset=(0, 0)):
    if marker is None:
        return
    bx, by, bw, bh, cx, cy = marker
    ox, oy = crop_offset
    canvas.draw_rectangle(bx - ox, by - oy, bw, bh,
                          color=STREAM_OVERLAY_BOX_COLOR,
                          thickness=STREAM_OVERLAY_THICKNESS)
    canvas.draw_cross(int(cx - ox), int(cy - oy),
                      color=STREAM_OVERLAY_CROSS_COLOR,
                      size=STREAM_OVERLAY_CROSS_SIZE,
                      thickness=STREAM_OVERLAY_THICKNESS)


def encode_video_jpeg(capture_img, vision_marker=None):
    pipe_crop = capture_img.copy(roi=PIPE_VIDEO_CROP)
    if STREAM_OVERLAY_ENABLE and vision_marker is not None:
        _draw_stream_marker(pipe_crop, vision_marker,
                            crop_offset=(PIPE_VIDEO_CROP[0], PIPE_VIDEO_CROP[1]))
    return bytes(pipe_crop.compress(quality=JPEG_Q))


# ============ Main ============

print("[K230] YOLO11 steel-ball detector (PipeLine edition)")

# UART
uart = None
try:
    fpioa = FPIOA()
    fpioa.set_function(UART_TX, fpioa.UART1_TXD)
    fpioa.set_function(UART_RX, fpioa.UART1_RXD)
    uart = UART(UART.UART1, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)
    print("[K230] UART1 IO{}/IO{} {} baud OK".format(UART_TX, UART_RX, UART_BAUD))
except Exception as error:
    print("[K230] UART unavailable:", error)

# Sensor + Display — direct API, Laoguigui2 init order.
# CRITICAL: set_framesize BEFORE set_pixformat avoids Yahboom v1.4.3 buf_init bug.
# CHN_0 (default) → RGB565 → virtual display + JPEG streaming
# CHN_2           → RGBP888 640x360 → AI2D → YOLO NPU
print("[K230] Init camera (CHN_0 RGB565 640x480 + CHN_2 RGBP888 {}x{})...".format(
    RGB888P_SIZE[0], RGB888P_SIZE[1]))
try:
    sensor = Sensor(id=2, width=640, height=480, fps=60)
    sensor.reset()
    # CHN_0: set_framesize BEFORE set_pixformat
    sensor.set_framesize(w=DISPLAY_SIZE[0], h=DISPLAY_SIZE[1], chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    print("[K230]   CHN_0 RGB565 {}x{}".format(
        sensor.width(), sensor.height()))
    # CHN_2: set_framesize BEFORE set_pixformat (Laoguigui2 order — no buf_init!)
    sensor.set_framesize(w=RGB888P_SIZE[0], h=RGB888P_SIZE[1], chn=CAM_CHN_ID_2)
    sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
    print("[K230]   CHN_2 RGBP888 {}x{}".format(
        sensor.width(chn=CAM_CHN_ID_2), sensor.height(chn=CAM_CHN_ID_2)))
    # Virtual display
    Display.init(Display.VIRT, DISPLAY_SIZE[0], DISPLAY_SIZE[1], to_ide=True)
    MediaManager.init()
    print("[K230]   Display VIRT {}x{} init ok".format(DISPLAY_SIZE[0], DISPLAY_SIZE[1]))
except Exception as e:
    print("[K230] CAMERA INIT FAILED:", e)
    raise

# YOLO11 model — loaded BEFORE sensor.run() to keep CHN_2 buffers fresh.
print("[K230] Loading YOLO11 steel-ball model (3 MB, ~15 s)...")
try:
    yolo = YOLO11(
        task_type="detect",
        mode="video",
        kmodel_path=MODEL_PATH,
        labels=MODEL_LABELS,
        rgb888p_size=RGB888P_SIZE,
        model_input_size=MODEL_INPUT_SIZE,
        display_size=DISPLAY_SIZE,
        conf_thresh=CONF_THRESH,
        nms_thresh=NMS_THRESH,
        max_boxes_num=MAX_BOXES,
        debug_mode=0
    )
    yolo.config_preprocess()
except Exception as e:
    print("[K230] YOLO model load FAILED:", e)
    raise
print("[K230] YOLO11 model ready")

# Start sensor + warm up
sensor.run()
time.sleep_ms(600)
for _ in range(8):
    try:
        sensor.snapshot(chn=CAM_CHN_ID_2)
    except Exception:
        time.sleep_ms(100)
print("[K230] Sensor running, frames warm")

# Streaming config
print("[K230] Video pipe ROI {} -> {}x{} Q{} @{}fps".format(
    PIPE_VIDEO_CROP, VIDEO_W, VIDEO_H, JPEG_Q, VIDEO_TARGET_FPS))

tracker = AlphaBetaTracker()
video_sender = VideoSender()
clock = time.clock()

frame_index = 0
last_video_enqueue_ms = 0
last_pc_attempt_ms = 0
last_wifi_attempt_ms = 0

wlan = None
pc_sock = None

video_stat_start_ms = time.ticks_ms()
video_count = 0
video_bytes = 0

perf_cap_ms = 0
perf_yolo_ms = 0
perf_jpeg_ms = 0
perf_send_ms = 0

print("[K230] Running: YOLO NPU → UART; JPEG WiFi streaming")

while True:
    clock.tick()
    now_ms = time.ticks_ms()

    # 1. Get AI frame from CHN_2 (RGBP888, 640x360)
    cap_start = time.ticks_ms()
    ai_img = sensor.snapshot(chn=CAM_CHN_ID_2)
    ai_np = ai_img.to_numpy_ref()
    perf_cap_ms += _ticks_diff(time.ticks_ms(), cap_start)

    # 2. YOLO NPU inference
    yolo_start = time.ticks_ms()
    ball_valid = False
    ball_cx = 0.0
    ball_cy = 0.0
    ball_conf = 0.0
    box_w = 0
    box_h = 0
    if VISION_ENABLE:
        try:
            results = yolo.run(ai_np)
        except Exception as e:
            # IDE interrupt or transient error — skip this frame
            results = ([], [], [])
        if results and results[0]:
            # Log ALL detections for debugging
            det_info = []
            best_idx = -1
            best_conf = 0.0
            for i in range(len(results[0])):
                score = float(results[2][i])
                if score > CONF_THRESH:
                    x, y, w, h = results[0][i]
                    det_info.append("[{},{},{},{},c{}]".format(x,y,w,h,results[1][i]))
                if score > best_conf:
                    best_conf = score
                    best_idx = i
            if det_info and frame_index % 30 == 0:
                print("[K230] dets({}): {}".format(len(det_info), " ".join(det_info)))
            if best_idx >= 0:
                x, y, w, h = results[0][best_idx]
                ball_cx = float(x) + float(w) / 2.0
                ball_cy = float(y) + float(h) / 2.0
                ball_conf = best_conf
                box_w = int(w)
                box_h = int(h)
                ball_valid = True
    perf_yolo_ms += _ticks_diff(time.ticks_ms(), yolo_start)

    # 3. Tracking & UART
    stream_marker = None
    if ball_valid:
        filtered_x = tracker.update(ball_cx, now_ms)
        x_cm = pixel_to_cm(filtered_x)
        x_cm_x100 = int(round(x_cm * 100.0))
        quality = int(_clamp(ball_conf * 100.0, 10, 90))
        stream_marker = (int(ball_cx - box_w / 2), int(ball_cy - box_h / 2),
                         box_w, box_h, int(filtered_x), int(ball_cy))
        send_ball(True, x_cm_x100, 0, quality, tracker.ready)
    else:
        tracker.miss()
        send_ball(False, 0, 0, 0, tracker.ready)

    # 4. WiFi connection management
    if wlan is None and frame_index % 50 == 0:
        wlan = wifi_connect_once()
    if wlan is not None:
        wifi_is_ready, last_wifi_attempt_ms = wifi_ready(wlan, now_ms, last_wifi_attempt_ms)
        if (pc_sock is None and wifi_is_ready and
                _ticks_diff(now_ms, last_pc_attempt_ms) >= PC_RETRY_MS):
            last_pc_attempt_ms = now_ms
            pc_sock = pc_connect_once()

    # 5. JPEG encoding & streaming (from CHN_0 RGB565)
    if (pc_sock is not None and not video_sender.pending() and
            _ticks_diff(now_ms, last_video_enqueue_ms) >= VIDEO_INTERVAL_MS):
        try:
            jpeg_start = time.ticks_ms()
            stream_img = sensor.snapshot()  # CHN_0, RGB565
            jpeg = encode_video_jpeg(stream_img, stream_marker)
            video_sender.enqueue_payload(STREAM_CODEC_JPEG, jpeg)
            perf_jpeg_ms += _ticks_diff(time.ticks_ms(), jpeg_start)
            last_video_enqueue_ms = now_ms
        except Exception as e:
            print("[K230] JPEG encode error:", e)
            video_sender.reset()

    if pc_sock is not None:
        send_start = time.ticks_ms()
        completed_bytes, connection_stalled = video_sender.flush(pc_sock, now_ms)
        perf_send_ms += _ticks_diff(time.ticks_ms(), send_start)
        if completed_bytes > 0:
            video_bytes += completed_bytes
            video_count += 1
        if connection_stalled:
            print("[K230] video stalled ({}) — reconnect".format(
                video_sender.stall_count()))
            try:
                pc_sock.close()
            except Exception:
                pass
            pc_sock = None
            last_pc_attempt_ms = now_ms  # don't reconnect immediately
            video_sender.reset()

    # 6. Status & GC
    frame_index += 1
    if frame_index % 60 == 0:
        try:
            gc.collect()
        except Exception:
            pass

    if frame_index % 30 == 0:
        if ball_valid:
            state = "x={:.2f}cm cx={:.1f}px c={:.2f}".format(
                x_cm_x100 / 100.0, ball_cx, ball_conf)
        else:
            state = "NO BALL"
        stat_elapsed_ms = max(1, _ticks_diff(now_ms, video_stat_start_ms))
        video_fps = video_count * 1000.0 / stat_elapsed_ms
        video_kb_s = video_bytes * 1000.0 / stat_elapsed_ms / 1024.0
        if PERFORMANCE_LOG:
            print("[K230] Loop:{:.1f} {} UART:{} Video:{:.1f}fps {:.1f}KB/s "
                  "ms/frame cap:{:.1f} yolo:{:.1f} enc:{:.1f} net:{:.1f}".format(
                clock.fps(), state, uart_seq, video_fps, video_kb_s,
                perf_cap_ms / 30.0, perf_yolo_ms / 30.0,
                perf_jpeg_ms / 30.0, perf_send_ms / 30.0
            ))
        else:
            print("[K230] Loop:{:.1f} {} UART:{} Video:{:.1f}fps {:.1f}KB/s".format(
                clock.fps(), state, uart_seq, video_fps, video_kb_s))
        video_stat_start_ms = now_ms
        video_count = 0
        video_bytes = 0
        perf_cap_ms = 0
        perf_yolo_ms = 0
        perf_jpeg_ms = 0
        perf_send_ms = 0
