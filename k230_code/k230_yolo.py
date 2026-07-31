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
# 2026-08-01: 原点实测 x=-2.5cm → 零位修正 345.0 → 294.75 (345.0 - 2.5*20.1)。
# 相机若再移动需用 k230_calibrate.py 重新标定。
ZERO_X_PX = 294.75
PX_PER_CM = 20.1
PIPE_ROI = (0, 120, 640, 240)

# BallTracker state machine — SEARCH→CONFIRM(2)→TRACK→HOLD(3)→LOST
# Prevents single-frame false positives from reaching MSPM0.
CONFIRM_DETECTIONS = 2        # consistent frames to lock on
MISS_HOLD_DETECTIONS = 3      # misses before LOST
MAX_TRACK_JUMP_PX = 80        # ~4cm — reject wild position jumps
FILTER_ALPHA = 0.35           # position EMA smoothing
TRACK_HALF_WIDTH_PX = 100     # local ROI half-width when tracking

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
# fps 是节流上限（VIDEO_INTERVAL_MS），实际帧率 = 主循环速率（YOLO NPU 推理时
# CPU 空闲，可穿插 JPEG 编码，实测 ~20fps）。Q 提高几乎不增加编码耗时（硬件 JPEG），
# 只增帧体积：640x240 Q55 ≈ 10-13KB/帧 × 20fps ≈ 2Mbps，WiFi 余量充足。
STREAM_PROFILE = "pipe_detail"
STREAM_PROFILES = {
    "control":     (320, 240, 8, 70),
    "pipe_detail": (640, 240, 25, 55),
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

PERFORMANCE_LOG = False   # NEVER enable — print() blocks REPL UART, causes LOST
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
    """MicroPython ticks diff — direct arithmetic, no try/except overhead."""
    return now - then

# Pre-allocated UART frame buffer to avoid per-frame GC pressure
_UART_FRAME = bytearray(12)  # AA+55+len+type+seq+6B_payload+checksum = 12 bytes

def build_vision_frame(seq, valid, x_cm_x100, y_offset_px, quality=0, tracked=False):
    """Fill pre-allocated buffer in-place.  Returns memoryview to avoid copy."""
    flags = 0
    if valid:
        flags |= VISION_FLAG_VALID
    if tracked:
        flags |= VISION_FLAG_TRACKED

    _UART_FRAME[0] = PROTO_HEAD_0     # 0xAA
    _UART_FRAME[1] = PROTO_HEAD_1     # 0x55
    _UART_FRAME[2] = 8                 # length = 6 payload + 2
    _UART_FRAME[3] = MSG_VISION_TARGET # type = 0x01
    _UART_FRAME[4] = seq & 0xFF        # sequence
    _UART_FRAME[5] = flags             # payload[0]
    _put_i16_le(_UART_FRAME, 6, x_cm_x100)    # payload[1:3]
    _put_i16_le(_UART_FRAME, 8, y_offset_px)  # payload[3:5]
    _UART_FRAME[10] = _clamp(quality, 0, 255) # payload[5]
    _UART_FRAME[11] = _checksum(_UART_FRAME, 2, 11)
    return _UART_FRAME

def pixel_to_cm(px_x):
    return (px_x - ZERO_X_PX) / PX_PER_CM


# ============ BallTracker (state machine) ============

class BallTracker:
    """SEARCH → CONFIRM(2) → TRACK → HOLD(3miss) → LOST

    Drop-in replacement for AlphaBetaTracker.  Two consistent YOLO detections
    are required before locking; wild jumps (>MAX_TRACK_JUMP_PX) are rejected.
    During brief HOLD the last position coasts, so MSPM0 never sees a spurious
    single-frame gap.
    """

    SEARCH = 0; CONFIRM = 1; TRACK = 2; HOLD = 3; LOST = 4
    _NAMES = {0: "S", 1: "C", 2: "T", 3: "H", 4: "L"}

    def __init__(self):
        self.state = self.SEARCH
        self._x = float(ZERO_X_PX)
        self._misses = 0
        self._confirm_count = 0
        self._pending_x = 0.0

    @property
    def ready(self):
        return self.state in (self.TRACK, self.HOLD)

    @property
    def misses(self):
        return self._misses

    def state_name(self):
        return self._NAMES.get(self.state, "?")

    def predicted_x(self, now_ms):
        return self._x

    def use_local_roi(self):
        return self.state in (self.TRACK, self.HOLD)

    def update(self, measured_x, now_ms):
        measured_x = float(measured_x)

        if self.state == self.LOST:
            self._pending_x = measured_x
            self._confirm_count = 1
            self.state = self.CONFIRM
            return self._pending_x

        if self.state == self.SEARCH:
            self._pending_x = measured_x
            self._confirm_count = 1
            self.state = self.CONFIRM
            return self._pending_x

        if self.state == self.CONFIRM:
            if abs(measured_x - self._pending_x) <= MAX_TRACK_JUMP_PX:
                self._confirm_count += 1
                self._pending_x = self._pending_x * 0.6 + measured_x * 0.4
                if self._confirm_count >= CONFIRM_DETECTIONS:
                    self.state = self.TRACK
                    self._x = self._pending_x
                    self._misses = 0
                return self._pending_x
            self._pending_x = measured_x
            self._confirm_count = 1
            return self._pending_x

        # TRACK / HOLD
        if abs(measured_x - self._x) <= MAX_TRACK_JUMP_PX:
            if self.state == self.HOLD:
                self.state = self.TRACK
            self._x = self._x * (1.0 - FILTER_ALPHA) + measured_x * FILTER_ALPHA
            self._misses = 0
        else:
            self.miss()
        return self._x

    def miss(self):
        self._misses += 1
        if self.state == self.TRACK:
            self.state = self.HOLD
        elif self.state == self.HOLD and self._misses >= MISS_HOLD_DETECTIONS:
            self.state = self.LOST
            self._confirm_count = 0


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

def wifi_init_nonblock():
    """Init WiFi once — connect() runs async, never blocks the main loop."""
    try:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(WIFI_SSID, WIFI_PASS)
        print("[K230] WiFi connecting (non-blocking)...")
        return wlan
    except Exception as e:
        print("[K230] WiFi init failed:", e)
        return None


def pc_connect_nonblock():
    """Try PC connect without blocking the detection loop."""
    try:
        addr = socket.getaddrinfo(PC_IP, PC_PORT)[0][-1]
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.05)  # 50ms max — don't block detection
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
        uart.write(frame)               # bytearray直接写入,不copy
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


# Pre-allocated video encode buffer — avoids per-frame img.copy() allocation
_VIDEO_BUF = None  # created after sensor init (needs Display to be ready)

def encode_video_jpeg(capture_img, vision_marker=None):
    global _VIDEO_BUF
    if _VIDEO_BUF is None:
        _VIDEO_BUF = image.Image(VIDEO_W, VIDEO_H, image.RGB565)
    # draw_image into pre-allocated buffer (zero-allocation copy)
    _VIDEO_BUF.draw_image(capture_img, 0, 0, roi=PIPE_VIDEO_CROP)
    if STREAM_OVERLAY_ENABLE and vision_marker is not None:
        _draw_stream_marker(_VIDEO_BUF, vision_marker,
                            crop_offset=(PIPE_VIDEO_CROP[0], PIPE_VIDEO_CROP[1]))
    return _VIDEO_BUF.compress(quality=JPEG_Q)


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
# Disable YOLO library's draw_result — we don't use the IDE display, and its
# deprecated draw_string() calls flood the REPL UART with warnings, causing
# buffer stalls that slow the detection loop.
try:
    yolo.draw_result = lambda *a, **kw: None
    print("[K230] YOLO draw_result disabled (saves CPU, avoids REPL spam)")
except Exception:
    pass
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

tracker = BallTracker()
video_sender = VideoSender()
clock = time.clock()

frame_index = 0
last_video_enqueue_ms = 0

wlan = None
try:
    wlan = wifi_init_nonblock()
except Exception:
    pass

pc_sock = None
last_pc_cool_ms = 0    # Wi-Fi reconnect cool-down

video_count = 0
video_bytes = 0

print("[K230] Running: YOLO NPU → UART; JPEG WiFi streaming")

while True:
    clock.tick()
    now_ms = time.ticks_ms()

    # 1. Get AI frame from CHN_2 (RGBP888, 640x480)
    ai_img = sensor.snapshot(chn=CAM_CHN_ID_2)
    ai_np = ai_img.to_numpy_ref()

    # 2. YOLO NPU inference
    ball_valid = False
    ball_cx = 0.0
    ball_cy = 0.0
    ball_conf = 0.0
    box_w = 0
    box_h = 0
    if VISION_ENABLE:
        try:
            results = yolo.run(ai_np)
        except Exception:
            results = ([], [], [])
        if results and results[0]:
            # Find best detection INSIDE the pipe ROI
            best_idx = -1
            best_conf = 0.0
            roi_x0, roi_y0, roi_w, roi_h = PIPE_ROI
            roi_x1 = roi_x0 + roi_w
            roi_y1 = roi_y0 + roi_h
            for i in range(len(results[0])):
                score = float(results[2][i])
                if score < best_conf:
                    continue
                x, y, w, h = results[0][i]
                cx = float(x) + float(w) / 2.0
                cy = float(y) + float(h) / 2.0
                # Gate: ball center must be inside pipe ROI
                if not (roi_x0 <= cx <= roi_x1 and roi_y0 <= cy <= roi_y1):
                    continue
                best_conf = score
                best_idx = i
            if best_idx >= 0:
                x, y, w, h = results[0][best_idx]
                ball_cx = float(x) + float(w) / 2.0
                ball_cy = float(y) + float(h) / 2.0
                ball_conf = best_conf
                box_w = int(w)
                box_h = int(h)
                ball_valid = True

    # 3. Tracking & UART (sends EVERY frame — this is the critical path)
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
        if tracker.ready:
            # Coast on last known position during brief HOLD (≤MISS_HOLD_DETECTIONS frames)
            pred_x = tracker.predicted_x(now_ms)
            pred_cm = pixel_to_cm(pred_x)
            send_ball(True, int(round(pred_cm * 100.0)), 0, 40, True)
        else:
            send_ball(False, 0, 0, 0, False)

    # 4. WiFi streaming (non-blocking, lower priority than detection+UART)
    wifi_ok = (wlan is not None) and wlan.isconnected()
    if pc_sock is None and wifi_ok:
        if _ticks_diff(now_ms, last_pc_cool_ms) >= PC_RETRY_MS:
            last_pc_cool_ms = now_ms
            pc_sock = pc_connect_nonblock()

    # 5. JPEG encoding & streaming (from CHN_0 RGB565)
    if (pc_sock is not None and not video_sender.pending() and
            _ticks_diff(now_ms, last_video_enqueue_ms) >= VIDEO_INTERVAL_MS):
        try:
            stream_img = sensor.snapshot()  # CHN_0, RGB565
            jpeg = encode_video_jpeg(stream_img, stream_marker)
            video_sender.enqueue_payload(STREAM_CODEC_JPEG, jpeg)
            last_video_enqueue_ms = now_ms
        except Exception:
            video_sender.reset()

    if pc_sock is not None:
        completed_bytes, connection_stalled = video_sender.flush(pc_sock, now_ms)
        if completed_bytes > 0:
            video_bytes += completed_bytes
            video_count += 1
        if connection_stalled:
            try:
                pc_sock.close()
            except Exception:
                pass
            pc_sock = None
            last_pc_cool_ms = now_ms
            video_sender.reset()

    # 6. GC — reduced allocations, run even less often
    frame_index += 1
    if frame_index % 150 == 0:         # ~7 seconds (JPEG 帧体积变大, GC 更勤一点)
        try:
            gc.collect()
        except Exception:
            pass

    # Status report (lightweight, every 120 frames ≈ 4s)
    if frame_index % 120 == 0:
        fps_now = clock.fps()
        if ball_valid:
            x_cm = pixel_to_cm(ball_cx)
            print("[K230] {:.1f}fps x={:.2f}cm c={:.2f} [{}] uart:{}".format(
                fps_now, x_cm, ball_conf, tracker.state_name(), uart_seq))
        elif tracker.ready:
            x_cm = pixel_to_cm(tracker.predicted_x(now_ms))
            print("[K230] {:.1f}fps pred={:.2f}cm [{}] uart:{}".format(
                fps_now, x_cm, tracker.state_name(), uart_seq))
        else:
            print("[K230] {:.1f}fps NO_BALL [{}] uart:{}".format(
                fps_now, tracker.state_name(), uart_seq))
