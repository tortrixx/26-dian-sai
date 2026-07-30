"""K230 YOLO11 NPU steel-ball detector with JPEG WiFi streaming.

Replaces the motion-based frame-differencing detector with YOLO11n running
on the K230 NPU.  The NPU runs independently from the CPU, so ball detection
does NOT compete with JPEG encoding or UART transmission.

Architecture:
  Sensor CHN_2 (RGB888) → YOLO11 NPU → ball (cx,cy,conf)
  Sensor CHN_2 snapshot    → RGB565 → JPEG → TCP WiFi → PC receiver
  Ball position            → UART   → MSPM0

Requires:
  /sdcard/yolo11n_det_320.kmodel          # from Laoguigui2/K230-
  /sdcard/app/libs/{AIBase,AI2D,PipeLine,Utils,YOLO}.py   # kendryte SDK
"""

from libs.PipeLine import PipeLine, ScopedTiming
from libs.YOLO import YOLO11
from libs.Utils import *
from media.sensor import *
from media.display import *
from machine import UART, FPIOA
import image, time, network, socket, gc, uctypes

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

# ---- YOLO / NPU ----
MODEL_PATH = "/sdcard/yolo11n_det_320.kmodel"
LABELS = ["steel"]
RGB888P_SIZE = [640, 480]       # AI camera resolution (full frame for streaming)
MODEL_INPUT_SIZE = [320, 320]   # kmodel input
DISPLAY_SIZE = [640, 480]       # virtual display = same as streaming frame
CONF_THRESH = 0.4               # lower = more detections, more false positives
NMS_THRESH = 0.45
MAX_BOXES = 10
YOLO_DEBUG = 0

# ---- Vision / tracking ----
# Pixel-to-cm calibration.  *** MUST BE RECALIBRATED for your camera mount! ***
# These are PLACEHOLDER values.  See calibration procedure in k230_final.py.
ZERO_X_PX = 320.0               # Ball at 0 cm = center of 640-wide frame
PX_PER_CM = 12.0                # Approximate: 1cm ≈ 12px at this distance
PIPE_ROI = (0, 120, 640, 240)   # x, y, w, h in 640x480 streaming frame

# Alpha-beta tracker (same as k230_final.py)
TRACK_HALF_WIDTH_PX = 100
MAX_TRACK_SPEED_PX_S = 700.0
TRACK_MISS_LIMIT = 10
ALPHA = 0.65
BETA = 0.12

# ---- UART ----
UART_BAUD = 115200
UART_TX = 9
UART_RX = 10

# ---- Wire protocol (same as k230_final.py) ----
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
VIDEO_CONNECT_TIMEOUT_S = 0.30
VIDEO_STALL_TIMEOUT_MS = 1500
WIFI_RETRY_MS = 5000
PC_RETRY_MS = 2000

# Streaming (JPEG over TCP — proven stable)
STREAM_PROFILE = "pipe_detail"
STREAM_PROFILES = {
    "control":     (320, 240, 8, 70),
    "pipe_detail": (640, 240, 6, 50),
}
VIDEO_W, VIDEO_H, VIDEO_TARGET_FPS, JPEG_Q = STREAM_PROFILES[STREAM_PROFILE]
VIDEO_INTERVAL_MS = max(1, 1000 // VIDEO_TARGET_FPS)
PIPE_VIDEO_CROP = (0, 120, 640, 240)  # x, y, w, h in 640x480 raw frame

# K23V protocol for TCP
STREAM_MAGIC = b'K23V'
STREAM_VERSION = 1
STREAM_CODEC_JPEG = 1

# Streaming overlay
STREAM_OVERLAY_ENABLE = True
STREAM_OVERLAY_BOX_COLOR = (255, 0, 0)
STREAM_OVERLAY_CROSS_COLOR = (0, 255, 0)
STREAM_OVERLAY_THICKNESS = 2
STREAM_OVERLAY_CROSS_SIZE = 12

OVERLAY_ENABLE = False
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
    """Linear pixel-to-cm conversion.  Recalibrate after camera mount!"""
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
    """Non-blocking packetised sender.  One packet = one complete JPEG frame."""
    def __init__(self):
        self._pending = None
        self._sent = 0
        self._start_ms = 0

    def enqueue_payload(self, codec, jpeg_bytes):
        self._pending = (codec, jpeg_bytes)
        self._sent = 0

    def pending(self):
        return self._pending is not None

    def reset(self):
        self._pending = None
        self._sent = 0

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
            header[6] = (length >> 16) & 0xFF
            header[7] = (length >> 8) & 0xFF
            header[8] = length & 0xFF
            self._buf = header + payload
            self._buf_len = len(self._buf)
        total_sent = 0
        try:
            while self._sent < self._buf_len:
                chunk = min(1400, self._buf_len - self._sent)
                sent = sock.send(self._buf[self._sent:self._sent + chunk])
                if sent <= 0:
                    self._pending = None
                    self._sent = 0
                    return 0, True
                self._sent += sent
                total_sent += sent
        except Exception:
            self._pending = None
            self._sent = 0
            return 0, True
        if self._sent >= self._buf_len:
            self._pending = None
            self._sent = 0
            return total_sent, False
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

def _draw_stream_marker(canvas, marker, source_mode="capture_pipe",
                        frame_w=640, frame_h=480):
    if marker is None:
        return
    bx, by, bw, bh, cx, cy = marker
    if source_mode == "capture_pipe":
        # marker coords are in 640x480 sensor frame
        canvas.draw_rectangle(bx, by, bw, bh,
                              color=STREAM_OVERLAY_BOX_COLOR,
                              thickness=STREAM_OVERLAY_THICKNESS)
        canvas.draw_cross(int(cx), int(cy),
                          color=STREAM_OVERLAY_CROSS_COLOR,
                          size=STREAM_OVERLAY_CROSS_SIZE,
                          thickness=STREAM_OVERLAY_THICKNESS)


def encode_video_jpeg(capture_img, vision_marker=None):
    """Crop pipe strip from full frame, draw overlay, return JPEG bytes."""
    pipe_crop = capture_img.copy(roi=PIPE_VIDEO_CROP)
    canvas = pipe_crop
    if STREAM_OVERLAY_ENABLE and vision_marker is not None:
        _draw_stream_marker(canvas, vision_marker, source_mode="capture_pipe",
                            frame_w=640, frame_h=480)
    return bytes(canvas.compress(quality=JPEG_Q))


# ============ Main ============

print("[K230] YOLO11 steel-ball detector starting...")

# UART
fpioa = FPIOA()
fpioa.set_function(UART_TX, FPIOA.UART1_TXD)
fpioa.set_function(UART_RX, FPIOA.UART1_RXD)
uart = UART(UART.UART1, UART_BAUD, 8, 1, 0, UART_TX, UART_RX)
print("[K230] UART1 IO{}/IO{} {} baud OK".format(UART_TX, UART_RX, UART_BAUD))

# PipeLine: manages dual-channel sensor + virtual display.
# CHN_0 (YUV420) → virtual display (Display.VIRT)
# CHN_2 (RGB888) → AI processing
pl = PipeLine(
    rgb888p_size=RGB888P_SIZE,
    display_mode="virt",
    display_size=DISPLAY_SIZE,
    debug_mode=0
)
pl.create(sensor_id=2)
print("[K230] PipeLine created: AI {}x{} @ virt display {}x{}".format(
    RGB888P_SIZE[0], RGB888P_SIZE[1], DISPLAY_SIZE[0], DISPLAY_SIZE[1]))

# YOLO11
yolo = YOLO11(
    task_type="detect",
    mode="video",
    kmodel_path=MODEL_PATH,
    labels=LABELS,
    rgb888p_size=RGB888P_SIZE,
    model_input_size=MODEL_INPUT_SIZE,
    display_size=DISPLAY_SIZE,
    conf_thresh=CONF_THRESH,
    nms_thresh=NMS_THRESH,
    max_boxes_num=MAX_BOXES,
    debug_mode=YOLO_DEBUG
)
yolo.config_preprocess()
print("[K230] YOLO11 model loaded: {}".format(MODEL_PATH))

# Streaming
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

perf_capture_ms = 0
perf_yolo_ms = 0
perf_jpeg_ms = 0
perf_send_ms = 0

print("[K230] Running: YOLO11 NPU -> UART; jpeg_cpu video")

while True:
    clock.tick()
    now_ms = time.ticks_ms()

    # 1. Get AI frame from sensor CHN_2 (RGB888)
    cap_start = time.ticks_ms()
    ai_img = pl.sensor.snapshot(chn=CAM_CHN_ID_2)
    ai_np = ai_img.to_numpy_ref()
    perf_capture_ms += _ticks_diff(time.ticks_ms(), cap_start)

    # 2. YOLO NPU inference
    yolo_start = time.ticks_ms()
    ball_valid = False
    ball_cx = 0.0
    ball_cy = 0.0
    ball_conf = 0.0
    results = []
    if VISION_ENABLE:
        try:
            results = yolo.run(ai_np)
        except Exception as e:
            print("[K230] YOLO run error:", e)
            results = []
        # results format: [boxes_list, class_ids, scores]
        # boxes_list[i] = [x, y, w, h] in display_size coords
        if results:
            best_idx = -1
            best_conf = 0.0
            for i in range(len(results[0])):
                cls_id = int(results[1][i])
                score = float(results[2][i])
                if cls_id == 0 and score > best_conf:  # class 0 = steel
                    best_conf = score
                    best_idx = i
            if best_idx >= 0:
                x, y, w, h = results[0][best_idx]
                ball_cx = float(x) + float(w) / 2.0
                ball_cy = float(y) + float(h) / 2.0
                ball_conf = best_conf
                ball_valid = True
    perf_yolo_ms += _ticks_diff(time.ticks_ms(), yolo_start)

    # 3. Tracking & UART
    stream_marker = None
    if ball_valid:
        filtered_x = tracker.update(ball_cx, now_ms)
        x_cm = pixel_to_cm(filtered_x)
        x_cm_x100 = int(round(x_cm * 100.0))
        quality = int(_clamp(ball_conf * 100.0, 10, 90))
        marker_w = int(w) if ball_valid else 12
        marker_h = int(h) if ball_valid else 12
        stream_marker = (int(ball_cx - marker_w / 2), int(ball_cy - marker_h / 2),
                         marker_w, marker_h, int(filtered_x), int(ball_cy))
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

    # 5. JPEG encoding & streaming
    if (pc_sock is not None and not video_sender.pending() and
            _ticks_diff(now_ms, last_video_enqueue_ms) >= VIDEO_INTERVAL_MS):
        try:
            jpeg_start = time.ticks_ms()
            # Convert AI's RGB888 frame to RGB565 for JPEG compression
            stream_img = ai_img.to_rgb565()
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
            print("[K230] video socket stalled; reconnecting")
            try:
                pc_sock.close()
            except Exception:
                pass
            pc_sock = None
            video_sender.reset()

    # 6. Status & GC
    frame_index += 1
    if frame_index % 60 == 0:
        try:
            gc.collect()
        except Exception:
            pass

    if frame_index % 30 == 0:
        state = "x={:.2f}cm c={:.2f}".format(x_cm_x100 / 100.0, ball_conf) if ball_valid else "NO BALL"
        stat_elapsed_ms = max(1, _ticks_diff(now_ms, video_stat_start_ms))
        video_fps = video_count * 1000.0 / stat_elapsed_ms
        video_kb_s = video_bytes * 1000.0 / stat_elapsed_ms / 1024.0
        if PERFORMANCE_LOG:
            print("[K230] Loop:{:.1f} {} UART:{} Video:{:.1f}fps {:.1f}KB/s "
                  "ms/frame cap:{:.1f} yolo:{:.1f} enc:{:.1f} net:{:.1f}".format(
                clock.fps(), state, uart_seq, video_fps, video_kb_s,
                perf_capture_ms / 30.0, perf_yolo_ms / 30.0,
                perf_jpeg_ms / 30.0, perf_send_ms / 30.0
            ))
        else:
            print("[K230] Loop:{:.1f} {} UART:{} Video:{:.1f}fps {:.1f}KB/s".format(
                clock.fps(), state, uart_seq, video_fps, video_kb_s))
        video_stat_start_ms = now_ms
        video_count = 0
        video_bytes = 0
        perf_capture_ms = 0
        perf_yolo_ms = 0
        perf_jpeg_ms = 0
        perf_send_ms = 0
