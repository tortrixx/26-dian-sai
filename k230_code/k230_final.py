"""K230 traditional-vision steel-ball detector and MSPM0 UART transmitter.

This is the only K230 program to deploy for the ball-control path.  It does
not use YOLO: a calibrated pipe band, adaptive lightness thresholds, blob
shape gates and an alpha-beta tracker identify an unpainted steel ball.

UART payload:
    AA 55 | LEN | 01 | SEQ | flags | x_cm_x100 | y_offset_px | quality | sum

``x_cm_x100`` is the calibrated ball-center position along the pipe.  The PC
receives video only; failure to connect or stream must never stop UART output.
"""

from media.sensor import *
from media.display import *
from machine import UART, FPIOA
import image, time, network, socket, gc, uctypes

# Keep an explicit reference before importing the encoder module.  The
# Yahboom v1.4.3 firmware exports overlapping names from some media modules;
# a wildcard VENC import can replace ``Sensor`` and breaks construction with
# ``'Sensor' object has no attribute 'buf_init'``.
K230Sensor = Sensor

# Do not import ``media.vencoder`` at module load time.  On the Yahboom
# v1.4.3 image, merely importing it before ordinary camera construction can
# disturb the sensor module.  Load it only when h264_hw is explicitly chosen.
venc = None
H264_MODULE_AVAILABLE = None


def _load_venc_module():
    global venc, H264_MODULE_AVAILABLE
    if H264_MODULE_AVAILABLE is not None:
        return H264_MODULE_AVAILABLE
    try:
        import media.vencoder as venc_module
        venc = venc_module
        H264_MODULE_AVAILABLE = True
    except Exception:
        venc = None
        H264_MODULE_AVAILABLE = False
    return H264_MODULE_AVAILABLE


# ---- 0. Clean up an IDE-created media pipeline. ----
try:
    Display.deinit()
except Exception:
    pass
try:
    MediaManager.deinit()
except Exception:
    pass


# ---- 1. Wire protocol.  Keep this self-contained for /sdcard deployment. ----
PROTO_HEAD_0 = 0xAA
PROTO_HEAD_1 = 0x55
MSG_VISION_TARGET = 0x01
VISION_FLAG_VALID = 0x01
VISION_FLAG_TRACKED = 0x02


def _checksum(buf, start, end):
    value = 0
    for index in range(start, end):
        value = (value + buf[index]) & 0xFF
    return value


def _clamp_i16(value):
    value = int(value)
    return max(-32768, min(32767, value))


def _clamp_u8(value):
    value = int(value)
    return max(0, min(255, value))


def _put_i16_le(buf, offset, value):
    value = _clamp_i16(value) & 0xFFFF
    buf[offset] = value & 0xFF
    buf[offset + 1] = (value >> 8) & 0xFF


def build_vision_frame(seq, valid, x_cm_x100, y_offset_px, quality=0,
                       tracked=False):
    """Build the 12-byte vision frame consumed by the MSPM0."""
    payload = bytearray(6)
    flags = 0
    if valid:
        flags |= VISION_FLAG_VALID
    if tracked:
        flags |= VISION_FLAG_TRACKED
    payload[0] = flags
    _put_i16_le(payload, 1, x_cm_x100)
    _put_i16_le(payload, 3, y_offset_px)
    payload[5] = _clamp_u8(quality)

    length = len(payload) + 2  # TYPE + SEQ + payload
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


# ============ Configuration: calibrate the values in this section ============
# Camera. Keep the proven VGA sensor output to preserve the full pipe field of
# view. The fast profile then downscales that full frame to QVGA internally for
# blob detection, so it gains speed without forcing a higher camera mount.
SENSOR_ID = 2
VISION_PROFILE = "fast_qvga"  # "fast_qvga" (competition) or "vga_precision"
VISION_PROFILES = {
    # sensor_w, sensor_h, vision_w, vision_h, zero_x, px_per_cm, pipe_roi,
    # min_pixels, max_pixels, nominal_pixels, track_half_width, max_speed
    # Latest real-ball five-point table (2026-07-29): visual zero=160.47 px.
    # Re-validate after any camera/focus change.
    "fast_qvga": (640, 480, 320, 240, 160.47, 11.80, (10, 68, 300, 110),
                   40, 140, 78, 50, 350.0),
    "vga_precision": (640, 480, 640, 480, 320, 15.0, (20, 135, 600, 220),
                      25, 600, 150, 100, 700.0),
}
(CAPTURE_W, CAPTURE_H, VISION_W, VISION_H, ZERO_X_PX, PX_PER_CM, PIPE_ROI,
 MIN_BLOB_PIXELS, MAX_BLOB_PIXELS, BALL_PIXELS_NOMINAL,
 TRACK_HALF_WIDTH_PX, MAX_TRACK_SPEED_PX_S) = VISION_PROFILES[VISION_PROFILE]
CROP_W, CROP_H = VISION_W, VISION_H
IDE_PREVIEW = False  # IDE USB preview is unnecessary during Wi-Fi deployment.

# Pixel-to-position calibration.  The five measured positions are not quite
# equally spaced in pixels because of side-view perspective and the physical
# tube geometry.  Piecewise interpolation is exact at every measured mark and
# stays monotonic between marks; remeasure it after any camera/mount change.
POSITION_MODEL = "piecewise"  # "piecewise" (current), "quadratic" or "linear".
POSITION_CAL_PX = (47.19, 102.57, 160.47, 221.28, 270.67)
POSITION_CAL_CM = (-10.0, -5.0, 0.0, 5.0, 10.0)
POSITION_QUAD_A = -0.0000395610
POSITION_QUAD_B = 0.1003420632
POSITION_QUAD_C = -14.8800357496
# Real-ball samples occupy y=105..112 in the internal 320x240 vision image.
# The wide ±35 px gate still covers the expected pipe displacement at ±8 deg,
# while rejecting the upper/lower pipe edges seen during initial diagnostics.
PIPE_CENTER_Y = 107
MAX_BALL_Y_OFFSET_PX = 35

# Ball appearance calibration.  The raw steel ball can appear either bright or
# dark, hence two adaptive L-channel thresholds are searched every frame.
MAX_ASPECT_RATIO = 1.70
MIN_ROUNDNESS = 0.35  # Used only when this CanMV Blob API exposes roundness().
L_CONTRAST_MARGIN = 12
MIN_QUALITY = 50
# Bright and dark masks must be scanned separately.  Combining them makes the
# dark pipe seam and bright pipe surface one large foreground region, hiding a
# steel ball that is otherwise visible to the camera.
SEPARATE_BRIGHT_DARK_BLOBS = True
# Once a ball has been tracked, its current bright/dark appearance normally
# persists for several frames.  Search that polarity first to avoid doing two
# expensive find_blobs() passes per control frame.  A miss or weak candidate
# immediately falls back to both polarities in the *same* frame.
TRACK_POLARITY_FAST_PATH = True
TRACK_POLARITY_FAST_MIN_QUALITY = 65

# Real-ball calibration helper.  Keep this False for normal operation.  When
# enabled, place the stationary ball at one known physical mark, set the label
# below, and copy one or more ``[CAL]`` summary lines from the IDE terminal.
# The fields use the *internal vision image* (320x240 for fast_qvga), not the
# 640x480 sensor pixels.  Those raw values are what the five-point fit needs.
CALIBRATION_ENABLE = False  # Enable only while collecting new calibration data.
CALIBRATION_LABEL = "five_point_manual"
CALIBRATION_REPORT_FRAMES = 60  # Valid observations per summary, about 2.5 s.
# A calibration summary is meaningful only while the manually placed ball is
# stationary.  A new position or a rolling ball resets the current window.
CALIBRATION_STABLE_SPREAD_PX = 2.5
# First-real-ball diagnostic. Enable it with OVERLAY_ENABLE to draw yellow
# boxes around raw candidates and print why a candidate is rejected. Turn it
# back off after the detector accepts the real ball; terminal I/O costs FPS.
CALIBRATION_DIAGNOSTIC = False
CALIBRATION_DIAGNOSTIC_EVERY = 30
calibration_count = 0
calibration_sum_cx = 0.0
calibration_sum_cy = 0.0
calibration_sum_cx2 = 0.0
calibration_sum_pixels = 0.0
calibration_sum_w = 0.0
calibration_sum_h = 0.0
calibration_sum_quality = 0.0
calibration_diagnostic_frames = 0

# Alpha-beta tracking and reacquisition.
TRACK_MISS_LIMIT = 3
ALPHA = 0.65
BETA = 0.12

# UART: K230 TX -> MSPM0 RX, K230 RX <- MSPM0 TX (optional), and common ground.
UART_BAUD = 115200
UART_TX = 9
UART_RX = 10

# Wi-Fi is display/recording only.  Socket attempts are short and rate-limited
# so absence of a PC cannot stall ball detection or the control UART.
# Fill these two values locally before deployment.  Do not commit a real
# hotspot credential to the repository or include it in logs/screenshots.
WIFI_SSID = "YOUR_HOTSPOT_SSID"
WIFI_PASS = "YOUR_HOTSPOT_PASSWORD"
PC_IP = "192.168.137.1"
PC_PORT = 8888
MAGIC = b'\xA5\x5A\xA5\x5A'

# Video backends:
# - "h264_hw": experimental sensor CH0 YUV420SP -> VENC H.264 and CH1 RGB565
#   vision path. It is **incompatible with the current Yahboom CanMV v1.4.3**
#   firmware (``buf_init`` failure) and must remain disabled.
# - "jpeg_cpu": previous all-RGB565 software JPEG path.  Retained only as a
#   compatibility fallback; it shares the CPU with vision and was measured at
#   about 9 FPS combined.
# - "off": no PC video; UART vision control keeps running.
# Keep the deployed control program on the known-good software-JPEG path.
VIDEO_BACKEND = "jpeg_cpu"

# JPEG profiles are used only by ``jpeg_cpu``.  ``pipe_detail`` keeps the
# control image at QVGA but encodes a 640-pixel-wide crop directly from the
# original sensor frame, so physical pipe ticks are not blurred by the vision
# downscale.  H.264 uses target FPS only when a compatible firmware exists.
STREAM_PROFILE = "pipe_detail"
STREAM_PROFILES = {
    # profile: (stream_width, stream_height, target_fps, JPEG_quality)
    "control":     (320, 240, 8, 70),
    "balanced":    (320, 240, 15, 70),
    # These profiles only enlarge/downscale the 320x240 vision image; they do
    # not add pipe-tick detail and are retained for comparison only.
    "clarity":     (480, 360, 12, 65),
    "quality":     (480, 360, 10, 75),
    # 640x240 original-frame crop, Q80 @8fps: recommended single-camera
    # evidence stream while 320x240 vision/UART remains the control priority.
    "pipe_detail": (640, 240, 8, 80),
}
VIDEO_W, VIDEO_H, VIDEO_TARGET_FPS, JPEG_Q = STREAM_PROFILES[STREAM_PROFILE]
VIDEO_INTERVAL_MS = max(1, 1000 // VIDEO_TARGET_FPS)

# ``vision`` encodes the 320x240 detector image. ``capture_pipe`` encodes a
# full-width horizontal crop from the 640x480 sensor frame. The crop covers the
# calibrated pipe activity band plus tilt margin (source-frame coordinates).
VIDEO_SOURCE_MODE = ("capture_pipe" if STREAM_PROFILE == "pipe_detail"
                     else "vision")
PIPE_VIDEO_CROP = (0, 120, 640, 240)  # x, y, w, h in the 640x480 raw frame.

# Keep vision and display paths separate. The installed Yahboom v1.4.3 image
# cannot configure the required YUV/VENC path, so JPEG is software encoded.
VIDEO_SCALE_X = VIDEO_W / float(VISION_W)
VIDEO_SCALE_Y = VIDEO_H / float(VISION_H)

# Hardware H.264 transport.  Baseline is the most widely decodable profile.
# 900 kbit/s is comfortable on a clean 2.4 GHz hotspot at VGA and materially
# cleaner than the prior Q50 software JPEG stream.  Lower it to 650 if the
# hotspot is crowded; raise it to 1200 only after a stable test.
H264_W, H264_H = 640, 480
H264_BITRATE_KBPS = 900
H264_GOP = 15  # A reconnect waits at most about one second for an IDR frame.
H264_VENC_CHN = 0
H264_OUT_BUFS = 6

VIDEO_CONNECT_TIMEOUT_S = 0.30
VIDEO_STALL_TIMEOUT_MS = 1500
WIFI_RETRY_MS = 5000
PC_RETRY_MS = 2000

# Versioned TCP transport.  A packet carries one complete JPEG or H.264 access
# unit, so PC loss/reconnect cannot contaminate the next frame boundary.
STREAM_MAGIC = b'K23V'
STREAM_VERSION = 1
STREAM_CODEC_JPEG = 1
STREAM_CODEC_H264 = 2

# Debug overlays are not required for the competition video and cost CPU time.
# Keep them off for normal streaming; the serial log still reports detection.
OVERLAY_ENABLE = False
OVERLAY_TEXT_SIZE = 20

# Set False only for a 30 s Wi-Fi/JPEG benchmark before the steel ball arrives.
# Normal competition operation must keep this True.
VISION_ENABLE = True
PERFORMANCE_LOG = True
# ===============================================================================


def _ticks_diff(now, then):
    try:
        return time.ticks_diff(now, then)
    except Exception:
        return now - then


def _clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _safe_roi(x, y, width, height):
    x = max(0, min(CROP_W - 1, int(x)))
    y = max(0, min(CROP_H - 1, int(y)))
    right = max(x + 1, min(CROP_W, int(x + width)))
    bottom = max(y + 1, min(CROP_H, int(y + height)))
    return (x, y, right - x, bottom - y)


overlay_advanced_available = True


def _draw_overlay_text(img, x, y, text, color):
    """Use the CanMV v1.4.3 API without producing a deprecation warning/frame."""
    global overlay_advanced_available
    if not OVERLAY_ENABLE or not overlay_advanced_available:
        return
    try:
        img.draw_string_advanced(x, y, OVERLAY_TEXT_SIZE, text, color=color)
    except Exception:
        overlay_advanced_available = False
        print("[K230] draw_string_advanced unavailable; overlays disabled")


def _blob_roundness(blob):
    """Return (available, roundness) without assuming a specific CanMV build."""
    try:
        return True, float(blob.roundness())
    except Exception:
        return False, 0.0


def _candidate_quality(blob, predicted_x, using_track_roi):
    pixels = blob.pixels()
    if pixels < MIN_BLOB_PIXELS or pixels > MAX_BLOB_PIXELS:
        return -1

    if abs(blob.cy() - PIPE_CENTER_Y) > MAX_BALL_Y_OFFSET_PX:
        return -1

    width = blob.w()
    height = blob.h()
    if width <= 1 or height <= 1:
        return -1
    aspect = float(max(width, height)) / float(min(width, height))
    if aspect > MAX_ASPECT_RATIO:
        return -1

    roundness_available, roundness = _blob_roundness(blob)
    if roundness_available and roundness < MIN_ROUNDNESS:
        return -1

    area_error = abs(pixels - BALL_PIXELS_NOMINAL) / float(max(1, BALL_PIXELS_NOMINAL))
    area_score = _clamp(1.0 - area_error, 0.0, 1.0)
    aspect_score = _clamp(1.0 - (aspect - 1.0) / max(0.01, MAX_ASPECT_RATIO - 1.0), 0.0, 1.0)
    score = 45.0 * area_score + 30.0 * aspect_score

    if roundness_available:
        round_score = _clamp(
            (roundness - MIN_ROUNDNESS) / max(0.01, 1.0 - MIN_ROUNDNESS), 0.0, 1.0
        )
        score += 15.0 * round_score
    else:
        score += 15.0  # Ratio + area gates are the firmware-independent fallback.

    if using_track_roi:
        distance = abs(blob.cx() - predicted_x)
        score += 10.0 * _clamp(1.0 - distance / float(TRACK_HALF_WIDTH_PX), 0.0, 1.0)

    return int(_clamp(score, 0.0, 100.0))


def _pixel_to_cm(x_px):
    """Convert internal vision x coordinate to physical pipe position."""
    x_px = float(x_px)
    if POSITION_MODEL == "piecewise":
        # Use the first/last local slope for rare observations just outside the
        # calibrated interval; within it, every knot maps to its physical mark.
        last = len(POSITION_CAL_PX) - 1
        index = 0
        if x_px >= POSITION_CAL_PX[last]:
            index = last - 1
        else:
            for index in range(last):
                if x_px <= POSITION_CAL_PX[index + 1]:
                    break
        px0 = POSITION_CAL_PX[index]
        px1 = POSITION_CAL_PX[index + 1]
        cm0 = POSITION_CAL_CM[index]
        cm1 = POSITION_CAL_CM[index + 1]
        return cm0 + (x_px - px0) * (cm1 - cm0) / (px1 - px0)
    if POSITION_MODEL == "quadratic":
        return (POSITION_QUAD_A * x_px * x_px +
                POSITION_QUAD_B * x_px + POSITION_QUAD_C)
    return (x_px - ZERO_X_PX) / PX_PER_CM


def _adaptive_thresholds(img, roi):
    """Build bright and dark LAB thresholds from the current pipe-band lightness."""
    try:
        stats = img.get_statistics(roi=roi)
        l_mean = int(stats.l_mean())
    except Exception:
        # A conservative fallback still uses only L, never a painted-ball color.
        l_mean = 50

    bright_l = int(_clamp(l_mean + L_CONTRAST_MARGIN, 0, 100))
    dark_l = int(_clamp(l_mean - L_CONTRAST_MARGIN, 0, 100))
    # Full a/b ranges deliberately make this a brightness, not color, detector.
    return (
        (bright_l, 100, -128, 127, -128, 127),
        (0, dark_l, -128, 127, -128, 127),
    )


def _diagnose_candidates(img, candidates, bright_threshold, dark_threshold,
                         predicted_x, using_track_roi, bright_count, dark_count):
    """Report raw candidates while tuning the first real-ball detection."""
    global calibration_diagnostic_frames
    if not CALIBRATION_DIAGNOSTIC:
        return
    calibration_diagnostic_frames += 1
    if calibration_diagnostic_frames % CALIBRATION_DIAGNOSTIC_EVERY != 0:
        return

    print("[CAL-DIAG] n={} (bright={} dark={}) Lbright>={} Ldark<={} roi={} tracked={}".format(
        len(candidates), bright_count, dark_count,
        bright_threshold[0], dark_threshold[1],
        PIPE_ROI, using_track_roi))
    for index in range(min(6, len(candidates))):
        blob = candidates[index]
        width = blob.w()
        height = blob.h()
        aspect = float(max(width, height)) / float(max(1, min(width, height)))
        has_roundness, roundness = _blob_roundness(blob)
        score = _candidate_quality(blob, predicted_x, using_track_roi)
        print("[CAL-DIAG] #{} cx={} cy={} px={} wh={}x{} ar={:.2f} r={} q={}".format(
            index, blob.cx(), blob.cy(), blob.pixels(), width, height, aspect,
            "{:.2f}".format(roundness) if has_roundness else "NA", score))
        if OVERLAY_ENABLE:
            try:
                img.draw_rectangle(blob.rect(), color=(255, 255, 0), thickness=1)
            except Exception:
                pass


def _choose_best_tagged_candidate(tagged_candidates, predicted_x,
                                  using_track_roi):
    """Return the best (blob, quality, polarity) from tagged Blob results."""
    best = None
    best_quality = -1
    best_polarity = None
    for blob, polarity in tagged_candidates:
        quality = _candidate_quality(blob, predicted_x, using_track_roi)
        if quality > best_quality:
            best = blob
            best_quality = quality
            best_polarity = polarity
    return best, best_quality, best_polarity


def _find_best_blob(img, roi, predicted_x, using_track_roi,
                    preferred_polarity=None):
    bright_threshold, dark_threshold = _adaptive_thresholds(img, roi)
    bright_candidates = []
    dark_candidates = []
    candidates = []
    tagged_candidates = []
    bright_scanned = False
    dark_scanned = False
    try:
        if SEPARATE_BRIGHT_DARK_BLOBS:
            # Do not OR the polarities into one foreground image: on the real
            # white/gray pipe this joined the complete pipe edge into a
            # 294x34-pixel blob and concealed the ball.
            if (TRACK_POLARITY_FAST_PATH and using_track_roi and
                    preferred_polarity in ("bright", "dark")):
                if preferred_polarity == "bright":
                    bright_candidates = img.find_blobs(
                        [bright_threshold], roi=roi,
                        pixels_threshold=MIN_BLOB_PIXELS,
                        area_threshold=MIN_BLOB_PIXELS, merge=False
                    )
                    bright_scanned = True
                else:
                    dark_candidates = img.find_blobs(
                        [dark_threshold], roi=roi,
                        pixels_threshold=MIN_BLOB_PIXELS,
                        area_threshold=MIN_BLOB_PIXELS, merge=False
                    )
                    dark_scanned = True
                tagged_candidates = ([(blob, "bright") for blob in bright_candidates] +
                                     [(blob, "dark") for blob in dark_candidates])
                fast_best, fast_quality, fast_polarity = _choose_best_tagged_candidate(
                    tagged_candidates, predicted_x, using_track_roi
                )
                if fast_quality >= TRACK_POLARITY_FAST_MIN_QUALITY:
                    candidates = bright_candidates + dark_candidates
                    _diagnose_candidates(
                        img, candidates, bright_threshold, dark_threshold,
                        predicted_x, using_track_roi,
                        len(bright_candidates), len(dark_candidates)
                    )
                    return fast_best, fast_quality, fast_polarity

            # First acquisition, a weak preferred result, or a changed ball
            # reflection: inspect both polarities before declaring a miss.
            if not bright_scanned:
                bright_candidates = img.find_blobs(
                    [bright_threshold], roi=roi,
                    pixels_threshold=MIN_BLOB_PIXELS,
                    area_threshold=MIN_BLOB_PIXELS, merge=False
                )
            if not dark_scanned:
                dark_candidates = img.find_blobs(
                    [dark_threshold], roi=roi,
                    pixels_threshold=MIN_BLOB_PIXELS,
                    area_threshold=MIN_BLOB_PIXELS, merge=False
                )
            candidates = bright_candidates + dark_candidates
            tagged_candidates = ([(blob, "bright") for blob in bright_candidates] +
                                 [(blob, "dark") for blob in dark_candidates])
        else:
            # Kept only for performance comparison; not suitable for the
            # current reflective pipe/steel-ball scene.
            candidates = img.find_blobs(
                [bright_threshold, dark_threshold], roi=roi,
                pixels_threshold=MIN_BLOB_PIXELS,
                area_threshold=MIN_BLOB_PIXELS, merge=True
            )
            bright_candidates = candidates
            dark_candidates = []
            tagged_candidates = [(blob, "combined") for blob in candidates]
    except Exception:
        # Keep the safety path alive even on a firmware-specific image failure.
        candidates = []
        bright_candidates = []
        dark_candidates = []
        tagged_candidates = []

    _diagnose_candidates(img, candidates, bright_threshold, dark_threshold,
                         predicted_x, using_track_roi,
                         len(bright_candidates), len(dark_candidates))

    best, best_quality, best_polarity = _choose_best_tagged_candidate(
        tagged_candidates, predicted_x, using_track_roi
    )

    if best_quality < MIN_QUALITY:
        return None, 0, None
    return best, best_quality, best_polarity


def _calibration_sample(blob, quality):
    """Periodically print raw blob statistics for the five-point calibration."""
    global calibration_count, calibration_sum_cx, calibration_sum_cy
    global calibration_sum_cx2, calibration_sum_pixels, calibration_sum_w
    global calibration_sum_h, calibration_sum_quality

    if not CALIBRATION_ENABLE:
        return

    cx = float(blob.cx())
    if calibration_count:
        running_cx = calibration_sum_cx / float(calibration_count)
        if abs(cx - running_cx) > CALIBRATION_STABLE_SPREAD_PX:
            print("[CAL] {} motion/reset; waiting for a stable ball".format(
                CALIBRATION_LABEL))
            calibration_count = 0
            calibration_sum_cx = 0.0
            calibration_sum_cy = 0.0
            calibration_sum_cx2 = 0.0
            calibration_sum_pixels = 0.0
            calibration_sum_w = 0.0
            calibration_sum_h = 0.0
            calibration_sum_quality = 0.0

    calibration_count += 1
    calibration_sum_cx += cx
    calibration_sum_cy += float(blob.cy())
    calibration_sum_cx2 += cx * cx
    calibration_sum_pixels += float(blob.pixels())
    calibration_sum_w += float(blob.w())
    calibration_sum_h += float(blob.h())
    calibration_sum_quality += float(quality)

    if calibration_count < CALIBRATION_REPORT_FRAMES:
        return

    count = float(calibration_count)
    mean_cx = calibration_sum_cx / count
    cx_var = max(0.0, calibration_sum_cx2 / count - mean_cx * mean_cx)
    print("[CAL] {} n={} cx={:.2f} cy={:.2f} px={:.1f} wh={:.2f}x{:.2f} "
          "q={:.1f} sx={:.3f}px".format(
              CALIBRATION_LABEL, calibration_count, mean_cx,
              calibration_sum_cy / count, calibration_sum_pixels / count,
              calibration_sum_w / count, calibration_sum_h / count,
              calibration_sum_quality / count, cx_var ** 0.5))

    calibration_count = 0
    calibration_sum_cx = 0.0
    calibration_sum_cy = 0.0
    calibration_sum_cx2 = 0.0
    calibration_sum_pixels = 0.0
    calibration_sum_w = 0.0
    calibration_sum_h = 0.0
    calibration_sum_quality = 0.0


class AlphaBetaTracker:
    def __init__(self):
        self.ready = False
        self.x = float(ZERO_X_PX)
        self.v = 0.0
        self.last_ms = 0
        self.misses = 0
        self.polarity = None

    def predicted_x(self, now_ms):
        if not self.ready:
            return float(ZERO_X_PX)
        dt_s = _clamp(_ticks_diff(now_ms, self.last_ms) / 1000.0, 0.0, 0.25)
        return self.x + self.v * dt_s

    def update(self, measured_x, now_ms, polarity=None):
        measured_x = float(measured_x)
        if polarity is not None:
            self.polarity = polarity
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


def detect_ball(img, tracker, now_ms):
    """Run one traditional-vision observation and return the UART fields."""
    predicted_x = tracker.predicted_x(now_ms)
    tracked = tracker.use_local_roi()
    if tracked:
        search_roi = _safe_roi(
            predicted_x - TRACK_HALF_WIDTH_PX, PIPE_ROI[1],
            TRACK_HALF_WIDTH_PX * 2, PIPE_ROI[3]
        )
    else:
        search_roi = PIPE_ROI

    best, quality, polarity = _find_best_blob(
        img, search_roi, predicted_x, tracked,
        tracker.polarity if tracked else None
    )
    if best is None:
        tracker.miss()
        _draw_overlay_text(img, 2, 2, "NO BALL ({})".format(tracker.misses),
                           color=(255, 0, 0))
        return False, 0, 0, 0, tracked

    _calibration_sample(best, quality)
    filtered_x_px = tracker.update(best.cx(), now_ms, polarity)
    x_cm = _pixel_to_cm(filtered_x_px)
    x_cm_x100 = int(round(x_cm * 100.0))
    y_offset_px = best.cy() - PIPE_CENTER_Y

    if OVERLAY_ENABLE:
        img.draw_rectangle(best.rect(), color=(255, 0, 0), thickness=2)
        img.draw_cross(best.cx(), best.cy(), color=(0, 255, 0), size=10, thickness=2)
        _draw_overlay_text(img, 2, 2, "x={:.2f}cm q={}".format(x_cm, quality),
                           color=(255, 255, 255))
    return True, x_cm_x100, y_offset_px, quality, tracked


# ---- UART ----
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

uart_seq = 0


def send_ball(valid, x_cm_x100=0, y_offset_px=0, quality=0, tracked=False):
    global uart_seq
    if uart is None:
        return
    try:
        frame = build_vision_frame(
            uart_seq, valid, x_cm_x100, y_offset_px, quality, tracked
        )
        uart.write(frame)
        uart_seq = (uart_seq + 1) & 0xFF
    except Exception:
        # The MSPM0 timeout path is responsible for returning the pipe to neutral.
        pass


# ---- Camera ----
def _lock_auto_controls(sensor):
    """Lock whichever exposure controls the installed CanMV firmware provides."""
    for name in ("set_auto_gain", "set_auto_whitebal", "set_auto_exposure"):
        try:
            getattr(sensor, name)(False)
            print("[K230] {} locked".format(name))
        except Exception:
            pass


class HardwareH264:
    """Camera-CH0 to VENC binding, with non-blocking H.264 stream retrieval.

    CH0 never crosses the MicroPython image API: VICAP feeds it directly into
    VENC.  CH1 is deliberately left for the RGB565 vision snapshot.  Keeping
    those paths separate is what prevents video quality from reducing UART
    control frequency.
    """
    def __init__(self):
        self.encoder = None
        self.stream = None
        self.link = None
        self.started = False
        self.header = b""
        self.wait_for_idr = True

    def prepare(self):
        if not _load_venc_module():
            raise RuntimeError("media.vencoder is not available in this firmware")
        self.encoder = venc.Encoder()
        # The VENC manual requires this before MediaManager.init().
        self.encoder.SetOutBufs(H264_VENC_CHN, H264_OUT_BUFS, H264_W, H264_H)

    def bind(self, sensor):
        self.link = MediaManager.link(
            sensor.bind_info(chn=CAM_CHN_ID_0)['src'],
            (venc.VIDEO_ENCODE_MOD_ID, venc.VENC_DEV_ID, H264_VENC_CHN)
        )

    def start(self):
        attr = venc.ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_BASELINE,
            H264_W, H264_H,
            bit_rate=H264_BITRATE_KBPS,
            gopLen=H264_GOP,
            src_frame_rate=30,
            dst_frame_rate=VIDEO_TARGET_FPS,
        )
        self.stream = venc.StreamData()
        self.encoder.Create(H264_VENC_CHN, attr)
        self.encoder.Start(H264_VENC_CHN)
        self.started = True

    def request_clean_decoder_start(self):
        """Next sent frame must include cached headers and be an IDR frame."""
        self.wait_for_idr = True

    def poll(self):
        """Return ``bytes`` for one displayable H.264 frame, or ``None``.

        All VENC buffers are released before returning.  This method must be
        called even when no PC is connected; otherwise the hardware output
        queue fills and the camera pipeline eventually stalls.
        """
        if not self.started:
            return None
        try:
            result = self.encoder.GetStream(H264_VENC_CHN, self.stream, timeout=0)
        except Exception:
            return None
        if result != 0 or self.stream.pack_cnt <= 0:
            return None

        try:
            stream_type = self.stream.stream_type[0]
            encoded = bytearray()
            for pack_index in range(self.stream.pack_cnt):
                size = self.stream.data_size[pack_index]
                if size > 0:
                    encoded.extend(uctypes.bytearray_at(
                        self.stream.data[pack_index], size
                    ))
            encoded = bytes(encoded)

            if stream_type == self.encoder.STREAM_TYPE_HEADER:
                self.header = encoded
                return None

            # A receiver that connected halfway through a GOP cannot decode P
            # frames.  On reconnect wait for the next IDR and prepend SPS/PPS.
            if self.wait_for_idr:
                if stream_type != self.encoder.STREAM_TYPE_I:
                    return None
                self.wait_for_idr = False
                return self.header + encoded
            return encoded
        finally:
            try:
                self.encoder.ReleaseStream(H264_VENC_CHN, self.stream)
            except Exception:
                pass

    def close(self):
        if self.started:
            try:
                self.encoder.Stop(H264_VENC_CHN)
            except Exception:
                pass
            try:
                self.encoder.Destroy(H264_VENC_CHN)
            except Exception:
                pass
            self.started = False
        try:
            self.link = None
        except Exception:
            pass


def init_camera(video_backend):
    sensor = K230Sensor(id=SENSOR_ID, width=CAPTURE_W, height=CAPTURE_H, fps=30)
    sensor.reset()
    h264 = None

    if video_backend == "h264_hw":
        # CH0 has a hardware-native format and is bound directly to VENC.  CH1
        # remains RGB565 because find_blobs()/drawing are image-module APIs.
        # Yahboom CanMV v1.4.3 has a broken ``chn=CAM_CHN_ID_0`` code path
        # (``buf_init`` is missing).  Configure the primary channel via the
        # default API; CH1 can still be named explicitly below.
        sensor.set_pixformat(Sensor.YUV420SP)
        sensor.set_framesize(width=H264_W, height=H264_H)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
        sensor.set_framesize(width=CAPTURE_W, height=CAPTURE_H,
                             chn=CAM_CHN_ID_1)
        h264 = HardwareH264()
        h264.prepare()
        h264.bind(sensor)
    else:
        sensor.set_pixformat(Sensor.RGB565)
        try:
            sensor.set_framesize(width=CAPTURE_W, height=CAPTURE_H)
        except Exception:
            try:
                sensor.set_framesize(Sensor.VGA)
            except Exception:
                pass

    # CanMV v1.4.3 needs the virtual-display pipeline initialized even with
    # ``to_ide=False``.  Omitting it made snapshot() jump from ~3 ms to ~20 ms
    # in the user's benchmark.  VIRT allocates no visible external display.
    preview_chn = CAM_CHN_ID_1 if video_backend == "h264_hw" else CAM_CHN_ID_0
    Display.init(Display.VIRT, sensor.width(chn=preview_chn),
                 sensor.height(chn=preview_chn), to_ide=IDE_PREVIEW)
    MediaManager.init()
    if h264 is not None:
        h264.start()
    sensor.run()
    time.sleep_ms(500)
    for _ in range(8):
        try:
            if video_backend == "h264_hw":
                sensor.snapshot(chn=CAM_CHN_ID_1)
            else:
                sensor.snapshot()  # Let automatic exposure settle before locking it.
        except Exception:
            time.sleep_ms(100)
    _lock_auto_controls(sensor)
    if video_backend == "h264_hw":
        print("[K230] Camera VENC:{}x{} + vision:{}x{} OK".format(
            sensor.width(), sensor.height(),
            sensor.width(chn=CAM_CHN_ID_1), sensor.height(chn=CAM_CHN_ID_1)
        ))
    else:
        print("[K230] Camera {}x{} OK".format(sensor.width(), sensor.height()))
    return sensor, h264


# ---- Wi-Fi/video: deliberately decoupled from control. ----
def wifi_start():
    try:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        return wlan
    except Exception as error:
        print("[K230] WiFi unavailable:", error)
        return None


def wifi_ready(wlan, now_ms, last_attempt_ms):
    if wlan is None:
        return False, last_attempt_ms
    try:
        if wlan.isconnected():
            return True, last_attempt_ms
    except Exception:
        return False, last_attempt_ms

    if _ticks_diff(now_ms, last_attempt_ms) >= WIFI_RETRY_MS:
        try:
            wlan.connect(WIFI_SSID, WIFI_PASS)
            print("[K230] WiFi connection requested")
        except Exception:
            pass
        last_attempt_ms = now_ms
    return False, last_attempt_ms


def pc_connect_once():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.settimeout(VIDEO_CONNECT_TIMEOUT_S)
        except Exception:
            pass
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
        sock.connect((PC_IP, PC_PORT))
        try:
            sock.setblocking(False)
        except Exception:
            try:
                sock.settimeout(0)
            except Exception:
                pass
        print("[K230] PC video receiver connected")
        return sock
    except Exception:
        try:
            sock.close()
        except Exception:
            pass
        return None


class VideoSender:
    """Send at most one encoded access unit without waiting for Wi-Fi buffers.

    TCP preserves packet order, so a partially sent frame can safely continue on
    the next main-loop iteration.  While it is pending, newer camera frames are
    deliberately dropped: live video is preferable to a growing backlog.
    """
    def __init__(self):
        self.packet = None
        self.payload_size = 0
        self.offset = 0
        self.blocked_since_ms = None

    def pending(self):
        return self.packet is not None

    def reset(self):
        self.packet = None
        self.payload_size = 0
        self.offset = 0
        self.blocked_since_ms = None

    def enqueue_payload(self, codec, payload):
        """Queue a complete encoded payload; never retain more than one."""
        if self.packet is not None:
            return False
        self.payload_size = len(payload)
        self.packet = (STREAM_MAGIC + bytes((STREAM_VERSION, codec)) +
                       self.payload_size.to_bytes(4, "big") + payload)
        self.offset = 0
        self.blocked_since_ms = None
        return True

    def flush(self, sock, now_ms):
        """Return (completed_jpeg_bytes, connection_stalled)."""
        if self.packet is None:
            return 0, False
        try:
            sent = sock.send(self.packet[self.offset:])
            if sent is None or sent <= 0:
                raise OSError("socket would block")
        except Exception:
            if self.blocked_since_ms is None:
                self.blocked_since_ms = now_ms
            stalled = _ticks_diff(now_ms, self.blocked_since_ms) >= VIDEO_STALL_TIMEOUT_MS
            return 0, stalled

        self.blocked_since_ms = None
        self.offset += sent
        if self.offset < len(self.packet):
            return 0, False

        completed_size = self.payload_size
        self.reset()
        return completed_size, False


def make_video_canvas():
    """Allocate the reusable JPEG canvas without touching the vision canvas."""
    if (VIDEO_SOURCE_MODE == "vision" and
            VIDEO_W == VISION_W and VIDEO_H == VISION_H):
        print("[K230] Video direct {}x{} Q{} @{}fps".format(
            VIDEO_W, VIDEO_H, JPEG_Q, VIDEO_TARGET_FPS
        ))
        return None
    try:
        canvas = image.Image(VIDEO_W, VIDEO_H, image.RGB565)
        if VIDEO_SOURCE_MODE == "capture_pipe":
            print("[K230] Video pipe ROI {} -> {}x{} Q{} @{}fps".format(
                PIPE_VIDEO_CROP, VIDEO_W, VIDEO_H, JPEG_Q, VIDEO_TARGET_FPS
            ))
        else:
            print("[K230] Video canvas {}x{} Q{} @{}fps".format(
                VIDEO_W, VIDEO_H, JPEG_Q, VIDEO_TARGET_FPS
            ))
        return canvas
    except Exception as error:
        print("[K230] Video canvas unavailable; using source JPEG:", error)
        return None


def encode_video_jpeg(capture_img, vision_img, canvas):
    """Encode display video while keeping the 320x240 vision path independent."""
    if VIDEO_SOURCE_MODE == "capture_pipe":
        crop_x, crop_y, crop_w, crop_h = PIPE_VIDEO_CROP
        if canvas is not None:
            try:
                # Drawing the full source at a negative offset clips exactly
                # to the canvas, avoiding a per-frame crop-image allocation.
                canvas.draw_image(capture_img, -crop_x, -crop_y)
                return bytes(canvas.compress(quality=JPEG_Q))
            except Exception as error:
                print("[K230] Pipe ROI canvas failed; trying copy(roi):", error)
        # ``copy(roi=...)`` is slower because it allocates a temporary image,
        # but retains the original 640-pixel pipe detail on firmwares that do
        # not clip draw_image() with negative coordinates.
        try:
            pipe_img = capture_img.copy(roi=(crop_x, crop_y, crop_w, crop_h))
            return bytes(pipe_img.compress(quality=JPEG_Q))
        except Exception as error:
            print("[K230] Pipe ROI copy failed; using vision JPEG:", error)
        # Last-resort stream continuity; UART/vision must remain independent.
        return bytes(vision_img.compress(quality=JPEG_Q))

    if canvas is not None:
        try:
            canvas.draw_image(vision_img, 0, 0,
                              x_scale=VIDEO_SCALE_X,
                              y_scale=VIDEO_SCALE_Y)
            return bytes(canvas.compress(quality=JPEG_Q))
        except Exception as error:
            print("[K230] Vision-video scale failed; using vision JPEG:", error)
    return bytes(vision_img.compress(quality=JPEG_Q))


def make_vision_canvas(sensor_width, sensor_height):
    """Create the QVGA detector frame while retaining the full sensor FOV."""
    if sensor_width == VISION_W and sensor_height == VISION_H:
        return None
    try:
        canvas = image.Image(VISION_W, VISION_H, image.RGB565)
        print("[K230] Vision full-FOV {}x{} -> {}x{}".format(
            sensor_width, sensor_height, VISION_W, VISION_H
        ))
        return canvas
    except Exception as error:
        print("[K230] Vision canvas unavailable; using sensor image:", error)
        return None


# ---- Main loop ----
print("[K230] Initializing vision control...")
active_video_backend = VIDEO_BACKEND
if active_video_backend == "h264_hw" and not _load_venc_module():
    print("[K230] VENC API unavailable; falling back to jpeg_cpu")
    active_video_backend = "jpeg_cpu"

try:
    sensor, h264 = init_camera(active_video_backend)
except Exception as error:
    # Never create Sensor(2) a second time after a partly failed VENC setup:
    # this firmware keeps the CSI device allocated until a soft reboot.  The
    # explicit error is actionable, while jpeg_cpu remains the safe default.
    print("[K230] Camera/video initialization failed:", error)
    if active_video_backend == "h264_hw":
        print("[K230] Set VIDEO_BACKEND = 'jpeg_cpu', soft reboot, then rerun.")
    raise

vision_chn = CAM_CHN_ID_1 if active_video_backend == "h264_hw" else CAM_CHN_ID_0
vision_width = sensor.width(chn=vision_chn)
vision_height = sensor.height(chn=vision_chn)
if vision_width < VISION_W or vision_height < VISION_H:
    raise ValueError("camera frame is smaller than configured vision image")
vision_canvas = make_vision_canvas(vision_width, vision_height)
VISION_INPUT_SCALE_X = VISION_W / float(vision_width)
VISION_INPUT_SCALE_Y = VISION_H / float(vision_height)

tracker = AlphaBetaTracker()
clock = time.clock()
wlan = wifi_start()
pc_sock = None
now_ms = time.ticks_ms()
last_wifi_attempt_ms = now_ms - WIFI_RETRY_MS
last_pc_attempt_ms = now_ms - PC_RETRY_MS
last_video_enqueue_ms = now_ms - VIDEO_INTERVAL_MS
video_stat_start_ms = now_ms
frame_index = 0
video_count = 0
video_bytes = 0
video_sender = VideoSender()
video_canvas = make_video_canvas() if active_video_backend == "jpeg_cpu" else None
perf_capture_ms = 0
perf_vision_ms = 0
perf_jpeg_ms = 0
perf_send_ms = 0

if VISION_ENABLE:
    print("[K230] Running: vision -> UART; {} video is best-effort only".format(
        active_video_backend
    ))
else:
    print("[K230] Running: STREAM BENCHMARK ONLY ({}); UART sends invalid observations".format(
        active_video_backend
    ))
while True:
    clock.tick()
    now_ms = time.ticks_ms()

    # 1. Keep a full-FOV sensor frame, then downscale into the detector canvas.
    # This prevents QVGA mode from narrowing the physical pipe view.
    capture_start_ms = time.ticks_ms()
    frame = sensor.snapshot(chn=vision_chn)
    if vision_canvas is not None:
        vision_canvas.draw_image(frame, 0, 0,
                                 x_scale=VISION_INPUT_SCALE_X,
                                 y_scale=VISION_INPUT_SCALE_Y)
        img = vision_canvas
    else:
        img = frame
    perf_capture_ms += _ticks_diff(time.ticks_ms(), capture_start_ms)

    # 2. Vision can be disabled temporarily to measure the true stream ceiling.
    vision_start_ms = time.ticks_ms()
    if VISION_ENABLE:
        valid, x_cm_x100, y_offset_px, quality, tracked = detect_ball(
            img, tracker, now_ms
        )
    else:
        x_cm_x100 = 0
        y_offset_px = 0
        quality = 0
        valid = False
        tracked = False
    perf_vision_ms += _ticks_diff(time.ticks_ms(), vision_start_ms)

    if OVERLAY_ENABLE:
        img.draw_rectangle(PIPE_ROI, color=(0, 0, 255), thickness=1)
        # The calibrated zero can be fractional; CanMV drawing APIs require
        # integer coordinates, while all position calculations keep the float.
        zero_line_x = int(round(ZERO_X_PX))
        img.draw_line(zero_line_x, PIPE_ROI[1], zero_line_x,
                      PIPE_ROI[1] + PIPE_ROI[3], color=(255, 255, 0), thickness=1)

    # 3. Send an observation every processed frame.  There is no K230 timestamp
    # in the protocol: MSPM0 uses its local receive time for the 200 ms watchdog.
    send_ball(valid, x_cm_x100, y_offset_px, quality, tracked)

    # 4. Periodically attempt video connectivity.  All failures are isolated from
    # the visual control path above.
    wifi_is_ready, last_wifi_attempt_ms = wifi_ready(
        wlan, now_ms, last_wifi_attempt_ms
    )
    if (pc_sock is None and wifi_is_ready and
            _ticks_diff(now_ms, last_pc_attempt_ms) >= PC_RETRY_MS):
        last_pc_attempt_ms = now_ms
        pc_sock = pc_connect_once()
        if pc_sock is not None and h264 is not None:
            h264.request_clean_decoder_start()

    # 5. Keep hardware encoder output drained regardless of network state.  The
    # camera-to-VENC link runs independently from the RGB565 vision snapshot.
    if h264 is not None:
        h264_start_ms = time.ticks_ms()
        h264_payload = h264.poll()
        perf_jpeg_ms += _ticks_diff(time.ticks_ms(), h264_start_ms)
        if (pc_sock is not None and h264_payload is not None and
                not video_sender.pending()):
            try:
                video_sender.enqueue_payload(STREAM_CODEC_H264, h264_payload)
            except Exception:
                video_sender.reset()
    elif (active_video_backend == "jpeg_cpu" and pc_sock is not None and
          not video_sender.pending() and
          _ticks_diff(now_ms, last_video_enqueue_ms) >= VIDEO_INTERVAL_MS):
        try:
            jpeg_start_ms = time.ticks_ms()
            jpeg = encode_video_jpeg(frame, img, video_canvas)
            video_sender.enqueue_payload(STREAM_CODEC_JPEG, jpeg)
            perf_jpeg_ms += _ticks_diff(time.ticks_ms(), jpeg_start_ms)
            last_video_enqueue_ms = now_ms
        except Exception:
            # JPEG compression failure is local to this display frame.
            video_sender.reset()

    if pc_sock is not None:
        send_start_ms = time.ticks_ms()
        completed_bytes, connection_stalled = video_sender.flush(pc_sock, now_ms)
        perf_send_ms += _ticks_diff(time.ticks_ms(), send_start_ms)
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
            if h264 is not None:
                h264.request_clean_decoder_start()

    frame_index += 1
    if frame_index % 30 == 0:
        state = "x={:.2f}cm q={}".format(x_cm_x100 / 100.0, quality) if valid else "NO BALL"
        stat_elapsed_ms = max(1, _ticks_diff(now_ms, video_stat_start_ms))
        video_fps = video_count * 1000.0 / stat_elapsed_ms
        video_kb_s = video_bytes * 1000.0 / stat_elapsed_ms / 1024.0
        if PERFORMANCE_LOG:
            quality_text = ("{} {}kbps".format(active_video_backend, H264_BITRATE_KBPS)
                            if h264 is not None else
                            "{} Q{}".format(active_video_backend, JPEG_Q))
            print("[K230] Loop:{:.1f} {} UART:{} Video:{:.1f}fps {:.1f}KB/s {} "
                  "ms/frame cap:{:.1f} vis:{:.1f} enc:{:.1f} net:{:.1f}".format(
                clock.fps(), state, uart_seq, video_fps, video_kb_s, quality_text,
                perf_capture_ms / 30.0, perf_vision_ms / 30.0,
                perf_jpeg_ms / 30.0, perf_send_ms / 30.0
            ))
        else:
            print("[K230] Loop:{:.1f} {} UART:{} Video:{:.1f}fps {:.1f}KB/s {}".format(
                clock.fps(), state, uart_seq, video_fps, video_kb_s,
                active_video_backend
            ))
        video_count = 0
        video_bytes = 0
        video_stat_start_ms = now_ms
        perf_capture_ms = 0
        perf_vision_ms = 0
        perf_jpeg_ms = 0
        perf_send_ms = 0
        gc.collect()
