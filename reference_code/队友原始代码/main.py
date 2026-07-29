"""K230 CanMV real-time steel-ball detection.

Copy this file and ``steel_ball_yolov8n_320.kmodel`` to
``/sdcard/steel_ball/``. Run this file from CanMV IDE. To auto-start, copy
the same code to ``/sdcard/main.py`` without changing MODEL_PATH.
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from libs.Utils import ScopedTiming
import gc
import os
import sys

try:
    import time
except Exception:
    time = None

try:
    from machine import UART, FPIOA
except Exception:
    UART = None
    FPIOA = None

try:
    from ybUtils.YbUart import YbUart
except Exception:
    YbUart = None

try:
    sys.path.append("/sdcard/app")
    sys.path.append("/sdcard/steel_ball")
except Exception:
    pass

try:
    from k230_mspm0_uart_protocol import Mspm0UartProtocol
except Exception:
    PROTO_HEAD_0 = 0xAA
    PROTO_HEAD_1 = 0x55
    MSG_VISION_TARGET = 0x01
    VISION_FLAG_FOUND = 0x01

    def _clamp_int16(value):
        value = int(value)
        if value < -32768:
            return -32768
        if value > 32767:
            return 32767
        return value

    def _put_i16_le(buf, offset, value):
        value = _clamp_int16(value) & 0xFFFF
        buf[offset] = value & 0xFF
        buf[offset + 1] = (value >> 8) & 0xFF

    def _checksum(frame, start, end):
        value = 0
        for index in range(start, end):
            value = (value + frame[index]) & 0xFF
        return value

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
            try:
                return self.uart.deinit()
            except Exception:
                return None

        def send_vision(self, found, err_x, err_y, quality=0):
            payload = bytearray(6)
            payload[0] = VISION_FLAG_FOUND if found else 0
            _put_i16_le(payload, 1, err_x)
            _put_i16_le(payload, 3, err_y)
            if quality < 0:
                quality = 0
            elif quality > 255:
                quality = 255
            payload[5] = int(quality)

            length = len(payload) + 2
            frame = bytearray(2 + 1 + length + 1)
            frame[0] = PROTO_HEAD_0
            frame[1] = PROTO_HEAD_1
            frame[2] = length
            frame[3] = MSG_VISION_TARGET
            frame[4] = self.seq
            for index in range(len(payload)):
                frame[5 + index] = payload[index]
            frame[len(frame) - 1] = _checksum(frame, 2, len(frame) - 1)

            self.seq = (self.seq + 1) & 0xFF
            return self._write(frame)


MODEL_PATHS = [
    "/sdcard/steel_ball/steel_ball_yolov8n_320.kmodel",
    "/sdcard/app/steel_ball_yolov8n_320.kmodel",
    "/sdcard/steel_ball_yolov8n_320.kmodel",
]
LABELS = ["steel_ball"]
MODEL_INPUT_SIZE = [320, 320]

# SENSOR_ID is used by CanMV v1.8+. Older firmware such as v1.4.3 ignores it.
SENSOR_ID = 2
DISPLAY_MODE = "lcd"
DISPLAY_SIZE = [640, 480]
RGB888P_SIZE = [640, 360]
H_MIRROR = False
V_FLIP = False

CONFIDENCE_THRESHOLD = 0.22
FILTER_CONFIDENCE_THRESHOLD = 0.35
TOUCHING_CONFIDENCE_THRESHOLD = 0.22
MIN_BOX_AREA_RATIO = 0.004
MAX_BOX_AREA_RATIO = 0.18
MIN_ASPECT_RATIO = 0.55
MAX_ASPECT_RATIO = 1.80
TOUCHING_DISTANCE_RATIO = 1.35
NMS_THRESHOLD = 0.45
MAX_BOXES = 50
PRINT_EVERY_N_FRAMES = 10

UART_ENABLE = True
UART_BAUDRATE = 115200
UART_TX_PIN = 9
UART_RX_PIN = 10
UART_SEND_PERIOD_MS = 80


def find_model_path():
    for path in MODEL_PATHS:
        try:
            os.stat(path)
            print("[steel_ball] model:", path)
            return path
        except OSError:
            pass
    print("[steel_ball] kmodel not found, tried:")
    for path in MODEL_PATHS:
        print("  ", path)
    raise OSError("Kmodel file not exist.")


def create_pipeline(pipeline):
    try:
        pipeline.create(
            sensor_id=SENSOR_ID,
            hmirror=H_MIRROR,
            vflip=V_FLIP,
        )
        return
    except TypeError as error:
        if "sensor_id" not in str(error):
            raise
        print("[steel_ball] PipeLine.create has no sensor_id; using old API")

    try:
        pipeline.create(
            hmirror=H_MIRROR,
            vflip=V_FLIP,
        )
        return
    except TypeError as error:
        if "hmirror" not in str(error) and "vflip" not in str(error):
            raise
        print("[steel_ball] PipeLine.create has no mirror/flip args; using defaults")

    pipeline.create()


def enable_uart_pin(fpioa, pin, func, is_tx):
    try:
        fpioa.set_function(pin, func, ie=0 if is_tx else 1, oe=1 if is_tx else 0)
        return
    except Exception:
        pass

    fpioa.set_function(pin, func)
    try:
        fpioa.set_function(pin, set_ie=0 if is_tx else 1, set_oe=1 if is_tx else 0)
    except Exception:
        pass


def init_uart():
    if not UART_ENABLE:
        return None

    if UART is not None and FPIOA is not None:
        try:
            fpioa = FPIOA()
            enable_uart_pin(fpioa, UART_TX_PIN, fpioa.UART1_TXD, True)
            enable_uart_pin(fpioa, UART_RX_PIN, fpioa.UART1_RXD, False)
            uart = UART(
                UART.UART1,
                baudrate=UART_BAUDRATE,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE,
            )
            print(
                "[steel_ball] UART1 ready: IO%d=TX, IO%d=RX, baud=%d"
                % (UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE)
            )
            return uart
        except Exception as error:
            print("[steel_ball] UART1 init failed:", error)

    if YbUart is not None:
        try:
            print("[steel_ball] YbUart fallback ready: baud=%d" % UART_BAUDRATE)
            return YbUart(baudrate=UART_BAUDRATE)
        except Exception as error:
            print("[steel_ball] YbUart init failed:", error)

    print("[steel_ball] UART init failed")
    return None


def is_reasonable_ball_shape(box, display_size):
    x, y, width, height = box
    if width <= 0 or height <= 0:
        return False

    aspect = width / height
    display_area = display_size[0] * display_size[1]
    area_ratio = width * height / display_area

    if aspect < MIN_ASPECT_RATIO or aspect > MAX_ASPECT_RATIO:
        return False
    if area_ratio < MIN_BOX_AREA_RATIO or area_ratio > MAX_BOX_AREA_RATIO:
        return False
    return True


def is_near_box(box, other_box):
    x, y, width, height = box
    other_x, other_y, other_width, other_height = other_box
    center_x = x + width / 2
    center_y = y + height / 2
    other_center_x = other_x + other_width / 2
    other_center_y = other_y + other_height / 2
    dx = center_x - other_center_x
    dy = center_y - other_center_y
    distance_sq = dx * dx + dy * dy
    near_distance = max(width, height, other_width, other_height) * TOUCHING_DISTANCE_RATIO
    return distance_sq <= near_distance * near_distance


def filter_result(result, display_size):
    if not result or len(result[0]) == 0:
        return result

    candidates = []
    strong_boxes = []
    for index in range(len(result[0])):
        box = result[0][index]
        score = float(result[2][index])
        if not is_reasonable_ball_shape(box, display_size):
            continue
        candidates.append((index, box, score))
        if score >= FILTER_CONFIDENCE_THRESHOLD:
            strong_boxes.append(box)

    boxes = []
    class_ids = []
    scores = []
    for index, box, score in candidates:
        keep = score >= FILTER_CONFIDENCE_THRESHOLD
        if not keep and score >= TOUCHING_CONFIDENCE_THRESHOLD:
            for strong_box in strong_boxes:
                if is_near_box(box, strong_box):
                    keep = True
                    break

        if keep:
            boxes.append(box)
            class_ids.append(result[1][index])
            scores.append(result[2][index])

    return [boxes, class_ids, scores]


def get_best_target(result):
    count = 0
    best_score = -1.0
    best_center = None
    best_box = None

    if result and len(result[0]) > 0:
        count = len(result[0])
        for index in range(count):
            x, y, width, height = result[0][index]
            score = float(result[2][index])
            if score > best_score:
                best_score = score
                best_center = (
                    int(round(x + width / 2)),
                    int(round(y + height / 2)),
                )
                best_box = (x, y, width, height)

    return count, best_center, best_score, best_box


def add_status_overlay(result, osd_image, frame_index):
    count, best_center, best_score, _ = get_best_target(result)

    osd_image.draw_string_advanced(
        5, 5, 24, "steel_ball: %d" % count, color=(0, 255, 0)
    )

    if best_center is not None:
        center_x, center_y = best_center
        osd_image.draw_cross(
            center_x,
            center_y,
            color=(255, 255, 0),
            size=12,
            thickness=3,
        )
        osd_image.draw_string_advanced(
            5,
            34,
            20,
            "best center: %d,%d" % (center_x, center_y),
            color=(255, 255, 0),
        )

    if frame_index % PRINT_EVERY_N_FRAMES == 0:
        if best_center is None:
            print("[steel_ball] count=0")
        else:
            print(
                "[steel_ball] count=%d best_center=(%d,%d) score=%.3f"
                % (count, best_center[0], best_center[1], best_score)
            )


def send_vision_frame(protocol, found, err_x, err_y, quality=0):
    if protocol is None:
        return
    try:
        protocol.send_vision(found, err_x, err_y, quality)
    except Exception as error:
        print("[steel_ball] UART send failed:", error)


def sleep_ms(ms):
    if time is not None and hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    elif time is not None:
        time.sleep(ms / 1000.0)


def ticks_ms():
    if time is not None and hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    if time is not None:
        return int(time.time() * 1000)
    return 0


def ticks_diff(now, then):
    if time is not None and hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, then)
    return now - then


def send_startup_uart_burst(protocol):
    if protocol is None:
        return

    print("[steel_ball] TX startup heartbeat")
    for index in range(20):
        send_vision_frame(protocol, False, 0, 0, index & 0xFF)
        sleep_ms(200)


def main():
    pipeline = None
    detector = None
    uart = None
    try:
        model_path = find_model_path()
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode=DISPLAY_MODE,
            display_size=DISPLAY_SIZE,
        )
        create_pipeline(pipeline)
        display_size = pipeline.get_display_size()
        image_center = (display_size[0] // 2, display_size[1] // 2)
        uart = init_uart()
        if uart is not None:
            uart = Mspm0UartProtocol(uart)
            print("[steel_ball] MSPM0 protocol ready")
            send_startup_uart_burst(uart)

        detector = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=model_path,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=display_size,
            conf_thresh=CONFIDENCE_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES,
            debug_mode=0,
        )
        detector.config_preprocess()

        frame_index = 0
        last_uart_send_ms = ticks_ms() - UART_SEND_PERIOD_MS
        while True:
            with ScopedTiming("total", 1):
                frame = pipeline.get_frame()
                result = detector.run(frame)
                result = filter_result(result, display_size)
                detector.draw_result(result, pipeline.osd_img)
                add_status_overlay(result, pipeline.osd_img, frame_index)

                count, best_center, best_score, _ = get_best_target(result)
                found = best_center is not None
                err_x = 0
                err_y = 0
                quality = 0
                if best_center is None:
                    pass
                else:
                    err_x = best_center[0] - image_center[0]
                    err_y = best_center[1] - image_center[1]
                    quality = int(best_score * 100)
                    if quality < 0:
                        quality = 0
                    elif quality > 100:
                        quality = 100

                now = ticks_ms()
                if ticks_diff(now, last_uart_send_ms) >= UART_SEND_PERIOD_MS:
                    send_vision_frame(uart, found, err_x, err_y, quality)
                    last_uart_send_ms = now

                pipeline.show_image()
                frame_index += 1
                gc.collect()
    except KeyboardInterrupt:
        print("[steel_ball] stopped by user")
    except BaseException as error:
        print("[steel_ball] error:", error)
        raise
    finally:
        if uart is not None:
            try:
                uart.deinit()
            except BaseException as cleanup_error:
                print("[steel_ball] uart cleanup error:", cleanup_error)
        if detector is not None:
            try:
                detector.deinit()
            except BaseException as cleanup_error:
                print("[steel_ball] detector cleanup error:", cleanup_error)
        if pipeline is not None and getattr(pipeline, "sensor", None) is not None:
            try:
                pipeline.destroy()
            except BaseException as cleanup_error:
                print("[steel_ball] pipeline cleanup error:", cleanup_error)
        gc.collect()


main()
