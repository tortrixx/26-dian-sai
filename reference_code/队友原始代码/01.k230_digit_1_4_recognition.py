from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aicube
import time
import gc
import os

try:
    from libs.YbProtocol import YbProtocol
    from ybUtils.YbUart import YbUart
except Exception:
    YbProtocol = None
    YbUart = None


DISPLAY_MODE = "lcd"
RGB888P_SIZE = [640, 480]
DISPLAY_SIZE = [640, 480]

OCR_DET_KMODEL = "/sdcard/kmodel/ocr_det_int16.kmodel"
OCR_REC_KMODEL = "/sdcard/kmodel/ocr_rec_int16.kmodel"
OCR_DICT_PATH = "/sdcard/utils/dict.txt"

OCR_DET_INPUT_SIZE = [640, 640]
OCR_REC_INPUT_SIZE = [512, 32]
MASK_THRESHOLD = 0.25
BOX_THRESHOLD = 0.40

VALID_DIGITS = ("1", "2", "3", "4")
STABLE_FRAMES = 2
LOST_CLEAR_FRAMES = 10
DETECT_EVERY = 2
SEND_INTERVAL_MS = 100
GC_INTERVAL_MS = 1000

# 只在画面中央寻找目标编号，避免把背景文字、屏幕边缘说明也送去 OCR。
TARGET_ROI = (80, 40, 480, 360)
MAX_RECOGNIZE_BOXES = 3
MIN_DIGIT_BOX_AREA = 350
MAX_DIGIT_BOX_AREA = 90000
MIN_DIGIT_ASPECT = 0.12
MAX_DIGIT_ASPECT = 1.80
DRAW_ALL_CANDIDATE_BOXES = False

DIGIT_ALIAS = {
    "１": "1",
    "２": "2",
    "３": "3",
    "４": "4",
    "一": "1",
    "二": "2",
    "三": "3",
    "四": "4",
    "I": "1",
    "l": "1",
    "|": "1",
}


class OCRDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, mask_threshold=0.5,
                 box_threshold=0.2, rgb888p_size=[224, 224],
                 display_size=[640, 480], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.mask_threshold = mask_threshold
        self.box_threshold = box_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("ocr det preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right = self.get_padding_param()
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [0, 0, 0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )

    def postprocess(self, results):
        with ScopedTiming("ocr det postprocess", self.debug_mode > 0):
            hwc_array = self.chw2hwc(self.cur_img)
            det_boxes = aicube.ocr_post_process(
                results[0][:, :, :, 0].reshape(-1),
                hwc_array.reshape(-1),
                self.model_input_size,
                self.rgb888p_size,
                self.mask_threshold,
                self.box_threshold,
            )
            return det_boxes

    def get_padding_param(self):
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        input_width = self.rgb888p_size[0]
        input_height = self.rgb888p_size[1]
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_height
        ratio = ratio_w if ratio_w < ratio_h else ratio_h
        new_w = int(ratio * input_width)
        new_h = int(ratio * input_height)
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = int(round(0))
        bottom = int(round(dh * 2 + 0.1))
        left = int(round(0))
        right = int(round(dw * 2 - 0.1))
        return top, bottom, left, right

    def chw2hwc(self, features):
        ori_shape = (features.shape[0], features.shape[1], features.shape[2])
        c_hw = features.reshape((ori_shape[0], ori_shape[1] * ori_shape[2]))
        hw_c = c_hw.transpose()
        new_array = hw_c.copy()
        hwc_array = new_array.reshape((ori_shape[1], ori_shape[2], ori_shape[0]))
        del c_hw
        del hw_c
        del new_array
        return hwc_array


class OCRRecognitionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, dict_path,
                 rgb888p_size=[640, 480], display_size=[640, 480], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.dict_path = dict_path
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.dict_word = None
        self.read_dict()
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.RGB_packed,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self, input_image_size=None, input_np=None):
        with ScopedTiming("ocr rec preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right = self.get_padding_param(ai2d_input_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [0, 0, 0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build(
                [input_np.shape[0], input_np.shape[1], input_np.shape[2], input_np.shape[3]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )

    def postprocess(self, results):
        with ScopedTiming("ocr rec postprocess", self.debug_mode > 0):
            preds = np.argmax(results[0], axis=2).reshape((-1))
            output_txt = ""
            for i in range(len(preds)):
                if preds[i] != (len(self.dict_word) - 1) and not (i > 0 and preds[i - 1] == preds[i]):
                    output_txt = output_txt + self.dict_word[preds[i]]
            return output_txt

    def get_padding_param(self, src_size, dst_size):
        dst_w = dst_size[0]
        dst_h = dst_size[1]
        input_width = src_size[0]
        input_height = src_size[1]
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_height
        ratio = ratio_w if ratio_w < ratio_h else ratio_h
        new_w = int(ratio * input_width)
        new_h = int(ratio * input_height)
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = int(round(0))
        bottom = int(round(dh * 2 + 0.1))
        left = int(round(0))
        right = int(round(dw * 2 - 0.1))
        return top, bottom, left, right

    def read_dict(self):
        with open(self.dict_path, "r") as f:
            raw = f.read(100000)
        words = raw.split("\r\n")
        self.dict_word = {}
        for i, char in enumerate(words):
            self.dict_word[i] = char.replace("\r", "").replace("\n", "")


class DigitRecognizer:
    def __init__(self):
        self.ocr_det = OCRDetectionApp(
            OCR_DET_KMODEL,
            model_input_size=OCR_DET_INPUT_SIZE,
            mask_threshold=MASK_THRESHOLD,
            box_threshold=BOX_THRESHOLD,
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            debug_mode=0,
        )
        self.ocr_rec = OCRRecognitionApp(
            OCR_REC_KMODEL,
            model_input_size=OCR_REC_INPUT_SIZE,
            dict_path=OCR_DICT_PATH,
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            debug_mode=0,
        )
        self.ocr_det.config_preprocess()
        self.last_candidate = None
        self.candidate_count = 0
        self.stable_digit = None
        self.lost_count = 0
        self.last_sent_digit = None
        self.last_send_ms = 0
        self.last_boxes = []
        self.last_texts = []
        self.live_candidate = None
        self.live_box = None
        self.uart = None
        self.pto = None
        if YbUart is not None:
            self.uart = YbUart(baudrate=115200)
        if YbProtocol is not None:
            self.pto = YbProtocol()

    def deinit(self):
        self.ocr_det.deinit()
        self.ocr_rec.deinit()
        if self.uart is not None:
            try:
                self.uart.deinit()
            except Exception:
                pass

    def run_ocr(self, input_np):
        det_res = self.ocr_det.run(input_np)
        det_res = self.select_detection_candidates(det_res)
        boxes = []
        texts = []
        for det in det_res:
            crop = det[0]
            box = det[1]
            self.ocr_rec.config_preprocess(
                input_image_size=[crop.shape[2], crop.shape[1]],
                input_np=crop,
            )
            text = self.ocr_rec.run(crop)
            boxes.append(box)
            texts.append(text)
        self.last_boxes = boxes
        self.last_texts = texts
        return boxes, texts

    def normalize_digit_char(self, ch):
        if ch in VALID_DIGITS:
            return ch
        if ch in DIGIT_ALIAS:
            return DIGIT_ALIAS[ch]
        return None

    def extract_valid_digit(self, text):
        if text is None:
            return None
        digits = []
        visible_count = 0
        for ch in text:
            if ch not in (" ", "\r", "\n", "\t"):
                visible_count += 1
            mapped = self.normalize_digit_char(ch)
            if mapped in VALID_DIGITS:
                digits.append(mapped)
        if len(digits) == 1 and visible_count <= 3:
            return digits[0]
        return None

    def box_rect(self, box):
        xs = [box[0], box[2], box[4], box[6]]
        ys = [box[1], box[3], box[5], box[7]]
        x0 = min(xs)
        y0 = min(ys)
        x1 = max(xs)
        y1 = max(ys)
        return x0, y0, x1, y1

    def box_area(self, box):
        x0, y0, x1, y1 = self.box_rect(box)
        return (x1 - x0) * (y1 - y0)

    def box_score(self, box):
        x0, y0, x1, y1 = self.box_rect(box)
        w = x1 - x0
        h = y1 - y0
        if w <= 0 or h <= 0:
            return -1

        area = w * h
        if area < MIN_DIGIT_BOX_AREA or area > MAX_DIGIT_BOX_AREA:
            return -1

        aspect = w / h
        if aspect < MIN_DIGIT_ASPECT or aspect > MAX_DIGIT_ASPECT:
            return -1

        roi_x, roi_y, roi_w, roi_h = TARGET_ROI
        cx = (x0 + x1) // 2
        cy = (y0 + y1) // 2
        if cx < roi_x or cx > roi_x + roi_w or cy < roi_y or cy > roi_y + roi_h:
            return -1

        center_x = roi_x + roi_w // 2
        center_y = roi_y + roi_h // 2
        center_penalty = abs(cx - center_x) + abs(cy - center_y)
        return area - center_penalty * 8

    def select_detection_candidates(self, det_res):
        if not det_res:
            return []
        scored = []
        for det in det_res:
            score = self.box_score(det[1])
            if score >= 0:
                scored.append((score, det))
        scored.sort(key=lambda item: item[0], reverse=True)
        return [item[1] for item in scored[:MAX_RECOGNIZE_BOXES]]

    def choose_candidate(self, boxes, texts):
        best_digit = None
        best_box = None
        best_score = -1
        for i in range(len(texts)):
            digit = self.extract_valid_digit(texts[i])
            if digit is None:
                continue
            score = self.box_score(boxes[i])
            if score > best_score:
                best_score = score
                best_digit = digit
                best_box = boxes[i]
        return best_digit, best_box

    def set_live_result(self, candidate, best_box):
        self.live_candidate = candidate
        self.live_box = best_box

    def update_stable_digit(self, candidate):
        if candidate is None:
            self.lost_count += 1
            if self.lost_count >= LOST_CLEAR_FRAMES:
                self.last_candidate = None
                self.candidate_count = 0
                self.stable_digit = None
            return self.stable_digit

        self.lost_count = 0
        if candidate == self.last_candidate:
            self.candidate_count += 1
        else:
            self.last_candidate = candidate
            self.candidate_count = 1

        if self.candidate_count >= STABLE_FRAMES:
            self.stable_digit = candidate
        return self.stable_digit

    def send_digit(self, digit):
        if digit is None or self.uart is None:
            return
        now = time.ticks_ms()
        should_send = digit != self.last_sent_digit
        if time.ticks_diff(now, self.last_send_ms) >= SEND_INTERVAL_MS:
            should_send = True
        if not should_send:
            return

        if self.pto is not None:
            data = self.pto.get_ocr_rec_data(str(digit))
        else:
            data = "$DIGIT,%s#" % str(digit)
        self.uart.send(data)
        self.last_sent_digit = digit
        self.last_send_ms = now
        print("digit:", digit)

    def draw_box(self, osd_img, box, color=(255, 0, 0, 255)):
        for i in range(4):
            x1 = int(box[(i * 2)] / RGB888P_SIZE[0] * DISPLAY_SIZE[0])
            y1 = int(box[(i * 2 + 1)] / RGB888P_SIZE[1] * DISPLAY_SIZE[1])
            x2 = int(box[((i + 1) * 2) % 8] / RGB888P_SIZE[0] * DISPLAY_SIZE[0])
            y2 = int(box[((i + 1) * 2 + 1) % 8] / RGB888P_SIZE[1] * DISPLAY_SIZE[1])
            osd_img.draw_line((x1, y1, x2, y2), color=color, thickness=4)

    def draw_result(self, pl):
        pl.osd_img.clear()
        roi_x, roi_y, roi_w, roi_h = TARGET_ROI
        pl.osd_img.draw_rectangle(roi_x, roi_y, roi_w, roi_h, color=(0, 120, 255, 180), thickness=2)

        if DRAW_ALL_CANDIDATE_BOXES:
            for i in range(len(self.last_boxes)):
                self.draw_box(pl.osd_img, self.last_boxes[i], color=(255, 255, 0, 255))

        if self.live_box is not None:
            self.draw_box(pl.osd_img, self.live_box, color=(0, 255, 0, 255))

        if self.live_candidate is not None:
            pl.osd_img.draw_string_advanced(20, 20, 32, "candidate: " + str(self.live_candidate), color=(0, 255, 255))
        else:
            pl.osd_img.draw_string_advanced(20, 20, 32, "candidate: none", color=(255, 180, 0))

        if self.stable_digit is not None:
            pl.osd_img.draw_string_advanced(20, 70, 56, "TARGET: " + str(self.stable_digit), color=(0, 255, 0))
        else:
            pl.osd_img.draw_string_advanced(20, 70, 40, "TARGET: --", color=(255, 255, 255))


def main():
    pl = None
    recognizer = None
    frame_count = 0
    last_gc_ms = time.ticks_ms()

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        pl = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode=DISPLAY_MODE,
        )
        pl.create()
        recognizer = DigitRecognizer()

        while True:
            os.exitpoint()
            frame_count += 1
            img = pl.get_frame()

            candidate = None
            best_box = None
            if frame_count % DETECT_EVERY == 0:
                with ScopedTiming("digit total", 0):
                    boxes, texts = recognizer.run_ocr(img)
                    candidate, best_box = recognizer.choose_candidate(boxes, texts)
                    recognizer.set_live_result(candidate, best_box)
                    recognizer.update_stable_digit(candidate)
                    recognizer.send_digit(recognizer.stable_digit)

            recognizer.draw_result(pl)
            pl.show_image()

            now = time.ticks_ms()
            if time.ticks_diff(now, last_gc_ms) >= GC_INTERVAL_MS:
                last_gc_ms = now
                gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except Exception as e:
        print("digit recognition error:", e)
    finally:
        if recognizer is not None:
            recognizer.deinit()
        if pl is not None:
            pl.destroy()


if __name__ == "__main__":
    main()
