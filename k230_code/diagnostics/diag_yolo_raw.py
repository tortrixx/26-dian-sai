"""Diagnose YOLO raw output — ulab-numpy compatible version."""
from media.sensor import *
from media.display import *
from media.media import *
import os, time, gc, sys

sys.path.insert(0, '/sdcard/app')

print("[D2] Cleanup...")
try: Display.deinit()
except: pass
try: MediaManager.deinit()
except: pass

print("[D2] Init sensor...")
sensor = Sensor(id=2, width=640, height=480, fps=60)
sensor.reset()
sensor.set_pixformat(Sensor.RGB565)
try: sensor.set_framesize(width=640, height=480)
except:
    try: sensor.set_framesize(Sensor.VGA)
    except: pass
sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
try: sensor.set_framesize(w=640, h=480, chn=CAM_CHN_ID_2)
except Exception as e:
    print("[D2] CHN_2 err:", e)
print("[D2] CHN_2: {}x{}".format(sensor.width(chn=CAM_CHN_ID_2), sensor.height(chn=CAM_CHN_ID_2)))

Display.init(Display.VIRT, 640, 480, to_ide=True)
MediaManager.init()

print("[D2] Loading model...")
from libs.YOLO import YOLO11
COCO80 = ["person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","couch","potted plant","bed","dining table","toilet",
    "tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors",
    "teddy bear","hair drier","toothbrush"]

yolo = YOLO11(
    task_type="detect", mode="video",
    kmodel_path="/sdcard/kmodel/yolo11n_det_320.kmodel",
    labels=COCO80,
    rgb888p_size=[640, 480],
    model_input_size=[320, 320],
    display_size=[640, 480],
    conf_thresh=0.4, nms_thresh=0.45, max_boxes_num=10, debug_mode=0
)
yolo.config_preprocess()
print("[D2] Model ready, starting sensor...")
sensor.run()
time.sleep_ms(800)
for _ in range(8):
    try: sensor.snapshot(chn=CAM_CHN_ID_2)
    except: time.sleep_ms(100)

# Take frame
print("\n[D2] === Frame numpy ===")
img = sensor.snapshot(chn=CAM_CHN_ID_2)
np_img = img.to_numpy_ref()
print("[D2] shape:", np_img.shape)
# Sample pixel values manually
print("[D2] Ch0[0,0:5]:", [np_img[0,0,i] for i in range(5)])
print("[D2] Ch0[240,0:5]:", [np_img[0,240,i] for i in range(5)])
print("[D2] Ch1[0,0:5]:", [np_img[1,0,i] for i in range(5)])
print("[D2] Ch2[0,0:5]:", [np_img[2,0,i] for i in range(5)])
# Check if all same value
v00 = np_img[0,0,0]
same_r = True
for i in range(0,640,16):
    if np_img[0,0,i] != v00:
        same_r = False
        break
print("[D2] Ch0 row0 all same as [0,0]?:", same_r, "value:", v00)
# Check variance in middle
vals = [np_img[0,240,i] for i in range(0,640,32)]
print("[D2] Ch0 row240 samples:", vals[:10])
# Check channel 1 too
vals1 = [np_img[1,240,i] for i in range(0,640,32)]
print("[D2] Ch1 row240 samples:", vals1[:10])

# Run YOLO
print("\n[D2] === YOLO raw output ===")
import nncase_runtime as nn
import ulab.numpy as np

tensors = yolo.preprocess(np_img)
print("[D2] Preprocess done")

# Get KPU output info
print("[D2] KPU inputs:", yolo.kpu.inputs_size())
print("[D2] KPU outputs:", yolo.kpu.outputs_size())

for i in range(yolo.kpu.inputs_size()):
    yolo.kpu.set_input_tensor(i, tensors[i])

yolo.kpu.run()
print("[D2] KPU ran")

raw = []
for i in range(yolo.kpu.outputs_size()):
    od = yolo.kpu.get_output_tensor(i)
    r = od.to_numpy()
    raw.append(r)
    del od
    print("[D2] Output[{}] shape: {}".format(i, r.shape))

# Try to inspect raw output values
# r[0] is batch dim, r[0,0] is first channel
out0 = raw[0]
print("[D2] Out[0] dims:", len(out0.shape))
for d in range(len(out0.shape)):
    print("[D2]   dim {}: {}".format(d, out0.shape[d]))

# Sample values from output
# First, get a 1D view by indexing deeply
# YOLO output shape is typically (1, 84, N) where N is num predictions
if len(out0.shape) == 3:
    # (C, H, W) format
    c, h, w = out0.shape[0], out0.shape[1], out0.shape[2]
    print("[D2] Out shape CHW: {}x{}x{}".format(c, h, w))
    # Sample first few channels at first spatial location
    for ci in range(min(c, 8)):
        print("[D2]   out[{},0,0] = {}".format(ci, out0[ci, 0, 0]))
    # Sample first spatial location across all channels (first 20)
    ch0_vals = [out0[ci, 0, 0] for ci in range(min(c, 20))]
    print("[D2] out[:,0,0] first 20:", ch0_vals)
    # Check second spatial location
    ch1_vals = [out0[ci, 0, 1] for ci in range(min(c, 20))]
    print("[D2] out[:,0,1] first 20:", ch1_vals)
    # Check a random location
    mid_h, mid_w = h//2, w//2
    mid_vals = [out0[ci, mid_h, mid_w] for ci in range(min(c, 20))]
    print("[D2] out[:,{},{}] first 20:".format(mid_h, mid_w), mid_vals)
elif len(out0.shape) == 4:
    b, c, h, w = out0.shape[0], out0.shape[1], out0.shape[2], out0.shape[3]
    print("[D2] Out shape NCHW: {}x{}x{}x{}".format(b, c, h, w))
    for ci in range(min(c, 8)):
        print("[D2]   out[0,{},0,0] = {}".format(ci, out0[0, ci, 0, 0]))

# Now run proper inference
print("\n[D2] === Normal YOLO run ===")
det_res = yolo.postprocess(raw)
print("[D2] Result type:", type(det_res))
if det_res:
    print("[D2] N boxes:", len(det_res[0]) if det_res[0] else 0)
    if det_res[0]:
        print("[D2] Box 0:", det_res[0][0])
        print("[D2] Class 0:", det_res[1][0])
        print("[D2] Score 0:", det_res[2][0])
else:
    print("[D2] Result is None/empty")

# Try with copied numpy
print("\n[D2] === Copied numpy test ===")
np_copy = np.array(np_img)
det_res2 = yolo.run(np_copy)
print("[D2] Copy result:", type(det_res2))
if det_res2 and det_res2[0]:
    print("[D2] Copy box 0:", det_res2[0][0])
    print("[D2] Copy score 0:", det_res2[2][0])

print("\n[D2] DONE")
sensor.stop()
Display.deinit()
MediaManager.deinit()
