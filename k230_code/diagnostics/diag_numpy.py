"""Diagnose RGBP888 numpy data from CHN_2 — verify pixel values and layout.

This script checks:
1. numpy shape, dtype, strides — is the memory layout what AI2D expects?
2. Per-channel min/max/mean — are pixel values in valid [0,255] range?
3. First few pixels per channel — do they look like real image data?
4. Compare CHN_2 RGBP888 vs CHN_0 RGB565 snapshot
"""
from media.sensor import *
from media.display import *
from media.media import *
import time

print("[DIAG] Cleanup...")
try: Display.deinit()
except: pass
try: MediaManager.deinit()
except: pass
print("[DIAG] Cleanup done")

print("[DIAG] Init sensor...")
sensor = Sensor(id=2, width=640, height=480, fps=30)
sensor.reset()

# CHN_0 for display
sensor.set_pixformat(Sensor.RGB565)
try:
    sensor.set_framesize(width=640, height=480)
except Exception:
    try:
        sensor.set_framesize(Sensor.VGA)
    except Exception:
        pass
print("[DIAG] CHN_0: {}x{}".format(sensor.width(), sensor.height()))

# CHN_2 for AI
sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
try:
    sensor.set_framesize(w=640, h=480, chn=CAM_CHN_ID_2)
except Exception as e:
    print("[DIAG] CHN_2 set_framesize threw:", e)
print("[DIAG] CHN_2: {}x{}".format(
    sensor.width(chn=CAM_CHN_ID_2), sensor.height(chn=CAM_CHN_ID_2)))

# Display
Display.init(Display.VIRT, 640, 480, to_ide=True)
MediaManager.init()

# Start sensor
sensor.run()
time.sleep_ms(800)  # let auto-exposure settle

# Warm up
for _ in range(5):
    try:
        sensor.snapshot(chn=CAM_CHN_ID_2)
    except:
        time.sleep_ms(100)

print("\n[DIAG] === CHN_2 RGBP888 numpy inspection ===")
try:
    img2 = sensor.snapshot(chn=CAM_CHN_ID_2)
    np2 = img2.to_numpy_ref()

    print("[DIAG] type:", type(np2))
    print("[DIAG] shape:", np2.shape)
    print("[DIAG] dtype:", np2.dtype)

    # strides tell us the real memory layout
    try:
        print("[DIAG] strides:", np2.strides)
    except Exception as e:
        print("[DIAG] strides not available:", e)

    # Per-channel stats
    for ch in range(np2.shape[0]):
        ch_data = np2[ch]
        print("[DIAG] Channel {}: min={} max={} mean={:.1f}".format(
            ch, ch_data.min(), ch_data.max(), ch_data.mean()))

    # First row, first 10 pixels of each channel
    print("[DIAG] Channel 0 (R?), row 0 first 10 px:", np2[0, 0, :10].tolist())
    print("[DIAG] Channel 1 (G?), row 0 first 10 px:", np2[1, 0, :10].tolist())
    print("[DIAG] Channel 2 (B?), row 0 first 10 px:", np2[2, 0, :10].tolist())

    # Middle pixel values
    cy, cx = np2.shape[1] // 2, np2.shape[2] // 2
    print("[DIAG] Center pixel (R,G,B):", np2[0, cy, cx], np2[1, cy, cx], np2[2, cy, cx])

    # Check if data looks valid (not all zeros, not all 255)
    total_pixels = np2.shape[0] * np2.shape[1] * np2.shape[2]
    zeros = 0
    for ch in range(3):
        zeros += (np2[ch] == 0).sum()
    print("[DIAG] Zero pixels: {}/{} ({:.1f}%)".format(zeros, total_pixels, 100*zeros/total_pixels))

    # Check if the numpy is contiguous
    try:
        print("[DIAG] is_contiguous:", np2.is_contiguous())
    except:
        print("[DIAG] is_contiguous: (check not available)")

except Exception as e:
    print("[DIAG] CHN_2 inspect FAILED:", e)
    import sys
    sys.print_exception(e)

# Also check CHN_0 RGB565
print("\n[DIAG] === CHN_0 RGB565 snapshot ===")
try:
    img0 = sensor.snapshot()  # default channel
    print("[DIAG] CHN_0 size: {}x{}".format(img0.width(), img0.height()))
    print("[DIAG] CHN_0 format:", img0.format())

    # Get a pixel from the center
    print("[DIAG] CHN_0 center pixel value:", img0.get_pixel(320, 240))
except Exception as e:
    print("[DIAG] CHN_0 inspect FAILED:", e)

# Test: what does to_numpy_ref return for CHN_0 (RGB565)?
print("\n[DIAG] === CHN_0 to_numpy_ref (expect HWC or CHW?) ===")
try:
    np0 = img0.to_numpy_ref()
    print("[DIAG] CHN_0 numpy shape:", np0.shape)
    print("[DIAG] CHN_0 numpy dtype:", np0.dtype)
except Exception as e:
    print("[DIAG] CHN_0 to_numpy_ref FAILED:", e)

print("\n[DIAG] DONE — copy the output above for analysis")
sensor.stop()
Display.deinit()
MediaManager.deinit()
