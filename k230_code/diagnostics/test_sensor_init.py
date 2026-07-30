"""Minimal sensor init test — isolate where the hang occurs."""
from media.sensor import *
from media.display import *
from media.media import *
import time

print("[TEST] Cleanup...")
try: Display.deinit()
except: pass
try: MediaManager.deinit()
except: pass
print("[TEST] Cleanup done")

print("[TEST] Step 1: Sensor(id=2, width=640, height=480, fps=30)...")
try:
    sensor = Sensor(id=2, width=640, height=480, fps=30)
    print("[TEST]   OK")
except Exception as e:
    print("[TEST]   FAIL:", e)
    raise

print("[TEST] Step 2: sensor.reset()...")
try:
    sensor.reset()
    print("[TEST]   OK")
except Exception as e:
    print("[TEST]   FAIL:", e)
    raise

print("[TEST] Step 3: CHN_0 config...")
try:
    sensor.set_pixformat(Sensor.RGB565)
    sensor.set_framesize(width=640, height=480)
    print("[TEST]   OK")
except Exception as e:
    print("[TEST]   FAIL:", e)
    raise

print("[TEST] Step 4: CHN_2 config (RGB888 for AI)...")
try:
    sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
    sensor.set_framesize(w=640, h=480, chn=CAM_CHN_ID_2)
    print("[TEST]   OK")
except Exception as e:
    print("[TEST]   FAIL:", e)
    raise

print("[TEST] Step 5: Display.init(VIRT)...")
try:
    Display.init(Display.VIRT, 640, 480, to_ide=True)
    print("[TEST]   OK")
except Exception as e:
    print("[TEST]   FAIL:", e)
    raise

print("[TEST] Step 6: MediaManager.init()...")
try:
    MediaManager.init()
    print("[TEST]   OK")
except Exception as e:
    print("[TEST]   FAIL:", e)
    raise

print("[TEST] Step 7: sensor.run()...")
try:
    sensor.run()
    print("[TEST]   OK")
except Exception as e:
    print("[TEST]   FAIL:", e)
    raise

time.sleep_ms(500)

print("[TEST] Step 8: snapshots from CHN_2...")
try:
    for i in range(5):
        img = sensor.snapshot(chn=CAM_CHN_ID_2)
        print("[TEST]   snapshot {}: {}x{}".format(i, img.width(), img.height()))
        time.sleep_ms(200)
    print("[TEST]   OK")
except Exception as e:
    print("[TEST]   FAIL:", e)
    raise

print("[TEST] ALL STEPS PASSED — sensor init works!")
