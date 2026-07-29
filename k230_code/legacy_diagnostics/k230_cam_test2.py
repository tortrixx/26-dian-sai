# 摄像头 snapshot 测试 —— 试不同配置组合
from media.sensor import *
from media.display import *
import time

print("===== snapshot 测试 =====")

# 配置 A: 带 Display.VIRT
print("\n--- 配置A: 带 Display.VIRT ---")
try:
    sensor = Sensor(id=2, width=1920, height=1080, fps=30)
    sensor.reset()
    sensor.set_pixformat(Sensor.RGB565)
    Display.init(Display.VIRT, sensor.width(), sensor.height(), to_ide=True)
    MediaManager.init()
    sensor.run()
    print("run() OK, 等帧...")
    time.sleep(3)
    for i in range(5):
        try:
            img = sensor.snapshot()
            print("snapshot OK! img size:", img.width(), "x", img.height())
            break
        except Exception as e:
            print("  snapshot retry {}: {}".format(i, e))
            time.sleep(1)
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
except Exception as e:
    print("FAIL:", e)
    try: sensor.stop()
    except: pass
    try: Display.deinit()
    except: pass
    try: MediaManager.deinit()
    except: pass

time.sleep(1)

# 配置 B: 不用 Display
print("\n--- 配置B: 不用 Display ---")
try:
    sensor = Sensor(id=2, width=1920, height=1080, fps=30)
    sensor.reset()
    sensor.set_pixformat(Sensor.RGB565)
    print("MediaManager.init()...")
    MediaManager.init()
    print("sensor.run()...")
    sensor.run()
    print("run() OK, 等帧...")
    time.sleep(3)
    for i in range(5):
        try:
            img = sensor.snapshot()
            print("snapshot OK! img size:", img.width(), "x", img.height())
            break
        except Exception as e:
            print("  snapshot retry {}: {}".format(i, e))
            time.sleep(1)
    sensor.stop()
    MediaManager.deinit()
except Exception as e:
    print("FAIL:", e)
    try: sensor.stop()
    except: pass
    try: MediaManager.deinit()
    except: pass

print("===== 完成 =====")
