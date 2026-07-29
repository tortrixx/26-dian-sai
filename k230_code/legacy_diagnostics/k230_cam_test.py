# 摄像头最小化测试 —— 试多种初始化方式
from media.sensor import *
import time

print("===== 摄像头最小化测试 =====")

# 试多种参数组合
tests = [
    ("Sensor()", lambda: Sensor()),
    ("Sensor(id=0)", lambda: Sensor(id=0)),
    ("Sensor(id=2)", lambda: Sensor(id=2)),
    ("Sensor(width=1920,height=1080)", lambda: Sensor(width=1920, height=1080)),
]

for label, factory in tests:
    print("\n--- 测试:", label, "---")
    try:
        sensor = factory()
        print("Sensor created OK")
        sensor.reset()
        print("reset() OK")
        time.sleep(0.5)
        sensor.run()
        print("run() OK!")
        w = sensor.width()
        h = sensor.height()
        print("{}x{} pixfmt:{}".format(w, h, sensor.get_pixformat()))

        # 试采一帧
        time.sleep(0.5)
        img = sensor.snapshot()
        print("snapshot() OK, img:", img)
        sensor.stop()
        MediaManager.deinit()
        print("===== 成功! =====")
        break
    except Exception as e:
        print("FAIL:", e)
        try: sensor.stop()
        except: pass
        # 重新 init 给下一个测试
        try: MediaManager.deinit()
        except: pass
        time.sleep(0.5)
        MediaManager.init()
