# 精确排查摄像头 API
import image
import sys

print("====== image 模块探索 ======")
print("dir(image):")
for name in sorted(dir(image)):
    if not name.startswith("_"):
        print("  ", name, "->", type(getattr(image, name)))

print("---")

# 尝试常见名称
for name in ["Sensor", "Camera", "Capture", "MediaManager", "sensor",
             "ImageSensor", "VideoSource"]:
    if hasattr(image, name):
        obj = getattr(image, name)
        print("[HAS] image.{} -> {}".format(name, obj))

# 尝试从 sys.modules 找隐藏模块
print("---")
print("sys.modules keys (all loaded):")
for key in sorted(sys.modules.keys()):
    if key not in ("sys", "os", "gc", "machine", "time", "math",
                   "image", "ulab", "network", "socket", "ssl",
                   "struct", "utime", "uio", "__main__"):
        print("  [+] ", key)

print("====== 完成 ======")
