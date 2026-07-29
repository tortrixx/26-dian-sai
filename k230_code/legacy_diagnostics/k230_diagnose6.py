import media
from media.sensor import *    # 这才是正确的 import 方式

print("====== Sensor 类 ======")
print("Sensor:", Sensor)
print("dir(Sensor):")
for n in sorted(dir(Sensor)):
    if not n.startswith("_"):
        print("  ", n)

print()
print("====== MediaManager 类 ======")
print("MediaManager:", MediaManager)
print("dir(MediaManager):")
for n in sorted(dir(MediaManager)):
    if not n.startswith("_"):
        print("  ", n)

print()
print("====== get_default_sensor ======")
print("get_default_sensor():", get_default_sensor())

print()
print("====== Display 类 ======")
from media.display import *
print("Display:", Display)
print("dir(Display):")
for n in sorted(dir(Display)):
    if not n.startswith("_"):
        print("  ", n)

print("====== 完成 ======")
