# K230 文件扫描 —— 找 PipeLine YOLO Utils
import os

print("===== 扫描 /sdcard =====")
def walk(path, indent=0):
    try:
        for item in os.listdir(path):
            full = path + "/" + item
            try:
                s = os.stat(full)
                if s[0] & 0x4000:
                    print("  " * indent + "[DIR]", item)
                    walk(full, indent + 1)
                else:
                    print("  " * indent + "  {}  ({}B)".format(item, s[6]))
            except:
                print("  " * indent + "  {}  (?)".format(item))
    except Exception as e:
        print("  " * indent + "err:", e)

walk("/sdcard")

print("\n===== 扫描 /data =====")
walk("/data")

print("\n===== 找 PipeLine =====")
import sys
for m in ["libs.PipeLine", "libs.YOLO", "libs.Utils",
          "PipeLine", "YOLO", "Utils"]:
    try:
        __import__(m)
        print(f"  [OK] import {m}")
    except:
        print(f"  [NO] import {m}")

print("\n===== 完成 =====")
