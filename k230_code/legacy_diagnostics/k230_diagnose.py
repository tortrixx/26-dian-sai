# K230 模块诊断 —— 烧录运行，看 IDE 串口输出
# 找出这块板子实际的摄像头 API 叫什么

print("====== K230 模块诊断 ======")

# 1. 基础模块
for m in ["sys", "os", "gc", "machine", "time", "math"]:
    try:
        __import__(m)
        print("[OK] import", m)
    except Exception as e:
        print("[NO] import", m, "->", e)

print("---")

# 2. 传感器/摄像头相关
for m in ["sensor", "image", "camera", "video", "lcd", "display",
           "ulab", "numpy", "nncase", "ai_lib", "kpuhal"]:
    try:
        __import__(m)
        print("[OK] import", m)
    except Exception as e:
        print("[NO] import", m, "->", e)

print("---")

# 3. 网络
for m in ["network", "socket", "wlan", "ssl"]:
    try:
        __import__(m)
        print("[OK] import", m)
    except Exception as e:
        print("[NO] import", m, "->", e)

print("---")

# 4. 查看所有已加载模块名
print("dir(sys.modules):")
import sys
for name in sorted(sys.modules.keys()):
    print(" ", name)

print("====== 诊断完成 ======")
