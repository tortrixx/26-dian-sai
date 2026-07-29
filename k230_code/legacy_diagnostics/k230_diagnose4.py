# 终极排查 —— 搜文件系统 + machine 模块 + 隐藏模块
import sys, os, machine, gc

print("====== 文件系统 ======")
try:
    for entry in os.ilistdir():
        print(" ", entry)
except Exception as e:
    print("ilistdir error:", e)

print("---")

# 递归列出所有文件
def walk(path=""):
    try:
        for name, typ, inode, size in os.ilistdir(path or "/"):
            full = (path + "/" + name).replace("//", "/")
            if typ == 0x4000:  # dir
                print("  [DIR]", full)
                walk(full)
            else:
                print("  [FILE]", full, "({}B)".format(size))
    except Exception as e:
        print("walk error:", e)

walk()

print("---")

# 搜 media 模块
for m in ["media", "maix", "k230", "canaan", "yahboom", "canmv",
          "cam", "ov5647", "ov5640", "gc2053"]:
    try:
        mod = __import__(m)
        print("[FOUND] module:", m, "->", mod)
    except Exception as e:
        print("[NO]", m, "->", e)

print("---")

# machine 模块看看
print("dir(machine):")
for n in sorted(dir(machine)):
    if not n.startswith("_"):
        print("   ", n)

print("====== 完成 ======")
