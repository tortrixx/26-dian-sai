import media

print("====== media 模块探索 ======")
print("dir(media):")
for n in sorted(dir(media)):
    if not n.startswith("_"):
        attr = getattr(media, n)
        print("  ", n, "->", type(attr))

# 尝试 sensor 子模块
for sub in ["sensor", "camera", "camerart", "video", "display",
            "lcd", "vo", "vi", "vpss", "venc", "vdec", "audio"]:
    try:
        mod = __import__("media." + sub)
        print("[FOUND] media.{}".format(sub))
        print("  dir:", [n for n in dir(getattr(media, sub)) if not n.startswith("_")])
    except Exception as e:
        print("[NO] media.{} -> {}".format(sub, e))

print("====== 完成 ======")
