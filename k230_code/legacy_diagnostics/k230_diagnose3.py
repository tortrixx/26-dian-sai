# 深入排查 ImageIO —— 很可能就是摄像头接口
import image

print("====== ImageIO 探索 ======")
print("image.ImageIO:", image.ImageIO)
print("dir(ImageIO):")
for name in sorted(dir(image.ImageIO)):
    if not name.startswith("_"):
        attr = getattr(image.ImageIO, name)
        print("  ", name, "->", type(attr), end="")
        # 如果是 function/method，尝试看签名
        if "function" in str(type(attr)):
            print("  [callable]")
        else:
            print()

print("---")

# fb_stat 看看
print("image.fb_stat():", image.fb_stat())

print("---")

# 也看看 Image 类
print("image.Image:", image.Image)
print("dir(Image):")
for name in sorted(dir(image.Image)):
    if not name.startswith("_"):
        attr = getattr(image.Image, name)
        print("  ", name, "->", type(attr), end="")
        if "function" in str(type(attr)) or "method" in str(type(attr)):
            print("  [callable]")
        else:
            print()

print("====== 完成 ======")
