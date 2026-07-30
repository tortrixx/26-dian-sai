"""Verify k230_yolo.py on SD card is the updated version."""
import os

PATH = "/sdcard/app/k230_yolo.py"

print("[CHECK] Reading", PATH)

try:
    stat = os.stat(PATH)
    print("[CHECK] File size:", stat[6], "bytes")
except Exception as e:
    print("[CHECK] os.stat failed:", e)

try:
    f = open(PATH, "r")
    data = f.read()
    f.close()
    lines = data.split("\n")
    print("[CHECK] Total lines:", len(lines))
    print("[CHECK] Total chars:", len(data))

    checks = {
        "UART fix (EIGHTBITS)": "UART.EIGHTBITS" in data,
        "sys.path insert": "sys.path.insert" in data,
        "Sensor direct (no PipeLine)": 'Sensor(id=2' in data,
        "Init camera debug": "Init camera" in data,
        "CHN_2 configured debug": "CHN_2 configured" in data,
        "YOLO loading debug": "Loading YOLO11 NPU model" in data,
        "Model ready message": "YOLO11 model ready" in data,
        "No PipeLine import": "from libs.PipeLine" not in data,
    }

    all_ok = True
    for label, ok in checks.items():
        mark = "OK" if ok else "MISSING!"
        if not ok:
            all_ok = False
        print("[CHECK] {}: {}".format(label, mark))

    if all_ok:
        print("\n[CHECK] FILE IS CORRECT — latest version is on the K230.")
    else:
        print("\n[CHECK] FILE IS OLD — re-copy k230_yolo.py from PC to /sdcard/app/")

    # Show first line to identify version
    print("[CHECK] First 3 lines:")
    for ln in lines[:3]:
        print("  ", ln)

except Exception as e:
    print("[CHECK] FAILED to read file:", e)
