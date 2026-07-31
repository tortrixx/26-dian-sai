"""Transfer k230_yolo.py — matches _reset_k230.py proven REPL capture."""
import serial, time, base64

COM = 'COM6'
LOCAL = r'C:\Users\sznnn\Desktop\26-dian-sai\k230_code\k230_yolo.py'
REMOTE = '/sdcard/app/k230_yolo.py'
AUTOSTART = '/sdcard/main.py'

ser = serial.Serial(COM, 115200, timeout=0.5)

# Step 1: Hard reset + Ctrl-C spam (proven technique from _reset_k230.py)
print("[1/5] DTR reset + Ctrl-C spam...")
ser.setDTR(False); time.sleep(0.15); ser.setDTR(True); time.sleep(0.3)
ser.read(ser.in_waiting)

for i in range(50):
    ser.write(b'\x03')
    time.sleep(0.03)
time.sleep(1.0)
ser.read(ser.in_waiting)

# Enter raw REPL
ser.write(b'\r\x01')
time.sleep(0.5)
data = ser.read(ser.in_waiting)
if b'raw REPL' not in data:
    ser.write(b'\r\x01')
    time.sleep(0.5)
    data += ser.read(ser.in_waiting)

if b'raw REPL' in data:
    print("  Raw REPL OK — soft reset")
    ser.write(b'\x04')
    time.sleep(1.0)
    data = ser.read(ser.in_waiting)
    print("  Reset:", data[:100] if data else "(empty)")
else:
    print("  No raw REPL, trying direct exec...")
    ser.write(b'\r\x03\x03\x03')
    time.sleep(0.5)
    ser.read(ser.in_waiting)

# ---- Test REPL ----
print("[2/5] Test REPL...")
ser.write(b"print(99999)\r\n\x04")
time.sleep(0.3)
out = b''
deadline = time.time() + 3
while time.time() < deadline:
    if ser.in_waiting:
        out += ser.read(ser.in_waiting)
    if b'99999' in out or b'OK' in out[-10:]:
        break
    time.sleep(0.05)
print("  Test:", out[:100] if out else "(empty)")

# ---- Delete old ----
print("[3/5] Delete old files...")
for remote_file in (REMOTE, AUTOSTART):
    code = ("import os\r\ntry:\r\n os.remove('" + remote_file + "')\r\n print('DEL')\r\nexcept Exception as e:\r\n print('SKIP')\r\n")
    ser.write(code.encode() + b'\r\x04')
    time.sleep(0.3)
    out = ser.read(ser.in_waiting)
    print("  " + remote_file + ":", out[:80] if out else "(empty)")

# ---- Transfer ----
with open(LOCAL, 'rb') as f:
    content = f.read()
size = len(content)
print("[4/5] Transfer {} bytes...".format(size))

encoded = base64.b64encode(content).decode('ascii')
CHUNK = 2000
total = (len(encoded) + CHUNK - 1) // CHUNK
for idx in range(0, len(encoded), CHUNK):
    n = idx // CHUNK
    chunk = encoded[idx:idx + CHUNK]
    code = ("import ubinascii\r\nd=ubinascii.a2b_base64('" + chunk + "')\r\nf=open('" + REMOTE + "','ab')\r\nf.write(d)\r\nf.close()\r\nprint('.')\r\n")
    ser.write(code.encode() + b'\r\x04')
    time.sleep(0.1)
    out = b''
    dl = time.time() + 5
    while time.time() < dl:
        if ser.in_waiting:
            out += ser.read(ser.in_waiting)
        if b'.' in out or b'OK' in out[-10:]:
            break
        time.sleep(0.03)
    if b'.' not in out and b'OK' not in out[-10:]:
        print("  FAIL chunk {}/{}".format(n + 1, total))
        ser.close()
        exit(1)
    if n % 20 == 0:
        print("  {}% ({}/{})".format(min(100, (idx + CHUNK) * 100 // len(encoded)), n + 1, total))

print("  100% OK")

# ---- Verify + auto-start ----
print("[5/5] Verify + auto-start...")
ser.write(b"import os; s=os.stat('" + REMOTE.encode() + b"'); print(s[6])\r\n\x04")
time.sleep(0.5)
out = ser.read(ser.in_waiting)
print("  Size:", out[:80] if out else "(empty)")

code = ("f=open('/sdcard/main.py','w')\r\nf.write(\"exec(open('/sdcard/app/k230_yolo.py').read())\")\r\nf.close()\r\nprint('AUTO OK')\r\n")
ser.write(code.encode() + b'\r\x04')
time.sleep(0.5)
out = ser.read(ser.in_waiting)
print("  Auto-start:", out[:80] if out else "(empty)")

# Soft reset
ser.write(b'\x04')
time.sleep(2.0)
data = ser.read(ser.in_waiting)
print("  Boot:", data[:300] if data else "(empty)")

ser.close()
print("\nDone!")
