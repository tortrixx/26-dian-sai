"""Reset K230 via DTR + Ctrl-C, then run k230_yolo.py."""
import serial, time

ser = serial.Serial("COM6", 115200, timeout=0.5)

# Hard reset via DTR
ser.setDTR(False); time.sleep(0.15); ser.setDTR(True); time.sleep(0.3)
ser.read(ser.in_waiting)

# Spam Ctrl-C during boot to catch REPL
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
    print("REPL OK — soft reset")
    ser.write(b'\x04')
    time.sleep(1.0)
    data = ser.read(ser.in_waiting)
    print("Reset response:", data[:200].decode('utf-8', errors='replace'))
else:
    print("No raw REPL, trying direct exec")
    ser.write(b'\r\x03\x03\x03')
    time.sleep(0.5)
    ser.read(ser.in_waiting)

# Run the new k230_final.py (motion-based detector)
print("Launching k230_final.py...")
cmd = b"exec(open('/sdcard/app/k230_final.py').read())\r\n"
ser.write(cmd)
time.sleep(2.0)
data = ser.read(ser.in_waiting)
print("Output:", data[:500].decode('utf-8', errors='replace'))

ser.close()
print("Done — K230 should be running new code")
