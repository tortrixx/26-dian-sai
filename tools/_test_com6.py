"""Aggressive K230 REPL capture — matches _reset_k230.py proven approach."""
import serial, time

ser = serial.Serial("COM6", 115200, timeout=0.3)

# Hard reset via DTR
ser.setDTR(False)
time.sleep(0.15)
ser.setDTR(True)
time.sleep(0.3)
ser.read(ser.in_waiting)

# Round 1: aggressive Ctrl-C spam during boot
for i in range(50):
    ser.write(b'\x03')
    time.sleep(0.03)
time.sleep(0.8)
data = ser.read(ser.in_waiting)
print("Round 1 - After Ctrl-C:", len(data), "bytes")

# Try raw REPL
ser.write(b'\r\x01')
time.sleep(0.5)
data = ser.read(ser.in_waiting)
print("Round 1 - REPL:", len(data), "bytes")

if b'raw REPL' in data:
    print(">>> RAW REPL OK (round 1)!")
else:
    # Round 2: try again with soft reset
    print("Round 1 failed, trying round 2...")
    ser.write(b'\r\x03\x03\x03')
    time.sleep(0.5)
    ser.read(ser.in_waiting)

    ser.write(b'\r\x01')
    time.sleep(0.5)
    data = ser.read(ser.in_waiting)
    print("Round 2 - REPL:", len(data), "bytes")

    if b'raw REPL' in data:
        print(">>> RAW REPL OK (round 2)!")
    else:
        # Round 3: hard reset again, faster spam
        print("Round 2 failed, trying round 3 (hard reset)...")
        ser.setDTR(False)
        time.sleep(0.1)
        ser.setDTR(True)
        time.sleep(0.15)
        ser.read(ser.in_waiting)

        for i in range(100):
            ser.write(b'\x03')
            time.sleep(0.015)
        time.sleep(0.5)

        ser.write(b'\r\x01')
        time.sleep(0.5)
        data = ser.read(ser.in_waiting)
        print("Round 3 - REPL:", len(data), "bytes")

        if b'raw REPL' in data:
            print(">>> RAW REPL OK (round 3)!")
        else:
            print(">>> ALL ROUNDS FAILED")
            if data:
                print("Last data:", data[-200:])

ser.close()
