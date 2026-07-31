"""Transfer k230_yolo.py — catches REPL before auto-script starts."""
import serial, time, base64

COM = 'COM6'
LOCAL = r'C:\Users\sznnn\Desktop\26-dian-sai\k230_code\k230_yolo.py'
REMOTE = '/sdcard/app/k230_yolo.py'

ser = serial.Serial(COM, 115200, timeout=0.5)

# Reset + spam Ctrl-C during boot to catch REPL before auto-script starts
ser.setDTR(False); time.sleep(0.1); ser.setDTR(True)
for i in range(30):
    ser.write(b'\x03')
    time.sleep(0.05)
time.sleep(0.5)
ser.read(ser.in_waiting)

# Enter raw REPL
ser.write(b'\r\x01')
time.sleep(0.5)
data = ser.read(ser.in_waiting)
if b'raw REPL' not in data:
    ser.write(b'\r\x01')
    time.sleep(0.5)
    data += ser.read(ser.in_waiting)
if b'raw REPL' not in data:
    print('FAIL: no raw REPL. Got:', data[:100])
    ser.close()
    exit(1)

print('Raw REPL entered')

def exec_repl(code, timeout=10):
    ser.write(code.encode('utf-8'))
    ser.write(b'\r\x04')
    time.sleep(0.3)
    out = b''
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            out += chunk
            if b'OK' in out[-20:] or b'\x04>' in out[-5:]:
                break
        time.sleep(0.05)
    return out

# Test REPL
out = exec_repl('print(99999)\r\n')
print('Test:', out[:100])

# Delete old file — must succeed before appending
out = exec_repl("import os\r\ntry:\r\n os.remove('" + REMOTE + "')\r\n print('DELETED')\r\nexcept Exception as e:\r\n print('NO_DELETE', e)\r\n")
print('Delete:', out[:100])
if b'DELETED' not in out and b'exist' not in out and b'ENOENT' not in out:
    print('WARNING: delete may have failed — retrying with rm via os')
    out = exec_repl("import os\r\nos.remove('" + REMOTE + "') if '" + REMOTE + "' in os.listdir('/sdcard/app') else None\r\nprint('OK')\r\n")
time.sleep(0.3)

# Read local file
with open(LOCAL, 'rb') as f:
    content = f.read()
size = len(content)
print('Transferring k230_yolo.py ({} bytes)...'.format(size))

# Transfer in base64 chunks
encoded = base64.b64encode(content).decode('ascii')
CHUNK = 2000
total_chunks = (len(encoded) + CHUNK - 1) // CHUNK
for i in range(0, len(encoded), CHUNK):
    n = i // CHUNK
    chunk_b64 = encoded[i:i + CHUNK]
    code = ("import ubinascii\r\n"
            "d=ubinascii.a2b_base64('" + chunk_b64 + "')\r\n"
            "f=open('" + REMOTE + "','ab')\r\n"
            "f.write(d)\r\n"
            "f.close()\r\n"
            "print('.')\r\n")
    out = exec_repl(code, timeout=10)
    if b'.' not in out:
        print('FAIL at chunk {}/{}'.format(n + 1, total_chunks))
        ser.close()
        exit(1)
    if n % 25 == 0:
        pct = min(100, (i + CHUNK) * 100 // len(encoded))
        print('  {}% ({}/{})'.format(pct, n + 1, total_chunks))

print('100% OK')

# Verify
out = exec_repl("import os; s=os.stat('" + REMOTE + "'); print(s[6])\r\n", timeout=5)
print('Remote verification: {}'.format(out))

ser.close()
print('Done!')
