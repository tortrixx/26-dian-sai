"""K230 file transfer over MicroPython raw REPL via serial port.

Usage: python transfer_to_k230.py <com_port>
"""

import serial
import sys
import os
import base64
import time

COM_PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
BAUD = 115200

# Files to transfer: (local_path, sdcard_path)
FILES = [
    # SD card root
    (r"C:\Users\sznnn\Downloads\Google下载\yolo11n_det_320.kmodel",
     "/sdcard/yolo11n_det_320.kmodel"),
    # Libs
    (r"C:\Users\sznnn\Desktop\26-dian-sai\k230_code\k230_yolo.py",
     "/sdcard/k230_yolo.py"),
    (r"C:\Users\sznnn\Desktop\26-dian-sai\k230_code\libs\AIBase.py",
     "/sdcard/app/libs/AIBase.py"),
    (r"C:\Users\sznnn\Desktop\26-dian-sai\k230_code\libs\AI2D.py",
     "/sdcard/app/libs/AI2D.py"),
    (r"C:\Users\sznnn\Desktop\26-dian-sai\k230_code\libs\PipeLine.py",
     "/sdcard/app/libs/PipeLine.py"),
    (r"C:\Users\sznnn\Desktop\26-dian-sai\k230_code\libs\Utils.py",
     "/sdcard/app/libs/Utils.py"),
    (r"C:\Users\sznnn\Desktop\26-dian-sai\k230_code\libs\YOLO.py",
     "/sdcard/app/libs/YOLO.py"),
]


def enter_raw_repl(ser):
    """Enter MicroPython raw REPL mode."""
    ser.write(b'\r\x03\x03')  # Ctrl-C twice to interrupt
    time.sleep(0.3)
    ser.read(ser.in_waiting)  # flush
    ser.write(b'\r\x01')      # Ctrl-A to enter raw REPL
    time.sleep(0.3)
    data = ser.read(ser.in_waiting)
    if b'raw REPL' in data:
        print("  Raw REPL entered")
        return True
    return False


def exec_repl(ser, code, timeout=10):
    """Execute Python code in raw REPL and return output."""
    ser.write(code.encode('utf-8'))
    ser.write(b'\r\x04')  # Ctrl-D to execute
    time.sleep(0.3)
    output = b''
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            output += chunk
            if b'OK' in output[-20:] or b'\x04>' in output[-5:]:
                break
        time.sleep(0.1)
    return output


def ensure_dir(ser, path):
    """Create directory on K230 if it doesn't exist."""
    dir_path = os.path.dirname(path)
    if not dir_path or dir_path == '/':
        return True
    code = f"import os\r\nos.makedirs('{dir_path}', exist_ok=True)\r\nprint('DIR_OK')\r\n"
    out = exec_repl(ser, code)
    return b'DIR_OK' in out or b'OK' in out


def transfer_text_file(ser, local_path, remote_path):
    """Transfer a .py text file to K230."""
    with open(local_path, 'rb') as f:
        content = f.read()
    size = len(content)
    print(f"  {os.path.basename(local_path)} ({size} bytes) → {remote_path}")

    # Make dirs
    ensure_dir(ser, remote_path)

    # For small files, send directly. For large files, chunk it.
    MAX_CHUNK = 4096  # Keep REPL commands manageable
    if size <= MAX_CHUNK:
        # Escape quotes and newlines for Python string
        escaped = content.decode('utf-8').replace('\\', '\\\\').replace("'", "\\'")
        code = f"f=open('{remote_path}','w')\r\nf.write('''{escaped}''')\r\nf.close()\r\nprint('OK')\r\n"
        out = exec_repl(ser, code, timeout=15)
        return b'OK' in out
    else:
        # Write in chunks using binary append
        # First, delete the file
        code = f"try:\r\n os.remove('{remote_path}')\r\nexcept:\r\n pass\r\n"
        exec_repl(ser, code)

        # Write each chunk as base64-decoded binary
        encoded = base64.b64encode(content).decode('ascii')
        CHUNK = 2000  # b64 chars per chunk
        for i in range(0, len(encoded), CHUNK):
            chunk_b64 = encoded[i:i + CHUNK]
            code = (f"import ubinascii\r\n"
                    f"d=ubinascii.a2b_base64('{chunk_b64}')\r\n"
                    f"f=open('{remote_path}','ab')\r\n"
                    f"f.write(d)\r\n"
                    f"f.close()\r\n"
                    f"print('.')\r\n")
            out = exec_repl(ser, code, timeout=10)
            if b'.' not in out:
                print(f"    chunk {i // CHUNK} failed!")
                return False
        return True


def transfer_binary_file(ser, local_path, remote_path):
    """Transfer a binary file (.kmodel) using base64 over REPL."""
    with open(local_path, 'rb') as f:
        content = f.read()
    size = len(content)
    print(f"  {os.path.basename(local_path)} ({size / 1024:.1f} KB) → {remote_path}")

    # Make dirs
    ensure_dir(ser, remote_path)

    # Delete existing file
    code = f"try:\r\n os.remove('{remote_path}')\r\nexcept:\r\n pass\r\nprint('READY')\r\n"
    out = exec_repl(ser, code)

    # Encode entire file as base64
    print(f"  Encoding {size} bytes to base64...")
    encoded = base64.b64encode(content).decode('ascii')
    print(f"  Encoded: {len(encoded)} chars, transferring in chunks...")

    # Transfer in chunks
    CHUNK = 2000  # b64 chars per chunk (1.5KB binary)
    total_chunks = (len(encoded) + CHUNK - 1) // CHUNK
    for i in range(0, len(encoded), CHUNK):
        chunk_num = i // CHUNK
        chunk_b64 = encoded[i:i + CHUNK]
        code = (f"import ubinascii\r\n"
                f"d=ubinascii.a2b_base64('{chunk_b64}')\r\n"
                f"f=open('{remote_path}','ab')\r\n"
                f"f.write(d)\r\n"
                f"f.close()\r\n"
                f"print('.')\r\n")
        out = exec_repl(ser, code, timeout=15)
        if b'.' not in out:
            print(f"    chunk {chunk_num}/{total_chunks} failed!")
            return False
        if chunk_num % 50 == 0:
            pct = (i + len(chunk_b64)) / len(encoded) * 100
            print(f"    {pct:.0f}% ({chunk_num}/{total_chunks})")

    return True


def main():
    print(f"Connecting to {COM_PORT} at {BAUD} baud...")
    ser = serial.Serial(COM_PORT, BAUD, timeout=1)
    ser.setDTR(False)
    time.sleep(0.1)
    ser.setDTR(True)
    time.sleep(1.5)
    ser.read(ser.in_waiting)

    # Try to enter raw REPL
    print("Entering raw REPL...")
    if not enter_raw_repl(ser):
        print("Failed to enter raw REPL. Is K230 connected and not running a script?")
        print("Try pressing Ctrl+C in CanMV IDE to stop any running script, then re-run.")
        ser.close()
        return 1

    # Check we can execute code
    out = exec_repl(ser, "print('HELLO_K230')\r\n")
    if b'HELLO_K230' not in out:
        print(f"REPL test failed. Output: {out[:100]}")
        ser.close()
        return 1
    print("REPL OK, starting file transfer...\n")

    # Transfer files
    ok, fail = 0, 0
    for local_path, remote_path in FILES:
        if not os.path.exists(local_path):
            print(f"  SKIP (not found): {local_path}")
            fail += 1
            continue

        try:
            if local_path.endswith('.kmodel'):
                success = transfer_binary_file(ser, local_path, remote_path)
            else:
                success = transfer_text_file(ser, local_path, remote_path)

            if success:
                print(f"  OK!")
                ok += 1
            else:
                print(f"  FAILED!")
                fail += 1
        except Exception as e:
            print(f"  ERROR: {e}")
            fail += 1

        # Soft-reboot between large files to clear memory
        if local_path.endswith('.kmodel'):
            print("  Soft reboot to clear buffers...")
            ser.write(b'\x04')  # Ctrl-D (soft reset in raw REPL)
            time.sleep(2)
            ser.read(ser.in_waiting)
            enter_raw_repl(ser)

    print(f"\nDone: {ok} transferred, {fail} failed")

    # Verify
    print("\nVerifying files on K230...")
    out = exec_repl(ser, "import os\r\nfor f in os.listdir('/sdcard'):\r\n print(f)\r\n", timeout=5)
    print(out.decode('utf-8', errors='replace'))

    ser.close()
    return 0 if fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
