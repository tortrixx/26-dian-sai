"""Passive test: listen on 8888 for K230 K23V video stream (15s window)."""
import socket, time

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", 8888))
srv.listen(1)
srv.settimeout(15)

print("[TEST] Listening on :8888 for 15s...")
try:
    conn, addr = srv.accept()
    print("[TEST] K230 connected from:", addr)
    conn.settimeout(5)
    total = 0
    magic_seen = None
    start = time.time()
    while time.time() - start < 8:
        try:
            data = conn.recv(65536)
        except socket.timeout:
            continue
        if not data:
            print("[TEST] connection closed by K230")
            break
        total += len(data)
        if b'K23V' in data and magic_seen is None:
            magic_seen = True
            print("[TEST] K23V header found! Streaming OK")
        if total > 200000:
            break
    print("[TEST] Received {} bytes in {:.1f}s".format(total, time.time() - start))
    conn.close()
except socket.timeout:
    print("[TEST] TIMEOUT — no K230 connection in 15s")
except Exception as e:
    print("[TEST] Error:", e)
finally:
    srv.close()
