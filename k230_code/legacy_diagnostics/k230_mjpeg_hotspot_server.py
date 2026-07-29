"""
CanMV-K230 MJPEG stream over a Windows mobile hotspot.

Before running:
1. Set WIFI_SSID and WIFI_PASSWORD.
2. Confirm the K230 board has a supported Wi-Fi module.
3. Save this file as /sdcard/main.py for automatic startup.

Open http://<K230_IP>:8080/stream on the PC.
"""

import gc
import socket
import time
import network

from media.sensor import Sensor
from media.media import MediaManager

WIFI_SSID = "K230_HOTSPOT"
WIFI_PASSWORD = "ChangeMe123"

HTTP_PORT = 8080
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
JPEG_QUALITY = 60
STREAM_FPS = 12


def connect_wifi(timeout_s=20):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print("Connecting to Wi-Fi:", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)

        start_ms = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), start_ms) > timeout_s * 1000:
                raise RuntimeError("Wi-Fi connection timeout")
            time.sleep_ms(200)

    config = wlan.ifconfig()
    print("Wi-Fi connected:", config)
    return wlan, config[0]


def init_camera():
    camera = Sensor()
    camera.reset()
    camera.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
    camera.set_pixformat(Sensor.RGB565)

    MediaManager.init()
    camera.run()
    return camera


def jpeg_bytes(image):
    compressed = image.compress(quality=JPEG_QUALITY)
    if hasattr(compressed, "bytearray"):
        return compressed.bytearray()
    return bytes(compressed)


def send_all(client, data):
    view = memoryview(data)
    sent = 0
    while sent < len(view):
        count = client.send(view[sent:])
        if count is None:
            return
        if count <= 0:
            raise OSError("socket closed")
        sent += count


def send_index(client, ip_address):
    body = (
        "<html><head><meta charset='utf-8'><title>K230 Camera</title></head>"
        "<body style='margin:0;background:#111;color:#eee;font-family:sans-serif'>"
        "<h3>K230 Camera - %s</h3>"
        "<img src='/stream' style='max-width:100%%;height:auto'>"
        "</body></html>"
    ) % ip_address

    header = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Cache-Control: no-store\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n"
    ) % len(body)

    send_all(client, header.encode())
    send_all(client, body.encode())


def stream_frames(client, camera):
    header = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
    )
    send_all(client, header.encode())

    frame_interval_ms = 1000 // STREAM_FPS
    while True:
        frame_start = time.ticks_ms()
        image = camera.snapshot()

        # Insert ball detection on this same image before JPEG compression.
        jpeg = jpeg_bytes(image)
        part_header = (
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %d\r\n\r\n"
        ) % len(jpeg)

        send_all(client, part_header.encode())
        send_all(client, jpeg)
        send_all(client, b"\r\n")

        elapsed = time.ticks_diff(time.ticks_ms(), frame_start)
        if elapsed < frame_interval_ms:
            time.sleep_ms(frame_interval_ms - elapsed)
        gc.collect()


def serve(camera, ip_address):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", HTTP_PORT))
    server.listen(1)

    print("Open on PC: http://%s:%d/" % (ip_address, HTTP_PORT))
    print("Stream URL: http://%s:%d/stream" % (ip_address, HTTP_PORT))

    while True:
        client = None
        try:
            client, address = server.accept()
            print("Client:", address)
            request = client.recv(512)
            if b"GET /stream" in request:
                stream_frames(client, camera)
            else:
                send_index(client, ip_address)
        except Exception as error:
            print("Client disconnected:", error)
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
            gc.collect()


wifi = None
camera = None

try:
    wifi, ip = connect_wifi()
    camera = init_camera()
    serve(camera, ip)
finally:
    if camera is not None:
        try:
            camera.stop()
        except Exception:
            pass
    MediaManager.deinit()

