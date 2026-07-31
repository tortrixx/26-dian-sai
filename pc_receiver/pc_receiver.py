"""PC receiver for the K230 competition video stream.

Default K230 configuration sends hardware-encoded H.264 as framed TCP packets.
Install once in the same Python environment:

    python -m pip install av opencv-python numpy

The parser also accepts the old ``A5 5A A5 5A | jpeg_size | JPEG`` protocol,
which is useful when K230 is temporarily switched to ``VIDEO_BACKEND=jpeg_cpu``.
"""

import os
import socket
import struct
import threading
import time

import cv2
import numpy as np

try:
    import av
    PYAV_AVAILABLE = True
except ImportError:
    av = None
    PYAV_AVAILABLE = False


# ---------- Configuration ----------
LISTEN_PORT = 8888
RECORD_DIR = r"C:\电赛录制"
# ``pipe_detail`` is 640x240. Integer nearest-neighbour scaling preserves
# one-pixel physical tick marks; the old 1.5x linear resize visibly blurred it.
DISPLAY_SCALE = 2.0
DISPLAY_INTERPOLATION = cv2.INTER_NEAREST
RECORD_FPS = 20  # Match K230 STREAM_PROFILE="pipe_detail" (target ~20 fps).
MAX_RECORD_QUEUE = 4
# Use PyAV for MP4 recording.  The installed OpenCV wheel can decode video but
# has no usable VideoWriter encoder on this PC; PyAV includes a working x264
# encoder.  AVI/MJPG remains a last-resort compatibility fallback.
RECORD_CONTAINER = "mp4"  # "mp4" preferred; change to "avi" only if needed.
RECORD_MP4_CODEC = "libx264"

# K230 v2 framed stream: magic | version | codec | payload_len_be | payload.
STREAM_MAGIC = b"K23V"
STREAM_VERSION = 1
CODEC_JPEG = 1
CODEC_H264 = 2
STREAM_HEADER_BYTES = 10

# Compatibility for the CPU-JPEG sender used in earlier tests.
LEGACY_JPEG_MAGIC = b"\xA5\x5A\xA5\x5A"
MAX_PAYLOAD_BYTES = 600_000
# -----------------------------------


os.makedirs(RECORD_DIR, exist_ok=True)
writer = None
recording = True
frame_buffer = []
buffer_lock = threading.Lock()
recorder_thread = None
recorder_stop_event = threading.Event()
recorder_path = None
display_size = None


class PyAVRecorder:
    """Small synchronous encoder used only from the recorder worker thread."""
    def __init__(self, path, codec_name, width, height):
        self.path = path
        self.container = av.open(path, mode="w", format="mp4")
        self.stream = self.container.add_stream(codec_name, rate=RECORD_FPS)
        self.stream.width = width
        self.stream.height = height
        self.stream.pix_fmt = "yuv420p"  # Plays in Windows players and browsers.

    def write(self, frame):
        video_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
        for packet in self.stream.encode(video_frame):
            self.container.mux(packet)

    def release(self):
        if self.container is None:
            return
        try:
            for packet in self.stream.encode():
                self.container.mux(packet)
        finally:
            self.container.close()
            self.container = None


def _open_writer(path, fourcc, width, height):
    """Return an opened writer, or None without leaving a broken empty file."""
    output = cv2.VideoWriter(
        path, cv2.VideoWriter_fourcc(*fourcc), RECORD_FPS, (width, height)
    )
    if output.isOpened():
        return output
    output.release()
    return None


def get_recorder(width, height):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    if RECORD_CONTAINER.lower() == "mp4":
        mp4_path = os.path.join(RECORD_DIR, f"record_{timestamp}.mp4")
        if PYAV_AVAILABLE:
            try:
                output = PyAVRecorder(mp4_path, RECORD_MP4_CODEC, width, height)
                print(f"[PC] Recording MP4 ({RECORD_MP4_CODEC}/PyAV): {mp4_path}")
                return output
            except Exception as error:
                print(f"[PC] PyAV MP4 recorder unavailable: {error}")
        else:
            print("[PC] PyAV unavailable; MP4 recorder cannot start")
        print("[PC] Falling back to AVI/MJPG")

    avi_path = os.path.join(RECORD_DIR, f"record_{timestamp}.avi")
    output = _open_writer(avi_path, "MJPG", width, height)
    if output is None:
        print(f"[PC] ERROR: cannot create recorder: {avi_path}")
    else:
        print(f"[PC] Recording AVI/MJPG: {avi_path}")
    return output


def recorder_loop(output):
    """Drain all queued frames before closing, so MP4 receives its index."""
    try:
        while True:
            with buffer_lock:
                frame = frame_buffer.pop(0) if frame_buffer else None
                should_stop = recorder_stop_event.is_set() and not frame_buffer
            if frame is not None:
                output.write(frame)
            elif should_stop:
                break
            else:
                time.sleep(0.01)
    except Exception as error:
        print(f"[PC] Recorder write error: {error}")
    finally:
        try:
            output.release()
        except Exception as error:
            print(f"[PC] Recorder finalization error: {error}")


def start_recorder(width, height):
    global writer, recorder_thread, recorder_path
    with buffer_lock:
        frame_buffer.clear()
    writer = get_recorder(width, height)
    if writer is None:
        return False
    recorder_path = getattr(writer, "path", None)
    recorder_stop_event.clear()
    recorder_thread = threading.Thread(target=recorder_loop, args=(writer,), daemon=True)
    recorder_thread.start()
    return True


def finalize_recorder():
    """Finish a file on disconnect, record-off or program exit."""
    global writer, recorder_thread, recorder_path
    if writer is None:
        return
    recorder_stop_event.set()
    if recorder_thread is not None and recorder_thread.is_alive():
        recorder_thread.join(timeout=5.0)
    if recorder_thread is not None and recorder_thread.is_alive():
        print("[PC] WARNING: recorder did not finish within 5 s")
        return
    print(f"[PC] Recording finalized: {recorder_path or 'video file'}")
    writer = None
    recorder_thread = None
    recorder_path = None


class H264Decoder:
    """Incremental Annex-B H.264 decoder backed by PyAV/FFmpeg."""
    def __init__(self):
        self.codec = av.CodecContext.create("h264", "r")

    def decode(self, encoded):
        decoded = []
        # parse() accepts arbitrary NAL boundaries; K230 provides complete
        # access units but SPS/PPS may be prefixed to the first IDR packet.
        for packet in self.codec.parse(encoded):
            for video_frame in self.codec.decode(packet):
                decoded.append(video_frame.to_ndarray(format="bgr24"))
        return decoded


def _first_magic_position(buffer):
    new_pos = buffer.find(STREAM_MAGIC)
    legacy_pos = buffer.find(LEGACY_JPEG_MAGIC)
    positions = [pos for pos in (new_pos, legacy_pos) if pos >= 0]
    return min(positions) if positions else -1


def extract_packets(buffer):
    """Yield complete (codec, payload) packets and retain an incomplete tail."""
    packets = []
    while True:
        magic_pos = _first_magic_position(buffer)
        if magic_pos < 0:
            # Retain the possible beginning of a 4-byte magic sequence.
            return packets, buffer[-3:]
        if magic_pos:
            buffer = buffer[magic_pos:]

        if buffer.startswith(STREAM_MAGIC):
            if len(buffer) < STREAM_HEADER_BYTES:
                return packets, buffer
            version = buffer[4]
            codec = buffer[5]
            size = struct.unpack(">I", bytes(buffer[6:10]))[0]
            if (version != STREAM_VERSION or codec not in (CODEC_JPEG, CODEC_H264)
                    or size < 1 or size > MAX_PAYLOAD_BYTES):
                buffer = buffer[1:]
                continue
            packet_end = STREAM_HEADER_BYTES + size
            if len(buffer) < packet_end:
                return packets, buffer
            packets.append((codec, bytes(buffer[STREAM_HEADER_BYTES:packet_end])))
            buffer = buffer[packet_end:]
            continue

        # Older K230 JPEG packet: magic | size_be | JPEG.
        if len(buffer) < 8:
            return packets, buffer
        size = struct.unpack(">I", bytes(buffer[4:8]))[0]
        if size < 500 or size > MAX_PAYLOAD_BYTES:
            buffer = buffer[1:]
            continue
        packet_end = 8 + size
        if len(buffer) < packet_end:
            return packets, buffer
        packets.append((CODEC_JPEG, bytes(buffer[8:packet_end])))
        buffer = buffer[packet_end:]


def decode_jpeg(payload):
    image_array = np.frombuffer(payload, dtype=np.uint8)
    return cv2.imdecode(image_array, cv2.IMREAD_COLOR)


def display_and_record(frame):
    """Display exactly the newest decoded frame and enqueue it for recording."""
    global writer, recording, display_size
    if frame is None:
        return False

    if recording and writer is None:
        height, width = frame.shape[:2]
        if not start_recorder(width, height):
            recording = False
            print("[PC] Recording disabled: no usable video encoder")

    if recording and writer is not None:
        with buffer_lock:
            # A slow drive must not turn a real-time stream into delayed video.
            while len(frame_buffer) >= MAX_RECORD_QUEUE:
                frame_buffer.pop(0)
            frame_buffer.append(frame)

    display = cv2.resize(frame, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                         interpolation=DISPLAY_INTERPOLATION)
    new_display_size = (display.shape[1], display.shape[0])
    if new_display_size != display_size:
        cv2.resizeWindow("K230", *new_display_size)
        display_size = new_display_size
    status = "REC" if recording else "LIVE"
    cv2.putText(display, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2)
    cv2.imshow("K230", display)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        return True
    if key == ord("r"):
        recording = not recording
        if recording:
            print("[PC] Recording ON")
            writer = None
        else:
            finalize_recorder()
            print("[PC] Recording OFF")
    return False


def main():
    global writer, recording
    print("=" * 50)
    print("  Electric Competition H - K230 PC video receiver")
    print("=" * 50)
    if not PYAV_AVAILABLE:
        print("[PC] PyAV not installed: H.264 frames cannot be decoded.")
        print("[PC] Run: python -m pip install av")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", LISTEN_PORT))
    server.listen(1)

    cv2.namedWindow("K230", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("K230", 960, 540)

    try:
        while True:
            print(f"[PC] Waiting for K230 (port {LISTEN_PORT})...")
            connection, address = server.accept()
            print(f"[PC] Connected: {address}")
            connection.settimeout(3)
            receive_buffer = bytearray()
            h264_decoder = H264Decoder() if PYAV_AVAILABLE else None
            fps_t0 = time.monotonic()
            fps_count = 0
            warned_h264 = False

            try:
                while True:
                    try:
                        chunk = connection.recv(65536)
                        if not chunk:
                            print("[PC] K230 disconnected")
                            break
                        receive_buffer.extend(chunk)
                    except socket.timeout:
                        continue
                    except OSError:
                        print("[PC] K230 disconnected")
                        break

                    packets, receive_buffer = extract_packets(receive_buffer)
                    for codec, payload in packets:
                        if codec == CODEC_JPEG:
                            frames = [decode_jpeg(payload)]
                        elif h264_decoder is not None:
                            try:
                                frames = h264_decoder.decode(payload)
                            except Exception as error:
                                # A reconnect can begin before a clean IDR.  K230
                                # sends SPS/PPS + IDR within one GOP; discard only
                                # this decode attempt, never the TCP connection.
                                print(f"[PC] H.264 decode reset: {error}")
                                h264_decoder = H264Decoder()
                                frames = []
                        else:
                            if not warned_h264:
                                print("[PC] Dropping H.264: install PyAV with `python -m pip install av`")
                                warned_h264 = True
                            frames = []

                        for frame in frames:
                            if display_and_record(frame):
                                return
                            fps_count += 1

                    if fps_count >= 30:
                        elapsed = max(0.001, time.monotonic() - fps_t0)
                        print(f"[PC] FPS:{fps_count / elapsed:.1f}")
                        fps_t0 = time.monotonic()
                        fps_count = 0
            finally:
                try:
                    connection.close()
                except OSError:
                    pass
                # An MP4 is only playable after its final index is written.
                # A later K230 reconnect will start a separate recording file.
                finalize_recorder()
    except KeyboardInterrupt:
        print("\n[PC] Exit")
    finally:
        recording = False
        finalize_recorder()
        server.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
