"""K230 <-> MSPM0 UART protocol for the steel-ball controller.

Frame layout (all multi-byte integers are little-endian)::

    AA 55 | LEN | TYPE | SEQ | FLAGS | X_CM_X100 | Y_OFFSET_PX | QUALITY | SUM

``LEN`` covers ``TYPE + SEQ + payload``.  The vision payload is always six
bytes, so LEN is eight.  ``X_CM_X100`` is the calibrated ball position along
the tube; it is *not* a pixel coordinate.  The MSPM0 timestamps a frame when
it receives it, because the two boards do not share a clock.
"""

try:
    import time
except Exception:
    time = None


PROTO_HEAD_0 = 0xAA
PROTO_HEAD_1 = 0x55
PROTO_MAX_PAYLOAD = 32

MSG_VISION_TARGET = 0x01

VISION_FLAG_VALID = 0x01
VISION_FLAG_TRACKED = 0x02
VISION_PAYLOAD_LEN = 6


def _checksum(buf, start, end):
    value = 0
    for i in range(start, end):
        value = (value + buf[i]) & 0xFF
    return value


def _clamp_int16(value):
    value = int(value)
    if value < -32768:
        return -32768
    if value > 32767:
        return 32767
    return value


def _clamp_u8(value):
    value = int(value)
    if value < 0:
        return 0
    if value > 255:
        return 255
    return value


def _put_i16_le(buf, offset, value):
    value = _clamp_int16(value) & 0xFFFF
    buf[offset] = value & 0xFF
    buf[offset + 1] = (value >> 8) & 0xFF


def _get_i16_le(buf, offset):
    value = buf[offset] | (buf[offset + 1] << 8)
    if value & 0x8000:
        value -= 0x10000
    return value


def build_frame(msg_id, seq, payload):
    payload_len = len(payload)
    if payload_len > PROTO_MAX_PAYLOAD:
        raise ValueError("payload too long")

    length = payload_len + 2
    frame = bytearray(2 + 1 + length + 1)
    frame[0] = PROTO_HEAD_0
    frame[1] = PROTO_HEAD_1
    frame[2] = length
    frame[3] = msg_id & 0xFF
    frame[4] = seq & 0xFF
    for i in range(payload_len):
        frame[5 + i] = payload[i] & 0xFF
    frame[len(frame) - 1] = _checksum(frame, 2, len(frame) - 1)
    return frame


def build_vision_target(seq, valid, x_cm_x100, y_offset_px, quality=0,
                        tracked=False):
    """Build a calibrated ball observation for the MSPM0.

    Args:
        valid: A measured position passed all visual gates.
        x_cm_x100: Ball center position along the tube in 0.01 cm units.
        y_offset_px: Vertical offset from the calibrated pipe band; diagnostics
            only and never used by the Ti control law.
        quality: Candidate confidence in the inclusive range 0..100.
        tracked: ``True`` if this observation came from the predicted local ROI.
    """
    payload = bytearray(VISION_PAYLOAD_LEN)
    flags = 0
    if valid:
        flags |= VISION_FLAG_VALID
    if tracked:
        flags |= VISION_FLAG_TRACKED
    payload[0] = flags
    _put_i16_le(payload, 1, x_cm_x100)
    _put_i16_le(payload, 3, y_offset_px)
    payload[5] = _clamp_u8(quality)
    return build_frame(MSG_VISION_TARGET, seq, payload)


def parse_vision_payload(payload):
    if len(payload) != VISION_PAYLOAD_LEN:
        return None
    return {
        "valid": (payload[0] & VISION_FLAG_VALID) != 0,
        "tracked": (payload[0] & VISION_FLAG_TRACKED) != 0,
        "x_cm_x100": _get_i16_le(payload, 1),
        "y_offset_px": _get_i16_le(payload, 3),
        "quality": payload[5],
    }


class Mspm0UartProtocol:
    def __init__(self, uart):
        self.uart = uart
        self.seq = 0
        self.rx = bytearray()

    def _write(self, frame):
        try:
            return self.uart.write(frame)
        except Exception:
            return self.uart.send(frame)

    def deinit(self):
        try:
            return self.uart.deinit()
        except Exception:
            return None

    def send_vision(self, valid, x_cm_x100, y_offset_px, quality=0,
                    tracked=False):
        frame = build_vision_target(
            self.seq, valid, x_cm_x100, y_offset_px, quality, tracked
        )
        self.seq = (self.seq + 1) & 0xFF
        return self._write(frame)

    def poll(self):
        if self.uart is None:
            return None

        data = None
        try:
            if hasattr(self.uart, "any") and self.uart.any() <= 0:
                return None
            data = self.uart.read()
        except Exception:
            try:
                data = self.uart.read()
            except Exception:
                data = None

        if not data:
            return None

        for b in data:
            self.rx.append(b)

        return self._pop_frame()

    def _pop_frame(self):
        while len(self.rx) >= 4:
            if self.rx[0] != PROTO_HEAD_0 or self.rx[1] != PROTO_HEAD_1:
                del self.rx[0]
                continue

            length = self.rx[2]
            if length < 2 or length > (PROTO_MAX_PAYLOAD + 2):
                del self.rx[0]
                continue

            frame_len = 2 + 1 + length + 1
            if len(self.rx) < frame_len:
                return None

            checksum = _checksum(self.rx, 2, frame_len - 1)
            if checksum != self.rx[frame_len - 1]:
                del self.rx[0]
                continue

            msg_id = self.rx[3]
            seq = self.rx[4]
            payload = bytes(self.rx[5:frame_len - 1])
            del self.rx[:frame_len]
            return msg_id, seq, payload


def sleep_ms(ms):
    if time is not None and hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    elif time is not None:
        time.sleep(ms / 1000.0)
