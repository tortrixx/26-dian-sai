"""One-time Yahboom / CanMV v1.4.3 dual-channel VENC capability probe.

This script intentionally has no Wi-Fi, UART or ball detector.  It answers one
question only: can this exact firmware run RGB565 snapshots on CH1 while CH0 is
bound to the hardware H.264 encoder?  Run it only after the normal
``k230_final.py`` CPU-JPEG path has started successfully.

Expected successful end marker:
    [PROBE] PASS: CH1 RGB565 + CH0 H.264 are usable together
"""

import time
import uctypes
import media.sensor as sensor_api
from media.media import MediaManager


SENSOR_ID = 2
WIDTH, HEIGHT = 640, 480
VENC_CHN = 0
TEST_MS = 5000


def main():
    sensor = None
    encoder = None
    link = None
    started = False
    try:
        print("[PROBE] Creating Sensor({}, {}x{})".format(SENSOR_ID, WIDTH, HEIGHT))
        sensor = sensor_api.Sensor(id=SENSOR_ID, width=WIDTH, height=HEIGHT, fps=30)
        sensor.reset()

        print("[PROBE] Configuring CH0=YUV420SP for VENC")
        # This Yahboom v1.4.3 build accepts CH0 only through the default API.
        # Passing ``chn=CAM_CHN_ID_0`` reaches a broken buf_init branch.
        sensor.set_pixformat(sensor_api.Sensor.YUV420SP)
        sensor.set_framesize(width=WIDTH, height=HEIGHT)
        print("[PROBE] Configuring CH1=RGB565 for vision")
        sensor.set_pixformat(sensor_api.Sensor.RGB565,
                             chn=sensor_api.CAM_CHN_ID_1)
        sensor.set_framesize(width=WIDTH, height=HEIGHT,
                             chn=sensor_api.CAM_CHN_ID_1)

        # Important for Yahboom CanMV v1.4.3: construct the normal Sensor
        # object first; importing VENC before it can corrupt Sensor init.
        import media.vencoder as venc
        encoder = venc.Encoder()
        encoder.SetOutBufs(VENC_CHN, 6, WIDTH, HEIGHT)
        link = MediaManager.link(
            sensor.bind_info()['src'],
            (venc.VIDEO_ENCODE_MOD_ID, venc.VENC_DEV_ID, VENC_CHN)
        )
        MediaManager.init()

        attr = venc.ChnAttrStr(
            encoder.PAYLOAD_TYPE_H264,
            encoder.H264_PROFILE_BASELINE,
            WIDTH, HEIGHT,
            bit_rate=700,
            gopLen=15,
            src_frame_rate=30,
            dst_frame_rate=15,
        )
        stream = venc.StreamData()
        encoder.Create(VENC_CHN, attr)
        encoder.Start(VENC_CHN)
        started = True
        sensor.run()

        start_ms = time.ticks_ms()
        rgb_frames = 0
        h264_frames = 0
        h264_bytes = 0
        while time.ticks_diff(time.ticks_ms(), start_ms) < TEST_MS:
            rgb = sensor.snapshot(chn=sensor_api.CAM_CHN_ID_1)
            if rgb == -1:
                continue
            rgb_frames += 1

            result = encoder.GetStream(VENC_CHN, stream, timeout=0)
            if result == 0 and stream.pack_cnt > 0:
                try:
                    h264_frames += 1
                    for index in range(stream.pack_cnt):
                        h264_bytes += stream.data_size[index]
                        # Touch the first byte to validate accessible VENC data
                        # without copying a whole stream to the MicroPython heap.
                        if stream.data_size[index] > 0:
                            uctypes.bytearray_at(stream.data[index], 1)
                finally:
                    encoder.ReleaseStream(VENC_CHN, stream)

        elapsed_ms = max(1, time.ticks_diff(time.ticks_ms(), start_ms))
        print("[PROBE] RGB snapshots:{} ({:.1f}fps) H264:{} ({:.1f}fps) {}KB/s".format(
            rgb_frames, rgb_frames * 1000.0 / elapsed_ms,
            h264_frames, h264_frames * 1000.0 / elapsed_ms,
            h264_bytes * 1000.0 / elapsed_ms / 1024.0,
        ))
        if rgb_frames > 10 and h264_frames > 5:
            print("[PROBE] PASS: CH1 RGB565 + CH0 H.264 are usable together")
        else:
            print("[PROBE] FAIL: one channel produced too few frames")
    except Exception as error:
        print("[PROBE] FAIL:", error)
        try:
            import sys
            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except Exception:
                pass
        if encoder is not None and started:
            try:
                encoder.Stop(VENC_CHN)
            except Exception:
                pass
            try:
                encoder.Destroy(VENC_CHN)
            except Exception:
                pass
        link = None
        try:
            MediaManager.deinit()
        except Exception:
            pass
        print("[PROBE] End; soft reboot before running another camera program")


main()
