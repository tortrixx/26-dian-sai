# K230 程序目录

## 比赛部署

只将 `k230_final.py` 烧录为 K230 的 `/sdcard/main.py`。它包含传统 Blob 钢球识别、厘米坐标 UART 输出、PC TCP 图传、断连隔离和性能日志。

当前已实测配置：

```text
640×480 全视场采集
  -> 320×240 内部传统视觉与 UART
  -> 原始帧 640×240 管子 ROI / JPEG Q80 / 目标 8 FPS 图传
```

实测细节图传下：K230 `Loop 25.7–26.7 FPS`、Video `6.8–8.2 FPS`。参数、标定和接手顺序见 [完整交接](../文档/2026-07-29_完整交接与开发日志.md)。

## 非部署脚本

- `k230_h264_dual_stream_probe.py`：历史 VENC 探针。当前 Yahboom CanMV v1.4.3 会报 `buf_init`，不可部署；运行后需要软重启。
- `legacy_diagnostics/`：历史相机、网络、MJPEG、YOLO、OpenMV 模板与 IDE 诊断归档。不得替代 `k230_final.py`。

## 修改约束

1. 图传失败不能阻塞 `send_ball()` 或视觉循环。
2. PC/Wi‑Fi 只能显示和录像，不得向车端回传平衡控制信息。
3. 修改相机、`VISION_PROFILE`、ROI 或标定参数前，先保存当前配置与性能日志；最终装车后必须重做五点标定。
4. `TRACK_POLARITY_FAST_PATH=True` 是已验证的性能优化；若最终装车的反光环境造成误检，可临时关闭它进行定位，不要直接删掉回退双极性搜索。
