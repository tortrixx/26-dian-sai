# K230 程序目录

## 比赛部署

只部署 `k230_final.py` 到 K230 的 `/sdcard/main.py`。

它包含：传统 Blob 钢球识别、厘米坐标 UART 输出、PC TCP 图传、断线隔离及性能日志。当前已验证配置为：

```text
640×480 全视场采集
  -> 320×240 内部视觉与 UART
  -> 320×240 / Q70 / 8 FPS PC 图传
```

## 非部署脚本

下列脚本仅保留为历史排障或探索记录，其他 agent 不应替换比赛入口：

- `k230_h264_dual_stream_probe.py`：YUV/VENC 双通道探针。当前 Yahboom CanMV v1.4.3 在 YUV `set_framesize()` 触发 `buf_init` 异常，**不可用于当前固件**。
- `legacy_diagnostics/`：早期摄像头、网络、MJPEG、YOLO、OpenMV 模板和 IDE 诊断脚本的归档目录；不用于部署。

## 修改约束

1. 图传失败不能阻塞 `send_ball()` 或视觉循环。
2. PC/Wi-Fi 只能显示和录像，不能向车回传平衡控制信息。
3. 调整 `VISION_PROFILE`、`ZERO_X_PX`、`PX_PER_CM`、`PIPE_ROI` 前，先保存当前配置和测试日志。
4. 真实钢球到位后必须完成五点标定，才可接入 Ti 驱动摆管。
