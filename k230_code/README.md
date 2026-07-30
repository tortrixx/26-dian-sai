# K230 程序目录

## 主程序：`k230_yolo.py`

YOLO11n NPU 钢球检测 + WiFi 图传。当前主力方案。

**部署方式**：在 CanMV IDE 中打开并运行，或复制到 `/sdcard/app/k230_yolo.py`。

### 管道架构

```
GC2093 摄像头
  ├─ CHN_0: RGB565 640×480 → JPEG Q50 640×240 pipe crop → K23V/TCP → PC 接收端
  └─ CHN_2: RGBP888 640×480 → AI2D → YOLO11n NPU 320×320 → 钢球坐标
                                                                    ↓
                                                              UART → MSPM0
```

### 关键配置

| 参数 | 值 | 说明 |
|------|-----|------|
| MODEL_PATH | `/sdcard/kmodel/yolo11n_det_320.kmodel` | Laoguigui2 钢球模型（1类） |
| labels | `{0: 'steel'}` | dict 格式，1 类 |
| RGB888P_SIZE | `[640, 480]` | AI 输入分辨率（全帧） |
| CONF_THRESH | `0.35` | 检测置信度阈值 |
| WIFI_SSID / WIFI_PASS | 见代码 | 需改为实际热点 |
| PC_IP / PC_PORT | `192.168.137.1` / `8888` | PC 接收端地址 |
| PIPE_VIDEO_CROP | `(0, 120, 640, 240)` | 图传 ROI（管子区域） |
| STREAM_PROFILE | `"pipe_detail"` | 640×240 Q50 @6fps 目标 |

### 性能

| 指标 | 数值 |
|------|------|
| 总循环帧率 | ~22 FPS |
| NPU 推理 | ~30 ms/帧 |
| JPEG 编码 | ~8 ms/帧 |
| 图传输出 | ~5-6 FPS, ~40-80 KB/s |
| 检测延迟（含 tracker） | ~1帧 |

### 已知问题与调试

- **buf_init 错误**（Yahboom v1.4.3）：传感器初始化必须 `set_framesize` 在 `set_pixformat` 之前
- **IDE interrupt**：脚本运行时不要点击 CanMV IDE 界面，会中断 MicroPython 执行
- **PC 无画面**：确认 K23V 协议长度字段 4 字节 BE（已修复），PC 接收端先于 K230 启动

## 备用程序：`k230_final.py`

传统 Blob 钢球识别（motion-based 帧差分）。不需要 NPU 模型，纯 CPU。

## 目录结构

```
k230_code/
├── k230_yolo.py          # 主力：YOLO NPU 检测 + WiFi 推流
├── k230_calibrate.py     # 标定工具：含 WiFi 推流 + ROI 过滤
├── k230_final.py         # 备用：motion-based 检测器
├── libs/                  # K230 SDK 库（YOLO, AI2D, AIBase, Utils, PipeLine）
├── diagnostics/           # 诊断脚本（sensor 测试、YOLO 调试）
└── README.md
```

## 修改约束

1. 图传失败不能阻塞 `send_ball()` 或视觉循环
2. PC/Wi‑Fi 只能显示和录像，不得向车端回传控制信息
3. 修改相机参数、ROI 或标定前先保存当前配置与性能日志
4. 最终装车后必须重做五点标定（见 `文档/钢球视觉标定操作.md`）
5. 传感器初始化顺序不可改：`set_framesize` → `set_pixformat`
