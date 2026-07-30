# 文档总览、交接与开发日志

> 本文是后续 agent / 队友的第一阅读入口。它记录 2026-07-29 ~ 2026-07-30 已验证事实、关键决策和未完成工作的唯一推荐顺序。部署入口现在是 `k230_code/k230_yolo.py`（YOLO NPU）；`k230_final.py` 为传统视觉备选。

## 1. 当前最终结论（2026-07-30 更新）

2026-07-30 完成 YOLO11n NPU 钢球检测管道调通，替换了原有的传统 Blob 视觉方案。同时完成原 LP-MSPM0G3507 上的 MPU-6050 独立点亮。

| 项目 | 最终值 | 实机证据 |
| --- | --- | --- |
| 视觉方案 | YOLO11n NPU（Laoguigui2 模型，1 类，320×320 输入） | 钢球检测 conf 0.56–0.82，稳定输出坐标 |
| K230 循环帧率 | ~22 FPS（含 NPU 推理 + JPEG 编码 + WiFi 推流） | 日志实测 |
| NPU 推理 | ~30 ms/帧 | YOLO 代码计时 |
| 图传 | CHN_0 RGB565 → JPEG Q50 @ 640×240 pipe crop → K23V/TCP | ~5-6 FPS, ~40-80 KB/s |
| 控制 | CHN_2 RGBP888 640×480 → AI2D → YOLO NPU → UART → MSPM0 | AA 55 协议，含 tracker |
| PC 接收 | `pc_receiver/pc_receiver.py`，K23V 协议，OpenCV 显示 + 录像 | 已验证画面正常，按 r 录像、q 退出 |
| MPU-6050 | I²C `PB2/PB3`、地址 `0x68`、`WHO_AM_I=0x68`、连续六轴读取 | 已实机通过 |

### 方案架构

```
GC2093 摄像头
  ├─ CHN_0: RGB565 640×480 → JPEG Q50 640×240 pipe crop → K23V/TCP WiFi → PC 接收端 ✅ 合规
  └─ CHN_2: RGBP888 640×480 → AI2D letterbox → YOLO11n NPU 320×320 → 钢球坐标
                                                                              ↓
                                                                        UART → MSPM0
PC 端仅接收显示 + 录屏                                                        ✅ 合规（不回车）
```

### 关键教训（四个已修复的 bug）

1. **buf_init 错误**：传感器初始化必须先 `set_framesize` 再 `set_pixformat`（Yahboom v1.4.3 固件 bug）
2. **模型输出通道**：1 类模型输出 shape `(1,5,2100)`，labels 必须用 dict `{0: 'steel'}`；用 80 类 list 导致 postprocess 越界 → 垃圾分数
3. **K23V 协议长度**：4 字节 BE（匹配接收端 `struct.unpack(">I")`）；曾误用 3 字节 → 帧大小膨胀 256x → PC 永远无画面
4. **AI 通道分辨率**：640×480 全帧（非 16:9 的 640×360），避免垂直裁剪切掉管子区域钢球

详见 `../CLAUDE.md` 和 memory `yolo-pipeline`。

## 2. 当前可部署配置

### 主力：`k230_code/k230_yolo.py`（YOLO NPU）

```text
MODEL_PATH           = /sdcard/kmodel/yolo11n_det_320.kmodel
MODEL_LABELS         = {0: 'steel'}      # dict，1 类
RGB888P_SIZE         = [640, 480]        # AI 通道全帧
CONF_THRESH          = 0.35
WIFI                 = test / 90z5M92#
PC_IP:PC_PORT        = 192.168.137.1:8888
PIPE_VIDEO_CROP      = (0, 120, 640, 240)
STREAM_PROFILE       = pipe_detail       # JPEG Q50 @ ~6fps
UART                 = UART1, IO9 TX / IO10 RX, 115200 8N1
```

### 备用：`k230_code/k230_final.py`（传统 Blob）

```text
VISION_PROFILE       = fast_qvga
capture              = 640×480 → vision 320×240
STREAM_PROFILE       = pipe_detail
video                = JPEG Q80, target 8 FPS
```

Wi‑Fi 仅用于赛外实时显示与录像；闭环唯一数据链是 K230→Ti 的车内有线 UART。PC 不得回传任何控制信息。

## 3. 开发历程与关键决策

### 第一阶段（2026-07-29）：传统视觉 + 图传

1. **早期低清图传的问题**：原方案先把全视场缩小到 320×240 做视觉，再将小图 JPEG 推送到 PC。即使提高质量或放大窗口，管口刻度也已被缩小丢失。
2. **不采用 YOLO（当时）**：钢球是普通未喷涂钢球；采用 L 亮度自适应亮/暗 Blob、面积、长宽比、圆度、位置预测和 α-β 跟踪。
3. **图传质量提升**：保持控制视觉 QVGA 不变；从原始 640×480 帧直接取 640×240 管子带，再 Q80 JPEG 编码。
4. **性能优化**：极性快速路径——稳定跟踪时只先跑对应极性 blob，质量不达标再补跑另一极性。vis 从 ~30 ms 降至 ~16 ms，Loop 回升至 25.7–26.7 FPS。

### 第二阶段（2026-07-30）：YOLO NPU 替代传统视觉

5. **引入 YOLO11n NPU**：用户指出 Laoguigui2 的 yolo11n_det_320.kmodel 是专为钢球训练的 1 类检测器。替换传统 Blob 方案后检测更鲁棒。
6. **弃用 PipeLine**：Yahboom CanMV v1.4.3 固件上 PipeLine.create() 卡死，改用直接 Sensor API。
7. **模型加载时序**：加载模型时 sensor 必须停止（避免 CHN_2 4 帧缓冲溢出）。
8. **置信度调优**：CONF_THRESH=0.5 时漏检严重，降至 0.35 后检测稳定。

## 4. 已验证数据与边界

### 4.1 YOLO 检测性能

- 钢球检测置信度：0.56–0.82（实际运行）
- 检测延迟（含 tracker）：~1 帧
- 总循环帧率：~22 FPS
- pixel-to-cm 参数：ZERO_X_PX=320.0, PX_PER_CM=12.0（**占位值，需装车后实测标定**）

### 4.2 传统视觉台架标定（仅供参考）

内部 320×240 视觉图的五点数据：

| 物理位置 cm | -10 | -5 | 0 | +5 | +10 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `cx` 像素 | 47.19 | 102.57 | 160.47 | 221.28 | 270.67 |

**注意**：以上数据来自传统 Blob 方案的 320×240 视觉图。YOLO 方案使用 640×480 全帧检测，标定参数 `ZERO_X_PX` 和 `PX_PER_CM` 需要以 YOLO 输出坐标重新标定。

### 4.3 尚未验证的项目

- 最终装车的 `-8° / 0° / +8°` 全管段可见性；
- 60 s 连续图传、MP4 回放与图传断连恢复；
- 无球、强反光、短遮挡、轻振动下 YOLO 不误报；
- MSPM0 UART 实际接收、坏帧/超时回中位；
- MG996 中位、摆角限幅、方向、机械死点、独立供电；
- pixel-to-cm 真实标定（当前 ZERO_X_PX/PX_PER_CM 为占位值）；
- `-5 / 0 / +5 cm` 静态 PD、动态 AB、整圈任务。

## 5. 下一个 agent 的推荐接手顺序

### 第一步：先读、先核对

1. 阅读本文件、`后续TODO清单.md`、`下一阶段装配与联调清单.md`。
2. 阅读 `../CLAUDE.md`（硬件约束 + YOLO 管道细节 + 已知 bug）。
3. 执行 `git status --short --branch`，保留现有改动。
4. K230 主力入口为 `k230_code/k230_yolo.py`；`k230_final.py` 为备选。

### 第二步：MPU 角度标定，再做机械、电源、MSPM0 的无球联调（当前主线）

1. 将已点亮的 MPU-6050 刚性固定到摆杆，完成 `-8° / 0° / +8°` 轴向、零偏和互补滤波校准；详见 [MPU6050独立测试与接入指引](MPU6050独立测试与接入指引.md)。
2. 完成左侧铰点摆管、MG996 连杆、端部防落球与相机刚性支架。
3. 舵机使用独立大电流电源；K230、MSPM0、舵机电源只共地，严禁从小板供电端驱动 MG996。
4. 先在无球状态接入 MSPM0：100 Hz 控制任务、50 Hz PWM、舵机中位、正负方向、±8°限幅和角速度限制。
5. 完成停止 K230、拔 UART、I²C 失败、坏校验帧、连续无效帧时 200 ms 内限速回中位的测试；图传断连不得影响 MSPM0 行为。

### 第三步：最终视觉复标与鲁棒性验收

1. 相机最终固定后，先查看 `-8° / 0° / +8°`；若图传裁切到管子，仅改 `PIPE_VIDEO_CROP` 的 `y/height`，保持 640 宽度。
2. 按 `-10/-5/0/+5/+10 cm` 收集 YOLO 检测坐标，更新 `ZERO_X_PX`、`PX_PER_CM`。
3. 在无球、反光、遮挡、轻振动下测试 YOLO 检测鲁棒性。调整 `CONF_THRESH`（当前 0.35）如需要。
4. 先 `Ki=0`，按 `Kp → Kd` 调整静态 `-5/0/+5 cm`；静态通过后才开放动态 AB，再开放整圈。

## 6. 不要做的事

- 不要使用 PC 图像处理回传、摄像头循迹或管壁传感器/色标；它们不符合题目约束。
- 不要在 K230 代码中将 `set_pixformat` 放在 `set_framesize` 之前——会触发 Yahboom v1.4.3 buf_init bug。
- 不要将 YOLO labels 写成 list（如 80 类 COCO）——模型是 1 类，必须用 `{0: 'steel'}`。
- 不要修改 K23V 协议的长度字段字节数（4 字节 BE，与 PC 接收端对齐）。
- 不要启用 `h264_hw` 或运行 `k230_h264_dual_stream_probe.py`；该板卡/固件组合不兼容。
- 不要强制结束 PC 接收端进程。录像必须按 `r` 停止或 `q` 退出，看到 `Recording finalized` 后 MP4 才完整。
- 不要在真实钢球标定完成前接通舵机闭环。

## 7. 运行、记录与交接要求

```powershell
# PC：先启动接收端
python pc_receiver/pc_receiver.py
```

K230 启动后应出现：

```text
[K230] Camera 640x480 OK
[K230] YOLO model loaded from /sdcard/kmodel/yolo11n_det_320.kmodel
[K230] WiFi connected: 192.168.137.x
[K230] Running: YOLO NPU detection -> UART; JPEG video streaming
```

每次实机测试至少保存：启动日志、连续 5 行性能日志、PC FPS、截图/录像、相机位置或接线变化、改动过的参数。把量化结果追加到 `测试记录表.csv`，并在提交信息中说明"为什么改"和"实测结果"。

## 8. 仓库与安全

- 当前仓库：`C:\Users\sznnn\Desktop\26-dian-sai`；分支：`main`；远程：`origin`。
- 今日相关提交：`79c05cf`（YOLO NPU + K23V fix + buf_init fix + docs）、`06bec3c` 等（传统视觉阶段）。
- 热点 SSID/密码已获仓库所有者明确授权，可随当前项目版本管理；仍应避免在无关截图、日志和公开 issue 中重复传播，热点参数变更后要同步更新部署文件。

## 9. 文档索引

| 目的 | 文件 |
| --- | --- |
| 硬件约束 + YOLO 管道 + 已知 bug | `../CLAUDE.md` |
| 项目基线 + 快速启动 | `../README.md` |
| K230 部署指南 + 性能指标 | `../k230_code/README.md` |
| 当前系统事实、性能、历程与接手操作 | **本文件** |
| 任务分解 | `后续TODO清单.md` |
| 机械/电源/MSPM0 无球联调 | `下一阶段装配与联调清单.md` |
| K230 热点图传操作 | `K230热点图传操作手册.md` |
| 五点标定步骤 | `钢球视觉标定操作.md` |
| UART / PD / 状态机约定 | `控制接口骨架.md`、`../ti_reference/firmware_state_machine_skeleton.c` |
| MPU-6050 接线、实机测试、构建与后续接入 | `MPU6050独立测试与接入指引.md`、`../ti_mpu6050_test/` |
| 可量化验收结果 | `测试记录表.csv` |
