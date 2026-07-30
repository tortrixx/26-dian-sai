# 文档总览、交接与开发日志

> 本文是后续 agent / 队友的第一阅读入口。记录 2026-07-29 ~ 2026-07-31 已验证事实、关键决策和待完成工作。
> 部署入口：`k230_code/k230_yolo.py`（YOLO NPU，主力）；`k230_final.py`（传统 Blob 备选）。

## 1. 当前最终结论（2026-07-31 更新）

2026-07-31 完成视觉标定、图传稳定性修复和 MSPM0 控制固件编写。MSPM0 固件尚未编译和联调。

| 项目 | 最终值 | 状态 |
| --- | --- | --- |
| 视觉方案 | YOLO11n NPU（Laoguigui2 模型，1 类，320×320 输入） | ✅ |
| pixel→cm 标定 | ZERO_X_PX=345.0, PX_PER_CM=20.1 | ✅ 五点实机 |
| K230 循环帧率 | ~26 FPS（含 NPU + JPEG + WiFi） | ✅ 日志实测 |
| NPU 推理 | ~30 ms/帧 | ✅ |
| 图传 | K23V/TCP，JPEG Q50，640×240 pipe crop，~5 FPS | ✅ 断线问题已修复 |
| PC 接收 | `pc_receiver/pc_receiver.py`，K23V，按 r 录像 | ✅ |
| MPU-6050 | I²C PB2/PB3，WHO_AM_I=0x68，互补滤波模块已编写 | ⚠️ 未接入当前方案 |
| MSPM0 固件 | `ti_control/msp_control.c`，纯视觉 PD，100Hz | 📝 已编写，待编译 |
| 硬件装配 | 摆管、MG996、相机、供电 | ✅ |

### 方案架构

```
GC2093 摄像头
  ├─ CHN_0: RGB565 640×480 → JPEG Q50 640×240 pipe crop → K23V/TCP WiFi → PC 接收端 ✅
  └─ CHN_2: RGBP888 640×480 → AI2D letterbox → YOLO11n NPU 320×320 → 钢球坐标
                                                                             ↓
                                                                      pixel_to_cm()
                                                                             ↓
                                                                       UART → MSPM0
K230 UART IO9(TX) → MSPM0 PA9(RX)  AA 55 协议，x_cm_x100
MSPM0 PA6(TIMG0) → MG996 信号线  50Hz PWM
PC 端仅接收显示 + 录屏                                             ✅ 合规（不回车）
```

## 2. 关键教训（五个已知问题）

### 已修复

1. **buf_init 错误**：传感器初始化必须先 `set_framesize` 再 `set_pixformat`（Yahboom v1.4.3 固件 bug）
2. **模型输出通道**：1 类模型 labels 必须用 dict `{0: 'steel'}`，不能用 80 类 list
3. **K23V 协议长度**：4 字节 BE（匹配接收端 `struct.unpack(">I")`），曾误用 3 字节
4. **AI 通道分辨率**：640×480 全帧（非 16:9 的 640×360）
5. **图传频繁断连**（2026-07-31 修复）：K230 非阻塞 socket 在 TCP 缓冲区满时返回 0 或 OSError(EAGAIN)，旧代码直接当断线。修复为 512 字节块 + 连续 60 次失败才断线

### 当前已知风险

- 标定参数 ZERO_X_PX/PX_PER_CM 依赖当前相机位置；若相机移动必须重新标定
- 赛场同频 WiFi 干扰（5.8G 图传模块作为备选未测试）
- MSPM0 固件未经编译验证（SysConfig 生成和编译可能需调整）

## 3. 当前可部署配置

### 主力：`k230_code/k230_yolo.py`

```text
MODEL_PATH           = /sdcard/kmodel/yolo11n_det_320.kmodel
MODEL_LABELS         = {0: 'steel'}
RGB888P_SIZE         = [640, 480]
ZERO_X_PX            = 345.0        ← 2026-07-31 五点标定
PX_PER_CM            = 20.1         ← 2026-07-31 五点标定
CONF_THRESH          = 0.35
WIFI                 = test / 90z5M92#
PC_IP:PC_PORT        = 192.168.137.1:8888
PIPE_VIDEO_CROP      = (0, 120, 640, 240)
STREAM_PROFILE       = pipe_detail  # JPEG Q50 @ ~6fps
UART                 = UART1, IO9 TX / IO10 RX, 115200 8N1
```

### 备用：`k230_code/k230_final.py`（传统 Blob 视觉）

Wi‑Fi 仅用于赛外实时显示与录像；闭环唯一数据链是 K230→MSPM0 的车内有线 UART。PC 不得回传任何控制信息。

## 4. 开发历程

### 第一阶段（2026-07-29）：传统视觉 + 图传

- 自适应亮/暗 Blob、面积/形状/圆度门限、α-β 跟踪
- 图传从 QVGA 升级到 640×240 管子带 ROI、JPEG Q80
- 性能优化：极性快速路径，Loop 25.7-26.7 FPS，图传 6.8-8.2 FPS

### 第二阶段（2026-07-30）：YOLO NPU 替代传统视觉

- 引入 Laoguigui2 yolo11n_det_320.kmodel（钢球专用 1 类检测器）
- 弃用 PipeLine.create()（Yahboom v1.4.3 卡死），直接用 Sensor API
- 修复 buf_init、K23V 长度、模型后处理越界、AI 通道分辨率四个 bug
- CONF_THRESH 从 0.5 降至 0.35，解决漏检
- 完成 MPU-6050 独立 I²C 点亮（PB2/PB3，WHO_AM_I=0x68）

### 第三阶段（2026-07-31）：标定、图传修复、MSPM0 固件

- 五点 pixel→cm 标定（ZERO_X_PX=345.0, PX_PER_CM=20.1）
- 图传断连问题修复（非阻塞 socket 容错）
- 编写 MSPM0 完整控制固件（ti_control/）——纯视觉 PD，无需 IMU
- 编写 MPU-6050 互补滤波 + 姿态估计模块（备用）
- K230 cx 像素坐标加入状态日志，编写标定工具 k230_calibrate.py

## 5. 下一个 agent 的接手顺序

### 第一步：先读

1. `CLAUDE.md`（硬件约束 + YOLO 管道 + 已知 bug）
2. 本文件
3. `后续TODO清单.md`
4. `../README.md`
5. `../ti_control/README.md`（MSPM0 固件详细说明）
6. `控制接口骨架.md`（UART 协议细节）

### 第二步：MSPM0 编译与无球联调（P0）

1. 运行 `ti_control/build.ps1` 编译固件
2. 烧录 `msp_control.out`
3. 按 `ti_control/README.md` 完成接线
4. 无球校准：中位脉宽、±8°限位、方向
5. 故障保护测试：断 UART / 坏帧 → 200ms 回中位

### 第三步：闭环调参（P1）

1. 视觉联调确认（K230→MSPM0 UART 数据正确）
2. PD 调参：Kp→Kd 顺序，Ki=0
3. 第三题测试：O→+5cm→-5cm，≤5s，误差≤1cm

### 第四步：后续任务

4. 动态 AB
5. 整圈中心、整圈指定点

## 6. 不要做的事

- 不要使用 PC 图像处理回传、摄像头循迹或管壁传感器/色标
- 不要在 K230 代码中 `set_pixformat` 在 `set_framesize` 之前
- 不要将 YOLO labels 写成 80 类 list
- 不要修改 K23V 协议长度字段字节数（4 字节 BE）
- 不要启用 `h264_hw`（板卡不兼容）
- 不要强制结束 PC 接收端进程（MP4 索引不完整）
- 不要在相机重新固定前使用已有标定参数做闭环

## 7. 运行与记录

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

每次实机测试至少保存：启动日志、连续 5 行性能日志、PC FPS、截图/录像、改动过的参数。

## 8. 文档索引

| 目的 | 文件 |
| --- | --- |
| 硬件约束 + YOLO 管道 + 已知 bug | `../CLAUDE.md` |
| 项目基线 + 快速启动 | `../README.md` |
| K230 部署指南 + 性能指标 | `../k230_code/README.md` |
| 当前系统事实、性能、历程与接手操作 | **本文件** |
| 任务分解（含已完成项） | `后续TODO清单.md` |
| 机械/电源/MSPM0 无球联调 | `下一阶段装配与联调清单.md` |
| MSPM0 固件接线/校准/命令 | `../ti_control/README.md` |
| K230 热点图传操作 | `K230热点图传操作手册.md` |
| 五点标定步骤 | `钢球视觉标定操作.md` |
| UART / PD / 状态机约定 | `控制接口骨架.md`、`../ti_reference/firmware_state_machine_skeleton.c` |
| MPU-6050 接线、构建、姿态模块 | `MPU6050独立测试与接入指引.md`、`../ti_mpu6050_test/` |
| 可量化验收结果 | `测试记录表.csv` |
