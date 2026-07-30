# 2026 电赛 H 题：车载平衡滚球控制系统

## 当前部署入口（2026-07-31 更新）

| 目标 | 文件 | 状态 |
| --- | --- | --- |
| K230 YOLO 视觉、UART、图传 | `k230_code/k230_yolo.py` | ✅ 主力入口，标定已完成 |
| K230 标定工具 | `k230_code/k230_calibrate.py` | ✅ 含 WiFi 推流 + ROI 过滤 |
| PC 图传接收与录像 | `pc_receiver/pc_receiver.py` | ✅ K23V 协议，按 r 录像 |
| MSPM0 控制固件 | `ti_control/msp_control.c` | 📝 已编写，**待编译联调** |
| MSPM0 SysConfig | `ti_control/msp_control.syscfg` | 📝 待 SysConfig CLI 生成 |
| Ti 控制逻辑参考 | `ti_reference/firmware_state_machine_skeleton.c` | 参考（非实际工程） |
| MPU-6050 点亮 + 姿态估计 | `ti_mpu6050_test/` | ✅ I²C 通过，姿态模块待接入 |
| K230↔MSPM0 协议参考 | `k230_libs/k230_mspm0_uart_protocol.py` | ✅ |
| 串口文件传输 | `tools/transfer_to_k230.py` | ✅ |

## 当前基线（2026-07-31）

### 视觉标定（已实机验证）

```
ZERO_X_PX = 345.0    # 球在管子 0cm 处 YOLO 检测的像素 x
PX_PER_CM = 20.1     # 每厘米约 20 个像素
```

五点标定数据（2026-07-31，最终相机位置）：

| 物理位置 cm | -10 | -5 | 0 | +5 | +10 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `cx` 像素 | 141.5 | 248.5 | 333.5 | 450.5 | 542.5 |

### K230 视觉

- YOLO11n NPU（Laoguigui2 模型，1 类 steel，320×320 输入）
- CHN_2 RGBP888 640×480 → AI2D → NPU 推理，~30ms
- 总循环 ~26 FPS（含 NPU + JPEG + WiFi）
- 置信度阈值 0.35
- 图传：CHN_0 RGB565 → JPEG Q50 640×240 pipe crop → K23V/TCP → ~5-6 FPS
- α-β tracker 内置

### 图传稳定性修复（2026-07-31）

K230 非阻塞 socket 在 TCP 缓冲区满时会返回 0 或抛 OSError(EAGAIN)，旧代码直接当作断线处理导致频繁重连。修复：
- 发送块大小 1400→512 字节
- 单次 flush 最多 4 次 send 尝试
- 连续 60 次失败（~3 秒）才判定为真正断线
- 重连冷却 2s→3s

### MSPM0 控制固件（已编写，待编译）

`ti_control/msp_control.c`：
- 纯视觉方案，不需要 MPU-6050
- UART1 RX (PA9) 接收 K230 AA 55 帧（状态机解析）
- TIMG0 PWM (PA6) 50Hz MG996 舵机输出
- TIMG4 100Hz 控制中断
- PD 控制器：Kp=0.80, Ki=0, Kd=0.15（初始值，待实车调参）
- 安全保护：视觉 200ms 超时→限速回中位、±8°摆角限制、边缘救球
- 调试串口 (UART0) 命令：m0-m5 切模式、t+5.0 设目标、pk/dk/ik 在线调参

### 硬件状态

- 摆管左铰点、MG996 连杆、相机支架：✅ 已装配
- MPU-6050：I²C 点亮完成，**未接入当前控制方案**（用户反馈不可用）
- MSPM0-K230-MG996 接线：**待接通**

## 快速启动

```powershell
# 1. PC 接收端（先启动）
cd pc_receiver
python pc_receiver.py

# 2. K230（CanMV IDE 中打开运行）
# 文件：k230_code/k230_yolo.py
# 确保 K230 热点 test/90z5M92# 已开启，PC IP 192.168.137.1

# 3. MSPM0 编译（待执行）
cd ti_control
Set-ExecutionPolicy -Scope Process Bypass
.\build.ps1
# 产出 msp_control.out，用 CCS 或 DSLite 烧录
```

## 目录约定

| 目录 | 内容 |
|------|------|
| `k230_code/` | K230 部署入口、YOLO 库、标定工具、诊断脚本 |
| `k230_libs/` | UART 协议参考 |
| `pc_receiver/` | PC 图传接收、显示与录像 |
| `ti_control/` | **MSPM0 完整控制固件（待编译联调）** |
| `ti_reference/` | Ti/MSPM0 控制逻辑参考骨架 |
| `ti_mpu6050_test/` | MPU-6050 独立测试 + 姿态估计模块 |
| `文档/` | 交接、装配、标定、图传和验收资料 |
| `reference_code/` | 队友原始代码 + Laoguigui2 参考 |
| `tools/` | K230 串口文件传输脚本 |

## 文档索引

| 阅读顺序 | 文件 | 内容 |
| --- | --- | --- |
| 1 | `CLAUDE.md` | 硬件约束 + 判题规则 + YOLO 管道已知 bug |
| 2 | `文档/README.md` | 开发历程、关键决策、完整文档索引 |
| 3 | `文档/后续TODO清单.md` | 待完成工作清单 |
| 4 | `文档/下一阶段装配与联调清单.md` | 机械/电源/MSPM0 联调步骤 |
| 5 | `ti_control/README.md` | MSPM0 固件接线、校准、命令参考 |
| 6 | `文档/控制接口骨架.md` | K230↔MSPM0 UART 协议 + PD 约定 |
| 7 | `文档/钢球视觉标定操作.md` | 五点 pixel→cm 标定步骤 |
