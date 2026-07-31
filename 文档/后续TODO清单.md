# 后续 TODO 清单

> 最后更新：2026-07-31。MSPM0 固件已编译通过，YOLO+BallTracker 优化完成，等待实机联调。

## 已完成 ✅

- [x] K230 YOLO11n NPU 钢球检测管道（k230_yolo.py，~26 FPS）
- [x] 五点 pixel→cm 标定（ZERO_X_PX=345.0, PX_PER_CM=20.1，最终相机位置实机验证）
- [x] K23V 图传稳定性修复（非阻塞 socket 容错，不再频繁断连）
- [x] PC TCP 接收、显示与录像（pc_receiver.py）
- [x] K230↔MSPM0 UART 协议（AA 55，含校验和、序号、超时）
- [x] 标定工具 k230_calibrate.py（含 WiFi 推流 + ROI 误检过滤）
- [x] MPU-6050 独立 I²C 点亮（WHO_AM_I=0x68, PB2/PB3）
- [x] MPU-6050 姿态估计模块（互补滤波 + 零偏标定，mpu6050.h/c）
- [x] MSPM0 完整控制固件编写 + 编译（ti_control/，15 模块，级联 PID）
- [x] 串口传输工具更新（tools/transfer_to_k230.py + _transfer_one.py + _reset_k230.py）
- [x] BallTracker 状态机（SEARCH→CONFIRM→TRACK→HOLD→LOST，替代 Alpha-Beta）
- [x] Pipe ROI 门控过滤管外 YOLO 误检
- [x] draw_result 禁用消除 REPL UART 阻塞
- [x] 视频 JPEG 编码预分配缓冲区（消除 GC 压力）
- [x] 代码清理：移除冗余 TRACK_MISS_LIMIT，修复 S3→StaticBall_Stop
- [x] 硬件装配（摆管、MG996、相机支架已固定）
- [x] 图传验收：连续推流 60s 无断连，K230 Loop ~26 FPS

## P0：MSPM0 固件编译与无球联调（下一步！）

- [x] **编译 MSPM0 控制固件** ✅
  - 运行 `ti_control/build.ps1`，产出 `msp_control.out` (~14 KB)
  - 用 DSLite 烧录到 LP-MSPM0G3507

- [ ] **接线**
  - K230 IO9 (TX) → MSPM0 PB16 (UART2 RX)
  - K230 IO10 (RX) ← MSPM0 PA21 (UART2 TX, 可选)
  - MSPM0 PA8 (GPIO 软件 PWM) → MG996 信号线
  - MG996 独立大电流电源（>2A），GND 与 MSPM0 GND 共地
  - K230 GND — MSPM0 GND — MG996 电源 GND 三者共地

- [ ] **无球空载联调**
  - 首次上电 MG996 先不接连杆，确认通电不异常
  - 调试串口 (XDS110 UART0) 发送 `m1`，确认舵机输出 1500µs
  - 若摆杆不水平：修改 `SERVO_NEUTRAL_US` 并重新编译
  - 发送 `t+5.0 go`，确认球滚向 +5cm——若反向则改 `SERVO_DIRECTION = -1.0f`
  - 标定 ±8° 对应的脉宽范围，更新 `SERVO_MIN_US` / `SERVO_MAX_US`
  - 调试串口命令参考 `ti_control/README.md`

- [ ] **视觉联调**
  - 启动 K230 运行 k230_yolo.py，PC 端开接收
  - MSPM0 调试串口发送 `m2`（STATIC_BALL 模式）
  - 球放在管子中心，MSPM0 串口应打印 `ball=~0cm`
  - 球放在 ±5cm，验证读数一致

- [ ] **故障保护测试**
  - 分别测试：K230 停止、UART 拔线、校验和错误
  - 每项要求：200ms 内按限速 20°/s 回中位
  - 图传断连不得影响 MSPM0 行为
  - 每项保存回中位时间与视频

## P1：PD 调参与任务联调

- [ ] **静态滚球（第三题）**
  - `Ki=0`；从 Kp=0.40 开始每次 +0.20，出现过冲后加 Kd
  - `-5、0、+5 cm` 各 10 次：5s 内稳定，最终误差 ≤0.8cm
  - 第三题流程：`t+5.0 go` → 稳定 → `t-5.0` → 稳定
  - 交付：最终 Kp/Kd、成功率、最大误差、视频

- [ ] **动态 AB**
  - 静态验收通过才开放
  - 10 次至少 8 次全程球误差 ≤1cm
  - 失败时先降车速/摆幅

- [ ] **整圈与指定点**
  - 顺序：DYN_LAP_CENTER 通过 → DYN_LAP_TARGET

## 不要做

- [ ] 不要在 PC 端做图像处理回传车端（违规）
- [ ] 不要用摄像头循迹（只能用红外）
- [ ] 不要在管壁添加传感器/色标（违规）
- [ ] 不要在真实标定完成前接通舵机闭环（已完成标定 ✅）
- [ ] 不要将 `set_pixformat` 放在 `set_framesize` 之前（Yahboom v1.4.3 bug）
- [ ] 不要修改 K23V 协议长度字段（4 字节 BE）

## 下一位 agent 接手最小信息包

1. 本文件和 `../README.md`、`CLAUDE.md`
2. `ti_control/README.md`（MSPM0 固件接线和命令表格）
3. K230 标定参数：`ZERO_X_PX=345.0, PX_PER_CM=20.1`
4. 当前 K230 入口：`k230_code/k230_yolo.py`
5. MSPM0 固件入口：`ti_control/empty.c` → `app.c`（15 模块级联 PID）
