# MSPM0 控制固件 —— 纯视觉 bang-bang，无 IMU

架构：K230 YOLO11n NPU → UART2 PB16 (AA 55) → MSPM0 → 舵机 PA8 (软件 PWM)

## 硬件接线

| MSPM0 LP-G3507 | 目标 | 说明 |
|---|---|---|
| `PB16` (UART2 RX) | K230 `IO9` (TX) | 钢球位置 AA 55 帧 |
| `PA21` (UART2 TX) | K230 `IO10` (RX) | 可选调试回传 |
| `PA8` (GPIO) | MG996 信号线 | 软件 PWM 50Hz, 1000-2000µs |
| `PB2/PB3` (I2C1) | OLED SSD1306 SCL/SDA | 菜单显示 |
| `PB15/17/18/19` | 按键 S1/S2/S3/S4 | 菜单导航 |
| `GND` | K230 GND + MG996 GND | 必须共地！ |

**MG996 供电**：独立 5V/>2A 电源。仅 GND 与 MSPM0 共地，正极不接 MSPM0。

## 构建

```powershell
cd ti_control
Set-ExecutionPolicy -Scope Process Bypass
.\build.ps1
```

产出 `msp_control.out`（~16 KB flash / ~1.5 KB RAM）。

## 烧录

```powershell
C:\ti\ccs2100\ccs\ccs_base\DebugServer\bin\DSLite.exe flash `
  -c targetConfigs\MSPM0G3507.ccxml `
  -l msp_control.out
```

或用 CCS Theia 通过 `targetConfigs/MSPM0G3507.ccxml` 烧录。

## 操作方式

OLED 菜单 + 4 按键：

| 按键 | 功能 |
|------|------|
| S1 | 在菜单项间切换 |
| S2 | 进入选中功能 |
| S3 | 停止当前功能 |
| S4 | 返回主菜单 |

`Static Ball` → S2 → 自动执行：+5cm → -5cm → 保持（±0.2cm 死区）

## AA 55 视觉帧协议

K230 → MSPM0，115200 8N1。

```
Byte  0:   0xAA          帧头 0
Byte  1:   0x55          帧头 1
Byte  2:   length        负载长度 + 2 (= 8 for vision)
Byte  3:   type          消息类型 (0x01 = vision target)
Byte  4:   sequence      帧序号 0-255
Byte  5:   flags         bit0=valid, bit1=tracked
Byte  6-7: xCmX100       钢球 X 坐标 (cm × 100, int16 LE)
Byte  8-9: yOffsetPx     钢球 Y 偏移 (像素, int16 LE)
Byte 10:   quality       检测置信度 0-255
Byte 11:   checksum      字节 2-10 的和 (mod 256)
```

最小帧长 = 2(头) + 1(长度) + 1(类型) + 1(序号) + 6(负载) + 1(校验) = 12 字节。

## 静态滚球控制（任务 3）

| 参数 | 值 | 说明 |
|------|-----|------|
| 控制周期 | 10ms | StaticBall_Task 轮询 |
| 移动倾角 | ±12° | 去目标位置时 |
| 保持倾角 | ±8° | 在目标位置死区内 |
| 死区 | ±0.2cm | 目标 ± 死区内倾角=0 |
| 到达判定 | ±0.5cm | 距目标 0.5cm 内算到达 |
| 视觉超时 | 200ms | 超时脱开舵机 |
| 连续无效帧 | 3 帧 | 超过脱开舵机 |
| 最小置信度 | 1 | quality >= 1 才接受 |

相序：WAIT_VISION → TO_POS(+5cm) → TO_NEG(-5cm) → HOLD_NEG(保持)

## 舵机控制

| 参数 | 值 |
|------|-----|
| 引脚 | PA8 (IOMUX_PINCM19)，软件 PWM |
| 频率 | 50Hz (20ms 周期) |
| 脉冲范围 | 1000µs (0°) - 2000µs (180°) |
| 中位 | 1500µs (90°) |
| 方向 | SERVO_DIRECTION = -1.0（翻转倾角方向） |

## 安全保护

| 故障 | 行为 |
|------|------|
| UART 200ms 无有效帧 | 舵机脱开 |
| 连续 3 帧无效 | 舵机脱开 |
| 舵机角度 | 软件限制 0-180° |
| 舵机脱开 | 引脚拉低，无 PWM 脉冲 |

## 首次上电步骤

### 1. 无球空载，确认舵机安全
- MG996 不接连杆
- 烧录固件，进入 `Static Ball` 菜单
- 舵机应保持不动（无有效视觉帧 → 脱开）

### 2. 标定中位
- 连接连杆机构
- 修改 `STATIC_BALL_SERVO_NEUTRAL_DEG` 使摆杆水平
- 重新编译烧录

### 3. 标定方向
- 观察球滚动方向
- 如果反了，修改 `STATIC_BALL_SERVO_DIRECTION` 为 `1.0f`

### 4. 视觉标定
- 相机固定后运行 `k230_code/k230_yolo.py`
- 用 `k230_code/k230_calibrate.py` 做五点标定
- 更新 `ZERO_X_PX` 和 `PX_PER_CM`

## 视觉标定（pixel → cm）

五点在管子上标记 +5, 0, -5 cm 位置：
1. 依次放钢球，记录 YOLO 输出的 cx 像素值
2. `PX_PER_CM = (cx_+5 - cx_-5) / 10.0`
3. `ZERO_X_PX = cx_0cm`
4. 更新 `k230_yolo.py` 中的值

## 架构说明

- **SysConfig**：仅配置 I2C1 (OLED)。其余外设全用 `DL_xxx` driverlib API 直驱
- **无 RTOS**：裸机主循环，SysTick 1KHz 系统时钟
- **无 IMU**：当前纯视觉控制。MPU-6050 驱动在 `ti_mpu6050_test/` 待集成
