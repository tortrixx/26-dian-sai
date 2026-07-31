# HANDOFF — 2026 电赛 H 题 车载平衡滚球运动控制系统

> 写给下一个接手开发的 agent/人。包含全部模块、数据流、协议、当前状态。

---

## 快速启动

```powershell
# 编译 MSPM0 固件
cd ti_control
.\build.ps1                     # → msp_control.out (~16 KB)

# 烧录（需要 LaunchPad 接 USB）
C:\ti\ccs2100\ccs\ccs_base\DebugServer\bin\DSLite.exe flash `
  -c ti_control\targetConfigs\MSPM0G3507.ccxml `
  -l ti_control\msp_control.out -u

# 传文件到 K230（需先复位 K230，脚本会自动抢占 REPL）
python tools\_transfer_one.py   # 或 python tools\transfer_to_k230.py COM6
```

---

## 一、系统架构

```
┌─────────────────────────────────────────────────────────┐
│  GC2093 摄像头                                            │
│    ↓                                                     │
│  K230 (CanMV)                                            │
│    ├─ CHN_2 RGBP888 640×480 → AI2D → YOLO11n NPU         │
│    │    ~30ms 推理 → 钢球坐标 (cx, cy, confidence)         │
│    │    → pixel_to_cm() → xCmX100                         │
│    │    → Alpha-Beta tracker → 滤波坐标                    │
│    │                                                      │
│    ├─ UART1 IO9/IO10 @ 115200 → AA 55 协议帧              │
│    │    → MSPM0 PB16 (UART2 RX)                           │
│    │                                                      │
│    └─ CHN_0 RGB565 → JPEG Q=50 crop 640×240               │
│         → K23V/TCP WiFi → PC 接收端                       │
│                                                           │
│  ┌──────────────────────────────────────────────────┐    │
│  │  MSPM0G3507 (Cortex-M0+ 32MHz)                    │    │
│  │                                                    │    │
│  │  main() → App_Init() → App_Run() 主循环            │    │
│  │                                                    │    │
│  │  K230Uart_Task  → AA 55 帧解析 (ISR + 环形缓冲)    │    │
│  │  StaticBall_Task → bang-bang 控制器 (10ms)         │    │
│  │  Servo_Task      → PA8 软件 PWM 50Hz               │    │
│  │  LineFollow_Task → 巡线状态机                       │    │
│  │  Motor_Task      → 双电机 PI 速度控制               │    │
│  │  Oled_Task       → SSD1306 I2C 显示                │    │
│  │  Menu_Render     → 按键菜单                          │    │
│  │                                                    │    │
│  │  硬件:                                              │    │
│  │  ├─ UART2 PB16 RX  ← K230 IO9                      │    │
│  │  ├─ PA8 软件 PWM   → MG996 舵机                     │    │
│  │  ├─ I2C1 PB2/PB3   → OLED SSD1306                  │    │
│  │  ├─ TIMG0 PA12/13  → 左电机 H桥                     │    │
│  │  ├─ TIMG7 PA28/31  → 右电机 H桥                     │    │
│  │  ├─ GPIOB P6/7     → 右编码器 (正交)                 │    │
│  │  ├─ GPIOB P8/12    → 左编码器 (正交)                 │    │
│  │  ├─ GPIOx 8ch      → 红外巡线传感器                  │    │
│  │  └─ GPIOB P15/17/18/19 → 按键 S1-S4               │    │
│  └──────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

---

## 二、MSPM0 固件模块详解

### 构建系统
- **`build.ps1`**: 调用 SysConfig CLI 生成 ti_msp_dl_config，然后用 tiarmclang 编译 15 个 .c 文件，链接为 msp_control.out
- **`device.opt`**: `-D__MSPM0G3507__ -D__USE_SYSCONFIG__`
- **`device_linker.cmd`**: FLASH 0x00000000 128KB, SRAM 0x20200000 32KB, stack=512B
- **依赖**: SDK `2.10.00.04`, 编译器 `tiarmclang 4.0.2.LTS`, SysConfig `1.26.2`

### `empty.c` — 程序入口
```
main() → App_Init() → App_Run() → while(1) 主循环
```
唯一的入口文件。不做任何初始化，只做转发。

### `app.c` / `app.h` — 初始化 + 主循环调度
```c
App_Init():
    SYSCFG_DL_init()     // SysConfig 生成 (I2C1 配置)
    SystemTime_Init()    // SysTick 1KHz
    Buttons_Init()       // PB15/17/18/19 上拉输入
    Oled_StartInit()     // SSD1306 状态机启动
    K230Uart_Init()      // UART2 PB16, 115200, ISR
    Servo_Init()         // PA8 GPIO 输出
    StaticBall_Init()    // 静态球控制初始化
    LineFollow_Init()    // 巡线 + 编码器 + 电机
    Menu_Init()          // 菜单 UI

App_Run():
    while(1):
        K230Uart_Task()      // 处理环形缓冲中的串口数据
        StaticBall_Task()    // 10ms 周期 bang-bang 控制
        Servo_Task()         // 20ms 周期软件 PWM 脉冲
        LineFollow_Task()    // 巡线控制
        Oled_Task()          // OLED 状态机 (上电初始化)
        event = Buttons_Poll()
        if event: Menu_HandleEvent(event)
        Menu_Render()        // 按需刷新 OLED
        delay_cycles(32000)  // ~1ms UI tick
```
**注意**: `buttons.h` 通过 `menu.h` 间接包含，但 `app.c` 已显式包含。

### `system_time.c` / `system_time.h` — 系统时钟
- SysTick 配置为 1KHz (CPUCLK_FREQ/1000)
- `SysTick_Handler()`: `gSystemMillis++`
- `SystemTime_Millis()`: 返回 32 位毫秒值 (~49.7 天溢出)
- **注意**: 不处理溢出。对比赛时长(分钟级)无影响

### `k230_uart.c` / `k230_uart.h` — K230 视觉数据接收
**最关键的通信模块。**

**硬件**: UART2, PB16 RX (IOMUX_PINCM33), PA21 TX (IOMUX_PINCM46), 115200 8N1

**数据流**:
```
K230 IO9 TX → PB16 RX → UART2_IRQHandler()
    → DL_UART_Main_receiveDataCheck() → K230_RingPushFromIsr()
    → 128 字节环形缓冲 (gRingBuf)

K230Uart_Task():
    → K230_RingPop() → K230_ParseByte()
    → 流式 AA 55 帧解析器 (gStreamBuf[64])
    → K230_OnVisionPayload() → gStatus
    → K230Uart_GetLatest() → StaticBall_Task 使用
```

**AA 55 协议**:
```
Byte 0-1:  0xAA 0x55      帧头
Byte 2:    length          负载长度+2 (=8 for vision)
Byte 3:    type            0x01 = vision target
Byte 4:    sequence        帧序号 0-255
Byte 5:    flags           bit0=valid, bit1=tracked
Byte 6-7:  xCmX100         int16 LE, 钢球 X 坐标 cm×100
Byte 8-9:  yOffsetPx       int16 LE, Y 偏移像素
Byte 10:   quality         置信度 0-255
Byte 11:   checksum        bytes 2-10 求和 mod 256
```
最小帧长 12 字节。流式解析器处理垃圾字节、长度无效、校验失败等异常。

**环形缓冲 ISR 安全**:
- `gRingBuf[128]` + `gRingHead`/`gRingTail` (uint8, 128=2^8, 溢出即回绕)
- ISR 只写 head, 主循环只读 tail
- 缓冲区满时丢弃最旧字节 (tail+1)

**注意**: `K230_ResetParser()` 同时重置 ring head/tail。仅在 `K230Uart_Init()` 中调用 (IRQ 尚未使能)，所以安全。

### `static_ball.c` / `static_ball.h` — 任务3：静态滚球控制
**核心控制器模块。**

```c
控制参数:
    CONTROL_PERIOD_MS = 10         // 控制周期
    VISION_TIMEOUT_MS = 200        // 视觉超时 → 脱开
    MIN_QUALITY = 1                // 最低置信度
    INVALID_LIMIT = 3              // 连续无效帧 → 脱开
    DIRECTION_DEADBAND_CM_X100 = 20  // ±0.2cm 死区

目标:
    POS_TARGET = +5.0 cm
    NEG_TARGET = -5.0 cm
    ARRIVE_BAND = 0.5 cm            // 到达判定

舵机:
    SERVO_NEUTRAL_DEG = 90          // 中位 90°=1500µs
    SERVO_DIRECTION = -1.0          // 翻转方向
    SERVO_DEG_PER_TILT_DEG = 1.0    // 1:1 映射

倾角 (可独立调节):
    MOVE_TILT_LEFT_DEG  = 12        // 球偏左,移动阶段
    MOVE_TILT_RIGHT_DEG = 12        // 球偏右,移动阶段
    HOLD_TILT_LEFT_DEG  = 8         // 球偏左,保持阶段
    HOLD_TILT_RIGHT_DEG = 8         // 球偏右,保持阶段
```

**相序状态机**:
```
WAIT_VISION(0) → 收到首帧 → TO_POS(1) → 到达+4.5cm → TO_NEG(2) → 到达-4.5cm → HOLD_NEG(3)
```
- TO_POS: 目标 +5cm, 大倾角 (MOVE_TILT_*)
- TO_NEG: 目标 -5cm, 大倾角
- HOLD_NEG: 目标 -5cm, 小倾角 (HOLD_TILT_*), ±0.2cm 死区内倾角=0

**安全**:
- 200ms 无有效帧 → `StaticBall_DisableServo()` (PA8 拉低, 无 PWM)
- 连续 3 帧无效 → 同上
- 舵机角度软件限制 0-180°

**菜单集成**:
- S2 启动: `StaticBall_Start()` (进入 WAIT_VISION)
- S3 停止: `StaticBall_Stop()` (脱开 + 重置)
- S4 返回: `StaticBall_Exit()` (同 Stop, 返回菜单)

### `servo.c` / `servo.h` — 软件 PWM 舵机驱动
**关键**: 不是硬件 PWM。用 `delay_cycles()` 产生 50Hz 信号。

```c
引脚: PA8 (IOMUX_PINCM19), GPIOA, DL_GPIO_PIN_8
频率: 50Hz (20ms 周期)
脉冲: 1000µs (0°) - 2000µs (180°), 中位 1500µs (90°)
实现: delay_cycles(pulseUs * 32)  @ 32MHz
```

`Servo_Task()` 每 20ms 调用一次 `Servo_SendPulse()`：
1. PA8 HIGH
2. `delay_cycles(pulseUs * 32)`  ← 阻塞 1-2ms
3. PA8 LOW
4. 返回，剩余 ~18ms 主循环干其他事

`Servo_Attach()`: 使能脉冲输出
`Servo_Detach()`: PA8 拉低，无脉冲 → 舵机脱力

**注意**: 软件 PWM 在主循环中阻塞 1-2ms。这是有意为之——简单可靠，无需定时器。对系统其他部分无影响（总循环 < 5ms）。

### `line_follow.c` / `line_follow.h` + `line_follow_config.h` — 巡线控制
**任务2 巡线。当前任务3测试时不需要，但已集成。**

状态机:
- 正常巡线 → 边沿检测 (LINE_EDGE_LEFT_MASK/RIGHT_MASK) → 差速转向
- 丢线 → 保持最后方向 LINE_LOST_HOLD_MS → 停车
- 全黑 → 停车 (LINE_STOP_BLACK_COUNT)
- 轮速平衡 → LINE_WHEEL_BALANCE_ENABLE, 自适应两侧轮速差

**关键参数** (line_follow_config.h):
```
LINE_SENSOR_BLACK_IS_LOW = 1   // 黑线=低电平
LINE_SENSOR_REVERSE_ORDER = 0  // 不反转
MOTOR_LEFT_DIRECTION_INVERT = 1
MOTOR_RIGHT_DIRECTION_INVERT = 0
MOTOR_LEFT_ENCODER_REVERSE = 0
MOTOR_RIGHT_ENCODER_REVERSE = 1
```

### `motor.c` / `motor.h` — 双电机 PI 速度控制
- 左电机: TIMG0, PA12(CCP0), PA13(CCP1), H桥
- 右电机: TIMG7, PA28(CCP0), PA31(CCP1), H桥
- PWM: 1KHz (period=1000), 边沿对齐
- 控制: PI 速度环, 10ms 周期
- 前馈: FEED_FORWARD + PWM_PER_TICK × target
- 积分分离: target=0 时清零积分
- 斜率限制: PWM_SLEW_STEP = 25/周期
- 测量滤波: 一阶低通 (FILTER_SHIFT=1, α≈0.5)

### `line_sensor.c` / `line_sensor.h` — 8 路红外传感器
```c
引脚: PA15, PA17, PA22, PA24, PA25, PB9, PA27, PB20
权重: [-3500, -2500, -1500, -500, +500, +1500, +2500, +3500]
```

`LineSensor_Read()` 返回:
- `blackMask`: 检测到黑线的传感器位掩码
- `activeCount`: 检测到黑线的传感器数量
- `position`: 加权平均位置 (负=偏左, 正=偏右)
- `lineLost`/`allBlack`: 状态标志

### `encoder.c` / `encoder.h` — 正交编码器
- 左: PB8(A), PB12(B) — IOMUX_PINCM25/PINCM29
- 右: PB6(A), PB7(B) — IOMUX_PINCM23/PINCM24
- 中断: GROUP1_IRQHandler (GPIOB 双边沿)
- 正交解码: 4位状态转换查找表 (gQuadratureDelta[16])
- 线程安全: `Encoder_GetAndResetDeltas()` 用 __disable_irq() 保护

**已知问题**: ISR 在任意 GPIOB 边沿触发时更新两个编码器。正交表对未变化的编码器返回 delta=0，所以不影响正确性，但增加不必要的 CPU 开销。

### `menu.c` / `menu.h` + `buttons.c` / `buttons.h` — 用户界面
**按键**:
- S1: PB15 → 菜单项切换 (向下)
- S2: PB17 → 确认/进入
- S3: PB18 → 停止当前功能
- S4: PB19 → 返回主菜单

消抖: 4 轮采样确认 (BUTTON_DEBOUNCE_TICKS)

**菜单结构**:
```
FOLLOW MENU
  > Line Follow     → 巡线 (含传感器/编码器/电机状态)
    Static Ball     → 静态球 (含 X/V/P/G/S 状态)
    AB Balance      → (TODO)
    Full Loop       → (TODO)
    Target Pos      → (TODO)
```

### `oled.c` / `oled.h` — SSD1306 OLED 128×64
- 接口: I2C1 (PB2 SCL, PB3 SDA), 地址 0x3C, 100KHz
- 初始化: 状态机 (OFF→POWER_WAIT→INIT_SEND→READY), 失败自动重试
- 字体: 6×8 ASCII (A-Z, a-z, 0-9, 符号)
- 缓冲: 128×64/8 = 1024 字节 (gOledBuffer)
- `Oled_Update()`: 逐页写入 I2C
- **注意**: `Oled_Update()` 在栈上有 `packet[129]`, 可能是最大的栈消费者

---

## 三、K230 端

### `k230_yolo.py` — 主程序
**文件位置**: `/sdcard/app/k230_yolo.py` (K230) / `k230_code/k230_yolo.py` (PC)

**工作流程**:
```
while True:
    1. sensor.snapshot(CHN_2) → RGBP888 640×480
    2. yolo.run(ai_np) → 检测结果
       - 最高置信度框 → 钢球中心坐标
       - 模型: yolo11n_det_320.kmodel, 1类, 置信度阈值 0.35
    3. AlphaBetaTracker.update/filter → 滤波坐标
    4. send_ball() → UART AA 55 帧 → MSPM0
    5. WiFi (非阻塞):
       - boot 时初始化一次 wifi_init_nonblock()
       - 主循环检查 isconnected(), 每 3s 尝试 PC 连接 (50ms 超时)
       - 不阻塞检测循环！
    6. JPEG 编码 + TCP 发送 (K23V 协议, 可选)
```

**标定值** (2026-07-31):
```python
ZERO_X_PX = 345.0          # 钢球在 0cm 处的像素 X 坐标
PX_PER_CM = 20.1           # 像素/cm 转换比
```

**UART**: UART1, IO9=TX, IO10=RX, 115200, FPIOA 映射

**WiFi 修复 (2026-07-31)**:
- 旧版: `wifi_connect_once()` 有 8s 阻塞循环 → 无 WiFi 时检测显示 LOST
- 新版: 初始化时不等待连接，主循环只检查状态，不阻塞

### K23V 视频协议 (PC 接收端)
- Header: `K23V` (4B) + version (1B) + codec (1B=JPEG) + 4-byte BE length
- PC: `pc_receiver/pc_receiver.py`, opencv + numpy, 按 r 录像
- 图传只在 PC 连接时发送，不影响检测

### 文件传输
- `tools/transfer_to_k230.py COM6`: 传全部文件 (含 libs, kmodel)
- `tools/_transfer_one.py`: 传单个文件，使用 DTR 复位 + Ctrl-C 抢占 REPL
- **诀窍**: K230 自启动脚本后需要复位 → 立即 Ctrl-C 抢占才能在脚本启动前进入 REPL

---

## 四、引脚分配总表

| 外设 | MSPM0 引脚 | IOMUX | 功能 |
|------|-----------|-------|------|
| K230 UART RX | PB16 | PINCM33 | UART2 RX, 115200 |
| K230 UART TX | PA21 | PINCM46 | UART2 TX |
| 舵机信号 | PA8 | PINCM19 | GPIO 软件 PWM |
| OLED SCL | PB2 | PINCM15 | I2C1 SCL |
| OLED SDA | PB3 | PINCM16 | I2C1 SDA |
| 左电机 CCP0 | PA12 | PINCM34 | TIMG0 CCP0 |
| 左电机 CCP1 | PA13 | PINCM35 | TIMG0 CCP1 |
| 右电机 CCP0 | PA28 | PINCM3 | TIMG7 CCP0 |
| 右电机 CCP1 | PA31 | PINCM6 | TIMG7 CCP1 |
| 左编码器 A | PB8 | PINCM25 | GPIO 正交 |
| 左编码器 B | PB12 | PINCM29 | GPIO 正交 |
| 右编码器 A | PB6 | PINCM23 | GPIO 正交 |
| 右编码器 B | PB7 | PINCM24 | GPIO 正交 |
| 红外 1 | PA15 | PINCM37 | 巡线 |
| 红外 2 | PA17 | PINCM39 | 巡线 |
| 红外 3 | PA22 | PINCM47 | 巡线 |
| 红外 4 | PA24 | PINCM54 | 巡线 |
| 红外 5 | PA25 | PINCM55 | 巡线 |
| 红外 6 | PB9 | PINCM26 | 巡线 |
| 红外 7 | PA27 | PINCM60 | 巡线 |
| 红外 8 | PB20 | PINCM48 | 巡线 |
| 按键 S1 | PB15 | PINCM32 | 上拉输入 |
| 按键 S2 | PB17 | PINCM43 | 上拉输入 |
| 按键 S3 | PB18 | PINCM44 | 上拉输入 |
| 按键 S4 | PB19 | PINCM45 | 上拉输入 |

---

## 五、当前状态 (2026-07-31)

### 已验证
- [x] 编译: `build.ps1` → `msp_control.out` (16,184 bytes)
- [x] 烧录: DSLite + XDS110 成功
- [x] OLED 菜单: 5 项菜单 + 按键导航
- [x] K230 → MSPM0 UART: AA 55 帧接收正常
- [x] K230 球检测: YOLO11n NPU 检测 + 坐标显示正常
- [x] WiFi 非阻塞: 无 WiFi 时检测不卡顿
- [x] 舵机 PWM: PA8 软件 PWM 编译通过

### 待验证
- [ ] 舵机实机运动 (MG996 需外接 5V/>2A)
- [ ] 方向标定 (SERVO_DIRECTION = -1.0 是否正确)
- [ ] 倾角幅度调参 (上升/下降不对称问题)
- [ ] Static Ball 完整流程 (+5cm → -5cm → hold)
- [ ] 巡线功能 (未测试)

### 已知问题
1. **舵机不对称**: 用户报告上升幅度小、下降幅度大。已添加独立 LEFT/RIGHT 倾角值待调。
2. **K230 自启动**: 脚本设为开机自启 → 串口 REPL 被占用 → 需复位+抢占方式传文件
3. **encoder ISR**: 两个编码器共享 GPIOB 中断, 任一变化都更新两者 (性能损失可接受)
4. **无 IMU**: MPU-6050 驱动在 `ti_mpu6050_test/`, 当前未集成

---

## 六、环境

| 工具 | 路径/版本 |
|------|----------|
| CCS | `C:\ti\ccs2100` |
| MSPM0 SDK | `C:\ti\mspm0_sdk_2_10_00_04` |
| TI Arm Clang | `C:\ti\ti_cgt_arm_llvm_4.0.2.LTS` |
| SysConfig | `C:\ti\sysconfig_1.26.2` |
| DSLite | `C:\ti\ccs2100\ccs\ccs_base\DebugServer\bin\DSLite.exe` |
| K230 串口 | COM6, 115200 |
| Python | 系统默认 (serial, time, base64) |

---

## 七、目录结构

```
26-dian-sai/
├── CLAUDE.md              # 竞赛规则 + 快速参考
├── HANDOFF.md             # 本文件: 完整技术文档
├── .gitignore
│
├── ti_control/            # MSPM0 固件 (15 模块)
│   ├── empty.c            # main() 入口
│   ├── app.c/h            # 初始化 + 主循环
│   ├── system_time.c/h    # SysTick 1KHz
│   ├── k230_uart.c/h      # UART2 AA55 帧接收
│   ├── static_ball.c/h    # 静态球 bang-bang
│   ├── servo.c/h          # PA8 软件 PWM
│   ├── line_follow.c/h    # 巡线
│   ├── line_follow_config.h # 巡线+电机调参
│   ├── line_sensor.c/h    # 8 路红外
│   ├── motor.c/h          # 双电机 PI
│   ├── encoder.c/h        # 正交编码器
│   ├── menu.c/h           # OLED 菜单
│   ├── buttons.c/h        # 4 按键
│   ├── oled.c/h           # SSD1306 I2C
│   ├── build.ps1          # 编译脚本
│   ├── msp_control.syscfg # I2C1 only
│   ├── device.opt         # 编译宏
│   ├── device_linker.cmd  # 链接脚本
│   ├── README.md          # MSPM0 使用说明
│   └── targetConfigs/     # CCS 烧录配置
│       └── MSPM0G3507.ccxml
│
├── k230_code/             # K230 端代码
│   ├── k230_yolo.py       # 主程序 (YOLO+UART+WiFi)
│   ├── k230_calibrate.py  # 标定工具
│   ├── k230_final.py      # 备用 (motion-based)
│   └── libs/              # K230 SDK 库
│       ├── YOLO.py, AI2D.py, AIBase.py
│       ├── PipeLine.py, Utils.py
│
├── pc_receiver/           # PC 接收端
│   └── pc_receiver.py     # K23V 接收 + 显示 + 录像
│
├── ti_mpu6050_test/       # MPU-6050 (待集成)
│   ├── mpu6050.c/h        # 互补滤波 + 零偏标定
│
├── tools/                 # 工具
│   ├── transfer_to_k230.py  # K230 批量文件传输
│   └── _transfer_one.py     # K230 单文件传输
│
└── reference_code/        # 参考代码
    └── laoguigui2/        # YOLO 模型来源
```
