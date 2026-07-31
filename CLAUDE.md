# 2026 电赛 H 题 —— 车载平衡滚球运动控制系统

> 基于官方 66 问 Q&A 总结的关键约束与判题规则。每次写代码前参考。

---

## 一、硬件约束速查

### MCU / 主控
- **不限型号**：TI/MSPM0、STM32、树莓派、FPGA 均可
- **允许多 MCU**：循迹、摆杆控制、视觉图传可分芯片控制
- 电机驱动板、红外循迹、陀螺仪、步进电机 **允许自带板载 MCU**

### 视觉模块 (K230 / MaixCAM / OpenMV)
- **允许**自带处理器，**允许**直接输出钢球坐标给主控
- **允许双摄像头**：一路检测钢球做闭环，一路单独负责图传
- 同一摄像头可同时兼顾识别 + 图传
- 安装位置不限制，可随摆杆一同摆动

### 摆杆结构
- 左侧链接：**必须使用合页 / 铰链**固定，不允许其他构型（如跷跷板式中支点）
- 控制机构：图示仅为示意图，舵机 / 步进 / 闭环步进 / 齿轮连杆 / 曲柄摇杆 **均允许**
- 摆杆装置 **不能超出小车车身**（双层小车上层计入车身尺寸）
- **水管禁止任何改造**：不可开孔、不可加装传感器拓展板
- **管壁不允许有任何检测及控制装置**
- 内壁可加装防滚落挡片（不影响观测区域即可）
- 钢球：**正常钢球，不允许喷色、磨砂**

### 循迹传感器
- 循迹 **只能使用红外光电模块**
- 禁止用摄像头循迹
- 允许搭配编码器、陀螺仪、加速度计辅助车速/转向/停车（仅黑线识别用红外）

### 屏幕
- **2 英寸限制仅针对计时显示屏幕**，其他屏幕不限
- 触摸屏调试参数不在此限

### 禁止项
- **水管禁止染色、喷涂、加装铁块/金属配件**
- **凹槽内禁止加装电磁铁**吸附钢球
- 钢球 **不允许喷涂彩色漆、磨砂漆**

---

## 二、图传要求

| 项目 | 规则 |
|------|------|
| 传输方式 | **必须无线** |
| 发送端 | K230、MaixCAM、大疆 Action、GoPro、树莓派摄像头均可 |
| 接收端 | 笔记本、平板、手机均可 |
| 画面要求 | 清晰即可，灰度黑白允许 |
| 录像存储 | 电脑/平板/手机录屏即可，**无需车载端 SD 卡存储** |
| 回放 | 手动操作，清晰即可 |
| 实时性 | 需全程实时显示在场外设备 |

### 关键约束
> **测试期间仅允许图传工作**（无线通信仅限于图传）

- 禁止图传回传控制指令给小车（不允许场外处理后再遥控）
- 禁止调试数据回传
- 测试前必须关闭/拆除其他无线模块

---

## 三、判题规则

### 钢球位置误差
- **全程误差 ≤1cm**（瞬时超差也算？"考察全程"）
- 钢球脱落 = 本次测试失败
- 电机卡死/机械故障可断电复位，继续完成剩余测试

### 计时
- 任务 2：行驶总长 ≤20s
- 任务 5/6：一圈 ≤30s
- 任务 3：移动 ≤5s
- 屏显信息仅为参考，以评委为准

### 测试流程
- 允许开机初始化、校准
- 每项任务测试次数由赛区自定
- 现场提供 220V 市电
- **不提供公共 WiFi**，自备热点/路由器
- 同频干扰自行解决
- 组委会不提供水管/3D打印/合页等备品

---

## 四、对我们方案的影响

### 当前方案（2026-07-31 更新）

```
GC2093 → K230 YOLO11n NPU 钢球检测 → UART → MSPM0 舵机控制
       → K230 CHN_0 RGB565 → JPEG → K23V/TCP WiFi → PC 接收端  ✅ 合规（无线图传）
       → K230 CHN_2 RGBP888 → AI2D → YOLO NPU 推理
PC 端仅接收显示 + 录屏（按 r 键）                                    ✅ 合规（不回车）
```

#### YOLO NPU 管道（k230_code/k230_yolo.py）
| 组件 | 细节 |
|------|------|
| 模型 | yolo11n_det_320.kmodel（Laoguigui2 钢球专用，1类，3MB） |
| 输入 | CHN_2 RGBP888 640×480 → AI2D letterbox pad + bilinear resize → 320×320 |
| 初始化顺序 | **set_framesize → set_pixformat**（顺序错误触发 Yahboom v1.4.3 buf_init bug） |
| 标定 | **ZERO_X_PX=345.0, PX_PER_CM=20.1**（2026-07-31 五点实机，最终相机位置） |
| 置信度阈值 | 0.35（0.5 以上漏检严重） |
| 推理速度 | ~30ms/帧（NPU 独立于 CPU） |
| 总帧率 | **~26 FPS**（含 JPEG 编码 + WiFi 推流） |
| 图传 FPS | ~5-6 FPS（JPEG Q=50 @ 640×240 pipe crop） |

#### K23V 视频协议（关键！）
- Header: `K23V` (4B) + version (1B) + codec (1B=JPEG) + **4-byte BE length**
- **图传稳定性修复（2026-07-31）**：非阻塞 socket 在 TCP 缓冲区满时返回 0/OSError(EAGAIN)，旧代码直接断线。修复：512B chunks + 连续 60 次失败才判定断线 + 重连冷却 3s
- PC 接收端：`pc_receiver/pc_receiver.py`，依赖 `opencv-python numpy`

#### MSPM0 控制固件（ti_control/，✅ 已编译验证）
| 组件 | 细节 |
|------|------|
| 方案 | **纯视觉 bang-bang，无 IMU**（MPU-6050 当前不可用） |
| 入口 | `ti_control/empty.c` → `app.c`（15 个模块） |
| UART | **UART2 PB16 RX** ← K230 IO9 TX，AA 55 协议，115200 8N1 |
| 舵机 | **PA8 软件 PWM**，50Hz，0-180°（90°=1500µs 中位） |
| 控制 | bang-bang 方向控制：球在目标左边→正倾角，右边→负倾角 |
| 控制周期 | 10ms（`StaticBall_Task` 轮询），move=±12°, hold=±8° |
| 安全 | 200ms 视觉超时→舵机脱开、3 帧连续无效→脱开 |
| 构建 | `cd ti_control && .\build.ps1` → `msp_control.out` |
| 烧录 | CCS 或 DSLite + `targetConfigs/MSPM0G3507.ccxml` |
| 模块 | `app`, `buttons`, `encoder`, `k230_uart`, `line_follow`, `line_sensor`, `menu`, `motor`, `oled`, `servo`, `static_ball`, `system_time` |
| SysConfig | **仅 I2C1 (PB2/PB3)**，其余外设全用 driverlib 直驱 |

#### 关键文件
| 文件 | 用途 |
|------|------|
| `k230_code/k230_yolo.py` | 主程序：YOLO 检测 + WiFi 推流 + UART |
| `k230_code/k230_calibrate.py` | 标定工具：含 WiFi 推流 + ROI 过滤 |
| `k230_code/k230_final.py` | 备用：motion-based 检测器（无 NPU） |
| `pc_receiver/pc_receiver.py` | PC 端接收 + 显示 + 录像（按 r 录，q 退） |
| `ti_control/empty.c` | MSPM0 入口 `main()` → App_Init/Run |
| `ti_control/app.c` | 应用初始化 + 主循环调度 |
| `ti_control/static_ball.c` | 任务3 静态滚球 bang-bang 控制器 |
| `ti_control/servo.c` | PA8 软件 PWM 舵机驱动 |
| `ti_control/k230_uart.c` | UART2 PB16 K230 视觉帧接收 + AA55 解析 |
| `ti_control/motor.c` | TIMG0/TIMG7 硬件 PWM 双电机 PI 速度控制 |
| `ti_control/line_follow.c` | 巡线状态机 + 差速转向 |
| `ti_control/line_sensor.c` | 8 路红外巡线传感器 |
| `ti_control/encoder.c` | GPIOB 正交编码器 |
| `ti_control/menu.c` | OLED 菜单系统 |
| `ti_control/msp_control.syscfg` | 仅 I2C1 (OLED)，其余外设用 driverlib 直驱 |
| `ti_control/README.md` | MSPM0 接线、校准说明 |
| `ti_mpu6050_test/mpu6050.h` | MPU-6050 姿态估计 API（互补滤波 + 零偏标定） |
| `ti_mpu6050_test/mpu6050.c` | MPU-6050 姿态估计实现 |
| `k230_code/libs/` | K230 SDK 库文件（YOLO, AI2D, AIBase, etc.） |
| `reference_code/laoguigui2/` | Laoguigui2 参考代码 + kmodel |
| `tools/transfer_to_k230.py` | 串口文件传输脚本 |

### 风险点
1. **同频 WiFi 干扰**：赛场多队同时用 2.4G，考虑 5.8G 图传模块作备选
2. **全程误差 ≤1cm**：YOLO 检测精度依赖 pixel-to-cm 标定，**相机移动必须重新标定**
3. **钢球脱落 = 失败**：防滚落挡片要做好
4. **不允许场外处理回传**：PC 端只做录像
5. **YOLO 模型是 1 类检测器**：只检测钢球，换其他模型需确认 class count 匹配
6. **MSPM0 固件已验证编译**（~14 KB/128 KB flash），待实机烧录联调

### 编码注意事项
- K230 传感器初始化：**必须先 set_framesize 再 set_pixformat**，否则 CHN_2 触发 buf_init
- YOLO labels 用 dict 格式：`{0: 'steel'}`，不是 list（class_num = len(labels)）
- K23V 协议长度字段用 **4 字节 BE**（与 PC 端 `struct.unpack(">I")` 对齐）
- 模型加载时 sensor 不要 run（避免 CHN_2 4 帧缓冲溢出）
- 图传 JPEG Q=50 @ 640×240 crop 约 8KB/帧，5fps ≈ 40KB/s
- 测试前关闭 K230 上除 WiFi 外的所有无线功能
- K230 非阻塞 socket send() 可能返回 0 或 OSError——不要当作断线，重试即可
- MSPM0 OLED 菜单 `Static Ball` → 自动 +5cm → -5cm → 保持；S3 停止，S4 返回
- MSPM0 接线：K230 IO9→MSPM0 PB16 (UART2 RX)，舵机信号→PA8，I2C OLED→PB2/PB3
- MSPM0 SysConfig 仅配置 I2C1；其余 UART/PWM/GPIO 用 `DL_xxx` API 直驱
- MSPM0 编译环境：CCS `C:\ti\ccs2100`，SDK `2.10.00.04`，TI Arm Clang `4.0.2.LTS`，SysConfig `1.26.2`
