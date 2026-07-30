# LP-MSPM0G3507 MPU-6050 独立测试工程

这是已经在原 **LP-MSPM0G3507 LaunchPad** 上实际烧录验证过的 MPU-6050 点亮工程。它只验证供电、I²C 寻址和原始六轴数据读取；**不会控制舵机、电机或 K230**，不能直接当作小车正式控制固件。

完整的测试证据、接线、构建/烧录和下一步集成边界见 [交接文档](../文档/MPU6050独立测试与接入指引.md)。

## 文件职责

| 文件 | 作用 |
| --- | --- |
| `msp_mpu6050_test.c` | 读取 `WHO_AM_I`，唤醒 MPU-6050，并通过调试串口持续输出原始加速度/陀螺仪数据。 |
| `msp_mpu6050_test.syscfg` | PB2/PB3 的 I2C1（100 kHz）和 PA10/PA11 的 UART0（115200）的唯一配置来源。 |
| `mspm0g3507.ccxml` | XDS110 + MSPM0G3507 的 DSLite/CCS 调试目标。 |
| `build.ps1` | 使用本机 CCS、SDK、SysConfig 的命令行构建脚本；自动生成并忽略 SysConfig/编译产物。 |

## 快速复现

在 PowerShell 中执行：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\build.ps1
```

默认工具路径适配当前电脑；不同电脑可传入 `-SdkRoot`、`-CompilerRoot` 和 `-SysConfigCli` 覆盖。构建成功后，用 CCS 或 DSLite 依据 `mspm0g3507.ccxml` 烧录 `msp_mpu6050_test.out`，再以 `115200` 打开 XDS110 Application/User UART。
