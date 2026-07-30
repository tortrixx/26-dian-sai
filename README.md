# 2026 电赛 H 题：车载平衡滚球控制系统

## 当前部署入口

| 目标 | 文件 | 状态 |
| --- | --- | --- |
| K230 YOLO 视觉、UART、图传 | `k230_code/k230_yolo.py` | ✅ 主力部署入口 |
| K230 备用传统检测 | `k230_code/k230_final.py` | 备选（motion-based） |
| PC 图传接收与录像 | `pc_receiver/pc_receiver.py` | ✅ 与 K23V 协议匹配 |
| Ti/MSPM0 控制逻辑参考 | `ti_reference/firmware_state_machine_skeleton.c` | 待合入实际 Ti 工程 |
| MPU-6050 独立点亮测试 | `ti_mpu6050_test/` | ✅ 已在 LP-MSPM0G3507 通过 |
| 协议参考 | `k230_libs/k230_mspm0_uart_protocol.py` | ✅ |

## 当前基线（2026-07-30）

- **K230 视觉**：YOLO11n NPU 钢球检测（Laoguigui2 模型，1 类，320×320 输入）
  - CHN_0 RGB565 640×480 → JPEG 图传
  - CHN_2 RGBP888 640×480 → AI2D → YOLO NPU 推理
  - 检测置信度 0.35，~22 FPS 总循环，~30ms NPU 推理
- **图传**：K23V 协议（4 字节 BE 长度），JPEG Q50，640×240 pipe crop，~5-6 FPS / 40-80 KB/s
- **控制**：K230 每帧经有线 UART 发送 `x_cm_x100` 给 MSPM0；PC 图传不参与控制
- **标定**：pixel-to-cm 参数（ZERO_X_PX, PX_PER_CM）需以实际装车后重新标定

新接手者先阅读 `CLAUDE.md`（硬件约束 + YOLO 管道细节）和 `文档/README.md`（交接文档）。

## 快速启动

```powershell
# 1. PC 接收端（先启动）
cd pc_receiver
python pc_receiver.py

# 2. K230（CanMV IDE 中打开运行）
# 文件：k230_code/k230_yolo.py
# 确保热点 test/90z5M92# 已开启，PC IP 192.168.137.1
```

## 目录约定

- `k230_code/`：K230 部署入口 + SDK 库 + 诊断脚本
- `k230_libs/`：UART 协议参考
- `pc_receiver/`：PC 图传接收、显示与录像（按 r 录像，q 退出）
- `ti_reference/`：Ti/MSPM0 控制参考骨架
- `ti_mpu6050_test/`：MPU-6050 独立 I²C 测试
- `文档/`：交接、TODO、装配、标定、图传和验收资料
- `reference_code/`：队友原始代码 + Laoguigui2 参考
- `tools/`：K230 串口文件传输脚本
