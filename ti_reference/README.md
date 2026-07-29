# Ti/MSPM0 参考目录

`firmware_state_machine_skeleton.c` 是协议解析、PD 外环、超时回中位和端部救球的**逻辑参考**，不是具体芯片工程。

后续接入真实 Ti 工程时，需补齐 UART/DMA、100 Hz 调度、50 Hz PWM 和 `platform_*` 硬件钩子。具体顺序见 [装配与联调清单](../文档/下一阶段装配与联调清单.md)。
