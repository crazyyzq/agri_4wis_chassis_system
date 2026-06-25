# ECU 板级测试接线

依据 `doc/ECU/SCH_ECU_Isolation_原理图.pdf` V1.0 第 13 页。连接器 U64 的针脚文字在原理图中旋转显示，以下已恢复正常方向。

## 数字输出

| 信号 | U64 针脚 | 信号 | U64 针脚 |
|---|---:|---|---:|
| EX_OUT1 | A46 | EX_OUT7 | A49 |
| EX_OUT2 | A31 | EX_OUT8 | A34 |
| EX_OUT3 | A47 | EX_OUT9 | A50 |
| EX_OUT4 | A32 | EX_OUT10 | A35 |
| EX_OUT5 | A48 | EX_OUT11 | A51 |
| EX_OUT6 | A33 | EX_OUT12 | A36 |

## 数字输入

| 通道 | 正端 | 负端 | 通道 | 正端 | 负端 |
|---|---:|---:|---|---:|---:|
| EX_IN1 | A37 | A52 | EX_IN7 | A43 | A58 |
| EX_IN2 | A38 | A53 | EX_IN8 | A44 | A59 |
| EX_IN3 | A39 | A54 | EX_IN9 | A8 | A23 |
| EX_IN4 | A40 | A55 | EX_IN10 | A9 | A24 |
| EX_IN5 | A41 | A56 | EX_IN11 | A10 | A25 |
| EX_IN6 | A42 | A57 | EX_IN12 | A11 | A26 |

## DIO 回环夹具

- 每路 `EX_INn+` 接隔离侧 12 V。
- 每路 `EX_INn-` 接对应 `EX_OUTn`；输出导通时将输入负端拉低。
- 初次上电前逐线检查，不允许相邻输出短接。
- DO 带载测试另接 `220 Ω / 2 W` 电阻，任意时刻只接通一路，最长 500 ms。
- BAT_GND、MCU_DGND、ISO_GND 是不同电气域；除原理图和夹具明确要求外不要短接。

## 通信夹具

- CAN1..4：外部适配器设为 Classic CAN、500 kbit/s；禁止开启 CAN-FD。
- RS485-1..3：115200 bit/s、8N1，适配器回显完整测试帧。
- RS232-1..4：115200 bit/s、8N1，使用交叉回环线或回显适配器。
- SBUS：100000 bit/s、8E2；板载三极管负责反相。
