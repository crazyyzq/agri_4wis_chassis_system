# ECU 板级测试固件完整通读指南

最后更新：2026-06-25  
适用分支：`codex/ecu-board-test`  
适用工程：`ecu/apps/ecu_board_test`

这份文档的目标不是替代代码注释，而是带你按真实执行顺序理解整个工程：先知道程序怎么跑起来，再知道每个文件夹放什么、每个文件负责什么、文件之间如何关联，最后按推荐路线从 `main()` 开始通读全部代码。

当前工程是 HPM6750 ECU 的板级测试/调试固件，不是整车控制程序。它用于验证自研 ECU 板的电源、RGB、存储器、ADC、DI、DO、DIO 回环、CAN、RS232、RS485、SBUS 和以太网占位项。所有危险输出都必须经过安全层，所有测试结果都通过 UART0 调试串口输出。

---

## 1. 先建立整体地图

工程核心目录如下：

```text
agri_4wis_chassis_system/
├─ doc/                                      原理图、芯片手册、模块资料
├─ docs/                                     设计、计划、进度、代码阅读文档
├─ ecu/
│  ├─ apps/ecu_board_test/                   当前板级测试固件应用
│  │  ├─ include/                            应用公共头文件
│  │  ├─ src/                                应用框架、服务和纯算法实现
│  │  ├─ tests/                              正式板级测试用例
│  │  ├─ selftest/                           上电目标侧软件自测
│  │  ├─ linkers/{gcc,iar,segger}/           三套工具链链接脚本
│  │  ├─ app.yaml                            HPM SDK 应用依赖声明
│  │  └─ CMakeLists.txt                      应用构建入口
│  ├─ boards/ecu_isolation/                  自定义 ECU 板级支持包
│  │  ├─ board.c / board.h                   手写 BSP、硬件映射和安全电平
│  │  ├─ pinmux.c / pinmux.h                 Pinmux Tool 生成的引脚初始化
│  │  ├─ pinmux.hpmpc                        Pinmux Tool 工程源文件
│  │  ├─ ecu_isolation.yaml                  SDK 板卡能力描述
│  │  ├─ ecu_isolation.cfg                   OpenOCD/板卡参考配置
│  │  └─ CMakeLists.txt                      板级源码注册
│  ├─ scripts/                               构建、SES 生成、J-Link、串口脚本
│  └─ sdk_env_v1.11.0/                       本地 SDK/工具链，Git 忽略
├─ test/fixtures/                            接线表和人工测试流程
└─ tmp/                                      构建产物和生成工程，Git 忽略
```

几个关键判断：

- 你主要通读 `ecu/apps/ecu_board_test/` 和 `ecu/boards/ecu_isolation/`。
- `ecu/sdk_env_v1.11.0/` 很大，是 SDK 和工具链，不要从这里开始读；遇到具体 HPM API 再跳进去查定义。
- `tmp/` 是生成产物，不是源码。
- `docs/superpowers/` 是设计/实施计划记录，不是固件运行路径。

---

## 2. 从 `main()` 看程序真实启动顺序

入口文件是：

```text
ecu/apps/ecu_board_test/src/main.c
```

核心调用顺序：

```text
main()
  -> board_init()
  -> status_led_init_default()
  -> status_led_set(STATUS_LED_BOOTING)
  -> ppor_reset_get_flags()
  -> safety_init(&real_safety_ops)
  -> selftest_run_all()
  -> safety_init(&real_safety_ops)
  -> status_led_init_default()
  -> periodic_tx_init_default()
  -> sbus_service_init()
  -> ecu_debug_monitor_init()
  -> status_led_set(READY or FAILED)
  -> app_shell_run()
```

逐段理解：

### 2.1 `board_init()`

位置：

```text
ecu/boards/ecu_isolation/board.c
```

作用：

- 初始化核心时钟。
- 初始化 UART0 调试串口。
- 初始化安全 GPIO。
- 初始化 PMP/PMA。
- 支持 SDRAM/XIP 等板级基础环境。

这是所有硬件访问的地基。你读 `main.c` 时看到的 `board_ecu_output_write()`、`board_set_can_termination()`、`board_init_uart()` 等最终都落到 `board.c`。

### 2.2 `status_led_init_default()` 和 `status_led_set(BOOTING)`

位置：

```text
ecu/apps/ecu_board_test/src/status_led.c
```

作用：

- 管理 RGB 状态灯。
- 上电后先进入 `STATUS_LED_BOOTING`，红灯常亮。
- 后面根据测试状态切换到 READY、TESTING、FAILED。

这不是普通 LED 闪烁函数，而是整个固件“是否还活着”的观察入口。

### 2.3 `safety_init(&real_safety_ops)`

位置：

```text
ecu/apps/ecu_board_test/src/safety_manager.c
```

作用：

- 安装真实硬件后端。
- 保证 DO、CAN 终端、RS485 方向控制进入安全状态。
- 所有危险资源都必须通过 Safety Manager。

`main.c` 里有三个真实后端适配函数：

```c
real_output_write()
real_can_term_write()
real_rs485_direction_write()
```

它们把安全层的逻辑动作转成板级 API。

### 2.4 `selftest_run_all()`

位置：

```text
ecu/apps/ecu_board_test/selftest/selftest.c
```

作用：

- 上电后先跑软件回归测试。
- 当前自测项包括：
  - result
  - safety
  - cli_runner
  - algorithms
  - status_led
  - periodic_tx
  - sbus_service
  - debug_monitor

这些自测不等于板级硬件全测通过。它们主要防止代码逻辑、格式化输出、安全清理、SBUS 解析等软件行为被改坏。

### 2.5 恢复真实后端

`selftest_run_all()` 中很多测试会安装 fake backend。例如：

- `selftest_safety.c` 会替换 Safety 后端。
- `selftest_status_led.c` 会替换 RGB 适配器。
- `selftest_periodic_tx.c` 会替换周期通信后端。
- `selftest_debug_monitor.c` 会替换调试监控后端。

所以 `main.c` 在自测后必须恢复：

```c
safety_init(&real_safety_ops);
status_led_init_default();
periodic_tx_init_default();
sbus_service_init();
ecu_debug_monitor_init();
```

这里的顺序很重要。尤其是 SBUS：`sbus_service_init()` 放在自测之后，是为了避免真实 UART1 中断和自测注入字节同时改 `g_sbus_debug_state`。

### 2.6 `app_shell_run()`

位置：

```text
ecu/apps/ecu_board_test/src/app_shell.c
```

作用：

- 进入 UART0 命令行。
- 解析 `help/list/board/run/status/report/abort/SELFTEST.ALL`。
- 根据测试注册表执行单项或全项测试。

这之后 `main()` 不再返回。

---

## 3. 前台循环：为什么没有 RTOS 也能同时跑灯、SBUS、调试监控

本固件没有 RTOS。主要靠前台轮询 + 少量中断。

UART0 命令行等待输入的位置是：

```text
ecu/apps/ecu_board_test/src/operator_io.c
```

核心函数：

```c
operator_read_line()
```

它在等待串口输入时，不是死等，而是每 1 ms 做一次：

```text
status_led_poll()
periodic_tx_poll()
sbus_service_poll()
ecu_debug_monitor_poll()
```

所以你看到的行为是：

- 绿灯可以持续心跳。
- 如果启用了周期 CAN/RS485/RS232，可以在 CLI 等待时发送。
- SBUS 连接状态可以根据超时更新。
- 调试串口可以周期打印 SBUS/ADC/DI/DO。

SBUS 字节接收本身不是靠这个 1 ms 轮询，而是 UART1 中断；`sbus_service_poll()` 只负责刷新 `connected` 这种基于时间的状态。

---

## 4. 命令行和测试执行链路

命令行从 `app_shell_run()` 开始。

简化链路：

```text
app_shell_run()
  -> operator_read_line()
  -> cli_parse()
  -> test_registry_find() / test_registry_all()
  -> run_descriptor() / run_all()
  -> test_runner_execute()
       -> prepare()
       -> execute()
       -> cleanup()
       -> safety_all_off()
  -> result_writer_print()
  -> test_session_add()
  -> status_led_set()
```

关键文件：

| 文件 | 作用 | 你读代码时看什么 |
|---|---|---|
| `src/app_shell.c` | CLI 主循环、单项/全项测试调度 | `app_shell_run()`、`run_all()`、`run_descriptor()` |
| `src/cli.c` | 把字符串命令解析成枚举 | `cli_parse()` 如何识别 `run xxx` |
| `src/operator_io.c` | UART0 行输入、确认、数字输入 | 为什么等待输入时还能轮询后台服务 |
| `src/test_registry.c` | 所有测试 ID、依赖、required/optional | `registry[]` 的顺序和依赖 |
| `src/test_runner.c` | prepare/execute/cleanup 生命周期 | 所有退出路径都 `safety_all_off()` |
| `src/result_writer.c` | 文本结果和 JSON 单行结果输出 | 日志怎么形成 |
| `src/test_types.c` | 单项/整板状态聚合 | PASS/INCOMPLETE/FAIL/ABORTED 的规则 |

---

## 5. 测试注册表：所有正式板级测试从这里进

位置：

```text
ecu/apps/ecu_board_test/src/test_registry.c
```

当前注册表：

| ID | 必测/可选 | 依赖 | 执行函数 | cleanup |
|---|---|---|---|---|
| `SAFE.BOOT` | required | 无 | `test_safe_boot` | 无 |
| `PWR.RAILS` | required | `SAFE.BOOT` | `test_power` | 无 |
| `RGB.ALL` | required | `PWR.RAILS` | `test_rgb` | `test_rgb_cleanup` |
| `MEM.INTERNAL` | required | `PWR.RAILS` | `test_internal_ram` | 无 |
| `MEM.SDRAM` | required | `PWR.RAILS` | `test_sdram` | 无 |
| `MEM.QSPI` | required | `PWR.RAILS` | `test_qspi_flash` | 无 |
| `MEM.EEPROM` | required | `PWR.RAILS` | `test_eeprom` | 无 |
| `ADC.ALL` | required | `PWR.RAILS` | `test_adc` | 无 |
| `DI.ALL` | required | `PWR.RAILS` | `test_digital_input` | 无 |
| `SBUS.RX` | required | `PWR.RAILS` | `test_sbus` | 无 |
| `RS232.ALL` | required | `PWR.RAILS` | `test_rs232` | 无 |
| `RS485.ALL` | required | `PWR.RAILS` | `test_rs485` | `test_rs485_cleanup` |
| `CAN.ALL` | required | `PWR.RAILS` | `test_can` | `test_can_cleanup` |
| `DO.ALL` | required | `PWR.RAILS` | `test_digital_output` | `test_output_cleanup` |
| `DIO.LOOP` | required | `PWR.RAILS` | `test_dio_loopback` | `test_output_cleanup` |
| `ETH.SKIP_NO_DEVICE` | optional | 无 | `test_ethernet_skip` | 无 |

读这个文件时要重点看三件事：

1. 执行顺序。
2. 哪些测试依赖 `PWR.RAILS`。
3. 哪些测试需要 cleanup。

`run all` 时，某个依赖没 PASS，后续依赖它的测试会 BLOCKED，而不是硬跑。

---

## 6. 安全层：所有危险输出的共同闸门

位置：

```text
ecu/apps/ecu_board_test/include/safety_manager.h
ecu/apps/ecu_board_test/src/safety_manager.c
```

Safety Manager 管理三类资源：

- 12 路 DO。
- 4 路 CAN 终端电阻。
- 3 路 RS485 方向控制。

主要 API：

| API | 作用 |
|---|---|
| `safety_init()` | 安装后端并全部关闭 |
| `safety_output_on()` | 打开一路 DO，要求同一时刻最多一路 |
| `safety_output_off()` | 关闭一路 DO |
| `safety_can_term_set()` | 控制 CAN 终端 |
| `safety_rs485_transmit()` | 设置 RS485 发送方向 |
| `safety_rs485_receive()` | 设置 RS485 接收方向 |
| `safety_all_off()` | 所有危险输出回安全状态 |
| `safety_snapshot()` | 给测试/调试查看状态 |

和其他模块的关系：

```text
tests/test_digital_output.c
tests/test_dio_loopback.c
tests/test_can.c
tests/test_rs485.c
        -> safety_manager.c
             -> board.c
```

你读测试用例时，只要看到“会让外部硬件动作”的地方，就应该检查它有没有走 Safety Manager，退出时有没有 cleanup。

---

## 7. 板级 BSP：硬件映射都在这里

主要文件：

```text
ecu/boards/ecu_isolation/board.h
ecu/boards/ecu_isolation/board.c
ecu/boards/ecu_isolation/pinmux.c
ecu/boards/ecu_isolation/pinmux.h
```

### 7.1 `board.h`

这是硬件映射总表。先读它可以建立“软件名字对应哪个硬件”的概念。

重点看：

- UART0 调试串口。
- UART1 SBUS。
- UART10/11/12 RS485。
- UART5/13/14/15 RS232。
- CAN0..CAN3 对应外部 CAN1..4。
- 12 路 DI。
- 12 路 DO。
- 4 路 ADC3 通道。
- RGB 三色 GPIO。
- SDRAM、QSPI、EEPROM、Ethernet。

### 7.2 `board.c`

这里是板级 API 的真实实现。应用层不直接写寄存器，而是尽量调用这里的函数。

重点函数：

| 函数 | 作用 |
|---|---|
| `board_init()` | 初始化板卡基础环境 |
| `board_init_safe_gpio()` | 初始化安全 GPIO 状态 |
| `board_rgb_write()` | 逻辑方式控制 RGB |
| `board_ecu_output_write()` | 逻辑方式控制一路 DO |
| `board_ecu_input_read()` | 逻辑方式读取一路 DI |
| `board_set_can_termination()` | 控制 CAN 终端 |
| `board_init_uart()` | 根据 UART 实例初始化对应 pinmux 和 clock |
| `board_init_can()` | 初始化 CAN pinmux/clock |
| `board_init_adc16_pins()` | 初始化 ADC 引脚 |

### 7.3 `pinmux.c/.h`

这是 HPM Pinmux Tool 生成的文件。不要优先手改，除非你明确知道生成工具会覆盖什么。

当前工程里要特别注意 CAN 终端 GPIO 的安全默认电平。重新生成 pinmux 前，需要确认相关 GPIO 默认输出电平仍是安全的。

---

## 8. 状态灯模块：不用串口也能判断程序状态

文件：

```text
ecu/apps/ecu_board_test/include/status_led.h
ecu/apps/ecu_board_test/src/status_led.c
```

状态定义：

| 状态 | 灯效 | 含义 |
|---|---|---|
| `STATUS_LED_BOOTING` | 红灯常亮 | 初始化/自测中 |
| `STATUS_LED_READY` | 绿灯 1 Hz | CLI 可用，主循环活着 |
| `STATUS_LED_TESTING` | 蓝灯快闪 | 正在执行测试 |
| `STATUS_LED_FAILED` | 红灯快闪 | 自测/测试/中止失败 |

关联：

```text
main.c
app_shell.c
test_runner.c
operator_io.c
tests/test_rgb.c
        -> status_led.c
             -> board_rgb_write()
```

`tests/test_rgb.c` 会临时 override RGB，让人工确认红绿蓝。cleanup 后恢复原状态。

---

## 9. 目标侧自测：防止软件逻辑回退

目录：

```text
ecu/apps/ecu_board_test/selftest/
```

入口：

```text
selftest/selftest.c
```

每个自测文件的作用：

| 文件 | 作用 | 关联模块 |
|---|---|---|
| `selftest.c` | 注册并顺序执行所有自测 | `selftest.h` |
| `selftest_result.c` | 验证整板状态聚合规则 | `test_types.c` |
| `selftest_safety.c` | 验证安全层初态、互锁、后端恢复 | `safety_manager.c` |
| `selftest_cli_runner.c` | 验证 CLI、abort、runner cleanup | `cli.c`、`test_runner.c`、`app_shell.c` |
| `selftest_algorithms.c` | 验证纯算法和注册表基本规则 | `adc_math.c`、`comm_pattern.c`、`memory_patterns.c`、`sbus_decoder.c` |
| `selftest_status_led.c` | 验证 RGB 状态机和 override | `status_led.c` |
| `selftest_periodic_tx.c` | 验证周期通信调度/格式/暂停恢复 | `periodic_tx.c` |
| `selftest_sbus_service.c` | 验证 SBUS 服务解析、连接状态、超时 | `sbus_service.c` |
| `selftest_debug_monitor.c` | 验证调试监控输出格式、周期、DO mask | `debug_monitor.c` |

读法：

1. 先读 `selftest.c`，知道顺序。
2. 再读每个 selftest，看它锁定了什么行为。
3. 回到被测模块，理解为什么实现要这样写。

这套 selftest 是你后面改代码的安全网。比如刚才 SBUS 服务和调试监控修改后，板上 `SELFTEST.ALL` 必须是 `pass=8 fail=0`。

---

## 10. 调试监控：SEGGER/J-Link 实时改变量，COM9 看输出

文件：

```text
ecu/apps/ecu_board_test/include/debug_monitor.h
ecu/apps/ecu_board_test/src/debug_monitor.c
```

核心全局变量：

```c
volatile ecu_debug_monitor_t g_ecu_debug_monitor;
```

字段：

| 字段 | 含义 |
|---|---|
| `enable` | 1 启用周期打印，0 关闭 |
| `view` | 1 SBUS，2 ADC，3 DI，4 DO，5 ALL |
| `channel` | 0 全部通道；否则选择一个一基通道 |
| `period_ms` | 打印周期，最小 50 ms |
| `do_enable` | 1 让 DO 跟随 `do_mask` |
| `do_mask` | 12-bit DO 输出掩码 |

当前符号地址示例：

```text
g_ecu_debug_monitor = 0x01080020
```

实际地址以每次构建后的 `nm` 输出为准。

打印内容：

- SBUS：连接状态、帧计数、错误数、16 通道、lost/failsafe。
- ADC：4 路 mV；读失败显示 `ERR`。
- DI：12 路 `0/1`。
- DO：12 路 `0/1` 和 mask。

关联：

```text
operator_io.c
  -> ecu_debug_monitor_poll()
       -> sbus_service_get_snapshot()
       -> ADC one-shot
       -> board_ecu_input_read()
       -> board_ecu_output_write()
```

注意：

- 正式测试执行期间，`app_shell.c` 会 suspend debug monitor，避免它和测试争 DO。
- suspended 时 DO 会被强制写 0。
- 现场调试结束后要把 `enable/do_enable/do_mask` 清零。

---

## 11. SBUS：中断接收 + 状态结构

文件：

```text
ecu/apps/ecu_board_test/include/sbus_service.h
ecu/apps/ecu_board_test/src/sbus_service.c
ecu/apps/ecu_board_test/include/sbus_decoder.h
ecu/apps/ecu_board_test/src/sbus_decoder.c
```

分工：

| 文件 | 作用 |
|---|---|
| `sbus_decoder.*` | 只负责把 25 字节 SBUS 帧解成 16 通道和状态位 |
| `sbus_service.*` | 负责 UART1 配置、中断接收、组帧、连接状态、调试结构体 |

核心全局变量：

```c
volatile sbus_debug_state_t g_sbus_debug_state;
```

字段：

| 字段 | 含义 |
|---|---|
| `connected` | 100 ms 内收到有效帧则为 1 |
| `frame_count` | 有效帧计数 |
| `decode_error_count` | 25 字节候选帧解码失败次数 |
| `uart_error_count` | UART 线错误/FIFO 错误次数 |
| `last_frame_ms` | 最后一帧有效帧时间 |
| `channels[16]` | 16 个 SBUS 通道 |
| `frame_lost` | SBUS lost 标志 |
| `failsafe` | SBUS failsafe 标志 |
| `channel17/channel18` | SBUS 数字通道 |

当前符号地址示例：

```text
g_sbus_debug_state = 0x01080220
```

执行链路：

```text
sbus_service_init()
  -> board_init_uart(UART1)
  -> uart_init(100000, 8E2)
  -> enable RX data/timeout, line status, RX idle interrupt
  -> intc_m_enable_irq_with_priority(IRQn_UART1)

UART1 ISR
  -> drain UART RX FIFO
  -> ingest_byte()
  -> collect 25-byte frame
  -> sbus_decode()
  -> update g_sbus_debug_state

foreground loop
  -> sbus_service_poll()
  -> update connected timeout
```

为什么这么设计：

- SBUS 是连续流，不能靠 200 ms 调试打印时再去 UART FIFO 里读。
- UART1 中断负责“不断收”，调试监控只负责“读结构体并打印”。
- 这样 COM9 打印慢也不会导致 SBUS FIFO 溢出。

---

## 12. 周期通信：CAN/RS485/RS232 自动发帧

文件：

```text
ecu/apps/ecu_board_test/include/periodic_tx.h
ecu/apps/ecu_board_test/src/periodic_tx.c
ecu/apps/ecu_board_test/src/periodic_tx_board.c
```

分工：

| 文件 | 作用 |
|---|---|
| `periodic_tx.c` | 硬件无关调度、计数、格式 |
| `periodic_tx_board.c` | HPM6750 CAN/UART 硬件后端 |

构建宏：

```text
ECU_PERIODIC_CAN_TX
ECU_PERIODIC_RS485_TX
ECU_PERIODIC_RS232_TX
```

默认关闭。需要周期发送测试时，在构建脚本里启用。

运行位置：

```text
operator_io.c -> periodic_tx_poll()
```

正式注册测试执行时：

```text
app_shell.c -> periodic_tx_suspend()
测试结束 -> periodic_tx_resume()
```

这样周期诊断帧不会污染 RS232/RS485/CAN 回环测试。

---

## 13. 纯算法模块：硬件无关，优先用 selftest 保证

| 文件 | 作用 | 被谁使用 |
|---|---|---|
| `adc_math.h/.c` | ADC 原始值和外部电压换算 | `test_adc.c`、`debug_monitor.c`、`selftest_algorithms.c` |
| `comm_pattern.h/.c` | RS232/RS485 测试帧编码、解码、CRC | `test_serial_common.c`、`selftest_algorithms.c` |
| `memory_patterns.h/.c` | RAM/SDRAM 破坏性图案测试 | `test_memory.c`、`selftest_algorithms.c` |
| `sbus_decoder.h/.c` | SBUS 25 字节帧解包 | `sbus_service.c`、`selftest_algorithms.c` |

读法建议：

- 这些文件先看 `.h`，再看 `.c`。
- 它们不应该直接依赖板级硬件。
- 对应 selftest 能帮助你理解输入输出预期。

---

## 14. 正式硬件测试用例

目录：

```text
ecu/apps/ecu_board_test/tests/
```

每个 `test_*.c` 都对应一个或多个注册表 ID。

| 文件 | 注册 ID | 做什么 | 重点风险 |
|---|---|---|---|
| `test_boot.c` | `SAFE.BOOT` | 检查安全初态 | 只证明软件安全状态，不替代万用表测量 |
| `test_power.c` | `PWR.RAILS` | 人工输入 12/5/3.3/1.8/1.1 V | 输入单位是 mV |
| `test_rgb.c` | `RGB.ALL` | 人工确认红绿蓝 | 使用 RGB override，cleanup 恢复 |
| `test_memory.c` | `MEM.INTERNAL`、`MEM.SDRAM`、`MEM.QSPI`、`MEM.EEPROM` | RAM/SDRAM/QSPI/EEPROM 测试 | RAM/SDRAM/QSPI 有破坏性边界 |
| `test_adc.c` | `ADC.ALL` | 4 路 ADC 多点输入测试 | 需要外部电压源 |
| `test_digital_input.c` | `DI.ALL` | 12 路 DI 0/12 V 人工确认 | 需要夹具输入 |
| `test_digital_output.c` | `DO.ALL` | 12 路 DO 逐路输出 | 必须受 Safety 管理，输出时间有限 |
| `test_dio_loopback.c` | `DIO.LOOP` | DO 到 DI 回环夹具 | 同时只开一路 DO |
| `test_can.c` | `CAN.ALL` | 4 路 Classic CAN | 不测 CAN-FD |
| `test_rs232.c` | `RS232.ALL` | 4 路 RS232 回环 | 115200 8N1 |
| `test_rs485.c` | `RS485.ALL` | 3 路 RS485 回环 | 硬件 DE |
| `test_sbus.c` | `SBUS.RX` | 通过 `sbus_service` 验证 100 帧和停发 | 接收段自动，停发需要人工确认 |
| `test_ethernet.c` | `ETH.SKIP_NO_DEVICE` | 当前固定 SKIP | 因为无设备，不得报 PASS |
| `test_serial_common.c/.h` | 内部公共逻辑 | RS232/RS485 共用 UART 回环帧 | 不是注册项 |

读测试用例时建议按注册表顺序读，不要按文件名顺序读。

---

## 15. 构建、链接、下载和 SES 工程

### 15.1 应用构建入口

```text
ecu/apps/ecu_board_test/CMakeLists.txt
```

作用：

- 指定三套链接脚本。
- 注册所有 `.c` 源文件。
- 注册 include 路径。
- 定义周期通信构建宏。
- 生成 SEGGER Embedded Studio 工程。

新增 `.c` 文件时必须在这里 `sdk_app_src()`。

### 15.2 链接脚本

| 文件 | 工具链 | 作用 |
|---|---|---|
| `linkers/gcc/ecu_board_test_linker.ld` | GNU | XIP/ILM/DLM/AXI 布局，保留末尾 Flash 区 |
| `linkers/iar/ecu_board_test_linker.icf` | IAR | IAR 等价布局 |
| `linkers/segger/ecu_board_test_linker.icf` | SES | SEGGER 等价布局 |

三个链接脚本要保持内存布局一致。特别是 QSPI Flash 末尾保留区，不能被固件镜像占用。

### 15.3 脚本

| 文件 | 作用 |
|---|---|
| `ecu/scripts/build_ecu_test.ps1` | 使用 SDK 1.11 和 GNU 工具链构建 |
| `ecu/scripts/generate_ses_ecu_test.ps1` | 生成 SES 8.28 工程 |
| `ecu/scripts/flash_ecu_test.ps1` | J-Link 下载 ELF，并检查下载证据 |
| `ecu/scripts/load_ecu_test.jlink` | J-Link Commander 命令模板 |
| `ecu/scripts/read_ecu_info.jlink` | 只读芯片/OTP/XIP 信息 |
| `ecu/scripts/serial_console.ps1` | 打开 UART0 调试串口并记录日志 |

常用命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\flash_ecu_test.ps1
```

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\generate_ses_ecu_test.ps1
```

---

## 16. 夹具和人工流程

目录：

```text
test/fixtures/
```

| 文件 | 作用 |
|---|---|
| `ecu-board-test-wiring.md` | 接线、夹具、外设连接说明 |
| `ecu-board-test-procedure.md` | 人工执行顺序和判据 |

这些文件和源码同等重要。板级测试不是“编译通过就是通过”，必须按夹具流程保存串口日志和测量证据。

---

## 17. 各文件夹和文件职责总表

### 17.1 `ecu/apps/ecu_board_test/include/`

| 文件 | 对应实现 | 作用 |
|---|---|---|
| `adc_math.h` | `src/adc_math.c` | ADC 电压换算 API |
| `app_shell.h` | `src/app_shell.c` | CLI shell 入口和测试结果到 LED 状态映射 |
| `cli.h` | `src/cli.c` | CLI 命令结构和解析结果枚举 |
| `comm_pattern.h` | `src/comm_pattern.c` | RS232/RS485 测试帧和 CRC |
| `debug_monitor.h` | `src/debug_monitor.c` | 调试器控制结构和监控后端接口 |
| `memory_patterns.h` | `src/memory_patterns.c` | RAM/SDRAM 测试图案 |
| `operator_io.h` | `src/operator_io.c` | UART0 输入/确认/数字读取 |
| `periodic_tx.h` | `src/periodic_tx.c`、`src/periodic_tx_board.c` | 周期通信调度和后端接口 |
| `result_writer.h` | `src/result_writer.c` | 测试结果文本/JSON 输出 |
| `safety_manager.h` | `src/safety_manager.c` | 危险输出安全管理 |
| `sbus_decoder.h` | `src/sbus_decoder.c` | SBUS 帧解码 |
| `sbus_service.h` | `src/sbus_service.c` | SBUS 中断接收服务和调试状态 |
| `selftest.h` | `selftest/*.c` | 目标侧自测声明 |
| `status_led.h` | `src/status_led.c` | RGB 状态机 |
| `test_cases.h` | `tests/test_*.c` | 正式测试用例声明 |
| `test_limits.h` | 多个测试文件 | 电压/时间/数量限制 |
| `test_registry.h` | `src/test_registry.c` | 测试注册表查询 |
| `test_runner.h` | `src/test_runner.c` | 测试生命周期和 abort |
| `test_types.h` | `src/test_types.c` | 测试状态、会话聚合 |

### 17.2 `ecu/apps/ecu_board_test/src/`

| 文件 | 主要职责 | 上游 | 下游 |
|---|---|---|---|
| `main.c` | 程序入口、初始化、进入 shell | 启动代码 | BSP、Safety、Selftest、Shell |
| `app_shell.c` | CLI 调度、run all、结果聚合 | `main.c` | CLI、Registry、Runner、Result、Safety |
| `cli.c` | 字符串命令解析 | `app_shell.c` | 无 |
| `operator_io.c` | UART0 行输入和后台轮询 | `app_shell.c`、测试用例 | LED、Periodic、SBUS、Monitor |
| `test_runner.c` | 测试生命周期、abort 识别、安全退出 | `app_shell.c` | Safety、Status LED |
| `test_registry.c` | 测试目录和依赖关系 | `app_shell.c` | `tests/test_*.c` |
| `test_types.c` | 会话状态聚合 | `app_shell.c` | 无 |
| `result_writer.c` | 输出 RESULT 和 JSON | `app_shell.c` | UART0/printf |
| `safety_manager.c` | DO/CAN 终端/RS485 安全控制 | 测试、Shell、Runner | `board.c` |
| `status_led.c` | RGB 状态机 | Main、Shell、Runner、Operator | `board_rgb_write()` |
| `periodic_tx.c` | 周期通信调度 | Operator | `periodic_tx_board.c` |
| `periodic_tx_board.c` | CAN/UART 周期发送硬件后端 | `periodic_tx.c` | Board、HPM CAN/UART |
| `debug_monitor.c` | 调试结构体控制和周期打印 | Operator | SBUS、ADC、DI、DO |
| `sbus_service.c` | UART1 SBUS 中断接收和状态发布 | Main、Operator、Monitor、Test | UART1、Decoder |
| `sbus_decoder.c` | SBUS 25 字节帧解包 | SBUS service、自测 | 无 |
| `adc_math.c` | ADC 固定点换算 | ADC 测试、Monitor、自测 | 无 |
| `comm_pattern.c` | 串口帧编码/解码/CRC | RS232/RS485 测试、自测 | 无 |
| `memory_patterns.c` | RAM 图案测试 | Memory 测试、自测 | 无 |

### 17.3 `ecu/apps/ecu_board_test/tests/`

| 文件 | 主要职责 |
|---|---|
| `test_boot.c` | 安全启动状态检查 |
| `test_power.c` | 电源轨人工输入和范围判断 |
| `test_rgb.c` | RGB 人工确认和 override cleanup |
| `test_memory.c` | 内部 RAM、SDRAM、QSPI、EEPROM 测试 |
| `test_adc.c` | 4 路 ADC 多点测试 |
| `test_digital_input.c` | 12 路 DI 测试 |
| `test_digital_output.c` | 12 路 DO 测试 |
| `test_dio_loopback.c` | DO-DI 回环测试 |
| `test_can.c` | 4 路 Classic CAN 测试 |
| `test_serial_common.c/.h` | RS232/RS485 共用回环逻辑 |
| `test_rs232.c` | 4 路 RS232 测试 |
| `test_rs485.c` | 3 路 RS485 测试和 cleanup |
| `test_sbus.c` | SBUS 接收和停发检查 |
| `test_ethernet.c` | 当前无设备，固定 SKIP |

### 17.4 `ecu/apps/ecu_board_test/selftest/`

| 文件 | 主要职责 |
|---|---|
| `selftest.c` | 自测注册和汇总 |
| `selftest_result.c` | 结果聚合规则 |
| `selftest_safety.c` | Safety Manager 规则 |
| `selftest_cli_runner.c` | CLI、Runner、abort、cleanup |
| `selftest_algorithms.c` | 纯算法和注册表基本约束 |
| `selftest_status_led.c` | RGB 状态机 |
| `selftest_periodic_tx.c` | 周期通信 |
| `selftest_sbus_service.c` | SBUS 服务 |
| `selftest_debug_monitor.c` | 调试监控 |

### 17.5 `ecu/boards/ecu_isolation/`

| 文件 | 主要职责 |
|---|---|
| `board.h` | 板级硬件映射和 BSP API 声明 |
| `board.c` | BSP API 实现、安全电平、时钟、外设初始化 |
| `pinmux.c` | 生成的引脚复用初始化 |
| `pinmux.h` | 生成的 pinmux 函数声明 |
| `pinmux.hpmpc` | Pinmux Tool 工程 |
| `ecu_isolation.yaml` | HPM SDK 板卡描述 |
| `ecu_isolation.cfg` | OpenOCD/参考配置 |
| `CMakeLists.txt` | 注册板级源码 |

### 17.6 `ecu/scripts/`

| 文件 | 主要职责 |
|---|---|
| `build_ecu_test.ps1` | 配置并构建 GNU 固件 |
| `generate_ses_ecu_test.ps1` | 生成 SES 8.28 工程 |
| `flash_ecu_test.ps1` | J-Link 下载并验证 transcript |
| `load_ecu_test.jlink` | J-Link 下载命令模板 |
| `read_ecu_info.jlink` | J-Link 只读芯片信息 |
| `serial_console.ps1` | COM 串口交互和日志 |

---

## 18. 推荐完整通读路线

下面是我建议你真正打开文件阅读的顺序。

### 第一轮：只读主流程

目标：知道程序怎么启动、怎么进入 CLI、怎么执行测试。

1. `ecu/apps/ecu_board_test/src/main.c`
2. `ecu/boards/ecu_isolation/board.h`
3. `ecu/boards/ecu_isolation/board.c`
4. `ecu/apps/ecu_board_test/selftest/selftest.c`
5. `ecu/apps/ecu_board_test/src/app_shell.c`
6. `ecu/apps/ecu_board_test/src/operator_io.c`
7. `ecu/apps/ecu_board_test/src/cli.c`
8. `ecu/apps/ecu_board_test/src/test_registry.c`
9. `ecu/apps/ecu_board_test/src/test_runner.c`
10. `ecu/apps/ecu_board_test/src/result_writer.c`
11. `ecu/apps/ecu_board_test/src/test_types.c`

第一轮读完，你应该能回答：

- 上电后为什么先跑 selftest？
- `run all` 怎么知道执行哪些测试？
- 测试失败后为什么能保证输出关闭？
- 为什么等串口输入时灯还会闪、监控还会打印？

### 第二轮：读安全和状态

目标：理解这套固件如何防止测试程序误伤外设。

1. `ecu/apps/ecu_board_test/include/safety_manager.h`
2. `ecu/apps/ecu_board_test/src/safety_manager.c`
3. `ecu/apps/ecu_board_test/include/status_led.h`
4. `ecu/apps/ecu_board_test/src/status_led.c`
5. `ecu/apps/ecu_board_test/tests/test_boot.c`
6. `ecu/apps/ecu_board_test/tests/test_rgb.c`
7. `ecu/apps/ecu_board_test/selftest/selftest_safety.c`
8. `ecu/apps/ecu_board_test/selftest/selftest_status_led.c`

第二轮读完，你应该能回答：

- DO 为什么同一时刻最多一路？
- cleanup 在哪里保证执行？
- RGB override 为什么不会永久抢占状态灯？

### 第三轮：读调试监控和 SBUS

目标：理解你现在用 SEGGER 实时改变量、COM9 看状态的机制。

1. `ecu/apps/ecu_board_test/include/debug_monitor.h`
2. `ecu/apps/ecu_board_test/src/debug_monitor.c`
3. `ecu/apps/ecu_board_test/include/sbus_service.h`
4. `ecu/apps/ecu_board_test/src/sbus_service.c`
5. `ecu/apps/ecu_board_test/include/sbus_decoder.h`
6. `ecu/apps/ecu_board_test/src/sbus_decoder.c`
7. `ecu/apps/ecu_board_test/tests/test_sbus.c`
8. `ecu/apps/ecu_board_test/selftest/selftest_sbus_service.c`
9. `ecu/apps/ecu_board_test/selftest/selftest_debug_monitor.c`
10. `docs/ecu-sbus-service-debug-notes.md`

第三轮读完，你应该能回答：

- 为什么 SBUS 必须用中断，而不是打印时轮询？
- `g_sbus_debug_state` 和 `g_ecu_debug_monitor` 分别是什么？
- 调试监控为什么不会和正式测试同时控制 DO？

### 第四轮：读各硬件测试

目标：理解每个测试项怎么判断 PASS/BLOCKED/FAIL。

按注册表顺序读：

1. `tests/test_power.c`
2. `tests/test_memory.c`
3. `tests/test_adc.c`
4. `tests/test_digital_input.c`
5. `tests/test_digital_output.c`
6. `tests/test_dio_loopback.c`
7. `tests/test_can.c`
8. `tests/test_serial_common.c`
9. `tests/test_rs232.c`
10. `tests/test_rs485.c`
11. `tests/test_ethernet.c`

第四轮读完，你应该能回答：

- 哪些测试需要人工输入？
- 哪些测试有破坏性？
- 哪些测试需要外部夹具？
- 哪些测试可以只靠软件自测证明一部分逻辑？

### 第五轮：读构建和工程文件

目标：知道怎么用 GNU/SES/J-Link 从源码到板上运行。

1. `ecu/apps/ecu_board_test/CMakeLists.txt`
2. `ecu/apps/ecu_board_test/app.yaml`
3. `ecu/apps/ecu_board_test/linkers/gcc/ecu_board_test_linker.ld`
4. `ecu/apps/ecu_board_test/linkers/segger/ecu_board_test_linker.icf`
5. `ecu/apps/ecu_board_test/linkers/iar/ecu_board_test_linker.icf`
6. `ecu/scripts/build_ecu_test.ps1`
7. `ecu/scripts/generate_ses_ecu_test.ps1`
8. `ecu/scripts/flash_ecu_test.ps1`
9. `ecu/scripts/serial_console.ps1`
10. `test/fixtures/ecu-board-test-wiring.md`
11. `test/fixtures/ecu-board-test-procedure.md`

第五轮读完，你应该能回答：

- 新增 `.c` 文件要在哪里注册？
- SES 工程从哪里生成？
- J-Link 下载如何判断不是假成功？
- 测试结果为什么必须保存串口日志？

---

## 19. 读代码时的几个关键原则

### 19.1 先看 `.h`，再看 `.c`

每个模块都先看头文件。头文件告诉你“别人应该怎么用它”，源文件告诉你“它内部怎么做”。

例如读 SBUS：

```text
sbus_service.h -> sbus_service.c -> sbus_decoder.h -> sbus_decoder.c
```

不要一开始就钻进中断细节。

### 19.2 看到硬件动作，立刻找 Safety 或 Board API

例如：

- DO 输出：找 `safety_output_on()` 或 `board_ecu_output_write()`。
- CAN 终端：找 `safety_can_term_set()` 或 `board_set_can_termination()`。
- RGB：找 `status_led_*()` 或 `board_rgb_write()`。
- UART：找 `board_init_uart()`。

如果某个测试绕过 Safety 直接控制危险输出，要重点警惕。

### 19.3 区分三种测试

| 类型 | 位置 | 作用 |
|---|---|---|
| 目标侧自测 | `selftest/` | 防软件逻辑回退 |
| 正式板级测试 | `tests/` | 生成板级测试结果 |
| 人工夹具流程 | `test/fixtures/` | 指导现场连接和判定 |

不要把 `SELFTEST.ALL pass=8 fail=0` 当成整板功能 PASS。它只说明固件内部逻辑自测通过。

### 19.4 任何“通过”都要看证据来源

有效证据包括：

- COM9 日志。
- J-Link transcript。
- CAN 分析仪记录。
- 串口回环结果。
- 万用表/示波器读数。
- 保存的 JSONL/文本结果。

编译通过、下载通过、灯亮，都不是完整功能测试 PASS。

---

## 20. 你可以按这个路线开始通读

实际阅读时，建议你先打开：

```text
ecu/apps/ecu_board_test/src/main.c
```

然后按下面的调用跳转：

```text
main.c
  -> board.h / board.c
  -> status_led.h / status_led.c
  -> safety_manager.h / safety_manager.c
  -> selftest.h / selftest.c
  -> periodic_tx.h / periodic_tx.c / periodic_tx_board.c
  -> sbus_service.h / sbus_service.c
  -> debug_monitor.h / debug_monitor.c
  -> app_shell.h / app_shell.c
       -> operator_io.h / operator_io.c
       -> cli.h / cli.c
       -> test_registry.h / test_registry.c
       -> test_runner.h / test_runner.c
       -> result_writer.h / result_writer.c
       -> test_types.h / test_types.c
       -> tests/test_*.c
```

读完这一条主线，再回头看：

```text
selftest/*.c
linkers/*
ecu/scripts/*
test/fixtures/*
```

这样读不会迷路：你始终知道“我现在看的文件是在启动链路、命令链路、测试链路、硬件抽象层，还是构建工具链里”。

