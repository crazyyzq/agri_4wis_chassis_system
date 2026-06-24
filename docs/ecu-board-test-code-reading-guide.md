# ECU 板级测试固件：项目结构与代码通读指南

最后更新：2026-06-24

## 1. 先明确这套程序做什么

`ecu_board_test` 是 HPM6750 ECU 的板级产测/调试固件，不是整车控制程序。它通过 UART0 命令行组织测试，Safety Manager 负责危险输出清理，Test Runner 负责 prepare/execute/cleanup 生命周期，RGB 灯在无串口时显示固件阶段。

当前边界：CAN 只测 Classic CAN；CAN-FD 不测；Ethernet 因无连接设备固定为可选 SKIP；任何编译、下载或亮灯结果都不能替代夹具 HIL 结果。

## 2. 目录树

```text
agri_4wis_chassis_system/
├─ doc/                                  原理图、模块手册和通信协议
├─ docs/                                 设计、实施计划、进度和本指南
├─ ecu/
│  ├─ apps/ecu_board_test/
│  │  ├─ include/                        手写公共接口
│  │  ├─ src/                            手写框架与纯算法实现
│  │  ├─ tests/                          手写板级测试用例
│  │  ├─ selftest/                       上电目标侧回归自测
│  │  ├─ linkers/{gcc,iar,segger}/       三套工具链链接文件
│  │  ├─ CMakeLists.txt                  HPM SDK 构建入口
│  │  └─ app.yaml                        SDK 应用依赖
│  ├─ boards/ecu_isolation/
│  │  ├─ board.c / board.h               手写板级支持和硬件定义
│  │  ├─ pinmux.c / pinmux.h             HPM Pinmux Tool 生成文件
│  │  ├─ pinmux.hpmpc                    Pinmux Tool 工程源
│  │  ├─ ecu_isolation.yaml              SDK 板卡能力和内存参数
│  │  ├─ ecu_isolation.cfg               OpenOCD 板卡配置参考
│  │  └─ CMakeLists.txt                  板级源码注册
│  ├─ scripts/                           构建、SES、J-Link、串口脚本
│  └─ sdk_env_v1.11.0/                   本机 SDK/工具链，Git 忽略
├─ test/fixtures/                         接线表和人工测试规程
└─ tmp/                                   生成工程和构建产物，Git 忽略
```

`sdk_env_v1.11.0/` 和 `tmp/` 不是项目源码。不要通读 SDK；遇到具体 HPM API 时再跳转定义。

## 3. 两条主调用链

启动链：

```text
main
  -> board_init（时钟、UART0、安全 GPIO、PMP、SDRAM 启动支持）
  -> status_led_init_default + BOOTING（红灯常亮）
  -> safety_init（DO/CAN 终端/RS485 全部安全）
  -> selftest_run_all
  -> 恢复真实 Safety/RGB 后端并初始化可选 periodic_tx
  -> READY（绿灯 1 Hz）或 FAILED（红灯快闪）
  -> app_shell_run（永久前台循环）
```

测试链：

```text
operator_read_line
  -> cli_parse
  -> test_registry_find / test_registry_all
  -> app_shell 设置 TESTING（蓝灯快闪）
  -> test_runner_execute
       -> prepare（可选）
       -> execute
       -> cleanup（进入 execute 后保证调用）
       -> safety_all_off（所有退出路径）
  -> result_writer_print（文本 + JSON）
  -> test_session_add / test_session_status
  -> READY 或 FAILED
```

`operator_read_line()` 不再使用阻塞 `getchar()`；它轮询 UART0，并在等待字符时调用 `status_led_poll()` 和 `periodic_tx_poll()`，所以主循环是否存活可以从绿灯心跳判断。注册硬件测试开始前暂停周期发送，结束后重新初始化相关通信口并恢复，避免诊断帧污染回显测试。

## 4. RGB 状态表

| 状态 | 显示 | 含义 |
|---|---|---|
| `STATUS_LED_BOOTING` | 红灯常亮 | 已进入应用，正在初始化或执行目标侧自测 |
| `STATUS_LED_READY` | 绿灯 1 Hz | CLI 可用，前台循环仍在运行 |
| `STATUS_LED_TESTING` | 蓝灯 125 ms 半周期快闪 | 正在执行板级测试 |
| `STATUS_LED_FAILED` | 红灯 125 ms 半周期快闪 | 自测、板级测试或人工中止失败 |

`RGB.ALL` 会调用 `status_led_override_begin()` 临时取得三色灯所有权；Runner 的 cleanup 调用 `status_led_override_end()`，立即恢复测试前状态。状态机不占中断或额外定时器，时间基准是 MCHTMR0。

## 5. 推荐通读顺序

1. 板级定义：先看 `board.h`，对照原理图，再看 `board.c` 和生成 pinmux。
2. 启动与框架：`main.c`、`app_shell.c`、`cli.c`、`test_registry.c`、`test_runner.c`。
3. 安全与状态：`safety_manager.*`、`status_led.*`、`periodic_tx.*`、`test_types.*`、`result_writer.*`。
4. 纯算法：`adc_math.*`、`comm_pattern.*`、`memory_patterns.*`、`sbus_decoder.*`。
5. 目标侧自测：按 `selftest.c` 的注册顺序阅读其余自测文件。
6. 硬件用例：先无破坏性项目，再看 DO/DIO、通信和内存破坏性测试。
7. 构建与下载：CMake、三份 linker、PowerShell 和 J-Link 脚本。

## 6. 板级文件

| 文件 | 职责与入口 | 阅读重点 |
|---|---|---|
| `ecu/boards/ecu_isolation/board.h` | 所有 UART/CAN/I2C/ADC/GPIO/SDRAM/XPI 映射和板级 API | 一基通道编号、低有效 RGB、低有效 CAN 终端、12 路 IO、8 MiB Flash 和 32 MiB SDRAM假设 |
| `ecu/boards/ecu_isolation/board.c` | `board_init()`、外设时钟、GPIO 安全态、FEMC、ENET 辅助函数 | GPIO 锁存器先写安全电平再开启 OE；错误时的停机循环；Flash option 与 MR6750 型号必须一致 |
| `ecu/boards/ecu_isolation/pinmux.c` | Pinmux Tool 生成的引脚初始化 | 当前唯一人工安全修正是四路 CAN 终端：先写高再开 OE；重新生成可能覆盖 |
| `ecu/boards/ecu_isolation/pinmux.h` | 生成函数声明 | 只确认 `board.c` 调用的函数存在，不逐行修改 |
| `ecu/boards/ecu_isolation/pinmux.hpmpc` | Pinmux Tool 工程源 | 重新生成前把 PB06/PB03/PB01/PB04 的 `outputLevel` 改为 1；文件含 `clientKey/secretKey` 字段，公开前应向工具厂商确认它们是否只是工程元数据 |
| `ecu/boards/ecu_isolation/ecu_isolation.yaml` | SDK 识别的 SOC、J-Link device、RAM/Flash 能力 | `HPM6750xVMx`、SDRAM 32M/16bit、QSPI 8M |
| `ecu/boards/ecu_isolation/ecu_isolation.cfg` | OpenOCD/QSPI/SDRAM 配置参考 | 当前下载流程使用 J-Link；本文件大部分初始化命令仍为注释参考 |
| `ecu/boards/ecu_isolation/CMakeLists.txt` | 向 SDK 注册 `board.c` 和 `pinmux.c` | 双核条件宏和板级 include 路径 |

## 7. 框架、状态和算法文件

| 文件 | 职责与入口 | 依赖/阅读重点 |
|---|---|---|
| `ecu/apps/ecu_board_test/src/main.c` | `main()` | 安全初始化顺序；自测会临时替换后端，返回后必须恢复真实后端 |
| `ecu/apps/ecu_board_test/include/app_shell.h` | Shell 和灯态映射接口 | `app_shell_run()` 永不返回 |
| `ecu/apps/ecu_board_test/src/app_shell.c` | 命令分派、依赖执行、结果聚合 | 注册表最大 32 项防护；依赖只接受先前 PASS；`SELFTEST.ALL` 保存/恢复 Safety 后端 |
| `ecu/apps/ecu_board_test/include/cli.h` | 命令数据结构 | 单行最多 96 字符 |
| `ecu/apps/ecu_board_test/src/cli.c` | `cli_parse()` | 无动态内存；去首尾空白；参数复制长度和终止符 |
| `ecu/apps/ecu_board_test/include/operator_io.h` | 人机输入 API | 阻塞到完整输入，但内部持续轮询状态灯 |
| `ecu/apps/ecu_board_test/src/operator_io.c` | UART0 行编辑、确认和十进制输入 | 退格、本地回显、`uint32_t` 溢出检查、1 ms 空闲轮询 |
| `ecu/apps/ecu_board_test/include/test_runner.h` | descriptor/context/lifecycle API | `abort_match_length` 属于单次上下文；`timeout_ms` 是测试元数据，具体用例仍自行控制等待 |
| `ecu/apps/ecu_board_test/src/test_runner.c` | `test_runner_execute()`、`test_runner_poll_abort()` | 所有返回都 `safety_all_off()`；只识别精确小写 `abort` 字节序列 |
| `ecu/apps/ecu_board_test/include/test_registry.h` | 注册表查询 API | 返回静态只读存储 |
| `ecu/apps/ecu_board_test/src/test_registry.c` | 16 个测试的唯一顺序表 | required/optional、依赖、超时、cleanup；无 CAN-FD 注册项 |
| `ecu/apps/ecu_board_test/include/test_types.h` | 单项/整板状态和会话计数 | optional SKIP 不阻止整板 PASS；required SKIP 视为 INCOMPLETE；只有声明了 required 总数的完整运行才可能 PASS |
| `ecu/apps/ecu_board_test/src/test_types.c` | 会话初始化、计数和名称映射 | 板号最多保存 23 字符；ABORTED 优先于 FAIL；单项/临时会话保持 INCOMPLETE |
| `ecu/apps/ecu_board_test/include/safety_manager.h` | 危险资源的硬件无关接口 | 12/4/3 路均使用一基编号；同一时间最多一路 DO 导通 |
| `ecu/apps/ecu_board_test/src/safety_manager.c` | 后端安装、互锁、快照、全关 | 不完整回调表被拒绝；`safety_backend()` 只用于作用域恢复 |
| `ecu/apps/ecu_board_test/include/status_led.h` | 四状态和可注入硬件适配器 | `poll()` 非阻塞；override 为幂等单层所有权 |
| `ecu/apps/ecu_board_test/src/status_led.c` | MCHTMR 时间换算、独占颜色和闪烁相位 | 32 位毫秒回绕用无符号差值处理；切状态立即重置相位 |
| `ecu/apps/ecu_board_test/include/periodic_tx.h` | 500 ms 周期发送配置、后端和快照 API | 三组功能独立编译控制；suspend/resume 不清除通道计数 |
| `ecu/apps/ecu_board_test/src/periodic_tx.c` | 硬件无关调度和 CAN/ASCII 帧格式 | CAN ID `0x601..0x604`；所有通道独立计数；跨 32 位毫秒回绕 |
| `ecu/apps/ecu_board_test/src/periodic_tx_board.c` | HPM6750 CAN/UART/MCHTMR 后端 | CAN 非阻塞发送；RS485 使用硬件 DE；默认三个宏均关闭 |
| `ecu/apps/ecu_board_test/include/result_writer.h` | 单项结果和 JSON API | 输出缓冲区固定上限 384 字节 |
| `ecu/apps/ecu_board_test/src/result_writer.c` | 文本和单行 JSON 输出 | 引号、反斜杠转义；控制字符替换为空格；容量不足返回 `0x0104` |
| `ecu/apps/ecu_board_test/include/adc_math.h` | ADC 分压换算接口 | 单位全部是微伏 |
| `ecu/apps/ecu_board_test/src/adc_math.c` | 100k/56k 分压固定点换算 | 使用 64 位中间值，避免乘法溢出 |
| `ecu/apps/ecu_board_test/include/comm_pattern.h` | RS232/RS485 测试帧格式 | 最大 payload 32 字节，CRC-16/CCITT-FALSE |
| `ecu/apps/ecu_board_test/src/comm_pattern.c` | 生成、编码、解码、CRC | 严格要求接收长度等于帧声明长度 |
| `ecu/apps/ecu_board_test/include/memory_patterns.h` | RAM 破坏性模式 API | mismatch 可选，记录首个错误地址 |
| `ecu/apps/ecu_board_test/src/memory_patterns.c` | 0/1/AA/55/address/walking-one | walking-one 执行 32 次整区写读 |
| `ecu/apps/ecu_board_test/include/sbus_decoder.h` | SBUS 解码结果 | 16 路 11 位模拟通道和四个状态位 |
| `ecu/apps/ecu_board_test/src/sbus_decoder.c` | 25 字节帧解包 | 先验证长度和首尾标记，再访问固定索引 |
| `ecu/apps/ecu_board_test/include/test_limits.h` | 电压、时间、帧数统一常量 | DO 最大通电 500 ms；ADC 容差 100000 µV |
| `ecu/apps/ecu_board_test/include/test_cases.h` | 所有注册用例入口声明 | 有危险资源的测试必须同时检查 cleanup |

## 8. 目标侧自测文件

| 文件 | 覆盖内容 | 阅读重点 |
|---|---|---|
| `ecu/apps/ecu_board_test/include/selftest.h` | 自测类型、断言宏和声明 | 断言失败立即返回 false |
| `ecu/apps/ecu_board_test/selftest/selftest.c` | `selftest_run_all()` | 执行顺序和最终返回码 |
| `ecu/apps/ecu_board_test/selftest/selftest_result.c` | required/optional 聚合 | PASS 与 INCOMPLETE 边界 |
| `ecu/apps/ecu_board_test/selftest/selftest_safety.c` | 假后端、安全初态、DO 互锁 | 不完整后端必须被拒绝 |
| `ecu/apps/ecu_board_test/selftest/selftest_cli_runner.c` | CLI、灯态映射、abort、Runner cleanup | 独立 fake Safety；部分 `abort` 不可跨上下文拼接 |
| `ecu/apps/ecu_board_test/selftest/selftest_algorithms.c` | ADC、RAM、通信、SBUS、注册表 | 非法枚举、空查询和 CRC 边界 |
| `ecu/apps/ecu_board_test/selftest/selftest_status_led.c` | 灯态周期和 override | 使用假时间，不真实等待 500 ms |
| `ecu/apps/ecu_board_test/selftest/selftest_periodic_tx.c` | 周期、帧格式、暂停恢复、失败隔离 | 假时钟和假通信后端；验证 499/500 ms 边界及时间回绕 |

## 9. 硬件测试文件

| 文件 | 测试内容 | 阅读重点/风险 |
|---|---|---|
| `ecu/apps/ecu_board_test/tests/test_boot.c` | SAFE.BOOT | 只检查 Safety Manager 跟踪状态；实物电平仍需测量 |
| `ecu/apps/ecu_board_test/tests/test_power.c` | 12/5/3.3/1.8/1.1 V | 人工输入单位 mV 和允许范围 |
| `ecu/apps/ecu_board_test/tests/test_rgb.c` | 红绿蓝人工确认 | override 在入口取得、cleanup 释放 |
| `ecu/apps/ecu_board_test/tests/test_memory.c` | 内部 RAM、32 MiB SDRAM、QSPI、EEPROM | SDRAM/RAM 为破坏性；QSPI 只能写 `0x807F0000..0x80800000`；EEPROM 备份后恢复 |
| `ecu/apps/ecu_board_test/tests/test_adc.c` | 4 路、4 个外部电压点 | ADC3 16 位单次采样，每点 128 次均值和峰峰值 |
| `ecu/apps/ecu_board_test/tests/test_digital_input.c` | 12 路 DI 0/12 V | 每状态连续 20 次、2 ms 间隔稳定性 |
| `ecu/apps/ecu_board_test/tests/test_digital_output.c` | 12 路低边 DO | 必须使用 220 Ω/2 W 与万用表峰值保持；每路严格脉冲 500 ms 后关闭 |
| `ecu/apps/ecu_board_test/tests/test_dio_loopback.c` | DO 到匹配 DI 的夹具回环 | 每次只开一路，失败由 Runner cleanup 全关 |
| `ecu/apps/ecu_board_test/tests/test_can.c` | 4 路 Classic CAN 500 kbit/s | `enable_canfd=false`，DLC 只为 0/8，2000 ms 接收超时可中止，终端测 115..125 Ω |
| `ecu/apps/ecu_board_test/tests/test_serial_common.h` | UART 测试内部接口 | RS232/RS485 共用，不对其他模块开放 |
| `ecu/apps/ecu_board_test/tests/test_serial_common.c` | UART 初始化和 1000 帧回显 | 单字节 2000 ms 超时；帧 CRC/通道/序列/payload 全比较 |
| `ecu/apps/ecu_board_test/tests/test_rs232.c` | 4 路 115200 8N1 | 除数字回显外还需人工确认物理极性 |
| `ecu/apps/ecu_board_test/tests/test_rs485.c` | 3 路 115200 8N1 | Pinmux 使用 UART11/12/10 硬件 DE；Safety 后端的 GPIO DE 操作为空实现 |
| `ecu/apps/ecu_board_test/tests/test_sbus.c` | 100000 8E2、100 帧解码和停发 | 50 ms 无新字节才判定停流成功；信号反相是否由板上电路完成需实物确认 |
| `ecu/apps/ecu_board_test/tests/test_ethernet.c` | 固定返回 SKIP | 当前无 PHY/link 设备，不得改成 PASS |

## 10. 构建、链接和工具文件

| 文件 | 职责 | 阅读重点 |
|---|---|---|
| `ecu/apps/ecu_board_test/CMakeLists.txt` | 注册全部源码、链接文件和 IDE 生成 | 新增 `.c` 必须显式加入；SES 使用 J-Link 4 MHz |
| `ecu/apps/ecu_board_test/app.yaml` | 声明 SDK board GPIO LED 依赖 | 保持最小依赖 |
| `ecu/apps/ecu_board_test/linkers/gcc/ecu_board_test_linker.ld` | GNU XIP/ILM/DLM/AXI 布局 | 最后 64 KiB 从 XPI0 image 排除并导出起止符号 |
| `ecu/apps/ecu_board_test/linkers/iar/ecu_board_test_linker.icf` | IAR 等价布局 | 保留区必须与 GNU 一致 |
| `ecu/apps/ecu_board_test/linkers/segger/ecu_board_test_linker.icf` | SES 8.28 等价布局 | 区域必须命名 `FLASH`；保留区同样为 64 KiB |
| `ecu/scripts/build_ecu_test.ps1` | 固定 SDK/toolchain 环境并构建 GNU image | 输出实际名称是 `tmp/ecu_board_test_build/output/demo.elf` |
| `ecu/scripts/generate_ses_ecu_test.ps1` | 生成并校验 `.emProject` | 修复 SDK 1.11 模板重复属性；转发三个周期发送开关 |
| `ecu/scripts/flash_ecu_test.ps1` | J-Link 非交互下载 | 同时检查 VTref、Downloading file 和 O.K.，避免假成功 |
| `ecu/scripts/load_ecu_test.jlink` | 下载命令模板 | `__ECU_TEST_ELF__` 由 PowerShell 替换 |
| `ecu/scripts/read_ecu_info.jlink` | 读取芯片/OTP/XIP 头信息 | 只读，不代表固件功能测试通过 |
| `ecu/scripts/serial_console.ps1` | UART0 交互与日志追加 | COM 口必填；115200 8N1；Ctrl-C 退出 |
| `test/fixtures/ecu-board-test-wiring.md` | 夹具和接口接线 | 通电前逐项核对，不凭软件映射猜线 |
| `test/fixtures/ecu-board-test-procedure.md` | 人工执行顺序和判据 | 结果必须保存日志/JSON，未执行只能标 BLOCKED/NOT TESTED |

## 11. 固定构建和调试命令

在隔离工作树根目录执行。若 PowerShell 执行策略禁止脚本，使用下面的 `Bypass` 形式。

GNU 全量构建：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

默认构建不会自动发送通信帧。生成三组全部启用的诊断固件：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\ecu\scripts\build_ecu_test.ps1 `
  -PeriodicCanTx -PeriodicRs485Tx -PeriodicRs232Tx
```

三个开关可分别省略或启用。周期发送只用于接口观测，不构成任何硬件 PASS 证据。

生成 SES 8.28 工程：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\generate_ses_ecu_test.ps1
```

工程位置：

```text
tmp/ecu_board_test_build/segger_embedded_studio/ecu_board_test.emProject
```

命令行重建 SES Debug：

```powershell
& 'D:\Program Files\SEGGER\SEGGER Embedded Studio 8.28\bin\emBuild.exe' `
  -config Debug -rebuild `
  .\tmp\ecu_board_test_build\segger_embedded_studio\ecu_board_test.emProject
```

J-Link 下载：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\flash_ecu_test.ps1
```

串口记录示例：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\serial_console.ps1 `
  -Port COM5 -LogPath .\test\reports\ECU-001.log
```

当前 ECU 调试串口为 COM9、115200 8N1。目标侧自测和周期帧必须以本次下载后的新日志重新确认；未连接、未观测的接口仍只能标记 UNVERIFIED，不能写“硬件测试 PASS”。
