# ECU 板级全功能测试：上下文与进度

最后更新：2026-06-23

## 1. 项目背景

- 项目对象：旱田农业机械，四轮驱动、四轮转向。
- 行走、转向和四腿变地隙均使用伺服电机。
- 变行距使用液压缸。
- 伺服通信采用 CANopen。
- 外置角度等传感器通过 ADC 采集模块接入，ADC 模块与 ECU 使用 RS485 Modbus 通信。
- ECU 为自研板，核心 MCU 是 HPMicro HPM6750IVM，实际使用周立功 MR6750/MB6750 最小系统模块。
- 开发环境：HPM SDK 1.11.0、SEGGER Embedded Studio 8.28、J-Link。
- 当前已知状态：程序可以下载；其他板载功能尚未完成代码验证。

## 2. 当前阶段边界

本阶段只完成 **ECU 板级全功能测试**，不做外部设备或整车控制联调。

已确认：

- 测试深度：非破坏性功能验证。
- 使用场景：后续多块样板可重复执行。
- ECU 实物与 `doc/ECU/SCH_ECU_Isolation_原理图.pdf` V1.0 一致。
- ECU 额定供电：12 V；允许电压范围尚未给出，因此首轮只按 12 V 标称电压测试。
- 测试台架完整：具备可调电源、万用表、示波器、USB-CAN、USB-RS485、USB-RS232、以太网、信号源或负载。
- Ethernet 暂不测试，因为当前没有连接设备。测试结果记为 `SKIP_NO_DEVICE`，且不影响整板 PASS。

## 3. 工作空间资料盘点

### ECU 核心资料

- `doc/ECU/SCH_ECU_Isolation_原理图.pdf`：13 页，版本 V1.0，更新日期 2026-03-06。
- `doc/ECU/HPMicro-MR6750-L管脚对应表.xlsx`：91 行引脚映射；工作表内部名称为 `MR6450-L`，名称存在不一致，但映射内容与原理图中 HPM6750 模块信号相符。
- `doc/ECU/MR6750用户手册网址.txt`：`https://manual.zlg.cn/web/#/237/9276`。
- `doc/先楫半导体/HPM67006400UMV26用户手册.pdf`：HPM6700/6400 用户手册 Rev2.6，共 1249 页。
- `doc/先楫半导体/HPMsdk的参考资料网址.txt`：SDK 1.11.0 在线文档地址。

### 外部设备资料（后续阶段使用）

- 伺服驱动器 BC/BC2 手册。
- DC/DH CANopen 应用说明、DS301 和三份 EDS。
- BMS 产品规格、线束与整车通信协议。
- 48 V DCDC、12 V DCDC、DCAC 逆变通信资料。
- G20 遥控器说明书。

### 当前明显缺少的外设资料

- 外置 ADC 采集模块的型号、Modbus 寄存器表和通信参数。
- 液压执行系统/阀控模块的接口与控制资料。
- 整车 CANopen 节点 ID、波特率和设备映射表。
- ECU 线束定义及整车连接关系表。

以上缺口不阻塞本阶段 ECU 板级测试。

## 4. ECU 板载功能清单

根据原理图和现有板级代码，ECU 至少包含：

- 12 路隔离数字输入。
- 12 路隔离低边数字输出。
- 4 路板载模拟输入，接 ADC3 的 INA4～INA7。
- 4 路隔离 CAN-FD，均带软件可控终端电阻。
- 3 路隔离 RS485。
- 4 路隔离 RS232。
- 1 路隔离 SBUS 输入。
- 1 路千兆 Ethernet：RTL8211E，RGMII 连接 HPM6750 ENET1。
- 1 片 AT24C02 EEPROM，I2C3，地址 `0x50`。
- RGB LED。
- 隔离 JTAG 与隔离调试串口。
- 模块自带约 8 MB QSPI Flash 和 32 MB SDRAM（以当前板配置为准）。
- 复位按键、BOOT0/BOOT1 拨码。
- BAT_GND、MCU_DGND、ISO_GND 三个电气域，以及 MCU_5V、MCU_3V3、MCU_3V3A、ISO_5V、ISO_3V3 电源轨。

## 5. 现有代码状态

### 自定义板级目录

`ecu/ecu_isolation/` 当前包含：

- `board.c/.h`
- `pinmux.c/.h`
- `pinmux.hpmpc`
- `ecu_isolation.yaml`
- `CMakeLists.txt`

现有板级代码已定义或初始化：时钟、UART、I2C、CAN、CAN 终端、RS485、RS232、GPIO 输入输出、ADC、Ethernet、RGB、SDRAM 等。

安全默认值：

- 12 路输出默认关闭。
- 4 路 CAN 终端默认关闭。
- 数字输入按低有效处理。
- RGB 为低电平点亮。

### 应用程序

`ecu/sdk_env_v1.11.0/user_template/user_app/src/main.c` 当前仅调用 `board_init()` 后进入空循环，没有任何功能测试逻辑。

### 构建验证

已用 SDK 1.11.0 和随附 GNU 13.2.0 工具链，在隔离目录 `tmp/context_build` 中执行 `flash_sdram_xip` 构建验证：

- 自定义板 `ecu_isolation` 能被 SDK 正确识别。
- `board.c`、`pinmux.c` 和空应用均成功编译链接。
- 生成 `demo.elf` 成功。
- XPI0 使用约 61 KB。
- 生成了临时 SES 工程。

注意：工作空间根目录不是 Git 仓库，没有提交历史。自定义板目录缺少与板名匹配的 OpenOCD `.cfg` 文件，因此临时 SES 工程生成时提示无法生成 OpenOCD 调试配置；用户当前使用 J-Link 且已能下载程序。

## 6. 已确认的测试固件架构

采用单一、可重复使用的 **ECU Test Runner 固件**，不采用每项功能独立烧录的方式。

### 分层

1. PC 隔离调试串口终端：选择测试项、输入参数、保存日志。
2. Test Runner：安全启动、命令解析、测试调度、结果汇总。
3. 独立测试模块：Memory、GPIO、ADC、CAN、RS485、RS232、SBUS、RGB；Ethernet 接口保留但暂缓执行。
4. 外部台架：回环线束、USB 总线适配器、信号源、假负载和测量仪器。

### 执行模式

- 单项模式：研发定位指定接口。
- 整板模式：按固定顺序测试每块样板。

### 命令行草案

- `list`
- `run <id>`
- `run all`
- `abort`
- `status`
- `report`

启动日志应包含固件版本、板级版本、复位原因、时钟和安全状态。

### 结果记录

每项记录：板卡编号、固件版本、测试编号、PASS/FAIL/SKIP、实测值、阈值、错误码、耗时和备注。

串口同时输出操作员易读文本和可由 PC 脚本解析的结构化结果行。时间以 PC 日志时间为准，ECU 只提供启动后毫秒计时。

## 7. 已确认的测试顺序

1. 断电预检：外观、连接器、BOOT 拨码、12 V 输入阻抗和工装接线。
2. 安全启动：限流上电、调试串口、时钟、复位原因、RGB、默认输出状态。
3. 内部资源：片上 RAM、QSPI Flash、SDRAM、EEPROM。
4. 低能量 I/O：12 路 DI、4 路 ADC、SBUS。
5. 通信接口：RS232、RS485、CAN；Ethernet 暂缓。
6. 受控输出：12 路 DO 使用低电流假负载逐路短时测试。
7. 汇总与复位：生成报告、全部输出关闭、软件复位并复查安全启动。

基础启动或供电失败时，禁止继续执行后续测试。独立模块失败不阻止其他无依赖测试继续执行。

## 8. 测试覆盖矩阵

| ID | 分组 | 状态 | 核心方法 |
|---|---|---|---|
| PWR | 12 V 输入及各电源轨 | 必测 | 万用表/示波器记录电压和纹波 |
| BOOT | 上电、复位、BOOT、看门狗 | 必测 | 启动日志和复位原因 |
| LED | RGB | 必测 | 独立亮灭，操作员确认 |
| MEM | RAM、SDRAM、QSPI Flash、EEPROM | 必测 | 数据模式读写及校验 |
| ADC | 4 路板载模拟输入 | 必测 | 信号源分点注入 |
| DI | 12 路隔离输入 | 必测 | 逐路施加 12 V |
| DO | 12 路隔离低边输出 | 必测 | 逐路低电流带载测试 |
| CAN | 4 路隔离 CAN-FD | 必测 | 逐路收发、错误与恢复 |
| CAN-TERM | 4 路可控终端 | 必测 | 开关前后测量 H-L 电阻 |
| RS485 | 3 路隔离 RS485 | 必测 | 逐路双向收发 |
| RS232 | 4 路隔离 RS232 | 必测 | 逐路双向收发 |
| SBUS | 隔离 SBUS 输入 | 必测 | 合法帧、丢帧、失联 |
| DEBUG | JTAG 和隔离调试串口 | 必测 | 下载、调试、双向串口 |
| SAFE | 安全状态恢复 | 必测 | 正常、失败、超时、中止和复位路径 |
| ETH | RTL8211E/RGMII/ENET1 | 暂缓 | 记为 `SKIP_NO_DEVICE`，不影响整板 PASS |

## 9. 已确认的工装连接方案

- 12 V 可调电源限流供电，万用表和示波器监测电源轨。
- 制作 12 路 DI/DO 回环线束：DI 正端接 12 V，DI 负端通过对应 DO 下拉；另用低电流假负载抽测输出能力。
- 4 路 ADC 使用信号源逐路注入。
- 4 路 CAN 使用 USB-CAN 逐口测试，并测量终端电阻。
- 3 路 RS485 使用 USB-RS485 逐口测试。
- 4 路 RS232 使用 USB-RS232 逐口测试，或制作两两交叉回环线。
- SBUS 接遥控接收机或专用帧发生器。
- 隔离调试串口负责命令和日志，J-Link 负责烧录和调试。

## 10. 已确认的安全规则

- 上电后所有 DO 和 CAN 终端保持关闭。
- 有外部动作的测试必须由操作员显式启动。
- DO 禁止多路同时开启，逐路短时测试，完成后立即关闭。
- 每个测试必须有超时。
- 正常完成、失败、超时和 `abort` 都必须调用统一清理流程：关闭 DO、关闭 CAN 终端、RS485 切回接收状态。
- 测试模块互相隔离；单项失败不能遗留外设或 GPIO 状态。

## 11. 后续实施时需要固化的内容

- 12 V 输入的允许工作范围。
- 板卡编号的输入与保存策略。
- Ethernet 测试恢复条件及后续测试方法。

已确认补充决策：

- 结果状态采用 PASS、FAIL、SKIP、BLOCKED；整板状态采用 PASS、FAIL、INCOMPLETE、ABORTED。
- 不测试 CAN-FD；Classic CAN 使用 500 kbit/s。
- RS485、RS232 使用 115200 bit/s、8N1。
- 固件采用裸机事件循环、统一状态机和 Safety Manager。
- 工程从厂商 `user_template` 中独立出来，按 `boards/`、`apps/`、`scripts/` 组织。
- 正式设计规格位于 `docs/superpowers/specs/2026-06-23-ecu-board-functional-test-design.md`。

## 12. 下一步

1. 正式设计规格已获用户批准。
2. 实施计划位于 `docs/superpowers/plans/2026-06-23-ecu-board-functional-test-implementation.md`。
3. 下一步选择执行方式，并按计划开发测试固件和工装资料。
