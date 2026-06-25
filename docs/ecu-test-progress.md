# ECU 板级全功能测试：上下文与进度

最后更新：2026-06-25

## 项目与硬件

- 旱田农机底盘：四轮驱动、四轮转向、四腿伺服变地隙，液压缸变行距。
- ECU：自研板，HPMicro HPM6750IVM，周立功 MR6750/MB6750 最小系统板。
- 开发环境：HPM SDK 1.11.0、SEGGER Embedded Studio 8.28、J-Link。
- ECU 额定输入 12 V。
- 本阶段只做 ECU 板级全功能测试，不做整车控制联调。

主要资料：

- `doc/ECU/SCH_ECU_Isolation_原理图.pdf`，V1.0，13 页。
- `doc/ECU/HPMicro-MR6750-L管脚对应表.xlsx`。
- `doc/ECU/MR6750用户手册网址.txt`。
- `doc/先楫半导体/HPM67006400UMV26用户手册.pdf`。

## 已锁定范围

- 12 路隔离 DI、12 路低边 DO。
- 4 路 ADC3：EX1/CH6、EX2/CH4、EX3/CH5、EX4/CH7。
- 4 路 Classic CAN，500 kbit/s；不测试 CAN-FD。
- 3 路 RS485、4 路 RS232，115200 bit/s、8N1。
- SBUS 100000 bit/s、8E2。
- AT24C02、32 MiB SDRAM、8 MiB QSPI Flash、RGB、复位和电源轨。
- Ethernet 当前无设备，固定为可选 `SKIP_NO_DEVICE`。

安全规则：上电后 DO 和 CAN 终端全关、RS485 为接收；DO 任意时刻最多一路导通且最长 500 ms；正常、失败、中止都统一清理。

## Git 与隔离工作树

- GitHub：`https://github.com/crazyyzq/agri_4wis_chassis_system`
- 主分支初始提交：`dd5a4a0`
- 实现分支：`codex/ecu-board-test`
- 隔离工作树：`D:\agri_4wis_chassis_system\.worktrees\ecu-board-test`
- SDK 目录很大且被 Git 忽略；工作树通过本地目录联接使用主工作区 SDK。

## 已实现

- 独立板目录 `ecu/boards/ecu_isolation` 和应用 `ecu/apps/ecu_board_test`。
- GNU、SEGGER、IAR 链接配置；8 MiB QSPI 最后 64 KiB 保留给破坏性 Flash 测试，GNU 链接结果为 `0x807F0000..0x80800000`。
- 可重复构建、SES 工程生成、J-Link 下载和串口记录脚本。
- 结果聚合、JSONL、CLI、Test Runner、Safety Manager。
- 目标侧自测：结果模型、安全互锁、CLI/Runner、ADC/内存/通信/SBUS 算法及注册表范围。
- 电源、RGB、片内 RAM、SDRAM、QSPI、EEPROM、ADC、DI、DO、DIO 回环测试。
- Classic CAN、RS485、RS232、SBUS 测试；CAN-FD 未注册。
- 原理图 U64 接线表和完整操作规程。
- RGB 运行状态：启动红灯常亮、READY 绿灯 1 Hz、测试蓝灯快闪、失败红灯快闪；`RGB.ALL` 可临时独占后恢复。
- 完成手写应用/板级/脚本静态审查和模块/公共 API 注释，通读指南见 `docs/ecu-board-test-code-reading-guide.md`。
- 审查修复：Runner 所有退出安全清理、abort 跨测试误匹配、SELFTEST 假后端恢复、DO 500 ms 上限、CAN 等待中止、SBUS 实际停流、QSPI 属性返回值、边界空指针和注册表容量保护。
- 生成 pinmux 中四路低有效 CAN 终端改为先写高电平锁存器再使能 OE；重新生成前必须在 Pinmux Tool 工程中把对应默认电平同步为 1。

## 已取得的证据

- 隔离工作树基线构建通过。
- 独立应用多次执行 `flash_sdram_xip` 干净构建通过。
- CMake 成功生成 SES 8.28 可打开的 `.emProject`。
- TDD 红灯已观察：加入自测后因 `test_types.h`、`safety_manager.h` 尚不存在而编译失败；实现后重新构建通过。
- 编译通过不能代表硬件 PASS；所有硬件结果仍需保存串口日志。
- 2026-06-24：J-Link V9.16 在 JTAG 4 MHz 下连接成功，VTref=3.283 V，检测到单个 RV32 TAP（ID `0x1000563D`）。
- 2026-06-24：`demo.elf` 下载成功；首次写入并校验 90,112 字节，复验显示 Flash 内容一致，复位后 MCU 正常运行。
- 2026-06-24：OTP Shadow 读得 Chip ID `0x20201341`、MAC0 `00:14:97:63:F2:19`、UUID words `F76E6CBB-440E0360-081180FF-C0B5E6E3`。
- 2026-06-24：RGB 状态机按 TDD 加入；缺少实现时链接失败，最小实现后 GNU 构建通过。
- 2026-06-24：代码审查修复后 GNU `flash_sdram_xip` 全量构建通过，XPI0 使用 93,288 B，保留区仍为 0 B image 占用。
- 2026-06-24：PowerShell 脚本语法解析通过；GNU/IAR/SES 三份链接文件均保留末尾 64 KiB；Classic CAN 配置确认 `enable_canfd=false` 且 DLC 仅 0/8。
- 2026-06-24：确认 SES 8.28 `emBuild` 必须使用 `-config Debug`；位置参数 `'ecu_board_test;Debug'` 会报 `-config must be specified`，指南已修正。
- 2026-06-24：删除旧构建目录后的 GNU 干净构建通过，共 89 步；XPI0 93,288 B，`ECU_TEST_FLASH` image 使用 0 B。
- 2026-06-24：SES 8.28 使用 `emBuild -config Debug -rebuild` 成功，生成 `demo.elf` 39,956,037 B、`demo.bin` 73,284 B、`demo.map` 253,699 B。
- 2026-06-24：GNU 和 SES ELF 均检查到保留区无加载节重叠；两者导出的 `__ecu_test_flash_start__/end` 都是 `0x807F0000/0x80800000`，且包含全部 `status_led_*` 和 `selftest_status_led` 符号。
- 2026-06-24：最新 GNU 固件由 J-Link V9.16 下载并校验成功，涉及 94,208 B；VTref 3.283 V、TAP ID `0x1000563D`。
- 2026-06-24：下载后 PC 位于 `operator_read_line()` 的 1 ms 时钟延时路径。运行间隔采样 GPIOE DO 得到 `0xA4 -> 0x84 -> 0x84 -> 0xA4 -> 0x84`；低有效绿灯位翻转、红蓝位保持关闭，证明固件已进入 READY 心跳。
- 2026-06-24：接入 COM9 后复现并修复会话完整性缺陷：过去只运行 `SAFE.BOOT` 就可能显示 `BOARD PASS`；现在只有 `run all` 声明并完成全部 required 项后才可能 PASS，临时单项会话保持 INCOMPLETE。
- 2026-06-24：COM9（USB Serial Device，115200 8N1）启动日志采集成功。时钟为 CPU0/1 816 MHz、AHB/AXI 200 MHz、MCHTMR 24 MHz、XPI0 133.333 MHz；复位标志 `0x00000010`。
- 2026-06-24：COM9 启动和重复执行 `SELFTEST.ALL` 均为 `pass=5 fail=0`；`help/list/status`、板号设置、未知测试处理、SAFE.BOOT 和 Ethernet SKIP 均符合预期。
- 2026-06-24：修复后的部分会话实测为 `BOARD INCOMPLETE serial=ECU-COM9 pass=1 fail=0 skip=1 blocked=0`，不再误报 PASS。
- 2026-06-24：在 `run all` 的首个电源输入处主动输入非数字，使未接外设安全阻塞；结果为 SAFE.BOOT PASS、PWR.RAILS 及其 14 个 required 路径 BLOCKED、Ethernet SKIP，汇总 `BOARD SUMMARY INCOMPLETE pass=1 fail=0 skip=1 blocked=14`，没有进入 DO/CAN/RS485 等危险执行路径。
- 2026-06-24：完成英文 Doxygen 函数契约审计：110/110 个公共函数声明均具备 `@brief`，全部参数和非 `void` 返回值标签匹配；另补充 2 个函数回调类型说明。
- 2026-06-24：对相对注释设计基线变更的 42 个 ECU `.c/.h` 文件剥离块注释、行注释和空白后逐文件比较，结果 `COMMENT_ONLY=PASS`；生成的 `pinmux.c/.h` 未改变，`git diff --check` 通过。
- 2026-06-24：注释增强后的 HPM SDK 1.11.0 GNU 全量构建通过（38 步），XPI0 使用 93,440 B，`ECU_TEST_FLASH` image 使用 0 B。
- 2026-06-24：注释增强后的 SEGGER Embedded Studio 8.28 工程重新生成并由 `emBuild -config Debug -rebuild` 构建通过；生成 `demo.elf` 39,957,764 B、`demo.bin` 73,476 B、`demo.map` 254,362 B。
- 2026-06-24：周期通信发送按 TDD 实现；新增自测首先因缺少 `periodic_tx.h` 正确编译失败，最小调度核心加入后 GNU 构建转绿。自测覆盖 499/500 ms 边界、CAN/ASCII 帧格式、暂停恢复、32 位时间回绕和单通道失败隔离。
- 2026-06-24：新增三个默认关闭、可独立启用的构建开关：`ECU_PERIODIC_CAN_TX`、`ECU_PERIODIC_RS485_TX`、`ECU_PERIODIC_RS232_TX`。普通关闭构建和三组全开 GNU 构建均通过；全开构建确认三个定义均为 `1`。
- 2026-06-24：三组全开的 SEGGER Embedded Studio 8.28 工程重新生成，并由 `emBuild -config Debug -rebuild` 构建通过。CAN 仍为 500 kbit/s Classic CAN，未启用 CAN-FD。
- 2026-06-24：三组全开的 GNU 固件由 J-Link V9.16 下载并校验成功，VTref=3.277 V、TAP ID `0x1000563D`，Flash download 涉及 98,304 字节。
- 2026-06-24：下载后经 COM9 执行 `SELFTEST.ALL`，结果为 `pass=6 fail=0`；新增 `periodic_tx` 自测 PASS。周期发送在 CLI 等待路径运行，注册测试执行期间暂停并在结束后重新初始化恢复。
- 2026-06-25：新增 SEGGER 调试结构体 `g_ecu_debug_monitor`。调试器可改 `enable/view/channel/period_ms/do_enable/do_mask` 选择打印 SBUS、ADC、DI、DO；ADC 以 mV 输出，DI/DO 以 0/1 输出，DO 由 `do_enable` 和 12-bit `do_mask` 保持控制，不自动脉冲。
- 2026-06-25：`debug_monitor` 按 TDD 实现；新增自测首先因缺少 `src/debug_monitor.c` 正确构建失败，最小核心加入后 GNU 构建转绿。自测覆盖首次打印、周期限速、SBUS 16 通道格式、ADC mV 格式、DI 0/1、DO mask 裁剪、通道选择和 suspend/resume。
- 2026-06-25：首次下载后板卡红灯快闪，COM9 复测定位为 `SELFTEST debug_monitor FAIL`。根因是 suspended 状态下 `ecu_debug_monitor_poll()` 仍先按 `do_mask` 写 DO，再返回；修复为 suspended 时只写 0 并立即返回。
- 2026-06-25：修复后默认 GNU 构建通过，XPI0 使用 101,776 B，`ECU_TEST_FLASH` image 使用 0 B；J-Link V9.16 下载校验成功，J-Link V13，VTref=3.277 V，TAP ID `0x1000563D`，Flash download 涉及 102,400 字节。
- 2026-06-25：修复后经 COM9 执行 `SELFTEST.ALL`，结果为 `pass=7 fail=0`，新增 `debug_monitor` 自测 PASS。使用 J-Link 写 `g_ecu_debug_monitor` 地址 `0x01080020` 为 DI view 后，COM9 实测周期输出 `DI EX_IN1=0 ... EX_IN12=0`；随后已将 `enable/do_enable/do_mask` 清零。

## 尚未取得的硬件证据

- COM9 已可用于调试串口并完成启动/CLI/自测冒烟验证，但尚未执行带夹具的完整 HIL。
- 尚未由现场人员或摄像头目视确认 RGB 实际发光；当前只有 GPIO 寄存器心跳证据。
- 未保存任何板卡的 PASS JSONL，因此不得声称 ECU 已通过全功能硬件测试。
- 软件复位、看门狗复位、外部复位需要分别实测并核对 `RESET flags`。
- EEPROM 当前实现同次运行的备份、写读和恢复；断电保持/跨启动恢复仍需在 HIL 阶段验证并补强。
- 尚未由外部分析仪保存本次周期发送帧：CAN1..4 的 `0x601..0x604`、RS485 1..3 和 RS232 1..4 当前仍为 UNVERIFIED；固件构建、自测和下载成功不能替代接口观测。
- 尚未由现场夹具确认 `debug_monitor` 的 SBUS 16 通道、ADC mV、DI 0/1 和 DO mask 实际电气结果；当前只有 COM9 软件路径与 DI 空载读数证据。

## 下一检查点

1. 由现场人员在 CAN 分析仪确认 CAN1..4 分别出现 `0x601..0x604`、DLC 8、约 500 ms 周期和递增计数。
2. 分别连接 RS485 1..3、RS232 1..4，确认 115200 8N1 ASCII 周期行。
3. 保存外部分析仪记录；未连接通道继续标记 UNVERIFIED。
4. 按 `test/fixtures/ecu-board-test-procedure.md` 接夹具执行 `run all`，只依据保存日志标记结果。
