# ECU 板级全功能测试：上下文与进度

最后更新：2026-06-23

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

## 已取得的证据

- 隔离工作树基线构建通过。
- 独立应用多次执行 `flash_sdram_xip` 干净构建通过。
- CMake 成功生成 SES 8.28 可打开的 `.emProject`。
- TDD 红灯已观察：加入自测后因 `test_types.h`、`safety_manager.h` 尚不存在而编译失败；实现后重新构建通过。
- 编译通过不能代表硬件 PASS；所有硬件结果仍需保存串口日志。

## 尚未取得的硬件证据

- 2026-06-23 最终检查时 J-Link 可枚举，但目标 `VTref=0.000V`，板卡未上电；下载脚本已正确返回失败而非假成功。
- Windows 未枚举出任何 COM 口，因此尚未完成串口 `SELFTEST.ALL` 运行和实物 HIL 全流程。
- 未保存任何板卡的 PASS JSONL，因此不得声称 ECU 已通过全功能硬件测试。
- 软件复位、看门狗复位、外部复位需要分别实测并核对 `RESET flags`。
- EEPROM 当前实现同次运行的备份、写读和恢复；断电保持/跨启动恢复仍需在 HIL 阶段验证并补强。

## 下一检查点

1. 清零编译警告并做静态检查、注册表审计、链接符号检查。
2. J-Link 下载并采集启动/自测日志。
3. 按 `test/fixtures/ecu-board-test-procedure.md` 接夹具执行 `run all`。
4. 只根据保存的 `.log`/`.jsonl` 标记 PASS、FAIL 或 BLOCKED。
