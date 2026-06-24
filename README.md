# agri_4wis_chassis_system

旱田四轮驱动、四轮转向农业机械的 ECU 与底盘控制系统项目。

## 当前阶段

当前优先完成自研 ECU 的板级全功能测试。测试范围和实施步骤见：

- `docs/superpowers/specs/2026-06-23-ecu-board-functional-test-design.md`
- `docs/superpowers/plans/2026-06-23-ecu-board-functional-test-implementation.md`
- `docs/ecu-test-progress.md`
- `docs/ecu-board-test-code-reading-guide.md`（项目结构、调用链和逐文件通读顺序）

## 硬件与工具链

- MCU：HPMicro HPM6750IVM
- 模块：ZLG MR6750/MB6750 最小系统板
- SDK：HPM SDK 1.11.0
- IDE：SEGGER Embedded Studio 8.28
- 下载调试：J-Link

## 本地 SDK

厂商 SDK、GNU 工具链和辅助工具约 2.9 GiB，不纳入 Git 仓库。开发机应单独准备：

```text
ecu/sdk_env_v1.11.0/
```

SDK 参考地址记录在 `doc/先楫半导体/HPMsdk的参考资料网址.txt`。

## 目录

```text
doc/                         硬件手册、协议和原理图
docs/                        设计、计划和项目进度
ecu/apps/ecu_board_test/     ECU 板级测试应用
ecu/boards/ecu_isolation/   ECU 自定义板级文件
ecu/scripts/                 构建、SES、J-Link 和串口脚本
ecu/sdk_env_v1.11.0/         本地厂商 SDK（Git 忽略）
tmp/                         构建及预览文件（Git 忽略）
```
