# agri_4wis_chassis_system

旱田四轮驱动、四轮转向农业机械 ECU 与底盘控制系统项目。

## 当前阶段

板级功能测试已经完成，主分支已经进入主控软件框架实现阶段。
当前代码已建立主控框架和可配置硬件适配层：业务逻辑只产生车辆级指令，CANopen、Modbus、DIO、ADC、UART 等硬件细节集中在 `config`、`drivers`、`protocol` 和 `devices` 层。

已完成验证的板级功能包括：

- RS485
- RS232
- CAN1-CAN4
- SBUS
- 数字输入/输出
- 模拟量输入

暂不纳入当前阶段的软件验证范围：

- Ethernet
- CAN-FD

## 硬件与工具链

- MCU：HPMicro HPM6750IVM
- 模块：ZLG MR6750/MB6750 最小系统板
- SDK：HPM SDK 1.11.0
- IDE：SEGGER Embedded Studio 8.28
- 下载调试：J-Link

本地厂商 SDK、工具链和辅助工具不纳入 Git 仓库，应单独准备：

```text
ecu/sdk_env_v1.11.0/
```

## 主要目录

```text
doc/                         硬件资料、需求文档、外设资料和原理图相关资料
docs/                        设计说明、实施计划、项目进度和代码阅读指南
ecu/apps/                    CPU0/CPU1 应用入口
ecu/common/                  通用类型和时间定义
ecu/config/                  全局配置、阈值、周期和优先级
ecu/control/                 运动控制和底盘调整控制逻辑
ecu/devices/                 面向 BMS、DC/DC、驱动、转向、升降、液压和警示灯的设备适配层
ecu/diag/                    诊断码定义
ecu/drivers/                 面向 CAN、DIO、ADC、UART、SBUS 等外设/中断的驱动服务层
ecu/ipc/                     CPU0/CPU1 快照和通信契约
ecu/os/                      FreeRTOS 任务编排
ecu/protocol/                协议层，例如 SBUS、CANopen、Modbus RTU
ecu/remote/                  遥控器输入解释和状态机
ecu/vehicle/                 指令仲裁、安全钳制、执行边界和车辆状态
ecu/ecu_isolation/           当前 ECU 自定义板级文件
tests/                       框架约束测试
tools/                       静态检查工具
```

## 从哪里开始读代码

建议按下面顺序通读：

1. `docs/ecu-main-control-architecture.md`
2. `ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c`
3. `ecu/os/src/ecu_tasks_cpu0.c`
4. `ecu/config/include/ecu_config.h`
5. `ecu/config/src/ecu_config.c`
6. `ecu/remote/include/remote_types.h`
7. `ecu/remote/src/remote_manager.c`
8. `ecu/vehicle/include/vehicle_types.h`
9. `ecu/vehicle/src/command_arbiter.c`
10. `ecu/vehicle/src/safety_manager.c`
11. `ecu/vehicle/src/vehicle_command_executor.c`
12. `ecu/devices/include/*.h`
13. `ecu/drivers/*/include/*.h`
14. `ecu/protocol/*/include/*.h`
15. `ecu/ipc/include/ipc_snapshot.h`

## 当前框架文档

- 主控框架阅读指南：`docs/ecu-main-control-architecture.md`
- 当前进度记录：`docs/ecu-main-control-progress.md`
- 主控框架实施计划：`docs/superpowers/plans/2026-06-27-ecu-main-control-framework.md`
- 硬件适配框架实施计划：`docs/superpowers/plans/2026-06-27-ecu-full-hardware-framework.md`
- 需求来源：`doc/ECU_Project_Implementation_v1.4.md`

## 可现场修改的猜测配置

所有当前需要现场确认的硬件值都应集中在 `ecu/config/include/ecu_config.h` 和 `ecu/config/src/ecu_config.c`：

- `ECU_GUESS_*` 宏：CAN 波特率、CANopen 节点号、PDO 基准 COB-ID、Modbus 从站号、DIO 掩码、液压阀掩码、ADC 量程等。
- `ecu_hardware_config_default()`：把这些宏整理成默认硬件配置结构，供设备适配层使用。

后续如果发现节点号、线束定义、继电器极性、ADC 量程或阀组位号不对，优先改这里，不要在 `remote`、`control`、`vehicle` 业务逻辑里写硬件魔法值。

## 本地检查

运行框架约束测试：

```powershell
python tests\python\run_tests.py
```

运行静态架构检查：

```powershell
python tools\check_no_forbidden_patterns.py
```

这些检查主要保护分层边界、安全边界、SBUS 阈值集中管理、猜测硬件参数集中管理、设备适配边界和 CPU0/CPU1 职责划分。

## SEGGER Embedded Studio 工程

当前 CPU0/CPU1 应用可以通过 HPM SDK CMake 生成 SES 工程：

```powershell
$env:HPM_SDK_BASE = (Resolve-Path 'ecu\sdk_env_v1.11.0\hpm_sdk').Path
$env:GNURISCV_TOOLCHAIN_PATH = (Resolve-Path 'ecu\sdk_env_v1.11.0\toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win').Path
$cmake = (Resolve-Path 'ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe').Path
$ninja = (Resolve-Path 'ecu\sdk_env_v1.11.0\tools\ninja\ninja.exe').Path
$py = (Resolve-Path 'ecu\sdk_env_v1.11.0\tools\python3\python.exe').Path

& $cmake -S ecu\apps\agri_chassis_control_cpu0 -B tmp\cmake_cpu0 `
  -G Ninja -DCMAKE_MAKE_PROGRAM="$ninja" -Dpython_exec="$py" `
  -DBOARD=ecu_isolation -DBOARD_SEARCH_PATH="$((Resolve-Path 'ecu').Path)"

& $cmake -S ecu\apps\agri_chassis_control_cpu1 -B tmp\cmake_cpu1 `
  -G Ninja -DCMAKE_MAKE_PROGRAM="$ninja" -Dpython_exec="$py" `
  -DBOARD=ecu_isolation -DBOARD_SEARCH_PATH="$((Resolve-Path 'ecu').Path)" `
  -DBUILD_FOR_SECONDARY_CORE=1
```

生成后的 SES 工程位于：

```text
tmp/cmake_cpu0/segger_embedded_studio/agri_chassis_control_cpu0.emProject
tmp/cmake_cpu1/segger_embedded_studio/agri_chassis_control_cpu1.emProject
```

如果 CMake 提示缺少 `yaml`，必须显式传入 SDK 自带 Python，也就是上面命令中的 `-Dpython_exec="$py"`。
