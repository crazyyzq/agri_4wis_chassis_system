# 工程文档入口

更新日期：2026-07-15

本页是当前工程文档的唯一入口。出现数字、节点、PDO 或操作逻辑冲突时，先查
`ecu/config/include/ecu_config.h`，再查本页列出的当前规范；历史调试记录只证明
当时明确写出的测试边界。

## 当前规范

| 内容 | 权威文档 |
|---|---|
| 工程概览、目录和构建 | `README.md` |
| 从 `main()` 通读代码 | `docs/ecu-main-control-architecture.md` |
| 原始架构与需求基线 | `doc/ECU_Project_Implementation_v1.4.md` |
| Node1–13 PDO 映射 | `doc/CANOPEN_PDO_MAPPING_RECORD_V1.md` |
| PDO 字节级操作 | `doc/ECU/CANopen_RPDO_Operation_Manual_Node1-13_V2.md` |
| 遥控器实际操作逻辑 | `doc/ECU/遥控操作逻辑说明书.md` |
| 四腿同步变地隙调试与失败恢复证据 | `doc/ECU/四腿同步变地隙调试记录_2026-07-16.md` |
| 待现场确认配置 | `docs/ecu-configuration-open-items.md` |
| 工具与脚本入口 | `tools/README.md` |

当前关键基线：

- CAN2 Node1–8、CAN3 Node9–13 使用 `current7 + sync1`。
- Node1–8/13 为 10000 count/rev；Node9–12 为 131072 count/rev。
- 变地隙机构为 20 motor rev/10 mm，即 262144 count/mm；正常范围
  10–490 mm，伸腿时位置向负方向变化。
- 变地隙正常轨迹为 20 mm/s、8 mm/s²；四轴使用同一绝对位置点，运行跟随窗口
  为 10 mm，回中/到端点仍需四腿误差和高度差不超过 3 mm 后统一失能抱闸。
- Node13 只允许反转；悬挂液压使用 1500 rpm，变轮距使用 2400 rpm。
- 固件默认仍为转向调试安全模式；编译成功不代表整车已经安全验收。

## 当前现场记录

- `doc/ECU/整车调试记录_2026-07-07.md`：持续追加的整车阶段记录。记录中的旧
  参数按日期保留；文首“当前状态”说明哪些结果仍适用。
- `doc/ECU/转向零点校准脚本说明.md`：转向找零脚本和 ECU B1 三击流程。

新增实测证据时应记录日期、硬件接线、固件提交、配置、原始日志路径、成功/失败
边界和未验证项。不得把软件队列成功写成驱动器已接受命令。

## 历史证据

以下文档保留用于追溯，不是当前生产配置：

- `docs/test_logs/2026-06-30-whole-vehicle-communication-test.md`
- `doc/ECU/CANopen_RPDO1_Remote_Steering_Commissioning_Guide_V1.md`
- `doc/ECU/CANopen_Motion8_Suspended_Wheel_Debug_Record.md`
- `doc/ECU/整车行走轮速度调试报告_2026-07-07.md`

已删除不再被当前方法引用的 Node5 RPDO2、`compact6`/`async255` 对比记录以及已
执行完成的临时设计/计划。保留下来的历史文件中若出现旧升降换算、旧速度或旧测试
上限，不得直接带回生产代码；需要复用时，必须重新对照当前配置、厂家手册、PDO
读回和分析仪抓包。

## 文档维护规则

1. 当前数字只在配置和当前规范中维护，不在多份历史记录中反复复制。
2. 历史记录不删除原始结果，但必须有醒目的归档声明和本页链接。
3. `out/`、`tmp/` 是生成目录。当前只长期保留 `tmp/cmake_cpu0`、明确引用的
   全行程/最近复测日志，以及 `out/` 中当前 PDO 读回和最终找零证据；不要在
   `tmp/` 累积带日期或 `latest/whole/audit` 等名字的重复构建工程。
4. `.worktrees/` 中可能有未合并分支或未提交板测代码，不属于普通构建缓存。
   只能清理其中可重新生成的旧构建；删除工作树前必须先确认工作树干净、分支已
   合并且不再需要。当前保留板测与完整控制闭环两个隔离工作树及各自最新 SEGGER
   工程。
5. 中文 Markdown 使用 UTF-8；修改后以 UTF-8 回读并运行 `git diff --check`。
