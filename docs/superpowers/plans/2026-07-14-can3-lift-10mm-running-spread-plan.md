# CAN3 变地隙四轴同步实施计划

1. 检查 CAN3 任务、命令邮箱、lift device 状态机及当前测试，确认没有车辆任务直接发送 PDO、没有第二个 SYNC 源。
2. 先增加失败的静态契约测试：10 mm 运动中 spread 只记录诊断，不触发停机/复位；最终 3 mm 到位契约保持不变。
3. 在 `ecu_config.h` 增加带单位说明的 10 mm 运动偏差配置，在 lift 状态中增加当前/最大 spread 与超差计数并明确初始化/复位路径。
4. 在 RUNNING 状态基于同一批有效 TPDO0 反馈更新诊断，继续使用现有有界双向修正，禁止因 10 mm 单次超差退出插补。
5. 检查并补强诊断快照，保证 COM 调试能够观察 CAN3 状态、四轴位置、spread、超差计数和恢复状态，且不阻塞 CAN3 周期任务。
6. 运行 `python tests\python\run_tests.py`、`git diff --check`，配置并编译 `whole_vehicle_motion` CPU0 工程，要求零错误零警告。
7. 下载整车固件；通过 COM9、J-Link 与 CAN 分析仪采集启动、遥控命令、四轴 RPDO2/SYNC、TPDO0 和故障证据。
8. 从约 10 mm 开始逐步做遥控伸出/回中验证；满足连续运动、运行 spread ≤10 mm、最终 spread ≤3 mm、稳定 5 周期失能抱闸后再报告硬件通过。
