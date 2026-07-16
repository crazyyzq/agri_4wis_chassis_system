# 工程工具与调试脚本

工具分为离线维护、只读诊断、危险运动测试和协议仿真四类。所有硬件运动脚本都
是台架工具，不属于 ECU 正常控制链。

## 安全规则

- 先确认总线通道、波特率、节点号、机械支撑和急停能力。
- PDO 配置时 ECU 必须断开总线；运动测试时避免 ECU 与脚本同时成为控制主站。
- 没有 `--allow-motion`、`--apply` 或专用确认参数时保持 dry-run。
- Flash 保存、写 `0x6064` 和真实运动都需要明确人工授权。
- 保存脚本输出和分析仪抓包；本地发送成功不等于远端接受。

## 当前维护工具

| 工具 | 用途 | 默认行为 |
|---|---|---|
| `canopen_pdo_config/configure_all_nodes.py` | 离线配置/读回 Node1–13 `current7 + sync1` | dry-run |
| `canopen_position_debug/steer4_zero_calibration_debug.py` | Node5–8 两段找限位、三段回中、验证后写 `0x6064=0` | dry-run |
| `canopen_position_debug/steer_rpdo1_follow_debug.py` | 隔离台架验证转向 RPDO1 连续跟随 | 要求 `--allow-motion` |
| `canopen_motion_debug/motion8_remote_sim_debug.py` | 悬空轮模拟四种遥控运动 | 要求 `--allow-motion` |
| `canopen_interp_debug/lift4_sync_debug.py` | Node9/11/12/10 四轴 RPDO2/SYNC 绝对高度调试 | dry-run；运动要求目标值和 `--allow-motion` |
| `canopen_interp_debug/lift4_sync_cycle_validation.py` | 固定参数重复执行 10–490–10 mm 并逐半程汇总电流/同步/恢复证据 | dry-run；运动要求 `--allow-motion` |
| `modbus/virtual_adc_module.py` | 在 PC 串口模拟 8 通道 Modbus ADC | 主动打开指定串口 |
| `modbus/rtu_probe.py` | Modbus RTU 只读/探测 | 按命令参数运行 |
| `can/can2_monitor.py` | CAN 分析仪监视 | 只读接收 |

PDO 配置的完整安全参数和保存方式见
`tools/canopen_pdo_config/README.md`。转向找零见
`doc/ECU/转向零点校准脚本说明.md`。

### 四腿同步变地隙

当前脚本与 ECU 使用同一组参数：`131072 count/rev`、`20 motor rev/10 mm`、
`262144 count/mm`、20 ms 插补周期、20 mm/s、8 mm/s²、三点预装、
`655360 count`（2.5 mm）正常公共反馈前导。四轴每周期接收完全相同的绝对位置点，公共目标
由梯形轨迹和当前最慢轴共同推进；反馈突变也不能让目标单周期跳过轨迹允许步长。
Windows/USB 偶发超过 20 ms 时只延长当次周期并重建节拍，不会紧接着补发历史点；
报告中的 `tick_resync_count` 和 `maximum_tick_lateness_us` 用于识别这类主机侧抖动。
单周期四轴 TPDO0 接收瞬态最多重试 2 次，每次只重复同一个四轴绝对位置点和
SYNC，不推进轨迹；EMCY、映射故障或连续超时仍立即进入同步失能兜底。
分析仪每个实时周期逐帧提交四个 RPDO2，再提交 SYNC。不能依赖 ControlCAN
批量调用的返回值推断硬件已按数组顺序完成发送，也不能把 SYNC 与 RPDO 放在同一批，
因为 `0x080` 会在 CAN 仲裁中优先于
`0x40x`；两次调用仍显著少于原来的五次 USB 往返，并保持 PDO→SYNC 顺序。
共同目标不会在反馈边界冻结。跟随误差超过软阈值后，轨迹速度在达到 2.9 mm
保护边界前连续下降到 2 mm/s；超过驱动器配置的 3 mm 跟随窗口才判定异常。
这样既不会积累追赶跳变，也不会因重复同一点陷入永久停滞。
每次配置先让四轴全部进入 `0x0000` 并确认静止，再在失能状态清插补缓存；
异常失能后也尽力清缓存，防止下次使能重放故障前的旧轨迹，同时不复位节点或改零点。
若启动时四腿存在不超过 3 mm 的允许高度差，脚本先按请求方向以最大 2 mm/s
的 cubic smoothstep 预找平到共同起点，再从零速度进入主轨迹；不会把允许高度差
一次性变成绝对位置阶跃。
运行中 15 mm 差值只触发记录；停止前必须
连续取得四轴新 TPDO0，并满足目标误差、轴间差不超过 3 mm 和零速条件。
每次运动报告还会按 `0x221C` 的 10 mA/单位统计各轴运动阶段平均绝对电流和
最大绝对电流，便于比较速度档位、机械负载和异常趋势。
使能前还会逐节点读回 RPDO1、RPDO2、TPDO0、TPDO1 的 COB-ID、type 和映射；
与已保存的 `current7 + sync1` 契约不一致时拒绝运动，不会在运行时改写或保存 PDO。

先只查看计划，不打开 CAN：

```powershell
python tools\canopen_interp_debug\lift4_sync_debug.py --absolute-target-mm 490
```

确认 ECU CAN3 已物理断开、Node9/11/12/10 全部在线且四腿机械安全后，才允许执行：

```powershell
python tools\canopen_interp_debug\lift4_sync_debug.py `
  --absolute-target-mm 490 --allow-motion
```

回到 10 mm 时重新执行并把目标改为 `10`。脚本拒绝单腿/子集运动，不自动循环
10–490 mm，也不复位节点或写驱动器 Flash；异常和人工中断会先发送四轴同步失能，
再执行逐轴 SDO 失能兜底。

完成单程验证后，可用循环器做固定参数的重复稳定性试验。循环器不会自动重试失败
半程，任一异常立即结束并保留已完成日志：

```powershell
python tools\canopen_interp_debug\lift4_sync_cycle_validation.py `
  --cycles 10 --speed-mm-s 20 --accel-mm-s2 8 --allow-motion
```

20 mm/s 已完成 10 次 10→490→10 mm 全行程耐久验证。固定参数包括：
`0x6081=53000000`、`0x6083/0x6084=250000`、正常反馈限速软阈值
`655360 count`（2.5 mm）和 10 mm 运行跟随窗口。测试期间 3 次插补缓冲饥饿均
通过清 bit4、三点实测位置预装、同步重触发和公共位置对齐自动恢复，未失能或复位节点。

## 历史或方法对比脚本

- `canopen_interp_debug/node5_rpdo2_debug.py`：Node5 插补可行性实验，不是当前转向
  控制方法。
- `canopen_interp_debug/lift4_profile_position_cycle.py`：profile-position 对比实验。
- `canopen_interp_debug/lift4_velocity_sync_debug.py`：速度闭环同步对比实验。

这些脚本可能保留旧默认速度、旧安全边界或实验性参数。运行前必须显式覆盖为当前
机械参数；不得据此修改生产 PDO 或 ECU 状态机。当前升降生产契约以
`ecu_config.h`、`lift_hydraulic_device.c` 和 `doc/README.md` 为准。

## 软件检查

```powershell
python tests\python\run_tests.py
python tools\check_no_forbidden_patterns.py
```

测试脚本只验证源代码契约，不替代目标编译、CAN 分析仪或实车证据。
