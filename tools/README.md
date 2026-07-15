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
| `canopen_interp_debug/lift4_sync_debug.py` | Node9–12 RPDO2/SYNC 插补诊断 | 要求 `--allow-motion` |
| `modbus/virtual_adc_module.py` | 在 PC 串口模拟 8 通道 Modbus ADC | 主动打开指定串口 |
| `modbus/rtu_probe.py` | Modbus RTU 只读/探测 | 按命令参数运行 |
| `can/can2_monitor.py` | CAN 分析仪监视 | 只读接收 |

PDO 配置的完整安全参数和保存方式见
`tools/canopen_pdo_config/README.md`。转向找零见
`doc/ECU/转向零点校准脚本说明.md`。

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
