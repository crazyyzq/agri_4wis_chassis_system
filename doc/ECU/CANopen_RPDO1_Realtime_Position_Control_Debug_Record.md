# CANopen RPDO1 实时位置控制调试记录

记录日期：2026-07-06  
测试目标：暂不使用插补模式，先把 CiA-402 profile-position 位置模式调成可用于后续遥控实时转向控制的方案。

> 当前工程决策更新：本文记录的是 `compact6` 与 `current7` 的历史调试过程。后续为了降低“初始化阶段模式设置错误”导致的运行风险，工程默认 PDO 标准已改为 `current7 + sync1`：速度 RPDO0、位置 RPDO1 都携带 `6060` 模式字节；另新增 RPDO3 电流/力矩通道，携带 `6040 + 6060 + 2340`。因此本文中“推荐 compact6 作为默认方案”的结论只作为历史测试记录保留，不再作为当前配置依据。

## 1. 结论

当前可用方案是：

```text
初始化 / 配置：SDO
实时目标更新：RPDO1
实时周期：20 ms 起步
控制方式：绝对位置 + 最新目标覆盖
触发顺序：所有节点 arm -> SYNC -> 所有节点 trigger -> SYNC
```

不要把摇杆实时控制做成“发一个轴，等到位，再发下一个轴”。实时阶段也不要连续用 SDO 发目标。SDO 只用于上电配置、诊断和低频服务。

本轮实测确认：

- Node5 单轴可以跟随连续变化目标，包括目标尚未到位时被新目标覆盖。
- Node5/6/7/8 四轴同步可以稳定执行连续目标覆盖模型，实测 0 EMCY。
- Node8 之前不动的根因是动力线断开。动力线恢复后，Node8 单轴和四轴同步均通过。
- 推荐把实时转向 RPDO1 从 7 字节映射改为 6 字节紧凑映射：`6040 + 607A`。`6060=1` 保留在初始化 SDO 阶段设置，不再每个实时周期重复发送。
- `compact6 + async255` 运动测试通过，但因为当前 TPDO 仍是同步 TPDO，不发送 SYNC 时测试期间没有实时 TPDO 反馈。生产默认建议先采用 `compact6 + sync1`；如果要进一步降低总线占用，再同步设计 TPDO 反馈策略后切到 `async255`。

## 2. 当前可靠 RPDO1 配置

当前最稳妥的生产候选 RPDO1 是 6 字节同步 PDO：

```text
COB-ID: 0x300 + node_id
Transmission type: 1
DLC: 6
Payload:
  byte 0..1: 0x6040 controlword, little-endian u16
  byte 2..5: 0x607A target position, little-endian i32
```

对应映射：

```text
1401:01 = 0x300 + node_id
1401:02 = 1
1601:00 = 2
1601:01 = 0x60400010
1601:02 = 0x607A0020
```

`6060:00` 不放进实时 PDO。每个节点初始化时用 SDO 写一次：

```text
6060:00 = 1
```

本轮也验证过旧 7 字节同步 PDO：

```text
COB-ID: 0x300 + node_id
Transmission type: 1
DLC: 7
Payload:
  byte 0..1: 0x6040 controlword, little-endian u16
  byte 2   : 0x6060 mode of operation, i8, profile-position = 1
  byte 3..6: 0x607A target position, little-endian i32
```

对应映射：

```text
1401:01 = 0x300 + node_id
1401:02 = 1
1601:00 = 3
1601:01 = 0x60400010
1601:02 = 0x60600008
1601:03 = 0x607A0020
```

旧 7 字节映射也稳定，但每个实时 PDO 多发送 1 字节 `6060`，而运行中模式不会变，属于可删除冗余。

## 3. 初始化顺序

每个转向节点初始化建议：

```text
NMT operational
2300:00 = 0x001E      ; control source = CANopen
6060:00 = 1           ; profile-position mode
6081:00 = profile velocity
6083:00 = profile acceleration
6084:00 = profile deceleration
6040:00 = 0x000F      ; enable operation
读取 6061，确认 mode display = 1
读取 6041 / 2183 / 60F4，确认无关键故障
```

本轮 Node5/6/7 的可用参数：

```text
6081 profile velocity      = 1666666
6083 profile acceleration  = 20000000
6084 profile deceleration  = 20000000
```

Node5 单轴在 `6081=2000000` 时也能通过部分遥控模型，但 `3333333` 级别会触发异常，不能直接采用。整车初调建议先用 `1666666`，再按每轴实测逐步提高。

## 4. 实时控制帧顺序

每个 20 ms 控制周期只使用最新目标，不排队旧目标。

以 `compact6 + sync1` 为例，Node5/6/7/8 目标分别为 `target[N]`：

```text
0x305: 2F 00 target5_i32_le
0x306: 2F 00 target6_i32_le
0x307: 2F 00 target7_i32_le
0x308: 2F 00 target8_i32_le
0x080: SYNC

0x305: 3F 00 target5_i32_le
0x306: 3F 00 target6_i32_le
0x307: 3F 00 target7_i32_le
0x308: 3F 00 target8_i32_le
0x080: SYNC
```

关键点：

- arm 和 trigger 必须分两段。
- 不允许 Node5 arm/trigger 完整跑完后再处理 Node6。
- 不等待 target reached。
- 不用 SDO 轮询阻塞实时周期。
- 某轴出错必须标记该轴故障，但不能让其它健康轴的发送任务被一个 SDO 等待卡死。

## 4.1 PDO 效率结论

本轮对比了三种实时命令方案：

| 方案 | 实测结果 | 反馈可见性 | 结论 |
|---|---:|---:|---|
| current7 + sync1 | 四轴通过，0 EMCY | 有同步 TPDO 反馈 | 稳定，但实时 PDO 有 1 字节冗余 |
| compact6 + sync1 | 四轴通过，0 EMCY | 有同步 TPDO 反馈 | 推荐作为当前 ECU 默认方案 |
| compact6 + async255 | 四轴运动通过，0 EMCY | 当前测试中无实时 TPDO 反馈 | 可作为后续优化方向，需同步修改 TPDO 策略 |

粗略按 1 Mbit/s 标准帧估算，四轴每 20 ms 一组 arm/trigger：

```text
current7 + sync1:
  8 个 7-byte RPDO + 2 个 SYNC
  约 918 bit / cycle，不含位填充
  约 45.9 kbit/s，约 4.6% bus load

compact6 + sync1:
  8 个 6-byte RPDO + 2 个 SYNC
  约 854 bit / cycle，不含位填充
  约 42.7 kbit/s，约 4.3% bus load
  相比 current7 + sync1 节省约 7%

compact6 + async255:
  8 个 6-byte RPDO，无 SYNC
  约 760 bit / cycle，不含位填充
  约 38.0 kbit/s，约 3.8% bus load
  相比 current7 + sync1 节省约 17%
```

`compact6 + async255` 的命令带宽最低，但现有 TPDO0/TPDO1 是同步反馈；不发 SYNC 时实时反馈不可见。若要采用 async255，应同时把反馈设计为：

```text
方案 A：TPDO0/TPDO1 改事件/定时发送，限制事件周期，避免反馈泛洪。
方案 B：命令 async255，但诊断任务低频发 SYNC，例如 50~100 ms 一次，只用于反馈采样。
```

在没有完成反馈策略验证前，不建议直接把生产 ECU 改成 async255。

## 5. 遥控接入建议

真实遥控不是“给一个目标等到位”，而是目标连续变化，甚至轮子没到目标时目标又变了。因此 ECU 侧建议：

1. SBUS 只产生当前摇杆请求。
2. vehicle/control 层在一个周期内生成完整四轮目标快照。
3. CAN2 motion task 只消费最新快照，不排队历史快照。
4. 对摇杆值做死区、变化阈值和目标变化率限制。
5. 初始周期使用 20 ms；稳定后再评估 10 ms。

建议初始整形参数：

```text
steer_target_limit_counts = +/-500000
steer_update_period_ms    = 20
steer_max_step_counts     = 5000 到 8333 count / 20 ms 起步
steer_deadband_counts     = 视遥控中位噪声设定，建议从 1000~3000 count 量级开始
steer_update_threshold    = 视抓包噪声设定，建议从 500~1000 count 量级开始
```

说明：

- `500000 count` 对应用户现场给出的约 `45 deg`。
- `5000 count / 20 ms` 等价全行程约 2 s；更保守。
- `8333 count / 20 ms` 是本轮三角模型用过的变化量级；Node5/6/7/8 均已通过。
- Node8 之前异常是动力线断开导致；恢复动力线后单轴和四轴均通过。

## 6. 已执行测试

测试工具：

```text
tools/canopen_position_debug/steer_rpdo1_follow_debug.py
```

共同条件：

```text
CAN analyzer CAN1 -> ECU CAN2 / motion CANopen bus
bitrate = 1 Mbit/s
RPDO1 = 7-byte current mapping
period = 20 ms
no NMT reset
no Flash/NVM save
```

### Node5 单轴 joystick 模型

命令：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes 5 --allow-motion --rpdo-map current7 --waveform joystick --amplitude 500000 --period-ms 20 --samples 600 --profile-velocity 2000000 --profile-accel 20000000 --timeout-ms 900 --live-feedback
```

结果：

```text
log_dir: out/steer_rpdo1_follow_5_20260706_201432
target range: -500000 .. +500000
actual position range: -477865 .. +502173
EMCY: 0
reached_zero: true
```

### Node5 单轴 remote_stress 覆盖模型

命令：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes 5 --allow-motion --rpdo-map current7 --waveform remote_stress --amplitude 500000 --period-ms 20 --samples 600 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback
```

结果：

```text
log_dir: out/steer_rpdo1_follow_5_20260706_201714
target range: -500000 .. +500000
max target delta per 20 ms: 100000
EMCY: 0
reached_zero: true
```

说明：这个模型故意比真实遥控激进，用来验证“旧目标覆盖、不排队”。实际位置不会跟到全幅，这是预期现象。

### Node5/6/7/8 四轴连续三角模型，current7 + sync1

命令：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes steer4 --allow-motion --rpdo-map current7 --waveform triangle --amplitude 500000 --period-ms 20 --samples 240 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback --fault-reset-before-test
```

结果：

```text
log_dir: out/steer_rpdo1_follow_steer4_20260706_204434
target range: -500000 .. +500000
EMCY: 0
reached_zero: true
max send duration: 8.826 ms
```

### Node5/6/7/8 四轴 remote_stress 覆盖模型，current7 + sync1

命令：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes steer4 --allow-motion --rpdo-map current7 --waveform remote_stress --amplitude 500000 --period-ms 20 --samples 300 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback --fault-reset-before-test
```

结果：

```text
log_dir: out/steer_rpdo1_follow_steer4_20260706_204458
target range: -500000 .. +500000
EMCY: 0
reached_zero: true
max send duration: 9.947 ms
```

### Node5/6/7/8 四轴连续三角模型，compact6 + sync1

命令：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes steer4 --allow-motion --rpdo-map compact6 --waveform triangle --amplitude 500000 --period-ms 20 --samples 240 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback --fault-reset-before-test
```

结果：

```text
log_dir: out/steer_rpdo1_follow_steer4_20260706_204540
target range: -500000 .. +500000
EMCY: 0
reached_zero: true
max send duration: 8.070 ms
```

### Node5/6/7/8 四轴 remote_stress 覆盖模型，compact6 + async255

命令：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes steer4 --allow-motion --rpdo-map compact6 --rpdo-transmission async255 --waveform remote_stress --amplitude 500000 --period-ms 20 --samples 300 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback --fault-reset-before-test --log-dir out\steer4_compact6_async255_remote_stress_20260706_manual
```

结果：

```text
log_dir: out/steer4_compact6_async255_remote_stress_20260706_manual
target range: -500000 .. +500000
EMCY: 0
reached_zero: true
max send duration: 5.666 ms
feedback_count: 0
```

`feedback_count=0` 是关键限制：命令异步执行通过，但因为当前 TPDO 仍同步，测试过程中没有实时反馈帧。

### 旧 Node5/6/7 三轴连续三角模型

命令：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes 5,6,7 --allow-motion --rpdo-map current7 --waveform triangle --amplitude 500000 --period-ms 20 --samples 240 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback
```

结果：

```text
log_dir: out/steer_rpdo1_follow_5_6_7_20260706_202410
target range: -500000 .. +500000
EMCY: 0
reached_zero: true
```

### Node5/6/7 三轴 remote_stress 覆盖模型

命令：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes 5,6,7 --allow-motion --rpdo-map current7 --waveform remote_stress --amplitude 500000 --period-ms 20 --samples 300 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback
```

结果：

```text
log_dir: out/steer_rpdo1_follow_5_6_7_20260706_202608
target range: -500000 .. +500000
EMCY: 0
reached_zero: true
```

### Node5/6/7 三轴中位噪声模型

命令：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes 5,6,7 --allow-motion --rpdo-map current7 --waveform center_noise --amplitude 500000 --period-ms 20 --samples 300 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback
```

结果：

```text
log_dir: out/steer_rpdo1_follow_5_6_7_20260706_202628
target range: about -50456 .. +50456
EMCY: 0
reached_zero: true
```

这个模型用于后续选择遥控器中位死区和目标变化阈值。

## 7. Node8 恢复记录

早期 Node8 测试表现为：

```text
607A target_position 可以写入
6064 actual_position 不变化
606C actual_velocity = 0
2183 latched_fault = 0
6041 statusword = 0x2E37
```

后续现场确认 Node8 动力线断开。动力线恢复后重新测试：

```text
Node8 single-axis +/-100000 triangle: pass, EMCY 0
Node8 single-axis +/-500000 triangle: pass, EMCY 0
Node5/6/7/8 current7 + sync1 triangle: pass, EMCY 0
Node5/6/7/8 current7 + sync1 remote_stress: pass, EMCY 0
Node5/6/7/8 compact6 + sync1 triangle: pass, EMCY 0
Node5/6/7/8 compact6 + async255 remote_stress: pass, EMCY 0
```

结论：Node8 问题不是 PDO 调度或 CANopen 命令格式导致，而是动力线断开导致的驱动侧无法执行运动。恢复动力线后，Node8 已能参与四轴同步实时位置控制。

## 8. ECU 代码落地建议

后续 ECU 代码应按以下结构实现：

```text
remote manager:
  解析 SBUS，输出归一化摇杆值

vehicle control:
  根据模式和四轮转向算法计算四个目标角
  做死区、限速、软限幅
  生成完整 steering command snapshot + sequence

CAN2 motion task:
  只读取完整 snapshot
  不等待到位
  不发送 SDO
  每 20 ms 发送一组 arm PDO + SYNC + trigger PDO + SYNC
  监控 TPDO/EMCY/状态字
```

不要在 remote task、vehicle task、debug helper 里直接发 CAN2 运动帧。

## 9. 推荐下一步

1. 将 ECU 侧实时转向 RPDO 默认方案改为 `compact6 + sync1`：

```text
RPDO1 = 6040 controlword u16 + 607A target position i32
6060 profile-position mode 只在初始化阶段 SDO 设置一次
```

2. 如果后续要继续降低 CAN2 占用率，再单独设计并验证 `compact6 + async255` 对应的 TPDO 反馈策略。
3. 真实遥控接入时先启用：

```text
20 ms update
target limit +/-500000
max step 5000 count / 20 ms
deadband + update threshold
```

稳定后再根据驾驶手感提高 max step 或控制频率。
