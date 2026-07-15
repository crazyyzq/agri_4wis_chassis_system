# CANopen RPDO1 四轮转向遥控接入调试操作文档 V1

日期：2026-07-07

> 历史调试操作记录，已于 2026-07-12 对齐当前 `current7 + sync1` PDO
> 契约。它只能指导隔离台架调试；生产参数、故障恢复和整车安全逻辑必须以
> `ecu/config/include/ecu_config.h`、PDO 契约和当前 ECU 代码为准。

适用范围：

- CAN 分析仪 CAN1 接 ECU CAN2，也就是整车行走/转向 CANopen 网络。
- CAN 分析仪 CAN2 接 ECU CAN3，也就是抬升/液压 CANopen 网络。
- 本文只覆盖四个转向轴 Node5/6/7/8 的 RPDO1 实时位置控制验证。
- 本文不授权行走、抬升、液压、电流/力矩输出测试。

## 1. 当前确认结论

本文当日验证、且与当前映射一致的转向实时控制方式是：

```text
CANopen 网络：ECU CAN2 / 分析仪 CAN1
节点：Node5/6/7/8
PDO：RPDO1
映射：current7
传输类型：sync1
控制周期：20 ms
控制方法：绝对位置 + 最新目标覆盖 + arm/trigger 分两拍 + SYNC 执行
```

`current7` 的意思是每帧 7 字节：

```text
byte 0..1: 0x6040 controlword, little-endian u16
byte 2   : 0x6060 mode of operation, i8, position mode = 1
byte 3..6: 0x607A target position, little-endian i32
```

选择 `current7` 而不是 `compact6` 的原因：

- `compact6` 少 1 字节，带宽略低。
- `current7` 每个实时命令都带 `6060=1`，可以避免驱动器因为初始化遗漏、异常恢复、人工调试切模式后仍接收位置目标但模式错误的问题。
- 在 1 Mbit/s CANopen 网络上，四轴 20 ms 同步位置控制的带宽占用仍可接受。

## 2. 当前 PDO 标准

所有 Node1..13 已按以下统一标准配置并保存：

```text
RPDO0: 0x200 + node, type 1, DLC 7
  6040:00 u16 controlword
  6060:00 i8  mode = 3
  60FF:00 i32 target velocity

RPDO1: 0x300 + node, type 1, DLC 7
  6040:00 u16 controlword
  6060:00 i8  mode = 1
  607A:00 i32 target position

RPDO2: 0x400 + node, type 1, DLC 4
  60C1:01 i32 interpolated position

RPDO3: 0x500 + node, type 1, DLC 5
  6040:00 u16 controlword
  6060:00 i8  mode = 4
  2340:00 i16 command current, unit = 10 mA

TPDO0: 0x180 + node, type 1, DLC 8
  6064:00 i32 actual position
  606C:00 i32 actual velocity

TPDO1: 0x280 + node, configured type 10, DLC 8
  2183:00 u32 latched fault
  6041:00 u16 statusword
  221C:00 i16 actual current

TPDO2/TPDO3:
  1A02:00 = 0
  1A03:00 = 0
```

掉电重启后的实测注意点：

- Node1..13 的 RPDO0/1/2/3、TPDO0/1 配置读回正确。
- TPDO2/TPDO3 不作为有效反馈通道，当前只要求映射数为 0。
- TPDO2/TPDO3 的 COB-ID bit31 禁用位掉电后未保持，读回为 `0x000003xx / 0x000004xx`。
- 但 `1A02:00 = 0`、`1A03:00 = 0` 保持，5 秒被动抓包未观察到 0x38x / 0x48x 帧。
- 因此当前工程按“零映射使 TPDO2/TPDO3 实际无效”处理；不要把 bit31 是否保持作为验收依据。

## 3. 四轴 RPDO1 实时控制帧顺序

一次转向目标更新必须以完整四轴组为单位发送。不要先完整处理 Node5，再处理 Node6。

以目标 `target5..target8` 为例：

```text
0x305: 2F 00 01 target5_i32_le
0x306: 2F 00 01 target6_i32_le
0x307: 2F 00 01 target7_i32_le
0x308: 2F 00 01 target8_i32_le
0x080: SYNC

0x305: 3F 00 01 target5_i32_le
0x306: 3F 00 01 target6_i32_le
0x307: 3F 00 01 target7_i32_le
0x308: 3F 00 01 target8_i32_le
0x080: SYNC
```

说明：

- `0x002F` 是 arm，bit4 为 0。
- `0x003F` 是 trigger，bit4 为 1。
- `2F -> 3F` 形成新的目标触发沿。
- arm 组和 trigger 组都必须覆盖 Node5/6/7/8。
- 每个控制周期只保留最新四轴目标，不排队旧目标。

## 4. 面向真实遥控器的控制思路

真实遥控器不是“给一个目标等到位”。实际逻辑应按下面的思想实现：

```text
SBUS 解码
  -> 得到当前摇杆原始值
  -> 做通道映射、有效性、失控判断
  -> 转为转向请求
  -> vehicle/control 在同一个周期计算四轮目标
  -> CAN2 motion task 只消费完整快照
  -> 20 ms 发送一组四轴 RPDO1 arm/trigger
```

关键规则：

1. 遥控解析层只产生请求，不直接发 CAN。
2. 四个转向轮的目标必须来自同一次 SBUS 采样和同一次运动学计算。
3. CAN2 motion task 是唯一实时 PDO 发送者。
4. 目标未到位时，如果摇杆变化，直接用最新目标覆盖旧目标。
5. 不等待 `target reached` 再发送下一轴。
6. 不用 SDO 做实时跟随。
7. 任意单轴故障必须被诊断标记，但不能让健康轴被某个 SDO 等待卡死。

## 5. 遥控目标建议参数

现场已知标定：

```text
转向位置：+500000 count 约等于向左 45 deg
转向位置：-500000 count 约等于向右 45 deg
```

真实遥控接入的初始建议值：

```text
控制周期：20 ms
目标限幅：-500000 .. +500000 count
目标最大变化率：8333 count / 20 ms 起步
中位死区：先按 SBUS 实测噪声设置，不建议直接用本文 center_noise 的 50000 count
位置更新阈值：建议从 500 .. 1000 count 起步
profile velocity：1666666
profile acceleration：20000000
profile deceleration：20000000
```

`8333 count / 20 ms` 对应从 0 到 500000 约 1.2 s。实际驾驶如果感觉太慢，可以逐步提高；如果机械冲击大或方向发抖，应降低。

## 6. 已执行硬件验证

### 6.1 PDO 掉电保持性验证

执行：

```powershell
python tools\canopen_pdo_config\configure_all_nodes.py --read-only --backend controlcan --bus can1,can2 --nodes 1-13 --timeout-ms 900 --retries 2 --log-dir out\canopen_pdo_config_v4_current7_torque_readback_after_power_cycle
```

结果：

```text
Node1..13 RPDO0/1/2/3 正确
Node1..13 TPDO0/1 正确
TPDO2/3 不作为有效反馈通道，映射数为 0
5 秒被动抓包未观察到 0x38x / 0x48x 保留 TPDO 帧
```

读回日志：

```text
out\canopen_pdo_config_v4_current7_torque_readback_after_power_cycle\20260707_105415
```

### 6.2 四轴 joystick 模型

目的：模拟人缓慢推杆、保持、反向、回中。

执行：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes steer4 --allow-motion --rpdo-map current7 --rpdo-transmission sync1 --waveform joystick --amplitude 500000 --period-ms 20 --samples 420 --max-step 8333 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback --fault-reset-before-test --log-dir out\steer4_rpdo1_remote_joystick_limited
```

结果：

```text
target range: -500000 .. +500000
feedback_count: 6733
emcy_count: 0
max_late_ms: 0.0
max_send_duration_ms: 7.184
reached_zero: true
```

### 6.3 四轴 remote_stress 模型

目的：模拟轮子尚未到目标位置时，摇杆目标已经再次变化。这个测试验证“最新目标覆盖旧目标”，不排队历史目标。

执行：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes steer4 --allow-motion --rpdo-map current7 --rpdo-transmission sync1 --waveform remote_stress --amplitude 500000 --period-ms 20 --samples 360 --max-step 0 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback --fault-reset-before-test --log-dir out\steer4_rpdo1_remote_target_override_unlimited
```

结果：

```text
target range: -500000 .. +500000
feedback_count: 5776
emcy_count: 0
max_late_ms: 0.0
max_send_duration_ms: 7.349
reached_zero: true
```

### 6.4 四轴 step + 目标限速模型

目的：模拟驾驶员突然拨杆，但 ECU 对目标做变化率限制。

执行：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes steer4 --allow-motion --rpdo-map current7 --rpdo-transmission sync1 --waveform step --amplitude 500000 --period-ms 20 --samples 360 --max-step 8333 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback --fault-reset-before-test --log-dir out\steer4_rpdo1_remote_step_limited
```

结果：

```text
requested target range: -500000 .. +500000
limited target range: -249970 .. +500000
feedback_count: 5774
emcy_count: 0
max_late_ms: 0.0
max_send_duration_ms: 7.201
reached_zero: true
```

说明：该测试样本数有限，负向目标在限速后尚未走到 -500000，这是限速模型的预期现象，不是故障。

### 6.5 四轴 center_noise 模型

目的：模拟遥控器中位附近噪声，为死区和更新阈值选择提供依据。

执行：

```powershell
python tools\canopen_position_debug\steer_rpdo1_follow_debug.py --nodes steer4 --allow-motion --rpdo-map current7 --rpdo-transmission sync1 --waveform center_noise --amplitude 500000 --period-ms 20 --samples 300 --max-step 0 --profile-velocity 1666666 --profile-accel 20000000 --timeout-ms 900 --live-feedback --fault-reset-before-test --log-dir out\steer4_rpdo1_remote_center_noise
```

结果：

```text
target range: -50456 .. +50456
feedback_count: 4816
emcy_count: 0
max_late_ms: 0.0
max_send_duration_ms: 6.962
reached_zero: true
```

说明：这个模型故意放大中位噪声，用于证明系统不会因小范围目标变化产生 EMCY。真实遥控接入时仍应在 SBUS 层和目标层做死区与更新阈值，不能让中位噪声持续触发方向动作。

## 7. 接入真实遥控器时的验收标准

接入真实遥控器后，先只开转向，不开行走。

最低验收标准：

```text
1. SBUS 在线，failsafe 未触发。
2. 摇杆中位时，四轴目标保持稳定，不因噪声反复触发。
3. 左右缓慢推杆时，Node5/6/7/8 都在同一 20 ms 周期内收到 RPDO1。
4. 分析仪能看到固定顺序：
   Node5 arm, Node6 arm, Node7 arm, Node8 arm, SYNC,
   Node5 trigger, Node6 trigger, Node7 trigger, Node8 trigger, SYNC
5. 不出现只更新一个节点、其它节点长时间无 RPDO1 的情况。
6. TPDO0 能看到四轴实际位置变化。
7. TPDO1 / EMCY 不出现新故障。
8. 摇杆回中后，四轴能回到目标零位附近。
```

建议首次真实遥控调试步骤：

```text
1. 确认四个转向轮离地或机械风险可控。
2. 保持行走输出禁止，保持 commissioning steer-only 策略。
3. 让遥控器保持中位，观察目标是否稳定。
4. 小幅左推，确认四个转向轴方向一致且与整车定义一致。
5. 小幅右推，确认方向相反。
6. 慢慢推到最大，确认限幅不超过 +/-500000 count。
7. 快速从左到右，确认目标覆盖逻辑正常，不排队旧目标。
8. 松手回中，确认四轴回零。
```

## 8. ECU 代码接入要点

后续把真实遥控接入 ECU 时，建议按以下职责划分：

```text
remote_sbus_mapper:
  只做通道解析、归一化、按钮/开关事件、failsafe。

motion_control / four_wheel_kinematics:
  根据当前模式计算四轮目标角。
  输出完整四轮转向目标快照。

command_arbiter / safety_manager:
  判断遥控授权、急停、失控、故障、是否允许转向。

motion_device / CAN2 task:
  只消费最终快照。
  按固定周期发送 RPDO1 arm/trigger/SYNC。
  监控 TPDO、EMCY、超时、发送完成状态。
```

不要把下面的逻辑写进遥控解析层：

```text
直接发 CAN
直接改驱动器控制字
等待某个节点到位
轮询 SDO
根据单轴状态阻塞其它轴更新
```

## 9. 当前仍需现场确认的事项

1. 转向正负方向要在整车装配状态下确认：
   - 已知正目标约等于向左，负目标约等于向右。
   - 仍需确认四轮在阿克曼模式下内外轮角度符号和幅值是否符合车体坐标。
2. 中位死区要用真实 SBUS 数据测量后定：
   - 不要直接套用 `center_noise` 的 50000 count。
3. `8333 count / 20 ms` 是当日台架调试起点，不是当前固件参数来源：
   - 手感慢可以提高。
   - 机械冲击或发抖则降低。
4. TPDO2/TPDO3 不作为有效反馈通道：
   - 当前实测零映射不会发帧。
   - 后续不要再给 TPDO2/TPDO3 增加映射，除非有明确的新反馈需求和带宽预算。
