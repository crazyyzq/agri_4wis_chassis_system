# CANopen Node1-8 悬空整车遥控同步调试记录

> 历史悬空测试记录，2026-07-12 已归档。它证明过驱动器和同步控制方法，
> 但不能替代当前 ECU 的 CAN2 自恢复、故障清除和安全联锁验收；现行 PDO
> 契约以 `doc/CANOPEN_PDO_MAPPING_RECORD_V1.md` 为准。

日期：2026-07-07

## 1. 测试目的

验证当前 PDO 标准是否能支持后续 ECU 遥控整车控制：

- Node1-4 行走轮速度控制；
- Node5-8 转向轮绝对位置控制；
- 正 Ackermann、反 Ackermann、蟹行、自转四种模式；
- 位置目标和速度目标都按同一次“遥控采样”的四轮快照发送；
- 任一周期不排队旧目标，只发送当前最新目标。

本次测试先绕过 ECU，直接用 CAN 分析仪驱动 ECU-CAN2 上的 8 个 CANopen 节点。这样可以把驱动器 PDO、SYNC、控制字、模式字问题和 ECU 任务调度问题分开。

## 2. 接线和前提

- CAN 分析仪 CAN1：接 ECU-CAN2 / 运动 CANopen 网络。
- 驱动器节点：
  - Node1-4：行走轮；
  - Node5-8：转向轮。
- 轮子已悬空，允许低速转动。
- PDO 已按 `current7 + sync1` 保存到驱动器 Flash：
  - RPDO0：`0x6040 controlword + 0x6060 mode=3 + 0x60FF target velocity`
  - RPDO1：`0x6040 controlword + 0x6060 mode=1 + 0x607A target position`
  - TPDO0：`0x6064 actual position + 0x606C actual velocity`
  - TPDO1：`0x2183 latched fault + 0x6041 statusword + 0x221C actual current`

## 3. 测试工具

新增脚本：

```powershell
python tools\canopen_motion_debug\motion8_remote_sim_debug.py --allow-motion ...
```

脚本行为：

1. 对 Node1-8 发送 NMT Operational。
2. 通过 SDO 设置控制源、模式和转向 profile 参数。
3. Node5-8 每个控制周期发送：
   - 4 帧 RPDO1 arm：`0x002F + mode=1 + target position`
   - 1 帧 SYNC
   - 4 帧 RPDO1 trigger：`0x003F + mode=1 + target position`
   - 1 帧 SYNC
4. Node1-4 每个控制周期发送：
   - 4 帧 RPDO0：`0x000F + mode=3 + target velocity`
   - 1 帧 SYNC
5. 结束时多次发送速度 0，然后发送 `0x0000 + mode=3 + velocity=0` 失能行走输出。

## 4. 测试结果

### 4.1 小幅正 Ackermann 冒烟测试

命令：

```powershell
python tools\canopen_motion_debug\motion8_remote_sim_debug.py `
  --allow-motion `
  --speed-kph 0.12 `
  --steer-deg 4 `
  --period-ms 80 `
  --samples-per-segment 24 `
  --modes ackermann `
  --fault-reset-before-test `
  --log-dir out\motion8_ackermann_smoke
```

结果：

- Node1-8 均收到 TPDO0/TPDO1；
- EMCY 数量：0；
- 停止后 Node1-4 `0x606C actual_velocity` 均为 0；
- 转向 Node5-8 停止后实际位置回到接近 0 count。

### 4.2 四模式 50 ms 周期测试

命令：

```powershell
python tools\canopen_motion_debug\motion8_remote_sim_debug.py `
  --allow-motion `
  --speed-kph 0.30 `
  --steer-deg 10 `
  --period-ms 50 `
  --samples-per-segment 45 `
  --modes ackermann,reverse_ackermann,crab,spin `
  --log-dir out\motion8_all_modes_030kph_10deg
```

结果：

- command_count：210；
- feedback_count：10040；
- EMCY 数量：0；
- Node1-8 均持续返回 TPDO0/TPDO1；
- 最大脚本发送耗时约 8.254 ms；
- 停止后 Node1-4 实际速度均为 0。

### 4.3 四模式 20 ms 周期测试

命令：

```powershell
python tools\canopen_motion_debug\motion8_remote_sim_debug.py `
  --allow-motion `
  --speed-kph 0.20 `
  --steer-deg 8 `
  --period-ms 20 `
  --samples-per-segment 45 `
  --modes ackermann,reverse_ackermann,crab,spin `
  --log-dir out\motion8_all_modes_020kph_8deg_20ms
```

结果：

- command_count：210；
- feedback_count：10063；
- EMCY 数量：0；
- Node1-8 均持续返回 TPDO0/TPDO1；
- 最大脚本发送耗时约 7.339 ms；
- 停止后 Node1-4 实际速度均为 0。

结论：驱动器和总线能够接受“4 个转向 arm + SYNC + 4 个转向 trigger + SYNC + 4 个速度 RPDO + SYNC”的 20 ms 遥控节奏。Python 分析仪脚本都能做到，ECU C 代码的发送开销应低于脚本。

### 4.4 四模式 20 ms 全角度测试

命令：

```powershell
python tools\canopen_motion_debug\motion8_remote_sim_debug.py `
  --allow-motion `
  --speed-kph 2.0 `
  --steer-deg 45 `
  --spin-deg 45 `
  --period-ms 20 `
  --samples-per-segment 60 `
  --modes ackermann,reverse_ackermann,crab,spin `
  --log-dir out\motion8_all_modes_2kph_45deg_20ms
```

结果：

- command_count：270；
- feedback_count：12921；
- EMCY 数量：0；
- Node1-8 均持续返回 TPDO0/TPDO1；
- 最大脚本发送耗时约 7.043 ms；
- 停止后 Node1-7 实际速度为 0，Node8 读回 -63，接近零速；
- Node5-8 停止后实际位置回到接近 0 count；
- 该测试确认 `±45°` 全角度范围下，四个转向轴可以跟随正 Ackermann、反 Ackermann、蟹行、自转的连续遥控目标。

### 4.5 四模式 20 ms 肉眼可见速度测试

命令：

```powershell
python tools\canopen_motion_debug\motion8_remote_sim_debug.py `
  --allow-motion `
  --speed-kph 6.0 `
  --steer-deg 45 `
  --spin-deg 45 `
  --period-ms 20 `
  --samples-per-segment 60 `
  --modes ackermann,reverse_ackermann,crab,spin `
  --log-dir out\motion8_all_modes_6kph_45deg_20ms
```

结果：

- command_count：270；
- feedback_count：12944；
- EMCY 数量：0；
- Node1-8 均持续返回 TPDO0/TPDO1；
- 最大脚本发送耗时约 7.178 ms；
- 停止后 Node1-8 `0x606C actual_velocity` 均为 0；
- Node1-4 最大反馈速度约为 0.57M 到 0.67M 计数/s；
- Node5-8 最大反馈速度约为 3.26M 到 3.53M 计数/s；
- 该测试确认悬空状态下，速度达到肉眼可见范围时，四轮速度和四轮转向仍能按同一遥控周期同步更新，且无驱动器 EMCY。

### 4.6 对后续 ECU 遥控接入的结论

- `20 ms` 遥控周期可以作为 ECU 初始整车控制周期；
- 速度和转向应继续使用“整车同一采样快照 -> CAN2 统一调度”的模型；
- 正 Ackermann、反 Ackermann、蟹行、自转都可以用当前 RPDO0/RPDO1 布局实现；
- 自转不能再使用写死角度，测试工具和 ECU 控制层都应把自转角度/半径作为明确参数或由遥控量计算；
- 真实落地行走前仍需单独确认轮胎接地负载下的加速度、转向回中、急停、遥控失联和刹车释放策略。

## 5. 回写到 ECU 代码的修改

本次测试暴露并修正了两个 ECU 侧问题：

1. 同步 TPDO 启动死锁。
   - 原逻辑：安全门先要求转向轴 `MOTION_STEER_AXIS_READY`，但 ready 状态又要在 realtime ready 阶段才会设置。
   - 修正：只要 TPDO0/TPDO1 新鲜且 `0x2183 latched fault == 0`，就把对应转向轴提升为 READY。
   - 空闲未 ready 时，ECU 会周期性发送 SYNC，用于拉取同步 TPDO 反馈。

2. 行走安全停止覆盖延迟。
   - 原逻辑：行走速度缓存只在命令变化或 500 ms refresh 时更新。
   - 风险：安全门关闭、刹车释放撤销或遥控失效时，之前非零速度可能不会立即被最新 0 速度覆盖。
   - 修正：`motion_device_apply()` 每个控制周期都更新 RAM 中的四轮速度快照；CAN 发送仍只由 `motion_device_flush_realtime()` 统一调度。

3. 自转/蟹行行走时序。
   - 原逻辑：自转和蟹行可以在转向轴尚未到目标角度时同时输出行走速度，悬空测试表现为动作逻辑不直观，落地后存在轮胎横向冲击风险。
   - 修正：ECU 继续由 vehicle/control 层一次性生成四轮 coherent snapshot，但 `motion_device_apply()` 会在自转/蟹行模式下读取 Node5-8 的 TPDO 位置反馈；四个转向轴均进入 `ECU_CANOPEN_PRESTEER_POSITION_TOLERANCE_COUNTS` 窗口前，Node1-4 的 drive RPDO 缓存为禁用/零速度。
   - 诊断：串口监控输出 `presteer_hold`、`presteer_ready`、`presteer_missing` 和 `presteer_timeouts`，用于判断是哪个转向轴未到位或反馈不新鲜。

## 6. ECU 接入规则

后续 ECU 真机遥控时应保持：

- vehicle/control 层只生成四轮 coherent snapshot；
- CAN2 motion task 统一发 RPDO；
- 转向和速度都通过 CANopen service 单一 FIFO 调度；
- 不允许 remote/parser/debug helper 直接发运动 CAN；
- 安全门关闭时必须覆盖为：
  - 行走速度 0；
  - 行走控制字 disable voltage；
  - 转向停止新目标组；
  - 故障和超时可诊断。

## 7. 下一步整车验证

1. 编译 `ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED` 配置。
2. 若只调转向，保持 `ECU_COMMISSIONING_STEER_ONLY_MODE=1`。
3. 若要行走轮也跟随遥控，确认轮子悬空或现场具备安全条件后再设：

```c
#define ECU_COMMISSIONING_STEER_ONLY_MODE (0U)
```

4. 用 CAN 分析仪检查 ECU 输出顺序：

```text
Node5..8 RPDO1 arm -> SYNC -> Node5..8 RPDO1 trigger -> SYNC
Node1..4 RPDO0 velocity -> SYNC
```

5. 遥控急停、P 档、失联时必须看到 Node1-4 速度清零并失能。
