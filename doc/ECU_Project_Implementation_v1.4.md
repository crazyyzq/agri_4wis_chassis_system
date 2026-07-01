# 四轮四转农业底盘 ECU 项目开发实施说明
## ——遥控器分层状态机、FreeRTOS 双核架构与代码质量规范（v1.4）

> **适用平台**：HPM6750 双核 MCU，FreeRTOS AMP 架构
> **目标**：安全、可读、可测、可扩展；禁止巨型状态机、巨型源文件、全局变量乱写与“能跑就行”的堆叠式代码。
>
> 开发工具:jlink和SEGGER Embedded Studio 8.28

---

# 1. 本版冻结的关键修正

1. **SBUS 在线与遥控动作使能分离。**
   `REMOTE_LINK_ONLINE` 只代表通信帧可信；`REMOTE_ARM_READY` 才代表档位、油门、摇杆、急停和安全条件都满足。驾驶员正常操作油门和转向时，链路状态不得被错误降级。

2. **HOME 切换模式域后，旧锁存按键不能重放。**
   HOME 从低位切高位时，CH15/CH16 当前状态只能作为基线；只有模式域稳定后发生的新按键状态变化，才能触发自转或蟹行。同样规则适用于 SBUS 重连和急停复位。

3. **P 档拆分为两个真实状态。**
   - `PARKED_BRAKED`：普通 P，轮速为零且抱闸有效；
   - `TRACK_COMPLIANT`：仍是 P，禁止普通行驶，但在行距调节中临时允许抱闸释放和低限幅辅助力矩。
   不能只用一个 `is_parked` 布尔变量混过去。

4. **模式请求不排队。**
   请求自转、蟹行、正反阿克曼、调节模式时，只要前置条件不满足就直接拒绝并给出诊断原因；不得等未来停车后自动执行旧请求。

5. **远程急停来源必须可追溯。**
   CH13、SBUS 超时、SBUS failsafe、连续解码错误和遥控可信度异常都可走同一受控停车链路，但必须保存不同的 `estop_source`。

6. **行距调节是整体行距控制，不伪造单杆独立闭环。**
   当前四根横移推杆共用液压阀回路时，只产生一个整体行距变化率；多路传感器只用于一致性、限位和故障判断。未增加独立液压支路前，禁止软件假装能单独修正某根推杆。

7. **状态机只决策，不直接打硬件。**
   状态机只输出请求、准入结果、目标状态与诊断原因；CANopen、DO、抱闸和高压最终由 `command_arbiter + safety_manager + vehicle_command_executor` 统一执行。

8. **车身方位和腿号定义冻结。**
   以车头方向为整车正方向。所有行走、转向、抬升数组均按腿号顺序 `1, 2, 3, 4` 保存，不按常见的 `FL, FR, RL, RR` 顺序保存：
   - 1 号行走轮、1 号转向轮、1 号抬升电机：车头右侧，前轮；
   - 2 号行走轮、2 号转向轮、2 号抬升电机：车头左侧，前轮；
   - 3 号行走轮、3 号转向轮、3 号抬升电机：车头左侧，后轮；
   - 4 号行走轮、4 号转向轮、4 号抬升电机：车头右侧，后轮。
   因此代码中的物理腿号顺序为 `前右、前左、后左、后右`。若需要按方位阅读代码，必须通过配置中的语义别名确认，不能自己假设 0 号数组元素是前左。

---

# 2. 代码质量红线

## 2.1 明确禁止

- 不准把遥控、能源、档位、急停、模式、灯光、液压、通信写进一个大 `switch` 或一个大循环；
- 不准在业务代码中直接写 CAN ID、PDO、MOS 编号、GPIO、引脚或 `1050/1500/1950`；
- 不准让多个任务或模块随意修改同一个全局状态；
- 不准让遥控事件直接调用电机、高压、抱闸、液压阀；
- 不准用 `vTaskDelay()`、`HAL_Delay()`、忙等等待设备反馈；
- 不准让 CPU1 直接控制行走、转向、抱闸、高压或液压阀；
- 不准在 ISR 内运行协议状态机、打印、EEPROM、网络、运动学或阻塞操作；
- 不准在运行期频繁 `malloc/free`；
- 不准用大量 `bool` 代替复杂状态；
- 不准对非法状态、超时或拒绝原因“默认不处理”；
- 不准使用 `common.c`、`misc.c`、`utils.c` 这类没有边界的垃圾桶文件。

## 2.2 必须做到

- 一个业务状态只使用一个明确枚举；
- 一个状态机只有一个 owner 任务；
- 状态机提供 `init()`、`update()`、`get_state()`，必要时提供 `reset()`；
- 输入快照、状态机、仲裁器、执行器四层分开；
- 所有硬件命令从 CPU0 的唯一最终执行出口发出；
- 所有超时都有成功条件、失败动作、诊断码和恢复路径；
- 所有物理映射、阈值、方向、限位、节点号、极性集中在 `config/`；
- 每个公共函数在头文件中写清输入单位、前置条件、失败行为、任务归属和是否可在 ISR 中调用；
- 每个新增功能必须有单元测试、状态变迁测试和至少一条故障注入测试。

---

# 3. 单向依赖与唯一执行出口

## 3.1 依赖方向

```text
board / drivers
        ↓
protocol
        ↓
devices
        ↓
control
        ↓
vehicle
        ↓
app

remote ───────────→ vehicle
diag   ───────────→ vehicle_snapshot
CPU1 IPC ─────────→ vehicle
```

约束：

- `drivers/` 只处理硬件和通信；
- `protocol/` 只处理 CANopen、SBUS、Modbus 等协议；
- `devices/` 只封装单设备协议与状态；
- `remote/` 只产生遥控请求，不 include CAN、DO、电机模块；
- `control/` 只做运动学、同步、规划和设备目标；
- `vehicle/` 负责安全、仲裁、模式和权限；
- `app/` 只启动系统和注册任务。

禁止循环依赖。`control/` 不得依赖 `remote/`；`remote/` 不得依赖 `devices/` 或 `drivers/can/`。

## 3.2 最终动作出口

CPU0 中只有 `vehicle_command_executor` 可以把最终命令发给：

- CAN1 能源管理；
- CAN2 行走、转向、抱闸；
- CAN3 抬升、液压站；
- 液压阀、灯光、喇叭等 DO。

```text
遥控器 / 自动控制 / CAN4 / Ethernet / 维护接口
                    ↓
               标准化请求
                    ↓
            command_arbiter
                    ↓
            safety_manager 覆盖
                    ↓
         vehicle_actuator_command
                    ↓
       vehicle_command_executor
                    ↓
   CANopen / DO / CAN1 / 设备抽象层
```

任何来源都不得绕过该链路。

---

# 4. 推荐目录结构

```text
ecu/
├─ app/                 # CPU0/CPU1 启动、任务注册
├─ board/               # 时钟、引脚、板级初始化
├─ drivers/             # CAN/UART/SBUS/DIO/ADC/EEPROM/Ethernet
├─ protocol/            # CANopen、Modbus、SBUS
├─ devices/             # BMS、DC/DC、BC/BC2、IMU、警示灯
├─ remote/              # 遥控输入、事件、各 FSM、遥控管理器
├─ vehicle/             # 安全、仲裁、整车状态、模式过渡、最终执行
├─ control/             # 行走、转向、四腿同步、行距、液压、灯光
├─ config/              # 标定、默认值、设备映射、EEPROM
├─ os/                  # FreeRTOS、看门狗、任务监控
├─ ipc/                 # 双核快照、邮箱、环形队列
├─ diag/                # 故障码、诊断、日志
└─ tests/               # 单测、状态机、HIL
```

文件边界要求：

- 单个 `.c` 超过约 800 行、单函数超过约 80 行、或连续嵌套超过 3 层时，必须在评审中说明并优先拆分；
- public API 少而稳定；内部函数尽可能 `static`；
- 不允许把不同状态机互相的私有状态暴露给外部直接改写。

---

# 5. FreeRTOS 双核 AMP

## 5.1 CPU0：安全与实时控制

CPU0 负责：SBUS、急停、安全仲裁、CAN1、CAN2、CAN3、D/P/R、模式过渡、抱闸、液压安全互锁、本地 DI/DO/ADC、看门狗。

| 任务 | 周期 | 优先级 | 职责 |
|---|---:|---|---|
| `task_safety_supervisor` | 1 ms | 最高 | 急停、关键故障、受控停车、看门狗 |
| `task_can2_motion` | 1～5 ms | 很高 | 行走/转向 PDO、状态、命令刷新 |
| `task_remote_manager` | 5 ms | 高 | 遥控快照、事件、遥控 FSM |
| `task_vehicle_control` | 5～10 ms | 高 | 仲裁、运动学、模式过渡 |
| `task_can1_power` | 10～20 ms | 中高 | 高压、BMS、DC/DC、逆变器 |
| `task_can3_lift_hydraulic` | 10～20 ms | 中高 | 四腿同步、液压站、液压阀 |
| `task_io_service` | 5～20 ms | 中 | DI/DO/ADC |
| `task_diag_cpu0` | 50～100 ms | 低 | 诊断快照、运行统计 |

## 5.2 CPU1：非关键通信和数据服务

CPU1 负责 CAN4、RS485、RS232、Ethernet、EEPROM 延迟写入、日志、自动控制请求的格式校验与 IPC 发布。

CPU1 禁止：下发行走速度、转向角、释放抱闸、打开液压阀、上高压、清除急停。

CPU1 异常、重启或 IPC 超时，只退出依赖 CPU1 的自动控制、IMU 调平和外部通信；CPU0 基础人工行走、转向、急停必须继续有效。

## 5.3 ISR 纪律

ISR 仅允许搬运数据、打时间戳、置错误位、写环形缓冲、通知任务。ISR 禁止状态机、CANopen、打印、EEPROM、网络和等待。

---

# 6. 数据所有权、快照与事件生命周期

## 6.1 一个状态只有一个写入者

| 数据 | 唯一写入者 |
|---|---|
| SBUS 原始帧 | `sbus_service` |
| 遥控稳定输入与 FSM | `task_remote_manager` |
| CAN2 设备状态 | `task_can2_motion` |
| 安全快照 | `vehicle_state` |
| 最终执行命令 | `command_arbiter + safety_manager` |
| CPU1 IPC 发布快照 | `task_ipc_service` |

禁止多个模块通过全局变量直接改写同一状态。

## 6.2 快照与 IPC

任务间和跨核使用双缓冲快照或有界队列。每份跨核数据必须带：

- 序号；
- 时间戳；
- 有效位；
- CRC 或校验；
- 明确的超时策略。

`volatile` 不能替代原子性、临界区、缓存维护与内存屏障。

## 6.3 事件必须过期

| 事件 | 有效期 | 过期后 |
|---|---:|---|
| 模式请求 | 250 ms | 丢弃，需重新操作 |
| 上/下高压请求 | 500 ms | 丢弃，需重新长按 |
| 急停触发 | 永不过期 | 立即锁存 |
| 急停复位请求 | 1000 ms | 需重新长按 |
| 灯光切换 | 1000 ms | 可丢弃 |

不安全时的模式、档位、高压请求不得排队等待未来执行。

---

## 6.4 整机 CANopen 伺服接口定义

CAN2 是控制整车的行走和转向部分，总线上有 8 个 BC 系列伺服驱动器，采用 CANopen 协议，具体参考 `doc/伺服电机驱动/DC和DE Canopen应用说明251022/CAN应用说明 - 旋转电机新.pdf` 和 `doc/伺服电机驱动/DC和DE Canopen应用说明251022/DCH_CANopen.pdf`。

CAN2 节点按功能连续分配：行走电机为节点 1-4，转向电机为节点 5-8。

| 功能 | 节点 | 驱动器 | 安全 I/O |
|---|---:|---|---|
| 腿 1 行走 | 1 | BC 系列 | 抱闸：J3 中的引脚 16 OUT1 |
| 腿 2 行走 | 2 | BC 系列 | 抱闸：J3 中的引脚 16 OUT1 |
| 腿 3 行走 | 3 | BC 系列 | 抱闸：J3 中的引脚 16 OUT1 |
| 腿 4 行走 | 4 | BC 系列 | 抱闸：J3 中的引脚 16 OUT1 |
| 腿 1 转向 | 5 | BC 系列 | 正限位：J3 引脚 5 IN2；负限位：J3 引脚 6 IN3 |
| 腿 2 转向 | 6 | BC 系列 | 正限位：J3 引脚 5 IN2；负限位：J3 引脚 6 IN3 |
| 腿 3 转向 | 7 | BC 系列 | 正限位：J3 引脚 5 IN2；负限位：J3 引脚 6 IN3 |
| 腿 4 转向 | 8 | BC 系列 | 正限位：J3 引脚 5 IN2；负限位：J3 引脚 6 IN3 |

IN2 是正限位，IN3 是负限位。ECU 通过 CANopen 读取驱动器输入状态对象 `0x2190`，按位读取 IN2/IN3；不得使用 PCB 本地输入替代转向限位。行走电机抱闸通过驱动器输出对象 `0x2194` 控制 OUT1；不得使用 PCB 输出控制电机抱闸。

CAN3 是整车的抬升功能（变地隙）实现和液压站电机控制，与两台 BC2 系列伺服驱动器和一台 BC 系列驱动器通讯。每台 BC2 带两台伺服电机，一台伺服电机控制 1 个脚的抬升；一台 BC 系列驱动器控制液压站（变行距液压杆）电机。CAN3 采用 CANopen 协议，参考同一套 CANopen 应用说明。

CAN3 节点分配：

| 功能 | 节点 | 驱动器 | 安全 I/O |
|---|---:|---|---|
| 腿 1 抬升 A 轴 | 9 | BC2 前驱动器 | 抱闸：控制信号 I/O 端子 J3 的引脚 8 OUT1；正限位：引脚 4 IN2_GP；负限位：引脚 5 IN3_GP |
| 腿 2 抬升 B 轴 | 10 | BC2 前驱动器 | 抱闸：引脚 17 OUT4；正限位：引脚 13 IN7_GP；负限位：引脚 14 IN8_GP |
| 腿 3 抬升 A 轴 | 11 | BC2 后驱动器 | 抱闸：控制信号 I/O 端子 J3 的引脚 8 OUT1；正限位：引脚 4 IN2_GP；负限位：引脚 5 IN3_GP |
| 腿 4 抬升 B 轴 | 12 | BC2 后驱动器 | 抱闸：引脚 17 OUT4；正限位：引脚 13 IN7_GP；负限位：引脚 14 IN8_GP |
| 液压站电机 | 13 | BC 系列 | 由 CANopen 速度/使能命令控制 |

BC2 的 SW 拨码只设置 A 轴节点号，B 轴节点号等于 A 轴节点号加 1。例如前 BC2 拨码为 9 时，A 轴为节点 9，B 轴为节点 10；后 BC2 拨码为 11 时，A 轴为节点 11，B 轴为节点 12。虽然 B 轴物理端子命名为 OUT4/IN7/IN8，但 CANopen 访问必须通过 B 轴自己的节点号进行。B 轴节点访问 `0x2194` 时仍写轴内 OUT1 对应的 bit0，访问 `0x2190` 时仍读轴内 IN2/IN3 对应的限位位。伺服电机默认抱闸，当前接线为 OUT 输出低时释放刹车；代码中必须保留抱闸输出极性宏，便于现场发现驱动器输出配置相反时快速调整。

A 轴电机的抱闸是控制信号 I/O 端子 J3 的引脚 8 OUT1。B 轴电机的抱闸是引脚 17 OUT4。

---

# 7. 遥控器输入层

## 7.1 链路 FSM 与 Arm FSM 分离

```c
typedef enum {
    REMOTE_LINK_OFFLINE = 0,
    REMOTE_LINK_QUALIFYING,
    REMOTE_LINK_ONLINE,
    REMOTE_LINK_FAILSAFE,
} remote_link_state_t;

typedef enum {
    REMOTE_ARM_DISARMED = 0,
    REMOTE_ARM_WAIT_NEUTRAL,
    REMOTE_ARM_READY,
} remote_arm_state_t;
```

`REMOTE_LINK_ONLINE` 仅要求帧有效、无 failsafe、帧率与解码正常。
`REMOTE_ARM_READY` 才要求：CH5=P、CH3 低位、CH1/CH2/CH4/CH14 中位、CH13 正常、无急停、无 A 类故障。

### Arm 状态的保持规则

中立条件只用于**进入** `REMOTE_ARM_READY`，不是要求车辆在驾驶过程中持续保持所有摇杆中立。

```text
DISARMED / WAIT_NEUTRAL
→ 满足安全中立条件并稳定
→ ARM_READY

ARM_READY
→ 正常驾驶时，即使 CH3、CH1 发生有效操作，仍保持 ARM_READY
→ 仅在 SBUS 失联、远程急停、A 类故障、CPU0 重启、远程控制权撤销或安全管理明确失能时回到 DISARMED
```

这样可以避免“驾驶员正常踩油门后遥控器突然被判定未使能”的错误实现。

## 7.2 离散通道去抖与基线同步

```c
typedef enum {
    REMOTE_POS_LOW = 0,
    REMOTE_POS_CENTER,
    REMOTE_POS_HIGH,
    REMOTE_POS_INVALID,
} remote_position_t;

typedef struct {
    remote_position_t candidate_position;
    remote_position_t stable_position;
    remote_position_t previous_stable_position;
    uint32_t candidate_since_ms;
    uint32_t stable_since_ms;
    bool changed;
} remote_discrete_channel_t;
```

规则：

1. 原始值跨阈值只更新候选状态；
2. 候选状态稳定达到去抖时间后才更新稳定状态；
3. 稳定状态变化只生成一次事件；
4. 首帧、SBUS 重连、HOME 域变化、急停复位后必须做基线同步；
5. 基线同步期间禁止普通模式、灯光、喇叭、自动控制切换事件。

建议：

```c
#define REMOTE_DISCRETE_DEBOUNCE_MS       80U
#define REMOTE_LINK_QUALIFY_MS          1000U
#define REMOTE_NEUTRAL_QUALIFY_MS        300U
#define REMOTE_FAILSAFE_TIMEOUT_MS       100U
#define REMOTE_DOMAIN_EVENT_GUARD_MS     150U
```

## 7.3 HOME 切域的旧事件抑制

HOME 域变化时：

```text
HOME 域变化
→ 进入 DOMAIN_EVENT_GUARD
→ 记录 CH7、CH8、CH9、CH10、CH11、CH12、CH15、CH16 当前稳定状态为新基线
→ 等待 150 ms
→ 只接受此后产生的新普通状态变化
```

**CH13 不参与事件抑制。** 无论首帧、域切换、基线同步还是防抖保护期间，只要 CH13 被识别为急停高位，必须立即触发远程急停。
CH5 的 D/P/R 也不参与事件抑制，它始终作为连续的档位请求输入；但实际档位是否生效仍由档位 FSM 决定。

因此 R1/R2 原本处于高位，不会在切进特殊域时自动触发自转或蟹行；同时不会牺牲急停和档位安全性。

---

# 8. 遥控分层状态机

## 8.1 急停 FSM

```c
typedef enum {
    REMOTE_ESTOP_CLEAR = 0,
    REMOTE_ESTOP_LATCHED,
    REMOTE_ESTOP_RESET_REQUESTED,
    REMOTE_ESTOP_CLEAR_WAIT_NORMAL,
} remote_estop_state_t;
```

急停来源：CH13 高位、SBUS 超时、SBUS failsafe、连续解码异常、遥控可信度异常。

CH13 的急停判定优先于普通离散按键去抖和域切换基线同步。SBUS 首帧或重连首帧中检测到 CH13 急停高位时，必须直接进入急停锁存，不得等待普通按键稳定时间。

```text
CLEAR → LATCHED：任一急停来源
LATCHED → RESET_REQUESTED：CH13 长按至中位且满足安全前置
RESET_REQUESTED → CLEAR_WAIT_NORMAL：安全条件稳定满足
CLEAR_WAIT_NORMAL → CLEAR：CH13 返回低位
```

急停动作：清空普通与自动控制、速度归零、Quick Stop/受控停车、禁止模式/地隙/行距/液压、关液压阀、停液压站、低速后抱闸、急停警示。

急停时不得立即断高压，必须先保障受控停车与抱闸。

## 8.2 能源 FSM

CH4 右拨保持 1 秒为上高压请求；左拨保持 1 秒为有序下电请求。

上电条件：P、零速、CH3 低位、CH1/CH2/CH14 中位、遥控链路在线且已 arm、无急停、无 A 类故障、BMS/互锁/低压/CAN1 正常。

下电条件：P、停车和抱闸确认、液压停止、液压阀关闭、无地隙/行距/模式过渡、无受控停车中。

不满足条件时直接拒绝，禁止延迟执行。

## 8.3 档位 FSM

```c
typedef enum {
    GEAR_STATE_PARKED_BRAKED = 0,
    GEAR_STATE_ARM_D,
    GEAR_STATE_ARM_R,
    GEAR_STATE_DRIVE_D,
    GEAR_STATE_DRIVE_R,
    GEAR_STATE_STOPPING,
    GEAR_STATE_TRACK_COMPLIANT,
    GEAR_STATE_SHIFT_REJECTED,
} remote_gear_state_t;
```

- `PARKED_BRAKED`：普通 P，抱闸有效；
- `TRACK_COMPLIANT`：P 档下仅允许行距调节的临时顺从状态；
- `ARM_D/ARM_R`：D/R 已请求，但等待 CH3 低位重新使能；
- `DRIVE_D/DRIVE_R`：允许速度命令。

P→D/R：

```text
选择 D/R
→ requested_gear 更新为 D/R
→ active_gear 仍保持 P
→ 检查安全条件
→ ARM_D / ARM_R
→ 观察到 CH3 低位稳定
→ CH3 再次超过起步阈值
→ 释放抱闸
→ active_gear 更新为 D/R
→ 进入 DRIVE_D / DRIVE_R
```

因此必须同时维护 `requested_gear` 与 `active_gear`。物理 CH5 已切到 D/R 但尚未完成油门重新使能、抱闸释放或驱动器确认时，实际档位仍为 P。

D↔R 有速度时直接互切：拒绝、进入受控停车或保持原安全状态、要求驾驶员先回 P。

## 8.4 HOME 与运动模式 FSM

HOME 域：

| HOME | 域 | 子模式 |
|---|---|---|
| 低位 | 阿克曼域 | 正阿克曼、反阿克曼 |
| 中位 | 调节域 | 地隙、行距 |
| 高位 | 特殊域 | 自转、蟹行 |

模式必须分离：

```text
requested_motion_mode
→ transition_motion_mode
→ active_motion_mode
```

HOME 物理位置、请求模式和实际生效模式也必须分开保存：

```text
requested_domain：遥控器 HOME 当前请求的模式域
active_domain：已完成退出/过渡后允许控制的实际模式域
requested_motion_mode：R1/R2 请求的目标模式
active_motion_mode：四轮姿态和安全条件确认后的实际模式
```

物理按键已经变化不等于车辆已经处于该模式；诊断界面必须同时显示请求态、过渡态、实际态和失败原因。

所有模式切换前置：P、零速、CH3 低位、CH1/CH2/CH14 中位、无急停、无 A 类故障、行走/转向正常、无其他过渡、模式事件为新事件。

失败时：不排队、不自动重试；保持当前 active 模式。若已经进入转向过渡发生异常，则速度归零、保持 P、抱闸、记录模式过渡失败。

阿克曼域：CH15/R1 选择正阿克曼；CH16/R2 选择反阿克曼。反阿克曼时 CH1 左推仍必须使实际行驶轨迹左偏。

特殊域：CH15/R1 选择自转；CH16/R2 选择蟹行。

- 自转：CH1 无效；CH3 控制速度；D/R 控制配置定义的正反自转方向；界面必须显示真实自转方向。
- 蟹行：CH1 产生共同蟹行角变化率，回中保持角度；高速冻结蟹行角；D/R 控制沿当前蟹行方向正反行驶。

## 8.5 调节 FSM

```c
typedef enum {
    ADJUST_STATE_IDLE = 0,
    ADJUST_STATE_CLEARANCE_ACTIVE,
    ADJUST_STATE_TRACK_PREPARE,
    ADJUST_STATE_TRACK_ACTIVE,
    ADJUST_STATE_TRACK_EXITING,
    ADJUST_STATE_ABORTED,
} remote_adjust_state_t;
```

调节域准入：HOME 中位、P、零速、CH3 低位、CH1/CH2/CH14 中位、自动控制退出、无急停/A 类故障、行走与转向通信正常。

### CH2：变地隙

CH2 的含义是**目标车身高度变化率**：

```text
CH2
→ body_height_rate_cmd
→ 目标高度积分与限幅
→ 每腿目标位置
→ 每腿位置闭环、误差补偿、速度规划
→ 四个抬升轴各自命令
```

禁止把 CH2 的同一个速度值原封不动发给四条腿。

必须有：目标高度限幅、积分防饱和、四腿差异保护、单腿软硬限位、超时、扭矩异常、CAN3 超时。

### CH14：变行距

行距流程：

```text
CH14 有效
→ TRACK_PREPARE
→ 四轮转到行距姿态
→ 角度到位确认
→ 行走速度=0
→ 抱闸受控释放
→ GEAR_STATE_TRACK_COMPLIANT
→ TRACK_ACTIVE
→ 回中/到位/异常
→ 关阀、液压软停、助力矩归零、轮速为零、抱闸
```

行距由共用液压阀组主导。行走电机仅提供低限幅辅助扭矩，禁止下发普通行驶速度。

每轮行距配置必须参数化：

```c
typedef struct {
    float steer_target_deg[4];
    float assist_torque_sign[4];
    float assist_torque_limit_nm[4];
    float assist_wheel_speed_limit_rpm[4];
} track_adjust_config_t;
```

CH2/CH14 互斥：第一个超过阈值的输入获得 owner；owner 有效期间另一输入被屏蔽；两者均回中稳定后才释放 owner。

### 调节模式的强制退出

以下任一条件出现时，不论 CH2/CH14 当前位置如何，都必须进入 `ADJUST_STATE_ABORTED` 或 `TRACK_EXITING`：

- CH5 离开 P；
- HOME 离开中位；
- CH13 急停或 SBUS failsafe；
- A 类故障；
- 行走、转向、抬升或液压关键通信异常；
- 行距轮角未到位、辅助轮速超限、辅助扭矩超限或传感器越界。

退出顺序固定：停止新的调节请求 → 关闭液压阀 → 液压站软停止 → 辅助扭矩归零 → 轮速确认零 → 抱闸。
物理 CH5 虽然已经被拨到 D/R，`active_gear` 仍必须保持 P，直到调节退出完成且驾驶员重新完成 D/R 使能流程。

## 8.6 控制权 FSM

CH7 中位是自动控制请求；低位/高位是人工控制请求。自动进入仅允许正/反阿克曼，且要求 P、零速、低油门、摇杆安全、无过渡、外部自动请求有效。

以下任一条件立即人工接管：CH7 离开中位、CH1/CH3 有效操作、CH5/HOME 变化、急停、SBUS 异常、外部请求超时或 A 类故障。接管后 CH3 必须重新回低位。

## 8.7 灯光与喇叭 FSM

```c
typedef enum {
    INDICATOR_OFF = 0,
    INDICATOR_LEFT,
    INDICATOR_RIGHT,
    INDICATOR_HAZARD_USER,
    INDICATOR_HAZARD_SAFETY,
} indicator_mode_t;
```

优先级：安全双闪 > 用户双闪 > 左/右转 > 关闭。

CH8、CH12、CH9 只改状态请求。切换时先关闭旧输出，等待 100 ms，再开启新输出。双闪左右必须共用节拍源。

CH10 高位喇叭开、低/中位关；SBUS 失联、低压关机、DO 故障时关。CH11 高位大灯开、低/中位关，最终还要通过低压与 DO 诊断。

---

# 9. 仲裁与最终命令

状态机产生请求；控制器产生候选命令；仲裁器生成唯一最终命令；安全管理覆盖危险字段；执行器下发硬件。

```c
void command_arbiter_update(
    const remote_control_request_t *remote,
    const auto_control_request_t *auto_request,
    const vehicle_safety_snapshot_t *safety,
    uint32_t now_ms,
    vehicle_actuator_command_t *out);
```

固定优先级：

```text
硬件安全 / A 类故障
> 远程急停 / SBUS failsafe
> 受控停车 / 抱闸 / 下电保护
> 模式过渡 / 调节退出
> 人工控制
> 自动控制
> 灯光、喇叭、非关键请求
```

最终命令必须每周期完整重建：

```text
安全默认命令
→ 候选请求
→ 仲裁选择
→ 安全覆盖
→ 完整最终命令
→ 执行
```

禁止多个模块对上一周期命令“打补丁”，避免残留速度、扭矩、液压阀或抱闸状态。

---

# 10. 命名、注释、接口与配置规范

## 10.1 命名必须带语义和单位

正确：

```c
float vehicle_speed_kph;
float target_height_mm;
float height_rate_mm_s;
uint32_t timeout_ms;
bool brake_release_confirmed;
```

禁止：

```c
float speed;
float val;
int flag;
bool status;
uint32_t t;
```

## 10.2 注释解释原因和约束

正确：

```c
/* P 档下仅行距调节顺从状态允许临时松抱闸；
 * 该状态永久禁止普通行驶速度命令。 */
```

无效：

```c
/* 设置速度为零 */
speed = 0;
```

## 10.3 配置化

以下必须通过配置和 EEPROM 管理：SBUS 阈值、去抖、长按、超时、速度、角度、扭矩、行程、加速度、节点号、DO 极性、液压映射、轮子方向、转向零位、行距姿态、抱闸时序。

业务代码禁止写：

```c
if (ch3 > 1900) { ... }
if (speed > 0.2f) { ... }
```

应通过标定与配置接口判断。

---

# 11. 非阻塞和故障处理规则

错误写法：

```c
brake_release();
vTaskDelay(pdMS_TO_TICKS(500));
send_drive_speed();
```

正确做法：

```text
状态 = WAIT_BRAKE_RELEASE
→ 等待反馈或超时
→ 成功进入 SEND_SPEED
→ 超时进入故障退出
```

每个等待必须有：成功条件、截止时间、超时动作和诊断码。

模式、行距、液压、高压、抱闸等退出时，必须显式清理本模块拥有的输出，不能依赖“下一周期应该会覆盖”。

---

# 12. 测试与代码评审门槛

## 12.1 PC 单元测试必须覆盖

- SBUS 去抖和首帧基线；
- HOME 切域后的 R1/R2 旧状态抑制；
- 急停触发、复位、回低位确认；
- D/P/R 与 CH3 重新使能；
- 非法 D↔R 拒绝；
- 模式请求不排队；
- CH2/CH14 owner 互锁；
- 行距准备、进入、异常退出；
- 左/右/双闪状态机；
- 仲裁优先级。

每个测试都必须写：初始状态 → 输入序列 → 时间推进 → 期望状态 → 期望输出 → 期望诊断原因。

## 12.2 HIL/实车测试必须覆盖

- SBUS 超时、failsafe、连续解码异常；
- CH13 急停与复位；
- 高油门挂 D/R；
- 行驶中 D↔R、D/R→P；
- 模式切换中急停；
- 转向到位超时；
- 行距传感器超范围、轮速超限、辅助扭矩异常；
- CPU1 重启、IPC CRC 错误、Ethernet 洪泛；
- EEPROM 写入期间的遥控/CAN2 周期稳定性。

## 12.3 代码评审十问

1. 模块是否只有一个职责？
2. 是否绕过仲裁直接操作硬件？
3. 是否引入新的全局可写状态？
4. 是否混淆离散事件与持续状态？
5. 是否包含阻塞等待？
6. 是否有魔法数？
7. 是否定义超时、失败动作与诊断？
8. 是否可脱离硬件测试？
9. CPU1 故障会不会影响基础人工行走？
10. SBUS 首帧、模式切换、锁存状态会不会误触发？

任一项没有明确答案，不得合并。

---

# 13. 文档级复审结论

## 第一轮：控制权与安全

已确认：遥控、自动和外部请求均不能绕过 CPU0；远程急停覆盖所有普通请求；非关键通信异常不无条件停止人工行走。

**结论：通过。**

## 第二轮：锁存键、档位和模式

已修正：HOME 低/高短按不经过中位；切域后抑制旧 R1/R2；D/R 不直接松抱闸；CH3 每次重新使能须回低；模式请求不排队。

**结论：通过。**

## 第三轮：地隙和行距

已修正：CH2 为目标高度变化率而非四轴同速；P 档拆分普通驻车与行距顺从；行距调节为整体液压目标；异常退出顺序固定。

**结论：通过。**

## 第四轮：并发与可维护性

已修正：单一写入者、快照、事件有效期、CPU0 唯一执行出口、ISR 轻量化、非阻塞状态等待、无巨型状态机。

**结论：通过。**

---

# 14. 仍需实车确认的项目

1. CH6～CH13、CH15、CH16 的真实 SBUS 范围与短按/长按时序；
2. CH13 高位急停、中位复位、低位正常的实际方向；
3. BC/BC2 抱闸、限位、电平与反馈时序；
4. 四轮零位、方向、正反阿克曼、自转、蟹行轮速符号；
5. 行距姿态、辅助扭矩方向、最大扭矩和轮速；
6. 液压阀 1～6 的最终映射；
7. 转向灯、双闪、喇叭和大灯的 MOS/继电器映射；
8. BMS、DC/DC、逆变器上电反馈与下电顺序；
9. 自动控制协议、刷新率、超时与权限。

---

# 15. 最终冻结规则

以下任何一项视为违反开发规范：

1. 遥控输入直接写 CANopen PDO、GPIO、MOS 或抱闸；
2. CPU1 直接控制基础行走、转向、高压或液压阀；
3. D/R 请求立即松抱闸；
4. 急停复位后自动恢复行驶；
5. 模式请求在不安全条件下排队；
6. 行距轮角未到位时释放抱闸；
7. 把 CH2 同一速度命令直接发给四个抬升轴；
8. 用未标定 SBUS 原始值进入业务逻辑；
9. 用全局变量、阻塞延时和巨型状态机拼逻辑；
10. 让非关键外部通信故障直接禁止基础人工行走。

> **先把边界写清楚，再写功能。**
> **状态机只做决策，执行器只接最终命令。**
> **宁可多一个边界清楚的小模块，也不要多一坨“万能代码”。**

---
