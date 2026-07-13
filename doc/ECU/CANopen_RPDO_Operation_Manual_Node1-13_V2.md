# CANopen RPDO Operation Manual for Node1-13 V4 Current7 Sync1

> Current operational reference, updated 2026-07-12. The mapping source is
> `doc/CANOPEN_PDO_MAPPING_RECORD_V1.md` and `tools/canopen_pdo_config`.
> Historic Node5-only/compact6 experiments are not production configuration.

This document records the current PDO profile configured on the BC/BC2 drives by
`tools/canopen_pdo_config/configure_all_nodes.py` and gives byte-level commands
for bench commissioning. It does not replace the vendor CANopen manual or ECU
safety state machine.

## 1. Bus and node allocation

| Analyzer channel | Vehicle bus | Bitrate | Nodes |
|---|---|---:|---|
| CAN1 | ECU_CAN2 motion bus | 1 Mbit/s | 1-8 |
| CAN2 | ECU_CAN3 lift/hydraulic bus | 1 Mbit/s | 9-13 |

| Node | Bus | Device role |
|---:|---|---|
| 1 | ECU_CAN2 | front-right drive |
| 2 | ECU_CAN2 | front-left drive |
| 3 | ECU_CAN2 | rear-left drive |
| 4 | ECU_CAN2 | rear-right drive |
| 5 | ECU_CAN2 | front-right steering |
| 6 | ECU_CAN2 | front-left steering |
| 7 | ECU_CAN2 | rear-left steering |
| 8 | ECU_CAN2 | rear-right steering |
| 9 | ECU_CAN3 | front-right lift |
| 10 | ECU_CAN3 | rear-right lift |
| 11 | ECU_CAN3 | front-left lift |
| 12 | ECU_CAN3 | rear-left lift |
| 13 | ECU_CAN3 | hydraulic pump servo |

## 2. Configured PDO profile

All node IDs use the same PDO layout. `N` means node ID.

| PDO | COB-ID | Transmission type | DLC | Mapped objects |
|---|---:|---:|---:|---|
| RPDO0 velocity command | `0x200 + N` | 1 | 7 | `6040:00` u16 + `6060:00` i8 + `60FF:00` i32 |
| RPDO1 position command | `0x300 + N` | 1 | 7 | `6040:00` u16 + `6060:00` i8 + `607A:00` i32 |
| RPDO2 interpolation point | `0x400 + N` | 1 | 4 | `60C1:01` i32 |
| RPDO3 torque/current command | `0x500 + N` | 1 | 5 | `6040:00` u16 + `6060:00` i8 + `2340:00` i16 |
| TPDO0 feedback | `0x180 + N` | 1 | 8 | `6064:00` i32 + `606C:00` i32 |
| TPDO1 status/current | `0x280 + N` | 10 | 8 | `2183:00` u32 + `6041:00` u16 + `221C:00` i16 |

`6060:00` is carried inside RPDO0/RPDO1/RPDO3. This is intentionally less compact
than a 6-byte speed/position command, but it is safer for whole-machine
commissioning because each command frame carries its own expected mode.

TPDO2 and TPDO3 are unmanaged and intentionally unused. The configuration tool
clears `1A02:00` and `1A03:00` to zero entries, so these PDOs have no payload
and do not consume cyclic bandwidth. Do not rely on the `1802:01` / `1803:01`
COB-ID disable bit being retained after power cycling on these drives.

## 3. Common payload rules

All multi-byte values are little-endian.

| Mode | `6060:00` value | Meaning |
|---|---:|---|
| Profile position | `0x01` | absolute/relative position mode |
| Profile velocity | `0x03` | velocity mode |
| Current mode | `0x04` | vendor current mode, runtime current through `2340` |

Common controlwords:

| Controlword | Meaning in this project |
|---:|---|
| `0x0000` | disable output |
| `0x000F` | enable operation |
| `0x002F` | position command armed, bit4 low |
| `0x003F` | position command trigger/update, bit4 high |

For position following, use an arm/trigger edge: first send `0x002F`, then send
`0x003F` after the new target has been loaded.

## 4. RPDO0 velocity command

```text
ID  = 0x200 + node_id
DLC = 7
byte0..1 = 6040 controlword, u16 little-endian
byte2    = 6060 operating mode, 0x03
byte3..6 = 60FF target velocity, i32 little-endian
```

Example: Node1 velocity mode, enable, target velocity `+333333` command units:

```text
COB-ID 0x201
Data   0F 00 03 15 16 05 00
```

Example: Node1 normal velocity stop:

```text
COB-ID 0x201
Data   0F 00 03 00 00 00 00
```

Example: Node1 disable output after speed is already zero:

```text
COB-ID 0x201
Data   00 00 03 00 00 00 00
```

For Node1–8 and Node13, one motor revolution is 10000 position counts and the
verified velocity-object scale is 0.1 count/s per command unit. Therefore
`rpm * 10000 * 10 / 60` gives the velocity command; for example, 500 rpm is
approximately `833333` units. Do not apply this conversion to Node9–12: their
position feedback is 131072 counts/rev and ground-clearance motion uses RPDO2
absolute interpolation points plus separately analyzer-tuned profile values.

## 5. RPDO1 absolute position command

```text
ID  = 0x300 + node_id
DLC = 7
byte0..1 = 6040 controlword, u16 little-endian
byte2    = 6060 operating mode, 0x01
byte3..6 = 607A target position, i32 little-endian
```

Example: Node5 arm absolute target `+500000` counts:

```text
COB-ID 0x305
Data   2F 00 01 20 A1 07 00
```

Example: Node5 trigger/update the same target:

```text
COB-ID 0x305
Data   3F 00 01 20 A1 07 00
```

Example: Node5 target `-500000` counts:

```text
COB-ID 0x305
Data   2F 00 01 E0 5E F8 FF
Data   3F 00 01 E0 5E F8 FF
```

Current steering calibration:

```text
+612500 counts = +45 degrees = left steering
-612500 counts = -45 degrees = right steering
```

For four-wheel steering, generate all wheel targets from one coherent vehicle
control snapshot, then send all arm frames first and all trigger frames second.

Expected analyzer order for one steering update:

```text
0x305 Node5 arm
0x306 Node6 arm
0x307 Node7 arm
0x308 Node8 arm
0x305 Node5 trigger
0x306 Node6 trigger
0x307 Node7 trigger
0x308 Node8 trigger
```

## 6. RPDO3 torque/current command

```text
ID  = 0x500 + node_id
DLC = 5
byte0..1 = 6040 controlword, u16 little-endian
byte2    = 6060 operating mode, 0x04
byte3..4 = 2340 command current, i16 little-endian, unit 10 mA
```

Example: Node5 current mode, enable, command `+0.5 A`:

```text
0.5 A = 50 * 10 mA
COB-ID 0x505
Data   0F 00 04 32 00
```

Example: Node5 command `-0.5 A`:

```text
COB-ID 0x505
Data   0F 00 04 CE FF
```

Example: Node5 current command zero:

```text
COB-ID 0x505
Data   0F 00 04 00 00
```

`2113` current ramp is not in RPDO3. Configure it by SDO when current mode is
being commissioned.

## 7. RPDO2 interpolation point

```text
ID  = 0x400 + node_id
DLC = 4
byte0..3 = 60C1:01 interpolation position point, i32 little-endian
```

RPDO2 is the active ground-clearance interpolation command path for CAN3 lift
nodes. The ECU sends one four-axis `60C1:01` point group and then one SYNC every
20 ms, so RPDO2 must be type 1. Type 4 would execute only every fourth SYNC and
is not suitable for synchronized lift motion.

During this realtime window no unrelated periodic SYNC may be inserted. Node13
pump RPDO0 is therefore sent only when start/stop changes or after a confirmed
speed-loss retry; a completed unchanged pump command is not refreshed on a
timer. This preserves the lift pattern as exactly four RPDO2 frames plus one
common SYNC per 20 ms cycle.

## 8. SYNC usage

The configured profile uses synchronous transmission types:

```text
RPDO0 type 1
RPDO1 type 1
RPDO2 type 1
RPDO3 type 1
TPDO0 type 1
TPDO1 type 10
SYNC COB-ID 0x080
```

For analyzer bench tests:

1. Send RPDO command frames.
2. Send SYNC `0x080` with DLC 0.
3. Check TPDO feedback after SYNC.

For steering edge-trigger tests, keep arm and trigger in separate logical steps:

```text
arm frames for all axes -> SYNC
trigger frames for all axes -> SYNC
```

## 9. Feedback decoding

TPDO0 `0x180 + N`, DLC 8:

| Byte | Field |
|---:|---|
| 0-3 | `6064:00` actual position, i32 little-endian |
| 4-7 | `606C:00` actual velocity, i32 little-endian |

TPDO1 `0x280 + N`, DLC 8:

| Byte | Field |
|---:|---|
| 0-3 | `2183:00` latching fault/status, u32 little-endian |
| 4-5 | `6041:00` statusword, u16 little-endian |
| 6-7 | `221C:00` actual motor current, i16 little-endian |

Do not claim that a command was accepted only because a CAN TX API accepted the
frame. Check TPDO state, SDO readback, analyzer evidence, or actual measured
motion.

## 10. Safe commissioning checklist

Before sending motion commands:

1. Confirm correct bus and node ID.
2. Confirm the drive is in the expected CiA-402 state.
3. Confirm no active drive fault from TPDO1 or SDO diagnostics.
4. For steering, keep target within configured mechanical limits.
5. For drive velocity/current, command zero before disable.
6. For four-axis steering updates, send a coherent group; do not interleave
   different joystick samples in one group.
7. If any frame in a steering group fails or times out, stop drive velocity
   through the safety path and require a defined recovery before sending fresh
   steering groups.
