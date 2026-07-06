# CANopen RPDO Operation Manual for Node1-13 V2

This document records the PDO profile configured on the BC/BC2 drives by
`tools/canopen_pdo_config/configure_all_nodes.py` and gives the byte-level
commands required for bench commissioning.

It is an operation note for the current vehicle hardware. It does not replace
the vendor CANopen manual or the ECU safety state machine.

## 1. Bus and node allocation

Analyzer connection used during configuration:

| Analyzer channel | Vehicle bus | Bitrate | Nodes |
|---|---|---:|---|
| CAN1 | ECU_CAN2 motion bus | 1 Mbit/s | 1-8 |
| CAN2 | ECU_CAN3 lift/hydraulic bus | 1 Mbit/s | 9-13 |

Node allocation:

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
| RPDO2 interpolation point | `0x400 + N` | 4 | 4 | `60C1:01` i32 |
| TPDO0 feedback | `0x180 + N` | 1 | 8 | `6064:00` i32 + `606C:00` i32 |
| TPDO1 status/current | `0x280 + N` | 4 | 8 | `2183:00` u32 + `6041:00` u16 + `221C:00` i16 |

Reserved PDOs:

| Reserved PDO | Communication object | Mapping object | Intended state |
|---|---|---|---|
| RPDO3 | `1403:01` | `1603:00` | mapping count 0 |
| TPDO2 | `1802:01` | `1A02:00` | mapping count 0 |
| TPDO3 | `1803:01` | `1A03:00` | mapping count 0 |

Power-cycle verification on 2026-07-06 showed that the active PDO layouts
above persisted after Flash save. The reserved PDO mapping counts also remained
0. The drives read back the reserved PDO COB-ID disable bit as cleared after
power cycle, so do not use the COB-ID disable bit alone as the acceptance
criterion for reserved PDOs on this hardware.

## 3. Common payload rules

All multi-byte values are little-endian.

CANopen operating mode values used here:

| Mode | `6060:00` value | Meaning |
|---|---:|---|
| Profile position | `0x01` | absolute/relative position mode |
| Profile velocity | `0x03` | velocity mode |
| Torque/current mode | `0x04` | vendor current mode, normally controlled by SDO `2340` |

Common CiA-402 / vendor controlwords used during commissioning:

| Controlword | Meaning in this project |
|---:|---|
| `0x0000` | disable output |
| `0x000F` | enable operation |
| `0x002F` | position command armed, bit4 low |
| `0x003F` | position command trigger/update, bit4 high |

For position following, do not send only the final trigger value repeatedly.
Use an arm/trigger edge: first send `0x002F`, then send `0x003F` after the new
target has been loaded.

## 4. RPDO0 velocity command

COB-ID:

```text
0x200 + node_id
```

Payload:

| Byte | Field |
|---:|---|
| 0-1 | `6040:00` controlword, u16 little-endian |
| 2 | `6060:00` operating mode, i8 |
| 3-6 | `60FF:00` target velocity, i32 little-endian |

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

The project commissioning limit has used 200 rpm as a safe test maximum.
The known manual example gives 500 rpm as `833333` command units, so 200 rpm is
approximately `333333` command units. Keep the final scale configurable in ECU
configuration and verify with measured shaft speed before production use.

## 5. RPDO1 absolute position command

COB-ID:

```text
0x300 + node_id
```

Payload:

| Byte | Field |
|---:|---|
| 0-1 | `6040:00` controlword, u16 little-endian |
| 2 | `6060:00` operating mode, i8 |
| 3-6 | `607A:00` target position, i32 little-endian |

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
+500000 counts = +45 degrees = left steering
-500000 counts = -45 degrees = right steering
```

That means:

```text
counts_per_degree = 500000 / 45 = 11111.111...
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

Do not wait for Node5 target reached before sending Node6/7/8. Each steering
axis must keep independent state.

## 6. RPDO2 interpolation point

COB-ID:

```text
0x400 + node_id
```

Payload:

| Byte | Field |
|---:|---|
| 0-3 | `60C1:01` interpolation position point, i32 little-endian |

Example: Node5 interpolation point `+500000` counts:

```text
COB-ID 0x405
Data   20 A1 07 00
```

RPDO2 is configured with transmission type 4. Use it only when the drive has
already been explicitly placed into a valid interpolation mode and the SYNC
behavior has been verified on one axis. Do not use RPDO2 as the default manual
steering command path until that validation is complete.

## 7. SYNC usage

The configured profile uses synchronous transmission types:

```text
RPDO0 type 1
RPDO1 type 1
RPDO2 type 4
TPDO0 type 1
TPDO1 type 4
SYNC COB-ID 0x080
```

For analyzer bench tests:

1. Send RPDO command frames.
2. Send SYNC `0x080` with DLC 0.
3. Check TPDO feedback after SYNC.

For steering edge-trigger tests, keep the arm and trigger in separate logical
steps. If hardware behavior is uncertain, use separate SYNC cycles:

```text
arm frames for all axes -> SYNC
trigger frames for all axes -> SYNC
```

Only collapse arm and trigger into one SYNC strategy after analyzer evidence
proves the exact drive configuration behaves correctly.

## 8. Feedback decoding

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

## 9. Flash configuration verification record

Configuration command used:

```powershell
python tools\canopen_pdo_config\configure_all_nodes.py `
  --apply `
  --confirm-physical-bus-disconnected-from-ecu `
  --backend controlcan `
  --bus can1,can2 `
  --nodes 1-13 `
  --timeout-ms 900 `
  --save-profile dc `
  --ack-flash-write
```

Result:

```text
Node1-13: SDO write success
Node1-13: Flash save command acknowledged
Node1-13: power-cycle readback reachable
Active RPDO0/RPDO1/RPDO2 and TPDO0/TPDO1 values: persisted
Reserved RPDO3/TPDO2/TPDO3 mapping counts: persisted as 0
Reserved RPDO3/TPDO2/TPDO3 COB-ID disable bit: not persisted as bit31=1
```

Logs:

```text
out/canopen_pdo_config_v2_save_dc_20260706_170732/20260706_170732
out/canopen_pdo_config_v2_powercycle_verify_20260706_170926/20260706_170926
```

## 10. Safe commissioning checklist

Before sending motion commands:

1. Confirm correct bus and node ID.
2. Confirm the drive is in the expected CiA-402 state.
3. Confirm no active drive fault from TPDO1 or SDO diagnostics.
4. For steering, keep target within configured mechanical limits.
5. For drive velocity, command zero speed before disable.
6. For four-axis steering updates, send a coherent group; do not interleave
   different joystick samples in one group.
7. If any frame in a steering group fails or times out, stop drive velocity
   through the safety path and require a defined recovery before sending fresh
   steering groups.

