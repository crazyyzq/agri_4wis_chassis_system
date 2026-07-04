# CANopen PDO Mapping Record V1

This document is the frozen PDO contract for the agricultural 4WIS chassis ECU.
All BC / BC2 drives have already been configured and saved by the offline CAN
analyzer maintenance tool. ECU firmware must use this contract; it must not
rewrite PDO mapping objects during normal boot or runtime.

## Global mapping for every CANopen node N

| PDO | COB-ID | Mapping | DLC | Transmission type |
|---|---:|---|---:|---:|
| RPDO0 | `0x200 + N` | `6040:00` 16-bit + `6060:00` 8-bit + `60FF:00` 32-bit | 7 | `0xFF` |
| RPDO1 | `0x300 + N` | `6040:00` 16-bit + `6060:00` 8-bit + `607A:00` 32-bit | 7 | `0xFF` |
| TPDO0 | `0x180 + N` | `6064:00` 32-bit + `606C:00` 32-bit | 8 | `0x01` |
| TPDO1 | `0x280 + N` | `2183:00` 32-bit + `6041:00` 16-bit + `221C:00` 16-bit | 8 | `0x01` |

## Mapping object values

RPDO0:

```text
1600:01 = 0x60400010
1600:02 = 0x60600008
1600:03 = 0x60FF0020
1600:00 = 3
```

RPDO1:

```text
1601:01 = 0x60400010
1601:02 = 0x60600008
1601:03 = 0x607A0020
1601:00 = 3
```

TPDO0:

```text
1A00:01 = 0x60640020
1A00:02 = 0x606C0020
1A00:00 = 2
```

TPDO1:

```text
1A01:01 = 0x21830020
1A01:02 = 0x60410010
1A01:03 = 0x221C0010
1A01:00 = 3
```

## ECU CAN2 motion network

Leg order is fixed:

```text
Leg1 = FR
Leg2 = FL
Leg3 = RL
Leg4 = RR
```

| Bus | Node | Role | Normal command PDO | TPDO0 | TPDO1 |
|---|---:|---|---|---:|---:|
| CAN2 | 1 | Leg1 / FR drive | `0x201` RPDO0, velocity mode `6060=0x03` | `0x181` | `0x281` |
| CAN2 | 2 | Leg2 / FL drive | `0x202` RPDO0, velocity mode `6060=0x03` | `0x182` | `0x282` |
| CAN2 | 3 | Leg3 / RL drive | `0x203` RPDO0, velocity mode `6060=0x03` | `0x183` | `0x283` |
| CAN2 | 4 | Leg4 / RR drive | `0x204` RPDO0, velocity mode `6060=0x03` | `0x184` | `0x284` |
| CAN2 | 5 | Leg1 / FR steering | `0x305` RPDO1, position mode `6060=0x01` | `0x185` | `0x285` |
| CAN2 | 6 | Leg2 / FL steering | `0x306` RPDO1, position mode `6060=0x01` | `0x186` | `0x286` |
| CAN2 | 7 | Leg3 / RL steering | `0x307` RPDO1, position mode `6060=0x01` | `0x187` | `0x287` |
| CAN2 | 8 | Leg4 / RR steering | `0x308` RPDO1, position mode `6060=0x01` | `0x188` | `0x288` |

## ECU CAN3 lift and hydraulic network

Physical leg order remains FR, FL, RL, RR. BC2 node IDs are:

```text
FR lift = 9
FL lift = 11
RL lift = 12
RR lift = 10
hydraulic pump = 13
```

| Bus | Node | Role | Normal command PDO | TPDO0 | TPDO1 |
|---|---:|---|---|---:|---:|
| CAN3 | 9 | Leg1 / FR lift | `0x309` RPDO1, position mode `6060=0x01` | `0x189` | `0x289` |
| CAN3 | 11 | Leg2 / FL lift | `0x30B` RPDO1, position mode `6060=0x01` | `0x18B` | `0x28B` |
| CAN3 | 12 | Leg3 / RL lift | `0x30C` RPDO1, position mode `6060=0x01` | `0x18C` | `0x28C` |
| CAN3 | 10 | Leg4 / RR lift | `0x30A` RPDO1, position mode `6060=0x01` | `0x18A` | `0x28A` |
| CAN3 | 13 | hydraulic pump | `0x20D` RPDO0, velocity mode `6060=0x03` | `0x18D` | `0x28D` |

## Wire formats

Velocity RPDO0:

```text
ID  = 0x200 + node
DLC = 7
byte0..1 = 6040 controlword, little-endian
byte2     = 0x03
byte3..6 = 60FF target velocity, signed int32 little-endian
```

Position RPDO1:

```text
ID  = 0x300 + node
DLC = 7
byte0..1 = 6040 controlword, little-endian
byte2     = 0x01
byte3..6 = 607A target position, signed int32 little-endian
```

TPDO0:

```text
ID  = 0x180 + node
DLC = 8
byte0..3 = 6064 actual position, signed int32 little-endian
byte4..7 = 606C actual velocity, signed int32 little-endian
```

TPDO1:

```text
ID  = 0x280 + node
DLC = 8
byte0..3 = 2183 latched fault, uint32 little-endian
byte4..5 = 6041 statusword, uint16 little-endian
byte6..7 = 221C actual current, signed int16 little-endian
```

## Firmware rule

CPU0 production firmware does not configure, verify, or save PDO mapping. It
must not write the mapping objects `0x1400`, `0x1600`, `0x1401`, `0x1601`,
`0x1800`, `0x1A00`, `0x1801`, or `0x1A01` during normal boot or runtime.

## Brake ownership

Brake-release polarity on the installed servo drives is active high, but the
brake output owner is the servo drive internal brake controller. ECU firmware
does not drive brake release through local DIO and does not write the drive
program-output object `0x2194` / OUT bits during normal runtime.

`vehicle_actuator_command_t.brake_release` remains only a high-level permission
to request a motion-capable CiA-402 state. It is not a brake wire level and it
is not `brake_release_confirmed`. Without an independent brake feedback input,
`brake_release_confirmed` must remain unavailable/false.

The normal runtime contract is therefore limited to the frozen RPDO/TPDO table
above plus read-only diagnosis. Mapping writes, flash save `0x1010`, and
program-output writes such as `0x2194` remain denied by default.
