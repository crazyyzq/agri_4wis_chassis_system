# CANopen PDO Mapping Record V1

This document is the frozen PDO contract for the agricultural 4WIS chassis ECU.
All BC / BC2 drives are configured by the offline CAN analyzer maintenance tool.
ECU firmware must use this contract and must not rewrite PDO mapping objects
during normal boot or runtime.

## Global mapping for every CANopen node N

| PDO | COB-ID | Mapping | DLC | Transmission type |
|---|---:|---|---:|---:|
| RPDO0 | `0x200 + N` | `6040:00` 16-bit + `6060:00` 8-bit + `60FF:00` 32-bit | 7 | `0x01` |
| RPDO1 | `0x300 + N` | `6040:00` 16-bit + `6060:00` 8-bit + `607A:00` 32-bit | 7 | `0x01` |
| RPDO2 | `0x400 + N` | `60C1:01` 32-bit interpolation point | 4 | `0x04` |
| RPDO3 | `0x500 + N` | `6040:00` 16-bit + `6060:00` 8-bit + `2340:00` 16-bit | 5 | `0x01` |
| TPDO0 | `0x180 + N` | `6064:00` 32-bit + `606C:00` 32-bit | 8 | `0x01` |
| TPDO1 | `0x280 + N` | `2183:00` 32-bit + `6041:00` 16-bit + `221C:00` 16-bit | 8 | `0x04` |

`6060:00` is intentionally included in RPDO0/RPDO1/RPDO3.  This costs one extra
byte in each speed/position command frame, but each realtime command is
self-describing and cannot be interpreted with a stale or incorrectly initialized
operating mode.

## Mapping object values

RPDO0 velocity:

```text
1400:01 = 0x200 + node
1400:02 = 0x01
1600:01 = 0x60400010
1600:02 = 0x60600008
1600:03 = 0x60FF0020
1600:00 = 3
```

RPDO1 position:

```text
1401:01 = 0x300 + node
1401:02 = 0x01
1601:01 = 0x60400010
1601:02 = 0x60600008
1601:03 = 0x607A0020
1601:00 = 3
```

RPDO2 interpolation:

```text
1402:01 = 0x400 + node
1402:02 = 0x04
1602:01 = 0x60C10120
1602:00 = 1
```

RPDO3 torque/current:

```text
1403:01 = 0x500 + node
1403:02 = 0x01
1603:01 = 0x60400010
1603:02 = 0x60600008
1603:03 = 0x23400010
1603:00 = 3
```

TPDO0:

```text
1800:01 = 0x180 + node
1800:02 = 0x01
1A00:01 = 0x60640020
1A00:02 = 0x606C0020
1A00:00 = 2
```

TPDO1:

```text
1801:01 = 0x280 + node
1801:02 = 0x04
1A01:01 = 0x21830020
1A01:02 = 0x60410010
1A01:03 = 0x221C0010
1A01:00 = 3
```

TPDO2 and TPDO3 are not part of the active feedback contract. The maintenance
tool clears `1A02:00` and `1A03:00` to zero entries so these PDOs carry no
payload and do not consume cyclic bus bandwidth. Some drives do not retain
the COB-ID disable bit for `1802:01` / `1803:01` after power cycling, so
firmware and verification must treat the zero mapping count as the active
contract for these unmanaged TPDOs.

## ECU CAN2 motion network

Leg order is fixed:

```text
Leg1 = FR
Leg2 = FL
Leg3 = RL
Leg4 = RR
```

| Bus | Node | Role | Normal command PDO | Torque/current PDO | TPDO0 | TPDO1 |
|---|---:|---|---|---|---:|---:|
| CAN2 | 1 | Leg1 / FR drive | `0x201` RPDO0, velocity mode `6060=0x03` | `0x501` RPDO3, current mode `6060=0x04` | `0x181` | `0x281` |
| CAN2 | 2 | Leg2 / FL drive | `0x202` RPDO0, velocity mode `6060=0x03` | `0x502` RPDO3 | `0x182` | `0x282` |
| CAN2 | 3 | Leg3 / RL drive | `0x203` RPDO0, velocity mode `6060=0x03` | `0x503` RPDO3 | `0x183` | `0x283` |
| CAN2 | 4 | Leg4 / RR drive | `0x204` RPDO0, velocity mode `6060=0x03` | `0x504` RPDO3 | `0x184` | `0x284` |
| CAN2 | 5 | Leg1 / FR steering | `0x305` RPDO1, position mode `6060=0x01` | `0x505` RPDO3 | `0x185` | `0x285` |
| CAN2 | 6 | Leg2 / FL steering | `0x306` RPDO1, position mode `6060=0x01` | `0x506` RPDO3 | `0x186` | `0x286` |
| CAN2 | 7 | Leg3 / RL steering | `0x307` RPDO1, position mode `6060=0x01` | `0x507` RPDO3 | `0x187` | `0x287` |
| CAN2 | 8 | Leg4 / RR steering | `0x308` RPDO1, position mode `6060=0x01` | `0x508` RPDO3 | `0x188` | `0x288` |

## ECU CAN3 lift and hydraulic network

Physical leg order remains FR, FL, RL, RR. BC2 node IDs are:

```text
FR lift = 9
FL lift = 11
RL lift = 12
RR lift = 10
hydraulic pump = 13
```

| Bus | Node | Role | Normal command PDO | Torque/current PDO | TPDO0 | TPDO1 |
|---|---:|---|---|---|---:|---:|
| CAN3 | 9 | Leg1 / FR lift | `0x309` RPDO1, position mode `6060=0x01` | `0x509` RPDO3 | `0x189` | `0x289` |
| CAN3 | 11 | Leg2 / FL lift | `0x30B` RPDO1, position mode `6060=0x01` | `0x50B` RPDO3 | `0x18B` | `0x28B` |
| CAN3 | 12 | Leg3 / RL lift | `0x30C` RPDO1, position mode `6060=0x01` | `0x50C` RPDO3 | `0x18C` | `0x28C` |
| CAN3 | 10 | Leg4 / RR lift | `0x30A` RPDO1, position mode `6060=0x01` | `0x50A` RPDO3 | `0x18A` | `0x28A` |
| CAN3 | 13 | hydraulic pump | `0x20D` RPDO0, velocity mode `6060=0x03` | `0x50D` RPDO3 | `0x18D` | `0x28D` |

## Wire formats

Velocity RPDO0:

```text
ID  = 0x200 + node
DLC = 7
byte0..1 = 6040 controlword, little-endian
byte2     = 6060 mode, 0x03 profile velocity
byte3..6 = 60FF target velocity, signed int32 little-endian
```

Position RPDO1:

```text
ID  = 0x300 + node
DLC = 7
byte0..1 = 6040 controlword, little-endian
byte2     = 6060 mode, 0x01 profile position
byte3..6 = 607A target position, signed int32 little-endian
```

Torque/current RPDO3:

```text
ID  = 0x500 + node
DLC = 5
byte0..1 = 6040 controlword, little-endian
byte2     = 6060 mode, 0x04 current mode
byte3..4 = 2340 command current, signed int16 little-endian, unit 10 mA
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
`0x1402`, `0x1602`, `0x1403`, `0x1603`, `0x1800`, `0x1A00`, `0x1801`, or
`0x1A01` during normal boot or runtime.

## Brake ownership

Brake-release polarity on the installed servo drives is active high, but the
brake output owner is the servo drive internal brake controller. ECU firmware
does not drive brake release through local DIO and does not write the drive
program-output object `0x2194` / OUT bits during normal runtime.

`vehicle_actuator_command_t.brake_release` remains only a high-level permission
to request a motion-capable CiA-402 state. It is not a brake wire level and it
is not `brake_release_confirmed`. Without an independent brake feedback input,
`brake_release_confirmed` must remain unavailable/false.
