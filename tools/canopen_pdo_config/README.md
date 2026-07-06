# CANopen PDO Configurator

独立工具，用 USB-CAN 分析仪配置 BC/BC2 驱动器 Node1~13 的 V2 统一 PDO 映射。

## Current vehicle status

当前整车 Node1~13 的 PDO 配置必须以项目最新统一 PDO 规范为准。该工具仅用于离线维护、
替换新驱动器、恢复出厂或明确人工批准的重配。禁止在 ECU 接入总线、
车辆可运动或普通调试时运行。

## Physical wiring

- Analyzer CAN1 -> ECU physical CAN2 -> Node 1..8
- Analyzer CAN2 -> ECU physical CAN3 -> Node 9..13
- ECU must stay powered off or physically disconnected from these CAN buses
- CAN bitrate: 1 Mbit/s, standard 11-bit CAN ID

## V2 PDO profile

All Node1..13 use the same active PDO layout. Only node ID, bus, COB-ID and
runtime role differ.

| PDO | COB-ID | Mapping | DLC | Transmission type |
|---|---:|---|---:|---:|
| RPDO0 velocity | `0x200 + node` | `6040:00` + `6060:00` + `60FF:00` | 7 | 1 |
| RPDO1 position | `0x300 + node` | `6040:00` + `6060:00` + `607A:00` | 7 | 1 |
| RPDO2 interpolation | `0x400 + node` | `60C1:01` | 4 | 4 |
| TPDO0 motion feedback | `0x180 + node` | `6064:00` + `606C:00` | 8 | 1 |
| TPDO1 health feedback | `0x280 + node` | `2183:00` + `6041:00` + `221C:00` | 8 | 4 |

Reserved PDOs are kept uniformly disabled:

- RPDO3: `1403` / `1603`
- TPDO2: `1802` / `1A02`
- TPDO3: `1803` / `1A03`

## Safe default

Default mode is dry-run. It writes only output files and does not open CAN:

```powershell
python tools\canopen_pdo_config\configure_all_nodes.py --dry-run
```

Real SDO writes require both:

```powershell
--apply
--confirm-physical-bus-disconnected-from-ecu
```

Flash save is never sent by default. It additionally requires:

```powershell
--save-profile dc --ack-flash-write
```

or:

```powershell
--save-profile de_de2 --ack-flash-write
```

BC/BC2 drives use the DC save sequence according to current project field notes.

## Examples

Read the generated plan only:

```powershell
python tools\canopen_pdo_config\configure_all_nodes.py --dry-run
```

Configure Node5 through analyzer CAN1, RAM only:

```powershell
python tools\canopen_pdo_config\configure_all_nodes.py `
  --apply --confirm-physical-bus-disconnected-from-ecu `
  --backend controlcan --bus can1 --nodes 5
```

Read only Node5 backup through analyzer CAN1:

```powershell
python tools\canopen_pdo_config\configure_all_nodes.py `
  --read-only --backend controlcan --bus can1 --nodes 5
```

Configure all nodes, RAM only:

```powershell
python tools\canopen_pdo_config\configure_all_nodes.py `
  --apply --confirm-physical-bus-disconnected-from-ecu `
  --backend controlcan --bus can1,can2 --nodes 1-13
```

Configure and save as DC profile:

```powershell
python tools\canopen_pdo_config\configure_all_nodes.py `
  --apply --confirm-physical-bus-disconnected-from-ecu `
  --backend controlcan --bus can1,can2 --nodes 1-13 `
  --save-profile dc --ack-flash-write
```

## Backend parameters

- `--backend controlcan`: uses `tools/can/controlcan.py` and the vendor `ControlCAN.dll`
- `--backend mock`: no hardware, used for tests
- `--channel-can1 0`: analyzer CAN1 channel index
- `--channel-can2 1`: analyzer CAN2 channel index
- `--bitrate 1000000`
- `--timeout-ms 300`
- `--log-dir out/canopen_pdo_config`

## Output

Each run writes:

```text
out/canopen_pdo_config/<timestamp>/
  plan.json
  transaction_log.jsonl
  node_01_before.json
  node_01_after.json
  summary.json
  summary.md
```

## What this tool never does

- It does not send motion RPDO frames on `0x200+node`, `0x300+node`, or `0x400+node`
- It does not send broadcast NMT Operational
- It does not send broadcast reset
- It does not enable motors or prove that motors can move
- It does not mark Flash persistence as power-cycle verified

After a Flash save, the reported state is only `MAPPED_SAVED_UNPOWER_CYCLE_UNVERIFIED`.
Power-cycle verification requires rebooting drives and reading the PDO objects again.
