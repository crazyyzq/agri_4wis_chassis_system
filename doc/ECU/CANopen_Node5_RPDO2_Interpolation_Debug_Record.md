# CANopen Node5 RPDO2 Interpolation Debug Record

Date: 2026-07-06

Scope:

- Bus: ECU_CAN2 through CAN analyzer CAN1, 1 Mbit/s
- Node: 5, front-right steering
- Goal: verify whether RPDO2 can be used as a smooth real-time interpolation
  path before integrating it into ECU four-wheel steering
- Safety boundary: Node5 only, no NMT reset, no Flash save, no Node6-13 motion

## 1. Baseline result

RPDO1 profile-position control is the current reliable Node5 baseline.

Validated command path:

```text
RPDO1 COB-ID 0x305
6040:00 controlword + 6060:00 mode 1 + 607A:00 target position
0x002F -> SYNC -> 0x003F -> SYNC
```

Field result:

```text
0 -> +500000 -> -500000 -> 0
```

The axis reached the requested positions after the test script was changed to
wait for actual position feedback instead of sending the next target too early.

Conclusion:

- Node5 drive, CANopen control source, position mode, and mechanical movement
  are usable.
- The unsmooth RPDO2 behavior is not explained by a bad motor, missing power,
  wrong node ID, or inability to move ±500000 counts.

## 2. Initial RPDO2 configuration

Current persistent RPDO2 mapping:

| Object | Readback |
|---|---|
| `1402:01` | `0x00000405` |
| `1402:02` | `1` |
| `1602:00` | `1` |
| `1602:01` | `0x60C10120` |

Meaning:

```text
COB-ID 0x405
DLC 4
60C1:01 interpolated position value only
```

This mapping was restored after all temporary tests.

## 3. Tests performed

### 3.1 4-byte `60C1:01`, fixed time, preload then trigger

Configuration:

```text
6060 = 7
60C0 = 0
60C2:01 = 20
60C2:02 = -3
RPDO2 = 60C1:01 only
6040 = 0x000F before points
6040 = 0x003F after preload
```

Result:

- Axis did not follow the ±500000 count trajectory.
- Motion was visibly intermittent.
- EMCY appeared: `90 73 81 00 00 28 00 00`.
- Buffer-related readbacks repeatedly showed abnormal conditions such as
  `2011=0` and `2012=0x12000000` or `0x16000000`.

Conclusion: FAIL.

### 3.2 4-byte `60C1:01`, fixed time, trigger before points

Configuration change:

```text
6040 = 0x003F before streaming RPDO2 points
```

Result:

- This was better than preload-then-trigger.
- With no live feedback blocking the sender, the analyzer achieved a stable
  20 ms send period and the axis reached about `502509` counts.
- The axis still did not return to zero reliably.
- After waiting, the axis could remain around hundreds of thousands of counts
  or tens of thousands of counts instead of final zero.
- EMCY still appeared.

Conclusion: PARTIAL / FAIL. It proves RPDO2 points can affect the axis, but the
stream is not stable enough for ECU control.

Representative log:

```text
out/node5_rpdo2_debug_rpdo2-short_20260706_193606
```

### 3.3 `1402:02` transmission type comparison

Tested:

```text
1402:02 = 1
1402:02 = 4
```

Result:

- Type 4 did not make the trajectory stable.
- Type 1 remains the current restored value.

Conclusion: transmission type alone is not the root cause.

### 3.4 5-byte `60C1:01 + 60C1:02`, variable time

Temporary RAM mapping:

```text
1602:01 = 0x60C10120
1602:02 = 0x60C10208
1602:00 = 2
60C0 = -1
```

Result:

- Mapping was accepted.
- The axis still did not follow the full trajectory.
- EMCY still appeared.
- Mapping was restored after the test.

Conclusion: FAIL.

Important detail from the manual:

- In variable-time mode, writing `60C1:02` commits the record.
- This confirms that adding the time byte was a reasonable test, but it still
  did not produce stable motion on this setup.

### 3.5 8-byte `60C1:01 + 60C1:03`, fixed time

Temporary RAM mapping:

```text
1602:01 = 0x60C10120
1602:02 = 0x60C10320
1602:00 = 2
60C0 = 0
```

Result:

- Mapping was accepted.
- Motion remained unstable and did not track the command profile.
- EMCY still appeared.
- Mapping was restored after the test.

Conclusion: FAIL.

### 3.6 `0x2010` IP move segment command

Manual finding:

- The DCH CANopen manual recommends alternate objects `0x2010/0x2011/0x2012/0x2013`
  for efficient PVT and variable-time interpolation streaming.
- `0x2010` is an 8-byte IP move segment command.

Attempted PDO mappings:

```text
1602:01 = 0x20100040
1602:01 = 0x20100140
```

Result:

```text
SDO abort 0x06040041
```

Meaning:

- The drive rejected `0x2010` as an RPDO mapping target in the tested form.
- This conflicts with the manual/EDS indication that `2010` is PDO mappable.

Recovery:

- RPDO2 was restored to `60C1:01` mapping immediately after the abort.

Conclusion: `0x2010` is not yet usable through the current RPDO2 mapping method.
It may require a different PDO number, a drive-specific mapping rule, or vendor
confirmation.

## 4. Manual facts confirmed during review

From `DCH_CANopen.pdf`:

- `60C0 = 0`: linear interpolation with constant time.
- `60C0 = -1`: linear interpolation with variable time.
- `60C0 = -2`: cubic polynomial / PVT interpolation.
- For mode 0, writing `60C1:01` commits the record.
- For mode -1, writing `60C1:02` commits the record.
- For mode -2, writing `60C1:03` commits the record.
- Interpolation requires trajectory buffer management.
- `2012` bits:
  - bits 0-15: next expected segment ID
  - bits 16-23: free buffer slots
  - bit 24: sequence error
  - bit 25: overflow
  - bit 26: underflow
  - bit 31: buffer empty
- `0x2010` is the preferred efficient segment command object, but mapping it
  through RPDO2 was rejected by the tested drive.

## 5. Engineering conclusion

Node5 RPDO2 interpolation is not stable enough for ECU integration yet.

Do not connect SBUS or four-wheel steering to RPDO2 based on the current
results. The observed behavior is consistent with an interpolation buffer /
record format / drive-specific PDO mapping problem, not with a general CAN
transport failure.

Current safe control recommendation:

```text
Use RPDO1 absolute profile-position commands for steering commissioning.
Keep RPDO2 disabled from production control until a single-axis test has:
1. no EMCY,
2. final position error within tolerance,
3. buffer status without sequence/overflow/underflow,
4. repeatable smooth motion over several runs,
5. clear documented PDO mapping and segment format.
```

## 6. Open questions

1. Why does the drive reject `0x2010` PDO mapping with `0x06040041` although the
   manual/EDS describes it as PDO mappable?
2. Is `0x2010` only mappable to a specific RPDO number or only under a specific
   CANopen state?
3. Does the BC/BC2 firmware require a vendor object or parameter to enable IP
   segment-command PDO mapping?
4. What is the exact meaning of EMCY `90 73 81 00 00 28 00 00` for this drive?
5. Should the ECU avoid RPDO2 interpolation entirely and instead implement
   smoother profile-position target generation with RPDO1 until the vendor
   confirms the correct PVT PDO setup?

## 7. Relevant logs

```text
out/node5_rpdo2_debug_rpdo1-baseline_20260706_192957
out/node5_rpdo2_debug_rpdo2-short_20260706_193606
out/node5_rpdo2_debug_rpdo2-short_20260706_193711
out/node5_rpdo2_debug_rpdo2-short_20260706_193825
out/node5_rpdo2_debug_rpdo2-short_20260706_194346
out/node5_try_2010_sub1_mapping
out/node5_restore_rpdo2_after_2010_abort
```

