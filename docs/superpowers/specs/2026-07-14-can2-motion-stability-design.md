# CAN2 motion stability design

## Goal

Eliminate the field-observed steering/traction oscillation and long response
gaps without weakening stale-command, emergency-stop, high-voltage, or
feedback supervision.

## Confirmed root causes

1. The 5 ms remote task performs one attempt to read the 1 ms safety snapshot.
   A normal higher-priority safety publication can change the sequence during
   that copy. The read then fails and the caller converts that single scheduling
   race into an A-class fault and emergency stop, which drops remote arm until
   the operator returns to neutral.
2. During CAN2 node recovery, SDO writes return a node to CiA-402 Operation
   Enabled, but the ordinary recovery stop path repeatedly publishes a
   zero-speed, disabled (`0x0000`) drive group. The next TPDO therefore reports
   not Operation Enabled and immediately re-enters recovery. Analyzer evidence
   showed this cycle at approximately 500 ms intervals.

## Design

- Double-buffer remote/safety snapshots use a bounded four-attempt read. A
  sequence change retries immediately; no task blocks or waits. Timestamp stale
  limits remain authoritative.
- CAN2 recovery keeps all traction velocity/current targets at zero. When the
  latest coherent command still has valid high-voltage feedback, no disable
  request, and a non-safety command source, recovery is allowed to send a
  zero-speed Operation Enabled group (`0x000F`). It must never publish a nonzero
  target or replay the cached pre-fault traction command.
- Emergency stop, safety-source takeover, explicit disable, or loss of
  high-voltage feedback always selects zero-speed disabled output (`0x0000`).
- Node recovery continues to use NMT Operational, vendor fault clear and
  CiA-402 transitions. It never resets a CANopen node and never writes a
  steering actual-position zero.
- Fresh nonzero traction is accepted only after all eight nodes are healthy and
  one coherent four-steering-axis resynchronization group has completed.
- Lift trajectory constants are unchanged until a focused CAN3 capture proves
  setup re-entry, interpolation-buffer starvation, target reversal, or another
  specific firmware-side failure.

## Acceptance evidence

- Static tests prove bounded mailbox reads and recovery-zero enable/disable
  selection.
- CPU0 whole-vehicle build completes with zero errors and zero warnings.
- With P gear and steering input, Node1-8 recovery counters remain stable,
  recovery mask remains zero after startup, and four steering RPDO groups update
  at the configured 20 ms cadence.
- During low-speed traction, controlwords do not oscillate between `0x0000` and
  `0x000F` unless the operator or safety state actually changes.
- A later focused CAN3 capture must show whether lift RPDO2 remains four points
  plus one SYNC every 20 ms and whether any SDO/RPDO1 setup is repeated while
  motion is active.
