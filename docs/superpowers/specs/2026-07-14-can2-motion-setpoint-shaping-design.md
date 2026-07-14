# CAN2 coherent motion setpoint shaping design

## Goal

Remove low-speed traction chatter and make Ackermann steering faster and
smoother without changing the saved PDO mapping, CAN2 task ownership, safety
stop behavior, or the already validated automatic recovery path.

## Root causes in the current implementation

1. SBUS throttle noise immediately above the calibrated low endpoint is mapped
   to a nonzero speed. The current 1000-unit velocity deadband is only about
   0.006 motor rpm, so it does not reject commands below the mechanical useful
   speed of the installed wheel and gearbox.
2. Each drive axis is ramped independently from its last transmitted value.
   During acceleration, deceleration, and Ackermann target changes this can
   distort the coherent four-wheel velocity vector and make axes reach zero at
   different times.
3. Steering target-shaper constants mix position-count/s and the drive's
   0.1-count/s velocity-object units. The largest configured target rate is ten
   times the physical count/s value represented by the same numeric drive
   velocity setting.
4. Each Ackermann steering axis owns a separate trajectory. The four axes can
   therefore advance by different fractions of the same kinematic target.

## Design

### Pure shaper module

Add one allocation-free `motion_setpoint_shaper` module under `ecu/control`.
It contains only bounded integer calculations and has no CAN, FreeRTOS, SDO,
logging, or policy dependencies. `motion_device` remains the CAN2 owner and
calls the shaper only while constructing one coherent four-axis PDO group.

### Steering

- Shape the four Ackermann position targets as one vector. The largest absolute
  axis error selects one of four motion bands; every axis advances by the same
  fraction of its current error during that update.
- The largest band is limited to 3000 motor rpm. With Node5..8 at 10000
  count/rev this is 500000 position count/s. The corresponding drive object
  value for 0x6081 is 5000000 because that object uses 0.1 count/s.
- Limit acceleration to at most 500000 count/s^2 (50 motor rps^2). Lower error
  bands use lower speed and acceleration. Commissioning uses deliberately
  assertive fine/small/medium rates (720/1800/3000 motor rpm) so a new target
  does not remain trapped in an unnecessarily slow band.
- Publish one coherent Ackermann target group every 50 ms while the target is
  changing. This matches the previously smooth analyzer-driven position-mode
  test and gives the BC drive time to execute each profile segment instead of
  restarting its internal profile every 20 ms. The 5 ms vehicle task continues
  to keep only the latest joystick target, so no stale target queue is created.
- A final position hold band suppresses joystick noise and repeated profile
  position trigger edges. Reaching this band may snap only the small residual
  target error, never a large motion segment.
- Keep the existing fixed-posture spin/crab transition planner isolated. This
  change applies to continuous Ackermann following and does not alter homing.
- Field readback on 2026-07-14 found all four steering drives left at
  `0x6081=1200000`, which limited measured speed to about 720 rpm regardless of
  the ECU target slope. The CAN2 owner now writes and reads back volatile
  `0x6081/0x6083/0x6084` values before enabling realtime steering. It never
  writes PDO mapping or saves parameters to drive NVM.
- Acceleration remains capped at 500000 count/s^2 (50 motor rps^2), while the
  deceleration envelope and 0x6084 use 350000 count/s^2. This starts braking
  earlier and avoids the visible final stop kick. The final snap window is
  reduced to 750 counts.

### Drive

- Apply throttle start/stop hysteresis in the stateful SBUS mapper. Values below
  the start threshold remain zero; once active, the lower stop threshold is
  used so endpoint noise cannot repeatedly start and stop motion.
- Separate command-zero deadband, PDO-change threshold, and trajectory step
  constants. They are different physical concepts and must not share one macro.
- Ramp the complete four-wheel velocity vector with one maximum delta. All
  axes advance by the same fraction and preserve one coherent kinematic sample.
- A requested direction reversal first ramps the complete vector to zero, then
  begins the opposite vector. It does not jump directly across zero.
- Operator zero with valid high voltage keeps CiA-402 Operation Enabled and
  ramps to zero. Emergency stop, stale command, high-voltage loss, explicit
  disable, and other safety sources still bypass the ramp and command immediate
  disabled zero.

## Validation

- Source-contract tests cover module ownership, unit conversions, throttle
  hysteresis, coherent group shaping, reversal-through-zero, and removal of the
  old independent per-axis limiters.
- The full no-dependency Python test runner and CPU0 CMake/Ninja build must pass
  with zero warnings.
- Hardware validation starts with suspended wheels: P-gear steering sweep,
  low-speed D-gear throttle steps, throttle release, reversal, and combined
  Ackermann. CAN analyzer evidence must show coherent four-node groups without
  unsolicited zero groups or enable/disable oscillation.
