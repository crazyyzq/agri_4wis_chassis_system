# CAN2 coherent motion setpoint shaping plan

1. Add failing source-contract tests for a pure group shaper, 3000 rpm steering
   conversion, throttle hysteresis, separated drive thresholds, and retained
   immediate safety stop behavior.
2. Add `motion_setpoint_shaper.c/.h` under `ecu/control` and register it in the
   CPU0 CMake target.
3. Replace the independent Ackermann steering-axis target ramps in
   `motion_device.c` with one coherent four-axis shaper. Leave spin/crab fixed
   posture and homing paths unchanged.
4. Replace independent drive-axis ramps with one coherent velocity-vector
   shaper and explicit reversal-through-zero behavior.
5. Add stateful CH3 start/stop hysteresis in `remote_sbus_mapper` and keep all
   tunable thresholds in `ecu_config.h`.
6. Expose selected steering band and drive reversal/zero state through existing
   motion-device diagnostics without adding realtime logging.
7. Run the complete Python tests, `git diff --check`, and the canonical CPU0
   configure/build to regenerate the SEGGER project.
8. Do not claim 3000 rpm or smooth hardware response until analyzer/TPDO and
   physical motion evidence confirms it.
