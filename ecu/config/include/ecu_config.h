/* Central calibration and timing configuration for the ECU main application. */
#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_types.h"

#define REMOTE_DISCRETE_DEBOUNCE_MS       (80U)
#define REMOTE_LINK_QUALIFY_MS          (1000U)
#define REMOTE_NEUTRAL_QUALIFY_MS        (300U)
#define REMOTE_FAILSAFE_TIMEOUT_MS       (100U)
#define REMOTE_DOMAIN_EVENT_GUARD_MS     (150U)
#define REMOTE_POWER_LONG_PRESS_MS       (350U)
#define REMOTE_ESTOP_CENTER_HOLD_MS     (1000U)
#define REMOTE_EVENT_MODE_REQUEST_TTL_MS (250U)
#define REMOTE_EVENT_POWER_REQUEST_TTL_MS (500U)
#define REMOTE_EVENT_ESTOP_RESET_TTL_MS (1000U)
#define REMOTE_EVENT_LIGHT_REQUEST_TTL_MS (1000U)

/* SBUS channel values are decoded from the receiver as protocol-native 11-bit
 * raw values.  The CPU0 remote task converts those raw values to PPM-equivalent
 * servo values before evaluating switches, throttle, steering and safety
 * states.
 *
 * Measured commissioning points on the installed receiver:
 *   raw low ~= 282, raw center ~= 1002, raw high ~= 1722.
 *
 * Remote state machines use the PPM-equivalent range because it matches the
 * transmitter manual and is easier to debug during whole-machine testing. */
#define ECU_SBUS_PROTOCOL_RAW_LOW        (282U)
#define ECU_SBUS_PROTOCOL_RAW_CENTER     (1002U)
#define ECU_SBUS_PROTOCOL_RAW_HIGH       (1722U)
#define ECU_SBUS_PPM_LOW                 (1050U)
#define ECU_SBUS_PPM_CENTER              (1500U)
#define ECU_SBUS_PPM_HIGH                (1950U)
#define ECU_SBUS_PPM_LOW_MAX             (1200U)
#define ECU_SBUS_PPM_CENTER_MIN          (1400U)
#define ECU_SBUS_PPM_CENTER_MAX          (1600U)
#define ECU_SBUS_PPM_HIGH_MIN            (1800U)
#define ECU_SBUS_PPM_THROTTLE_START      (1100U)
#define ECU_SBUS_PPM_CREDIBLE_MIN        (1000U)
#define ECU_SBUS_PPM_CREDIBLE_MAX        (2000U)
#define ECU_SBUS_DECODE_ERROR_LIMIT      (10U)

#define ECU_CPU0_SAFETY_PERIOD_MS        (1U)
#define ECU_CPU0_CAN2_MOTION_PERIOD_MS   (2U)
#define ECU_CPU0_REMOTE_PERIOD_MS        (5U)
#define ECU_CPU0_CONTROL_PERIOD_MS       (5U)
#define ECU_CPU0_POWER_PERIOD_MS         (10U)
#define ECU_CPU0_LIFT_HYD_PERIOD_MS      (10U)
#define ECU_CPU0_IO_PERIOD_MS            (10U)
#define ECU_CPU0_DIAG_PERIOD_MS          (100U)
#define ECU_CPU1_SERVICE_PERIOD_MS       (20U)
/* CAN3 owns hydraulic valves.  If the vehicle task stops publishing a fresh
 * coherent command, the CAN3 task must fail closed instead of leaving the last
 * nonzero valve output latched.
 */
#define ECU_CAN3_COMMAND_STALE_TIMEOUT_MS (50U)
#define ECU_CAN2_COMMAND_STALE_TIMEOUT_MS (50U)

#ifndef ECU_ENABLE_DEBUG_MONITOR
#define ECU_ENABLE_DEBUG_MONITOR         (1)
#endif

#ifndef ECU_BUILD_PROFILE_TEXT
#define ECU_BUILD_PROFILE_TEXT           "safe"
#endif

#ifndef ECU_BUILD_PROFILE_STEER4_REMOTE
#define ECU_BUILD_PROFILE_STEER4_REMOTE  (0)
#endif

#ifndef ECU_BUILD_PROFILE_STEER4_REMOTE_90
#define ECU_BUILD_PROFILE_STEER4_REMOTE_90 (0)
#endif

#ifndef ECU_BUILD_PROFILE_WHOLE_VEHICLE_MOTION
#define ECU_BUILD_PROFILE_WHOLE_VEHICLE_MOTION (0)
#endif

#ifndef ECU_DEBUG_MONITOR_PERIOD_MS
#define ECU_DEBUG_MONITOR_PERIOD_MS      (500U)
#endif

#ifndef ECU_DEBUG_MONITOR_VERBOSE
#define ECU_DEBUG_MONITOR_VERBOSE        (0)
#endif

/* Commissioning-only power debug.
 *
 * This J-Link/Watch controlled hook is for whole-machine communication
 * checkout when the remote controller is not present.  It may request BMS high
 * voltage only; it must not release brakes, enable hydraulics, or send motion
 * targets.  Keep the hook compiled in but inactive until the magic value and
 * high_voltage_enable flag are written through the debugger.
 */
#ifndef ECU_ENABLE_COMMISSIONING_POWER_DEBUG
#define ECU_ENABLE_COMMISSIONING_POWER_DEBUG (1)
#endif
#define ECU_COMMISSIONING_CONTROL_MAGIC      (0xEC0C0DEUL)
#define ECU_COMMISSIONING_HV_REQUEST_TIMEOUT_MS (600000U)

/* Read-only SDO scanner for whole-machine CANopen communication checkout.
 * It queues one 0x6041 statusword upload at a time across all configured servo
 * nodes.  This provides per-node traffic evidence without enabling operation.
 */
#ifndef ECU_ENABLE_COMMISSIONING_CANOPEN_SCAN
#define ECU_ENABLE_COMMISSIONING_CANOPEN_SCAN (1)
#endif
#define ECU_CANOPEN_COMMISSIONING_SCAN_PERIOD_MS (50U)

#ifndef ECU_ENABLE_CANOPENNODE
#define ECU_ENABLE_CANOPENNODE           (0)
#endif

/* Physical bus defaults.  CAN1 is reserved for power devices, CAN2 for drive
 * and steering servos, and CAN3 for lift/hydraulic servos.  These values are
 * deliberately centralized so field changes do not require touching drivers. */
#define ECU_CAN1_POWER_BITRATE           (250000UL)
#define ECU_CAN2_MOTION_BITRATE          (1000000UL)
#define ECU_CAN3_LIFT_HYDRAULIC_BITRATE  (1000000UL)
#define ECU_CAN4_AUXILIARY_BITRATE       (500000UL)
#define ECU_CAN1_TERMINATION_ENABLE      (1)
#define ECU_CAN2_TERMINATION_ENABLE      (1)
#define ECU_CAN3_TERMINATION_ENABLE      (1)
#define ECU_CAN4_TERMINATION_ENABLE      (1)

/* Bench-only CAN4 physical-layer test.  Keep disabled for whole-vehicle runs so
 * an unconnected CAN4 transceiver does not create continuous transmit errors.
 * Define ECU_ENABLE_CAN4_PHYSICAL_TEST_TX=1 only when CAN4 is under test.  The
 * payload is intentionally not a CANopen, BMS or actuator command.
 */
#ifndef ECU_ENABLE_CAN4_PHYSICAL_TEST_TX
#define ECU_ENABLE_CAN4_PHYSICAL_TEST_TX (0)
#endif
#define ECU_CAN4_PHYSICAL_TEST_TX_PERIOD_MS (500U)
#define ECU_CAN4_PHYSICAL_TEST_FRAME_ID     (0x444UL)

#ifndef ECU_ENABLE_CAN3_LIFT_CANOPEN
#define ECU_ENABLE_CAN3_LIFT_CANOPEN     (1)
#endif

#ifndef ECU_POWER_CAN_TX_ENABLE
#define ECU_POWER_CAN_TX_ENABLE              (1)
#endif
#define ECU_POWER_ENABLE_BMS                 (1)
#define ECU_POWER_ENABLE_DCDC48              (1)
#define ECU_POWER_ENABLE_DCDC12              (1)
#define ECU_POWER_ENABLE_DCAC                (1)
#define ECU_POWER_BMS_COMMAND_PERIOD_MS      (20U)
#define ECU_POWER_DCDC48_COMMAND_PERIOD_MS   (100U)
#define ECU_POWER_DCDC12_COMMAND_PERIOD_MS   (200U)
#define ECU_POWER_DCAC_COMMAND_PERIOD_MS     (500U)
#define ECU_POWER_BMS_STATUS_TIMEOUT_MS      (250U)
#define ECU_POWER_DCDC48_STATUS_TIMEOUT_MS   (500U)
#define ECU_POWER_DCDC12_STATUS_TIMEOUT_MS   (1500U)
#define ECU_POWER_DCAC_STATUS_TIMEOUT_MS     (1500U)
#define ECU_DCDC48_DEFAULT_TERMINAL_VOLTAGE_DV (140U)
#define ECU_DCDC48_DEFAULT_CURRENT_DA        (200U)
#define ECU_DCDC12_DEFAULT_OUTPUT_VOLTAGE_DV (138U)
#define ECU_DCDC12_DEFAULT_OUTPUT_CURRENT_DA (1000U)
#define ECU_DCAC_DEFAULT_OUTPUT_VOLTAGE_DV   (2200U)

/* Vehicle CANopen node contract.
 *
 * Vehicle direction is defined with the front of the vehicle as positive X.
 * All four actuator arrays use vehicle leg order, not the common FL/FR/RL/RR
 * order:
 *   - Leg 1: front-right drive, steering and lift motors.
 *   - Leg 2: front-left drive, steering and lift motors.
 *   - Leg 3: rear-left drive, steering and lift motors.
 *   - Leg 4: rear-right drive, steering and lift motors.
 *
 * Keep this contract explicit.  Field wiring, CANopen node IDs, steering signs,
 * lift signs and diagnostic print order all become ambiguous if the code silently
 * switches to another wheel order.
 *
 * CAN2:
 *   - Drive motors use nodes 1..4 in leg order.
 *   - Steering motors use nodes 5..8 in leg order.
 *
 * CAN3:
 *   - Lift motors use nodes 9, 11, 12 and 10 in vehicle leg order.
 *   - The hydraulic station motor uses node 13.
 *
 * BC2 dual-axis drives expose A and B axes as separate CANopen nodes.  The SW
 * BCD switch sets the A-axis node ID and the B-axis node ID is A + 1. */
#define ECU_WHEEL_LEG1_FRONT_RIGHT (0U)
#define ECU_WHEEL_LEG2_FRONT_LEFT  (1U)
#define ECU_WHEEL_LEG3_REAR_LEFT   (2U)
#define ECU_WHEEL_LEG4_REAR_RIGHT  (3U)

#define ECU_CANOPEN_LEG1_DRIVE_NODE_ID (0x01U)
#define ECU_CANOPEN_LEG2_DRIVE_NODE_ID (0x02U)
#define ECU_CANOPEN_LEG3_DRIVE_NODE_ID (0x03U)
#define ECU_CANOPEN_LEG4_DRIVE_NODE_ID (0x04U)
#define ECU_CANOPEN_LEG1_STEER_NODE_ID (0x05U)
#define ECU_CANOPEN_LEG2_STEER_NODE_ID (0x06U)
#define ECU_CANOPEN_LEG3_STEER_NODE_ID (0x07U)
#define ECU_CANOPEN_LEG4_STEER_NODE_ID (0x08U)
#define ECU_CANOPEN_LIFT_LEG1_NODE_ID  (0x09U)
#define ECU_CANOPEN_LIFT_LEG2_NODE_ID  (0x0BU)
#define ECU_CANOPEN_LIFT_LEG3_NODE_ID  (0x0CU)
#define ECU_CANOPEN_LIFT_LEG4_NODE_ID  (0x0AU)
#define ECU_CANOPEN_HYDRAULIC_PUMP_NODE_ID (0x0DU)

/* Semantic aliases used by control code and diagnostics.  The aliases keep
 * position-oriented code readable while the primary storage order remains
 * explicit vehicle leg order: FR, FL, RL, RR. */
#define ECU_CANOPEN_DRIVE_FR_NODE_ID   ECU_CANOPEN_LEG1_DRIVE_NODE_ID
#define ECU_CANOPEN_DRIVE_FL_NODE_ID   ECU_CANOPEN_LEG2_DRIVE_NODE_ID
#define ECU_CANOPEN_DRIVE_RL_NODE_ID   ECU_CANOPEN_LEG3_DRIVE_NODE_ID
#define ECU_CANOPEN_DRIVE_RR_NODE_ID   ECU_CANOPEN_LEG4_DRIVE_NODE_ID
#define ECU_CANOPEN_STEER_FR_NODE_ID   ECU_CANOPEN_LEG1_STEER_NODE_ID
#define ECU_CANOPEN_STEER_FL_NODE_ID   ECU_CANOPEN_LEG2_STEER_NODE_ID
#define ECU_CANOPEN_STEER_RL_NODE_ID   ECU_CANOPEN_LEG3_STEER_NODE_ID
#define ECU_CANOPEN_STEER_RR_NODE_ID   ECU_CANOPEN_LEG4_STEER_NODE_ID
#define ECU_CANOPEN_LIFT_FR_NODE_ID    ECU_CANOPEN_LIFT_LEG1_NODE_ID
#define ECU_CANOPEN_LIFT_FL_NODE_ID    ECU_CANOPEN_LIFT_LEG2_NODE_ID
#define ECU_CANOPEN_LIFT_RL_NODE_ID    ECU_CANOPEN_LIFT_LEG3_NODE_ID
#define ECU_CANOPEN_LIFT_RR_NODE_ID    ECU_CANOPEN_LIFT_LEG4_NODE_ID

/* Drive motor polarity in vehicle leg order.
 *
 * Positive vehicle-frame wheel speed means the chassis moves toward the
 * physical front of the machine when all wheels are straight.  Field wiring
 * confirms that this requires motor nodes 1 and 4 to receive positive 0x60FF
 * values, while nodes 2 and 3 must receive negative values.
 */
#define ECU_CANOPEN_LEG1_DRIVE_DIRECTION_SIGN (1)
#define ECU_CANOPEN_LEG2_DRIVE_DIRECTION_SIGN (-1)
#define ECU_CANOPEN_LEG3_DRIVE_DIRECTION_SIGN (-1)
#define ECU_CANOPEN_LEG4_DRIVE_DIRECTION_SIGN (1)

/* Default CiA-301 COB-ID bases.  Device adapters build TPDO/RPDO/heartbeat
 * identifiers from these bases plus the node ID unless a device-specific
 * configuration overrides them. */
#define ECU_CANOPEN_TPDO1_BASE     (0x180UL)
#define ECU_CANOPEN_TPDO2_BASE     (0x280UL)
#define ECU_CANOPEN_RPDO1_BASE     (0x200UL)
#define ECU_CANOPEN_RPDO2_BASE     (0x300UL)
#define ECU_CANOPEN_RPDO3_BASE     (0x400UL)
#define ECU_CANOPEN_RPDO4_BASE     (0x500UL)
#define ECU_CANOPEN_RPDO5_BASE     (0x80000000UL)
#define ECU_CANOPEN_HEARTBEAT_BASE (0x700UL)
#define ECU_CANOPEN_TIMEOUT_MS     (100U)
#define ECU_CANOPEN_COB_ID_DISABLED      (0x80000000UL)

#define ECU_CANOPEN_MASTER_NODE_ID       (0x7FU)
#define ECU_CANOPEN_BC2_DIAG_NODE_ID     (ECU_CANOPEN_DRIVE_FR_NODE_ID)
#define ECU_CANOPEN_SDO_TIMEOUT_MS       (100U)
#define ECU_CANOPEN_SDO_PERIOD_MS        (100U)
/* Device adapters cache the last successfully queued actuator command to avoid
 * flooding the CANopen SDO queue every scheduler tick.  Queue success does not
 * prove the remote drive accepted the SDO later, so unchanged commands are
 * refreshed periodically.  This keeps the command path self-healing after a
 * transient full queue, timeout, reset or abort without generating 5 ms SDO
 * bursts during steady state.
 */
#define ECU_CANOPEN_MOTION_COMMAND_REFRESH_MS (500U)
#define ECU_CANOPEN_LIFT_COMMAND_REFRESH_MS   (500U)
/* Joystick commands are calculated faster than SDO downloads can be completed.
 * Device adapters therefore push only meaningful target changes and cap the
 * fastest target-update cadence.  Safety stop/brake commands bypass this limit.
 */
#define ECU_CANOPEN_MOTION_TARGET_MIN_INTERVAL_MS (50U)
#define ECU_CANOPEN_STEER_LIMIT_INPUT_GATING_ENABLED (0)
#define ECU_CANOPEN_DRIVE_VELOCITY_DEADBAND_UNITS (1000)
#define ECU_CANOPEN_ZERO_SPEED_RPM_TOLERANCE      (3.0f)
#define ECU_CAN2_ZERO_SPEED_VELOCITY_UNITS \
    ((int32_t)((ECU_CANOPEN_ZERO_SPEED_RPM_TOLERANCE * \
                ECU_CAN2_MOTION_VELOCITY_UNITS_PER_RPM) + 0.5f))
#define ECU_HYDRAULIC_PUMP_ZERO_SPEED_VELOCITY_UNITS \
    ((int32_t)((ECU_CANOPEN_ZERO_SPEED_RPM_TOLERANCE * \
                ECU_HYDRAULIC_PUMP_VELOCITY_UNITS_PER_RPM) + 0.5f))
#define ECU_CANOPEN_DRIVE_PDO_PERIOD_MS           (20U)
/* Keep velocity-mode RPDO alive even when the requested speed is unchanged.
 * Several BC/BC2 commissioning tests showed that a steady nonzero velocity
 * command must be refreshed; otherwise the drive side may stop while ECU-side
 * command state still looks valid.  This is a PDO refresh only, not an SDO
 * setup retry.
 */
#define ECU_CANOPEN_DRIVE_VELOCITY_REFRESH_MS     (100U)
#define ECU_CANOPEN_LIFT_INTERPOLATION_PERIOD_MS  (20U)
#define ECU_CANOPEN_LIFT_INTERPOLATION_REFRESH_MS (20U)
#define ECU_CANOPEN_LIFT_FEEDBACK_TIMEOUT_MS      (200U)
#define ECU_CANOPEN_LIFT_SETUP_TIMEOUT_MS         (5000U)
#define ECU_CANOPEN_LIFT_PRELOAD_POINTS           (3U)
#define ECU_CANOPEN_LIFT_INTERPOLATION_MODE       (0)
#define ECU_CANOPEN_LIFT_ENABLE_SETTLE_MS         (1500U)
#define ECU_CANOPEN_PUMP_VELOCITY_REFRESH_MS      (1000U)
/* A single non-trigger PDO TX fault must not permanently freeze the vehicle.
 * The CAN2 realtime owner cancels the local group, discards stale queued
 * frames, and retries from the newest vehicle command.  Trigger-phase steering
 * failures remain safety-critical and latch immediately because one or more
 * steering axes may already have accepted a target edge.
 */
#define ECU_CANOPEN_REALTIME_TRANSIENT_FAILURE_LIMIT (3U)
#define ECU_CANOPEN_TRANSPORT_RECOVERY_BACKOFF_MS     (50U)
#define ECU_CANOPEN_NODE_RECOVERY_RETRY_MS            (500U)
#define ECU_CANOPEN_NODE_RECOVERY_COOLDOWN_MS         (5000U)
#define ECU_CANOPEN_NODE_RECOVERY_FAST_ATTEMPTS       (3U)
#define ECU_CANOPEN_MOTION_FEEDBACK_STALE_RECOVERY_MS (500U)
#define ECU_CANOPEN_PARTIAL_GROUP_RECOVERY_STABLE_MS  (200U)
#define ECU_CANOPEN_HEARTBEAT_TIMEOUT_MS              (1500U)
#define ECU_CANOPEN_TPDO1_STATUS_TIMEOUT_MS           (750U)

/*
 * Whole-vehicle commissioning startup gate:
 *
 * The BC/BC2 drives have their PDO mapping configured and saved by the CAN
 * analyzer before ECU control.  Some field configurations do not produce a
 * fresh steering TPDO before the ECU has already sent the first realtime RPDO.
 * If steering output waits strictly for fresh TPDO0/TPDO1, CAN2 can deadlock:
 * steering is blocked as "axis not ready", and drive is blocked by pre-steer.
 *
 * In the explicit whole_vehicle_motion image only, allow a recent operational
 * heartbeat as the initial "remote node is alive" evidence.  The runtime still
 * requires high-voltage feedback, no remote estop, no latched fault, ordered
 * PDO group completion, and continues to replace this evidence with TPDO
 * feedback as soon as TPDOs arrive.
 */
#ifndef ECU_CANOPEN_STEER_BOOT_HEARTBEAT_EVIDENCE_ENABLED
#define ECU_CANOPEN_STEER_BOOT_HEARTBEAT_EVIDENCE_ENABLED \
    (ECU_BUILD_PROFILE_WHOLE_VEHICLE_MOTION)
#endif
#define ECU_CANOPEN_CAN2_MOTION_NODE_COUNT             (8U)
/* Drive velocity is ramped in a few discrete commissioning-friendly bands.
 * Reversal is intentionally the slowest path so a D/R sign change first eases
 * through zero instead of commanding an abrupt torque step.
 */
#define ECU_CANOPEN_DRIVE_VELOCITY_RATE_LIMIT_SMALL_UNITS_PER_SEC    (600000)
#define ECU_CANOPEN_DRIVE_VELOCITY_RATE_LIMIT_MEDIUM_UNITS_PER_SEC   (1200000)
#define ECU_CANOPEN_DRIVE_VELOCITY_RATE_LIMIT_LARGE_UNITS_PER_SEC    (2000000)
#define ECU_CANOPEN_DRIVE_VELOCITY_RATE_LIMIT_REVERSAL_UNITS_PER_SEC (500000)
#define ECU_CANOPEN_STEER_POSITION_DEADBAND_COUNTS (1000)
/* Steering joystick following is transmitted by RPDO at a fixed, moderate
 * cadence.  The vehicle/control layer owns target generation; the CAN2 motion
 * task owns all realtime CAN2 steering PDO traffic.  The 50 ms value matches
 * the stable CAN-analyzer remote simulation and avoids excessive SYNC/TPDO
 * bursts while the raw remote target is still refreshed by the 5 ms vehicle
 * control path.
 */
#define ECU_CANOPEN_STEER_PDO_PERIOD_MS  (20U)
/* A single profile-position steering update is an ordered group:
 * four arm RPDOs, SYNC, four trigger RPDOs, SYNC.  The CAN2 task may call the
 * realtime PDO pump multiple times inside one 2 ms scheduler tick so a freshly
 * queued group is pushed to the bus immediately instead of waiting for
 * background CANopen/SDO processing.
 */
#define ECU_CANOPEN_REALTIME_PDO_PUMP_PASSES (4U)
#define ECU_CANOPEN_STEER_POSITION_TRIGGER_THRESHOLD_COUNTS (1000)
#define ECU_CANOPEN_STEER_POSITION_NEUTRAL_DEADBAND_COUNTS  (1500)
/* Steering target smoothing is a trajectory layer:
 * - steer_latest_target_counts is the raw target from the latest remote sample.
 * - steer_commanded_target_counts is the accel/velocity-limited target sent by
 *   PDO.
 * Small errors use low velocity/acceleration so hand jitter is not amplified;
 * large errors use the profile values proven with the CAN analyzer.
 */
#define ECU_CANOPEN_STEER_TARGET_ERROR_NEAR_COUNTS                 (25000)
#define ECU_CANOPEN_STEER_TARGET_ERROR_SMALL_COUNTS                (150000)
#define ECU_CANOPEN_STEER_TARGET_ERROR_MEDIUM_COUNTS               (450000)
#define ECU_CANOPEN_STEER_TARGET_RATE_LIMIT_NEAR_COUNTS_PER_SEC    (120000)
#define ECU_CANOPEN_STEER_TARGET_RATE_LIMIT_SMALL_COUNTS_PER_SEC   (350000)
#define ECU_CANOPEN_STEER_TARGET_RATE_LIMIT_MEDIUM_COUNTS_PER_SEC  (1400000)
#define ECU_CANOPEN_STEER_TARGET_RATE_LIMIT_LARGE_COUNTS_PER_SEC   (5000000)
#define ECU_CANOPEN_STEER_TARGET_ACCEL_NEAR_COUNTS_PER_SEC2        (100000)
#define ECU_CANOPEN_STEER_TARGET_ACCEL_SMALL_COUNTS_PER_SEC2       (300000)
#define ECU_CANOPEN_STEER_TARGET_ACCEL_MEDIUM_COUNTS_PER_SEC2      (500000)
#define ECU_CANOPEN_STEER_TARGET_ACCEL_LARGE_COUNTS_PER_SEC2       (500000)
/* Fixed-posture transitions (spin/crab) are planned as one four-axis
 * trajectory from TPDO actual positions.  Keep the speed below the drive
 * profile velocity, but high enough that a 135 degree spin->crab axis does not
 * lag seconds behind the shorter axes.
 */
#define ECU_STEER_FIXED_TRANSITION_MAX_SPEED_COUNTS_PER_SEC (2700000)
#define ECU_STEER_FIXED_TRANSITION_MIN_MS                   (900U)
#define ECU_STEER_FIXED_TRANSITION_MAX_MS                   (2200U)
/* Field return-to-zero tuning, CAN analyzer 2026-07-10:
 * - profile position mode, RPDO1, 0x000F arm -> 0x001F trigger;
 * - 0x6081 = 2400000, 0x6083/0x6084 = 1200000;
 * - arrival judged by ±20000 counts and ±30000 velocity-units for 3 feedback
 *   samples.  Keep these separate from joystick smoothing so calibration and
 *   fixed-posture recovery can be tightened without changing normal steering.
 */
#define ECU_STEER_RETURN_ZERO_PROFILE_VELOCITY_COUNTS_PER_SEC (2400000)
#define ECU_STEER_RETURN_ZERO_PROFILE_ACCEL_COUNTS_PER_SEC2   (1200000)
#define ECU_STEER_RETURN_ZERO_POSITION_TOLERANCE_COUNTS       (20000)
#define ECU_STEER_RETURN_ZERO_VELOCITY_TOLERANCE_UNITS        (30000)
#define ECU_STEER_RETURN_ZERO_STABLE_SAMPLES                  (3U)
/* Field steering zero-search parameters, CAN analyzer reference v4.
 *
 * The calibration workflow is intentionally velocity-mode only.  Limit search
 * uses two stages: fast, then a 30% slower approach near the expected end stop.
 * Return-to-zero uses three stages: fast, medium, and slow only when the axis is
 * already close to the calculated midpoint.  The CAN2 motion task owns the whole
 * sequence; normal remote steering and drive PDOs are blocked until complete.
 */
#define ECU_STEER_ZERO_SEARCH_FAST_VELOCITY_UNITS             (1600000)
#define ECU_STEER_ZERO_SEARCH_SLOW_VELOCITY_UNITS             (1120000)
#define ECU_STEER_ZERO_SEARCH_SLOWDOWN_ABS_COUNTS             (2400000)
#define ECU_STEER_ZERO_SEARCH_PROFILE_VELOCITY_COUNTS_PER_SEC (2000000)
#define ECU_STEER_ZERO_SEARCH_PROFILE_ACCEL_COUNTS_PER_SEC2   (1000000)
#define ECU_STEER_ZERO_STALL_CURRENT_10MA                     (750)
#define ECU_STEER_ZERO_PROTECTION_CURRENT_10MA                (900)
#define ECU_STEER_ZERO_STALL_DWELL_MS                         (150U)
#define ECU_STEER_ZERO_STALL_ARM_DELAY_MS                     (400U)
#define ECU_STEER_ZERO_MAX_TRAVEL_COUNTS                      (4000000)
#define ECU_STEER_ZERO_MID_RETURN_FAST_VELOCITY_UNITS         (1600000)
#define ECU_STEER_ZERO_MID_RETURN_MEDIUM_VELOCITY_UNITS       (700000)
#define ECU_STEER_ZERO_MID_RETURN_SLOW_VELOCITY_UNITS         (180000)
#define ECU_STEER_ZERO_MID_RETURN_MEDIUM_ERROR_COUNTS         (500000)
#define ECU_STEER_ZERO_MID_RETURN_SLOW_ERROR_COUNTS           (50000)
#define ECU_STEER_ZERO_MIDPOINT_TOLERANCE_COUNTS              (10000)
#define ECU_STEER_ZERO_MIDPOINT_STABLE_SAMPLES                (2U)
/* Physical-left search direction for Node5..8 in vehicle leg order. */
#define ECU_STEER_ZERO_LEG1_LEFT_SIGN                         (1)
#define ECU_STEER_ZERO_LEG2_LEFT_SIGN                         (-1)
#define ECU_STEER_ZERO_LEG3_LEFT_SIGN                         (1)
#define ECU_STEER_ZERO_LEG4_LEFT_SIGN                         (-1)
#define ECU_REMOTE_B1_ZERO_CALIBRATION_PRESS_COUNT            (3U)
#define ECU_REMOTE_B1_ZERO_CALIBRATION_WINDOW_MS              (2000U)
#define ECU_REMOTE_B1_ZERO_CALIBRATION_REQUEST_HOLD_MS        (600U)
#define ECU_STEER_ZERO_VELOCITY_PDO_PERIOD_MS                 (20U)
#define ECU_STEER_ZERO_SETUP_TIMEOUT_MS                       (30000U)
#define ECU_STEER_ZERO_SEARCH_TIMEOUT_MS                      (60000U)
#define ECU_STEER_ZERO_RETURN_TIMEOUT_MS                      (35000U)
#define ECU_STEER_ZERO_VERIFY_TIMEOUT_MS                      (2000U)
#define ECU_STEER_ZERO_SDO_TIMEOUT_MS                         (1000U)
/* Node6 and Node8 were measured to arrive late when changing from spin to crab
 * because they can travel roughly twice the steering distance of the other two
 * axes.  Keep this as a configuration mask so field tuning can change the
 * compensation without touching the planner logic.  bit1=Node6, bit3=Node8.
 */
#define ECU_STEER_CRAB_FAST_AXIS_MASK                       ((uint8_t)0x0AU)
#define ECU_STEER_CRAB_FAST_AXIS_PROGRESS_NUMERATOR         (2U)
#define ECU_STEER_CRAB_FAST_AXIS_PROGRESS_DENOMINATOR       (1U)
/* Spin and crab are not safe to drive while the wheels are still slewing
 * toward their large steering angles.  The CAN2 motion adapter therefore keeps
 * drive RPDOs disabled until TPDO position feedback from all four steering
 * axes is within this window.  50000 counts is about 3.7 degrees with the
 * installed 490:1 steering reducer and 10000 count/rev encoder feedback.
 */
#define ECU_CANOPEN_PRESTEER_POSITION_TOLERANCE_COUNTS             (50000)
/* Track-width hydraulics only need the wheels roughly sideways before the
 * valve opens.  Keep this looser than the drive presteer tolerance, but still
 * require fresh feedback from all four steering axes before any valve request
 * reaches the hydraulic device.  100000 counts is about 7.35 degrees.
 */
#define ECU_TRACK_ASSIST_STEER_APPROX_TOLERANCE_COUNTS             (100000)
/* Track-width valves must not open from one lucky steering feedback sample.
 * Require all four steering axes to stay inside the approximate sideways
 * posture window for this time before CAN3 may energize a valve.
 */
#define ECU_TRACK_ASSIST_STEER_READY_STABLE_MS                     (300U)
#define ECU_CANOPEN_PRESTEER_TIMEOUT_MS                            (12000U)
#define ECU_CANOPEN_PRESTEER_REQUIRED_AXIS_MASK \
    ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL
/* Device-level steering absolute-position clamp.  The normal Ackermann remote
 * limit is lower, but spin/crab require large fixed postures.  Crab mode must
 * be able to command four absolute +90 deg targets:
 * 2500-line encoder * 4x * 490:1 / 4 = 1,225,000 counts.
 */
#define ECU_CANOPEN_STEER_MAX_POSITION_COUNTS               (1225000)
#define ECU_CANOPEN_STEER_SETUP_SETTLE_MS                   (100U)
/* V8 remote steering commissioning starts with a deliberately slow, small and
 * explicitly-authorized control envelope.  These values are independent from
 * the normal ±45 degree vehicle steering envelope so first-motion tests cannot
 * accidentally inherit production steering authority.
 */
#define ECU_STEER_REMOTE_COMMISSION_AUTH_MAGIC              (0x53544552UL)
#define ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL           (0x0FU)
#ifndef ECU_STEER_REMOTE_COMMISSION_MAX_DEG
#if ECU_BUILD_PROFILE_STEER4_REMOTE_90
#define ECU_STEER_REMOTE_COMMISSION_MAX_DEG                 (90.0f)
#else
#define ECU_STEER_REMOTE_COMMISSION_MAX_DEG                 (5.0f)
#endif
#endif
#define ECU_STEER_REMOTE_COMMISSION_PERIOD_MS               (100U)
#define ECU_STEER_REMOTE_COMMISSION_TRIGGER_THRESHOLD_COUNTS (5000)
#define ECU_STEER_REMOTE_COMMISSION_NEUTRAL_MS              (300U)
#if ECU_BUILD_PROFILE_WHOLE_VEHICLE_MOTION
/* The current unified PDO profile uses synchronous TPDO feedback and the
 * whole-vehicle debug image keeps SYNC rate conservative while the chassis is
 * suspended.  Use a commissioning-only freshness window that covers the
 * observed TPDO cadence without weakening the safe/default images.
 */
#define ECU_STEER_REMOTE_COMMISSION_FEEDBACK_TIMEOUT_MS     (1500U)
#else
#define ECU_STEER_REMOTE_COMMISSION_FEEDBACK_TIMEOUT_MS     (250U)
#endif
#define ECU_STEER_REMOTE_COMMISSION_POST_COMMAND_TPDO_TIMEOUT_MS (250U)
#define ECU_STEER_REMOTE_COMMISSION_CENTER_TOLERANCE_COUNTS (10000)
#define ECU_STEER_REMOTE_COMMISSION_RAMP_COUNTS_PER_SEC     (250000)
#define ECU_STEER_REMOTE_COMMISSION_SYNC_TIMEOUT_MS         (50U)
#if ECU_BUILD_PROFILE_STEER4_REMOTE_90 || ECU_BUILD_PROFILE_WHOLE_VEHICLE_MOTION
/* Field commissioning workaround: the HPM classic CAN receive path currently
 * observes Node8 TPDO0 (0x188) but not Node8 TPDO1 (0x288), although the
 * external analyzer confirms 0x288 is present on the bus.  Keep this limited to
 * explicit hardware commissioning images; normal/safe builds must not accept a
 * missing TPDO1 status/fault frame as healthy feedback.
 */
#define ECU_CANOPEN_NODE8_TPDO1_ACCEPTANCE_WORKAROUND       (1U)
#else
#define ECU_CANOPEN_NODE8_TPDO1_ACCEPTANCE_WORKAROUND       (0U)
#endif
/* Analyzer-only PDO capture switch.  Default is disabled because real drives
 * have not yet confirmed their RPDO mapping/readback.  Setting this to 1 is
 * only for a bounded CAN analyzer capture on an unpopulated actuator bus; it is
 * not evidence that any servo accepted or executed the command.
 */
#define ECU_CAN2_BENCH_PDO_CAPTURE_MODE (0)

#define ECU_CANOPEN_COMMISSIONING_POLICY_MAPPING_VERIFY_ONLY  (0U)
#define ECU_CANOPEN_COMMISSIONING_POLICY_TPDO_MONITOR_ONLY    (1U)
#define ECU_CANOPEN_COMMISSIONING_POLICY_NODE5_STEER_PDO_ONLY (2U)
#define ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED   (3U)
#define ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING (4U)

#ifndef ECU_CANOPEN_COMMISSIONING_POLICY
#define ECU_CANOPEN_COMMISSIONING_POLICY \
    ECU_CANOPEN_COMMISSIONING_POLICY_MAPPING_VERIFY_ONLY
#endif

/* Whole-machine commissioning switch.  Keep the drive-wheel command path built
 * and tested, but force walking motors to zero/braked while steering is being
 * tuned on the real vehicle.  Set to 0 after steering response is verified.
 */
#ifndef ECU_COMMISSIONING_STEER_ONLY_MODE
#define ECU_COMMISSIONING_STEER_ONLY_MODE (1U)
#endif

/* Shared offline retry backoff for no-device bench bring-up.  Missing BMS,
 * CANopen nodes or Modbus devices must stay diagnostic-visible without causing
 * high-rate timeout floods in periodic tasks.
 */
#define ECU_OFFLINE_BACKOFF_MIN_MS    (100U)
#define ECU_OFFLINE_BACKOFF_STEP1_MS  (500U)
#define ECU_OFFLINE_BACKOFF_STEP2_MS  (2000U)
#define ECU_OFFLINE_BACKOFF_MAX_MS    (5000U)

/* CANopen object indexes used by the BC/BC2 servo adapter.  0x2190 reports
 * drive terminal input states.  The ECU production firmware does not write
 * drive terminal outputs such as 0x2194; servo brake wiring is owned by the
 * drive internal brake controller.
 */
#define ECU_CANOPEN_OBJ_DEVICE_TYPE      (0x1000U)
#define ECU_CANOPEN_OBJ_ERROR_REGISTER   (0x1001U)
#define ECU_CANOPEN_OBJ_STORE_PARAMETERS (0x1010U)
#define ECU_CANOPEN_OBJ_IDENTITY         (0x1018U)
#define ECU_CANOPEN_OBJ_STATUSWORD       (0x6041U)
#define ECU_CANOPEN_OBJ_CONTROLWORD      (0x6040U)
#define ECU_CANOPEN_OBJ_MODES_OF_OPERATION (0x6060U)
#define ECU_CANOPEN_OBJ_MODES_OF_OPERATION_DISPLAY (0x6061U)
#define ECU_CANOPEN_OBJ_PROFILE_VELOCITY (0x6081U)
#define ECU_CANOPEN_OBJ_PROFILE_ACCELERATION (0x6083U)
#define ECU_CANOPEN_OBJ_PROFILE_DECELERATION (0x6084U)
#define ECU_CANOPEN_OBJ_FOLLOWING_ERROR_WINDOW (0x2120U)
/* BC/BC2 vendor interpolation option used by the CAN-analyzer lift script.
 * Field test showed this write is part of the stable 10..490 mm setup
 * sequence. Keep the object/value centralized until the vendor manual naming
 * is confirmed in project documentation.
 */
#define ECU_CANOPEN_OBJ_BC_INTERPOLATION_OPTION (0x2300U)
#define ECU_CANOPEN_BC_INTERPOLATION_OPTION_VALUE (0x001EU)
#define ECU_CANOPEN_OBJ_COMMAND_CURRENT_RAMP (0x2113U)
#define ECU_CANOPEN_OBJ_COMMAND_CURRENT  (0x2340U)
#define ECU_CANOPEN_OBJ_TARGET_POSITION  (0x607AU)
#define ECU_CANOPEN_OBJ_TARGET_VELOCITY  (0x60FFU)
#define ECU_CANOPEN_OBJ_INTERPOLATION_DATA_RECORD (0x60C1U)
#define ECU_CANOPEN_OBJ_INTERPOLATION_TIME_PERIOD (0x60C2U)
#define ECU_CANOPEN_OBJ_INTERPOLATION_MODE        (0x60C0U)
#define ECU_CANOPEN_OBJ_INTERPOLATION_BUFFER_CLEAR (0x60C4U)
#define ECU_CANOPEN_OBJ_DIGITAL_INPUT_STATES (0x2190U)
#define ECU_CANOPEN_OBJ_DENY_PROGRAM_CONTROL_OUTPUT_STATES (0x2194U)
#define ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM (0x1400U)
#define ECU_CANOPEN_OBJ_RPDO1_MAPPING    (0x1600U)
#define ECU_CANOPEN_OBJ_RPDO2_COMM_PARAM (0x1401U)
#define ECU_CANOPEN_OBJ_RPDO2_MAPPING    (0x1601U)
#define ECU_CANOPEN_OBJ_RPDO3_COMM_PARAM (0x1402U)
#define ECU_CANOPEN_OBJ_RPDO3_MAPPING    (0x1602U)
#define ECU_CANOPEN_OBJ_RPDO4_COMM_PARAM (0x1403U)
#define ECU_CANOPEN_OBJ_RPDO4_MAPPING    (0x1603U)
#define ECU_CANOPEN_OBJ_TPDO1_COMM_PARAM (0x1800U)
#define ECU_CANOPEN_OBJ_TPDO1_MAPPING    (0x1A00U)
#define ECU_CANOPEN_OBJ_TPDO2_COMM_PARAM (0x1801U)
#define ECU_CANOPEN_OBJ_TPDO2_MAPPING    (0x1A01U)
#define ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX (0x01U)
#define ECU_CANOPEN_OBJ_PDO_TRANSMISSION_TYPE_SUBINDEX (0x02U)
#define ECU_CANOPEN_OBJ_PDO_MAPPING_COUNT_SUBINDEX (0x00U)
#define ECU_CANOPEN_OBJ_PDO_MAPPING_FIRST_SUBINDEX (0x01U)
#define ECU_CANOPEN_OBJ_PDO_MAPPING_SECOND_SUBINDEX (0x02U)
#define ECU_CANOPEN_OBJ_PDO_MAPPING_THIRD_SUBINDEX (0x03U)
#define ECU_CANOPEN_PDO_MAP_CONTROLWORD_16       (0x60400010UL)
#define ECU_CANOPEN_PDO_MAP_MODE_OF_OPERATION_8  (0x60600008UL)
#define ECU_CANOPEN_PDO_MAP_TARGET_POSITION_32   (0x607A0020UL)
#define ECU_CANOPEN_PDO_MAP_TARGET_VELOCITY_32   (0x60FF0020UL)
#define ECU_CANOPEN_PDO_MAP_ACTUAL_POSITION_32   (0x60640020UL)
#define ECU_CANOPEN_OBJ_ACTUAL_POSITION          (0x6064U)
#define ECU_CANOPEN_OBJ_FAULT_LATCHED            (0x2183U)
#define ECU_CANOPEN_PDO_MAP_ACTUAL_VELOCITY_32   (0x606C0020UL)
#define ECU_CANOPEN_PDO_MAP_FAULT_LATCHED_32     (0x21830020UL)
#define ECU_CANOPEN_PDO_MAP_STATUSWORD_16        (0x60410010UL)
#define ECU_CANOPEN_PDO_MAP_ACTUAL_CURRENT_16    (0x221C0010UL)
#define ECU_CANOPEN_RPDO_TRANSMISSION_ASYNC      (0xFFU)
#define ECU_CANOPEN_TPDO_TRANSMISSION_SYNC1      (0x01U)

/* Production SDO writes are disabled by default.
 *
 * PDO mapping and drive flash save have already been configured by the CAN
 * analyzer.  Normal firmware must not use debug/generic SDO writes as a
 * backdoor for mapping, flash-save, actuator target, controlword, or drive
 * terminal-output writes.  A future maintenance image may override this only
 * with a separate physical safety procedure.
 */
#ifndef ECU_ENABLE_MAINTENANCE_SDO_WRITES
#define ECU_ENABLE_MAINTENANCE_SDO_WRITES (0)
#endif

typedef enum {
    ECU_BRAKE_ACTUATION_OWNER_SERVO_DRIVE_INTERNAL = 0
} ecu_brake_actuation_owner_t;

/* Brake ownership.
 *
 * Field wiring confirms the brake-release signal is active high, but the
 * signal is driven by the servo drive internal brake controller.  ECU software
 * may request CiA-402 operation enable through approved motion commands; it
 * must not synthesize a local DIO output or 0x2194/OUT1 write for brake
 * release.
 */
/* Encoder position-count contracts are node-role specific.
 *
 * Node1..8 (drive + steering) and Node13 (hydraulic pump) report 10000
 * position counts per motor revolution. Node9..12 lift axes report 131072
 * position counts per motor revolution and must never use either 10000-count
 * conversion below. The velocity objects use 0.1 encoder-count/s units on the
 * measured Node1..8/13 drives.
 */
#define ECU_CAN2_MOTION_ENCODER_COUNTS_PER_REV          (10000.0f)
#define ECU_HYDRAULIC_PUMP_ENCODER_COUNTS_PER_REV       (10000.0f)
#define ECU_CANOPEN_VELOCITY_UNITS_PER_COUNT_PER_SEC    (10.0f)
#define ECU_CAN2_MOTION_VELOCITY_UNITS_PER_RPM \
    ((ECU_CAN2_MOTION_ENCODER_COUNTS_PER_REV * \
      ECU_CANOPEN_VELOCITY_UNITS_PER_COUNT_PER_SEC) / 60.0f)
#define ECU_HYDRAULIC_PUMP_VELOCITY_UNITS_PER_RPM \
    ((ECU_HYDRAULIC_PUMP_ENCODER_COUNTS_PER_REV * \
      ECU_CANOPEN_VELOCITY_UNITS_PER_COUNT_PER_SEC) / 60.0f)
#define ECU_SERVO_MOTION_MAX_RPM                     (3000.0f)
/* 3000 rpm * 10000 encoder-count/rev / 0.1 / 60 = 5,000,000 velocity units. */
#define ECU_SERVO_MOTION_MAX_VELOCITY_UNITS_FROM_RPM (5000000)
#define ECU_DRIVE_GEAR_REDUCTION                     (86.6f)
#define ECU_DRIVE_WHEEL_DIAMETER_M                   (0.580f)
#define ECU_DRIVE_WHEEL_CIRCUMFERENCE_M              (1.822124f)
#define ECU_DRIVE_MOTOR_MAX_RPM                      ECU_SERVO_MOTION_MAX_RPM
#define ECU_DRIVE_MAX_SPEED_MPS \
    ((ECU_DRIVE_MOTOR_MAX_RPM / ECU_DRIVE_GEAR_REDUCTION) * \
     ECU_DRIVE_WHEEL_CIRCUMFERENCE_M / 60.0f)
/* Current whole-machine commissioning keeps operator speed authority at
 * 0.50 m/s.  The CAN2/CAN3 motion-servo ceiling is separately capped at
 * 3000 rpm (about 1.05 m/s at the installed 86.6:1 gearbox and 580 mm wheel),
 * but
 * field tuning stays below that ceiling until brake release, current limit
 * and drive alarm behavior are verified on each wheel.
 */
#define ECU_DRIVE_COMMISSIONING_MAX_SPEED_MPS        (0.50f)
#define ECU_STEER_GEAR_REDUCTION                     (490.0f)
#define ECU_STEER_COUNTS_PER_OUTPUT_REV \
    (ECU_CAN2_MOTION_ENCODER_COUNTS_PER_REV * ECU_STEER_GEAR_REDUCTION)
#define ECU_STEER_POSITION_SPEED_UNITS               (5000000)
/* Steering has a 490:1 reducer and was field-tested as too slow at the generic
 * old hydraulic-pump 2400 rpm commissioning limit.  CAN2 drive and steering
 * servos are allowed to use the field-confirmed 3000 rpm motor ceiling.
 */
#if ECU_STEER_POSITION_SPEED_UNITS > ECU_SERVO_MOTION_MAX_VELOCITY_UNITS_FROM_RPM
#error ECU_STEER_POSITION_SPEED_UNITS <= ECU_SERVO_MOTION_MAX_VELOCITY_UNITS_FROM_RPM
#endif
/* CAN3 lift calibration is separate from Node1..8/13 10000-count motor units.
 * Field measurement on the installed ground-clearance actuator:
 * 131072 count/motor-rev and 12 motor rev per 10 mm linear travel.
 */
#define ECU_LIFT_ENCODER_COUNTS_PER_REV              (131072.0f)
#define ECU_LIFT_MOTOR_REVS_PER_MM                   (1.2f)
#define ECU_LIFT_MM_TO_COUNTS \
    (ECU_LIFT_ENCODER_COUNTS_PER_REV * ECU_LIFT_MOTOR_REVS_PER_MM)
#define ECU_LIFT_SHORTEST_POSITION_COUNTS            (-10000)
#define ECU_LIFT_LONGEST_POSITION_COUNTS \
    ((int32_t)(-(ECU_LIFT_MM_TO_COUNTS * 490.0f)))
#define ECU_LIFT_INTERPOLATION_SPEED_COUNTS_PER_SEC \
    ((int32_t)(ECU_LIFT_MM_TO_COUNTS * 5.0f))
#define ECU_LIFT_INTERPOLATION_ACCEL_COUNTS_PER_SEC2 \
    ((int32_t)(ECU_LIFT_MM_TO_COUNTS * 8.0f))
/* Analyzer-derived full-stroke lift parameters, 2026-07-11:
 * 5.0 mm/s stream, 8.0 mm/s^2 stream ramp, 75000-count nominal target lead,
 * 24,000,000 vendor profile-velocity units and 500,000 vendor
 * profile-acceleration units. These two vendor object values are analyzer
 * tuned; do not derive them from the Node1..8 10000-count velocity scale.
 * The successful analyzer sequence starts all four trajectories from measured
 * positions and runs 10->490->10 mm without EMCY. 20 mm/s and 7.5 mm/s
 * full-stroke attempts tripped Node9 0x7390.
 * ECU uses the script's minimum proven preload of three stationary points.
 * Every running point uses a per-axis final displacement, monotonic lead clamp
 * and bounded 0.25 feedback correction; the four-axis batch is followed by one
 * common SYNC.
 */
#define ECU_LIFT_PROFILE_VELOCITY_UNITS             (24000000)
#define ECU_LIFT_PROFILE_ACCEL_UNITS                (500000)
#define ECU_LIFT_FOLLOWING_ERROR_WINDOW_COUNTS \
    ((int32_t)(ECU_LIFT_MM_TO_COUNTS * 3.0f))
#define ECU_LIFT_INTERPOLATION_TARGET_LEAD_COUNTS \
    (75000)
#define ECU_LIFT_SYNC_CORRECTION_GAIN_NUMERATOR   (1)
#define ECU_LIFT_SYNC_CORRECTION_GAIN_DENOMINATOR (4)
#define ECU_LIFT_SYNC_CORRECTION_MAX_COUNTS       (50000)
#define ECU_LIFT_TARGET_REACHED_TOLERANCE_COUNTS \
    ((int32_t)(ECU_LIFT_MM_TO_COUNTS * 3.0f))
/* Analyzer-proven lift contract:
 * - 20 ms interpolation period;
 * - three preloaded stationary points;
 * - no impossible pre-trigger leveling stage;
 * - one coherent absolute-position target stream per four-axis cycle;
 * - 75000-count maximum command lead over measured feedback;
 * - final acceptance only after all axes are within 3 mm.
 */
#define ECU_LIFT_FINAL_SPREAD_TOLERANCE_COUNTS \
    ((int32_t)(ECU_LIFT_MM_TO_COUNTS * 3.0f))
/* Normal remote-center confirmation while lift interpolation is already
 * running. This filters SBUS threshold chatter only; safety-source stops,
 * high-voltage loss, CAN faults and non-remote commands still stop immediately.
 */
#define ECU_LIFT_REMOTE_NEUTRAL_STOP_CONFIRM_MS     (150U)
#define ECU_LIFT_STALL_PROGRESS_COUNTS \
    ((int32_t)(ECU_LIFT_MM_TO_COUNTS * 0.05f))
#define ECU_LIFT_STALL_TIMEOUT_MS                   (5000U)
#define ECU_LIFT_RECOVERY_BACKOFF_MS                (1000U)
#define ECU_LIFT_RECOVERY_COOLDOWN_MS               (10000U)
#define ECU_LIFT_RECOVERY_FAST_ATTEMPTS             (3U)
#define ECU_HYDRAULIC_PUMP_WORK_RPM                  (1500.0f)
#define ECU_HYDRAULIC_PUMP_MAX_REVERSE_RPM           (2400.0f)
#define ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_RPM        (800.0f)
#define ECU_HYDRAULIC_PUMP_SPEED_READY_SAMPLES       (3U)
#define ECU_HYDRAULIC_PUMP_START_TIMEOUT_MS          (3000U)
#define ECU_HYDRAULIC_VALVE_CHANGE_DEADTIME_MS       (10U)
#define ECU_HYDRAULIC_PUMP_ENABLE_VELOCITY_UNITS \
    ((int32_t)((ECU_HYDRAULIC_PUMP_WORK_RPM * \
                ECU_HYDRAULIC_PUMP_VELOCITY_UNITS_PER_RPM) + 0.5f))
#define ECU_HYDRAULIC_PUMP_MIN_WORK_VELOCITY_UNITS \
    ECU_HYDRAULIC_PUMP_ENABLE_VELOCITY_UNITS
#define ECU_HYDRAULIC_PUMP_MAX_REVERSE_VELOCITY_UNITS \
    ((int32_t)((ECU_HYDRAULIC_PUMP_MAX_REVERSE_RPM * \
                ECU_HYDRAULIC_PUMP_VELOCITY_UNITS_PER_RPM) + 0.5f))
#define ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_VELOCITY_UNITS \
    ((int32_t)((ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_RPM * \
                ECU_HYDRAULIC_PUMP_VELOCITY_UNITS_PER_RPM) + 0.5f))
#define ECU_HYDRAULIC_PUMP_DIRECTION_SIGN (-1)
#define ECU_HYDRAULIC_PUMP_ALLOW_POSITIVE_VELOCITY (0)
#define ECU_SERVO_COMMAND_CURRENT_RAMP_MA_PER_SEC    (1000)
/* Track-width assist is intentionally conservative for hydraulic tests.
 * The four drive motors use RPDO3 current mode only after the track valve is
 * physically opened by the CAN3 hydraulic adapter.  Unit: 10 mA.
 *
 * Field setting: legs 1/4 stay at 7 A; legs 2/3 are raised to 10 A because
 * they still needed more assist under track-width load. */
#define ECU_TRACK_ASSIST_LEG1_CURRENT_10MA           (700)
#define ECU_TRACK_ASSIST_LEG2_CURRENT_10MA           (1000)
#define ECU_TRACK_ASSIST_LEG3_CURRENT_10MA           (1000)
#define ECU_TRACK_ASSIST_LEG4_CURRENT_10MA           (700)
#define ECU_REMOTE_TRACK_ASSIST_CENTER_EXIT_MS       (5000U)

/* Commissioning scale factors.  BC/BC2 0x60FF and 0x606C use velocity units of
 * 0.1 motor-encoder count/s.  The drive conversion includes the installed
 * wheel gearbox and 580 mm tire diameter so a vehicle-speed request produces a
 * motor-side target accepted by the drive. */
#define ECU_DRIVE_SPEED_MPS_TO_COUNTS_PER_SEC \
    ((ECU_CAN2_MOTION_ENCODER_COUNTS_PER_REV * ECU_DRIVE_GEAR_REDUCTION * \
      ECU_CANOPEN_VELOCITY_UNITS_PER_COUNT_PER_SEC) / \
     ECU_DRIVE_WHEEL_CIRCUMFERENCE_M)
/* Field calibration from installed steering gearbox:
 * 2500-line encoder * 4x drive decoding * 490:1 reduction = 4,900,000 counts
 * per steering output revolution.  Positive target is left, negative is right. */
#define ECU_STEER_DEG_TO_COUNTS \
    (ECU_STEER_COUNTS_PER_OUTPUT_REV / 360.0f)

/* Local digital outputs stay limited to board-level loads.  Servo brakes are
 * controlled by the drive internal brake controller, not by PCB DIO and not by
 * ECU writes to drive output objects.
 *
 * Relay box wiring v1.5:
 *   - MOS1  / EX_OUT1  -> hydraulic valve 1 control, relay input active low.
 *   - MOS2  / EX_OUT2  -> headlight control, relay input active low.
 *   - MOS3  / EX_OUT3  -> hydraulic valve 2 control, relay input active low.
 *   - MOS4  / EX_OUT4  -> horn control, relay input active low.
 *   - MOS5  / EX_OUT5  -> hydraulic valve 3 control, relay input active low.
 *   - MOS6  / EX_OUT6  -> battery key / high-voltage key control, relay input active low.
 *   - MOS7  / EX_OUT7  -> hydraulic valve 4 control, relay input active low.
 *   - MOS8  / EX_OUT8  -> reserved 3 control, relay input active low.
 *   - MOS9  / EX_OUT9  -> hydraulic valve 5 control, relay input active low.
 *   - MOS10 / EX_OUT10 -> flash relay 1 / left indicator control, relay input active low.
 *   - MOS11 / EX_OUT11 -> hydraulic valve 6 control, relay input active low.
 *   - MOS12 / EX_OUT12 -> flash relay 2 / right indicator control, relay input active low.
 *
 * The relay input is low-level active, but the ECU GPIO command is active high:
 * GPIO high turns the MOS output on, and the MOS output pulls the relay control
 * terminal low.  Therefore dio_active_high remains true.
 *
 * MOS6 / EX_OUT6 is the battery-key high-voltage request output.  Keep MOS8
 * reserved so the old high-voltage output assignment cannot be reintroduced by
 * mistake.
 */
#define ECU_DIO_BRAKE_RELEASE_MASK       (0UL)
#define ECU_DIO_HYDRAULIC_ENABLE_MASK    (0UL)
#define ECU_DIO_HEADLIGHT_MASK           (1UL << 1)
#define ECU_DIO_HORN_MASK                (1UL << 3)
#define ECU_DIO_HIGH_VOLTAGE_RELAY_MASK  (1UL << 5)
#define ECU_DIO_RESERVED3_MASK           (1UL << 7)
#define ECU_DIO_LEFT_INDICATOR_MASK      (1UL << 9)
#define ECU_DIO_RIGHT_INDICATOR_MASK     (1UL << 11)
#define ECU_DIO_MANAGED_OUTPUT_MASK      (ECU_DIO_HYDRAULIC_ENABLE_MASK | \
                                                ECU_DIO_HEADLIGHT_MASK | \
                                                ECU_DIO_HORN_MASK | \
                                                ECU_DIO_HIGH_VOLTAGE_RELAY_MASK | \
                                                ECU_DIO_RESERVED3_MASK | \
                                                ECU_DIO_LEFT_INDICATOR_MASK | \
                                                ECU_DIO_RIGHT_INDICATOR_MASK)

#define ECU_HYD_VALVE1_MASK              (1UL << 0)
#define ECU_HYD_VALVE2_MASK              (1UL << 2)
#define ECU_HYD_VALVE3_MASK              (1UL << 4)
#define ECU_HYD_VALVE4_MASK              (1UL << 6)
#define ECU_HYD_VALVE5_MASK              (1UL << 8)
#define ECU_HYD_VALVE6_MASK              (1UL << 10)
/* Hydraulic function mapping confirmed from relay-box wiring.
 *
 * Valve 1/2 operate the front suspension and valve 3/4 operate track width.
 * Valve 5/6 operate the rear suspension.  A later wiring correction should
 * only modify these masks; pump/valve sequencing stays unchanged.
 */
#define ECU_HYD_VALVE_TRACK_EXTEND_MASK  ECU_HYD_VALVE4_MASK
#define ECU_HYD_VALVE_TRACK_RETRACT_MASK ECU_HYD_VALVE3_MASK
#define ECU_HYD_VALVE_FRONT_SUSPENSION_RETRACT_MASK ECU_HYD_VALVE1_MASK
#define ECU_HYD_VALVE_FRONT_SUSPENSION_EXTEND_MASK  ECU_HYD_VALVE2_MASK
#define ECU_HYD_VALVE_REAR_SUSPENSION_RETRACT_MASK  ECU_HYD_VALVE5_MASK
#define ECU_HYD_VALVE_REAR_SUSPENSION_EXTEND_MASK   ECU_HYD_VALVE6_MASK
#define ECU_HYD_VALVE_PAIR12_MASK        (ECU_HYD_VALVE1_MASK | ECU_HYD_VALVE2_MASK)
#define ECU_HYD_VALVE_PAIR34_MASK        (ECU_HYD_VALVE3_MASK | ECU_HYD_VALVE4_MASK)
#define ECU_HYD_VALVE_PAIR56_MASK        (ECU_HYD_VALVE5_MASK | ECU_HYD_VALVE6_MASK)
#define ECU_HYD_VALVE_MANAGED_MASK       (ECU_HYD_VALVE1_MASK | \
                                          ECU_HYD_VALVE2_MASK | \
                                          ECU_HYD_VALVE3_MASK | \
                                          ECU_HYD_VALVE4_MASK | \
                                          ECU_HYD_VALVE5_MASK | \
                                          ECU_HYD_VALVE6_MASK)

#define ECU_ADC_CHANNEL_COUNT       (8U)
#define ECU_ADC_RAW_MAX             (4095U)
#define ECU_ADC_EXTERNAL_MV_MAX     (10000U)
#define ECU_ANALOG_SENSOR_FRONT_SUSPENSION_ANGLE_CH (0U)
#define ECU_ANALOG_SENSOR_REAR_SUSPENSION_ANGLE_CH  (1U)
#define ECU_ANALOG_SENSOR_LEG1_TRACK_CYLINDER_CH    (2U)
#define ECU_ANALOG_SENSOR_LEG2_TRACK_CYLINDER_CH    (3U)
#define ECU_ANALOG_SENSOR_LEG3_TRACK_CYLINDER_CH    (4U)
#define ECU_ANALOG_SENSOR_LEG4_TRACK_CYLINDER_CH    (5U)
#define ECU_ANALOG_SENSOR_RESERVED6_CH              (6U)
#define ECU_ANALOG_SENSOR_RESERVED7_CH              (7U)
#define ECU_MODBUS_ADC_SLAVE_ID     (0x01U)
#define ECU_MODBUS_ADC_BAUDRATE     (9600UL)
#define ECU_MODBUS_ADC_START_REGISTER (0U)
#define ECU_MODBUS_ADC_REGISTER_COUNT (8U)
#define ECU_MODBUS_ADC_RAW_MAX      (65535U)
#define ECU_MODBUS_ADC_POLL_PERIOD_MS (100U)
#define ECU_MODBUS_ADC_RESPONSE_TIMEOUT_MS (100U)
#define ECU_MODBUS_WARNING_LIGHT_SLAVE_ID (0xFFU)
#define ECU_MODBUS_WARNING_LIGHT_REGISTER (0x00C2U)
#define ECU_MODBUS_WARNING_LIGHT_BAUDRATE (9600UL)
#define ECU_MODBUS_WARNING_LIGHT_REQUEST_PERIOD_MS (100U)
#define ECU_MODBUS_WARNING_LIGHT_RESPONSE_TIMEOUT_MS (100U)
#define ECU_WARNING_LIGHT_VALUE_OFF (0x0060U)
#define ECU_WARNING_LIGHT_VALUE_YELLOW_SLOW_FLASH (0x0022U)
#define ECU_WARNING_LIGHT_VALUE_RED_STEADY_BUZZER (0x0014U)
#define ECU_REMOTE_MAX_SPEED_MPS          ECU_DRIVE_COMMISSIONING_MAX_SPEED_MPS
#define ECU_REMOTE_STEER_INPUT_SIGN       (-1)
#if ECU_BUILD_PROFILE_STEER4_REMOTE_90
#define ECU_REMOTE_MAX_STEER_DEG          (90.0f)
#else
#define ECU_REMOTE_MAX_STEER_DEG          (50.0f)
#endif
/* Vehicle geometry used by four-wheel kinematics.
 *
 * Wheelbase is the longitudinal distance between front and rear axle centers.
 * Track width is the lateral distance between left and right wheel centers.
 * The hydraulic track-width mechanism changes the lateral distance at runtime;
 * until closed-loop position feedback is wired into the command path, the
 * default value below is used by the remote and automatic command builders.
 */
#define ECU_VEHICLE_WHEELBASE_MIN_MM      (2880.0f)
#define ECU_VEHICLE_WHEELBASE_MM          (2880.0f)
#define ECU_VEHICLE_WHEELBASE_MAX_MM      (2880.0f)
#define ECU_VEHICLE_TRACK_WIDTH_MIN_MM    (1980.0f)
#define ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM (1980.0f)
#define ECU_VEHICLE_TRACK_WIDTH_MAX_MM    (2880.0f)
#define ECU_VEHICLE_MIN_TURN_RADIUS_MM    (1500.0f)
#define ECU_MOTION_SPIN_STEER_DEG         (45.0f)
#define ECU_MOTION_CRAB_STEER_DEG         (90.0f)
#define ECU_REMOTE_MIN_HEIGHT_TARGET_MM   (10.0f)
#define ECU_REMOTE_MAX_HEIGHT_TARGET_MM   (490.0f)
#define ECU_REMOTE_MAX_HEIGHT_RATE_MM_S   (20.0f)
#define ECU_REMOTE_CLEARANCE_UP_PER_MILLE_MIN     (778)
#define ECU_REMOTE_CLEARANCE_DOWN_PER_MILLE_MAX   (-778)
#define ECU_REMOTE_MAX_TRACK_RATE_MM_S    (20.0f)
#define ECU_REMOTE_TRACK_EXTEND_PER_MILLE_MIN     (778)
#define ECU_REMOTE_TRACK_RETRACT_PER_MILLE_MAX    (-778)
/* CH14 is an analog wheel.  The engage/release split prevents valve chatter
 * when the wheel is near the threshold or mechanically springs back slowly.
 */
#define ECU_REMOTE_TRACK_EXTEND_RELEASE_PER_MILLE_MIN     (650)
#define ECU_REMOTE_TRACK_RETRACT_RELEASE_PER_MILLE_MAX    (-650)
#define ECU_REMOTE_TRACK_REQUEST_STABLE_MS                (150U)
#define ECU_RS485_BAUDRATE          (115200UL)
#define ECU_RS232_BAUDRATE          (115200UL)
#define ECU_SBUS_UART_RX_IDLE_BITS  (24U)
#define ECU_SBUS_UART_IRQ_PRIORITY  (2U)
#define ECU_RS485_UART_IRQ_PRIORITY (2U)

typedef enum {
    ECU_SBUS_CH_STEER = 0,
    ECU_SBUS_CH_CLEARANCE = 1,
    ECU_SBUS_CH_THROTTLE = 2,
    ECU_SBUS_CH_POWER = 3,
    ECU_SBUS_CH_GEAR = 4,
    ECU_SBUS_CH_RIGHT_INDICATOR = 5,
    ECU_SBUS_CH_AUTHORITY = 6,
    ECU_SBUS_CH_HOME = 7,
    ECU_SBUS_CH_HAZARD = 8,
    ECU_SBUS_CH_HORN = 9,
    ECU_SBUS_CH_HEADLIGHT = 10,
    ECU_SBUS_CH_LEFT_INDICATOR = 11,
    ECU_SBUS_CH_ESTOP = 12,
    ECU_SBUS_CH_TRACK = 13,
    ECU_SBUS_CH_R1 = 14,
    ECU_SBUS_CH_R2 = 15,
    ECU_SBUS_CHANNEL_COUNT
} ecu_sbus_channel_role_t;

typedef struct {
    uint32_t magic;
    uint32_t command_sequence;
    bool high_voltage_enable;
    uint32_t request_time_ms;
} ecu_commissioning_control_t;

typedef struct {
    uint32_t magic;
    bool steer_remote_commission_enable;
    uint8_t enabled_axis_mask; /* bit0=Node5, bit1=Node6, bit2=Node7, bit3=Node8. */
    uint32_t expiry_ms;
} ecu_steer_commissioning_control_t;

typedef struct {
    bool valid;
    int8_t direction_sign; /* +1 means positive steering command increases counts. */
    int32_t straight_zero_offset_counts;
    int32_t minimum_position_counts;
    int32_t maximum_position_counts;
    float commissioning_max_abs_deg;
} steer_axis_calibration_t;

#define ECU_STEER_CALIBRATION_OVERRIDE_MAGIC (0x53544341UL)

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    bool enable;
    steer_axis_calibration_t axis[ECU_WHEEL_COUNT];
} ecu_steer_calibration_override_t;

extern volatile ecu_steer_calibration_override_t g_ecu_steer_calibration_override;

typedef struct {
    uint16_t low_max;
    uint16_t center_min;
    uint16_t center_max;
    uint16_t high_min;
    uint16_t stick_min;
    uint16_t stick_neutral;
    uint16_t stick_max;
    uint16_t neutral;
    uint16_t throttle_min;
    uint16_t throttle_max;
    uint16_t throttle_start;
} ecu_sbus_thresholds_t;

typedef struct {
    uint32_t discrete_debounce_ms;
    uint32_t link_qualify_ms;
    uint32_t neutral_qualify_ms;
    uint32_t failsafe_timeout_ms;
    uint32_t domain_event_guard_ms;
    uint32_t power_long_press_ms;
    uint32_t mode_request_ttl_ms;
    uint32_t power_request_ttl_ms;
    uint32_t estop_reset_ttl_ms;
    uint32_t light_request_ttl_ms;
    ecu_sbus_thresholds_t sbus_thresholds;
    uint32_t cpu0_task_period_ms[8];
    uint32_t cpu1_service_period_ms;
} ecu_config_t;

typedef struct {
    float steer_target_deg[ECU_WHEEL_COUNT];
    float assist_torque_sign[ECU_WHEEL_COUNT];
    int16_t assist_current_10ma[ECU_WHEEL_COUNT];
    float assist_wheel_speed_limit_rpm[ECU_WHEEL_COUNT];
} track_adjust_config_t;

typedef struct {
    uint8_t node_id;
    uint32_t tpdo1_cob_id;
    uint32_t rpdo1_cob_id;
    uint32_t rpdo2_cob_id;
    uint32_t rpdo3_cob_id;
    uint32_t rpdo4_cob_id;
    uint32_t rpdo5_cob_id;
    uint32_t heartbeat_cob_id;
    uint32_t timeout_ms;
} ecu_canopen_node_config_t;

typedef enum {
    ECU_POWER_PROTOCOL_DISABLED = 0,
    ECU_POWER_PROTOCOL_SUPPLIER_CAN
} ecu_power_protocol_t;

typedef struct {
    bool power_ready;
    bool low_voltage_ok;
    bool can1_power_online;
    bool can2_motion_online;
    bool can3_lift_hydraulic_online;
    bool brake_release_confirmed;
    bool hydraulic_stopped;
    bool zero_speed_confirmed;
} ecu_hardware_feedback_snapshot_t;

typedef struct {
    uint32_t can1_bitrate;
    uint32_t can2_bitrate;
    uint32_t can3_bitrate;
    uint32_t can4_bitrate;
    ecu_power_protocol_t power_protocol;
    ecu_canopen_node_config_t drive_nodes[ECU_WHEEL_COUNT];
    ecu_canopen_node_config_t steer_nodes[ECU_WHEEL_COUNT];
    ecu_canopen_node_config_t lift_nodes[ECU_WHEEL_COUNT];
    ecu_canopen_node_config_t hydraulic_pump_node;
    uint32_t dio_brake_release_mask;
    uint32_t dio_hydraulic_enable_mask;
    uint32_t dio_horn_mask;
    uint32_t dio_headlight_mask;
    uint32_t dio_left_indicator_mask;
    uint32_t dio_right_indicator_mask;
    uint32_t dio_high_voltage_relay_mask;
    uint32_t dio_managed_output_mask;
    bool dio_active_high;
    uint32_t hydraulic_track_extend_mask;
    uint32_t hydraulic_track_retract_mask;
    uint32_t hydraulic_managed_valve_mask;
    uint32_t hydraulic_valve_interlock_pair12_mask;
    uint32_t hydraulic_valve_interlock_pair34_mask;
    uint32_t hydraulic_valve_interlock_pair56_mask;
    uint32_t adc_channel_count;
    uint32_t adc_raw_max;
    uint32_t adc_external_mv_max;
    uint8_t modbus_adc_slave_id;
    uint32_t modbus_adc_baudrate;
    uint16_t modbus_adc_start_register;
    uint16_t modbus_adc_register_count;
    uint32_t modbus_adc_raw_max;
    uint32_t modbus_adc_poll_period_ms;
    uint32_t modbus_adc_response_timeout_ms;
    uint8_t modbus_warning_light_slave_id;
    uint16_t modbus_warning_light_register;
    uint32_t modbus_warning_light_baudrate;
    uint32_t modbus_warning_light_request_period_ms;
    uint32_t modbus_warning_light_response_timeout_ms;
    uint32_t rs485_baudrate;
    uint32_t rs232_baudrate;
    float drive_speed_mps_to_counts_per_sec;
    int8_t drive_direction_sign[ECU_WHEEL_COUNT];
    float steer_deg_to_counts;
    float lift_mm_to_counts;
    steer_axis_calibration_t steer_axis_calibration[ECU_WHEEL_COUNT];
} ecu_hardware_config_t;

const ecu_config_t *ecu_config_default(void);
const track_adjust_config_t *ecu_track_adjust_config_default(void);
const ecu_hardware_config_t *ecu_hardware_config_default(void);

#endif /* ECU_CONFIG_H */
