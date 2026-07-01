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
#define REMOTE_POWER_LONG_PRESS_MS      (1000U)
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

#ifndef ECU_ENABLE_DEBUG_MONITOR
#define ECU_ENABLE_DEBUG_MONITOR         (1)
#endif

#ifndef ECU_DEBUG_MONITOR_PERIOD_MS
#define ECU_DEBUG_MONITOR_PERIOD_MS      (500U)
#endif

#ifndef ECU_DEBUG_MONITOR_VERBOSE
#define ECU_DEBUG_MONITOR_VERBOSE        (1)
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
#define ECU_CAN1_TERMINATION_ENABLE      (0)
#define ECU_CAN2_TERMINATION_ENABLE      (0)
#define ECU_CAN3_TERMINATION_ENABLE      (1)
#define ECU_CAN4_TERMINATION_ENABLE      (1)

/* Bench-only CAN4 physical-layer test.  When enabled, CPU0 initializes the
 * auxiliary CAN4 controller and sends a harmless standard CAN frame at a fixed
 * period.  The payload is intentionally not a CANopen, BMS or actuator command.
 */
#ifndef ECU_ENABLE_CAN4_PHYSICAL_TEST_TX
#define ECU_ENABLE_CAN4_PHYSICAL_TEST_TX (1)
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
#define ECU_DCDC12_DEFAULT_OUTPUT_VOLTAGE_DV (275U)
#define ECU_DCDC12_DEFAULT_OUTPUT_CURRENT_DA (100U)
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

/* Default CiA-301 COB-ID bases.  Device adapters build TPDO/RPDO/heartbeat
 * identifiers from these bases plus the node ID unless a device-specific
 * configuration overrides them. */
#define ECU_CANOPEN_TPDO1_BASE     (0x180UL)
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

/* CANopen object indexes used by the BC/BC2 servo adapter.  0x2190 reports
 * drive terminal input states; 0x2194 writes outputs configured for program
 * control, including the brake-release output. */
#define ECU_CANOPEN_OBJ_DEVICE_TYPE      (0x1000U)
#define ECU_CANOPEN_OBJ_ERROR_REGISTER   (0x1001U)
#define ECU_CANOPEN_OBJ_IDENTITY         (0x1018U)
#define ECU_CANOPEN_OBJ_STATUSWORD       (0x6041U)
#define ECU_CANOPEN_OBJ_CONTROLWORD      (0x6040U)
#define ECU_CANOPEN_OBJ_MODES_OF_OPERATION (0x6060U)
#define ECU_CANOPEN_OBJ_MODES_OF_OPERATION_DISPLAY (0x6061U)
#define ECU_CANOPEN_OBJ_TARGET_TORQUE    (0x6071U)
#define ECU_CANOPEN_OBJ_TARGET_POSITION  (0x607AU)
#define ECU_CANOPEN_OBJ_TARGET_VELOCITY  (0x60FFU)
#define ECU_CANOPEN_OBJ_DIGITAL_INPUT_STATES (0x2190U)
#define ECU_CANOPEN_OBJ_OUTPUT_STATES_PROGRAM_CONTROL (0x2194U)

/* Brake output polarity.
 *
 * The installed servos are mechanically braked by default.  Current field
 * wiring releases the brake when the drive output is low.  Keep both polarity
 * macros in configuration so reversing the drive terminal polarity is a single
 * compile-time change instead of a logic edit in the device layer. */
#define ECU_SERVO_BRAKE_RELEASE_OUTPUT_ACTIVE_LEVEL (0U)
#define ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT  (0U)
#define ECU_HYDRAULIC_PUMP_ENABLE_VELOCITY_COUNTS_PER_SEC (1000)

/* Commissioning scale factors.  These convert high-level vehicle commands into
 * drive counts and remain centralized for motor/gearbox calibration updates on
 * the complete machine. */
#define ECU_DRIVE_SPEED_KPH_TO_COUNTS_PER_SEC (100.0f)
#define ECU_STEER_DEG_TO_COUNTS               (100.0f)
#define ECU_LIFT_MM_TO_COUNTS                 (100.0f)

/* Local digital outputs stay limited to board-level loads.  Servo brakes are
 * controlled through drive terminal outputs over CANopen, not through PCB DIO.
 */
#define ECU_DIO_BRAKE_RELEASE_MASK       (0UL)
#define ECU_DIO_HYDRAULIC_ENABLE_MASK    (1UL << 1)
#define ECU_DIO_HORN_MASK                (1UL << 2)
#define ECU_DIO_HEADLIGHT_MASK           (1UL << 3)
#define ECU_DIO_LEFT_INDICATOR_MASK      (1UL << 4)
#define ECU_DIO_RIGHT_INDICATOR_MASK     (1UL << 5)
#define ECU_DIO_MANAGED_OUTPUT_MASK      (ECU_DIO_HYDRAULIC_ENABLE_MASK | \
                                                ECU_DIO_HORN_MASK | \
                                                ECU_DIO_HEADLIGHT_MASK | \
                                                ECU_DIO_LEFT_INDICATOR_MASK | \
                                                ECU_DIO_RIGHT_INDICATOR_MASK)

#define ECU_HYD_VALVE_TRACK_EXTEND_MASK  (1UL << 8)
#define ECU_HYD_VALVE_TRACK_RETRACT_MASK (1UL << 9)
#define ECU_HYD_VALVE_LIFT_UP_MASK       (1UL << 10)
#define ECU_HYD_VALVE_LIFT_DOWN_MASK     (1UL << 11)
#define ECU_HYD_VALVE_MANAGED_MASK       (ECU_HYD_VALVE_TRACK_EXTEND_MASK | \
                                                ECU_HYD_VALVE_TRACK_RETRACT_MASK | \
                                                ECU_HYD_VALVE_LIFT_UP_MASK | \
                                                ECU_HYD_VALVE_LIFT_DOWN_MASK)

#define ECU_ADC_CHANNEL_COUNT       (8U)
#define ECU_ADC_RAW_MAX             (4095U)
#define ECU_ADC_EXTERNAL_MV_MAX     (5000U)
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
#define ECU_REMOTE_MAX_SPEED_KPH          (6.0f)
#define ECU_REMOTE_MAX_STEER_DEG          (35.0f)
/* Vehicle geometry used by four-wheel kinematics.
 *
 * Wheelbase is the longitudinal distance between front and rear axle centers.
 * Track width is the lateral distance between left and right wheel centers.
 * The hydraulic track-width mechanism changes the lateral distance at runtime;
 * until closed-loop position feedback is wired into the command path, the
 * default value below is used by the remote and automatic command builders.
 */
#define ECU_VEHICLE_WHEELBASE_MIN_MM      (1200.0f)
#define ECU_VEHICLE_WHEELBASE_MM          (2200.0f)
#define ECU_VEHICLE_WHEELBASE_MAX_MM      (3200.0f)
#define ECU_VEHICLE_TRACK_WIDTH_MIN_MM    (1200.0f)
#define ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM (1800.0f)
#define ECU_VEHICLE_TRACK_WIDTH_MAX_MM    (2600.0f)
#define ECU_VEHICLE_MIN_TURN_RADIUS_MM    (1500.0f)
#define ECU_MOTION_SPIN_STEER_DEG         (45.0f)
#define ECU_REMOTE_MIN_HEIGHT_TARGET_MM   (0.0f)
#define ECU_REMOTE_MAX_HEIGHT_TARGET_MM   (400.0f)
#define ECU_REMOTE_MAX_HEIGHT_RATE_MM_S   (20.0f)
#define ECU_REMOTE_MAX_TRACK_RATE_MM_S    (20.0f)
#define ECU_RS485_BAUDRATE          (115200UL)
#define ECU_RS232_BAUDRATE          (115200UL)
#define ECU_SBUS_UART_RX_IDLE_BITS  (24U)
#define ECU_SBUS_UART_IRQ_PRIORITY  (2U)
#define ECU_RS485_UART_IRQ_PRIORITY (2U)

typedef enum {
    ECU_SBUS_CH_STEER = 0,
    ECU_SBUS_CH_CLEARANCE,
    ECU_SBUS_CH_THROTTLE,
    ECU_SBUS_CH_POWER,
    ECU_SBUS_CH_GEAR,
    ECU_SBUS_CH_HOME,
    ECU_SBUS_CH_AUTHORITY,
    ECU_SBUS_CH_LEFT_INDICATOR,
    ECU_SBUS_CH_HAZARD,
    ECU_SBUS_CH_HORN,
    ECU_SBUS_CH_HEADLIGHT,
    ECU_SBUS_CH_RIGHT_INDICATOR,
    ECU_SBUS_CH_ESTOP,
    ECU_SBUS_CH_TRACK,
    ECU_SBUS_CH_R1,
    ECU_SBUS_CH_R2,
    ECU_SBUS_CHANNEL_COUNT
} ecu_sbus_channel_role_t;

typedef struct {
    uint32_t magic;
    uint32_t command_sequence;
    bool high_voltage_enable;
    uint32_t request_time_ms;
} ecu_commissioning_control_t;

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
    float assist_torque_limit_nm[ECU_WHEEL_COUNT];
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
    uint32_t dio_managed_output_mask;
    bool dio_active_high;
    uint32_t hydraulic_track_extend_mask;
    uint32_t hydraulic_track_retract_mask;
    uint32_t hydraulic_lift_up_mask;
    uint32_t hydraulic_lift_down_mask;
    uint32_t hydraulic_managed_valve_mask;
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
    float drive_speed_kph_to_counts_per_sec;
    float steer_deg_to_counts;
    float lift_mm_to_counts;
} ecu_hardware_config_t;

const ecu_config_t *ecu_config_default(void);
const track_adjust_config_t *ecu_track_adjust_config_default(void);
const ecu_hardware_config_t *ecu_hardware_config_default(void);

#endif /* ECU_CONFIG_H */
