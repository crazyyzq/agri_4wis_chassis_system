#include "ecu_config.h"

/* Build the standard CANopen communication identifiers for one slave node.
 * The ECU keeps this as a macro because the default hardware table is static
 * const data and must be fully initialized at compile time. */
#define CANOPEN_NODE_CONFIG(node) \
    { \
        .node_id = (node), \
        .tpdo1_cob_id = ECU_CANOPEN_TPDO1_BASE + (node), \
        .rpdo1_cob_id = ECU_CANOPEN_RPDO1_BASE + (node), \
        .rpdo2_cob_id = ECU_CANOPEN_RPDO2_BASE + (node), \
        .rpdo3_cob_id = ECU_CANOPEN_RPDO3_BASE + (node), \
        .rpdo4_cob_id = ECU_CANOPEN_RPDO4_BASE + (node), \
        .rpdo5_cob_id = ECU_CANOPEN_RPDO5_BASE, \
        .heartbeat_cob_id = ECU_CANOPEN_HEARTBEAT_BASE + (node), \
        .timeout_ms = ECU_CANOPEN_TIMEOUT_MS \
    }

/* Runtime-independent defaults consumed by the remote, control, and task
 * layers.  Timing values are kept here so all state machines share the same
 * debounce, timeout, and diagnostic cadence. */
static const ecu_config_t s_default_config = {
    .discrete_debounce_ms = REMOTE_DISCRETE_DEBOUNCE_MS,
    .link_qualify_ms = REMOTE_LINK_QUALIFY_MS,
    .neutral_qualify_ms = REMOTE_NEUTRAL_QUALIFY_MS,
    .failsafe_timeout_ms = REMOTE_FAILSAFE_TIMEOUT_MS,
    .domain_event_guard_ms = REMOTE_DOMAIN_EVENT_GUARD_MS,
    .power_long_press_ms = REMOTE_POWER_LONG_PRESS_MS,
    .mode_request_ttl_ms = REMOTE_EVENT_MODE_REQUEST_TTL_MS,
    .power_request_ttl_ms = REMOTE_EVENT_POWER_REQUEST_TTL_MS,
    .estop_reset_ttl_ms = REMOTE_EVENT_ESTOP_RESET_TTL_MS,
    .light_request_ttl_ms = REMOTE_EVENT_LIGHT_REQUEST_TTL_MS,
    .sbus_thresholds = {
        .low_max = ECU_SBUS_PPM_LOW_MAX,
        .center_min = ECU_SBUS_PPM_CENTER_MIN,
        .center_max = ECU_SBUS_PPM_CENTER_MAX,
        .high_min = ECU_SBUS_PPM_HIGH_MIN,
        .stick_min = ECU_SBUS_PPM_LOW,
        .stick_neutral = ECU_SBUS_PPM_CENTER,
        .stick_max = ECU_SBUS_PPM_HIGH,
        .neutral = ECU_SBUS_PPM_CENTER,
        .throttle_min = ECU_SBUS_PPM_LOW,
        .throttle_max = ECU_SBUS_PPM_HIGH,
        .throttle_start = ECU_SBUS_PPM_THROTTLE_START
    },
    .cpu0_task_period_ms = {
        ECU_CPU0_SAFETY_PERIOD_MS,
        ECU_CPU0_CAN2_MOTION_PERIOD_MS,
        ECU_CPU0_REMOTE_PERIOD_MS,
        ECU_CPU0_CONTROL_PERIOD_MS,
        ECU_CPU0_POWER_PERIOD_MS,
        ECU_CPU0_LIFT_HYD_PERIOD_MS,
        ECU_CPU0_IO_PERIOD_MS,
        ECU_CPU0_DIAG_PERIOD_MS
    },
    .cpu1_service_period_ms = ECU_CPU1_SERVICE_PERIOD_MS
};

/* Track-width adjustment helper gains in vehicle leg order:
 *   leg 1 front-right, leg 2 front-left, leg 3 rear-left, leg 4 rear-right.
 * Steering targets point the wheels into the adjustment posture.  Assist torque
 * signs are positive on the right side and negative on the left side, matching
 * the confirmed vehicle geometry. */
static const track_adjust_config_t s_track_adjust_config = {
    .steer_target_deg = { 90.0f, -90.0f, -90.0f, 90.0f },
    .assist_torque_sign = { 1.0f, -1.0f, -1.0f, 1.0f },
    .assist_torque_limit_nm = { 8.0f, 8.0f, 8.0f, 8.0f },
    .assist_wheel_speed_limit_rpm = { 12.0f, 12.0f, 12.0f, 12.0f }
};

/* Hardware binding table for CPU0.
 *
 * The order of each node array is the vehicle leg order used throughout the
 * control stack:
 *   - Leg 1: front-right
 *   - Leg 2: front-left
 *   - Leg 3: rear-left
 *   - Leg 4: rear-right
 *
 * This keeps motion-control code independent from the actual CANopen node
 * numbers while making the physical position of each array slot explicit. */
static const ecu_hardware_config_t s_hardware_config = {
    .can1_bitrate = ECU_CAN1_POWER_BITRATE,
    .can2_bitrate = ECU_CAN2_MOTION_BITRATE,
    .can3_bitrate = ECU_CAN3_LIFT_HYDRAULIC_BITRATE,
    .can4_bitrate = ECU_CAN4_AUXILIARY_BITRATE,
    .power_protocol = ECU_POWER_PROTOCOL_SUPPLIER_CAN,

    /* CAN2 drive motors: node 1..4. */
    .drive_nodes = {
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LEG1_DRIVE_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LEG2_DRIVE_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LEG3_DRIVE_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LEG4_DRIVE_NODE_ID)
    },

    /* CAN2 steering motors: node 5..8. */
    .steer_nodes = {
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LEG1_STEER_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LEG2_STEER_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LEG3_STEER_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LEG4_STEER_NODE_ID)
    },

    /* CAN3 lift motors: node 9, 11, 12 and 10 in vehicle leg order.  On BC2
     * drives the B axis is addressed as its own CANopen node, not as extra
     * bits on the A-axis node. */
    .lift_nodes = {
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LIFT_LEG1_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LIFT_LEG2_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LIFT_LEG3_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_CANOPEN_LIFT_LEG4_NODE_ID)
    },

    /* CAN3 hydraulic-station motor: node 13, commanded in velocity mode. */
    .hydraulic_pump_node = CANOPEN_NODE_CONFIG(ECU_CANOPEN_HYDRAULIC_PUMP_NODE_ID),
    .dio_brake_release_mask = ECU_DIO_BRAKE_RELEASE_MASK,
    .dio_hydraulic_enable_mask = ECU_DIO_HYDRAULIC_ENABLE_MASK,
    .dio_horn_mask = ECU_DIO_HORN_MASK,
    .dio_headlight_mask = ECU_DIO_HEADLIGHT_MASK,
    .dio_left_indicator_mask = ECU_DIO_LEFT_INDICATOR_MASK,
    .dio_right_indicator_mask = ECU_DIO_RIGHT_INDICATOR_MASK,
    .dio_managed_output_mask = ECU_DIO_MANAGED_OUTPUT_MASK | ECU_HYD_VALVE_MANAGED_MASK,
    .dio_active_high = true,
    .hydraulic_track_extend_mask = ECU_HYD_VALVE_TRACK_EXTEND_MASK,
    .hydraulic_track_retract_mask = ECU_HYD_VALVE_TRACK_RETRACT_MASK,
    .hydraulic_lift_up_mask = ECU_HYD_VALVE_LIFT_UP_MASK,
    .hydraulic_lift_down_mask = ECU_HYD_VALVE_LIFT_DOWN_MASK,
    .hydraulic_managed_valve_mask = ECU_HYD_VALVE_MANAGED_MASK,
    .adc_channel_count = ECU_ADC_CHANNEL_COUNT,
    .adc_raw_max = ECU_ADC_RAW_MAX,
    .adc_external_mv_max = ECU_ADC_EXTERNAL_MV_MAX,
    .modbus_adc_slave_id = ECU_MODBUS_ADC_SLAVE_ID,
    .modbus_adc_baudrate = ECU_MODBUS_ADC_BAUDRATE,
    .modbus_adc_start_register = ECU_MODBUS_ADC_START_REGISTER,
    .modbus_adc_register_count = ECU_MODBUS_ADC_REGISTER_COUNT,
    .modbus_adc_raw_max = ECU_MODBUS_ADC_RAW_MAX,
    .modbus_adc_poll_period_ms = ECU_MODBUS_ADC_POLL_PERIOD_MS,
    .modbus_adc_response_timeout_ms = ECU_MODBUS_ADC_RESPONSE_TIMEOUT_MS,
    .modbus_warning_light_slave_id = ECU_MODBUS_WARNING_LIGHT_SLAVE_ID,
    .modbus_warning_light_register = ECU_MODBUS_WARNING_LIGHT_REGISTER,
    .modbus_warning_light_baudrate = ECU_MODBUS_WARNING_LIGHT_BAUDRATE,
    .modbus_warning_light_request_period_ms = ECU_MODBUS_WARNING_LIGHT_REQUEST_PERIOD_MS,
    .modbus_warning_light_response_timeout_ms = ECU_MODBUS_WARNING_LIGHT_RESPONSE_TIMEOUT_MS,
    .rs485_baudrate = ECU_RS485_BAUDRATE,
    .rs232_baudrate = ECU_RS232_BAUDRATE,
    .drive_speed_kph_to_counts_per_sec = ECU_DRIVE_SPEED_KPH_TO_COUNTS_PER_SEC,
    .steer_deg_to_counts = ECU_STEER_DEG_TO_COUNTS,
    .lift_mm_to_counts = ECU_LIFT_MM_TO_COUNTS
};

const ecu_config_t *ecu_config_default(void)
{
    return &s_default_config;
}

const track_adjust_config_t *ecu_track_adjust_config_default(void)
{
    return &s_track_adjust_config;
}

const ecu_hardware_config_t *ecu_hardware_config_default(void)
{
    return &s_hardware_config;
}
