#include "ecu_config.h"

#define CANOPEN_NODE_CONFIG(node) \
    { \
        .node_id = (node), \
        .tpdo1_cob_id = ECU_GUESS_CANOPEN_TPDO1_BASE + (node), \
        .rpdo1_cob_id = ECU_GUESS_CANOPEN_RPDO1_BASE + (node), \
        .heartbeat_cob_id = ECU_GUESS_CANOPEN_HEARTBEAT_BASE + (node), \
        .timeout_ms = ECU_GUESS_CANOPEN_TIMEOUT_MS \
    }

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
        .low_max = ECU_SBUS_RAW_LOW_MAX,
        .center_min = ECU_SBUS_RAW_CENTER_MIN,
        .center_max = ECU_SBUS_RAW_CENTER_MAX,
        .high_min = ECU_SBUS_RAW_HIGH_MIN,
        .neutral = ECU_SBUS_RAW_NEUTRAL,
        .throttle_start = ECU_SBUS_RAW_THROTTLE_START
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

static const track_adjust_config_t s_track_adjust_config = {
    .steer_target_deg = { 90.0f, -90.0f, -90.0f, 90.0f },
    .assist_torque_sign = { 1.0f, -1.0f, -1.0f, 1.0f },
    .assist_torque_limit_nm = { 8.0f, 8.0f, 8.0f, 8.0f },
    .assist_wheel_speed_limit_rpm = { 12.0f, 12.0f, 12.0f, 12.0f }
};

static const ecu_hardware_config_t s_hardware_config = {
    .can1_bitrate = ECU_GUESS_CAN1_BITRATE,
    .can2_bitrate = ECU_GUESS_CAN2_BITRATE,
    .can3_bitrate = ECU_GUESS_CAN3_BITRATE,
    .can4_bitrate = ECU_GUESS_CAN4_BITRATE,
    .bms_node = CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_BMS_NODE_ID),
    .dcdc_48v_node = CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_DCDC_48V_NODE_ID),
    .dcdc_12v_node = CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_DCDC_12V_NODE_ID),
    .inverter_node = CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_INVERTER_NODE_ID),
    .drive_nodes = {
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_DRIVE_FL_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_DRIVE_FR_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_DRIVE_RL_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_DRIVE_RR_NODE_ID)
    },
    .steer_nodes = {
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_STEER_FL_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_STEER_FR_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_STEER_RL_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_STEER_RR_NODE_ID)
    },
    .lift_nodes = {
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_LIFT_FL_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_LIFT_FR_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_LIFT_RL_NODE_ID),
        CANOPEN_NODE_CONFIG(ECU_GUESS_CANOPEN_LIFT_RR_NODE_ID)
    },
    .dio_brake_release_mask = ECU_GUESS_DIO_BRAKE_RELEASE_MASK,
    .dio_hydraulic_enable_mask = ECU_GUESS_DIO_HYDRAULIC_ENABLE_MASK,
    .dio_horn_mask = ECU_GUESS_DIO_HORN_MASK,
    .dio_headlight_mask = ECU_GUESS_DIO_HEADLIGHT_MASK,
    .dio_left_indicator_mask = ECU_GUESS_DIO_LEFT_INDICATOR_MASK,
    .dio_right_indicator_mask = ECU_GUESS_DIO_RIGHT_INDICATOR_MASK,
    .dio_managed_output_mask = ECU_GUESS_DIO_MANAGED_OUTPUT_MASK | ECU_GUESS_HYD_VALVE_MANAGED_MASK,
    .dio_active_high = true,
    .hydraulic_track_extend_mask = ECU_GUESS_HYD_VALVE_TRACK_EXTEND_MASK,
    .hydraulic_track_retract_mask = ECU_GUESS_HYD_VALVE_TRACK_RETRACT_MASK,
    .hydraulic_lift_up_mask = ECU_GUESS_HYD_VALVE_LIFT_UP_MASK,
    .hydraulic_lift_down_mask = ECU_GUESS_HYD_VALVE_LIFT_DOWN_MASK,
    .hydraulic_managed_valve_mask = ECU_GUESS_HYD_VALVE_MANAGED_MASK,
    .adc_channel_count = ECU_GUESS_ADC_CHANNEL_COUNT,
    .adc_raw_max = ECU_GUESS_ADC_RAW_MAX,
    .adc_external_mv_max = ECU_GUESS_ADC_EXTERNAL_MV_MAX,
    .modbus_adc_slave_id = ECU_GUESS_MODBUS_ADC_SLAVE_ID,
    .modbus_warning_light_slave_id = ECU_GUESS_MODBUS_WARNING_LIGHT_SLAVE_ID,
    .rs485_baudrate = ECU_GUESS_RS485_BAUDRATE,
    .rs232_baudrate = ECU_GUESS_RS232_BAUDRATE
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
