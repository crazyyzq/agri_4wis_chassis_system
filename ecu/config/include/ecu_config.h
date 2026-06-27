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

#define ECU_SBUS_RAW_LOW_MAX             (1050U)
#define ECU_SBUS_RAW_CENTER_MIN          (1400U)
#define ECU_SBUS_RAW_CENTER_MAX          (1600U)
#define ECU_SBUS_RAW_HIGH_MIN            (1950U)
#define ECU_SBUS_RAW_NEUTRAL             (1500U)
#define ECU_SBUS_RAW_THROTTLE_START      (1900U)

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

#define ECU_GUESS_CAN1_BITRATE           (500000UL)
#define ECU_GUESS_CAN2_BITRATE           (500000UL)
#define ECU_GUESS_CAN3_BITRATE           (500000UL)
#define ECU_GUESS_CAN4_BITRATE           (500000UL)

#define ECU_GUESS_CANOPEN_BMS_NODE_ID        (0x10U)
#define ECU_GUESS_CANOPEN_DCDC_48V_NODE_ID   (0x11U)
#define ECU_GUESS_CANOPEN_DCDC_12V_NODE_ID   (0x12U)
#define ECU_GUESS_CANOPEN_INVERTER_NODE_ID   (0x13U)
#define ECU_GUESS_CANOPEN_DRIVE_FL_NODE_ID   (0x01U)
#define ECU_GUESS_CANOPEN_DRIVE_FR_NODE_ID   (0x02U)
#define ECU_GUESS_CANOPEN_DRIVE_RL_NODE_ID   (0x03U)
#define ECU_GUESS_CANOPEN_DRIVE_RR_NODE_ID   (0x04U)
#define ECU_GUESS_CANOPEN_STEER_FL_NODE_ID   (0x05U)
#define ECU_GUESS_CANOPEN_STEER_FR_NODE_ID   (0x06U)
#define ECU_GUESS_CANOPEN_STEER_RL_NODE_ID   (0x07U)
#define ECU_GUESS_CANOPEN_STEER_RR_NODE_ID   (0x08U)
#define ECU_GUESS_CANOPEN_LIFT_FL_NODE_ID    (0x09U)
#define ECU_GUESS_CANOPEN_LIFT_FR_NODE_ID    (0x0AU)
#define ECU_GUESS_CANOPEN_LIFT_RL_NODE_ID    (0x0BU)
#define ECU_GUESS_CANOPEN_LIFT_RR_NODE_ID    (0x0CU)

#define ECU_GUESS_CANOPEN_TPDO1_BASE     (0x180UL)
#define ECU_GUESS_CANOPEN_RPDO1_BASE     (0x200UL)
#define ECU_GUESS_CANOPEN_HEARTBEAT_BASE (0x700UL)
#define ECU_GUESS_CANOPEN_TIMEOUT_MS     (100U)

#define ECU_GUESS_DIO_BRAKE_RELEASE_MASK       (1UL << 0)
#define ECU_GUESS_DIO_HYDRAULIC_ENABLE_MASK    (1UL << 1)
#define ECU_GUESS_DIO_HORN_MASK                (1UL << 2)
#define ECU_GUESS_DIO_HEADLIGHT_MASK           (1UL << 3)
#define ECU_GUESS_DIO_LEFT_INDICATOR_MASK      (1UL << 4)
#define ECU_GUESS_DIO_RIGHT_INDICATOR_MASK     (1UL << 5)
#define ECU_GUESS_DIO_MANAGED_OUTPUT_MASK      (ECU_GUESS_DIO_BRAKE_RELEASE_MASK | \
                                                ECU_GUESS_DIO_HYDRAULIC_ENABLE_MASK | \
                                                ECU_GUESS_DIO_HORN_MASK | \
                                                ECU_GUESS_DIO_HEADLIGHT_MASK | \
                                                ECU_GUESS_DIO_LEFT_INDICATOR_MASK | \
                                                ECU_GUESS_DIO_RIGHT_INDICATOR_MASK)

#define ECU_GUESS_HYD_VALVE_TRACK_EXTEND_MASK  (1UL << 8)
#define ECU_GUESS_HYD_VALVE_TRACK_RETRACT_MASK (1UL << 9)
#define ECU_GUESS_HYD_VALVE_LIFT_UP_MASK       (1UL << 10)
#define ECU_GUESS_HYD_VALVE_LIFT_DOWN_MASK     (1UL << 11)
#define ECU_GUESS_HYD_VALVE_MANAGED_MASK       (ECU_GUESS_HYD_VALVE_TRACK_EXTEND_MASK | \
                                                ECU_GUESS_HYD_VALVE_TRACK_RETRACT_MASK | \
                                                ECU_GUESS_HYD_VALVE_LIFT_UP_MASK | \
                                                ECU_GUESS_HYD_VALVE_LIFT_DOWN_MASK)

#define ECU_GUESS_ADC_CHANNEL_COUNT       (8U)
#define ECU_GUESS_ADC_RAW_MAX             (4095U)
#define ECU_GUESS_ADC_EXTERNAL_MV_MAX     (5000U)
#define ECU_GUESS_MODBUS_ADC_SLAVE_ID     (0x02U)
#define ECU_GUESS_MODBUS_WARNING_LIGHT_SLAVE_ID (0x03U)
#define ECU_GUESS_RS485_BAUDRATE          (115200UL)
#define ECU_GUESS_RS232_BAUDRATE          (115200UL)

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
    uint16_t low_max;
    uint16_t center_min;
    uint16_t center_max;
    uint16_t high_min;
    uint16_t neutral;
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
    uint32_t heartbeat_cob_id;
    uint32_t timeout_ms;
} ecu_canopen_node_config_t;

typedef struct {
    uint32_t can1_bitrate;
    uint32_t can2_bitrate;
    uint32_t can3_bitrate;
    uint32_t can4_bitrate;
    ecu_canopen_node_config_t bms_node;
    ecu_canopen_node_config_t dcdc_48v_node;
    ecu_canopen_node_config_t dcdc_12v_node;
    ecu_canopen_node_config_t inverter_node;
    ecu_canopen_node_config_t drive_nodes[ECU_WHEEL_COUNT];
    ecu_canopen_node_config_t steer_nodes[ECU_WHEEL_COUNT];
    ecu_canopen_node_config_t lift_nodes[ECU_WHEEL_COUNT];
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
    uint8_t modbus_warning_light_slave_id;
    uint32_t rs485_baudrate;
    uint32_t rs232_baudrate;
} ecu_hardware_config_t;

const ecu_config_t *ecu_config_default(void);
const track_adjust_config_t *ecu_track_adjust_config_default(void);
const ecu_hardware_config_t *ecu_hardware_config_default(void);

#endif /* ECU_CONFIG_H */
