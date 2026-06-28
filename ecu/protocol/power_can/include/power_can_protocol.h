#ifndef POWER_CAN_PROTOCOL_H
#define POWER_CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "can_bus_service.h"

#define POWER_CAN_BMS_VCU_COMMAND_ID        (0x1410F41EUL)
#define POWER_CAN_BMS_STATUS_ID             (0x18111EF4UL)
#define POWER_CAN_BMS_CELL_VOLTAGE_ID       (0x18141EF4UL)
#define POWER_CAN_BMS_CELL_TEMPERATURE_ID   (0x18151EF4UL)
#define POWER_CAN_BMS_SOF_ID                (0x18161EF4UL)
#define POWER_CAN_BMS_SOP_ID                (0x18171EF4UL)
#define POWER_CAN_BMS_SOH_STATISTICS_ID     (0x18211EF4UL)
#define POWER_CAN_BMS_ERROR_STATUS_ID       (0x18221EF4UL)
#define POWER_CAN_DCDC48_CONTROL_ID         (0x10262B27UL)
#define POWER_CAN_DCDC48_STATUS_ID          (0x18F8622BUL)
#define POWER_CAN_DCDC12_CONTROL_ID         (0x18EF3010UL)
#define POWER_CAN_DCDC12_STATUS_ID          (0x18FF3247UL)
#define POWER_CAN_DCAC_CONTROL_ID           (0x1806B6A5UL)
#define POWER_CAN_DCAC_STATUS_ID            (0x18FF50B6UL)
#define POWER_CAN_DCAC_INPUT_STATUS_ID      (0x18FE50B6UL)

#define POWER_CAN_FRAME_SIZE                (8U)

typedef enum {
    POWER_CAN_BMS_CONTACTOR_DISCONNECT = 0,
    POWER_CAN_BMS_CONTACTOR_CONNECT = 1,
    POWER_CAN_BMS_CONTACTOR_EMERGENCY_DISCONNECT = 2
} power_can_bms_contactor_request_t;

typedef enum {
    POWER_CAN_BMS_KEY_LOCKED = 0,
    POWER_CAN_BMS_KEY_ACC = 1,
    POWER_CAN_BMS_KEY_ON = 2,
    POWER_CAN_BMS_KEY_START = 3
} power_can_bms_key_status_t;

typedef struct {
    power_can_bms_contactor_request_t contactor_request;
    power_can_bms_key_status_t key_status;
} power_can_bms_command_t;

typedef struct {
    bool enable;
    uint16_t terminal_voltage_dv;
    uint16_t max_current_da;
} power_can_dcdc48_command_t;

typedef struct {
    bool enable;
    uint16_t output_voltage_dv;
    uint16_t output_current_da;
} power_can_dcdc12_command_t;

typedef struct {
    bool output_enable;
    uint16_t output_voltage_dv;
} power_can_dcac_command_t;

typedef struct {
    bool valid;
    uint8_t life;
    uint16_t pack_voltage_dv;
    int32_t pack_current_da;
    uint8_t soc_half_percent;
    bool mil_request;
    uint8_t relay_status;
    bool negative_contactor_closed;
    bool positive_contactor_closed;
    bool precharge_contactor_closed;
    bool heat_relay_closed;
    bool charge_relay_closed;
} power_can_bms_status_t;

typedef struct {
    bool valid;
    uint16_t min_mv;
    uint16_t max_mv;
    uint8_t min_index;
    uint8_t max_index;
} power_can_bms_cell_voltage_t;

typedef struct {
    bool valid;
    int16_t min_c;
    int16_t max_c;
    uint8_t min_index;
    uint8_t max_index;
} power_can_bms_cell_temperature_t;

typedef struct {
    bool valid;
    int32_t discharge_current_limit_da;
    int32_t charge_current_limit_da;
    int32_t discharge_current_limit_10s_da;
    int32_t charge_current_limit_10s_da;
} power_can_bms_sof_t;

typedef struct {
    bool valid;
    int32_t discharge_power_limit_dkw;
    int32_t charge_power_limit_dkw;
    int32_t discharge_power_limit_10s_dkw;
    int32_t charge_power_limit_10s_dkw;
    int32_t pack_power_dkw;
} power_can_bms_sop_t;

typedef struct {
    bool valid;
    uint16_t capacity_ah;
    uint8_t soh_percent;
    uint32_t charge_energy_kwh;
    uint32_t input_ah;
} power_can_bms_soh_statistics_t;

typedef struct {
    bool valid;
    uint8_t level;
    uint32_t code;
    uint8_t raw_code4;
    uint8_t raw_code5;
} power_can_bms_error_status_t;

typedef struct {
    bool valid;
    uint8_t work_state;
    uint8_t fault_level;
    uint8_t system_state;
    int16_t temperature_c;
    uint32_t output_voltage_mv;
    uint32_t output_current_ma;
    uint8_t error_code;
    uint8_t version;
} power_can_dcdc48_status_t;

typedef struct {
    bool valid;
    uint8_t fault_code;
    bool total_fault;
    bool short_circuit;
    bool over_temperature;
    bool can_interrupted;
    bool output_on;
    bool input_over_voltage;
    bool input_under_voltage;
    uint16_t input_voltage_v;
    uint32_t output_voltage_mv;
    uint32_t output_current_ma;
    int16_t radiator_temperature_c;
} power_can_dcdc12_status_t;

typedef struct {
    bool valid;
    uint32_t output_voltage_mv;
    uint32_t output_current_ma;
    uint8_t status_flags1;
    uint8_t status_flags2;
    bool hardware_fault;
    bool over_temperature;
    bool input_voltage_fault;
    bool input_current_fault;
    bool output_voltage_fault;
    bool output_current_fault;
    bool output_stopped;
    bool communication_timeout;
    bool bus_voltage_fault;
    bool bus_current_fault;
    int16_t temperature_c;
} power_can_dcac_status_t;

typedef struct {
    bool valid;
    uint32_t input_voltage_mv;
    uint32_t input_current_ma;
} power_can_dcac_input_status_t;

uint16_t power_can_read_u16_le(const uint8_t *data);
uint16_t power_can_read_u16_be(const uint8_t *data);
void power_can_write_u16_le(uint8_t *data, uint16_t value);
void power_can_write_u16_be(uint8_t *data, uint16_t value);

bool power_can_build_bms_command(const power_can_bms_command_t *command,
                                 ecu_can_frame_t *out);
bool power_can_build_dcdc48_control(const power_can_dcdc48_command_t *command,
                                    ecu_can_frame_t *out);
bool power_can_build_dcdc12_control(const power_can_dcdc12_command_t *command,
                                    ecu_can_frame_t *out);
bool power_can_build_dcac_control(const power_can_dcac_command_t *command,
                                  ecu_can_frame_t *out);

bool power_can_parse_bms_status(const ecu_can_frame_t *frame,
                                power_can_bms_status_t *out);
bool power_can_parse_bms_cell_voltage(const ecu_can_frame_t *frame,
                                      power_can_bms_cell_voltage_t *out);
bool power_can_parse_bms_cell_temperature(const ecu_can_frame_t *frame,
                                          power_can_bms_cell_temperature_t *out);
bool power_can_parse_bms_sof(const ecu_can_frame_t *frame,
                             power_can_bms_sof_t *out);
bool power_can_parse_bms_sop(const ecu_can_frame_t *frame,
                             power_can_bms_sop_t *out);
bool power_can_parse_bms_soh_statistics(const ecu_can_frame_t *frame,
                                        power_can_bms_soh_statistics_t *out);
bool power_can_parse_bms_error_status(const ecu_can_frame_t *frame,
                                      power_can_bms_error_status_t *out);
bool power_can_parse_dcdc48_status(const ecu_can_frame_t *frame,
                                   power_can_dcdc48_status_t *out);
bool power_can_parse_dcdc12_status(const ecu_can_frame_t *frame,
                                   power_can_dcdc12_status_t *out);
bool power_can_parse_dcac_status(const ecu_can_frame_t *frame,
                                 power_can_dcac_status_t *out);
bool power_can_parse_dcac_input_status(const ecu_can_frame_t *frame,
                                       power_can_dcac_input_status_t *out);

#endif /* POWER_CAN_PROTOCOL_H */
