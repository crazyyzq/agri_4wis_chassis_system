#include "power_can_protocol.h"

#include <string.h>

uint16_t power_can_read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

uint16_t power_can_read_u16_be(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

void power_can_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void power_can_write_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)((value >> 8) & 0xFFU);
    data[1] = (uint8_t)(value & 0xFFU);
}

static uint32_t power_can_read_u24_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16);
}

static void power_can_prepare_frame(ecu_can_frame_t *frame, uint32_t id)
{
    memset(frame, 0, sizeof(*frame));
    frame->id = id;
    frame->size = POWER_CAN_FRAME_SIZE;
    frame->extended = true;
    frame->remote = false;
}

static bool power_can_frame_matches(const ecu_can_frame_t *frame, uint32_t id)
{
    return frame != 0 &&
           frame->id == id &&
           frame->extended &&
           !frame->remote &&
           frame->size >= POWER_CAN_FRAME_SIZE;
}

static int32_t power_can_current_da_from_bms_raw(uint16_t raw)
{
    return (int32_t)raw - 20000;
}

static int32_t power_can_power_dkw_from_bms_raw(uint16_t raw)
{
    return ((int32_t)raw * 5) - 10000;
}

static uint16_t power_can_read_bms_sop_12(const uint8_t *data, uint32_t index)
{
    if (index == 0U) {
        return (uint16_t)data[0] | (((uint16_t)data[1] & 0x0FU) << 8);
    }
    return ((uint16_t)data[1] >> 4) | ((uint16_t)data[2] << 4);
}

bool power_can_build_bms_command(const power_can_bms_command_t *command,
                                 ecu_can_frame_t *out)
{
    if (command == 0 || out == 0) {
        return false;
    }

    power_can_prepare_frame(out, POWER_CAN_BMS_VCU_COMMAND_ID);
    out->data[2] = (uint8_t)command->contactor_request & 0x03U;
    out->data[7] = (uint8_t)command->key_status & 0x03U;
    return true;
}

bool power_can_build_dcdc48_control(const power_can_dcdc48_command_t *command,
                                    ecu_can_frame_t *out)
{
    if (command == 0 || out == 0) {
        return false;
    }

    power_can_prepare_frame(out, POWER_CAN_DCDC48_CONTROL_ID);
    out->data[0] = command->enable ? 0x01U : 0x00U;
    power_can_write_u16_le(&out->data[1], command->terminal_voltage_dv);
    power_can_write_u16_le(&out->data[3], command->max_current_da);
    return true;
}

bool power_can_build_dcdc12_control(const power_can_dcdc12_command_t *command,
                                    ecu_can_frame_t *out)
{
    if (command == 0 || out == 0) {
        return false;
    }

    power_can_prepare_frame(out, POWER_CAN_DCDC12_CONTROL_ID);
    out->data[2] = command->enable ? 0x10U : 0x00U;
    power_can_write_u16_le(&out->data[3], command->output_voltage_dv);
    power_can_write_u16_le(&out->data[5], command->output_current_da);
    return true;
}

bool power_can_build_dcac_control(const power_can_dcac_command_t *command,
                                  ecu_can_frame_t *out)
{
    if (command == 0 || out == 0) {
        return false;
    }

    power_can_prepare_frame(out, POWER_CAN_DCAC_CONTROL_ID);
    power_can_write_u16_be(&out->data[0], command->output_voltage_dv);
    out->data[4] = command->output_enable ? 0x00U : 0x01U;
    return true;
}

bool power_can_parse_bms_status(const ecu_can_frame_t *frame,
                                power_can_bms_status_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_BMS_STATUS_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    uint16_t voltage_raw = power_can_read_u16_le(&frame->data[1]);
    uint16_t current_raw = power_can_read_u16_le(&frame->data[3]);
    out->valid = voltage_raw != 0xFFFFU &&
                 current_raw != 0xFFFFU &&
                 frame->data[5] != 0xFFU;
    out->life = (frame->data[0] >> 4) & 0x0FU;
    out->pack_voltage_dv = voltage_raw;
    out->pack_current_da = power_can_current_da_from_bms_raw(current_raw);
    out->soc_half_percent = frame->data[5];
    out->mil_request = (frame->data[6] & 0x01U) != 0U;
    out->relay_status = frame->data[7];
    out->negative_contactor_closed = (frame->data[7] & (1U << 0)) != 0U;
    out->positive_contactor_closed = (frame->data[7] & (1U << 1)) != 0U;
    out->precharge_contactor_closed = (frame->data[7] & (1U << 2)) != 0U;
    out->heat_relay_closed = (frame->data[7] & (1U << 3)) != 0U;
    out->charge_relay_closed = (frame->data[7] & (1U << 4)) != 0U;
    return true;
}

bool power_can_parse_bms_cell_voltage(const ecu_can_frame_t *frame,
                                      power_can_bms_cell_voltage_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_BMS_CELL_VOLTAGE_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->min_mv = power_can_read_u16_le(&frame->data[0]);
    out->max_mv = power_can_read_u16_le(&frame->data[2]);
    out->min_index = frame->data[4];
    out->max_index = frame->data[5];
    out->valid = out->min_mv != 0xFFFFU &&
                 out->max_mv != 0xFFFFU &&
                 out->min_index != 0xFFU &&
                 out->max_index != 0xFFU;
    return true;
}

bool power_can_parse_bms_cell_temperature(const ecu_can_frame_t *frame,
                                          power_can_bms_cell_temperature_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_BMS_CELL_TEMPERATURE_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->min_c = (int16_t)frame->data[0] - 50;
    out->max_c = (int16_t)frame->data[1] - 50;
    out->min_index = frame->data[2];
    out->max_index = frame->data[3];
    out->valid = frame->data[0] != 0xFFU &&
                 frame->data[1] != 0xFFU &&
                 out->min_index != 0xFFU &&
                 out->max_index != 0xFFU;
    return true;
}

bool power_can_parse_bms_sof(const ecu_can_frame_t *frame,
                             power_can_bms_sof_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_BMS_SOF_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    uint16_t discharge = power_can_read_u16_le(&frame->data[0]);
    uint16_t charge = power_can_read_u16_le(&frame->data[2]);
    uint16_t discharge_10s = power_can_read_u16_le(&frame->data[4]);
    uint16_t charge_10s = power_can_read_u16_le(&frame->data[6]);
    out->valid = discharge != 0xFFFFU &&
                 charge != 0xFFFFU &&
                 discharge_10s != 0xFFFFU &&
                 charge_10s != 0xFFFFU;
    out->discharge_current_limit_da = power_can_current_da_from_bms_raw(discharge);
    out->charge_current_limit_da = power_can_current_da_from_bms_raw(charge);
    out->discharge_current_limit_10s_da = power_can_current_da_from_bms_raw(discharge_10s);
    out->charge_current_limit_10s_da = power_can_current_da_from_bms_raw(charge_10s);
    return true;
}

bool power_can_parse_bms_sop(const ecu_can_frame_t *frame,
                             power_can_bms_sop_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_BMS_SOP_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    uint16_t discharge = power_can_read_bms_sop_12(&frame->data[0], 0U);
    uint16_t charge = power_can_read_bms_sop_12(&frame->data[0], 1U);
    uint16_t discharge_10s = power_can_read_bms_sop_12(&frame->data[3], 0U);
    uint16_t charge_10s = power_can_read_bms_sop_12(&frame->data[3], 1U);
    uint16_t pack_power = power_can_read_u16_le(&frame->data[6]);
    out->valid = discharge != 0x0FFFU &&
                 charge != 0x0FFFU &&
                 discharge_10s != 0x0FFFU &&
                 charge_10s != 0x0FFFU &&
                 pack_power != 0xFFFFU;
    out->discharge_power_limit_dkw = power_can_power_dkw_from_bms_raw(discharge);
    out->charge_power_limit_dkw = power_can_power_dkw_from_bms_raw(charge);
    out->discharge_power_limit_10s_dkw = power_can_power_dkw_from_bms_raw(discharge_10s);
    out->charge_power_limit_10s_dkw = power_can_power_dkw_from_bms_raw(charge_10s);
    out->pack_power_dkw = power_can_power_dkw_from_bms_raw(pack_power);
    return true;
}

bool power_can_parse_bms_soh_statistics(const ecu_can_frame_t *frame,
                                        power_can_bms_soh_statistics_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_BMS_SOH_STATISTICS_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    uint32_t charge_energy = power_can_read_u24_le(&frame->data[2]);
    uint32_t input_ah = power_can_read_u24_le(&frame->data[5]);
    out->valid = frame->data[0] != 0xFFU &&
                 frame->data[1] != 0xFFU &&
                 charge_energy != 0xFFFFFFUL &&
                 input_ah != 0xFFFFFFUL;
    out->capacity_ah = (uint16_t)frame->data[0] * 2U;
    out->soh_percent = frame->data[1];
    out->charge_energy_kwh = charge_energy;
    out->input_ah = input_ah;
    return true;
}

bool power_can_parse_bms_error_status(const ecu_can_frame_t *frame,
                                      power_can_bms_error_status_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_BMS_ERROR_STATUS_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->level = frame->data[0] & 0x0FU;
    out->valid = out->level != 0x0FU;
    out->code = (((uint32_t)frame->data[1] & 0x0FU) << 16) |
                ((uint32_t)frame->data[2] << 8) |
                (uint32_t)frame->data[3];
    out->raw_code4 = frame->data[4];
    out->raw_code5 = frame->data[5];
    return true;
}

bool power_can_parse_dcdc48_status(const ecu_can_frame_t *frame,
                                   power_can_dcdc48_status_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_DCDC48_STATUS_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    uint16_t voltage_raw = power_can_read_u16_le(&frame->data[2]);
    uint16_t current_raw = power_can_read_u16_le(&frame->data[4]);
    out->valid = true;
    out->work_state = (frame->data[0] >> 1) & 0x03U;
    out->fault_level = (frame->data[0] >> 3) & 0x03U;
    out->system_state = (frame->data[0] >> 5) & 0x07U;
    out->temperature_c = (int16_t)frame->data[1] - 40;
    out->output_voltage_mv = (uint32_t)voltage_raw * 50U;
    out->output_current_ma = (uint32_t)current_raw * 50U;
    out->error_code = frame->data[6];
    out->version = frame->data[7];
    return true;
}

bool power_can_parse_dcdc12_status(const ecu_can_frame_t *frame,
                                   power_can_dcdc12_status_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_DCDC12_STATUS_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    uint16_t output_voltage_raw = power_can_read_u16_le(&frame->data[3]);
    uint16_t output_current_raw = power_can_read_u16_le(&frame->data[5]);
    out->valid = true;
    out->fault_code = frame->data[0];
    out->total_fault = (frame->data[0] & (1U << 7)) != 0U;
    out->short_circuit = (frame->data[0] & (1U << 5)) != 0U;
    out->over_temperature = (frame->data[0] & (1U << 4)) != 0U;
    out->can_interrupted = (frame->data[0] & (1U << 3)) != 0U;
    out->output_on = (frame->data[0] & (1U << 2)) != 0U;
    out->input_over_voltage = (frame->data[0] & (1U << 1)) != 0U;
    out->input_under_voltage = (frame->data[0] & (1U << 0)) != 0U;
    out->input_voltage_v = power_can_read_u16_le(&frame->data[1]);
    out->output_voltage_mv = (uint32_t)output_voltage_raw * 100U;
    out->output_current_ma = (uint32_t)output_current_raw * 100U;
    out->radiator_temperature_c = (int16_t)frame->data[7] - 50;
    return true;
}

bool power_can_parse_dcac_status(const ecu_can_frame_t *frame,
                                 power_can_dcac_status_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_DCAC_STATUS_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    uint16_t output_voltage_raw = power_can_read_u16_be(&frame->data[0]);
    uint16_t output_current_raw = power_can_read_u16_be(&frame->data[2]);
    out->valid = true;
    out->output_voltage_mv = (uint32_t)output_voltage_raw * 100U;
    out->output_current_ma = (uint32_t)output_current_raw * 100U;
    out->status_flags1 = frame->data[4];
    out->status_flags2 = frame->data[5];
    out->hardware_fault = (frame->data[4] & (1U << 0)) != 0U;
    out->over_temperature = (frame->data[4] & (1U << 1)) != 0U;
    out->input_voltage_fault = (frame->data[4] & (1U << 2)) != 0U;
    out->input_current_fault = (frame->data[4] & (1U << 3)) != 0U;
    out->output_voltage_fault = (frame->data[4] & (1U << 4)) != 0U;
    out->output_current_fault = (frame->data[4] & (1U << 5)) != 0U;
    out->output_stopped = (frame->data[4] & (1U << 6)) != 0U;
    out->communication_timeout = (frame->data[4] & (1U << 7)) != 0U;
    out->bus_voltage_fault = (frame->data[5] & (1U << 0)) != 0U;
    out->bus_current_fault = (frame->data[5] & (1U << 1)) != 0U;
    out->temperature_c = (int16_t)frame->data[7] - 40;
    return true;
}

bool power_can_parse_dcac_input_status(const ecu_can_frame_t *frame,
                                       power_can_dcac_input_status_t *out)
{
    if (!power_can_frame_matches(frame, POWER_CAN_DCAC_INPUT_STATUS_ID) || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->valid = true;
    out->input_voltage_mv = (uint32_t)power_can_read_u16_be(&frame->data[0]) * 100U;
    out->input_current_ma = (uint32_t)power_can_read_u16_be(&frame->data[2]) * 100U;
    return true;
}
