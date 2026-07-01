#include <string.h>

#include "power_device.h"

#define POWER_DEVICE_TIMESTAMP_INVALID (0xFFFFFFFFUL)

static bool power_device_elapsed(uint32_t now_ms, uint32_t last_ms, uint32_t period_ms)
{
    return last_ms == POWER_DEVICE_TIMESTAMP_INVALID ||
           (uint32_t)(now_ms - last_ms) >= period_ms;
}

static bool power_device_recent(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms)
{
    return last_ms != POWER_DEVICE_TIMESTAMP_INVALID &&
           (uint32_t)(now_ms - last_ms) <= timeout_ms;
}

static void power_device_init_timestamps(power_device_state_t *state)
{
    state->last_bms_command_ms = POWER_DEVICE_TIMESTAMP_INVALID;
    state->last_dcdc48_command_ms = POWER_DEVICE_TIMESTAMP_INVALID;
    state->last_dcdc12_command_ms = POWER_DEVICE_TIMESTAMP_INVALID;
    state->last_dcac_command_ms = POWER_DEVICE_TIMESTAMP_INVALID;
    state->snapshot.bms_last_rx_ms = POWER_DEVICE_TIMESTAMP_INVALID;
    state->snapshot.dcdc48_last_rx_ms = POWER_DEVICE_TIMESTAMP_INVALID;
    state->snapshot.dcdc12_last_rx_ms = POWER_DEVICE_TIMESTAMP_INVALID;
    state->snapshot.dcac_last_rx_ms = POWER_DEVICE_TIMESTAMP_INVALID;
    state->snapshot.last_rx_ms = POWER_DEVICE_TIMESTAMP_INVALID;
    state->snapshot.last_tx_ms = POWER_DEVICE_TIMESTAMP_INVALID;
}

static bool power_device_send_frame(power_device_state_t *state,
                                    can_bus_service_t *can1,
                                    const ecu_can_frame_t *frame,
                                    uint32_t now_ms)
{
    if (!can_bus_service_send_frame(can1, frame)) {
        return false;
    }

    state->snapshot.last_tx_frame = *frame;
    state->snapshot.last_tx_ms = now_ms;
    state->snapshot.command_tx_count++;
    return true;
}

static void power_device_refresh_online(power_device_state_t *state,
                                        const can_bus_service_t *can1,
                                        uint32_t now_ms)
{
    power_device_snapshot_t *snapshot = &state->snapshot;
    /* CAN service error_count is cumulative diagnostic information.  Do not
     * use it as an online latch, otherwise one transient TX/RX fault keeps the
     * recovered power bus offline forever.  Current bus health is represented
     * by can1->online; device-level online flags below are based on fresh
     * protocol frames and their own timeouts.
     */
    snapshot->can1_online = can1 != 0 && can1->online;
    snapshot->bms_online =
        ECU_POWER_ENABLE_BMS != 0 &&
        power_device_recent(now_ms,
                            snapshot->bms_last_rx_ms,
                            ECU_POWER_BMS_STATUS_TIMEOUT_MS);
    snapshot->dcdc48_online =
        ECU_POWER_ENABLE_DCDC48 != 0 &&
        power_device_recent(now_ms,
                            snapshot->dcdc48_last_rx_ms,
                            ECU_POWER_DCDC48_STATUS_TIMEOUT_MS);
    snapshot->dcdc12_online =
        ECU_POWER_ENABLE_DCDC12 != 0 &&
        power_device_recent(now_ms,
                            snapshot->dcdc12_last_rx_ms,
                            ECU_POWER_DCDC12_STATUS_TIMEOUT_MS);
    snapshot->dcac_online =
        ECU_POWER_ENABLE_DCAC != 0 &&
        power_device_recent(now_ms,
                            snapshot->dcac_last_rx_ms,
                            ECU_POWER_DCAC_STATUS_TIMEOUT_MS);

    bool dcdc48_ok = ECU_POWER_ENABLE_DCDC48 == 0 ||
                     (snapshot->dcdc48_online &&
                      snapshot->dcdc48.valid &&
                      snapshot->dcdc48.error_code == 0U);
    bool dcdc12_ok = ECU_POWER_ENABLE_DCDC12 == 0 ||
                     (snapshot->dcdc12_online &&
                      snapshot->dcdc12.valid &&
                      !snapshot->dcdc12.total_fault &&
                      !snapshot->dcdc12.can_interrupted);
    snapshot->low_voltage_ok = dcdc48_ok && dcdc12_ok;

    bool bms_fault_free = snapshot->bms_error.valid &&
                          snapshot->bms_error.level == 0U;
    bool hv_feedback_ok = !snapshot->high_voltage_requested ||
                          (snapshot->bms.positive_contactor_closed &&
                           snapshot->bms.negative_contactor_closed);
    snapshot->power_ready = snapshot->bms_online &&
                            snapshot->bms.valid &&
                            bms_fault_free &&
                            hv_feedback_ok;
}

static void power_device_process_bms_frame(power_device_state_t *state,
                                           const ecu_can_frame_t *frame,
                                           uint32_t now_ms)
{
    power_device_snapshot_t *snapshot = &state->snapshot;
    bool matched = false;

    switch (frame->id) {
    case POWER_CAN_BMS_STATUS_ID:
        matched = power_can_parse_bms_status(frame, &snapshot->bms);
        break;
    case POWER_CAN_BMS_CELL_VOLTAGE_ID:
        matched = power_can_parse_bms_cell_voltage(frame, &snapshot->bms_cell_voltage);
        break;
    case POWER_CAN_BMS_CELL_TEMPERATURE_ID:
        matched = power_can_parse_bms_cell_temperature(frame, &snapshot->bms_cell_temperature);
        break;
    case POWER_CAN_BMS_SOF_ID:
        matched = power_can_parse_bms_sof(frame, &snapshot->bms_sof);
        break;
    case POWER_CAN_BMS_SOP_ID:
        matched = power_can_parse_bms_sop(frame, &snapshot->bms_sop);
        break;
    case POWER_CAN_BMS_SOH_STATISTICS_ID:
        matched = power_can_parse_bms_soh_statistics(frame, &snapshot->bms_soh);
        break;
    case POWER_CAN_BMS_ERROR_STATUS_ID:
        matched = power_can_parse_bms_error_status(frame, &snapshot->bms_error);
        break;
    default:
        break;
    }

    if (matched) {
        snapshot->bms_last_rx_ms = now_ms;
    }
}

static void power_device_process_frame(power_device_state_t *state,
                                       const ecu_can_frame_t *frame,
                                       uint32_t now_ms)
{
    power_device_snapshot_t *snapshot = &state->snapshot;
    snapshot->last_rx_frame = *frame;
    snapshot->last_rx_ms = now_ms;

    power_device_process_bms_frame(state, frame, now_ms);
    if (power_can_parse_dcdc48_status(frame, &snapshot->dcdc48)) {
        snapshot->dcdc48_last_rx_ms = now_ms;
    }
    if (power_can_parse_dcdc12_status(frame, &snapshot->dcdc12)) {
        snapshot->dcdc12_last_rx_ms = now_ms;
    }
    if (power_can_parse_dcac_status(frame, &snapshot->dcac)) {
        snapshot->dcac_last_rx_ms = now_ms;
    }
    if (power_can_parse_dcac_input_status(frame, &snapshot->dcac_input)) {
        snapshot->dcac_last_rx_ms = now_ms;
    }
}

void power_device_init(power_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
        state->snapshot.last_result = ECU_DEVICE_APPLY_OK;
        power_device_init_timestamps(state);
    }
}

void power_device_process_rx(power_device_state_t *state,
                             can_bus_service_t *can1,
                             const ecu_hardware_config_t *config,
                             uint32_t now_ms)
{
    if (state == 0 || can1 == 0 || config == 0) {
        return;
    }
    if (config->power_protocol != ECU_POWER_PROTOCOL_SUPPLIER_CAN) {
        power_device_refresh_online(state, can1, now_ms);
        return;
    }

    ecu_can_frame_t frame;
    while (can_bus_service_pop_rx_frame(can1, &frame)) {
        power_device_process_frame(state, &frame, now_ms);
    }
    power_device_refresh_online(state, can1, now_ms);
}

static bool power_device_send_bms_if_due(power_device_state_t *state,
                                         can_bus_service_t *can1,
                                         bool high_voltage_enable,
                                         uint32_t now_ms)
{
    if (ECU_POWER_ENABLE_BMS == 0 ||
        !power_device_elapsed(now_ms,
                              state->last_bms_command_ms,
                              ECU_POWER_BMS_COMMAND_PERIOD_MS)) {
        return true;
    }

    power_can_bms_command_t command = {
        .contactor_request = high_voltage_enable ?
                             POWER_CAN_BMS_CONTACTOR_CONNECT :
                             POWER_CAN_BMS_CONTACTOR_DISCONNECT,
        .key_status = POWER_CAN_BMS_KEY_ON
    };
    ecu_can_frame_t frame;
    if (!power_can_build_bms_command(&command, &frame) ||
        !power_device_send_frame(state, can1, &frame, now_ms)) {
        return false;
    }
    state->last_bms_command_ms = now_ms;
    return true;
}

static bool power_device_send_dcdc48_if_due(power_device_state_t *state,
                                            can_bus_service_t *can1,
                                            bool high_voltage_enable,
                                            uint32_t now_ms)
{
    if (ECU_POWER_ENABLE_DCDC48 == 0 ||
        !power_device_elapsed(now_ms,
                              state->last_dcdc48_command_ms,
                              ECU_POWER_DCDC48_COMMAND_PERIOD_MS)) {
        return true;
    }

    power_can_dcdc48_command_t command = {
        .enable = high_voltage_enable,
        .terminal_voltage_dv = ECU_DCDC48_DEFAULT_TERMINAL_VOLTAGE_DV,
        .max_current_da = ECU_DCDC48_DEFAULT_CURRENT_DA
    };
    ecu_can_frame_t frame;
    if (!power_can_build_dcdc48_control(&command, &frame) ||
        !power_device_send_frame(state, can1, &frame, now_ms)) {
        return false;
    }
    state->last_dcdc48_command_ms = now_ms;
    return true;
}

static bool power_device_send_dcdc12_if_due(power_device_state_t *state,
                                            can_bus_service_t *can1,
                                            bool high_voltage_enable,
                                            uint32_t now_ms)
{
    if (ECU_POWER_ENABLE_DCDC12 == 0 ||
        !power_device_elapsed(now_ms,
                              state->last_dcdc12_command_ms,
                              ECU_POWER_DCDC12_COMMAND_PERIOD_MS)) {
        return true;
    }

    power_can_dcdc12_command_t command = {
        .enable = high_voltage_enable,
        .output_voltage_dv = ECU_DCDC12_DEFAULT_OUTPUT_VOLTAGE_DV,
        .output_current_da = ECU_DCDC12_DEFAULT_OUTPUT_CURRENT_DA
    };
    ecu_can_frame_t frame;
    if (!power_can_build_dcdc12_control(&command, &frame) ||
        !power_device_send_frame(state, can1, &frame, now_ms)) {
        return false;
    }
    state->last_dcdc12_command_ms = now_ms;
    return true;
}

static bool power_device_send_dcac_if_due(power_device_state_t *state,
                                          can_bus_service_t *can1,
                                          bool high_voltage_enable,
                                          uint32_t now_ms)
{
    if (ECU_POWER_ENABLE_DCAC == 0 ||
        !power_device_elapsed(now_ms,
                              state->last_dcac_command_ms,
                              ECU_POWER_DCAC_COMMAND_PERIOD_MS)) {
        return true;
    }

    power_can_dcac_command_t command = {
        .output_enable = high_voltage_enable,
        .output_voltage_dv = ECU_DCAC_DEFAULT_OUTPUT_VOLTAGE_DV
    };
    ecu_can_frame_t frame;
    if (!power_can_build_dcac_control(&command, &frame) ||
        !power_device_send_frame(state, can1, &frame, now_ms)) {
        return false;
    }
    state->last_dcac_command_ms = now_ms;
    return true;
}

ecu_device_apply_result_t power_device_apply(power_device_state_t *state,
                                             can_bus_service_t *can1,
                                             const ecu_hardware_config_t *config,
                                             bool high_voltage_enable,
                                             uint32_t now_ms)
{
    if (state == 0 || can1 == 0 || config == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }

    state->high_voltage_requested = high_voltage_enable;
    state->snapshot.high_voltage_requested = high_voltage_enable;
    state->apply_count++;
    state->snapshot.apply_count = state->apply_count;

    if (!can1->online) {
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        state->snapshot.last_result = state->last_result;
        return state->last_result;
    }

    if (config->power_protocol == ECU_POWER_PROTOCOL_DISABLED) {
        state->last_result = high_voltage_enable ?
                             ECU_DEVICE_APPLY_UNCONFIGURED :
                             ECU_DEVICE_APPLY_OK;
        state->snapshot.last_result = state->last_result;
        return state->last_result;
    }
    if (config->power_protocol != ECU_POWER_PROTOCOL_SUPPLIER_CAN) {
        state->last_result = ECU_DEVICE_APPLY_UNCONFIGURED;
        state->snapshot.last_result = state->last_result;
        return state->last_result;
    }

#if ECU_POWER_CAN_TX_ENABLE
    bool ok = power_device_send_bms_if_due(state, can1, high_voltage_enable, now_ms);
    ok = power_device_send_dcdc48_if_due(state, can1, high_voltage_enable, now_ms) && ok;
    ok = power_device_send_dcdc12_if_due(state, can1, high_voltage_enable, now_ms) && ok;
    ok = power_device_send_dcac_if_due(state, can1, high_voltage_enable, now_ms) && ok;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
#else
    state->last_result = ECU_DEVICE_APPLY_OK;
#endif
    state->snapshot.last_result = state->last_result;
    return state->last_result;
}

void power_device_get_snapshot(const power_device_state_t *state,
                               power_device_snapshot_t *out)
{
    if (state != 0 && out != 0) {
        *out = state->snapshot;
    }
}
