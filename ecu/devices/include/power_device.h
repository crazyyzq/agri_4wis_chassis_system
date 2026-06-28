#ifndef POWER_DEVICE_H
#define POWER_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_bus_service.h"
#include "ecu_config.h"
#include "ecu_types.h"
#include "power_can_protocol.h"

typedef struct {
    bool high_voltage_requested;
    uint32_t apply_count;
    ecu_device_apply_result_t last_result;
    bool can1_online;
    bool power_ready;
    bool low_voltage_ok;
    bool bms_online;
    bool dcdc48_online;
    bool dcdc12_online;
    bool dcac_online;
    uint32_t bms_last_rx_ms;
    uint32_t dcdc48_last_rx_ms;
    uint32_t dcdc12_last_rx_ms;
    uint32_t dcac_last_rx_ms;
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;
    uint32_t command_tx_count;
    ecu_can_frame_t last_rx_frame;
    ecu_can_frame_t last_tx_frame;
    power_can_bms_status_t bms;
    power_can_bms_cell_voltage_t bms_cell_voltage;
    power_can_bms_cell_temperature_t bms_cell_temperature;
    power_can_bms_sof_t bms_sof;
    power_can_bms_sop_t bms_sop;
    power_can_bms_soh_statistics_t bms_soh;
    power_can_bms_error_status_t bms_error;
    power_can_dcdc48_status_t dcdc48;
    power_can_dcdc12_status_t dcdc12;
    power_can_dcac_status_t dcac;
    power_can_dcac_input_status_t dcac_input;
} power_device_snapshot_t;

typedef struct {
    bool high_voltage_requested;
    uint32_t apply_count;
    ecu_device_apply_result_t last_result;
    power_device_snapshot_t snapshot;
    uint32_t last_bms_command_ms;
    uint32_t last_dcdc48_command_ms;
    uint32_t last_dcdc12_command_ms;
    uint32_t last_dcac_command_ms;
} power_device_state_t;

/* Initialize the CPU0-owned power device adapter.
 *
 * Owner: task_can1_power / vehicle executor path on CPU0.
 * ISR: not safe.
 * Failure behavior: null state is ignored.
 */
void power_device_init(power_device_state_t *state);

/* Decode all queued CAN1 power-bus frames into the power snapshot.
 *
 * Owner: task_can1_power on CPU0.
 * ISR: not safe. RX frames are drained through can_bus_service_pop_rx_frame().
 * Failure behavior: invalid arguments are ignored.
 */
void power_device_process_rx(power_device_state_t *state,
                             can_bus_service_t *can1,
                             const ecu_hardware_config_t *config,
                             uint32_t now_ms);

/* Apply high-voltage intent to CAN1 power devices.
 *
 * Units: high_voltage_enable is a logical request after safety clamping. Timing
 * is in milliseconds from the CPU0 scheduler.
 * Dependencies: CAN1 service and the explicit power protocol selected in the
 * hardware configuration.
 * Failure behavior: returns a device result and records it in state; no retry or
 * delayed execution is hidden inside this function.
 */
ecu_device_apply_result_t power_device_apply(power_device_state_t *state,
                                             can_bus_service_t *can1,
                                             const ecu_hardware_config_t *config,
                                             bool high_voltage_enable,
                                             uint32_t now_ms);

void power_device_get_snapshot(const power_device_state_t *state,
                               power_device_snapshot_t *out);

#endif /* POWER_DEVICE_H */
