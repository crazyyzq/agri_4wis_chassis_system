/* Runtime monitor print contract for the CPU0 debug UART.
 *
 * This module is intentionally diagnostic-only. It receives a compact snapshot
 * from the task layer and prints it to the board debug console. It must not own
 * control state, decide safety policy, access hardware registers or call the
 * vehicle executor.
 */
#ifndef RUNTIME_MONITOR_H
#define RUNTIME_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#include "diag_codes.h"
#include "ecu_types.h"
#include "remote_types.h"

typedef struct {
    uint32_t now_ms;
    uint32_t executor_sequence;

    bool sbus_valid;
    bool sbus_connected;
    bool sbus_failsafe;
    uint32_t sbus_frame_count;
    uint32_t sbus_decode_error_count;

    remote_link_state_t link_state;
    remote_estop_state_t estop_state;
    diag_code_t diagnostic;

    ecu_command_source_t source;
    ecu_motion_mode_t motion_mode;
    ecu_gear_request_t active_gear;
    int32_t target_speed_centi_kph;
    int32_t target_steer_centi_deg[ECU_WHEEL_COUNT];
    bool brake_release;
    bool high_voltage_enable;
    bool hydraulic_enable;
    uint32_t hydraulic_valve_mask;

    ecu_device_apply_result_t power_result;
    ecu_device_apply_result_t motion_result;
    ecu_device_apply_result_t lift_hydraulic_result;
    ecu_device_apply_result_t local_io_result;
    ecu_device_apply_result_t warning_light_result;
} runtime_monitor_snapshot_t;

/* Print one CPU0 runtime snapshot.
 *
 * Caller: CPU0 diagnostic task only.
 * Rate limit: handled by the task layer before this function is called.
 */
void runtime_monitor_print_cpu0(const runtime_monitor_snapshot_t *snapshot);

#endif /* RUNTIME_MONITOR_H */
