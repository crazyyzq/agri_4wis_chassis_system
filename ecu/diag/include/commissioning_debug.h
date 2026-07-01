/* Commissioning-only diagnostic helpers.
 *
 * This module keeps bench and whole-machine bring-up hooks out of the CPU0
 * task orchestrator.  It may request read-only communication checks or a
 * guarded high-voltage request, but it must not own normal vehicle control
 * policy or send actuator enable/motion commands.
 */
#ifndef COMMISSIONING_DEBUG_H
#define COMMISSIONING_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

#include "can_bus_service.h"
#include "canopen_master_service.h"
#include "ecu_config.h"
#include "vehicle_types.h"

typedef struct {
    bool power_debug_active;
    can_bus_service_t can4_physical_test;
    uint32_t last_can2_scan_ms;
    uint32_t last_can3_scan_ms;
    uint32_t last_can4_physical_test_tx_ms;
    uint8_t can2_scan_index;
    uint8_t can3_scan_index;
    uint8_t can4_physical_test_sequence;
} commissioning_debug_context_t;

extern volatile ecu_commissioning_control_t g_ecu_commissioning_control;

void commissioning_debug_init(commissioning_debug_context_t *context,
                              const ecu_hardware_config_t *hardware_config);

bool commissioning_debug_apply_power_request(commissioning_debug_context_t *context,
                                             const vehicle_safety_snapshot_t *safety,
                                             vehicle_actuator_command_t *command,
                                             uint32_t now_ms);

void commissioning_debug_scan_can2(commissioning_debug_context_t *context,
                                   canopen_master_service_t *canopen,
                                   uint32_t now_ms);

void commissioning_debug_scan_can3(commissioning_debug_context_t *context,
                                   canopen_master_service_t *canopen,
                                   uint32_t now_ms);

void commissioning_debug_process_can4_physical_test(commissioning_debug_context_t *context,
                                                    uint32_t now_ms);

void commissioning_debug_get_can4_snapshot(const commissioning_debug_context_t *context,
                                           can_bus_service_t *out);

#endif /* COMMISSIONING_DEBUG_H */
