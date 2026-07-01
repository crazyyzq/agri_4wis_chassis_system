#include "commissioning_debug.h"

#include <string.h>

#include "can_bus_hw.h"

volatile ecu_commissioning_control_t g_ecu_commissioning_control;

void commissioning_debug_init(commissioning_debug_context_t *context,
                              const ecu_hardware_config_t *hardware_config)
{
    if (context == 0) {
        return;
    }

    memset(context, 0, sizeof(*context));

#if ECU_ENABLE_CAN4_PHYSICAL_TEST_TX
    if (hardware_config == 0) {
        return;
    }

    can_bus_service_init(&context->can4_physical_test,
                         hardware_config->can4_bitrate);
    if (!can_bus_hw_init_can4_auxiliary(&context->can4_physical_test,
                                        hardware_config->can4_bitrate)) {
        can_bus_service_note_error_from_isr(&context->can4_physical_test);
    }
#else
    (void)hardware_config;
#endif
}

static bool commissioning_power_debug_is_allowed(const vehicle_safety_snapshot_t *safety)
{
#if ECU_ENABLE_COMMISSIONING_POWER_DEBUG
    /* The remote/SBUS estop latch is allowed during no-remote commissioning
     * because this hook is limited to high-voltage contactor request only.
     * Motion, lift, hydraulic valves and brake release remain forced off.
     */
    return safety != 0 &&
           !safety->a_class_fault &&
           !safety->shutdown_protect_active;
#else
    (void)safety;
    return false;
#endif
}

static bool commissioning_power_debug_request_is_active(uint32_t now_ms)
{
#if ECU_ENABLE_COMMISSIONING_POWER_DEBUG
    if (g_ecu_commissioning_control.magic != ECU_COMMISSIONING_CONTROL_MAGIC ||
        !g_ecu_commissioning_control.high_voltage_enable) {
        g_ecu_commissioning_control.request_time_ms = 0U;
        return false;
    }

    if (g_ecu_commissioning_control.request_time_ms == 0U) {
        g_ecu_commissioning_control.request_time_ms = now_ms;
    }

    return (uint32_t)(now_ms - g_ecu_commissioning_control.request_time_ms) <=
           ECU_COMMISSIONING_HV_REQUEST_TIMEOUT_MS;
#else
    (void)now_ms;
    return false;
#endif
}

bool commissioning_debug_apply_power_request(commissioning_debug_context_t *context,
                                             const vehicle_safety_snapshot_t *safety,
                                             vehicle_actuator_command_t *command,
                                             uint32_t now_ms)
{
    if (context != 0) {
        context->power_debug_active = false;
    }

    if (context == 0 ||
        command == 0 ||
        !commissioning_power_debug_is_allowed(safety) ||
        !commissioning_power_debug_request_is_active(now_ms)) {
        return false;
    }

    command->target_speed_kph = 0.0f;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        command->target_wheel_speed_kph[wheel] = 0.0f;
        command->target_steer_deg[wheel] = 0.0f;
    }
    command->height_rate_mm_s = 0.0f;
    command->track_rate_mm_s = 0.0f;
    command->brake_release = false;
    command->high_voltage_enable = true;
    command->hydraulic_enable = false;
    command->hydraulic_valve_mask = 0U;
    context->power_debug_active = true;
    return true;
}

static bool commissioning_scan_due(uint32_t now_ms, uint32_t *last_scan_ms)
{
    if (last_scan_ms == 0) {
        return false;
    }
    if (*last_scan_ms == 0U ||
        (uint32_t)(now_ms - *last_scan_ms) >= ECU_CANOPEN_COMMISSIONING_SCAN_PERIOD_MS) {
        *last_scan_ms = now_ms;
        return true;
    }
    return false;
}

void commissioning_debug_scan_can2(commissioning_debug_context_t *context,
                                   canopen_master_service_t *canopen,
                                   uint32_t now_ms)
{
#if ECU_ENABLE_COMMISSIONING_CANOPEN_SCAN
    if (context == 0 || canopen == 0 ||
        !commissioning_scan_due(now_ms, &context->last_can2_scan_ms)) {
        return;
    }

    const ecu_hardware_config_t *config = ecu_hardware_config_default();
    uint8_t scan_index = context->can2_scan_index % (ECU_WHEEL_COUNT * 2U);
    const ecu_canopen_node_config_t *node =
        scan_index < ECU_WHEEL_COUNT ?
        &config->drive_nodes[scan_index] :
        &config->steer_nodes[scan_index - ECU_WHEEL_COUNT];

    (void)canopen_master_service_request_sdo_read(canopen,
                                                  node->node_id,
                                                  ECU_CANOPEN_OBJ_STATUSWORD,
                                                  0U);
    context->can2_scan_index = (uint8_t)(scan_index + 1U);
#else
    (void)context;
    (void)canopen;
    (void)now_ms;
#endif
}

void commissioning_debug_scan_can3(commissioning_debug_context_t *context,
                                   canopen_master_service_t *canopen,
                                   uint32_t now_ms)
{
#if ECU_ENABLE_COMMISSIONING_CANOPEN_SCAN
    if (context == 0 || canopen == 0 ||
        !commissioning_scan_due(now_ms, &context->last_can3_scan_ms)) {
        return;
    }

    const ecu_hardware_config_t *config = ecu_hardware_config_default();
    uint8_t scan_index = context->can3_scan_index % (ECU_WHEEL_COUNT + 1U);
    const ecu_canopen_node_config_t *node =
        scan_index < ECU_WHEEL_COUNT ?
        &config->lift_nodes[scan_index] :
        &config->hydraulic_pump_node;

    (void)canopen_master_service_request_sdo_read(canopen,
                                                  node->node_id,
                                                  ECU_CANOPEN_OBJ_STATUSWORD,
                                                  0U);
    context->can3_scan_index = (uint8_t)(scan_index + 1U);
#else
    (void)context;
    (void)canopen;
    (void)now_ms;
#endif
}

void commissioning_debug_process_can4_physical_test(commissioning_debug_context_t *context,
                                                    uint32_t now_ms)
{
#if ECU_ENABLE_CAN4_PHYSICAL_TEST_TX
    if (context == 0) {
        return;
    }
    if (context->last_can4_physical_test_tx_ms != 0U &&
        (uint32_t)(now_ms - context->last_can4_physical_test_tx_ms) <
        ECU_CAN4_PHYSICAL_TEST_TX_PERIOD_MS) {
        return;
    }

    context->last_can4_physical_test_tx_ms = now_ms;

    ecu_can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = ECU_CAN4_PHYSICAL_TEST_FRAME_ID;
    frame.size = 8U;
    frame.extended = false;
    frame.remote = false;
    frame.data[0] = 0xC4U;
    frame.data[1] = context->can4_physical_test_sequence++;
    frame.data[2] = (uint8_t)(now_ms & 0xFFU);
    frame.data[3] = (uint8_t)((now_ms >> 8U) & 0xFFU);
    frame.data[4] = (uint8_t)((now_ms >> 16U) & 0xFFU);
    frame.data[5] = (uint8_t)((now_ms >> 24U) & 0xFFU);
    frame.data[6] = 0x55U;
    frame.data[7] = 0xAAU;

    (void)can_bus_service_send_frame(&context->can4_physical_test, &frame);
    can_bus_hw_poll_can4_rx(&context->can4_physical_test);
#else
    (void)context;
    (void)now_ms;
#endif
}

void commissioning_debug_get_can4_snapshot(const commissioning_debug_context_t *context,
                                           can_bus_service_t *out)
{
    if (context == 0 || out == 0) {
        return;
    }
    can_bus_service_get_snapshot(&context->can4_physical_test, out);
}
