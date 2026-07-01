#include "safety_manager.h"

void safety_manager_apply(const vehicle_safety_snapshot_t *safety,
                          vehicle_actuator_command_t *command)
{
    if (safety == 0 || command == 0) {
        return;
    }

    bool brake_release_allowed = safety->brake_release_allowed &&
                                 !safety->a_class_fault &&
                                 !safety->estop_latched &&
                                 !safety->sbus_failsafe &&
                                 !safety->shutdown_protect_active;

    if (safety->a_class_fault || safety->estop_latched ||
        safety->sbus_failsafe || safety->controlled_stop_active ||
        safety->shutdown_protect_active) {
        command->source = COMMAND_SOURCE_SAFETY;
        command->target_speed_kph = 0.0f;
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            command->target_wheel_speed_kph[wheel] = 0.0f;
        }
        command->height_rate_mm_s = 0.0f;
        command->track_rate_mm_s = 0.0f;
        command->hydraulic_enable = false;
        command->high_voltage_enable = false;
        command->hydraulic_valve_mask = 0U;
        command->horn_on = false;
        command->diagnostic = safety->primary_diag;
    }

    if (!brake_release_allowed) {
        command->brake_release = false;
    }
}
