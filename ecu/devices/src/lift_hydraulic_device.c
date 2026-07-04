#include <string.h>
#include <stdint.h>

#include "lift_hydraulic_device.h"
/* Convert signed track-width rate into the PCB hydraulic valve output mask.
 * The final DIO write is still gated by the hydraulic-enable command. */
static uint32_t valve_mask_from_track_rate(const ecu_hardware_config_t *config,
                                           float track_rate_mm_s)
{
    if (track_rate_mm_s > 0.0f) {
        return config->hydraulic_track_extend_mask;
    }
    if (track_rate_mm_s < 0.0f) {
        return config->hydraulic_track_retract_mask;
    }
    return 0U;
}

/* Avoid resending identical CANopen SDO sequences every scheduler tick.
 *
 * CAN3 servo outputs are disabled in the V7 static-contract commit.  The cache
 * still tracks high-level lift and hydraulic intent so later standard-PDO CAN3
 * work can re-enable command publishing without reintroducing SDO control.
 */
static bool can3_actuator_command_changed(const lift_hydraulic_device_state_t *state,
                                          const vehicle_actuator_command_t *command)
{
    return !state->last_lift_command_valid ||
           state->last_lift_command.target_height_mm != command->target_height_mm ||
           state->last_lift_command.height_rate_mm_s != command->height_rate_mm_s ||
           state->last_lift_command.track_rate_mm_s != command->track_rate_mm_s ||
           state->last_lift_command.hydraulic_enable != command->hydraulic_enable ||
           state->last_lift_command.hydraulic_valve_mask != command->hydraulic_valve_mask ||
           state->last_lift_command.source != command->source;
}

/* Reset counters and cached command state.  The device adapter owns no hardware
 * resources directly; CANopen and DIO services are injected at apply time. */
void lift_hydraulic_device_init(lift_hydraulic_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

/* Apply the vehicle lift/hydraulic command to CAN3 servos and local valves.
 *
 * Lift axes are processed in vehicle leg order.  Each BC2 axis receives its own
 * SDO writes through its own CANopen node, including the brake-release output.
 * Local PCB outputs are limited to hydraulic enable and valve coils; servo
 * motor brakes are never driven through PCB DIO channels here. */
ecu_device_apply_result_t lift_hydraulic_device_apply(lift_hydraulic_device_state_t *state,
                                                      canopen_master_service_t *canopen,
                                                      dio_service_t *dio,
                                                      const ecu_hardware_config_t *config,
                                                      const vehicle_actuator_command_t *command,
                                                      uint32_t now_ms)
{
    if (state == 0 || canopen == 0 || dio == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!canopen->snapshot.initialized || !canopen->snapshot.can_normal) {
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }

    bool ok = true;
    if (can3_actuator_command_changed(state, command)) {
        state->last_lift_command = *command;
        state->last_lift_command_valid = true;
        state->last_lift_command_queue_ms = now_ms;
    } else {
        state->skipped_lift_canopen_count++;
    }

    uint32_t valve_mask = command->hydraulic_valve_mask |
                          valve_mask_from_track_rate(config, command->track_rate_mm_s);
    valve_mask &= config->hydraulic_managed_valve_mask;
    dio_service_write_masked(dio, config->dio_hydraulic_enable_mask, command->hydraulic_enable);
    dio_service_write_masked(dio, config->hydraulic_managed_valve_mask, false);
    dio_service_write_masked(dio, valve_mask, command->hydraulic_enable && valve_mask != 0U);
    state->last_valve_mask = valve_mask;
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
