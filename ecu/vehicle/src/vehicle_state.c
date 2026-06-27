#include "vehicle_state.h"

void vehicle_state_publish(vehicle_state_snapshot_t *state,
                           uint32_t now_ms,
                           const vehicle_safety_snapshot_t *safety,
                           const vehicle_actuator_command_t *command)
{
    if (state == 0 || safety == 0 || command == 0) {
        return;
    }
    state->sequence++;
    state->timestamp_ms = now_ms;
    state->safety = *safety;
    state->command = *command;
}

void vehicle_state_copy(const vehicle_state_snapshot_t *state,
                        vehicle_state_snapshot_t *out)
{
    if (state != 0 && out != 0) {
        *out = *state;
    }
}
