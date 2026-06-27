#include <string.h>

#include "command_arbiter.h"
#include "vehicle_command_executor.h"

void vehicle_command_executor_init(vehicle_executor_state_t *executor)
{
    if (executor == 0) {
        return;
    }
    memset(executor, 0, sizeof(*executor));
    vehicle_actuator_command_safe_default(&executor->last_command);
}

bool vehicle_command_executor_apply(vehicle_executor_state_t *executor,
                                    const vehicle_actuator_command_t *command)
{
    if (executor == 0 || command == 0) {
        return false;
    }
    executor->last_command = *command;
    executor->applied_sequence++;
    return true;
}

void vehicle_command_executor_get_state(const vehicle_executor_state_t *executor,
                                        vehicle_executor_state_t *out)
{
    if (executor != 0 && out != 0) {
        *out = *executor;
    }
}
