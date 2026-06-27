#ifndef VEHICLE_COMMAND_EXECUTOR_H
#define VEHICLE_COMMAND_EXECUTOR_H

#include "vehicle_types.h"

void vehicle_command_executor_init(vehicle_executor_state_t *executor);
bool vehicle_command_executor_apply(vehicle_executor_state_t *executor,
                                    const vehicle_actuator_command_t *command);
void vehicle_command_executor_get_state(const vehicle_executor_state_t *executor,
                                        vehicle_executor_state_t *out);

#endif /* VEHICLE_COMMAND_EXECUTOR_H */
