#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "vehicle_types.h"

void safety_manager_apply(const vehicle_safety_snapshot_t *safety,
                          vehicle_actuator_command_t *command);

#endif /* SAFETY_MANAGER_H */
