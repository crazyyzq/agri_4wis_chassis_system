#ifndef COMMAND_ARBITER_H
#define COMMAND_ARBITER_H

#include "vehicle_types.h"

void vehicle_actuator_command_safe_default(vehicle_actuator_command_t *out);
void command_arbiter_update(const remote_control_request_t *remote,
                            const auto_control_request_t *auto_request,
                            const vehicle_safety_snapshot_t *safety,
                            uint32_t now_ms,
                            vehicle_actuator_command_t *out);

#endif /* COMMAND_ARBITER_H */
