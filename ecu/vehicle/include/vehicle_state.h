#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include "vehicle_types.h"

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
    vehicle_safety_snapshot_t safety;
    vehicle_actuator_command_t command;
} vehicle_state_snapshot_t;

void vehicle_state_publish(vehicle_state_snapshot_t *state,
                           uint32_t now_ms,
                           const vehicle_safety_snapshot_t *safety,
                           const vehicle_actuator_command_t *command);
void vehicle_state_copy(const vehicle_state_snapshot_t *state,
                        vehicle_state_snapshot_t *out);

#endif /* VEHICLE_STATE_H */
