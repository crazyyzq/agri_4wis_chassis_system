#ifndef REMOTE_LIGHTS_FSM_H
#define REMOTE_LIGHTS_FSM_H

#include "remote_types.h"

typedef struct {
    indicator_mode_t indicator_mode;
    bool horn_on;
    bool headlight_on;
} remote_lights_fsm_t;

void remote_lights_fsm_init(remote_lights_fsm_t *fsm);
void remote_lights_fsm_update(remote_lights_fsm_t *fsm,
                              const remote_input_snapshot_t *input,
                              bool safety_hazard_active);
indicator_mode_t remote_lights_fsm_get_state(const remote_lights_fsm_t *fsm);

#endif /* REMOTE_LIGHTS_FSM_H */
