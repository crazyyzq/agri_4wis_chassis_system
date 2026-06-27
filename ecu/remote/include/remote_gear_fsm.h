#ifndef REMOTE_GEAR_FSM_H
#define REMOTE_GEAR_FSM_H

#include "remote_types.h"

typedef struct {
    remote_gear_state_t state;
    ecu_gear_request_t requested_gear;
    ecu_gear_request_t active_gear;
    bool request_rejected;
    diag_code_t diagnostic;
} remote_gear_fsm_t;

void remote_gear_fsm_init(remote_gear_fsm_t *fsm);
void remote_gear_fsm_update(remote_gear_fsm_t *fsm,
                            const remote_input_snapshot_t *input,
                            const remote_preconditions_t *preconditions);
remote_gear_state_t remote_gear_fsm_get_state(const remote_gear_fsm_t *fsm);

#endif /* REMOTE_GEAR_FSM_H */
