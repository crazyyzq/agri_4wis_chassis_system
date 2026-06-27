#ifndef REMOTE_ADJUST_FSM_H
#define REMOTE_ADJUST_FSM_H

#include "remote_types.h"

typedef struct {
    remote_adjust_state_t state;
    remote_adjust_owner_t adjust_owner;
    bool request_rejected;
    diag_code_t diagnostic;
} remote_adjust_fsm_t;

void remote_adjust_fsm_init(remote_adjust_fsm_t *fsm);
void remote_adjust_fsm_update(remote_adjust_fsm_t *fsm,
                              const remote_input_snapshot_t *input,
                              const remote_preconditions_t *preconditions);
remote_adjust_state_t remote_adjust_fsm_get_state(const remote_adjust_fsm_t *fsm);

#endif /* REMOTE_ADJUST_FSM_H */
