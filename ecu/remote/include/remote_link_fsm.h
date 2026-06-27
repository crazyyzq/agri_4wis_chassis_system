#ifndef REMOTE_LINK_FSM_H
#define REMOTE_LINK_FSM_H

#include "remote_types.h"

typedef struct {
    remote_link_state_t state;
    uint32_t qualify_since_ms;
    diag_code_t diagnostic;
} remote_link_fsm_t;

void remote_link_fsm_init(remote_link_fsm_t *fsm, uint32_t now_ms);
void remote_link_fsm_update(remote_link_fsm_t *fsm, const remote_input_snapshot_t *input, uint32_t qualify_ms);
remote_link_state_t remote_link_fsm_get_state(const remote_link_fsm_t *fsm);

#endif /* REMOTE_LINK_FSM_H */
