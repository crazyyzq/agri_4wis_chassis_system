#ifndef REMOTE_ESTOP_FSM_H
#define REMOTE_ESTOP_FSM_H

#include "remote_types.h"

typedef struct {
    remote_estop_state_t state;
    ecu_estop_source_t estop_source;
    uint32_t reset_since_ms;
    diag_code_t diagnostic;
} remote_estop_fsm_t;

void remote_estop_fsm_init(remote_estop_fsm_t *fsm, uint32_t now_ms);
void remote_estop_fsm_update(remote_estop_fsm_t *fsm,
                             const remote_input_snapshot_t *input,
                             const remote_preconditions_t *preconditions);
remote_estop_state_t remote_estop_fsm_get_state(const remote_estop_fsm_t *fsm);
ecu_estop_source_t remote_estop_fsm_get_source(const remote_estop_fsm_t *fsm);

#endif /* REMOTE_ESTOP_FSM_H */
