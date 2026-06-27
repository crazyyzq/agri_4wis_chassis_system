#ifndef REMOTE_AUTHORITY_FSM_H
#define REMOTE_AUTHORITY_FSM_H

#include "remote_types.h"

typedef struct {
    remote_authority_state_t state;
    bool auto_control_allowed;
    bool request_rejected;
    diag_code_t diagnostic;
} remote_authority_fsm_t;

/* Initialize remote control-authority state.
 *
 * Owner: task_remote_manager.
 * ISR: not safe.
 * Failure behavior: null pointers are ignored.
 */
void remote_authority_fsm_init(remote_authority_fsm_t *fsm);

/* Update manual/automatic authority from CH7 and safety preconditions.
 *
 * CH7 center requests automatic control. Any manual stick operation, gear/home
 * departure, e-stop, SBUS fault, external request timeout, or A-class fault
 * returns authority to manual and requires CH3/CH1 neutral before re-entry.
 */
void remote_authority_fsm_update(remote_authority_fsm_t *fsm,
                                 const remote_input_snapshot_t *input,
                                 const remote_preconditions_t *preconditions);

remote_authority_state_t remote_authority_fsm_get_state(const remote_authority_fsm_t *fsm);

#endif /* REMOTE_AUTHORITY_FSM_H */
