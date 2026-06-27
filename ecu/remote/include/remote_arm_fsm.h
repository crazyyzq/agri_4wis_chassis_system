#ifndef REMOTE_ARM_FSM_H
#define REMOTE_ARM_FSM_H

#include "remote_types.h"

typedef struct {
    remote_arm_state_t state;
    uint32_t neutral_since_ms;
    diag_code_t diagnostic;
} remote_arm_fsm_t;

void remote_arm_fsm_init(remote_arm_fsm_t *fsm, uint32_t now_ms);
void remote_arm_fsm_update(remote_arm_fsm_t *fsm,
                           const remote_input_snapshot_t *input,
                           const remote_preconditions_t *preconditions,
                           uint32_t neutral_qualify_ms);
remote_arm_state_t remote_arm_fsm_get_state(const remote_arm_fsm_t *fsm);

#endif /* REMOTE_ARM_FSM_H */
