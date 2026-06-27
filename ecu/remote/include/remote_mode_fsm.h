#ifndef REMOTE_MODE_FSM_H
#define REMOTE_MODE_FSM_H

#include "remote_types.h"

typedef struct {
    ecu_home_domain_t requested_domain;
    ecu_home_domain_t active_domain;
    ecu_motion_mode_t requested_motion_mode;
    ecu_motion_mode_t active_motion_mode;
    uint32_t domain_guard_until_ms;
    bool request_rejected;
    diag_code_t diagnostic;
} remote_mode_fsm_t;

void remote_mode_fsm_init(remote_mode_fsm_t *fsm, uint32_t now_ms);
void remote_mode_fsm_update(remote_mode_fsm_t *fsm,
                            const remote_input_snapshot_t *input,
                            const remote_preconditions_t *preconditions,
                            uint32_t guard_ms);
ecu_motion_mode_t remote_mode_fsm_get_state(const remote_mode_fsm_t *fsm);

#endif /* REMOTE_MODE_FSM_H */
