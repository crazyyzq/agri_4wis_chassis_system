#include "ecu_time.h"
#include "remote_link_fsm.h"

void remote_link_fsm_init(remote_link_fsm_t *fsm, uint32_t now_ms)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = REMOTE_LINK_OFFLINE;
    fsm->qualify_since_ms = now_ms;
    fsm->diagnostic = DIAG_OK;
}

void remote_link_fsm_update(remote_link_fsm_t *fsm, const remote_input_snapshot_t *input, uint32_t qualify_ms)
{
    if (fsm == 0 || input == 0) {
        return;
    }
    if (input->sbus_timeout) {
        fsm->state = REMOTE_LINK_OFFLINE;
        fsm->qualify_since_ms = input->now_ms;
        fsm->diagnostic = DIAG_REMOTE_ESTOP_SBUS_TIMEOUT;
        return;
    }
    if (input->sbus_failsafe) {
        fsm->state = REMOTE_LINK_FAILSAFE;
        fsm->qualify_since_ms = input->now_ms;
        fsm->diagnostic = DIAG_REMOTE_ESTOP_FAILSAFE;
        return;
    }
    if (input->decode_error_limit) {
        fsm->state = REMOTE_LINK_FAILSAFE;
        fsm->qualify_since_ms = input->now_ms;
        fsm->diagnostic = DIAG_REMOTE_ESTOP_DECODE_ERRORS;
        return;
    }
    if (input->credibility_error) {
        fsm->state = REMOTE_LINK_FAILSAFE;
        fsm->qualify_since_ms = input->now_ms;
        fsm->diagnostic = DIAG_REMOTE_ESTOP_CREDIBILITY;
        return;
    }
    if (!input->sbus_valid) {
        fsm->state = REMOTE_LINK_OFFLINE;
        fsm->qualify_since_ms = input->now_ms;
        return;
    }
    if (fsm->state == REMOTE_LINK_ONLINE) {
        fsm->diagnostic = DIAG_OK;
        return;
    }
    if (fsm->state != REMOTE_LINK_QUALIFYING) {
        fsm->state = REMOTE_LINK_QUALIFYING;
        fsm->qualify_since_ms = input->now_ms;
    }
    if (ecu_time_elapsed(input->now_ms, fsm->qualify_since_ms, qualify_ms)) {
        fsm->state = REMOTE_LINK_ONLINE;
        fsm->diagnostic = DIAG_OK;
    }
}

remote_link_state_t remote_link_fsm_get_state(const remote_link_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : REMOTE_LINK_OFFLINE;
}
