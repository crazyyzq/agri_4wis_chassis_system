#include "remote_adjust_fsm.h"

static bool adjust_preconditions_ok(const remote_input_snapshot_t *input, const remote_preconditions_t *preconditions)
{
    return input->home == REMOTE_POS_CENTER &&
           preconditions->active_gear == ECU_GEAR_REQUEST_P &&
           preconditions->zero_speed &&
           preconditions->throttle_low &&
           preconditions->steering_neutral &&
           !preconditions->estop_latched &&
           !preconditions->a_class_fault;
}

void remote_adjust_fsm_init(remote_adjust_fsm_t *fsm)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = ADJUST_STATE_IDLE;
    fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
    fsm->request_rejected = false;
    fsm->diagnostic = DIAG_OK;
}

void remote_adjust_fsm_update(remote_adjust_fsm_t *fsm,
                              const remote_input_snapshot_t *input,
                              const remote_preconditions_t *preconditions)
{
    if (fsm == 0 || input == 0 || preconditions == 0) {
        return;
    }
    fsm->request_rejected = false;
    if (!adjust_preconditions_ok(input, preconditions)) {
        if (fsm->state != ADJUST_STATE_IDLE) {
            fsm->state = fsm->adjust_owner == REMOTE_ADJUST_OWNER_TRACK ?
                         ADJUST_STATE_TRACK_EXITING : ADJUST_STATE_ABORTED;
            fsm->diagnostic = DIAG_TRACK_ADJUST_ABORTED;
        }
        return;
    }
    bool clearance_active = input->clearance == REMOTE_POS_LOW ||
                            input->clearance == REMOTE_POS_HIGH;
    bool track_active = input->track == REMOTE_POS_LOW ||
                        input->track == REMOTE_POS_HIGH;
    if (fsm->adjust_owner == REMOTE_ADJUST_OWNER_NONE) {
        if (clearance_active && track_active) {
            fsm->request_rejected = true;
            fsm->diagnostic = DIAG_REJECT_ADJUST_OWNER_CONFLICT;
        } else if (clearance_active) {
            fsm->adjust_owner = REMOTE_ADJUST_OWNER_CLEARANCE;
            fsm->state = ADJUST_STATE_CLEARANCE_ACTIVE;
        } else if (track_active) {
            fsm->adjust_owner = REMOTE_ADJUST_OWNER_TRACK;
            fsm->state = ADJUST_STATE_TRACK_PREPARE;
        }
    } else if (!clearance_active && !track_active) {
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
        fsm->state = ADJUST_STATE_IDLE;
    } else if (fsm->adjust_owner == REMOTE_ADJUST_OWNER_TRACK &&
               fsm->state == ADJUST_STATE_TRACK_PREPARE) {
        fsm->state = preconditions->brake_release_confirmed ?
                     ADJUST_STATE_TRACK_ACTIVE : ADJUST_STATE_TRACK_PREPARE;
    }
}

remote_adjust_state_t remote_adjust_fsm_get_state(const remote_adjust_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : ADJUST_STATE_ABORTED;
}
