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

static bool clearance_request_active(const remote_input_snapshot_t *input)
{
    return input->clearance == REMOTE_POS_LOW ||
           input->clearance == REMOTE_POS_HIGH;
}

static bool track_request_active(const remote_input_snapshot_t *input)
{
    return input->track == REMOTE_POS_LOW ||
           input->track == REMOTE_POS_HIGH;
}

static bool suspension_request_active(const remote_input_snapshot_t *input)
{
    return input->gear == REMOTE_POS_LOW ||
           input->gear == REMOTE_POS_HIGH;
}

void remote_adjust_fsm_init(remote_adjust_fsm_t *fsm)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = ADJUST_STATE_IDLE;
    fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
    fsm->hydraulic_suspension_target = REMOTE_HYDRAULIC_SUSPENSION_FRONT;
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
    if (input->home != REMOTE_POS_CENTER) {
        fsm->state = ADJUST_STATE_IDLE;
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
        fsm->diagnostic = DIAG_OK;
        return;
    }
    if (!adjust_preconditions_ok(input, preconditions)) {
        if (fsm->state != ADJUST_STATE_IDLE) {
            fsm->state = fsm->adjust_owner == REMOTE_ADJUST_OWNER_TRACK ?
                         ADJUST_STATE_TRACK_EXITING : ADJUST_STATE_ABORTED;
            fsm->diagnostic = DIAG_TRACK_ADJUST_ABORTED;
        }
        return;
    }

    if (input->r1_changed) {
        fsm->hydraulic_suspension_target = REMOTE_HYDRAULIC_SUSPENSION_FRONT;
    }
    if (input->r2_changed) {
        fsm->hydraulic_suspension_target = REMOTE_HYDRAULIC_SUSPENSION_REAR;
    }

    bool clearance_active = clearance_request_active(input);
    bool track_active = track_request_active(input);
    bool suspension_active = suspension_request_active(input);

    if (!clearance_active && !track_active && !suspension_active) {
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
        fsm->state = ADJUST_STATE_READY;
        fsm->diagnostic = DIAG_OK;
        return;
    }

    if (track_active || suspension_active) {
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_HYDRAULIC;
        fsm->state = ADJUST_STATE_HYDRAULIC_ACTIVE;
        fsm->diagnostic = DIAG_OK;
        return;
    }

    if (clearance_active) {
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_CLEARANCE;
        fsm->state = ADJUST_STATE_CLEARANCE_ACTIVE;
        fsm->diagnostic = DIAG_OK;
    }
}

remote_adjust_state_t remote_adjust_fsm_get_state(const remote_adjust_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : ADJUST_STATE_ABORTED;
}
