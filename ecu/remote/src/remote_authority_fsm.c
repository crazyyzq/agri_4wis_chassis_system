#include "remote_authority_fsm.h"

static bool auto_preconditions_ok(const remote_input_snapshot_t *input,
                                  const remote_preconditions_t *preconditions)
{
    return input->home == REMOTE_POS_LOW &&
           preconditions->active_gear == ECU_GEAR_REQUEST_P &&
           preconditions->zero_speed &&
           preconditions->throttle_low &&
           preconditions->steering_neutral &&
           !preconditions->active_transition &&
           !preconditions->estop_latched &&
           !preconditions->a_class_fault &&
           preconditions->external_auto_request_valid;
}

static bool manual_takeover_requested(const remote_input_snapshot_t *input,
                                      const remote_preconditions_t *preconditions)
{
    return input->authority != REMOTE_POS_CENTER ||
           !preconditions->throttle_low ||
           !preconditions->steering_neutral ||
           input->gear != REMOTE_POS_CENTER ||
           input->home != REMOTE_POS_LOW ||
           preconditions->estop_latched ||
           preconditions->a_class_fault ||
           !preconditions->external_auto_request_valid;
}

void remote_authority_fsm_init(remote_authority_fsm_t *fsm)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = REMOTE_AUTHORITY_MANUAL;
    fsm->auto_control_allowed = false;
    fsm->request_rejected = false;
    fsm->diagnostic = DIAG_OK;
}

void remote_authority_fsm_update(remote_authority_fsm_t *fsm,
                                 const remote_input_snapshot_t *input,
                                 const remote_preconditions_t *preconditions)
{
    if (fsm == 0 || input == 0 || preconditions == 0) {
        return;
    }

    fsm->request_rejected = false;
    fsm->auto_control_allowed = false;

    if (fsm->state == REMOTE_AUTHORITY_AUTO_ACTIVE &&
        manual_takeover_requested(input, preconditions)) {
        fsm->state = REMOTE_AUTHORITY_TAKEOVER_WAIT_NEUTRAL;
        fsm->diagnostic = DIAG_REJECT_AUTHORITY_PRECONDITION;
        return;
    }

    if (fsm->state == REMOTE_AUTHORITY_TAKEOVER_WAIT_NEUTRAL) {
        if (preconditions->throttle_low && preconditions->steering_neutral) {
            fsm->state = REMOTE_AUTHORITY_MANUAL;
            fsm->diagnostic = DIAG_OK;
        }
        return;
    }

    if (input->authority != REMOTE_POS_CENTER) {
        fsm->state = REMOTE_AUTHORITY_MANUAL;
        fsm->diagnostic = DIAG_OK;
        return;
    }

    fsm->state = REMOTE_AUTHORITY_AUTO_REQUESTED;
    if (auto_preconditions_ok(input, preconditions)) {
        fsm->state = REMOTE_AUTHORITY_AUTO_ACTIVE;
        fsm->auto_control_allowed = true;
        fsm->diagnostic = DIAG_OK;
    } else {
        fsm->state = REMOTE_AUTHORITY_REJECTED;
        fsm->request_rejected = true;
        fsm->diagnostic = DIAG_REJECT_AUTHORITY_PRECONDITION;
    }
}

remote_authority_state_t remote_authority_fsm_get_state(const remote_authority_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : REMOTE_AUTHORITY_REJECTED;
}
