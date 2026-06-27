#include "remote_gear_fsm.h"

static ecu_gear_request_t gear_from_position(remote_position_t position)
{
    if (position == REMOTE_POS_LOW) return ECU_GEAR_REQUEST_R;
    if (position == REMOTE_POS_HIGH) return ECU_GEAR_REQUEST_D;
    return ECU_GEAR_REQUEST_P;
}

void remote_gear_fsm_init(remote_gear_fsm_t *fsm)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = GEAR_STATE_PARKED_BRAKED;
    fsm->requested_gear = ECU_GEAR_REQUEST_P;
    fsm->active_gear = ECU_GEAR_REQUEST_P;
    fsm->request_rejected = false;
    fsm->diagnostic = DIAG_OK;
}

void remote_gear_fsm_update(remote_gear_fsm_t *fsm,
                            const remote_input_snapshot_t *input,
                            const remote_preconditions_t *preconditions)
{
    if (fsm == 0 || input == 0 || preconditions == 0) {
        return;
    }
    fsm->request_rejected = false;
    fsm->requested_gear = gear_from_position(input->gear);

    if (preconditions->adjustment_active && fsm->requested_gear == ECU_GEAR_REQUEST_P) {
        fsm->state = GEAR_STATE_TRACK_COMPLIANT; /* TRACK_COMPLIANT keeps active P while allowing track adjust. */
        fsm->active_gear = ECU_GEAR_REQUEST_P;
        fsm->diagnostic = DIAG_OK;
        return;
    }
    if (fsm->requested_gear == ECU_GEAR_REQUEST_P) {
        fsm->state = preconditions->zero_speed ? GEAR_STATE_PARKED_BRAKED : GEAR_STATE_STOPPING;
        fsm->active_gear = ECU_GEAR_REQUEST_P;
        fsm->diagnostic = DIAG_OK;
        return;
    }
    if (!preconditions->zero_speed && fsm->active_gear != fsm->requested_gear) {
        fsm->state = GEAR_STATE_SHIFT_REJECTED;
        fsm->request_rejected = true;
        fsm->diagnostic = DIAG_REJECT_DRIVE_REVERSE_WHILE_MOVING;
        return;
    }
    if (!preconditions->throttle_low) {
        fsm->state = fsm->requested_gear == ECU_GEAR_REQUEST_D ? GEAR_STATE_ARM_D : GEAR_STATE_ARM_R;
        fsm->active_gear = ECU_GEAR_REQUEST_P;
        fsm->diagnostic = DIAG_REJECT_THROTTLE_NOT_LOW;
        return;
    }
    if (!preconditions->brake_release_confirmed) {
        fsm->state = fsm->requested_gear == ECU_GEAR_REQUEST_D ? GEAR_STATE_ARM_D : GEAR_STATE_ARM_R;
        fsm->active_gear = ECU_GEAR_REQUEST_P;
        fsm->diagnostic = DIAG_OK;
        return;
    }
    fsm->active_gear = fsm->requested_gear;
    fsm->state = fsm->active_gear == ECU_GEAR_REQUEST_D ? GEAR_STATE_DRIVE_D : GEAR_STATE_DRIVE_R;
    fsm->diagnostic = DIAG_OK;
}

remote_gear_state_t remote_gear_fsm_get_state(const remote_gear_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : GEAR_STATE_SHIFT_REJECTED;
}
