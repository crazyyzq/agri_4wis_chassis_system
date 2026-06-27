#include "ecu_time.h"
#include "remote_arm_fsm.h"

static bool neutral_entry_only(const remote_input_snapshot_t *input, const remote_preconditions_t *preconditions)
{
    return input != 0 && preconditions != 0 &&
           preconditions->link_online &&
           !preconditions->estop_latched &&
           !preconditions->a_class_fault &&
           input->gear == REMOTE_POS_CENTER &&
           input->throttle == REMOTE_POS_LOW &&
           input->steering == REMOTE_POS_CENTER;
}

void remote_arm_fsm_init(remote_arm_fsm_t *fsm, uint32_t now_ms)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = REMOTE_ARM_DISARMED;
    fsm->neutral_since_ms = now_ms;
    fsm->diagnostic = DIAG_OK;
}

void remote_arm_fsm_update(remote_arm_fsm_t *fsm,
                           const remote_input_snapshot_t *input,
                           const remote_preconditions_t *preconditions,
                           uint32_t neutral_qualify_ms)
{
    if (fsm == 0 || input == 0 || preconditions == 0) {
        return;
    }
    if (!preconditions->link_online || preconditions->estop_latched || preconditions->a_class_fault) {
        fsm->state = REMOTE_ARM_DISARMED;
        fsm->neutral_since_ms = input->now_ms;
        return;
    }
    if (fsm->state == REMOTE_ARM_READY) {
        /* Neutral is required to enter READY, not to keep READY while driving. */
        return;
    }
    if (!neutral_entry_only(input, preconditions)) {
        fsm->state = REMOTE_ARM_WAIT_NEUTRAL;
        fsm->neutral_since_ms = input->now_ms;
        fsm->diagnostic = DIAG_REJECT_THROTTLE_NOT_LOW;
        return;
    }
    if (ecu_time_elapsed(input->now_ms, fsm->neutral_since_ms, neutral_qualify_ms)) {
        fsm->state = REMOTE_ARM_READY;
        fsm->diagnostic = DIAG_OK;
    } else {
        fsm->state = REMOTE_ARM_WAIT_NEUTRAL;
    }
}

remote_arm_state_t remote_arm_fsm_get_state(const remote_arm_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : REMOTE_ARM_DISARMED;
}
