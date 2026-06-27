#include "remote_lights_fsm.h"

void remote_lights_fsm_init(remote_lights_fsm_t *fsm)
{
    if (fsm == 0) {
        return;
    }
    fsm->indicator_mode = INDICATOR_OFF;
    fsm->horn_on = false;
    fsm->headlight_on = false;
}

void remote_lights_fsm_update(remote_lights_fsm_t *fsm,
                              const remote_input_snapshot_t *input,
                              bool safety_hazard_active)
{
    if (fsm == 0 || input == 0) {
        return;
    }
    if (safety_hazard_active) {
        fsm->indicator_mode = INDICATOR_HAZARD_SAFETY;
    } else if (input->hazard == REMOTE_POS_HIGH) {
        fsm->indicator_mode = INDICATOR_HAZARD_USER;
    } else if (input->left_indicator == REMOTE_POS_HIGH) {
        fsm->indicator_mode = INDICATOR_LEFT;
    } else if (input->right_indicator == REMOTE_POS_HIGH) {
        fsm->indicator_mode = INDICATOR_RIGHT;
    } else {
        fsm->indicator_mode = INDICATOR_OFF;
    }
    fsm->horn_on = input->horn == REMOTE_POS_HIGH && !safety_hazard_active;
    fsm->headlight_on = input->headlight == REMOTE_POS_HIGH;
}

indicator_mode_t remote_lights_fsm_get_state(const remote_lights_fsm_t *fsm)
{
    return fsm != 0 ? fsm->indicator_mode : INDICATOR_HAZARD_SAFETY;
}
