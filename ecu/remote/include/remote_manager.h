#ifndef REMOTE_MANAGER_H
#define REMOTE_MANAGER_H

#include "ecu_config.h"
#include "remote_adjust_fsm.h"
#include "remote_authority_fsm.h"
#include "remote_arm_fsm.h"
#include "remote_estop_fsm.h"
#include "remote_event_lifecycle.h"
#include "remote_gear_fsm.h"
#include "remote_lights_fsm.h"
#include "remote_link_fsm.h"
#include "remote_mode_fsm.h"
#include "remote_power_fsm.h"

typedef struct {
    remote_link_fsm_t link;
    remote_arm_fsm_t arm;
    remote_estop_fsm_t estop;
    remote_gear_fsm_t gear;
    remote_mode_fsm_t mode;
    remote_adjust_fsm_t adjust;
    remote_power_fsm_t power;
    remote_authority_fsm_t authority;
    remote_lights_fsm_t lights;
    remote_event_lifecycle_t mode_event;
    remote_event_lifecycle_t power_event;
    remote_event_lifecycle_t estop_reset_event;
    remote_event_lifecycle_t light_event;
    remote_control_request_t request;
} remote_manager_t;

void remote_manager_init(remote_manager_t *manager, uint32_t now_ms);
void remote_manager_update(remote_manager_t *manager,
                           const remote_input_snapshot_t *input,
                           const remote_preconditions_t *preconditions,
                           const ecu_config_t *config);
void remote_manager_get_request(const remote_manager_t *manager, remote_control_request_t *out);

#endif /* REMOTE_MANAGER_H */
