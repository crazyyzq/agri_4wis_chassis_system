#include <string.h>

#include "remote_manager.h"

void remote_manager_init(remote_manager_t *manager, uint32_t now_ms)
{
    if (manager == 0) {
        return;
    }
    memset(manager, 0, sizeof(*manager));
    remote_link_fsm_init(&manager->link, now_ms);
    remote_arm_fsm_init(&manager->arm, now_ms);
    remote_estop_fsm_init(&manager->estop, now_ms);
    remote_gear_fsm_init(&manager->gear);
    remote_mode_fsm_init(&manager->mode, now_ms);
    remote_adjust_fsm_init(&manager->adjust);
    remote_power_fsm_init(&manager->power, now_ms);
    remote_authority_fsm_init(&manager->authority);
    remote_lights_fsm_init(&manager->lights);
    remote_event_lifecycle_init(&manager->mode_event, REMOTE_EVENT_MODE_REQUEST);
    remote_event_lifecycle_init(&manager->power_event, REMOTE_EVENT_POWER_REQUEST);
    remote_event_lifecycle_init(&manager->estop_reset_event, REMOTE_EVENT_ESTOP_RESET);
    remote_event_lifecycle_init(&manager->light_event, REMOTE_EVENT_LIGHT_REQUEST);
}

void remote_manager_update(remote_manager_t *manager,
                           const remote_input_snapshot_t *input,
                           const remote_preconditions_t *preconditions,
                           const ecu_config_t *config)
{
    if (manager == 0 || input == 0 || preconditions == 0 || config == 0) {
        return;
    }
    remote_link_fsm_update(&manager->link, input, config->link_qualify_ms);
    remote_estop_fsm_update(&manager->estop, input, preconditions);

    remote_preconditions_t derived = *preconditions;
    derived.link_online = manager->link.state == REMOTE_LINK_ONLINE;
    derived.estop_latched = manager->estop.state != REMOTE_ESTOP_CLEAR;
    remote_arm_fsm_update(&manager->arm, input, &derived, config->neutral_qualify_ms);
    derived.arm_ready = manager->arm.state == REMOTE_ARM_READY;

    remote_adjust_fsm_update(&manager->adjust, input, &derived);
    derived.adjustment_active = manager->adjust.state != ADJUST_STATE_IDLE;
    remote_gear_fsm_update(&manager->gear, input, &derived);
    derived.active_gear = manager->gear.active_gear;
    derived.requested_gear = manager->gear.requested_gear;

    remote_mode_fsm_update(&manager->mode, input, &derived, config->domain_event_guard_ms);
    remote_power_fsm_update(&manager->power, input, &derived, config);
    remote_authority_fsm_update(&manager->authority, input, &derived);
    remote_lights_fsm_update(&manager->lights, input, derived.estop_latched || derived.a_class_fault);

    manager->request.link_state = manager->link.state;
    manager->request.arm_state = manager->arm.state;
    manager->request.estop_state = manager->estop.state;
    manager->request.estop_source = manager->estop.estop_source;
    manager->request.gear_state = manager->gear.state;
    manager->request.requested_gear = manager->gear.requested_gear;
    manager->request.active_gear = manager->gear.active_gear;
    manager->request.requested_domain = manager->mode.requested_domain;
    manager->request.active_domain = manager->mode.active_domain;
    manager->request.requested_motion_mode = manager->mode.requested_motion_mode;
    manager->request.active_motion_mode = manager->mode.active_motion_mode;
    manager->request.adjust_state = manager->adjust.state;
    manager->request.adjust_owner = manager->adjust.adjust_owner;
    manager->request.power_state = manager->power.state;
    manager->request.authority_state = manager->authority.state;
    manager->request.high_voltage_enable_request = manager->power.high_voltage_enable_request;
    manager->request.orderly_shutdown_request = manager->power.orderly_shutdown_request;
    manager->request.auto_control_allowed = manager->authority.auto_control_allowed;
    manager->request.indicator_mode = manager->lights.indicator_mode;
    manager->request.horn_on = manager->lights.horn_on;
    manager->request.headlight_on = manager->lights.headlight_on;
    manager->request.request_rejected = manager->gear.request_rejected ||
                                        manager->mode.request_rejected ||
                                        manager->adjust.request_rejected ||
                                        manager->power.request_rejected ||
                                        manager->authority.request_rejected;
    manager->request.diagnostic = manager->estop.diagnostic != DIAG_OK ? manager->estop.diagnostic :
                                  manager->gear.diagnostic != DIAG_OK ? manager->gear.diagnostic :
                                  manager->mode.diagnostic != DIAG_OK ? manager->mode.diagnostic :
                                  manager->adjust.diagnostic != DIAG_OK ? manager->adjust.diagnostic :
                                  manager->power.diagnostic != DIAG_OK ? manager->power.diagnostic :
                                  manager->authority.diagnostic;
}

void remote_manager_get_request(const remote_manager_t *manager, remote_control_request_t *out)
{
    if (manager != 0 && out != 0) {
        *out = manager->request;
    }
}
