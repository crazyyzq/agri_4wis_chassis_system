#include <string.h>

#include "remote_manager.h"

static bool remote_manager_update_b1_zero_calibration_request(remote_manager_t *manager,
                                                              const remote_input_snapshot_t *input,
                                                              bool gate_open)
{
    if (manager == 0 || input == 0) {
        return false;
    }

    manager->b1_zero_calibration_last_raw_request = false;
    manager->b1_zero_calibration_last_gate_blocked = false;
    manager->b1_zero_calibration_press_latched = false;

    /* A maintenance gesture must be formed entirely while all safety gates are
     * valid.  Do not carry partial click counts or the 600 ms request hold
     * across a link/arm/estop/gear/HOME boundary. */
    if (!gate_open) {
        manager->b1_zero_calibration_press_count = 0U;
        manager->b1_zero_calibration_window_start_ms = input->now_ms;
        manager->b1_zero_calibration_last_request_ms = 0U;
        manager->b1_zero_calibration_input_initialized = false;
        /* Report the current gate state, not only the rare cycle on which a
         * blocked B1 edge happened. This makes zero_blk actionable on COM9. */
        manager->b1_zero_calibration_last_gate_blocked = true;
        return false;
    }

    /* The mapper reports a state transition when the first valid receiver
     * sample establishes CH10's stable endpoint.  That synchronization sample
     * is a baseline, not an operator click; otherwise startup begins at
     * b1_cnt=1 and only two real transitions can start steering calibration. */
    if (!manager->b1_zero_calibration_input_initialized) {
        manager->b1_zero_calibration_input_initialized = true;
        manager->b1_zero_calibration_press_count = 0U;
        manager->b1_zero_calibration_window_start_ms = input->now_ms;
        return false;
    }

    /* Field operation requirement: B1/CH10 is a maintenance gesture button.
     * Count every debounced CH10 state transition as one short press, regardless
     * of whether the new stable endpoint is low or high.  This matches the
     * actual transmitter behavior used during commissioning.
     */
    manager->b1_zero_calibration_press_latched = input->b1_changed;
    if (!input->b1_changed) {
        return false;
    }

    uint32_t now_ms = input->now_ms;
    if (manager->b1_zero_calibration_press_count == 0U ||
        (uint32_t)(now_ms - manager->b1_zero_calibration_window_start_ms) >
            ECU_REMOTE_B1_ZERO_CALIBRATION_WINDOW_MS) {
        manager->b1_zero_calibration_press_count = 0U;
        manager->b1_zero_calibration_window_start_ms = now_ms;
    }

    manager->b1_zero_calibration_press_count++;
    if (manager->b1_zero_calibration_press_count >=
        ECU_REMOTE_B1_ZERO_CALIBRATION_PRESS_COUNT) {
        manager->b1_zero_calibration_press_count = 0U;
        manager->b1_zero_calibration_window_start_ms = now_ms;
        manager->b1_zero_calibration_last_request_ms = now_ms;
        manager->b1_zero_calibration_last_raw_request = true;
        return true;
    }

    return false;
}

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

    /* HOME-center is an actuator-domain boundary.  Gate the gear FSM before
     * updating the adjustment FSM so the physical D/P/R switch remains a
     * hydraulic direction selector and never becomes an active drive gear while
     * the operator is in the adjustment domain.
     */
    derived.adjustment_active = input->home == REMOTE_POS_CENTER ||
                                manager->adjust.state != ADJUST_STATE_IDLE;
    remote_gear_fsm_update(&manager->gear, input, &derived);
    derived.active_gear = manager->gear.active_gear;
    derived.requested_gear = manager->gear.requested_gear;
    remote_adjust_fsm_update(&manager->adjust, input, &derived);
    derived.adjustment_active = manager->adjust.state != ADJUST_STATE_IDLE;

    remote_mode_fsm_update(&manager->mode, input, &derived, config->domain_event_guard_ms);
    remote_power_fsm_update(&manager->power, input, &derived, config);
    remote_authority_fsm_update(&manager->authority, input, &derived);
    remote_lights_fsm_update(&manager->lights, input, derived.estop_latched || derived.a_class_fault);
    bool steer_zero_calibration_gate_open =
        manager->link.state == REMOTE_LINK_ONLINE &&
        manager->arm.state == REMOTE_ARM_READY &&
        manager->estop.state == REMOTE_ESTOP_CLEAR &&
        manager->gear.active_gear == ECU_GEAR_REQUEST_P &&
        input->home == REMOTE_POS_CENTER;
    (void)remote_manager_update_b1_zero_calibration_request(manager,
                                                            input,
                                                            steer_zero_calibration_gate_open);
    bool steer_zero_calibration_request_hold_active =
        manager->b1_zero_calibration_last_request_ms != 0U &&
        (uint32_t)(input->now_ms - manager->b1_zero_calibration_last_request_ms) <=
            ECU_REMOTE_B1_ZERO_CALIBRATION_REQUEST_HOLD_MS;
    bool steer_zero_calibration_raw_event =
        manager->b1_zero_calibration_last_raw_request;
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
    manager->request.hydraulic_suspension_target =
        manager->adjust.hydraulic_suspension_target;
    manager->request.power_state = manager->power.state;
    manager->request.authority_state = manager->authority.state;
    manager->request.power_on_block_mask =
        manager->power.power_on_block_mask;
    manager->request.high_voltage_enable_request = manager->power.high_voltage_enable_request;
    manager->request.high_voltage_disable_request = manager->power.high_voltage_disable_request;
    manager->request.steer_zero_calibration_request =
        steer_zero_calibration_request_hold_active && steer_zero_calibration_gate_open;
    manager->request.b1_zero_calibration_pressed_latched =
        manager->b1_zero_calibration_press_latched;
    manager->request.b1_zero_calibration_raw_request =
        steer_zero_calibration_raw_event || steer_zero_calibration_request_hold_active;
    manager->request.b1_zero_calibration_gate_blocked =
        manager->b1_zero_calibration_last_gate_blocked;
    manager->request.b1_zero_calibration_press_count =
        manager->b1_zero_calibration_press_count;
    manager->request.orderly_shutdown_request = manager->power.orderly_shutdown_request;
    manager->request.auto_control_allowed = manager->authority.auto_control_allowed;
    manager->request.steer_per_mille = input->steer_per_mille;
    manager->request.throttle_per_mille = input->throttle_per_mille;
    manager->request.clearance_per_mille = input->clearance_per_mille;
    int8_t stable_track_direction =
        remote_adjust_fsm_get_track_direction(&manager->adjust);
    manager->request.track_per_mille =
        (int16_t)(stable_track_direction > 0 ? 1000 :
                  stable_track_direction < 0 ? -1000 : 0);
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
