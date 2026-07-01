#include <string.h>
#include <stdint.h>

#include "lift_hydraulic_device.h"
#include "servo_drive_canopen.h"

/*
 * BC2 exposes the A axis and B axis as independent CANopen nodes.  The physical
 * J3 terminal names are different on the second axis (OUT4/IN7/IN8), but the
 * CANopen object map is addressed through the selected axis node.  Therefore
 * each lift node uses the same axis-local logical bits:
 *   - 0x2194 bit 0 controls the axis brake-release output.
 *   - 0x2190 bit 1 is the positive limit input.
 *   - 0x2190 bit 2 is the negative limit input.
 */
#define BC2_AXIS_OUTPUT_BRAKE_MASK          SERVO_DRIVE_OUTPUT_OUT1_MASK
#define BC2_AXIS_INPUT_POSITIVE_LIMIT_MASK  SERVO_DRIVE_INPUT_IN2_MASK
#define BC2_AXIS_INPUT_NEGATIVE_LIMIT_MASK  SERVO_DRIVE_INPUT_IN3_MASK

/* Convert a floating-point engineering command into the signed integer counts
 * accepted by the servo drive.  Saturation avoids undefined overflow if a
 * debug command or upstream controller supplies an out-of-range value. */
static int32_t scaled_float_to_i32(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 2147483647.0f) {
        return 2147483647;
    }
    if (scaled < -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)scaled;
}

/* Send one lift-axis motion command.
 *
 * Before commanding a new target position, the adapter reads the drive terminal
 * input-state object and blocks motion into an active limit switch.  If the
 * read fails, the command is still sent; the drive-side limit function remains
 * the last line of defense and the failed SDO is visible in diagnostics. */
static bool send_lift_command(canopen_master_service_t *canopen,
                              const ecu_canopen_node_config_t *node,
                              float height_mm,
                              float height_rate_mm_s,
                              float lift_scale,
                              uint16_t positive_limit_mask,
                              uint16_t negative_limit_mask,
                              bool *blocked_by_limit)
{
    uint16_t control_word = (height_rate_mm_s == 0.0f) ?
                            SERVO_DRIVE_CONTROL_QUICK_STOP :
                            SERVO_DRIVE_CONTROL_ENABLE_OPERATION;
    uint16_t input_states = 0U;
    if (blocked_by_limit != 0) {
        *blocked_by_limit = false;
    }

    if (servo_drive_canopen_read_input_states(canopen, node, &input_states)) {
        bool positive_limit = (input_states & positive_limit_mask) != 0U;
        bool negative_limit = (input_states & negative_limit_mask) != 0U;
        if ((height_rate_mm_s > 0.0f && positive_limit) ||
            (height_rate_mm_s < 0.0f && negative_limit)) {
            if (blocked_by_limit != 0) {
                *blocked_by_limit = true;
            }
            (void)servo_drive_canopen_send_control_word(canopen,
                                                        node,
                                                        SERVO_DRIVE_CONTROL_QUICK_STOP);
            return false;
        }
    }

    int32_t height_counts = scaled_float_to_i32(height_mm, lift_scale);

    return servo_drive_canopen_select_mode(canopen,
                                           node,
                                           control_word,
                                           SERVO_DRIVE_MODE_PROFILE_POSITION) &&
           servo_drive_canopen_set_target_position(canopen,
                                                   node,
                                                   control_word,
                                                   height_counts);
}

/* Command the hydraulic station motor through the BC drive on CAN3.
 *
 * The station is treated as a velocity-mode servo: enable means run at the
 * configured service velocity, disable means quick-stop and zero target speed. */
static bool send_hydraulic_pump_command(canopen_master_service_t *canopen,
                                        const ecu_canopen_node_config_t *node,
                                        bool hydraulic_enable)
{
    uint16_t control_word = hydraulic_enable ?
                            SERVO_DRIVE_CONTROL_ENABLE_OPERATION :
                            SERVO_DRIVE_CONTROL_QUICK_STOP;
    int32_t velocity_counts = hydraulic_enable ?
                              ECU_HYDRAULIC_PUMP_ENABLE_VELOCITY_COUNTS_PER_SEC :
                              0;

    return servo_drive_canopen_select_mode(canopen,
                                           node,
                                           control_word,
                                           SERVO_DRIVE_MODE_PROFILE_VELOCITY) &&
           servo_drive_canopen_set_target_velocity(canopen,
                                                   node,
                                                   control_word,
                                                   velocity_counts);
}

/* Translate the vehicle-level "brake release requested" flag into the active
 * bit written to 0x2194.  The polarity is intentionally configuration-driven
 * because drive terminal output inversion is often changed during commissioning. */
static uint16_t brake_active_mask(bool brake_release, uint16_t output_mask)
{
    bool canopen_active = brake_release ?
                          (ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT != 0U) :
                          (ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT == 0U);
    return canopen_active ? output_mask : 0U;
}

/* Convert signed track-width rate into the PCB hydraulic valve output mask.
 * The final DIO write is still gated by the hydraulic-enable command. */
static uint32_t valve_mask_from_track_rate(const ecu_hardware_config_t *config,
                                           float track_rate_mm_s)
{
    if (track_rate_mm_s > 0.0f) {
        return config->hydraulic_track_extend_mask;
    }
    if (track_rate_mm_s < 0.0f) {
        return config->hydraulic_track_retract_mask;
    }
    return 0U;
}

/* Only sources that have passed the authority/safety pipeline may update
 * CANopen lift outputs.  This prevents stale passive sources from repeatedly
 * rewriting brake and motion objects. */
static bool command_source_allows_lift_output(ecu_command_source_t source)
{
    return source == COMMAND_SOURCE_REMOTE ||
           source == COMMAND_SOURCE_AUTO ||
           source == COMMAND_SOURCE_MAINTENANCE ||
           source == COMMAND_SOURCE_CPU1 ||
           source == COMMAND_SOURCE_SAFETY;
}

/* Avoid resending identical CANopen SDO sequences every scheduler tick.
 *
 * Lift axes and the hydraulic pump share CAN3.  Track-width adjustment may
 * change only the hydraulic pump/valve intent while height fields remain
 * unchanged, so the cache key must include both lift and hydraulic fields.  The
 * command source is included so a safety override is sent even when target
 * values numerically match the previous command.
 */
static bool can3_actuator_command_changed(const lift_hydraulic_device_state_t *state,
                                          const vehicle_actuator_command_t *command)
{
    return !state->last_lift_command_valid ||
           state->last_lift_command.target_height_mm != command->target_height_mm ||
           state->last_lift_command.height_rate_mm_s != command->height_rate_mm_s ||
           state->last_lift_command.track_rate_mm_s != command->track_rate_mm_s ||
           state->last_lift_command.hydraulic_enable != command->hydraulic_enable ||
           state->last_lift_command.hydraulic_valve_mask != command->hydraulic_valve_mask ||
           state->last_lift_command.source != command->source;
}

/* Decide when an unchanged CAN3 command must be re-sent.
 *
 * This protects lift brakes, lift motion and hydraulic-pump commands from a
 * subtle SDO queue failure mode: the adapter can cache a command after all
 * request entries were accepted, but one of those downloads can still fail
 * later in the CANopen service.  The periodic refresh gives the drive network a
 * bounded retry without flooding CAN3 every scheduler cycle.
 */
static bool lift_command_refresh_due(const lift_hydraulic_device_state_t *state,
                                     uint32_t now_ms)
{
    return !state->last_lift_command_valid ||
           (uint32_t)(now_ms - state->last_lift_command_queue_ms) >=
               ECU_CANOPEN_LIFT_COMMAND_REFRESH_MS;
}

/* Reset counters and cached command state.  The device adapter owns no hardware
 * resources directly; CANopen and DIO services are injected at apply time. */
void lift_hydraulic_device_init(lift_hydraulic_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

/* Apply the vehicle lift/hydraulic command to CAN3 servos and local valves.
 *
 * Lift axes are processed in vehicle leg order.  Each BC2 axis receives its own
 * SDO writes through its own CANopen node, including the brake-release output.
 * Local PCB outputs are limited to hydraulic enable and valve coils; servo
 * motor brakes are never driven through PCB DIO channels here. */
ecu_device_apply_result_t lift_hydraulic_device_apply(lift_hydraulic_device_state_t *state,
                                                      canopen_master_service_t *canopen,
                                                      dio_service_t *dio,
                                                      const ecu_hardware_config_t *config,
                                                      const vehicle_actuator_command_t *command,
                                                      uint32_t now_ms)
{
    if (state == 0 || canopen == 0 || dio == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!canopen->snapshot.initialized || !canopen->snapshot.can_normal) {
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }

    bool ok = true;
    bool changed = can3_actuator_command_changed(state, command);
    bool refresh_due = lift_command_refresh_due(state, now_ms);
    if (command_source_allows_lift_output(command->source) &&
        (changed || refresh_due)) {
        bool lift_brake_release = command->height_rate_mm_s != 0.0f;

        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            const ecu_canopen_node_config_t *node = &config->lift_nodes[wheel];
            bool axis_blocked_by_limit = false;
            bool axis_brake_release;

            bool lift_ok = send_lift_command(canopen,
                                             node,
                                             command->target_height_mm,
                                             command->height_rate_mm_s,
                                             config->lift_mm_to_counts,
                                             BC2_AXIS_INPUT_POSITIVE_LIMIT_MASK,
                                             BC2_AXIS_INPUT_NEGATIVE_LIMIT_MASK,
                                             &axis_blocked_by_limit);
            ok = lift_ok && ok;
            axis_brake_release = lift_brake_release && !axis_blocked_by_limit;
            ok = servo_drive_canopen_set_output_state(
                     canopen,
                     node,
                     BC2_AXIS_OUTPUT_BRAKE_MASK,
                     brake_active_mask(axis_brake_release,
                                       BC2_AXIS_OUTPUT_BRAKE_MASK)) && ok;
        }
        ok = send_hydraulic_pump_command(canopen,
                                         &config->hydraulic_pump_node,
                                         command->hydraulic_enable) && ok;
        if (ok) {
            state->last_lift_command = *command;
            state->last_lift_command_valid = true;
            state->last_lift_command_queue_ms = now_ms;
        }
    } else {
        state->skipped_lift_canopen_count++;
    }

    uint32_t valve_mask = command->hydraulic_valve_mask |
                          valve_mask_from_track_rate(config, command->track_rate_mm_s);
    valve_mask &= config->hydraulic_managed_valve_mask;
    dio_service_write_masked(dio, config->dio_hydraulic_enable_mask, command->hydraulic_enable);
    dio_service_write_masked(dio, config->hydraulic_managed_valve_mask, false);
    dio_service_write_masked(dio, valve_mask, command->hydraulic_enable && valve_mask != 0U);
    state->last_valve_mask = valve_mask;
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
