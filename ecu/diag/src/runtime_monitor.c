#include <stdio.h>

#include "ecu_config.h"
#include "motion_device.h"
#include "runtime_monitor.h"

static const char *bool_text(bool value)
{
    return value ? "1" : "0";
}

#if ECU_DEBUG_MONITOR_VERBOSE
static const char *steer_inhibit_reason_text(uint8_t reason)
{
    switch (reason) {
    case 0U: return "none";
    case 1U: return "estop_latched";
    case 2U: return "sbus_offline";
    case 3U: return "remote_disarmed";
    case 4U: return "gear_park";
    case 5U: return "axis_not_ready";
    case 6U: return "group_degraded";
    case 7U: return "command_source_not_authorized";
    case 8U: return "bench_mode_disabled";
    default: return "unknown";
    }
}

static const char *steer_commission_state_text(uint8_t state)
{
    switch (state) {
    case STEER_REMOTE_COMMISSION_DISABLED: return "disabled";
    case STEER_REMOTE_COMMISSION_WAIT_AUTH: return "wait_auth";
    case STEER_REMOTE_COMMISSION_WAIT_CALIBRATION: return "wait_calibration";
    case STEER_REMOTE_COMMISSION_WAIT_NEUTRAL: return "wait_neutral";
    case STEER_REMOTE_COMMISSION_TPDO_MONITOR: return "tpdo_monitor";
    case STEER_REMOTE_COMMISSION_AXIS_READY: return "axis_ready";
    case STEER_REMOTE_COMMISSION_CENTERING: return "centering";
    case STEER_REMOTE_COMMISSION_WAIT_SYNC_TX_COMPLETE: return "wait_sync_tx";
    case STEER_REMOTE_COMMISSION_WAIT_CENTER_SETTLE: return "wait_center_settle";
    case STEER_REMOTE_COMMISSION_ACTIVE: return "active";
    case STEER_REMOTE_COMMISSION_WAIT_POST_COMMAND_TPDO: return "wait_post_tpdo";
    case STEER_REMOTE_COMMISSION_FAULT: return "fault";
    default: return "unknown";
    }
}
#endif

static const char *commissioning_policy_text(void)
{
#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING
    return "STEER4_REMOTE_COMMISSIONING";
#elif ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED
    return "PDO_OUTPUT_ENABLED";
#elif ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_NODE5_STEER_PDO_ONLY
    return "NODE5_STEER_PDO_ONLY";
#elif ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_TPDO_MONITOR_ONLY
    return "TPDO_MONITOR_ONLY";
#else
    return "MAPPING_VERIFY_ONLY";
#endif
}

static const char *remote_steer_range_text(void)
{
#if ECU_BUILD_PROFILE_STEER4_REMOTE_90
    return "-90..90";
#else
    return "-50..50";
#endif
}

static const char *enabled_text_from_int(int enabled)
{
    return enabled != 0 ? "1" : "0";
}

static const char *link_state_text(remote_link_state_t state)
{
    switch (state) {
    case REMOTE_LINK_OFFLINE: return "offline";
    case REMOTE_LINK_QUALIFYING: return "qualifying";
    case REMOTE_LINK_ONLINE: return "online";
    case REMOTE_LINK_FAILSAFE: return "failsafe";
    default: return "unknown";
    }
}

static const char *estop_state_text(remote_estop_state_t state)
{
    switch (state) {
    case REMOTE_ESTOP_CLEAR: return "clear";
    case REMOTE_ESTOP_LATCHED: return "latched";
    case REMOTE_ESTOP_RESET_REQUESTED: return "reset_req";
    case REMOTE_ESTOP_CLEAR_WAIT_NORMAL: return "wait_normal";
    default: return "unknown";
    }
}

static const char *arm_state_text(remote_arm_state_t state)
{
    switch (state) {
    case REMOTE_ARM_DISARMED: return "disarmed";
    case REMOTE_ARM_WAIT_NEUTRAL: return "wait_neutral";
    case REMOTE_ARM_READY: return "ready";
    default: return "unknown";
    }
}

static const char *gear_state_text(remote_gear_state_t state)
{
    switch (state) {
    case GEAR_STATE_PARKED_BRAKED: return "parked";
    case GEAR_STATE_ARM_D: return "arm_d";
    case GEAR_STATE_ARM_R: return "arm_r";
    case GEAR_STATE_DRIVE_D: return "drive_d";
    case GEAR_STATE_DRIVE_R: return "drive_r";
    case GEAR_STATE_STOPPING: return "stopping";
    case GEAR_STATE_TRACK_COMPLIANT: return "track_compliant";
    case GEAR_STATE_SHIFT_REJECTED: return "shift_rejected";
    default: return "unknown";
    }
}

static const char *power_state_text(remote_power_state_t state)
{
    switch (state) {
    case REMOTE_POWER_OFF: return "off";
    case REMOTE_POWER_ON_REQUESTED: return "on_req";
    case REMOTE_POWER_ON: return "on";
    case REMOTE_POWER_DOWN_REQUESTED: return "down_req";
    case REMOTE_POWER_SHUTDOWN_PROTECT: return "shutdown_protect";
    case REMOTE_POWER_REJECTED: return "rejected";
    default: return "unknown";
    }
}

static const char *authority_state_text(remote_authority_state_t state)
{
    switch (state) {
    case REMOTE_AUTHORITY_MANUAL: return "manual";
    case REMOTE_AUTHORITY_AUTO_REQUESTED: return "auto_req";
    case REMOTE_AUTHORITY_AUTO_ACTIVE: return "auto";
    case REMOTE_AUTHORITY_TAKEOVER_WAIT_NEUTRAL: return "takeover_wait";
    case REMOTE_AUTHORITY_REJECTED: return "rejected";
    default: return "unknown";
    }
}

static const char *adjust_state_text(remote_adjust_state_t state)
{
    switch (state) {
    case ADJUST_STATE_IDLE: return "idle";
    case ADJUST_STATE_READY: return "ready";
    case ADJUST_STATE_CLEARANCE_ACTIVE: return "clearance";
    case ADJUST_STATE_TRACK_PREPARE: return "track_prepare";
    case ADJUST_STATE_TRACK_ACTIVE: return "track";
    case ADJUST_STATE_HYDRAULIC_ACTIVE: return "hydraulic";
    case ADJUST_STATE_TRACK_EXITING: return "track_exiting";
    case ADJUST_STATE_ABORTED: return "aborted";
    default: return "unknown";
    }
}

static const char *status_led_pattern_text(status_led_pattern_t pattern)
{
    switch (pattern) {
    case STATUS_LED_PATTERN_BOOT: return "boot";
    case STATUS_LED_PATTERN_NO_REMOTE: return "no_remote";
    case STATUS_LED_PATTERN_READY: return "ready";
    case STATUS_LED_PATTERN_ACTIVE: return "active";
    case STATUS_LED_PATTERN_WARNING: return "warning";
    case STATUS_LED_PATTERN_ESTOP: return "estop";
    case STATUS_LED_PATTERN_FATAL: return "fatal";
    default: return "unknown";
    }
}

#if ECU_DEBUG_MONITOR_VERBOSE
static const char *source_text(ecu_command_source_t source)
{
    switch (source) {
    case COMMAND_SOURCE_NONE: return "none";
    case COMMAND_SOURCE_SAFETY: return "safety";
    case COMMAND_SOURCE_REMOTE: return "remote";
    case COMMAND_SOURCE_AUTO: return "auto";
    case COMMAND_SOURCE_MAINTENANCE: return "maint";
    case COMMAND_SOURCE_CPU1: return "cpu1";
    default: return "unknown";
    }
}

static const char *motion_mode_text(ecu_motion_mode_t mode)
{
    switch (mode) {
    case ECU_MOTION_MODE_POSITIVE_ACKERMANN: return "ackermann";
    case ECU_MOTION_MODE_REVERSE_ACKERMANN: return "reverse_ack";
    case ECU_MOTION_MODE_SPIN: return "spin";
    case ECU_MOTION_MODE_CRAB: return "crab";
    default: return "unknown";
    }
}

static const char *gear_text(ecu_gear_request_t gear)
{
    switch (gear) {
    case ECU_GEAR_REQUEST_P: return "P";
    case ECU_GEAR_REQUEST_D: return "D";
    case ECU_GEAR_REQUEST_R: return "R";
    default: return "?";
    }
}

static const char *device_result_text(ecu_device_apply_result_t result)
{
    switch (result) {
    case ECU_DEVICE_APPLY_OK: return "ok";
    case ECU_DEVICE_APPLY_INVALID_ARGUMENT: return "bad_arg";
    case ECU_DEVICE_APPLY_BACKEND_OFFLINE: return "offline";
    case ECU_DEVICE_APPLY_UNCONFIGURED: return "unconfigured";
    case ECU_DEVICE_APPLY_REJECTED: return "rejected";
    default: return "unknown";
    }
}

static void print_centi_value(int32_t value)
{
    uint32_t magnitude;
    if (value < 0) {
        printf("-");
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }
    printf("%lu.%02lu",
           (unsigned long)(magnitude / 100U),
           (unsigned long)(magnitude % 100U));
}

static void print_milli_value(int32_t value)
{
    uint32_t magnitude;
    if (value < 0) {
        printf("-");
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }
    printf("%lu.%03lu",
           (unsigned long)(magnitude / 1000U),
           (unsigned long)(magnitude % 1000U));
}
#endif

void runtime_monitor_print_cpu0(const runtime_monitor_snapshot_t *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    printf("[ECU BUILD] profile=%s build_profile=%s policy=%s "
           "remote_range_deg=%s axis_mask=0x0F "
           "drive_rpdo=%s can3_rpdo=0 "
           "mapping_write=%u flash_write=0 brake_control=none\r\n",
           ECU_BUILD_PROFILE_TEXT,
           ECU_BUILD_PROFILE_TEXT,
           commissioning_policy_text(),
           remote_steer_range_text(),
           enabled_text_from_int(
               ECU_CANOPEN_COMMISSIONING_POLICY ==
               ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED &&
               ECU_COMMISSIONING_STEER_ONLY_MODE == 0),
           (unsigned int)ECU_ENABLE_MAINTENANCE_SDO_WRITES);

    printf("[ECU MON] t=%lu seq=%lu led=%s sbus_valid=%s sbus_conn=%s fs=%s "
           "frames=%lu dec_err=%lu link=%s "
           "arm=%s gear_fsm=%s power=%s auth=%s adjust=%s "
           "estop=%s diag=%s\r\n",
           (unsigned long)snapshot->now_ms,
           (unsigned long)snapshot->executor_sequence,
           status_led_pattern_text(snapshot->status_led_pattern),
           bool_text(snapshot->sbus_valid),
           bool_text(snapshot->sbus_connected),
           bool_text(snapshot->sbus_failsafe),
           (unsigned long)snapshot->sbus_frame_count,
           (unsigned long)snapshot->sbus_decode_error_count,
           link_state_text(snapshot->link_state),
           arm_state_text(snapshot->arm_state),
           gear_state_text(snapshot->gear_state),
           power_state_text(snapshot->power_state),
           authority_state_text(snapshot->authority_state),
           adjust_state_text(snapshot->adjust_state),
           estop_state_text(snapshot->estop_state),
           diag_code_name(snapshot->diagnostic));

    /* Keep one compact command line outside verbose mode.  During vehicle
     * commissioning this is the minimum evidence needed to distinguish a
     * remote/arbiter target-generation problem from a CAN2 PDO transport
     * problem, while avoiding the heavy SBUS/CANopen printf burst that can
     * interfere with realtime motion.
     */
    printf("[ECU CMD] src=%u mode=%u gear=%u speed_milli=%ld steer_cdeg=[%ld,%ld,%ld,%ld] "
           "brake=%s hv=%s hv_fb=%s hv_latch=%s res[pwr=%u mot=%u]\r\n",
           (unsigned int)snapshot->source,
           (unsigned int)snapshot->motion_mode,
           (unsigned int)snapshot->active_gear,
           (long)snapshot->target_speed_milli_mps,
           (long)snapshot->target_steer_centi_deg[0],
           (long)snapshot->target_steer_centi_deg[1],
           (long)snapshot->target_steer_centi_deg[2],
           (long)snapshot->target_steer_centi_deg[3],
           bool_text(snapshot->brake_release),
           bool_text(snapshot->high_voltage_enable),
           bool_text(snapshot->high_voltage_feedback_ready),
           bool_text(snapshot->high_voltage_relay_latched),
           (unsigned int)snapshot->power_result,
           (unsigned int)snapshot->motion_result);

    uint8_t drive_fresh_mask = 0U;
    uint8_t drive_fault_mask = 0U;
    uint8_t steer_fresh_mask = 0U;
    uint8_t steer_fault_mask = 0U;
    for (uint8_t node = ECU_CANOPEN_DRIVE_FR_NODE_ID;
         node <= ECU_CANOPEN_DRIVE_RR_NODE_ID;
         ++node) {
        const canopen_node_feedback_t *feedback =
            &snapshot->can2_canopen_snapshot.node_feedback[node];
        uint8_t bit = (uint8_t)(1U << (node - ECU_CANOPEN_DRIVE_FR_NODE_ID));
        if (feedback->feedback_fresh) {
            drive_fresh_mask |= bit;
        }
        if (feedback->fault_latched != 0U) {
            drive_fault_mask |= bit;
        }
    }
    for (uint8_t node = ECU_CANOPEN_STEER_FR_NODE_ID;
         node <= ECU_CANOPEN_STEER_RR_NODE_ID;
         ++node) {
        const canopen_node_feedback_t *feedback =
            &snapshot->can2_canopen_snapshot.node_feedback[node];
        uint8_t bit = (uint8_t)(1U << (node - ECU_CANOPEN_STEER_FR_NODE_ID));
        if (feedback->feedback_fresh) {
            steer_fresh_mask |= bit;
        }
        if (feedback->fault_latched != 0U) {
            steer_fault_mask |= bit;
        }
    }
    printf("[ECU CAN2 PDO] q=%lu drop=%lu tx=%lu tx_err=%lu state=%u group=%lu "
           "exp=%u done=%u fail=%u inflight=%u last_fail[g=%lu cob=0x%03x n=%u e=%ld] "
           "fresh[d=0x%02x s=0x%02x] fault[d=0x%02x s=0x%02x] "
           "recover=%lu consec_fail=%lu last_recover_ms=%lu\r\n",
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_queued_count,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_dropped_count,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_tx_count,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_tx_error_count,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_group_state,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_group_sequence,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_expected_frames,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_tx_complete_frames,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_failed_frames,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_in_flight_frames,
           (unsigned long)snapshot->can2_canopen_snapshot.last_pdo_failed_group_sequence,
           (unsigned int)snapshot->can2_canopen_snapshot.last_pdo_failed_cob_id,
           (unsigned int)snapshot->can2_canopen_snapshot.last_pdo_failed_node_id,
           (long)snapshot->can2_canopen_snapshot.last_pdo_failed_error,
           (unsigned int)drive_fresh_mask,
           (unsigned int)steer_fresh_mask,
           (unsigned int)drive_fault_mask,
           (unsigned int)steer_fault_mask,
           (unsigned long)snapshot->can2_realtime_transient_recovery_count,
           (unsigned long)snapshot->can2_realtime_consecutive_failure_count,
           (unsigned long)snapshot->can2_realtime_last_recovery_ms);

#if ECU_DEBUG_MONITOR_VERBOSE
    printf("[ECU SBUS RAW]");
    for (uint32_t channel = 0U; channel < ECU_SBUS_CHANNEL_COUNT; ++channel) {
        printf(" ch%02lu=%u",
               (unsigned long)(channel + 1U),
               (unsigned int)snapshot->sbus_channels[channel]);
    }
    printf(" ch17=%s ch18=%s lost=%s uart_err=%lu last=%lums\r\n",
           bool_text(snapshot->sbus_channel17),
           bool_text(snapshot->sbus_channel18),
           bool_text(snapshot->sbus_frame_lost),
           (unsigned long)snapshot->sbus_uart_error_count,
           (unsigned long)snapshot->sbus_last_frame_ms);

    printf("[ECU SBUS PPM]");
    for (uint32_t channel = 0U; channel < ECU_SBUS_CHANNEL_COUNT; ++channel) {
        printf(" ch%02lu=%u",
               (unsigned long)(channel + 1U),
               (unsigned int)snapshot->sbus_ppm_channels[channel]);
    }
    printf("\r\n");

    printf("[ECU CAN2] rx=%lu err=%lu rbuf=%u flags=0x%02x eflags=0x%02x rec=%u tec=%u lek=%u "
           "last_id=0x%03lx ext=%s rtr=%s dlc=%u data=[",
           (unsigned long)snapshot->can2_rx_count,
           (unsigned long)snapshot->can2_error_count,
           (unsigned int)snapshot->can2_rx_buffer_status,
           (unsigned int)snapshot->can2_tx_rx_flags,
           (unsigned int)snapshot->can2_error_flags,
           (unsigned int)snapshot->can2_receive_error_count,
           (unsigned int)snapshot->can2_transmit_error_count,
           (unsigned int)snapshot->can2_last_error_kind,
           (unsigned long)snapshot->can2_last_rx_id,
           bool_text(snapshot->can2_last_rx_extended),
           bool_text(snapshot->can2_last_rx_remote),
           (unsigned int)snapshot->can2_last_rx_size);
    for (uint32_t byte = 0U; byte < snapshot->can2_last_rx_size && byte < 8U; ++byte) {
        if (byte > 0U) {
            printf(" ");
        }
        printf("%02x", snapshot->can2_last_rx_data[byte]);
    }
    printf("]\r\n");

    printf("[ECU CANopen CAN2] init=%s state=%u normal=%s bitrate=%lu local=%u remote=%u "
           "proc=%lu hb=%lu hb_state=%u hb_last=%lums sdo_ok=%lu sdo_abort=%lu "
           "last_node=%u last=0x%04x:%u size=%u value=0x%08lx abort=0x%08lx err=%ld\r\n",
           bool_text(snapshot->can2_canopen_initialized),
           (unsigned int)snapshot->can2_canopen_snapshot.state,
           bool_text(snapshot->can2_canopen_snapshot.can_normal),
           (unsigned long)snapshot->can2_canopen_snapshot.bitrate,
           (unsigned int)snapshot->can2_canopen_snapshot.local_node_id,
           (unsigned int)snapshot->can2_canopen_snapshot.remote_node_id,
           (unsigned long)snapshot->can2_canopen_snapshot.process_count,
           (unsigned long)snapshot->can2_canopen_snapshot.heartbeat_count,
           (unsigned int)snapshot->can2_canopen_snapshot.last_heartbeat_state,
           (unsigned long)snapshot->can2_canopen_snapshot.last_heartbeat_ms,
           (unsigned long)snapshot->can2_canopen_snapshot.sdo_upload_count,
           (unsigned long)snapshot->can2_canopen_snapshot.sdo_abort_count,
           (unsigned int)snapshot->can2_canopen_snapshot.last_sdo_node_id,
           (unsigned int)snapshot->can2_canopen_snapshot.last_sdo_index,
           (unsigned int)snapshot->can2_canopen_snapshot.last_sdo_subindex,
           (unsigned int)snapshot->can2_canopen_snapshot.last_sdo_size,
           (unsigned long)snapshot->can2_canopen_snapshot.last_sdo_value,
           (unsigned long)snapshot->can2_canopen_snapshot.last_sdo_abort_code,
           (long)snapshot->can2_canopen_snapshot.last_error);

    printf("[ECU CANopen CMD] seq=%lu cmd=%u node=%u nmt=%lu dl_ok=%lu dl_abort=%lu "
           "last=0x%04x:%u size=%u value=%ld abort=0x%08lx err=%lu last_err=%ld\r\n",
           (unsigned long)snapshot->can2_canopen_snapshot.last_command_sequence,
           (unsigned int)snapshot->canopen_command,
           (unsigned int)snapshot->can2_canopen_snapshot.last_command_node_id,
           (unsigned long)snapshot->can2_canopen_snapshot.nmt_command_count,
           (unsigned long)snapshot->can2_canopen_snapshot.sdo_download_count,
           (unsigned long)snapshot->can2_canopen_snapshot.sdo_download_abort_count,
           (unsigned int)snapshot->can2_canopen_snapshot.last_download_index,
           (unsigned int)snapshot->can2_canopen_snapshot.last_download_subindex,
           (unsigned int)snapshot->can2_canopen_snapshot.last_download_size,
           (long)snapshot->can2_canopen_snapshot.last_download_value,
           (unsigned long)snapshot->can2_canopen_snapshot.last_download_abort_code,
           (unsigned long)snapshot->can2_canopen_snapshot.command_error_count,
           (long)snapshot->can2_canopen_snapshot.last_error);

    printf("[ECU CANopen PDO CAN2] queued=%lu dropped=%lu tx=%lu tx_err=%lu "
           "pdo_group_state=%u pdo_group=%lu pdo_expected_frames=%u "
           "pdo_arm_frame_count=%u pdo_trigger_frame_count=%u pdo_axis_mask=0x%02x "
           "pdo_position_group=%s sync_tx=%lu sync_done=%lu sync_inflight=%s "
           "sync_err=%lu last_sync=%lums last_sync_done=%lums "
           "tpdo_observer_ready=%s tpdo0_mask=0x%02x tpdo1_mask=0x%02x "
           "tpdo_observer_err_mask=0x%02x tpdo_observer_errs=%lu "
           "pdo_tx_complete_frames=%u pdo_failed_frames=%u pdo_in_flight_frames=%u "
           "pdo_arm_complete_frames=%u pdo_trigger_complete_frames=%u "
           "last_pdo_tx_complete_ms=%lu last_pdo_tx_timeout_ms=%lu "
           "pdo_current_err=%ld pdo_last_err=%ld "
           "last_tx[group=%lu cob=0x%03x node=%u phase=%u] "
           "last_fail[group=%lu group_id=%lu cob=0x%03x node=%u phase=%u err=%ld "
           "hist_err=%ld reason=%u failed_ms=%lu] drops[qfull=%lu conflict=%lu "
           "safety=%lu coalesce=%lu]\r\n",
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_queued_count,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_dropped_count,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_tx_count,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_tx_error_count,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_group_state,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_group_sequence,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_expected_frames,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_arm_frame_count,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_trigger_frame_count,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_axis_mask,
           bool_text(snapshot->can2_canopen_snapshot.pdo_position_group),
           (unsigned long)snapshot->can2_canopen_snapshot.sync_tx_count,
           (unsigned long)snapshot->can2_canopen_snapshot.sync_tx_complete_count,
           bool_text(snapshot->can2_canopen_snapshot.sync_in_flight),
           (unsigned long)snapshot->can2_canopen_snapshot.sync_tx_error_count,
           (unsigned long)snapshot->can2_canopen_snapshot.last_sync_tx_ms,
           (unsigned long)snapshot->can2_canopen_snapshot.last_sync_tx_complete_ms,
           bool_text(snapshot->can2_canopen_snapshot.steer_tpdo_observer_ready),
           (unsigned int)snapshot->can2_canopen_snapshot.tpdo0_observer_registered_mask,
           (unsigned int)snapshot->can2_canopen_snapshot.tpdo1_observer_registered_mask,
           (unsigned int)snapshot->can2_canopen_snapshot.steer_tpdo_observer_error_mask,
           (unsigned long)snapshot->can2_canopen_snapshot.tpdo_observer_registration_error_count,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_tx_complete_frames,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_failed_frames,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_in_flight_frames,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_arm_complete_frames,
           (unsigned int)snapshot->can2_canopen_snapshot.pdo_trigger_complete_frames,
           (unsigned long)snapshot->can2_canopen_snapshot.last_pdo_tx_complete_ms,
           (unsigned long)snapshot->can2_canopen_snapshot.last_pdo_tx_timeout_ms,
           (long)snapshot->can2_canopen_snapshot.last_pdo_current_error,
           (long)snapshot->can2_canopen_snapshot.last_pdo_error,
           (unsigned long)snapshot->can2_canopen_snapshot.last_pdo_tx_group_sequence,
           (unsigned int)snapshot->can2_canopen_snapshot.last_pdo_tx_cob_id,
           (unsigned int)snapshot->can2_canopen_snapshot.last_pdo_tx_node_id,
           (unsigned int)snapshot->can2_canopen_snapshot.last_pdo_tx_phase,
           (unsigned long)snapshot->can2_canopen_snapshot.last_pdo_failed_group_sequence,
           (unsigned long)snapshot->can2_canopen_snapshot.last_pdo_failed_group_id,
           (unsigned int)snapshot->can2_canopen_snapshot.last_pdo_failed_cob_id,
           (unsigned int)snapshot->can2_canopen_snapshot.last_pdo_failed_node_id,
           (unsigned int)snapshot->can2_canopen_snapshot.last_pdo_failed_phase,
           (long)snapshot->can2_canopen_snapshot.last_pdo_current_error,
           (long)snapshot->can2_canopen_snapshot.last_pdo_failed_error,
           (unsigned int)snapshot->can2_canopen_snapshot.last_pdo_failed_reason,
           (unsigned long)snapshot->can2_canopen_snapshot.last_pdo_failed_ms,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_queue_full_drop_count,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_group_conflict_drop_count,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_safety_inhibit_count,
           (unsigned long)snapshot->can2_canopen_snapshot.pdo_same_target_coalesce_count);

    printf("[ECU CAN2 MOTION TPDO]");
    for (uint8_t node = ECU_CANOPEN_DRIVE_FR_NODE_ID;
         node <= ECU_CANOPEN_STEER_RR_NODE_ID;
         ++node) {
        const canopen_node_feedback_t *feedback =
            &snapshot->can2_canopen_snapshot.node_feedback[node];
        printf(" n%u[fresh=%s p0=%s/%lu@%lu p1=%s/%lu@%lu pos=%ld vel=%ld "
               "fault=0x%08lx sw=0x%04x cur=%d mal=%lu]",
               (unsigned int)node,
               bool_text(feedback->feedback_fresh),
               bool_text(feedback->tpdo0_valid),
               (unsigned long)feedback->tpdo0_rx_count,
               (unsigned long)feedback->last_tpdo0_ms,
               bool_text(feedback->tpdo1_valid),
               (unsigned long)feedback->tpdo1_rx_count,
               (unsigned long)feedback->last_tpdo1_ms,
               (long)feedback->actual_position_counts,
               (long)feedback->actual_velocity_units,
               (unsigned long)feedback->fault_latched,
               (unsigned int)feedback->statusword,
               (int)feedback->actual_current_raw,
               (unsigned long)feedback->malformed_tpdo_count);
    }
    printf("\r\n");

    printf("[ECU STEER SAFETY] steer_normal_pdo_allowed=%s "
           "steer_safety_inhibited=%s steer_inhibit_reason=%u(%s) "
           "steer_safety_inhibit_count=%lu "
           "steer_last_allowed_to_inhibited_ms=%lu "
           "steer_safe_stop_pending=%s steer_commission_state=%u(%s) "
           "steer_commission_axis_mask=0x%02x steer_commission_nmt_sent_mask=0x%02x "
           "steer_commission_auth_clears=%lu "
           "post_tpdo_pending=%s post_tpdo_axis=0x%02x post_tpdo_missing=0x%02x "
           "post_tpdo_timeouts=%lu can2_rt_recover=%lu can2_rt_consecutive_fail=%lu "
           "can2_rt_last_recovery_ms=%lu "
           "presteer_hold=%s presteer_ready=%s presteer_mode=%u(%s) "
           "presteer_missing=0x%02x presteer_timeouts=%lu\r\n",
           bool_text(snapshot->steer_normal_pdo_allowed),
           bool_text(snapshot->steer_safety_inhibited),
           (unsigned int)snapshot->steer_inhibit_reason,
           steer_inhibit_reason_text(snapshot->steer_inhibit_reason),
           (unsigned long)snapshot->steer_safety_inhibit_count,
           (unsigned long)snapshot->steer_last_allowed_to_inhibited_ms,
           bool_text(snapshot->steer_safe_stop_pending),
           (unsigned int)snapshot->steer_commission_state,
           steer_commission_state_text(snapshot->steer_commission_state),
           (unsigned int)snapshot->steer_commission_axis_mask,
           (unsigned int)snapshot->steer_commission_nmt_sent_mask,
           (unsigned long)snapshot->steer_commission_authorization_clear_count,
           bool_text(snapshot->steer_commission_post_command_tpdo_pending),
           (unsigned int)snapshot->steer_commission_post_command_axis_mask,
           (unsigned int)snapshot->steer_commission_post_command_missing_mask,
           (unsigned long)snapshot->steer_commission_post_command_timeout_count,
           (unsigned long)snapshot->can2_realtime_transient_recovery_count,
           (unsigned long)snapshot->can2_realtime_consecutive_failure_count,
           (unsigned long)snapshot->can2_realtime_last_recovery_ms,
           bool_text(snapshot->presteer_drive_hold_active),
           bool_text(snapshot->presteer_target_reached),
           (unsigned int)snapshot->presteer_mode,
           motion_mode_text((ecu_motion_mode_t)snapshot->presteer_mode),
           (unsigned int)snapshot->presteer_missing_axis_mask,
           (unsigned long)snapshot->presteer_timeout_count);

    printf("[ECU STEER PLAN] active=%s done=%s stale_reject=%s id=%lu "
           "fresh=0x%02x moving=0x%02x max_dist=%ld "
           "actual=[%ld,%ld,%ld,%ld] out=[%ld,%ld,%ld,%ld] "
           "err=[%ld,%ld,%ld,%ld]\r\n",
           bool_text(snapshot->steer_transition_active),
           bool_text(snapshot->steer_transition_completed),
           bool_text(snapshot->steer_transition_rejected_stale_feedback),
           (unsigned long)snapshot->steer_transition_id,
           (unsigned int)snapshot->steer_transition_feedback_fresh_mask,
           (unsigned int)snapshot->steer_transition_moving_axis_mask,
           (long)snapshot->steer_transition_max_distance_counts,
           (long)snapshot->steer_transition_actual_counts[0],
           (long)snapshot->steer_transition_actual_counts[1],
           (long)snapshot->steer_transition_actual_counts[2],
           (long)snapshot->steer_transition_actual_counts[3],
           (long)snapshot->steer_transition_output_counts[0],
           (long)snapshot->steer_transition_output_counts[1],
           (long)snapshot->steer_transition_output_counts[2],
           (long)snapshot->steer_transition_output_counts[3],
           (long)snapshot->steer_transition_error_counts[0],
           (long)snapshot->steer_transition_error_counts[1],
           (long)snapshot->steer_transition_error_counts[2],
           (long)snapshot->steer_transition_error_counts[3]);

    printf("[ECU STEER CAL] ram_override=%s valid=%s seq=%lu",
           bool_text(snapshot->steer_calibration_ram_override_enabled),
           bool_text(snapshot->steer_calibration_ram_override_valid),
           (unsigned long)snapshot->steer_calibration_ram_override_sequence);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        const steer_axis_calibration_t *axis =
            &snapshot->steer_effective_calibration[wheel];
        printf(" w%lu[valid=%s sign=%d zero=%ld min=%ld max=%ld max_deg=",
               (unsigned long)(wheel + 1U),
               bool_text(axis->valid),
               (int)axis->direction_sign,
               (long)axis->straight_zero_offset_counts,
               (long)axis->minimum_position_counts,
               (long)axis->maximum_position_counts);
        print_centi_value((int32_t)(axis->commissioning_max_abs_deg * 100.0f));
        printf("]");
    }
    printf("\r\n");

    printf("[ECU CANopen CAN3] init=%s state=%u normal=%s bitrate=%lu local=%u remote=%u "
           "proc=%lu sdo_ok=%lu sdo_abort=%lu dl_ok=%lu dl_abort=%lu queued=%lu dropped=%lu "
           "last_node=%u last=0x%04x:%u size=%u value=0x%08lx abort=0x%08lx last_err=%ld\r\n",
           bool_text(snapshot->can3_canopen_initialized),
           (unsigned int)snapshot->can3_canopen_snapshot.state,
           bool_text(snapshot->can3_canopen_snapshot.can_normal),
           (unsigned long)snapshot->can3_canopen_snapshot.bitrate,
           (unsigned int)snapshot->can3_canopen_snapshot.local_node_id,
           (unsigned int)snapshot->can3_canopen_snapshot.remote_node_id,
           (unsigned long)snapshot->can3_canopen_snapshot.process_count,
           (unsigned long)snapshot->can3_canopen_snapshot.sdo_upload_count,
           (unsigned long)snapshot->can3_canopen_snapshot.sdo_abort_count,
           (unsigned long)snapshot->can3_canopen_snapshot.sdo_download_count,
           (unsigned long)snapshot->can3_canopen_snapshot.sdo_download_abort_count,
           (unsigned long)snapshot->can3_canopen_snapshot.queued_command_count,
           (unsigned long)snapshot->can3_canopen_snapshot.dropped_command_count,
           (unsigned int)snapshot->can3_canopen_snapshot.last_sdo_node_id,
           (unsigned int)snapshot->can3_canopen_snapshot.last_sdo_index,
           (unsigned int)snapshot->can3_canopen_snapshot.last_sdo_subindex,
           (unsigned int)snapshot->can3_canopen_snapshot.last_sdo_size,
           (unsigned long)snapshot->can3_canopen_snapshot.last_sdo_value,
           (unsigned long)snapshot->can3_canopen_snapshot.last_sdo_abort_code,
           (long)snapshot->can3_canopen_snapshot.last_error);

    printf("[ECU CANopen PDO CAN3] queued=%lu dropped=%lu tx=%lu tx_err=%lu "
           "pdo_group_state=%u pdo_group=%lu pdo_expected_frames=%u "
           "pdo_tx_complete_frames=%u pdo_failed_frames=%u pdo_in_flight_frames=%u "
           "pdo_arm_complete_frames=%u pdo_trigger_complete_frames=%u "
           "last_pdo_tx_complete_ms=%lu last_pdo_tx_timeout_ms=%lu "
           "last_tx[group=%lu cob=0x%03x node=%u phase=%u] "
           "last_fail[group=%lu cob=0x%03x node=%u phase=%u err=%ld]\r\n",
           (unsigned long)snapshot->can3_canopen_snapshot.pdo_queued_count,
           (unsigned long)snapshot->can3_canopen_snapshot.pdo_dropped_count,
           (unsigned long)snapshot->can3_canopen_snapshot.pdo_tx_count,
           (unsigned long)snapshot->can3_canopen_snapshot.pdo_tx_error_count,
           (unsigned int)snapshot->can3_canopen_snapshot.pdo_group_state,
           (unsigned long)snapshot->can3_canopen_snapshot.pdo_group_sequence,
           (unsigned int)snapshot->can3_canopen_snapshot.pdo_expected_frames,
           (unsigned int)snapshot->can3_canopen_snapshot.pdo_tx_complete_frames,
           (unsigned int)snapshot->can3_canopen_snapshot.pdo_failed_frames,
           (unsigned int)snapshot->can3_canopen_snapshot.pdo_in_flight_frames,
           (unsigned int)snapshot->can3_canopen_snapshot.pdo_arm_complete_frames,
           (unsigned int)snapshot->can3_canopen_snapshot.pdo_trigger_complete_frames,
           (unsigned long)snapshot->can3_canopen_snapshot.last_pdo_tx_complete_ms,
           (unsigned long)snapshot->can3_canopen_snapshot.last_pdo_tx_timeout_ms,
           (unsigned long)snapshot->can3_canopen_snapshot.last_pdo_tx_group_sequence,
           (unsigned int)snapshot->can3_canopen_snapshot.last_pdo_tx_cob_id,
           (unsigned int)snapshot->can3_canopen_snapshot.last_pdo_tx_node_id,
           (unsigned int)snapshot->can3_canopen_snapshot.last_pdo_tx_phase,
           (unsigned long)snapshot->can3_canopen_snapshot.last_pdo_failed_group_sequence,
           (unsigned int)snapshot->can3_canopen_snapshot.last_pdo_failed_cob_id,
           (unsigned int)snapshot->can3_canopen_snapshot.last_pdo_failed_node_id,
           (unsigned int)snapshot->can3_canopen_snapshot.last_pdo_failed_phase,
           (long)snapshot->can3_canopen_snapshot.last_pdo_error);

    printf("[ECU CAN3 LIFT TPDO]");
    for (uint8_t node = ECU_CANOPEN_LIFT_FR_NODE_ID;
         node <= ECU_CANOPEN_HYDRAULIC_PUMP_NODE_ID;
         ++node) {
        const canopen_node_feedback_t *feedback =
            &snapshot->can3_canopen_snapshot.node_feedback[node];
        printf(" n%u[fresh=%s p0=%s/%lu@%lu p1=%s/%lu@%lu pos=%ld vel=%ld "
               "fault=0x%08lx sw=0x%04x cur=%d mal=%lu]",
               (unsigned int)node,
               bool_text(feedback->feedback_fresh),
               bool_text(feedback->tpdo0_valid),
               (unsigned long)feedback->tpdo0_rx_count,
               (unsigned long)feedback->last_tpdo0_ms,
               bool_text(feedback->tpdo1_valid),
               (unsigned long)feedback->tpdo1_rx_count,
               (unsigned long)feedback->last_tpdo1_ms,
               (long)feedback->actual_position_counts,
               (long)feedback->actual_velocity_units,
               (unsigned long)feedback->fault_latched,
               (unsigned int)feedback->statusword,
               (int)feedback->actual_current_raw,
               (unsigned long)feedback->malformed_tpdo_count);
    }
    printf("\r\n");

    printf("[ECU CAN1] tx=%lu rx=%lu err=%lu last_tx_id=0x%08lx ext=%s dlc=%u "
           "last_rx_id=0x%08lx ext=%s dlc=%u data=[",
           (unsigned long)snapshot->can1_tx_count,
           (unsigned long)snapshot->can1_rx_count,
           (unsigned long)snapshot->can1_error_count,
           (unsigned long)snapshot->can1_last_tx_id,
           bool_text(snapshot->can1_last_tx_extended),
           (unsigned int)snapshot->can1_last_tx_size,
           (unsigned long)snapshot->can1_last_rx_id,
           bool_text(snapshot->can1_last_rx_extended),
           (unsigned int)snapshot->can1_last_rx_size);
    for (uint32_t byte = 0U; byte < snapshot->can1_last_rx_size && byte < 8U; ++byte) {
        if (byte > 0U) {
            printf(" ");
        }
        printf("%02x", snapshot->can1_last_rx_data[byte]);
    }
    printf("]\r\n");

#if ECU_ENABLE_CAN4_PHYSICAL_TEST_TX
    printf("[ECU CAN4 TEST] tx=%lu rx=%lu err=%lu rbuf=%u flags=0x%02x eflags=0x%02x "
           "rec=%u tec=%u lek=%u last_tx_id=0x%03lx dlc=%u data=[",
           (unsigned long)snapshot->can4_test_tx_count,
           (unsigned long)snapshot->can4_test_rx_count,
           (unsigned long)snapshot->can4_test_error_count,
           (unsigned int)snapshot->can4_test_rx_buffer_status,
           (unsigned int)snapshot->can4_test_tx_rx_flags,
           (unsigned int)snapshot->can4_test_error_flags,
           (unsigned int)snapshot->can4_test_receive_error_count,
           (unsigned int)snapshot->can4_test_transmit_error_count,
           (unsigned int)snapshot->can4_test_last_error_kind,
           (unsigned long)snapshot->can4_test_last_tx_id,
           (unsigned int)snapshot->can4_test_last_tx_size);
    for (uint32_t byte = 0U; byte < snapshot->can4_test_last_tx_size && byte < 8U; ++byte) {
        if (byte > 0U) {
            printf(" ");
        }
        printf("%02x", snapshot->can4_test_last_tx_data[byte]);
    }
    printf("]\r\n");
#endif

    printf("[ECU POWER] hv_req=%s ready=%s lv_ok=%s online[bms=%s dcdc48=%s dcdc12=%s dcac=%s] "
           "bms_soc=%u.%u%% bms_v=%lumV bms_i=%lddA bms_err=%u "
           "dcdc48_v=%lumV dcdc48_i=%lumA dcdc48_err=%u "
           "dcdc12_v=%lumV dcdc12_i=%lumA dcdc12_fault=%s "
           "dcac_out=%lumV dcac_in=%lumV tx_cmd=%lu\r\n",
           bool_text(snapshot->power_snapshot.high_voltage_requested),
           bool_text(snapshot->power_snapshot.power_ready),
           bool_text(snapshot->power_snapshot.low_voltage_ok),
           bool_text(snapshot->power_snapshot.bms_online),
           bool_text(snapshot->power_snapshot.dcdc48_online),
           bool_text(snapshot->power_snapshot.dcdc12_online),
           bool_text(snapshot->power_snapshot.dcac_online),
           (unsigned int)(snapshot->power_snapshot.bms.soc_half_percent / 2U),
           (unsigned int)((snapshot->power_snapshot.bms.soc_half_percent & 1U) ? 5U : 0U),
           (unsigned long)snapshot->power_snapshot.bms.pack_voltage_dv * 100UL,
           (long)snapshot->power_snapshot.bms.pack_current_da,
           (unsigned int)snapshot->power_snapshot.bms_error.level,
           (unsigned long)snapshot->power_snapshot.dcdc48.output_voltage_mv,
           (unsigned long)snapshot->power_snapshot.dcdc48.output_current_ma,
           (unsigned int)snapshot->power_snapshot.dcdc48.error_code,
           (unsigned long)snapshot->power_snapshot.dcdc12.output_voltage_mv,
           (unsigned long)snapshot->power_snapshot.dcdc12.output_current_ma,
           bool_text(snapshot->power_snapshot.dcdc12.total_fault),
           (unsigned long)snapshot->power_snapshot.dcac.output_voltage_mv,
           (unsigned long)snapshot->power_snapshot.dcac_input.input_voltage_mv,
           (unsigned long)snapshot->power_snapshot.command_tx_count);

    printf("[ECU MODBUS ADC] init=%s state=%u tx=%lu rx=%lu timeout=%lu err=%lu "
           "online=%s last=%lums raw=[",
           bool_text(snapshot->modbus_adc_master.initialized),
           (unsigned int)snapshot->modbus_adc_master.state,
           (unsigned long)snapshot->modbus_adc_master.tx_count,
           (unsigned long)snapshot->modbus_adc_master.rx_count,
           (unsigned long)snapshot->modbus_adc_master.timeout_count,
           (unsigned long)snapshot->modbus_adc_master.error_count,
           bool_text(snapshot->analog_modbus_adc.online),
           (unsigned long)snapshot->analog_modbus_adc.last_response_ms);
    for (uint32_t channel = 0U; channel < ECU_ADC_CHANNEL_COUNT; ++channel) {
        if (channel > 0U) {
            printf(",");
        }
        printf("%u", (unsigned int)snapshot->analog_modbus_adc.raw[channel]);
    }
    printf("] mv=[");
    for (uint32_t channel = 0U; channel < ECU_ADC_CHANNEL_COUNT; ++channel) {
        if (channel > 0U) {
            printf(",");
        }
        printf("%lu", (unsigned long)snapshot->analog_modbus_adc.millivolt[channel]);
    }
    printf("]\r\n");

    printf("[ECU HW] power_ready=%s low_voltage=%s can1=%s can2=%s can3=%s "
           "brake_release_fb=%s hydraulic_stopped=%s zero_speed=%s\r\n",
           bool_text(snapshot->hardware_feedback.power_ready),
           bool_text(snapshot->hardware_feedback.low_voltage_ok),
           bool_text(snapshot->hardware_feedback.can1_power_online),
           bool_text(snapshot->hardware_feedback.can2_motion_online),
           bool_text(snapshot->hardware_feedback.can3_lift_hydraulic_online),
           bool_text(snapshot->hardware_feedback.brake_release_confirmed),
           bool_text(snapshot->hardware_feedback.hydraulic_stopped),
           bool_text(snapshot->hardware_feedback.zero_speed_confirmed));

    printf("[ECU CMD] src=%s mode=%s gear=%s speed=",
           source_text(snapshot->source),
           motion_mode_text(snapshot->motion_mode),
           gear_text(snapshot->active_gear));
    print_milli_value(snapshot->target_speed_milli_mps);
    printf("m/s steer=[");
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (wheel > 0U) {
            printf(",");
        }
        print_centi_value(snapshot->target_steer_centi_deg[wheel]);
    }
    printf("]deg brake=%s hv=%s hv_latch=%s comm_hv=%s hyd=%s "
           "valve_cmd=0x%08lx valve_req=0x%08lx valve_out=0x%08lx "
           "valve_block=0x%08lx valve_block_cnt=%lu "
           "pump[state=%u fb=%s vel=%ld timeout=%lu] "
           "res[pwr=%s mot=%s lift=%s io=%s warn=%s]\r\n",
           bool_text(snapshot->brake_release),
           bool_text(snapshot->high_voltage_enable),
           bool_text(snapshot->high_voltage_relay_latched),
           bool_text(snapshot->commissioning_power_debug_active),
           bool_text(snapshot->hydraulic_enable),
           (unsigned long)snapshot->hydraulic_valve_mask,
           (unsigned long)snapshot->hydraulic_requested_valve_mask,
           (unsigned long)snapshot->hydraulic_applied_valve_mask,
           (unsigned long)snapshot->hydraulic_interlocked_valve_mask,
           (unsigned long)snapshot->hydraulic_valve_interlock_reject_count,
           (unsigned int)snapshot->hydraulic_pump_state,
           bool_text(snapshot->hydraulic_pump_feedback_valid),
           (long)snapshot->hydraulic_pump_actual_velocity_units,
           (unsigned long)snapshot->hydraulic_pump_start_timeout_count,
           device_result_text(snapshot->power_result),
           device_result_text(snapshot->motion_result),
           device_result_text(snapshot->lift_hydraulic_result),
           device_result_text(snapshot->local_io_result),
           device_result_text(snapshot->warning_light_result));
#endif
}
