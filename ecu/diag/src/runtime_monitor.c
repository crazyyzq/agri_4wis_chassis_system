#include <stdio.h>

#include "ecu_config.h"
#include "runtime_monitor.h"

static const char *bool_text(bool value)
{
    return value ? "1" : "0";
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

void runtime_monitor_print_cpu0(const runtime_monitor_snapshot_t *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    printf("[ECU MON] t=%lu seq=%lu sbus_valid=%s sbus_conn=%s fs=%s "
           "frames=%lu dec_err=%lu link=%s estop=%s diag=%s\r\n",
           (unsigned long)snapshot->now_ms,
           (unsigned long)snapshot->executor_sequence,
           bool_text(snapshot->sbus_valid),
           bool_text(snapshot->sbus_connected),
           bool_text(snapshot->sbus_failsafe),
           (unsigned long)snapshot->sbus_frame_count,
           (unsigned long)snapshot->sbus_decode_error_count,
           link_state_text(snapshot->link_state),
           estop_state_text(snapshot->estop_state),
           diag_code_name(snapshot->diagnostic));

#if ECU_DEBUG_MONITOR_VERBOSE
    printf("[ECU SBUS] ch=[");
    for (uint32_t channel = 0U; channel < ECU_SBUS_CHANNEL_COUNT; ++channel) {
        if (channel > 0U) {
            printf(",");
        }
        printf("%u", (unsigned int)snapshot->sbus_channels[channel]);
    }
    printf("] ch17=%s ch18=%s lost=%s uart_err=%lu last=%lums\r\n",
           bool_text(snapshot->sbus_channel17),
           bool_text(snapshot->sbus_channel18),
           bool_text(snapshot->sbus_frame_lost),
           (unsigned long)snapshot->sbus_uart_error_count,
           (unsigned long)snapshot->sbus_last_frame_ms);

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
           "last=0x%04x:%u size=%u value=0x%08lx abort=0x%08lx err=%ld\r\n",
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
    print_centi_value(snapshot->target_speed_centi_kph);
    printf("kph steer=[");
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (wheel > 0U) {
            printf(",");
        }
        print_centi_value(snapshot->target_steer_centi_deg[wheel]);
    }
    printf("]deg brake=%s hv=%s hyd=%s valve=0x%08lx "
           "res[pwr=%s mot=%s lift=%s io=%s warn=%s]\r\n",
           bool_text(snapshot->brake_release),
           bool_text(snapshot->high_voltage_enable),
           bool_text(snapshot->hydraulic_enable),
           (unsigned long)snapshot->hydraulic_valve_mask,
           device_result_text(snapshot->power_result),
           device_result_text(snapshot->motion_result),
           device_result_text(snapshot->lift_hydraulic_result),
           device_result_text(snapshot->local_io_result),
           device_result_text(snapshot->warning_light_result));
#endif
}
