#include "canopen_master_service.h"

#include <stddef.h>
#include <string.h>

#include "board.h"
#include "ecu_config.h"
#include "hpm_canopen.h"
#include "hpm_interrupt.h"
#include "user_config.h"
#include "OD.h"

#define CANOPEN_MASTER_CAN_INDEX (0)
#define CANOPEN_MASTER_FIRST_HB_TIME_MS (0U)
#define CANOPEN_MASTER_SDO_SERVER_TIMEOUT_MS (1000U)
#define CANOPEN_MASTER_SDO_CLIENT_TIMEOUT_MS (500U)
#define CANOPEN_MASTER_NMT_CONTROL (CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

typedef struct {
    uint16_t index;
    uint8_t subindex;
} canopen_master_sdo_query_t;

typedef struct {
    uint32_t command_sequence;
    canopen_master_debug_command_t command;
    uint8_t node_id;
    uint16_t index;
    uint8_t subindex;
    uint8_t size;
    int32_t value;
} canopen_master_command_request_t;

static const canopen_master_sdo_query_t s_sdo_queries[] = {
    { ECU_CANOPEN_OBJ_DEVICE_TYPE, 0U },
    { ECU_CANOPEN_OBJ_ERROR_REGISTER, 0U },
    { ECU_CANOPEN_OBJ_IDENTITY, 1U },
    { ECU_CANOPEN_OBJ_IDENTITY, 2U },
    { ECU_CANOPEN_OBJ_IDENTITY, 3U },
    { ECU_CANOPEN_OBJ_IDENTITY, 4U },
    { ECU_CANOPEN_OBJ_STATUSWORD, 0U },
    { ECU_CANOPEN_OBJ_MODES_OF_OPERATION_DISPLAY, 0U }
};

/*
 * HPMicro's CANopenNode port references this application-level symbol from
 * CO_driver.c when it reports driver errors. Keep it bound to the same object
 * used by the diagnostic service instead of hiding the stack instance locally.
 */
CO_t *co;

static CO_t *s_canopen;
static struct canopen_context s_canopen_context;
hpm_can_config_t hpm_canopen_config[MAX_CANOPEN_DEVICE] = {0};
hpm_can_data_t hpm_canopen_data[MAX_CANOPEN_DEVICE] = {0};
struct device hpm_canopen_dev[MAX_CANOPEN_DEVICE] = {0};
volatile hpm_master_receive_buf_t canopen_rx_buf = {0};
volatile canopen_master_debug_control_t g_canopen_master_debug_control = {0};

static can_info_t s_can_info[MAX_CANOPEN_DEVICE] = {
    {
        .can_base = BOARD_CAN2_BASE,
        .irq_num = BOARD_CAN2_IRQn,
        .priority = ECU_SBUS_UART_IRQ_PRIORITY,
    }
};

SDK_DECLARE_EXT_ISR_M(BOARD_CAN2_IRQn, canopen_master_service_can2_isr)
void canopen_master_service_can2_isr(void)
{
    canopen_irq_handler((struct device *)&hpm_canopen_dev[CANOPEN_MASTER_CAN_INDEX]);
}

static uint32_t elapsed_us_since(uint32_t now_ms, uint32_t *previous_ms)
{
    uint32_t elapsed_ms = 1U;
    if (*previous_ms != 0U) {
        elapsed_ms = now_ms - *previous_ms;
        if (elapsed_ms == 0U) {
            elapsed_ms = 1U;
        }
    }
    *previous_ms = now_ms;
    return elapsed_ms * 1000U;
}

static void canopen_master_note_error(canopen_master_service_t *service,
                                      int32_t error)
{
    service->snapshot.state = CANOPEN_MASTER_STATE_ERROR;
    service->snapshot.last_error = error;
}

static uint32_t read_le_u32(const uint8_t *data, size_t size)
{
    uint32_t value = 0U;
    size_t limit = size < sizeof(value) ? size : sizeof(value);
    for (size_t i = 0U; i < limit; ++i) {
        value |= ((uint32_t)data[i]) << (8U * i);
    }
    return value;
}

static void write_le_value(uint8_t *data, uint8_t size, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    for (uint8_t i = 0U; i < size; ++i) {
        data[i] = (uint8_t)((raw >> (8U * i)) & 0xFFU);
    }
}

static void canopen_master_note_rx(canopen_master_service_t *service,
                                   uint32_t now_ms)
{
    if (canopen_rx_buf.has_received_message == 0U) {
        return;
    }
    canopen_rx_buf.has_received_message = 0U;

#ifdef HPMSOC_HAS_HPMSDK_MCAN
    uint32_t can_id = canopen_rx_buf.rx_buf.use_ext_id ?
                      canopen_rx_buf.rx_buf.ext_id :
                      canopen_rx_buf.rx_buf.std_id;
    uint8_t heartbeat_state = canopen_rx_buf.rx_buf.data_8[0];
#else
    uint32_t can_id = canopen_rx_buf.rx_buf.id;
    uint8_t heartbeat_state = canopen_rx_buf.rx_buf.data[0];
#endif

    if (can_id == (CO_CAN_ID_HEARTBEAT + service->snapshot.remote_node_id)) {
        service->snapshot.heartbeat_count++;
        service->snapshot.last_heartbeat_state = heartbeat_state;
        service->snapshot.last_heartbeat_ms = now_ms;
    }
}

static bool canopen_master_start_sdo(canopen_master_service_t *service)
{
    if (s_canopen == NULL || s_canopen->SDOclient == NULL) {
        canopen_master_note_error(service, -1);
        return false;
    }

    const canopen_master_sdo_query_t *query = &s_sdo_queries[service->next_query];
    service->active_sdo_index = query->index;
    service->active_sdo_subindex = query->subindex;

    CO_SDO_return_t setup_result =
        CO_SDOclient_setup(&s_canopen->SDOclient[0],
                           CO_CAN_ID_SDO_CLI + service->snapshot.remote_node_id,
                           CO_CAN_ID_SDO_SRV + service->snapshot.remote_node_id,
                           service->snapshot.remote_node_id);
    if (setup_result != CO_SDO_RT_ok_communicationEnd) {
        canopen_master_note_error(service, (int32_t)setup_result);
        return false;
    }

    CO_SDO_return_t upload_result =
        CO_SDOclientUploadInitiate(&s_canopen->SDOclient[0],
                                   query->index,
                                   query->subindex,
                                   ECU_CANOPEN_SDO_TIMEOUT_MS,
                                   false);
    if (upload_result != CO_SDO_RT_ok_communicationEnd) {
        canopen_master_note_error(service, (int32_t)upload_result);
        return false;
    }

    service->sdo_active = true;
    return true;
}

static bool canopen_master_copy_debug_control(canopen_master_debug_control_t *out)
{
    const volatile canopen_master_debug_control_t *src =
        &g_canopen_master_debug_control;
    uint32_t sequence_before = src->command_sequence;

    if (out == NULL) {
        return false;
    }

    out->command_sequence = sequence_before;
    out->command = src->command;
    out->node_id = src->node_id;
    out->index = src->index;
    out->subindex = src->subindex;
    out->size = src->size;
    out->value = src->value;
    out->controlword = src->controlword;
    out->mode = src->mode;
    out->target_position = src->target_position;
    out->target_velocity = src->target_velocity;
    out->target_torque = src->target_torque;

    return sequence_before == src->command_sequence;
}

static bool canopen_master_make_command_request(
    const canopen_master_service_t *service,
    const canopen_master_debug_control_t *control,
    canopen_master_command_request_t *out)
{
    if (service == NULL || control == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->command_sequence = control->command_sequence;
    out->command = control->command;
    out->node_id = control->node_id != 0U ?
                   control->node_id :
                   service->snapshot.remote_node_id;

    switch (control->command) {
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_CONTROLWORD:
        out->index = ECU_CANOPEN_OBJ_CONTROLWORD;
        out->subindex = 0U;
        out->size = 2U;
        out->value = (int32_t)control->controlword;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_MODE:
        out->index = ECU_CANOPEN_OBJ_MODES_OF_OPERATION;
        out->subindex = 0U;
        out->size = 1U;
        out->value = (int32_t)control->mode;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_POSITION:
        out->index = ECU_CANOPEN_OBJ_TARGET_POSITION;
        out->subindex = 0U;
        out->size = 4U;
        out->value = control->target_position;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_VELOCITY:
        out->index = ECU_CANOPEN_OBJ_TARGET_VELOCITY;
        out->subindex = 0U;
        out->size = 4U;
        out->value = control->target_velocity;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_TORQUE:
        out->index = ECU_CANOPEN_OBJ_TARGET_TORQUE;
        out->subindex = 0U;
        out->size = 2U;
        out->value = (int32_t)control->target_torque;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_GENERIC:
        out->index = control->index;
        out->subindex = control->subindex;
        out->size = control->size;
        out->value = control->value;
        return out->index != 0U &&
               (out->size == 1U || out->size == 2U || out->size == 4U);
    default:
        return false;
    }
}

static bool canopen_master_debug_command_to_nmt(
    canopen_master_debug_command_t command,
    CO_NMT_command_t *out)
{
    if (out == NULL) {
        return false;
    }

    switch (command) {
    case CANOPEN_MASTER_DEBUG_COMMAND_NMT_PRE_OPERATIONAL:
        *out = CO_NMT_ENTER_PRE_OPERATIONAL;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL:
        *out = CO_NMT_ENTER_OPERATIONAL;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_NMT_STOPPED:
        *out = CO_NMT_ENTER_STOPPED;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET_NODE:
        *out = CO_NMT_RESET_NODE;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET_COMMUNICATION:
        *out = CO_NMT_RESET_COMMUNICATION;
        return true;
    default:
        return false;
    }
}

static void canopen_master_start_sdo_download(
    canopen_master_service_t *service,
    const canopen_master_command_request_t *request)
{
    uint8_t payload[4] = {0};

    if (service == NULL || request == NULL ||
        s_canopen == NULL || s_canopen->SDOclient == NULL) {
        return;
    }

    CO_SDO_return_t setup_result =
        CO_SDOclient_setup(&s_canopen->SDOclient[0],
                           CO_CAN_ID_SDO_CLI + request->node_id,
                           CO_CAN_ID_SDO_SRV + request->node_id,
                           request->node_id);
    if (setup_result != CO_SDO_RT_ok_communicationEnd) {
        service->snapshot.command_error_count++;
        service->snapshot.last_error = (int32_t)setup_result;
        return;
    }

    CO_SDO_return_t initiate_result =
        CO_SDOclientDownloadInitiate(&s_canopen->SDOclient[0],
                                     request->index,
                                     request->subindex,
                                     request->size,
                                     ECU_CANOPEN_SDO_TIMEOUT_MS,
                                     false);
    if (initiate_result != CO_SDO_RT_ok_communicationEnd) {
        service->snapshot.command_error_count++;
        service->snapshot.last_error = (int32_t)initiate_result;
        CO_SDOclientClose(&s_canopen->SDOclient[0]);
        return;
    }

    write_le_value(payload, request->size, request->value);
    if (CO_SDOclientDownloadBufWrite(&s_canopen->SDOclient[0],
                                     payload,
                                     request->size) != request->size) {
        service->snapshot.command_error_count++;
        service->snapshot.last_error = -3;
        CO_SDOclientClose(&s_canopen->SDOclient[0]);
        return;
    }

    service->active_download_index = request->index;
    service->active_download_subindex = request->subindex;
    service->active_download_size = request->size;
    service->active_download_value = request->value;
    service->sdo_download_active = true;
}

static void canopen_master_finish_sdo_download(canopen_master_service_t *service,
                                               uint32_t elapsed_us)
{
    CO_SDO_abortCode_t abort_code = CO_SDO_AB_NONE;
    size_t transferred = 0U;

    if (service == NULL || s_canopen == NULL || s_canopen->SDOclient == NULL) {
        return;
    }

    CO_SDO_return_t result = CO_SDOclientDownload(&s_canopen->SDOclient[0],
                                                  elapsed_us,
                                                  false,
                                                  false,
                                                  &abort_code,
                                                  &transferred,
                                                  NULL);
    if (result > CO_SDO_RT_ok_communicationEnd) {
        return;
    }

    service->snapshot.last_download_index = service->active_download_index;
    service->snapshot.last_download_subindex = service->active_download_subindex;
    service->snapshot.last_download_size = service->active_download_size;
    service->snapshot.last_download_value = service->active_download_value;

    if (result < CO_SDO_RT_ok_communicationEnd) {
        service->snapshot.sdo_download_abort_count++;
        service->snapshot.last_download_abort_code = (uint32_t)abort_code;
        service->snapshot.command_error_count++;
        service->snapshot.last_error = (int32_t)result;
    } else {
        service->snapshot.sdo_download_count++;
        service->snapshot.last_download_abort_code = 0U;
        service->snapshot.last_error = 0;
    }

    (void)transferred;
    CO_SDOclientClose(&s_canopen->SDOclient[0]);
    service->sdo_download_active = false;
}

static void canopen_master_handle_debug_command(canopen_master_service_t *service)
{
    canopen_master_debug_control_t control;
    canopen_master_command_request_t request;
    CO_NMT_command_t nmt_command = CO_NMT_NO_COMMAND;

    if (service == NULL || s_canopen == NULL) {
        return;
    }
    if (!canopen_master_copy_debug_control(&control)) {
        return;
    }
    if (control.command_sequence == service->last_debug_sequence) {
        return;
    }

    service->last_debug_sequence = control.command_sequence;
    service->snapshot.last_command_sequence = control.command_sequence;
    service->snapshot.last_command = control.command;
    service->snapshot.last_command_node_id =
        control.node_id != 0U ? control.node_id : service->snapshot.remote_node_id;

    if (control.command == CANOPEN_MASTER_DEBUG_COMMAND_NONE) {
        return;
    }

    if (canopen_master_debug_command_to_nmt(control.command, &nmt_command)) {
        CO_ReturnError_t result =
            CO_NMT_sendCommand(s_canopen->NMT,
                               nmt_command,
                               service->snapshot.last_command_node_id);
        if (result == CO_ERROR_NO) {
            service->snapshot.nmt_command_count++;
            service->snapshot.last_error = 0;
        } else {
            service->snapshot.command_error_count++;
            service->snapshot.last_error = (int32_t)result;
        }
        return;
    }

    if (canopen_master_make_command_request(service, &control, &request)) {
        canopen_master_start_sdo_download(service, &request);
    } else {
        service->snapshot.command_error_count++;
        service->snapshot.last_error = -4;
    }
}

static void canopen_master_finish_sdo(canopen_master_service_t *service,
                                      uint32_t elapsed_us)
{
    CO_SDO_abortCode_t abort_code = CO_SDO_AB_NONE;
    CO_SDO_return_t result = CO_SDOclientUpload(&s_canopen->SDOclient[0],
                                                elapsed_us,
                                                false,
                                                &abort_code,
                                                NULL,
                                                NULL,
                                                NULL);
    if (result > CO_SDO_RT_ok_communicationEnd) {
        return;
    }

    if (result < CO_SDO_RT_ok_communicationEnd) {
        service->snapshot.last_sdo_index = service->active_sdo_index;
        service->snapshot.last_sdo_subindex = service->active_sdo_subindex;
        service->snapshot.sdo_abort_count++;
        service->snapshot.last_sdo_abort_code = (uint32_t)abort_code;
        service->snapshot.last_error = (int32_t)result;
    } else {
        uint8_t data[8] = {0};
        size_t read_size = CO_SDOclientUploadBufRead(&s_canopen->SDOclient[0],
                                                     data,
                                                     sizeof(data));
        service->snapshot.last_sdo_index = service->active_sdo_index;
        service->snapshot.last_sdo_subindex = service->active_sdo_subindex;
        service->snapshot.sdo_upload_count++;
        service->snapshot.last_sdo_size = (uint8_t)read_size;
        service->snapshot.last_sdo_value = read_le_u32(data, read_size);
        service->snapshot.last_sdo_abort_code = 0U;
        service->snapshot.last_error = 0;
    }

    CO_SDOclientClose(&s_canopen->SDOclient[0]);
    service->sdo_active = false;
    service->next_query++;
    if (service->next_query >= (sizeof(s_sdo_queries) / sizeof(s_sdo_queries[0]))) {
        service->next_query = 0U;
    }
}

bool canopen_master_service_init(canopen_master_service_t *service,
                                 uint32_t bitrate,
                                 uint8_t local_node_id,
                                 uint8_t remote_node_id,
                                 uint32_t now_ms)
{
    if (service == NULL || bitrate == 0U || local_node_id == 0U || remote_node_id == 0U) {
        return false;
    }

    memset(service, 0, sizeof(*service));
    g_canopen_master_debug_control.command = CANOPEN_MASTER_DEBUG_COMMAND_NONE;
    service->snapshot.bitrate = bitrate;
    service->snapshot.local_node_id = local_node_id;
    service->snapshot.remote_node_id = remote_node_id;
    service->last_debug_sequence =
        g_canopen_master_debug_control.command_sequence;
    service->last_process_ms = now_ms;
    service->next_sdo_ms = now_ms + ECU_CANOPEN_SDO_PERIOD_MS;

    uint32_t heap_memory_used = 0U;
    s_canopen = CO_new(NULL, &heap_memory_used);
    if (s_canopen == NULL) {
        canopen_master_note_error(service, -2);
        return false;
    }
    co = s_canopen;

    board_set_can_termination(2U, ECU_CAN2_TERMINATION_ENABLE != 0);
    canopen_controller_init(&s_canopen_context,
                            &s_can_info[CANOPEN_MASTER_CAN_INDEX],
                            bitrate,
                            CANOPEN_MASTER_CAN_INDEX);

    CO_CANsetConfigurationMode((void *)&s_canopen_context);
    CO_CANmodule_disable(s_canopen->CANmodule);

    CO_ReturnError_t result = CO_CANinit(s_canopen,
                                         &s_canopen_context,
                                         (uint16_t)bitrate);
    if (result != CO_ERROR_NO) {
        canopen_master_note_error(service, (int32_t)result);
        return false;
    }

    uint32_t err_info = 0U;
    result = CO_CANopenInit(s_canopen,
                            NULL,
                            NULL,
                            OD,
                            NULL,
                            CANOPEN_MASTER_NMT_CONTROL,
                            CANOPEN_MASTER_FIRST_HB_TIME_MS,
                            CANOPEN_MASTER_SDO_SERVER_TIMEOUT_MS,
                            CANOPEN_MASTER_SDO_CLIENT_TIMEOUT_MS,
                            false,
                            local_node_id,
                            &err_info);
    if (result != CO_ERROR_NO) {
        service->snapshot.last_sdo_abort_code = err_info;
        canopen_master_note_error(service, (int32_t)result);
        return false;
    }

    CO_CANsetNormalMode(s_canopen->CANmodule);
    service->snapshot.initialized = true;
    service->snapshot.can_normal = true;
    service->snapshot.state = CANOPEN_MASTER_STATE_RUNNING;
    service->snapshot.last_error = 0;
    return true;
}

void canopen_master_service_process(canopen_master_service_t *service,
                                    uint32_t now_ms)
{
    if (service == NULL || !service->snapshot.initialized || s_canopen == NULL) {
        return;
    }

    uint32_t elapsed_us = elapsed_us_since(now_ms, &service->last_process_ms);
    uint32_t timer_next_us = 1000U;
    (void)CO_process(s_canopen, false, elapsed_us, &timer_next_us);
    service->snapshot.process_count++;
    canopen_master_note_rx(service, now_ms);

    if (service->sdo_download_active) {
        canopen_master_finish_sdo_download(service, elapsed_us);
    } else if (service->sdo_active) {
        canopen_master_finish_sdo(service, elapsed_us);
    } else if (g_canopen_master_debug_control.command_sequence !=
               service->last_debug_sequence) {
        canopen_master_handle_debug_command(service);
    } else if (now_ms >= service->next_sdo_ms) {
        service->next_sdo_ms = now_ms + ECU_CANOPEN_SDO_PERIOD_MS;
        (void)canopen_master_start_sdo(service);
    }
}

void canopen_master_service_get_snapshot(const canopen_master_service_t *service,
                                         canopen_master_snapshot_t *out)
{
    if (service == NULL || out == NULL) {
        return;
    }
    *out = service->snapshot;
}
