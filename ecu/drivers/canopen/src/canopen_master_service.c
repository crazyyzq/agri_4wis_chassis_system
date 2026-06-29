#include "canopen_master_service.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board.h"
#include "ecu_config.h"
#include "hpm_common.h"
#include "hpm_canopen.h"
#include "hpm_interrupt.h"
#include "task.h"
#include "user_config.h"
#include "OD.h"

#define CANOPEN_MASTER_CAN2_INDEX (0U)
#define CANOPEN_MASTER_CAN3_INDEX (1U)
#define CANOPEN_MASTER_FIRST_HB_TIME_MS (0U)
#define CANOPEN_MASTER_SDO_SERVER_TIMEOUT_MS (1000U)
#define CANOPEN_MASTER_SDO_CLIENT_TIMEOUT_MS (500U)
#define CANOPEN_MASTER_NMT_CONTROL (CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

typedef struct {
    uint16_t index;
    uint8_t subindex;
} canopen_master_sdo_query_t;

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

/* HPM SDK CO_driver.c reports driver errors through this symbol. The service
 * selects the active stack before calling any CANopenNode API that may use it.
 */
CO_t *co;

static CO_t *s_canopen[CANOPEN_MASTER_BUS_COUNT];
static struct canopen_context s_canopen_context[CANOPEN_MASTER_BUS_COUNT];
hpm_can_config_t hpm_canopen_config[MAX_CANOPEN_DEVICE] = {0};
hpm_can_data_t hpm_canopen_data[MAX_CANOPEN_DEVICE] = {0};
struct device hpm_canopen_dev[MAX_CANOPEN_DEVICE] = {0};
volatile hpm_master_receive_buf_t canopen_rx_buf = {0};
ATTR_PLACE_AT_NONCACHEABLE_BSS
volatile canopen_master_debug_control_t g_canopen_master_debug_control;

static can_info_t s_can_info[CANOPEN_MASTER_BUS_COUNT] = {
    {
        .can_base = BOARD_CAN2_BASE,
        .irq_num = BOARD_CAN2_IRQn,
        .priority = ECU_SBUS_UART_IRQ_PRIORITY,
    },
    {
        .can_base = BOARD_CAN3_BASE,
        .irq_num = BOARD_CAN3_IRQn,
        .priority = ECU_SBUS_UART_IRQ_PRIORITY,
    }
};

SDK_DECLARE_EXT_ISR_M(BOARD_CAN2_IRQn, canopen_master_service_can2_isr)
void canopen_master_service_can2_isr(void)
{
    canopen_irq_handler((struct device *)&hpm_canopen_dev[CANOPEN_MASTER_CAN2_INDEX]);
}

SDK_DECLARE_EXT_ISR_M(BOARD_CAN3_IRQn, canopen_master_service_can3_isr)
void canopen_master_service_can3_isr(void)
{
    canopen_irq_handler((struct device *)&hpm_canopen_dev[CANOPEN_MASTER_CAN3_INDEX]);
}

static uint8_t bus_to_index(canopen_master_bus_t bus)
{
    return bus == CANOPEN_MASTER_BUS_CAN3 ?
           (uint8_t)CANOPEN_MASTER_CAN3_INDEX :
           (uint8_t)CANOPEN_MASTER_CAN2_INDEX;
}

static void select_stack(const canopen_master_service_t *service)
{
    if (service != NULL && service->can_index < CANOPEN_MASTER_BUS_COUNT) {
        co = s_canopen[service->can_index];
    }
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

static void note_error(canopen_master_service_t *service, int32_t error)
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

static bool valid_sdo_size(uint8_t size)
{
    return size == 1U || size == 2U || size == 4U;
}

static bool debug_command_to_nmt(canopen_master_debug_command_t command,
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

static bool make_debug_sdo_request(const canopen_master_service_t *service,
                                   const canopen_master_debug_control_t *control,
                                   canopen_master_sdo_write_request_t *out)
{
    if (service == NULL || control == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->node_id = control->node_id != 0U ?
                   control->node_id :
                   service->snapshot.remote_node_id;

    switch (control->command) {
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_CONTROLWORD:
        out->index = ECU_CANOPEN_OBJ_CONTROLWORD;
        out->size = 2U;
        out->value = (int32_t)control->controlword;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_MODE:
        out->index = ECU_CANOPEN_OBJ_MODES_OF_OPERATION;
        out->size = 1U;
        out->value = (int32_t)control->mode;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_POSITION:
        out->index = ECU_CANOPEN_OBJ_TARGET_POSITION;
        out->size = 4U;
        out->value = control->target_position;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_VELOCITY:
        out->index = ECU_CANOPEN_OBJ_TARGET_VELOCITY;
        out->size = 4U;
        out->value = control->target_velocity;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_TORQUE:
        out->index = ECU_CANOPEN_OBJ_TARGET_TORQUE;
        out->size = 2U;
        out->value = (int32_t)control->target_torque;
        return true;
    case CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_GENERIC:
        out->index = control->index;
        out->subindex = control->subindex;
        out->size = control->size;
        out->value = control->value;
        return out->index != 0U && valid_sdo_size(out->size);
    default:
        return false;
    }
}

static bool pop_queued_sdo(canopen_master_service_t *service,
                           canopen_master_sdo_write_request_t *out)
{
    bool ok = false;

    if (service == NULL || out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    if (service->command_queue_count > 0U) {
        *out = service->command_queue[service->command_queue_tail];
        service->command_queue_tail =
            (uint8_t)((service->command_queue_tail + 1U) %
                      CANOPEN_MASTER_COMMAND_QUEUE_CAPACITY);
        service->command_queue_count--;
        service->snapshot.queued_command_count = service->command_queue_count;
        ok = true;
    }
    taskEXIT_CRITICAL();
    return ok;
}

static bool pop_queued_sdo_read(canopen_master_service_t *service,
                                canopen_master_sdo_read_request_t *out)
{
    bool ok = false;

    if (service == NULL || out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    if (service->read_queue_count > 0U) {
        *out = service->read_queue[service->read_queue_tail];
        service->read_queue_tail =
            (uint8_t)((service->read_queue_tail + 1U) %
                      CANOPEN_MASTER_READ_QUEUE_CAPACITY);
        service->read_queue_count--;
        ok = true;
    }
    taskEXIT_CRITICAL();
    return ok;
}

bool canopen_master_service_request_sdo_write(canopen_master_service_t *service,
                                              uint8_t node_id,
                                              uint16_t index,
                                              uint8_t subindex,
                                              uint8_t size,
                                              int32_t value)
{
    canopen_master_sdo_write_request_t request;

    if (service == NULL || !service->snapshot.initialized ||
        node_id == 0U || index == 0U || !valid_sdo_size(size)) {
        return false;
    }

    request.node_id = node_id;
    request.index = index;
    request.subindex = subindex;
    request.size = size;
    request.value = value;

    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < service->command_queue_count; ++i) {
        uint8_t slot = (uint8_t)((service->command_queue_tail + i) %
                                 CANOPEN_MASTER_COMMAND_QUEUE_CAPACITY);
        canopen_master_sdo_write_request_t *queued =
            &service->command_queue[slot];
        if (queued->node_id == node_id &&
            queued->index == index &&
            queued->subindex == subindex) {
            *queued = request;
            taskEXIT_CRITICAL();
            return true;
        }
    }

    if (service->command_queue_count >= CANOPEN_MASTER_COMMAND_QUEUE_CAPACITY) {
        service->snapshot.dropped_command_count++;
        taskEXIT_CRITICAL();
        return false;
    }

    service->command_queue[service->command_queue_head] = request;
    service->command_queue_head =
        (uint8_t)((service->command_queue_head + 1U) %
                  CANOPEN_MASTER_COMMAND_QUEUE_CAPACITY);
    service->command_queue_count++;
    service->snapshot.queued_command_count = service->command_queue_count;
    taskEXIT_CRITICAL();
    return true;
}

bool canopen_master_service_request_sdo_read(canopen_master_service_t *service,
                                             uint8_t node_id,
                                             uint16_t index,
                                             uint8_t subindex)
{
    canopen_master_sdo_read_request_t request;

    if (service == NULL || !service->snapshot.initialized ||
        node_id == 0U || index == 0U) {
        return false;
    }

    request.node_id = node_id;
    request.index = index;
    request.subindex = subindex;

    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < service->read_queue_count; ++i) {
        uint8_t slot = (uint8_t)((service->read_queue_tail + i) %
                                 CANOPEN_MASTER_READ_QUEUE_CAPACITY);
        canopen_master_sdo_read_request_t *queued = &service->read_queue[slot];
        if (queued->node_id == node_id &&
            queued->index == index &&
            queued->subindex == subindex) {
            taskEXIT_CRITICAL();
            return true;
        }
    }

    if (service->read_queue_count >= CANOPEN_MASTER_READ_QUEUE_CAPACITY) {
        service->snapshot.dropped_command_count++;
        taskEXIT_CRITICAL();
        return false;
    }

    service->read_queue[service->read_queue_head] = request;
    service->read_queue_head =
        (uint8_t)((service->read_queue_head + 1U) %
                  CANOPEN_MASTER_READ_QUEUE_CAPACITY);
    service->read_queue_count++;
    taskEXIT_CRITICAL();
    return true;
}

bool canopen_master_service_request_nmt(canopen_master_service_t *service,
                                        uint8_t node_id,
                                        canopen_master_debug_command_t command)
{
    CO_NMT_command_t nmt_command = CO_NMT_NO_COMMAND;

    if (service == NULL || !service->snapshot.initialized ||
        node_id == 0U || !debug_command_to_nmt(command, &nmt_command)) {
        return false;
    }

    select_stack(service);
    CO_ReturnError_t result = CO_NMT_sendCommand(co->NMT, nmt_command, node_id);
    if (result == CO_ERROR_NO) {
        service->snapshot.nmt_command_count++;
        service->snapshot.last_error = 0;
        return true;
    }

    service->snapshot.command_error_count++;
    service->snapshot.last_error = (int32_t)result;
    return false;
}

static bool start_sdo_upload(canopen_master_service_t *service,
                             uint8_t node_id,
                             uint16_t index,
                             uint8_t subindex)
{
    if (service == NULL || co == NULL || co->SDOclient == NULL) {
        note_error(service, -1);
        return false;
    }

    service->active_sdo_node_id = node_id;
    service->active_sdo_index = index;
    service->active_sdo_subindex = subindex;

    CO_SDO_return_t setup_result =
        CO_SDOclient_setup(&co->SDOclient[0],
                           CO_CAN_ID_SDO_CLI + node_id,
                           CO_CAN_ID_SDO_SRV + node_id,
                           node_id);
    if (setup_result != CO_SDO_RT_ok_communicationEnd) {
        note_error(service, (int32_t)setup_result);
        return false;
    }

    CO_SDO_return_t upload_result =
        CO_SDOclientUploadInitiate(&co->SDOclient[0],
                                   index,
                                   subindex,
                                   ECU_CANOPEN_SDO_TIMEOUT_MS,
                                   false);
    if (upload_result != CO_SDO_RT_ok_communicationEnd) {
        note_error(service, (int32_t)upload_result);
        return false;
    }

    service->sdo_active = true;
    return true;
}

static bool start_sdo_download(canopen_master_service_t *service,
                               const canopen_master_sdo_write_request_t *request)
{
    uint8_t payload[4] = {0};

    if (service == NULL || request == NULL || co == NULL ||
        co->SDOclient == NULL || !valid_sdo_size(request->size)) {
        return false;
    }

    CO_SDO_return_t setup_result =
        CO_SDOclient_setup(&co->SDOclient[0],
                           CO_CAN_ID_SDO_CLI + request->node_id,
                           CO_CAN_ID_SDO_SRV + request->node_id,
                           request->node_id);
    if (setup_result != CO_SDO_RT_ok_communicationEnd) {
        service->snapshot.command_error_count++;
        service->snapshot.last_error = (int32_t)setup_result;
        return false;
    }

    CO_SDO_return_t initiate_result =
        CO_SDOclientDownloadInitiate(&co->SDOclient[0],
                                     request->index,
                                     request->subindex,
                                     request->size,
                                     ECU_CANOPEN_SDO_TIMEOUT_MS,
                                     false);
    if (initiate_result != CO_SDO_RT_ok_communicationEnd) {
        service->snapshot.command_error_count++;
        service->snapshot.last_error = (int32_t)initiate_result;
        CO_SDOclientClose(&co->SDOclient[0]);
        return false;
    }

    write_le_value(payload, request->size, request->value);
    if (CO_SDOclientDownloadBufWrite(&co->SDOclient[0],
                                     payload,
                                     request->size) != request->size) {
        service->snapshot.command_error_count++;
        service->snapshot.last_error = -3;
        CO_SDOclientClose(&co->SDOclient[0]);
        return false;
    }

    service->active_download_index = request->index;
    service->active_download_subindex = request->subindex;
    service->active_download_size = request->size;
    service->active_download_value = request->value;
    service->active_download_request = *request;
    service->sdo_download_active = true;
    return true;
}

static void finish_sdo_download(canopen_master_service_t *service,
                                uint32_t elapsed_us)
{
    CO_SDO_abortCode_t abort_code = CO_SDO_AB_NONE;
    size_t transferred = 0U;

    if (service == NULL || co == NULL || co->SDOclient == NULL) {
        return;
    }

    CO_SDO_return_t result = CO_SDOclientDownload(&co->SDOclient[0],
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
    CO_SDOclientClose(&co->SDOclient[0]);
    service->sdo_download_active = false;
}

static void finish_sdo_upload(canopen_master_service_t *service,
                              uint32_t elapsed_us)
{
    CO_SDO_abortCode_t abort_code = CO_SDO_AB_NONE;
    CO_SDO_return_t result = CO_SDOclientUpload(&co->SDOclient[0],
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
        service->snapshot.last_sdo_node_id = service->active_sdo_node_id;
        service->snapshot.last_sdo_index = service->active_sdo_index;
        service->snapshot.last_sdo_subindex = service->active_sdo_subindex;
        service->snapshot.sdo_abort_count++;
        service->snapshot.last_sdo_abort_code = (uint32_t)abort_code;
        service->snapshot.last_error = (int32_t)result;
    } else {
        uint8_t data[8] = {0};
        size_t read_size = CO_SDOclientUploadBufRead(&co->SDOclient[0],
                                                     data,
                                                     sizeof(data));
        service->snapshot.last_sdo_node_id = service->active_sdo_node_id;
        service->snapshot.last_sdo_index = service->active_sdo_index;
        service->snapshot.last_sdo_subindex = service->active_sdo_subindex;
        service->snapshot.sdo_upload_count++;
        service->snapshot.last_sdo_size = (uint8_t)read_size;
        service->snapshot.last_sdo_value = read_le_u32(data, read_size);
        service->snapshot.last_sdo_abort_code = 0U;
        service->snapshot.last_error = 0;
    }

    CO_SDOclientClose(&co->SDOclient[0]);
    service->sdo_active = false;
}

static bool copy_debug_control(canopen_master_debug_control_t *out)
{
    const volatile canopen_master_debug_control_t *src =
        &g_canopen_master_debug_control;
    uint32_t sequence_before = src->command_sequence;

    if (out == NULL) {
        return false;
    }

    out->command_sequence = sequence_before;
    out->command = src->command;
    out->bus = src->bus;
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

static void handle_debug_command(canopen_master_service_t *service)
{
    canopen_master_debug_control_t control;
    canopen_master_sdo_write_request_t request;
    CO_NMT_command_t nmt_command = CO_NMT_NO_COMMAND;

    if (service == NULL || co == NULL || !copy_debug_control(&control)) {
        return;
    }
    if (control.command_sequence == service->last_debug_sequence) {
        return;
    }
    if (control.bus != service->snapshot.bus) {
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

    if (debug_command_to_nmt(control.command, &nmt_command)) {
        (void)canopen_master_service_request_nmt(service,
                                                 service->snapshot.last_command_node_id,
                                                 control.command);
        return;
    }

    if (make_debug_sdo_request(service, &control, &request)) {
        (void)canopen_master_service_request_sdo_write(service,
                                                       request.node_id,
                                                       request.index,
                                                       request.subindex,
                                                       request.size,
                                                       request.value);
    } else {
        service->snapshot.command_error_count++;
        service->snapshot.last_error = -4;
    }
}

bool canopen_master_service_init(canopen_master_service_t *service,
                                 canopen_master_bus_t bus,
                                 uint32_t bitrate,
                                 uint8_t local_node_id,
                                 uint8_t remote_node_id,
                                 uint32_t now_ms)
{
    if (service == NULL || bus >= CANOPEN_MASTER_BUS_COUNT ||
        bitrate == 0U || local_node_id == 0U || remote_node_id == 0U) {
        return false;
    }

    memset(service, 0, sizeof(*service));
    g_canopen_master_debug_control.command = CANOPEN_MASTER_DEBUG_COMMAND_NONE;
    service->can_index = bus_to_index(bus);
    service->snapshot.bus = bus;
    service->snapshot.bitrate = bitrate;
    service->snapshot.local_node_id = local_node_id;
    service->snapshot.remote_node_id = remote_node_id;
    service->last_debug_sequence = g_canopen_master_debug_control.command_sequence;
    service->last_process_ms = now_ms;
    service->next_sdo_ms = now_ms + ECU_CANOPEN_SDO_PERIOD_MS;

    uint32_t heap_memory_used = 0U;
    s_canopen[service->can_index] = CO_new(NULL, &heap_memory_used);
    if (s_canopen[service->can_index] == NULL) {
        note_error(service, -2);
        return false;
    }
    select_stack(service);

    uint8_t physical_bus = bus == CANOPEN_MASTER_BUS_CAN3 ? 3U : 2U;
    bool termination_enable = bus == CANOPEN_MASTER_BUS_CAN3 ?
                              (ECU_CAN3_TERMINATION_ENABLE != 0) :
                              (ECU_CAN2_TERMINATION_ENABLE != 0);
    board_set_can_termination(physical_bus, termination_enable);
    canopen_controller_init(&s_canopen_context[service->can_index],
                            &s_can_info[service->can_index],
                            bitrate,
                            service->can_index);

    CO_CANsetConfigurationMode((void *)&s_canopen_context[service->can_index]);
    CO_CANmodule_disable(co->CANmodule);

    CO_ReturnError_t result = CO_CANinit(co,
                                         &s_canopen_context[service->can_index],
                                         (uint16_t)bitrate);
    if (result != CO_ERROR_NO) {
        note_error(service, (int32_t)result);
        return false;
    }

    uint32_t err_info = 0U;
    result = CO_CANopenInit(co,
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
        note_error(service, (int32_t)result);
        return false;
    }

    CO_CANsetNormalMode(co->CANmodule);
    service->snapshot.initialized = true;
    service->snapshot.can_normal = true;
    service->snapshot.state = CANOPEN_MASTER_STATE_RUNNING;
    service->snapshot.last_error = 0;
    (void)heap_memory_used;
    return true;
}

void canopen_master_service_process(canopen_master_service_t *service,
                                    uint32_t now_ms)
{
    if (service == NULL || !service->snapshot.initialized ||
        service->can_index >= CANOPEN_MASTER_BUS_COUNT) {
        return;
    }

    select_stack(service);
    if (co == NULL) {
        return;
    }

    uint32_t elapsed_us = elapsed_us_since(now_ms, &service->last_process_ms);
    uint32_t timer_next_us = 1000U;
    (void)CO_process(co, false, elapsed_us, &timer_next_us);
    service->snapshot.process_count++;

    /* Debug commands are sequence-gated and may be issued from a debugger while
     * the periodic diagnostic upload state machine is active.  Check them
     * before servicing the current SDO state so manual bench commands cannot be
     * starved by the cyclic upload sweep.
     */
    handle_debug_command(service);

    if (service->sdo_download_active) {
        finish_sdo_download(service, elapsed_us);
    } else if (service->sdo_active) {
        finish_sdo_upload(service, elapsed_us);
    } else {
        canopen_master_sdo_write_request_t request;
        canopen_master_sdo_read_request_t read_request;
        if (pop_queued_sdo(service, &request)) {
            (void)start_sdo_download(service, &request);
        } else if (pop_queued_sdo_read(service, &read_request)) {
            (void)start_sdo_upload(service,
                                   read_request.node_id,
                                   read_request.index,
                                   read_request.subindex);
        } else if (now_ms >= service->next_sdo_ms) {
            const canopen_master_sdo_query_t *query = &s_sdo_queries[service->next_query];
            service->next_sdo_ms = now_ms + ECU_CANOPEN_SDO_PERIOD_MS;
            (void)start_sdo_upload(service,
                                   service->snapshot.remote_node_id,
                                   query->index,
                                   query->subindex);
            service->next_query++;
            if (service->next_query >= (sizeof(s_sdo_queries) / sizeof(s_sdo_queries[0]))) {
                service->next_query = 0U;
            }
        }
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
