#include "canopen_master_service.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board.h"
#include "ecu_config.h"
#include "hpm_common.h"
#include "hpm_canopen.h"
#include "hpm_interrupt.h"
#include "semphr.h"
#include "task.h"
#include "user_config.h"
#include "OD.h"

#define CANOPEN_MASTER_CAN2_INDEX (0U)
#define CANOPEN_MASTER_CAN3_INDEX (1U)
#define CANOPEN_MASTER_FIRST_HB_TIME_MS (0U)
#define CANOPEN_MASTER_SDO_SERVER_TIMEOUT_MS (1000U)
#define CANOPEN_MASTER_SDO_CLIENT_TIMEOUT_MS (500U)
#define CANOPEN_MASTER_NMT_CONTROL (CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
#define CANOPEN_MASTER_PDO_TX_COMPLETE_POLL_US (300U)

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

static bool debug_command_to_nmt(canopen_master_debug_command_t command,
                                 CO_NMT_command_t *out);

/* HPM SDK CO_driver.c reports driver errors through this symbol. The service
 * selects the active stack before calling any CANopenNode API that may use it.
 */
CO_t *co;

static CO_t *s_canopen[CANOPEN_MASTER_BUS_COUNT];
static SemaphoreHandle_t s_canopen_lock;
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

static volatile uint32_t s_canopen_pdo_tx_complete_count[CANOPEN_MASTER_BUS_COUNT];

static void note_canopen_tx_flags_from_isr(uint8_t can_index)
{
    if (can_index >= CANOPEN_MASTER_BUS_COUNT) {
        return;
    }

    uint8_t tx_rx_flags = can_get_tx_rx_flags(s_can_info[can_index].can_base);
    if ((tx_rx_flags & CAN_EVENT_TX_PRIMARY_BUF) != 0U) {
        /* Only the primary TX buffer is used by the realtime PDO scheduler.
         * The SDK IRQ handler clears this flag, so the service records it
         * before handing control to CANopenNode's HAL.
         */
        s_canopen_pdo_tx_complete_count[can_index]++;
    }
}

SDK_DECLARE_EXT_ISR_M(BOARD_CAN2_IRQn, canopen_master_service_can2_isr)
void canopen_master_service_can2_isr(void)
{
    note_canopen_tx_flags_from_isr(CANOPEN_MASTER_CAN2_INDEX);
    canopen_irq_handler((struct device *)&hpm_canopen_dev[CANOPEN_MASTER_CAN2_INDEX]);
}

SDK_DECLARE_EXT_ISR_M(BOARD_CAN3_IRQn, canopen_master_service_can3_isr)
void canopen_master_service_can3_isr(void)
{
    note_canopen_tx_flags_from_isr(CANOPEN_MASTER_CAN3_INDEX);
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

static void canopen_master_lock_init_once(void)
{
    taskENTER_CRITICAL();
    if (s_canopen_lock == NULL) {
        s_canopen_lock = xSemaphoreCreateMutex();
    }
    taskEXIT_CRITICAL();
}

static bool canopen_master_lock(void)
{
    canopen_master_lock_init_once();
    if (s_canopen_lock == NULL) {
        return false;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return true;
    }
    return xSemaphoreTake(s_canopen_lock, portMAX_DELAY) == pdTRUE;
}

static void canopen_master_unlock(void)
{
    if (s_canopen_lock != NULL &&
        xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        (void)xSemaphoreGive(s_canopen_lock);
    }
}

static bool canopen_master_service_request_nmt_locked(canopen_master_service_t *service,
                                                      uint8_t node_id,
                                                      canopen_master_debug_command_t command)
{
    CO_NMT_command_t nmt_command = CO_NMT_NO_COMMAND;

    if (service == NULL || co == NULL || !service->snapshot.initialized ||
        node_id == 0U || !debug_command_to_nmt(command, &nmt_command)) {
        return false;
    }

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

static bool canopen_master_sdo_write_requires_order(uint16_t index)
{
    /* CiA 402 control-word writes encode state transitions.  A sequence such as
     * shutdown -> switch on -> enable operation must reach the drive exactly in
     * that order; replacing it with only the final value can leave a drive in
     * Switch On Disabled.
     *
     * PDO mapping also contains value-dependent state transitions: map count
     * must be written to zero before new map entries are written, then restored
     * to the final count.  Coalescing those writes would silently leave the old
     * mapping active, which is unacceptable for the realtime steering RPDO.
     */
    return index == ECU_CANOPEN_OBJ_CONTROLWORD ||
           index == ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM ||
           index == ECU_CANOPEN_OBJ_RPDO1_MAPPING;
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

static void sync_pdo_group_snapshot(canopen_master_service_t *service)
{
    if (service == NULL) {
        return;
    }

    service->snapshot.pdo_group_sequence = service->active_pdo_group_sequence;
    service->snapshot.pdo_group_state = (uint8_t)service->active_pdo_group_state;
    service->snapshot.pdo_expected_frames = service->active_pdo_expected_frames;
    service->snapshot.pdo_submitted_frames = service->active_pdo_submitted_frames;
    service->snapshot.pdo_tx_complete_frames = service->active_pdo_tx_complete_frames;
    service->snapshot.pdo_failed_frames = service->active_pdo_failed_frames;
    service->snapshot.pdo_in_flight_frames = service->active_pdo_in_flight_frames;
    service->snapshot.pdo_arm_complete_frames = service->active_pdo_arm_complete_frames;
    service->snapshot.pdo_trigger_complete_frames = service->active_pdo_trigger_complete_frames;
}

static bool pdo_group_is_active(const canopen_master_service_t *service)
{
    if (service == NULL || service->active_pdo_group_sequence == 0U) {
        return false;
    }

    return service->active_pdo_group_state == CANOPEN_MASTER_PDO_GROUP_STATE_QUEUED ||
           service->active_pdo_group_state == CANOPEN_MASTER_PDO_GROUP_STATE_ARM_IN_FLIGHT ||
           service->active_pdo_group_state == CANOPEN_MASTER_PDO_GROUP_STATE_TRIGGER_IN_FLIGHT;
}

static void begin_pdo_group(canopen_master_service_t *service,
                            uint32_t group_sequence,
                            uint8_t expected_frames)
{
    if (service == NULL || group_sequence == 0U || expected_frames == 0U) {
        return;
    }

    service->active_pdo_group_sequence = group_sequence;
    service->active_pdo_group_state = CANOPEN_MASTER_PDO_GROUP_STATE_QUEUED;
    service->active_pdo_expected_frames = expected_frames;
    service->active_pdo_submitted_frames = 0U;
    service->active_pdo_tx_complete_frames = 0U;
    service->active_pdo_failed_frames = 0U;
    service->active_pdo_in_flight_frames = 0U;
    service->active_pdo_arm_complete_frames = 0U;
    service->active_pdo_trigger_complete_frames = 0U;
    sync_pdo_group_snapshot(service);
}

static bool can_accept_pdo_group(const canopen_master_service_t *service,
                                 uint32_t group_sequence)
{
    if (service == NULL || group_sequence == 0U) {
        return false;
    }

    return !pdo_group_is_active(service) ||
           service->active_pdo_group_sequence == group_sequence;
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
        out->index = ECU_CANOPEN_OBJ_COMMAND_CURRENT;
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
    request.preserve_order = canopen_master_sdo_write_requires_order(index);

    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < service->command_queue_count; ++i) {
        uint8_t slot = (uint8_t)((service->command_queue_tail + i) %
                                 CANOPEN_MASTER_COMMAND_QUEUE_CAPACITY);
        canopen_master_sdo_write_request_t *queued =
            &service->command_queue[slot];
        if (queued->node_id == node_id &&
            queued->index == index &&
            queued->subindex == subindex) {
            /* Control-word edges such as 0x000F -> 0x001F must remain ordered,
             * but repeated identical control words are just backlog.  Replacing
             * the older identical value preserves the required edge semantics
             * while keeping the field bus responsive during joystick motion.
             */
            if ((queued->preserve_order || request.preserve_order) &&
                queued->value != request.value) {
                continue;
            }
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
    if (service == NULL || !service->snapshot.initialized ||
        node_id == 0U) {
        return false;
    }

    if (!canopen_master_lock()) {
        return false;
    }
    select_stack(service);
    if (co == NULL) {
        canopen_master_unlock();
        return false;
    }

    bool result = canopen_master_service_request_nmt_locked(service, node_id, command);
    canopen_master_unlock();
    return result;
}

bool canopen_master_service_queue_pdo(canopen_master_service_t *service,
                                      uint16_t cob_id,
                                      const uint8_t *data,
                                      uint8_t size,
                                      uint8_t node_id,
                                      uint32_t group_sequence,
                                      canopen_master_pdo_phase_t phase)
{
    canopen_master_pdo_request_t request;

    if (service == NULL || data == NULL || size > CAN_MAX_DLC ||
        !service->snapshot.initialized || !service->snapshot.can_normal ||
        service->can_index >= CANOPEN_MASTER_BUS_COUNT ||
        cob_id == 0U || (cob_id & 0xF800U) != 0U ||
        group_sequence == 0U || phase == CANOPEN_MASTER_PDO_PHASE_NONE) {
        return false;
    }

    memset(&request, 0, sizeof(request));
    request.cob_id = cob_id;
    request.size = size;
    request.node_id = node_id;
    request.group_sequence = group_sequence;
    request.phase = phase;
    memcpy(request.data, data, size);

    taskENTER_CRITICAL();
    if (!can_accept_pdo_group(service, group_sequence)) {
        service->snapshot.pdo_dropped_count++;
        taskEXIT_CRITICAL();
        return false;
    }
    if (service->pdo_queue_count >= CANOPEN_MASTER_PDO_QUEUE_CAPACITY) {
        service->snapshot.pdo_dropped_count++;
        taskEXIT_CRITICAL();
        return false;
    }
    if (!pdo_group_is_active(service)) {
        begin_pdo_group(service, group_sequence, 1U);
    }
    service->pdo_queue[service->pdo_queue_head] = request;
    service->pdo_queue_head =
        (uint8_t)((service->pdo_queue_head + 1U) % CANOPEN_MASTER_PDO_QUEUE_CAPACITY);
    service->pdo_queue_count++;
    service->snapshot.pdo_queued_count = service->pdo_queue_count;
    taskEXIT_CRITICAL();
    return true;
}

static bool pdo_request_is_valid(const canopen_master_pdo_request_t *request)
{
    return request != NULL &&
           request->size <= CAN_MAX_DLC &&
           request->cob_id != 0U &&
           (request->cob_id & 0xF800U) == 0U &&
           request->group_sequence != 0U &&
           request->phase != CANOPEN_MASTER_PDO_PHASE_NONE;
}

bool canopen_master_service_queue_pdo_batch(canopen_master_service_t *service,
                                            const canopen_master_pdo_request_t *requests,
                                            uint8_t count)
{
    if (service == NULL || requests == NULL || count == 0U ||
        count > CANOPEN_MASTER_PDO_QUEUE_CAPACITY ||
        !service->snapshot.initialized || !service->snapshot.can_normal ||
        service->can_index >= CANOPEN_MASTER_BUS_COUNT) {
        return false;
    }

    for (uint8_t i = 0U; i < count; ++i) {
        if (!pdo_request_is_valid(&requests[i]) ||
            requests[i].group_sequence != requests[0].group_sequence) {
            return false;
        }
    }

    taskENTER_CRITICAL();
    if (!can_accept_pdo_group(service, requests[0].group_sequence)) {
        service->snapshot.pdo_dropped_count++;
        taskEXIT_CRITICAL();
        return false;
    }
    if ((uint8_t)(CANOPEN_MASTER_PDO_QUEUE_CAPACITY - service->pdo_queue_count) < count) {
        service->snapshot.pdo_dropped_count++;
        taskEXIT_CRITICAL();
        return false;
    }
    if (!pdo_group_is_active(service)) {
        begin_pdo_group(service, requests[0].group_sequence, count);
    }

    for (uint8_t i = 0U; i < count; ++i) {
        service->pdo_queue[service->pdo_queue_head] = requests[i];
        service->pdo_queue_head =
            (uint8_t)((service->pdo_queue_head + 1U) % CANOPEN_MASTER_PDO_QUEUE_CAPACITY);
        service->pdo_queue_count++;
    }
    service->snapshot.pdo_queued_count = service->pdo_queue_count;
    taskEXIT_CRITICAL();
    return true;
}

uint8_t canopen_master_service_pdo_queue_available(const canopen_master_service_t *service)
{
    uint8_t available = 0U;

    if (service == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();
    if (service->pdo_queue_count < CANOPEN_MASTER_PDO_QUEUE_CAPACITY) {
        available = (uint8_t)(CANOPEN_MASTER_PDO_QUEUE_CAPACITY - service->pdo_queue_count);
    }
    taskEXIT_CRITICAL();
    return available;
}

bool canopen_master_service_pdo_group_pending(const canopen_master_service_t *service,
                                              uint32_t group_sequence)
{
    if (service == NULL || group_sequence == 0U) {
        return false;
    }

    taskENTER_CRITICAL();
    if (service->active_pdo_group_sequence == group_sequence &&
        pdo_group_is_active(service)) {
        taskEXIT_CRITICAL();
        return true;
    }
    if (service->pdo_in_flight &&
        service->pdo_in_flight_request.group_sequence == group_sequence) {
        taskEXIT_CRITICAL();
        return true;
    }
    for (uint8_t i = 0U; i < service->pdo_queue_count; ++i) {
        uint8_t slot =
            (uint8_t)((service->pdo_queue_tail + i) % CANOPEN_MASTER_PDO_QUEUE_CAPACITY);
        if (service->pdo_queue[slot].group_sequence == group_sequence) {
            taskEXIT_CRITICAL();
            return true;
        }
    }
    taskEXIT_CRITICAL();
    return false;
}

bool canopen_master_service_pdo_group_failed(const canopen_master_service_t *service,
                                             uint32_t group_sequence)
{
    return service != NULL && group_sequence != 0U &&
           service->snapshot.last_pdo_failed_group_sequence == group_sequence;
}

static void drop_current_pdo_queue_item(canopen_master_service_t *service)
{
    if (service == NULL || service->pdo_queue_count == 0U) {
        return;
    }
    service->pdo_queue_tail =
        (uint8_t)((service->pdo_queue_tail + 1U) % CANOPEN_MASTER_PDO_QUEUE_CAPACITY);
    service->pdo_queue_count--;
    service->snapshot.pdo_queued_count = service->pdo_queue_count;
}

static void note_pdo_failure(canopen_master_service_t *service,
                             const canopen_master_pdo_request_t *request,
                             int error)
{
    service->snapshot.pdo_tx_error_count++;
    service->snapshot.command_error_count++;
    service->snapshot.last_pdo_failed_group_sequence = request->group_sequence;
    service->snapshot.last_pdo_failed_cob_id = request->cob_id;
    service->snapshot.last_pdo_failed_node_id = request->node_id;
    service->snapshot.last_pdo_failed_phase = (uint8_t)request->phase;
    service->snapshot.last_pdo_error = error;
    service->snapshot.last_error = error;
}

static bool pdo_tail_matches(const canopen_master_service_t *service,
                             const canopen_master_pdo_request_t *request)
{
    if (service == NULL || request == NULL || service->pdo_queue_count == 0U) {
        return false;
    }

    const canopen_master_pdo_request_t *tail =
        &service->pdo_queue[service->pdo_queue_tail];
    return tail->group_sequence == request->group_sequence &&
           tail->cob_id == request->cob_id &&
           tail->phase == request->phase &&
           tail->node_id == request->node_id;
}

static void cancel_queued_pdo_group(canopen_master_service_t *service,
                                    uint32_t group_sequence)
{
    if (service == NULL || group_sequence == 0U || service->pdo_queue_count == 0U) {
        return;
    }

    uint8_t old_tail = service->pdo_queue_tail;
    uint8_t old_count = service->pdo_queue_count;
    uint8_t write = old_tail;
    uint8_t kept = 0U;

    for (uint8_t i = 0U; i < old_count; ++i) {
        uint8_t read = (uint8_t)((old_tail + i) % CANOPEN_MASTER_PDO_QUEUE_CAPACITY);
        if (service->pdo_queue[read].group_sequence == group_sequence) {
            service->snapshot.pdo_dropped_count++;
            continue;
        }

        if (write != read) {
            service->pdo_queue[write] = service->pdo_queue[read];
        }
        write = (uint8_t)((write + 1U) % CANOPEN_MASTER_PDO_QUEUE_CAPACITY);
        kept++;
    }

    service->pdo_queue_tail = old_tail;
    service->pdo_queue_head = write;
    service->pdo_queue_count = kept;
    service->snapshot.pdo_queued_count = kept;
}

static void fail_active_pdo_group(canopen_master_service_t *service,
                                  const canopen_master_pdo_request_t *request,
                                  int error,
                                  uint32_t now_ms)
{
    uint32_t group_sequence;

    if (service == NULL) {
        return;
    }

    group_sequence = request != NULL ?
                     request->group_sequence :
                     service->active_pdo_group_sequence;
    if (group_sequence == 0U) {
        return;
    }

    if (request != NULL) {
        note_pdo_failure(service, request, error);
    }

    taskENTER_CRITICAL();
    cancel_queued_pdo_group(service, group_sequence);
    taskEXIT_CRITICAL();

    service->pdo_in_flight = false;
    service->active_pdo_group_sequence = group_sequence;
    service->active_pdo_group_state = CANOPEN_MASTER_PDO_GROUP_STATE_FAILED;
    service->active_pdo_failed_frames++;
    service->active_pdo_in_flight_frames = 0U;
    if (error == -ETIMEDOUT) {
        service->snapshot.last_pdo_tx_timeout_ms = now_ms;
    }
    sync_pdo_group_snapshot(service);
}

static void complete_in_flight_pdo(canopen_master_service_t *service,
                                   uint32_t now_ms)
{
    bool tail_ok = false;
    canopen_master_pdo_request_t request;

    if (service == NULL || !service->pdo_in_flight) {
        return;
    }

    request = service->pdo_in_flight_request;

    taskENTER_CRITICAL();
    tail_ok = pdo_tail_matches(service, &request);
    if (tail_ok) {
        drop_current_pdo_queue_item(service);
    }
    taskEXIT_CRITICAL();

    if (!tail_ok) {
        fail_active_pdo_group(service, &request, -EIO, now_ms);
        return;
    }

    service->pdo_in_flight = false;
    service->active_pdo_in_flight_frames = 0U;
    service->active_pdo_tx_complete_frames++;
    if (request.phase == CANOPEN_MASTER_PDO_PHASE_STEER_ARM) {
        service->active_pdo_arm_complete_frames++;
    } else if (request.phase == CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER) {
        service->active_pdo_trigger_complete_frames++;
    }

    service->snapshot.pdo_tx_count++;
    service->snapshot.last_pdo_tx_complete_ms = now_ms;
    service->snapshot.last_pdo_tx_group_sequence = request.group_sequence;
    service->snapshot.last_pdo_tx_cob_id = request.cob_id;
    service->snapshot.last_pdo_tx_node_id = request.node_id;
    service->snapshot.last_pdo_tx_phase = (uint8_t)request.phase;
    service->snapshot.last_pdo_error = 0;
    service->snapshot.last_error = 0;

    if (service->active_pdo_group_sequence == request.group_sequence &&
        service->active_pdo_failed_frames == 0U &&
        service->active_pdo_tx_complete_frames >= service->active_pdo_expected_frames &&
        service->active_pdo_in_flight_frames == 0U) {
        service->active_pdo_group_state = CANOPEN_MASTER_PDO_GROUP_STATE_COMPLETE;
    }

    sync_pdo_group_snapshot(service);
}

static void harvest_polled_primary_tx_complete(canopen_master_service_t *service)
{
    if (service == NULL || service->can_index >= CANOPEN_MASTER_BUS_COUNT) {
        return;
    }

    CAN_Type *can = s_can_info[service->can_index].can_base;
    uint8_t tx_rx_flags = can_get_tx_rx_flags(can);
    if ((tx_rx_flags & CAN_EVENT_TX_PRIMARY_BUF) != 0U) {
        can_clear_tx_rx_flags(can, CAN_EVENT_TX_PRIMARY_BUF);
        s_canopen_pdo_tx_complete_count[service->can_index]++;
    }
}

static void process_pdo_tx_complete_events(canopen_master_service_t *service,
                                           uint32_t now_ms)
{
    if (service == NULL || service->can_index >= CANOPEN_MASTER_BUS_COUNT) {
        return;
    }

    harvest_polled_primary_tx_complete(service);

    uint32_t complete_count = s_canopen_pdo_tx_complete_count[service->can_index];
    while (service->observed_pdo_tx_complete_count != complete_count) {
        service->observed_pdo_tx_complete_count++;
        if (service->pdo_in_flight) {
            complete_in_flight_pdo(service, now_ms);
        }
    }
}

static bool pdo_in_flight_timed_out(const canopen_master_service_t *service,
                                    uint32_t now_ms)
{
    return service != NULL &&
           service->pdo_in_flight &&
           (now_ms - service->pdo_in_flight_submit_ms) >=
               CANOPEN_MASTER_PDO_TX_TIMEOUT_MS;
}

static bool start_next_pdo_frame(canopen_master_service_t *service,
                                 uint32_t now_ms)
{
    canopen_master_pdo_request_t request;

    if (service == NULL || service->pdo_in_flight) {
        return false;
    }

    taskENTER_CRITICAL();
    if (service->pdo_queue_count == 0U) {
        taskEXIT_CRITICAL();
        return false;
    }
    request = service->pdo_queue[service->pdo_queue_tail];
    taskEXIT_CRITICAL();

    if (!pdo_group_is_active(service)) {
        begin_pdo_group(service, request.group_sequence, 1U);
    }

    if (service->active_pdo_group_sequence != request.group_sequence) {
        return false;
    }

    if (request.phase == CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER &&
        service->active_pdo_expected_frames == (ECU_WHEEL_COUNT * 2U) &&
        service->active_pdo_arm_complete_frames < ECU_WHEEL_COUNT) {
        fail_active_pdo_group(service, &request, -EIO, now_ms);
        return false;
    }

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = request.cob_id;
    frame.dlc = request.size;
    memcpy(frame.data, request.data, request.size);

    int result = hpm_can_send((struct device *)&hpm_canopen_dev[service->can_index],
                              &frame);
    if (result == 0) {
        service->pdo_in_flight_request = request;
        service->pdo_in_flight = true;
        service->pdo_in_flight_submit_ms = now_ms;
        service->active_pdo_submitted_frames++;
        service->active_pdo_in_flight_frames = 1U;
        service->active_pdo_group_state =
            request.phase == CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER ?
            CANOPEN_MASTER_PDO_GROUP_STATE_TRIGGER_IN_FLIGHT :
            CANOPEN_MASTER_PDO_GROUP_STATE_ARM_IN_FLIGHT;
        service->snapshot.last_pdo_error = 0;
        service->snapshot.last_error = 0;
        sync_pdo_group_snapshot(service);
        return true;
    }

    taskENTER_CRITICAL();
    if (pdo_tail_matches(service, &request)) {
        service->pdo_queue[service->pdo_queue_tail].retry_count++;
        request.retry_count = service->pdo_queue[service->pdo_queue_tail].retry_count;
    }
    taskEXIT_CRITICAL();

    if (request.retry_count >= CANOPEN_MASTER_PDO_TX_MAX_RETRIES) {
        fail_active_pdo_group(service, &request, result, now_ms);
    } else {
        service->snapshot.last_pdo_error = result;
    }
    return false;
}

static void wait_briefly_for_pdo_tx_complete(canopen_master_service_t *service,
                                             uint32_t now_ms)
{
    for (uint32_t elapsed_us = 0U;
         service != NULL && service->pdo_in_flight &&
         elapsed_us < CANOPEN_MASTER_PDO_TX_COMPLETE_POLL_US;
         ++elapsed_us) {
        process_pdo_tx_complete_events(service, now_ms);
        if (!service->pdo_in_flight) {
            return;
        }
        board_delay_us(1U);
    }
}

static void process_pdo_tx_queue(canopen_master_service_t *service,
                                 uint32_t now_ms)
{
    process_pdo_tx_complete_events(service, now_ms);

    if (pdo_in_flight_timed_out(service, now_ms)) {
        fail_active_pdo_group(service,
                              &service->pdo_in_flight_request,
                              -ETIMEDOUT,
                              now_ms);
        return;
    }

    for (uint8_t sent = 0U; sent < CANOPEN_MASTER_PDO_TX_BURST_LIMIT; ++sent) {
        if (!start_next_pdo_frame(service, now_ms)) {
            return;
        }

        wait_briefly_for_pdo_tx_complete(service, now_ms);
        if (service->pdo_in_flight ||
            service->active_pdo_group_state == CANOPEN_MASTER_PDO_GROUP_STATE_FAILED ||
            service->active_pdo_group_state == CANOPEN_MASTER_PDO_GROUP_STATE_CANCELLED) {
            return;
        }
    }
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
        (void)nmt_command;
        (void)canopen_master_service_request_nmt_locked(
            service,
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

    if (!canopen_master_lock()) {
        note_error(service, -3);
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
        canopen_master_unlock();
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
        canopen_master_unlock();
        return false;
    }

    CO_CANsetNormalMode(co->CANmodule);
    service->snapshot.initialized = true;
    service->snapshot.can_normal = true;
    service->snapshot.state = CANOPEN_MASTER_STATE_RUNNING;
    service->snapshot.last_error = 0;
    (void)heap_memory_used;
    canopen_master_unlock();
    return true;
}

void canopen_master_service_process(canopen_master_service_t *service,
                                    uint32_t now_ms)
{
    if (service == NULL || !service->snapshot.initialized ||
        service->can_index >= CANOPEN_MASTER_BUS_COUNT) {
        return;
    }

    if (!canopen_master_lock()) {
        return;
    }
    select_stack(service);
    if (co == NULL) {
        canopen_master_unlock();
        return;
    }

    process_pdo_tx_queue(service, now_ms);
    if (service->pdo_in_flight) {
        /* TX-complete flags are controller-level, not PDO-specific.  While a
         * realtime PDO is in-flight, do not start unrelated CANopenNode SDO/NMT
         * transmissions that could produce a TX-complete event and be mistaken
         * for the PDO frame.
         */
        canopen_master_unlock();
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
    canopen_master_unlock();
}

void canopen_master_service_get_snapshot(const canopen_master_service_t *service,
                                         canopen_master_snapshot_t *out)
{
    if (service == NULL || out == NULL) {
        return;
    }
    *out = service->snapshot;
}
