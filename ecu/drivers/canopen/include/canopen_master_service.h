/* CANopenNode-based master service for ECU CANopen networks.
 *
 * CPU0 owns two CANopen networks. This service wraps HPM SDK CANopenNode and
 * exposes only device-level NMT/SDO requests to the rest of the ECU.
 */
#ifndef CANOPEN_MASTER_SERVICE_H
#define CANOPEN_MASTER_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CANOPEN_MASTER_BUS_CAN2 = 0,
    CANOPEN_MASTER_BUS_CAN3 = 1,
    CANOPEN_MASTER_BUS_COUNT
} canopen_master_bus_t;

typedef enum {
    CANOPEN_MASTER_STATE_STOPPED = 0,
    CANOPEN_MASTER_STATE_INITIALIZED,
    CANOPEN_MASTER_STATE_RUNNING,
    CANOPEN_MASTER_STATE_ERROR
} canopen_master_state_t;

typedef enum {
    CANOPEN_MASTER_DEBUG_COMMAND_NONE = 0,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_PRE_OPERATIONAL,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_STOPPED,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET_NODE,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET_COMMUNICATION,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_CONTROLWORD,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_MODE,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_POSITION,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_VELOCITY,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_TORQUE,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_GENERIC
} canopen_master_debug_command_t;

typedef struct {
    uint32_t command_sequence;
    canopen_master_debug_command_t command;
    canopen_master_bus_t bus;
    uint8_t node_id;
    uint16_t index;
    uint8_t subindex;
    uint8_t size;
    int32_t value;
    uint16_t controlword;
    int8_t mode;
    int32_t target_position;
    int32_t target_velocity;
    int16_t target_torque;
} canopen_master_debug_control_t;

typedef struct {
    uint8_t node_id;
    uint16_t index;
    uint8_t subindex;
    uint8_t size;
    int32_t value;
} canopen_master_sdo_write_request_t;

typedef struct {
    uint8_t node_id;
    uint16_t index;
    uint8_t subindex;
} canopen_master_sdo_read_request_t;

#define CANOPEN_MASTER_COMMAND_QUEUE_CAPACITY (32U)
#define CANOPEN_MASTER_READ_QUEUE_CAPACITY (32U)

typedef struct {
    canopen_master_state_t state;
    bool initialized;
    bool can_normal;
    uint32_t bitrate;
    canopen_master_bus_t bus;
    uint8_t local_node_id;
    uint8_t remote_node_id;
    uint32_t process_count;
    uint32_t heartbeat_count;
    uint8_t last_heartbeat_state;
    uint32_t last_heartbeat_ms;
    uint32_t sdo_upload_count;
    uint32_t sdo_abort_count;
    uint8_t last_sdo_node_id;
    uint16_t last_sdo_index;
    uint8_t last_sdo_subindex;
    uint32_t last_sdo_value;
    uint8_t last_sdo_size;
    uint32_t last_sdo_abort_code;
    uint32_t last_command_sequence;
    canopen_master_debug_command_t last_command;
    uint8_t last_command_node_id;
    uint32_t nmt_command_count;
    uint32_t sdo_download_count;
    uint32_t sdo_download_abort_count;
    uint32_t command_error_count;
    uint32_t queued_command_count;
    uint32_t dropped_command_count;
    uint16_t last_download_index;
    uint8_t last_download_subindex;
    uint8_t last_download_size;
    int32_t last_download_value;
    uint32_t last_download_abort_code;
    int32_t last_error;
} canopen_master_snapshot_t;

typedef struct {
    canopen_master_snapshot_t snapshot;
    uint8_t can_index;
    uint32_t last_process_ms;
    uint32_t next_sdo_ms;
    uint8_t active_sdo_node_id;
    uint16_t active_sdo_index;
    uint8_t active_sdo_subindex;
    uint16_t active_download_index;
    uint8_t active_download_subindex;
    uint8_t active_download_size;
    int32_t active_download_value;
    uint32_t last_debug_sequence;
    uint8_t next_query;
    bool sdo_active;
    bool sdo_download_active;
    canopen_master_sdo_write_request_t active_download_request;
    canopen_master_sdo_write_request_t command_queue[CANOPEN_MASTER_COMMAND_QUEUE_CAPACITY];
    uint8_t command_queue_head;
    uint8_t command_queue_tail;
    uint8_t command_queue_count;
    canopen_master_sdo_read_request_t read_queue[CANOPEN_MASTER_READ_QUEUE_CAPACITY];
    uint8_t read_queue_head;
    uint8_t read_queue_tail;
    uint8_t read_queue_count;
} canopen_master_service_t;

extern volatile canopen_master_debug_control_t g_canopen_master_debug_control;

/* Initialize CANopenNode on one external CANopen network.
 *
 * Caller: CPU0 task initialization.
 * Safety: debug commands are initialized to NONE and are sequence-gated.
 */
bool canopen_master_service_init(canopen_master_service_t *service,
                                 canopen_master_bus_t bus,
                                 uint32_t bitrate,
                                 uint8_t local_node_id,
                                 uint8_t remote_node_id,
                                 uint32_t now_ms);

/* Process CANopen timers and one non-blocking SDO upload state machine step. */
void canopen_master_service_process(canopen_master_service_t *service,
                                    uint32_t now_ms);

/* Copy the latest diagnostic state for COM9 runtime printing. */
void canopen_master_service_get_snapshot(const canopen_master_service_t *service,
                                         canopen_master_snapshot_t *out);

/* Queue one SDO download through CANopenNode.
 *
 * Units: value is encoded little-endian with size 1, 2 or 4 bytes.
 * Caller: CPU0 foreground tasks only. ISR: not safe.
 * Failure: returns false when arguments are invalid or the queue is full.
 */
bool canopen_master_service_request_sdo_write(canopen_master_service_t *service,
                                              uint8_t node_id,
                                              uint16_t index,
                                              uint8_t subindex,
                                              uint8_t size,
                                              int32_t value);

/* Queue one SDO upload through CANopenNode.
 *
 * The latest completed upload is exposed in canopen_master_snapshot_t. Callers
 * use this as a non-blocking request/observe interface.
 */
bool canopen_master_service_request_sdo_read(canopen_master_service_t *service,
                                             uint8_t node_id,
                                             uint16_t index,
                                             uint8_t subindex);

/* Send one NMT command through CANopenNode immediately.
 *
 * Caller: CPU0 foreground tasks only. ISR: not safe.
 */
bool canopen_master_service_request_nmt(canopen_master_service_t *service,
                                        uint8_t node_id,
                                        canopen_master_debug_command_t command);

void canopen_master_service_can2_isr(void);
void canopen_master_service_can3_isr(void);

#endif /* CANOPEN_MASTER_SERVICE_H */
