/* CANopenNode-based diagnostic and command-debug master for ECU CAN2.
 *
 * Normal processing is passive: heartbeat tracking and cyclic SDO uploads.
 * Command output is only sent when a debugger changes
 * g_canopen_master_debug_control.command_sequence.
 */
#ifndef CANOPEN_MASTER_SERVICE_H
#define CANOPEN_MASTER_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

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
    canopen_master_state_t state;
    bool initialized;
    bool can_normal;
    uint32_t bitrate;
    uint8_t local_node_id;
    uint8_t remote_node_id;
    uint32_t process_count;
    uint32_t heartbeat_count;
    uint8_t last_heartbeat_state;
    uint32_t last_heartbeat_ms;
    uint32_t sdo_upload_count;
    uint32_t sdo_abort_count;
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
    uint16_t last_download_index;
    uint8_t last_download_subindex;
    uint8_t last_download_size;
    int32_t last_download_value;
    uint32_t last_download_abort_code;
    int32_t last_error;
} canopen_master_snapshot_t;

typedef struct {
    canopen_master_snapshot_t snapshot;
    uint32_t last_process_ms;
    uint32_t next_sdo_ms;
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
} canopen_master_service_t;

extern volatile canopen_master_debug_control_t g_canopen_master_debug_control;

/* Initialize CANopenNode on external CAN2.
 *
 * Caller: CPU0 task initialization.
 * Safety: debug commands are initialized to NONE and are sequence-gated.
 */
bool canopen_master_service_init(canopen_master_service_t *service,
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

void canopen_master_service_can2_isr(void);

#endif /* CANOPEN_MASTER_SERVICE_H */
