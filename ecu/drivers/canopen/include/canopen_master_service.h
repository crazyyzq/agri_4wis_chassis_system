/* CANopenNode-based master service for ECU CANopen networks.
 *
 * CPU0 owns two CANopen networks. This service wraps HPM SDK CANopenNode and
 * exposes only device-level NMT/SDO requests to the rest of the ECU.
 */
#ifndef CANOPEN_MASTER_SERVICE_H
#define CANOPEN_MASTER_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_config.h"

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
    bool preserve_order;
} canopen_master_sdo_write_request_t;

typedef struct {
    uint8_t node_id;
    uint16_t index;
    uint8_t subindex;
} canopen_master_sdo_read_request_t;

#define CANOPEN_MASTER_COMMAND_QUEUE_CAPACITY (96U)
#define CANOPEN_MASTER_READ_QUEUE_CAPACITY (32U)
#define CANOPEN_MASTER_PDO_QUEUE_CAPACITY (64U)
#define CANOPEN_MASTER_PDO_TX_BURST_LIMIT (8U)
#define CANOPEN_MASTER_PDO_TX_MAX_RETRIES (8U)
#define CANOPEN_MASTER_PDO_TX_TIMEOUT_MS (20U)

typedef enum {
    CANOPEN_MASTER_PDO_PHASE_NONE = 0,
    CANOPEN_MASTER_PDO_PHASE_STEER_ARM,
    CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER,
    CANOPEN_MASTER_PDO_PHASE_NODE5_POSITION_ARM,
    CANOPEN_MASTER_PDO_PHASE_NODE5_POSITION_TRIGGER,
    CANOPEN_MASTER_PDO_PHASE_DRIVE_VELOCITY,
    CANOPEN_MASTER_PDO_PHASE_SAFE_STOP
} canopen_master_pdo_phase_t;

typedef enum {
    CANOPEN_MASTER_PDO_GROUP_STATE_IDLE = 0,
    CANOPEN_MASTER_PDO_GROUP_STATE_QUEUED,
    CANOPEN_MASTER_PDO_GROUP_STATE_ARM_IN_FLIGHT,
    CANOPEN_MASTER_PDO_GROUP_STATE_TRIGGER_IN_FLIGHT,
    CANOPEN_MASTER_PDO_GROUP_STATE_COMPLETE,
    CANOPEN_MASTER_PDO_GROUP_STATE_FAILED,
    CANOPEN_MASTER_PDO_GROUP_STATE_CANCELLED
} canopen_master_pdo_group_state_t;

typedef enum {
    CANOPEN_MASTER_PDO_FAIL_NONE = 0,
    CANOPEN_MASTER_PDO_FAIL_SUBMIT_BUSY,
    CANOPEN_MASTER_PDO_FAIL_SUBMIT_ERROR,
    CANOPEN_MASTER_PDO_FAIL_TX_TIMEOUT,
    CANOPEN_MASTER_PDO_FAIL_TX_ERROR_EVENT,
    CANOPEN_MASTER_PDO_FAIL_GROUP_CANCELLED,
    CANOPEN_MASTER_PDO_FAIL_SAFETY_INHIBITED,
    CANOPEN_MASTER_PDO_FAIL_GROUP_CONFLICT,
    CANOPEN_MASTER_PDO_FAIL_QUEUE_FULL
} canopen_master_pdo_fail_reason_t;

typedef struct {
    uint16_t cob_id;
    uint8_t size;
    uint8_t node_id;
    uint32_t group_sequence;
    canopen_master_pdo_phase_t phase;
    uint8_t retry_count;
    uint8_t data[8];
} canopen_master_pdo_request_t;

typedef struct {
    uint8_t expected_frames;
    uint8_t arm_frame_count;
    uint8_t trigger_frame_count;
    uint8_t axis_mask;
    bool position_group;
} canopen_master_pdo_group_descriptor_t;

typedef struct {
    uint32_t feedback_sequence;     /* Even when stable; odd while ISR writes this node snapshot. */
    bool tpdo0_valid;
    bool tpdo1_valid;
    bool feedback_fresh;
    uint32_t last_tpdo0_ms;
    uint32_t last_tpdo1_ms;
    uint32_t tpdo0_rx_count;
    uint32_t tpdo1_rx_count;
    uint32_t malformed_tpdo_count;
    uint32_t unexpected_tpdo_count;
    int32_t actual_position_counts; /* Actual position, count units from the drive. */
    int32_t actual_velocity_units;  /* Actual velocity, vendor velocity units. */
    uint32_t fault_latched;         /* Vendor latched-fault word. */
    uint16_t statusword;            /* CiA-402 stateword. */
    int16_t actual_current_raw;     /* Vendor raw current feedback. */
} canopen_node_feedback_t;

#define CANOPEN_MASTER_NODE_FEEDBACK_SLOTS (14U)

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
    uint32_t pdo_tx_count;
    uint32_t pdo_tx_error_count;
    uint32_t pdo_queued_count;
    uint32_t pdo_dropped_count;
    uint32_t pdo_group_sequence;
    uint8_t pdo_group_state;
    uint8_t pdo_expected_frames;
    uint8_t pdo_submitted_frames;
    uint8_t pdo_tx_complete_frames;
    uint8_t pdo_failed_frames;
    uint8_t pdo_in_flight_frames;
    uint8_t pdo_arm_complete_frames;
    uint8_t pdo_trigger_complete_frames;
    uint8_t pdo_arm_frame_count;
    uint8_t pdo_trigger_frame_count;
    uint8_t pdo_axis_mask;
    bool pdo_position_group;
    uint32_t last_pdo_tx_complete_ms;
    uint32_t last_pdo_tx_timeout_ms;
    uint32_t last_pdo_tx_group_sequence;
    uint32_t last_pdo_failed_group_sequence;
    uint32_t last_pdo_failed_group_id;
    uint16_t last_pdo_tx_cob_id;
    uint16_t last_pdo_failed_cob_id;
    uint8_t last_pdo_tx_node_id;
    uint8_t last_pdo_failed_node_id;
    uint8_t last_pdo_tx_phase;
    uint8_t last_pdo_failed_phase;
    int32_t last_pdo_error;
    int32_t last_pdo_current_error;
    int32_t last_pdo_failed_error;
    uint32_t last_pdo_failed_ms;
    uint8_t last_pdo_failed_reason;
    uint32_t pdo_queue_full_drop_count;
    uint32_t pdo_group_conflict_drop_count;
    uint32_t pdo_safety_inhibit_count;
    uint32_t pdo_same_target_coalesce_count;
    uint32_t command_error_count;
    uint32_t queued_command_count;
    uint32_t dropped_command_count;
    uint16_t last_download_index;
    uint8_t last_download_subindex;
    uint8_t last_download_size;
    int32_t last_download_value;
    uint32_t last_download_abort_code;
    int32_t last_error;
    uint32_t sync_tx_count;
    uint32_t sync_tx_error_count;
    uint32_t sync_tx_complete_count;
    uint32_t last_sync_tx_ms;
    uint32_t last_sync_tx_complete_ms;
    uint32_t sync_tx_timeout_count;
    uint32_t sync_in_flight_submit_ms;
    int32_t last_sync_error;
    bool sync_in_flight;
    uint8_t tpdo0_observer_registered_mask;
    uint8_t tpdo1_observer_registered_mask;
    uint8_t tpdo0_hal_fallback_registered_mask;
    uint8_t tpdo1_hal_fallback_registered_mask;
    uint8_t steer_tpdo_observer_error_mask;
    bool steer_tpdo_observer_ready;
    uint32_t tpdo_observer_registration_error_count;
    canopen_node_feedback_t node_feedback[CANOPEN_MASTER_NODE_FEEDBACK_SLOTS];
} canopen_master_snapshot_t;

typedef struct {
    canopen_master_snapshot_t snapshot;
    uint8_t can_index;
    uint32_t last_process_ms;
    uint32_t next_sdo_ms;
    uint32_t sdo_retry_backoff_ms;
    uint32_t sdo_next_retry_ms;
    uint32_t sdo_offline_since_ms;
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
    canopen_master_pdo_request_t pdo_queue[CANOPEN_MASTER_PDO_QUEUE_CAPACITY];
    uint8_t pdo_queue_head;
    uint8_t pdo_queue_tail;
    uint8_t pdo_queue_count;
    canopen_master_pdo_request_t pdo_in_flight_request;
    bool pdo_in_flight;
    uint32_t pdo_in_flight_submit_ms;
    bool sync_in_flight;
    uint32_t sync_in_flight_submit_ms;
    uint32_t observed_pdo_tx_complete_count;
    uint32_t active_pdo_group_sequence;
    canopen_master_pdo_group_state_t active_pdo_group_state;
    uint8_t active_pdo_expected_frames;
    uint8_t active_pdo_submitted_frames;
    uint8_t active_pdo_tx_complete_frames;
    uint8_t active_pdo_failed_frames;
    uint8_t active_pdo_in_flight_frames;
    uint8_t active_pdo_arm_complete_frames;
    uint8_t active_pdo_trigger_complete_frames;
    canopen_master_pdo_group_descriptor_t active_pdo_group_descriptor;
    bool active_pdo_cancel_requested;
    bool active_pdo_cancel_after_inflight;
    bool active_pdo_trigger_started;
    uint8_t active_pdo_trigger_complete_frames_at_cancel;
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

/* Send one already-mapped PDO frame on this CANopen bus.
 *
 * This is intentionally a raw PDO transmitter, not a second CAN driver.  The
 * service uses the HPM CANopenNode device that was initialized for the bus, so
 * all CAN2/CAN3 CANopen traffic still passes through one owner.
 */
/* Queue one PDO for the service-owned software TX scheduler.
 *
 * The queue preserves group ordering.  Motion code should enqueue a complete
 * group first, then let canopen_master_service_process() submit frames to the
 * hardware TX mailboxes when they are available.
 */
bool canopen_master_service_queue_pdo(canopen_master_service_t *service,
                                      uint16_t cob_id,
                                      const uint8_t *data,
                                      uint8_t size,
                                      uint8_t node_id,
                                      uint32_t group_sequence,
                                      canopen_master_pdo_phase_t phase);
bool canopen_master_service_queue_pdo_batch(canopen_master_service_t *service,
                                            const canopen_master_pdo_request_t *requests,
                                            uint8_t count);
bool canopen_master_service_queue_pdo_batch_with_descriptor(
    canopen_master_service_t *service,
    const canopen_master_pdo_request_t *requests,
    uint8_t count,
    const canopen_master_pdo_group_descriptor_t *descriptor);
bool canopen_master_service_send_sync(canopen_master_service_t *service,
                                      uint32_t now_ms);
bool canopen_master_service_get_node_feedback(const canopen_master_service_t *service,
                                              uint8_t node_id,
                                              canopen_node_feedback_t *out);
bool canopen_master_service_steer_tpdo_observers_ready(const canopen_master_service_t *service);

uint8_t canopen_master_service_pdo_queue_available(const canopen_master_service_t *service);
bool canopen_master_service_pdo_group_pending(const canopen_master_service_t *service,
                                              uint32_t group_sequence);
bool canopen_master_service_pdo_group_failed(const canopen_master_service_t *service,
                                             uint32_t group_sequence);
bool canopen_master_service_pdo_group_cancelled(const canopen_master_service_t *service,
                                                uint32_t group_sequence);
bool canopen_master_service_cancel_pdo_group(canopen_master_service_t *service,
                                             uint32_t group_sequence);
void canopen_master_service_note_pdo_safety_inhibit(canopen_master_service_t *service);
void canopen_master_service_note_pdo_same_target_coalesced(canopen_master_service_t *service);
bool canopen_master_service_has_node_evidence(const canopen_master_service_t *service,
                                              uint8_t node_id);
bool canopen_master_service_diagnostic_scan_allowed(const canopen_master_service_t *service,
                                                    uint32_t now_ms);

void canopen_master_service_can2_isr(void);
void canopen_master_service_can3_isr(void);

#endif /* CANOPEN_MASTER_SERVICE_H */
