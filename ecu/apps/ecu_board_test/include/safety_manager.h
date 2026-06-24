/* Central ownership and fail-safe shutdown for vehicle-affecting test IO. */
#ifndef ECU_SAFETY_MANAGER_H
#define ECU_SAFETY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#define SAFETY_OUTPUT_COUNT 12U
#define SAFETY_CAN_COUNT 4U
#define SAFETY_RS485_COUNT 3U

typedef enum { SAFETY_OK, SAFETY_INVALID, SAFETY_BUSY } safety_status_t;

typedef struct {
    void (*output_write)(uint8_t index, bool on);
    void (*can_term_write)(uint8_t index, bool enable);
    void (*rs485_direction_write)(uint8_t index, bool transmit);
} safety_hw_ops_t;

typedef struct {
    uint8_t active_output;
    uint8_t can_term_mask;
    uint8_t rs485_tx_mask;
} safety_snapshot_t;

/* Install a complete backend and immediately drive every resource safe. */
void safety_init(const safety_hw_ops_t *ops);
/* Borrow the installed backend pointer so scoped self-tests can restore it. */
const safety_hw_ops_t *safety_backend(void);
/* Output, CAN and RS485 indexes are one-based external channel numbers. */
safety_status_t safety_output_on(uint8_t index);
void safety_output_off(uint8_t index);
safety_status_t safety_can_term_set(uint8_t index, bool enable);
safety_status_t safety_rs485_transmit(uint8_t index);
void safety_rs485_receive(uint8_t index);
/* Idempotently disable all outputs/terminations and select RS485 receive. */
void safety_all_off(void);
safety_snapshot_t safety_snapshot(void);

#endif
