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

/**
 * @brief Install the hardware backend and immediately drive all resources safe.
 *
 * @param ops Statically valid backend with all three callbacks, or null/partial
 *            backend to disable managed hardware access.
 *
 * @note The backend pointer is borrowed, not copied, and must remain valid until
 *       replaced. Initialization clears tracked state and calls safety_all_off().
 */
void safety_init(const safety_hw_ops_t *ops);
/**
 * @brief Borrow the currently installed backend pointer.
 *
 * @return The backend supplied to safety_init(), or null when none is valid.
 *
 * @warning The returned pointer is not owned by the caller. It exists only so a
 *          scoped self-test can save and restore the previous backend.
 */
const safety_hw_ops_t *safety_backend(void);
/**
 * @brief Enable one external digital output under exclusive ownership.
 *
 * @param index One-based output number in 1..SAFETY_OUTPUT_COUNT.
 * @return SAFETY_OK on success, SAFETY_INVALID for an invalid index/backend, or
 *         SAFETY_BUSY when a different output is already enabled.
 */
safety_status_t safety_output_on(uint8_t index);
/**
 * @brief Disable one external digital output and release it if currently owned.
 *
 * @param index One-based output number; invalid indexes/backends are ignored.
 */
void safety_output_off(uint8_t index);
/**
 * @brief Set one external CAN termination control.
 *
 * @param index  One-based CAN channel in 1..SAFETY_CAN_COUNT.
 * @param enable true to connect termination, false to disconnect it.
 * @return SAFETY_OK on success or SAFETY_INVALID for an invalid index/backend.
 *
 * @note Electrical active-low polarity is owned by the board backend.
 */
safety_status_t safety_can_term_set(uint8_t index, bool enable);
/**
 * @brief Request transmit direction for one managed RS485 channel.
 *
 * @param index One-based RS485 channel in 1..SAFETY_RS485_COUNT.
 * @return SAFETY_OK on success or SAFETY_INVALID for an invalid index/backend.
 *
 * @note On this ECU, UART hardware DE owns the physical direction signal; the
 *       backend may therefore use this call only for logical state tracking.
 */
safety_status_t safety_rs485_transmit(uint8_t index);
/**
 * @brief Return one managed RS485 channel to receive direction.
 *
 * @param index One-based RS485 channel; invalid indexes/backends are ignored.
 */
void safety_rs485_receive(uint8_t index);
/**
 * @brief Drive every managed output and termination off and select RS485 receive.
 *
 * @note This operation is idempotent. With no valid backend it performs no I/O;
 *       callers may invoke it unconditionally on every exit path.
 */
void safety_all_off(void);
/**
 * @brief Copy the Safety Manager's current logical resource state.
 *
 * @return Value snapshot containing the active one-based output and bit masks
 *         for CAN termination and RS485 transmit direction.
 */
safety_snapshot_t safety_snapshot(void);

#endif
