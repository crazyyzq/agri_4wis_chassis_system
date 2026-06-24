/* Classic CAN 2.0 loopback/termination test for four isolated channels. */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "hpm_can_drv.h"
#include "operator_io.h"
#include "safety_manager.h"
#include "test_cases.h"
#include "test_limits.h"

static CAN_Type *const can_bases[] = { BOARD_CAN1_BASE, BOARD_CAN2_BASE, BOARD_CAN3_BASE, BOARD_CAN4_BASE };
static bool can_receive_timeout(CAN_Type *base, can_receive_buf_t *message,
                                test_context_t *context)
{
    for (uint32_t wait = 0U; wait < 2000U; ++wait) {
        if (test_runner_poll_abort(context)) return false;
        if (can_is_data_available_in_receive_buffer(base)) {
            can_read_received_message(base, message);
            return true;
        }
        board_delay_ms(1U);
    }
    return false;
}

void test_can_cleanup(test_context_t *context)
{
    (void)context;
    for (uint8_t i = 1U; i <= 4U; ++i) safety_can_term_set(i, false);
}

test_status_t test_can(test_context_t *context)
{
    if (!operator_confirm("Classic CAN echo adapter is connected and configured for 500 kbit/s")) return TEST_BLOCKED;
    for (uint8_t port = 0U; port < 4U; ++port) {
        can_config_t config;
        can_get_default_config(&config);
        config.baudrate = 500000U;
        config.mode = can_mode_normal;
        config.enable_canfd = false; /* CAN-FD is deliberately out of this release scope. */
        board_init_can(can_bases[port]);
        uint32_t clock = board_init_can_clock(can_bases[port]);
        if (can_init(can_bases[port], &config, clock) != status_success) return TEST_FAIL;
        for (uint32_t sequence = 0U; sequence < TEST_COMM_FRAME_COUNT; ++sequence) {
            if (test_runner_poll_abort(context)) return TEST_BLOCKED;
            can_transmit_buf_t tx = { 0 };
            can_receive_buf_t rx = { 0 };
            tx.id = (sequence & 1U) ? (0x1ABCDE0U | port) : (0x600U | port);
            tx.extend_id = (sequence & 1U) != 0U;
            tx.dlc = (sequence % 17U == 0U) ? 0U : 8U;
            for (uint8_t i = 0U; i < tx.dlc; ++i) tx.data[i] = (uint8_t)(sequence + i + port);
            if (can_send_message_blocking(can_bases[port], &tx) != status_success ||
                !can_receive_timeout(can_bases[port], &rx, context) || rx.id != tx.id ||
                rx.extend_id != tx.extend_id || rx.dlc != tx.dlc ||
                memcmp(rx.data, tx.data, tx.dlc) != 0) {
                printf("CAN%u failure sequence=%lu\n", port + 1U, (unsigned long)sequence);
                return TEST_FAIL;
            }
        }
        char prompt[96];
        snprintf(prompt, sizeof(prompt), "Disconnect CAN%u bus; confirm timeout observed by adapter", port + 1U);
        if (!operator_confirm(prompt)) return TEST_FAIL;
        snprintf(prompt, sizeof(prompt), "Reconnect CAN%u and confirm adapter ready", port + 1U);
        if (!operator_confirm(prompt)) return TEST_BLOCKED;
        snprintf(prompt, sizeof(prompt), "CAN%u isolated: enable termination and measure 115..125 ohm", port + 1U);
        safety_can_term_set(port + 1U, true);
        uint32_t ohms;
        if (!operator_read_u32(prompt, &ohms)) return TEST_BLOCKED;
        safety_can_term_set(port + 1U, false);
        if (ohms < 115U || ohms > 125U || !operator_confirm("Termination disabled; meter shows high resistance")) return TEST_FAIL;
    }
    return TEST_PASS;
}
