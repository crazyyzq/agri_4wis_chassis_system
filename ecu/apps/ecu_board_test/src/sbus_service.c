/* Interrupt-driven SBUS receiver service with debugger-readable state. */
#include <string.h>

#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_interrupt.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_uart_drv.h"
#include "sbus_decoder.h"
#include "sbus_service.h"

volatile sbus_debug_state_t g_sbus_debug_state;

static uint8_t s_frame_buffer[SBUS_SERVICE_FRAME_LENGTH];
static uint8_t s_frame_position;
static uint32_t s_service_now_ms;
static bool s_hardware_initialized;

/**
 * @brief Return a monotonic millisecond timestamp from MCHTMR.
 *
 * The service only needs coarse freshness timing, so the same conversion used
 * by the debug monitor is sufficient. A zero clock leaves the timestamp at 0
 * instead of dividing by zero during very early startup or test-only paths.
 */
static uint32_t service_now_ms(void)
{
    uint32_t frequency = clock_get_frequency(clock_mchtmr0);
    if (frequency == 0U) return 0U;
    return (uint32_t)((mchtmr_get_count(HPM_MCHTMR) * 1000ULL) / frequency);
}

/**
 * @brief Copy decoded channel and status bits into the public debug state.
 *
 * This function is called from the UART ISR after a complete frame decodes
 * successfully. Keep it deterministic and avoid console output here.
 */
static void publish_valid_frame(uint32_t now_ms, const sbus_frame_t *frame)
{
    for (uint8_t i = 0U; i < SBUS_SERVICE_CHANNEL_COUNT; ++i) {
        g_sbus_debug_state.channels[i] = frame->channels[i];
    }
    g_sbus_debug_state.frame_lost = frame->frame_lost ? 1U : 0U;
    g_sbus_debug_state.failsafe = frame->failsafe ? 1U : 0U;
    g_sbus_debug_state.channel17 = frame->channel17 ? 1U : 0U;
    g_sbus_debug_state.channel18 = frame->channel18 ? 1U : 0U;
    g_sbus_debug_state.last_frame_ms = now_ms;
    ++g_sbus_debug_state.frame_count;
    g_sbus_debug_state.connected = 1U;
}

/**
 * @brief Refresh timeout-derived connection state.
 *
 * A receiver is considered connected only while valid frames keep arriving.
 * The latest decoded channel values remain available after timeout so the
 * debugger can still inspect the last known signal values.
 */
static void update_connected(uint32_t now_ms)
{
    if (g_sbus_debug_state.frame_count == 0U) {
        g_sbus_debug_state.connected = 0U;
        return;
    }
    g_sbus_debug_state.connected =
        ((uint32_t)(now_ms - g_sbus_debug_state.last_frame_ms) <= SBUS_SERVICE_CONNECTED_TIMEOUT_MS)
            ? 1U
            : 0U;
}

/**
 * @brief Feed one received byte into the frame assembler.
 *
 * The parser resynchronizes on the SBUS start byte and decodes exactly one
 * 25-byte candidate frame at a time. Invalid complete candidates are counted
 * and then discarded so the next valid start byte can recover the stream.
 */
static void ingest_byte(uint32_t now_ms, uint8_t byte)
{
    if (s_frame_position == 0U && byte != 0x0FU) {
        return;
    }

    s_frame_buffer[s_frame_position++] = byte;
    if (s_frame_position < SBUS_SERVICE_FRAME_LENGTH) {
        return;
    }

    sbus_frame_t decoded;
    if (sbus_decode(s_frame_buffer, sizeof(s_frame_buffer), &decoded) == SBUS_OK) {
        publish_valid_frame(now_ms, &decoded);
    } else {
        ++g_sbus_debug_state.decode_error_count;
    }
    s_frame_position = 0U;
}

/**
 * @brief Drain all currently buffered UART bytes.
 *
 * The ISR calls this for both RX data and RX timeout/idle paths. Draining until
 * the FIFO is empty avoids overrun when SBUS frames continue while the console
 * is printing or the operator is idle at a prompt.
 */
static void drain_uart_rx(uint32_t now_ms)
{
    uint8_t byte;
    while (uart_try_receive_byte(BOARD_SBUS_UART_BASE, &byte) == status_success) {
        ingest_byte(now_ms, byte);
    }
}

SDK_DECLARE_EXT_ISR_M(BOARD_SBUS_UART_IRQ, sbus_uart_isr)
void sbus_uart_isr(void)
{
    uint32_t status = uart_get_status(BOARD_SBUS_UART_BASE);
    uint32_t now_ms = service_now_ms();

    if ((status & (uart_stat_overrun_error | uart_stat_parity_error |
                   uart_stat_framing_error | uart_stat_line_break |
                   uart_stat_rx_fifo_error)) != 0U) {
        ++g_sbus_debug_state.uart_error_count;
    }

    drain_uart_rx(now_ms);

#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    if (uart_is_rxline_idle(BOARD_SBUS_UART_BASE)) {
        uart_clear_rxline_idle_flag(BOARD_SBUS_UART_BASE);
    }
#endif
}

void sbus_service_init(void)
{
    board_init_uart(BOARD_SBUS_UART_BASE);

    uart_config_t config;
    uart_default_config(BOARD_SBUS_UART_BASE, &config);
    config.src_freq_in_hz = board_init_uart_clock(BOARD_SBUS_UART_BASE);
    config.baudrate = BOARD_SBUS_BAUDRATE;
    config.word_length = word_length_8_bits;
    config.parity = parity_even;
    config.num_of_stop_bits = stop_bits_2;
    config.fifo_enable = true;
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    config.rxidle_config.detect_enable = true;
    config.rxidle_config.detect_irq_enable = true;
    config.rxidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
    config.rxidle_config.threshold = 22U;
#endif

    if (uart_init(BOARD_SBUS_UART_BASE, &config) == status_success) {
        uart_clear_rx_fifo(BOARD_SBUS_UART_BASE);
        uart_enable_irq(BOARD_SBUS_UART_BASE,
                        uart_intr_rx_data_avail_or_timeout |
                        uart_intr_rx_line_stat
#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
                        | uart_intr_rx_line_idle
#endif
        );
        intc_m_enable_irq_with_priority(BOARD_SBUS_UART_IRQ, 2);
        s_hardware_initialized = true;
    } else {
        ++g_sbus_debug_state.uart_error_count;
        s_hardware_initialized = false;
    }
}

void sbus_service_poll(void)
{
    uint32_t now_ms = s_hardware_initialized ? service_now_ms() : s_service_now_ms;
    update_connected(now_ms);
}

void sbus_service_get_snapshot(sbus_debug_state_t *snapshot)
{
    if (snapshot == NULL) return;
    *snapshot = g_sbus_debug_state;
}

void sbus_service_test_reset(void)
{
    memset((void *)&g_sbus_debug_state, 0, sizeof(g_sbus_debug_state));
    memset(s_frame_buffer, 0, sizeof(s_frame_buffer));
    s_frame_position = 0U;
    s_service_now_ms = 0U;
    s_hardware_initialized = false;
}

void sbus_service_test_feed_byte(uint32_t now_ms, uint8_t byte)
{
    s_service_now_ms = now_ms;
    ingest_byte(now_ms, byte);
    update_connected(now_ms);
}

void sbus_service_test_poll_at(uint32_t now_ms)
{
    s_service_now_ms = now_ms;
    update_connected(now_ms);
}
