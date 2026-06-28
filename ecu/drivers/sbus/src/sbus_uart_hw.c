#include "sbus_uart_hw.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "board.h"
#include "ecu_config.h"
#include "hpm_common.h"
#include "hpm_interrupt.h"
#include "hpm_uart_drv.h"
#include "task.h"

/* SBUS uses 8E2 framing. At 100 kbaud one character is 12 bits
 * (start + 8 data + parity + 2 stop). A threshold of two characters gives a
 * clean end-of-frame marker without firing between normal frame bytes.
 */
/* Defensive limit only. The FIFO is much smaller than this; the limit prevents
 * a faulty status bit from trapping the CPU inside the interrupt forever.
 */
#ifndef SBUS_UART_MAX_BYTES_PER_ISR
#define SBUS_UART_MAX_BYTES_PER_ISR (64U)
#endif

static sbus_service_t *s_sbus_service;

static uint32_t sbus_uart_now_ms_from_isr(void)
{
    return (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
}

static void sbus_uart_drain_rx_fifo(uint32_t now_ms)
{
    uint32_t drained = 0U;

    while (uart_check_status(BOARD_SBUS_UART_BASE, uart_stat_data_ready)) {
        uint8_t byte = uart_read_byte(BOARD_SBUS_UART_BASE);
        if (s_sbus_service != 0) {
            sbus_service_feed_byte_from_isr(s_sbus_service, byte, now_ms);
        }

        drained++;
        if (drained >= SBUS_UART_MAX_BYTES_PER_ISR) {
            if (s_sbus_service != 0) {
                sbus_service_note_uart_error_from_isr(s_sbus_service);
            }
            break;
        }
    }
}

static void sbus_uart_note_line_status_errors(void)
{
    uint32_t status = uart_get_status(BOARD_SBUS_UART_BASE);
    const uint32_t error_mask = (uint32_t)uart_stat_overrun_error |
                                (uint32_t)uart_stat_parity_error |
                                (uint32_t)uart_stat_framing_error |
                                (uint32_t)uart_stat_line_break |
                                (uint32_t)uart_stat_rx_fifo_error;

    if ((status & error_mask) != 0U && s_sbus_service != 0) {
        sbus_service_note_uart_error_from_isr(s_sbus_service);
    }
}

bool sbus_uart_hw_init(sbus_service_t *service)
{
    if (service == 0) {
        return false;
    }

    uart_config_t config = {0};
    board_init_uart(BOARD_SBUS_UART_BASE);
    uart_default_config(BOARD_SBUS_UART_BASE, &config);

    config.src_freq_in_hz = board_init_uart_clock(BOARD_SBUS_UART_BASE);
    config.baudrate = BOARD_SBUS_BAUDRATE;
    config.word_length = word_length_8_bits;
    config.parity = parity_even;
    config.num_of_stop_bits = stop_bits_2;
    config.fifo_enable = true;
    config.dma_enable = false;
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;

#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    config.rxidle_config.detect_enable = true;
    config.rxidle_config.detect_irq_enable = true;
    config.rxidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
    config.rxidle_config.threshold = ECU_SBUS_UART_RX_IDLE_BITS;
#endif

    hpm_stat_t stat = uart_init(BOARD_SBUS_UART_BASE, &config);
    if (stat != status_success) {
        s_sbus_service = 0;
        return false;
    }

    s_sbus_service = service;
    uart_reset_rx_fifo(BOARD_SBUS_UART_BASE);
    uart_enable_irq(BOARD_SBUS_UART_BASE,
                    uart_intr_rx_data_avail_or_timeout | uart_intr_rx_line_stat);

#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    uart_clear_rxline_idle_flag(BOARD_SBUS_UART_BASE);
    uart_enable_irq(BOARD_SBUS_UART_BASE, uart_intr_rx_line_idle);
#endif

    intc_m_enable_irq_with_priority(BOARD_SBUS_UART_IRQ, ECU_SBUS_UART_IRQ_PRIORITY);
    return true;
}

SDK_DECLARE_EXT_ISR_M(BOARD_SBUS_UART_IRQ, sbus_uart_hw_isr)
void sbus_uart_hw_isr(void)
{
    uint32_t now_ms = sbus_uart_now_ms_from_isr();
    uint8_t irq_id = uart_get_irq_id(BOARD_SBUS_UART_BASE);

    sbus_uart_note_line_status_errors();

    if ((irq_id == uart_intr_id_rx_data_avail) ||
        (irq_id == uart_intr_id_rx_timeout) ||
        uart_check_status(BOARD_SBUS_UART_BASE, uart_stat_data_ready)) {
        sbus_uart_drain_rx_fifo(now_ms);
    }

#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    if (uart_is_rxline_idle(BOARD_SBUS_UART_BASE)) {
        /* Drain before marking idle so a complete 25-byte frame is decoded
         * first. If a partial frame remains, the service discards it below.
         */
        sbus_uart_drain_rx_fifo(now_ms);
        uart_clear_rxline_idle_flag(BOARD_SBUS_UART_BASE);
        if (s_sbus_service != 0) {
            sbus_service_note_rx_idle_from_isr(s_sbus_service);
        }
    }
#endif
}
