#include "uart_rs485_hw.h"

#include <string.h>

#include "FreeRTOS.h"
#include "board.h"
#include "ecu_config.h"
#include "hpm_common.h"
#include "hpm_gpio_drv.h"
#include "hpm_interrupt.h"
#include "hpm_uart_drv.h"
#include "task.h"

#ifndef UART_RS485_MAX_BYTES_PER_ISR
#define UART_RS485_MAX_BYTES_PER_ISR (64U)
#endif

#ifndef UART_RS485_BITS_PER_8N1_CHAR
#define UART_RS485_BITS_PER_8N1_CHAR (10U)
#endif

#ifndef UART_RS485_TX_WAIT_MARGIN_US
#define UART_RS485_TX_WAIT_MARGIN_US (100U)
#endif

#ifndef UART_RS485_DE_SETUP_US
#define UART_RS485_DE_SETUP_US (2U)
#endif

#ifndef UART_RS485_DE_HOLD_US
#define UART_RS485_DE_HOLD_US (2U)
#endif

static uart_rs485_hw_t *s_rs485_1;

static void uart_rs485_1_set_receive_direction(void)
{
#if BOARD_RS485_DE_USING_GPIO
    gpio_write_pin(BOARD_RS485_1_DE_GPIO_CTRL,
                   BOARD_RS485_1_DE_GPIO_INDEX,
                   BOARD_RS485_1_DE_GPIO_PIN,
                   BOARD_RS485_RX_ENABLE_LEVEL);
#endif
}

static void uart_rs485_1_set_transmit_direction(void)
{
#if BOARD_RS485_DE_USING_GPIO
    gpio_write_pin(BOARD_RS485_1_DE_GPIO_CTRL,
                   BOARD_RS485_1_DE_GPIO_INDEX,
                   BOARD_RS485_1_DE_GPIO_PIN,
                   BOARD_RS485_TX_ENABLE_LEVEL);
#endif
}

static void uart_rs485_note_line_status_errors(void)
{
    uint32_t status = uart_get_status(BOARD_RS485_1_UART_BASE);
    const uint32_t error_mask = (uint32_t)uart_stat_overrun_error |
                                (uint32_t)uart_stat_parity_error |
                                (uint32_t)uart_stat_framing_error |
                                (uint32_t)uart_stat_line_break |
                                (uint32_t)uart_stat_rx_fifo_error;

    if ((status & error_mask) != 0U && s_rs485_1 != 0) {
        s_rs485_1->line_error_count++;
    }
}

static void uart_rs485_push_rx_byte(uint8_t byte)
{
    if (s_rs485_1 == 0) {
        return;
    }

    if (s_rs485_1->rx_size >= sizeof(s_rs485_1->rx_buffer)) {
        s_rs485_1->rx_overflow_count++;
        return;
    }

    s_rs485_1->rx_buffer[s_rs485_1->rx_head] = byte;
    s_rs485_1->rx_head++;
    if (s_rs485_1->rx_head >= sizeof(s_rs485_1->rx_buffer)) {
        s_rs485_1->rx_head = 0U;
    }
    s_rs485_1->rx_size++;
    s_rs485_1->rx_count++;
}

static void uart_rs485_drain_rx_fifo(void)
{
    uint32_t drained = 0U;

    while (uart_check_status(BOARD_RS485_1_UART_BASE, uart_stat_data_ready)) {
        uart_rs485_push_rx_byte(uart_read_byte(BOARD_RS485_1_UART_BASE));
        drained++;
        if (drained >= UART_RS485_MAX_BYTES_PER_ISR) {
            if (s_rs485_1 != 0) {
                s_rs485_1->line_error_count++;
            }
            break;
        }
    }
}

static uint32_t uart_rs485_char_time_us(uint32_t baudrate)
{
    if (baudrate == 0U) {
        return 1000U;
    }

    return ((UART_RS485_BITS_PER_8N1_CHAR * 1000000UL) + baudrate - 1U) /
           baudrate;
}

static bool uart_rs485_wait_status(uart_stat_t status, uint32_t timeout_us)
{
    uint32_t elapsed_us = 0U;

    while (!uart_check_status(BOARD_RS485_1_UART_BASE, status)) {
        if (elapsed_us >= timeout_us) {
            return false;
        }
        board_delay_us(1U);
        elapsed_us++;
    }

    return true;
}

bool uart_rs485_1_hw_init(uart_rs485_hw_t *service, uint32_t baudrate)
{
    uart_config_t config = {0};

    if (service == 0 || baudrate == 0U) {
        return false;
    }

    memset(service, 0, sizeof(*service));
    service->baudrate = baudrate;
    board_init_uart(BOARD_RS485_1_UART_BASE);
    uart_default_config(BOARD_RS485_1_UART_BASE, &config);

    config.src_freq_in_hz = board_init_uart_clock(BOARD_RS485_1_UART_BASE);
    config.baudrate = baudrate;
    config.word_length = word_length_8_bits;
    config.parity = parity_none;
    config.num_of_stop_bits = stop_bits_1;
    config.fifo_enable = true;
    config.dma_enable = false;
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;

#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    config.rxidle_config.detect_enable = true;
    config.rxidle_config.detect_irq_enable = true;
    config.rxidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
    config.rxidle_config.threshold = 24U;
#endif

    hpm_stat_t stat = uart_init(BOARD_RS485_1_UART_BASE, &config);
    if (stat != status_success) {
        s_rs485_1 = 0;
        return false;
    }

#if defined(HPM_IP_FEATURE_UART_DE_DELAY) && (HPM_IP_FEATURE_UART_DE_DELAY == 1)
    uart_set_de_delay_before_start_bit(BOARD_RS485_1_UART_BASE, 1U);
    uart_set_de_delay_after_stop_bit(BOARD_RS485_1_UART_BASE, 1U);
#endif

    s_rs485_1 = service;
    service->initialized = true;
    uart_rs485_1_set_receive_direction();
    uart_reset_rx_fifo(BOARD_RS485_1_UART_BASE);
    uart_reset_tx_fifo(BOARD_RS485_1_UART_BASE);
    uart_enable_irq(BOARD_RS485_1_UART_BASE,
                    uart_intr_rx_data_avail_or_timeout | uart_intr_rx_line_stat);

#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    uart_clear_rxline_idle_flag(BOARD_RS485_1_UART_BASE);
    uart_enable_irq(BOARD_RS485_1_UART_BASE, uart_intr_rx_line_idle);
#endif

    intc_m_enable_irq_with_priority(BOARD_RS485_1_UART_IRQ,
                                    ECU_RS485_UART_IRQ_PRIORITY);
    return true;
}

bool uart_rs485_1_hw_send(uart_rs485_hw_t *service,
                          const uint8_t *data,
                          size_t size)
{
    uint32_t char_time_us;
    uint32_t slot_timeout_us;
    uint32_t empty_timeout_us;

    if (service == 0 || !service->initialized || data == 0 || size == 0U) {
        return false;
    }

    char_time_us = uart_rs485_char_time_us(service->baudrate);
    slot_timeout_us = (char_time_us * 2U) + UART_RS485_TX_WAIT_MARGIN_US;
    empty_timeout_us = ((uint32_t)size + 2U) * char_time_us +
                       UART_RS485_TX_WAIT_MARGIN_US;

    /* HPM SDK uart_send_byte()/uart_flush() use a fixed short polling count.
     * That is fine at high baud rates but can time out at 9600 baud before the
     * next character slot opens.  RS485 Modbus uses low baud rates, so wait in
     * real time and then write the hardware TX register. RS485_1 direction is
     * controlled explicitly because HPM6750 SDK 1.11 does not expose automatic
     * UART DE direction switching for this UART IP.
     */
    uart_rs485_1_set_transmit_direction();
    board_delay_us(UART_RS485_DE_SETUP_US);

    for (size_t i = 0U; i < size; ++i) {
        if (!uart_rs485_wait_status(uart_stat_tx_slot_avail,
                                    slot_timeout_us)) {
            service->line_error_count++;
            uart_rs485_1_set_receive_direction();
            return false;
        }
        uart_write_byte(BOARD_RS485_1_UART_BASE, data[i]);
    }

    if (!uart_rs485_wait_status(uart_stat_transmitter_empty,
                                empty_timeout_us)) {
        service->line_error_count++;
        uart_rs485_1_set_receive_direction();
        return false;
    }

    board_delay_us(UART_RS485_DE_HOLD_US);
    uart_rs485_1_set_receive_direction();
    service->tx_count += (uint32_t)size;
    return true;
}

size_t uart_rs485_1_hw_read(uart_rs485_hw_t *service,
                            uint8_t *out,
                            size_t max_size)
{
    size_t copied = 0U;

    if (service == 0 || out == 0 || max_size == 0U) {
        return 0U;
    }

    taskENTER_CRITICAL();
    while (copied < max_size && service->rx_size > 0U) {
        out[copied] = service->rx_buffer[service->rx_tail];
        service->rx_tail++;
        if (service->rx_tail >= sizeof(service->rx_buffer)) {
            service->rx_tail = 0U;
        }
        service->rx_size--;
        copied++;
    }
    taskEXIT_CRITICAL();

    return copied;
}

void uart_rs485_1_hw_clear_rx(uart_rs485_hw_t *service)
{
    if (service == 0) {
        return;
    }

    taskENTER_CRITICAL();
    service->rx_size = 0U;
    service->rx_head = 0U;
    service->rx_tail = 0U;
    uart_reset_rx_fifo(BOARD_RS485_1_UART_BASE);
    taskEXIT_CRITICAL();
}

SDK_DECLARE_EXT_ISR_M(BOARD_RS485_1_UART_IRQ, uart_rs485_1_hw_isr)
void uart_rs485_1_hw_isr(void)
{
    uint8_t irq_id = uart_get_irq_id(BOARD_RS485_1_UART_BASE);

    uart_rs485_note_line_status_errors();

    if ((irq_id == uart_intr_id_rx_data_avail) ||
        (irq_id == uart_intr_id_rx_timeout) ||
        uart_check_status(BOARD_RS485_1_UART_BASE, uart_stat_data_ready)) {
        uart_rs485_drain_rx_fifo();
    }

#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    if (uart_is_rxline_idle(BOARD_RS485_1_UART_BASE)) {
        uart_rs485_drain_rx_fifo();
        uart_clear_rxline_idle_flag(BOARD_RS485_1_UART_BASE);
    }
#endif
}
