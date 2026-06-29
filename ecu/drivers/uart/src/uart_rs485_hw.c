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

typedef struct {
    UART_Type *base;
    int32_t irq;
    GPIO_Type *de_gpio_ctrl;
    uint32_t de_gpio_index;
    uint32_t de_gpio_pin;
} uart_rs485_port_desc_t;

static const uart_rs485_port_desc_t s_rs485_ports[UART_RS485_HW_PORT_COUNT] = {
    {
        .base = BOARD_RS485_1_UART_BASE,
        .irq = BOARD_RS485_1_UART_IRQ,
        .de_gpio_ctrl = BOARD_RS485_1_DE_GPIO_CTRL,
        .de_gpio_index = BOARD_RS485_1_DE_GPIO_INDEX,
        .de_gpio_pin = BOARD_RS485_1_DE_GPIO_PIN,
    },
    {
        .base = BOARD_RS485_2_UART_BASE,
        .irq = BOARD_RS485_2_UART_IRQ,
        .de_gpio_ctrl = BOARD_RS485_2_DE_GPIO_CTRL,
        .de_gpio_index = BOARD_RS485_2_DE_GPIO_INDEX,
        .de_gpio_pin = BOARD_RS485_2_DE_GPIO_PIN,
    },
};

static uart_rs485_hw_t *s_rs485_services[UART_RS485_HW_PORT_COUNT];

static const uart_rs485_port_desc_t *uart_rs485_get_desc(uint8_t port_index)
{
    if (port_index >= (uint8_t)UART_RS485_HW_PORT_COUNT) {
        return 0;
    }
    return &s_rs485_ports[port_index];
}

static void uart_rs485_set_receive_direction(const uart_rs485_port_desc_t *desc)
{
    if (desc == 0) {
        return;
    }
#if BOARD_RS485_DE_USING_GPIO
    gpio_write_pin(desc->de_gpio_ctrl,
                   desc->de_gpio_index,
                   desc->de_gpio_pin,
                   BOARD_RS485_RX_ENABLE_LEVEL);
#endif
}

static void uart_rs485_set_transmit_direction(const uart_rs485_port_desc_t *desc)
{
    if (desc == 0) {
        return;
    }
#if BOARD_RS485_DE_USING_GPIO
    gpio_write_pin(desc->de_gpio_ctrl,
                   desc->de_gpio_index,
                   desc->de_gpio_pin,
                   BOARD_RS485_TX_ENABLE_LEVEL);
#endif
}

static void uart_rs485_1_set_receive_direction(void)
{
    uart_rs485_set_receive_direction(&s_rs485_ports[UART_RS485_HW_PORT_1]);
}

static void uart_rs485_1_set_transmit_direction(void)
{
    uart_rs485_set_transmit_direction(&s_rs485_ports[UART_RS485_HW_PORT_1]);
}

static void uart_rs485_note_line_status_errors(uint8_t port_index)
{
    const uart_rs485_port_desc_t *desc = uart_rs485_get_desc(port_index);
    uart_rs485_hw_t *service = port_index < (uint8_t)UART_RS485_HW_PORT_COUNT ?
                               s_rs485_services[port_index] : 0;
    if (desc == 0) {
        return;
    }

    uint32_t status = uart_get_status(desc->base);
    const uint32_t error_mask = (uint32_t)uart_stat_overrun_error |
                                (uint32_t)uart_stat_parity_error |
                                (uint32_t)uart_stat_framing_error |
                                (uint32_t)uart_stat_line_break |
                                (uint32_t)uart_stat_rx_fifo_error;

    if ((status & error_mask) != 0U && service != 0) {
        service->line_error_count++;
    }
}

static void uart_rs485_push_rx_byte(uint8_t port_index, uint8_t byte)
{
    uart_rs485_hw_t *service = port_index < (uint8_t)UART_RS485_HW_PORT_COUNT ?
                               s_rs485_services[port_index] : 0;
    if (service == 0) {
        return;
    }

    if (service->rx_size >= sizeof(service->rx_buffer)) {
        service->rx_overflow_count++;
        return;
    }

    service->rx_buffer[service->rx_head] = byte;
    service->rx_head++;
    if (service->rx_head >= sizeof(service->rx_buffer)) {
        service->rx_head = 0U;
    }
    service->rx_size++;
    service->rx_count++;
}

static void uart_rs485_drain_rx_fifo(uint8_t port_index)
{
    const uart_rs485_port_desc_t *desc = uart_rs485_get_desc(port_index);
    uart_rs485_hw_t *service = port_index < (uint8_t)UART_RS485_HW_PORT_COUNT ?
                               s_rs485_services[port_index] : 0;
    if (desc == 0) {
        return;
    }

    uint32_t drained = 0U;
    while (uart_check_status(desc->base, uart_stat_data_ready)) {
        uart_rs485_push_rx_byte(port_index, uart_read_byte(desc->base));
        drained++;
        if (drained >= UART_RS485_MAX_BYTES_PER_ISR) {
            if (service != 0) {
                service->line_error_count++;
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

static bool uart_rs485_wait_status(UART_Type *base,
                                   uart_stat_t status,
                                   uint32_t timeout_us)
{
    uint32_t elapsed_us = 0U;

    while (!uart_check_status(base, status)) {
        if (elapsed_us >= timeout_us) {
            return false;
        }
        board_delay_us(1U);
        elapsed_us++;
    }

    return true;
}

static bool uart_rs485_hw_init_port(uart_rs485_hw_t *service,
                                    uart_rs485_hw_port_t port,
                                    uint32_t baudrate)
{
    const uart_rs485_port_desc_t *desc = uart_rs485_get_desc((uint8_t)port);
    uart_config_t config = {0};

    if (service == 0 || desc == 0 || baudrate == 0U) {
        return false;
    }

    memset(service, 0, sizeof(*service));
    service->baudrate = baudrate;
    service->port_index = (uint8_t)port;
    board_init_uart(desc->base);
    uart_default_config(desc->base, &config);

    config.src_freq_in_hz = board_init_uart_clock(desc->base);
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

    hpm_stat_t stat = uart_init(desc->base, &config);
    if (stat != status_success) {
        s_rs485_services[(uint8_t)port] = 0;
        return false;
    }

#if defined(HPM_IP_FEATURE_UART_DE_DELAY) && (HPM_IP_FEATURE_UART_DE_DELAY == 1)
    uart_set_de_delay_before_start_bit(desc->base, 1U);
    uart_set_de_delay_after_stop_bit(desc->base, 1U);
#endif

    s_rs485_services[(uint8_t)port] = service;
    service->initialized = true;
    if (port == UART_RS485_HW_PORT_1) {
        uart_rs485_1_set_receive_direction();
    } else {
        uart_rs485_set_receive_direction(desc);
    }
    uart_reset_rx_fifo(desc->base);
    uart_reset_tx_fifo(desc->base);
    uart_enable_irq(desc->base,
                    uart_intr_rx_data_avail_or_timeout | uart_intr_rx_line_stat);

#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    uart_clear_rxline_idle_flag(desc->base);
    uart_enable_irq(desc->base, uart_intr_rx_line_idle);
#endif

    intc_m_enable_irq_with_priority((uint32_t)desc->irq,
                                    ECU_RS485_UART_IRQ_PRIORITY);
    return true;
}

bool uart_rs485_1_hw_init(uart_rs485_hw_t *service, uint32_t baudrate)
{
    return uart_rs485_hw_init_port(service, UART_RS485_HW_PORT_1, baudrate);
}

bool uart_rs485_2_hw_init(uart_rs485_hw_t *service, uint32_t baudrate)
{
    return uart_rs485_hw_init_port(service, UART_RS485_HW_PORT_2, baudrate);
}

bool uart_rs485_hw_send(uart_rs485_hw_t *service,
                        const uint8_t *data,
                        size_t size)
{
    if (service == 0 || !service->initialized || data == 0 || size == 0U) {
        return false;
    }

    const uart_rs485_port_desc_t *desc = uart_rs485_get_desc(service->port_index);
    if (desc == 0) {
        return false;
    }

    uint32_t char_time_us = uart_rs485_char_time_us(service->baudrate);
    uint32_t slot_timeout_us = (char_time_us * 2U) + UART_RS485_TX_WAIT_MARGIN_US;
    uint32_t empty_timeout_us = ((uint32_t)size + 2U) * char_time_us +
                                UART_RS485_TX_WAIT_MARGIN_US;

    /* HPM SDK uart_send_byte()/uart_flush() use a short polling count that can
     * expire at 9600 baud.  Modbus RTU frames are small, so this driver waits
     * in real time for each character slot and returns the transceiver to RX
     * immediately after the final stop bit has left the shifter.
     */
    if (service->port_index == (uint8_t)UART_RS485_HW_PORT_1) {
        uart_rs485_1_set_transmit_direction();
    } else {
        uart_rs485_set_transmit_direction(desc);
    }
    board_delay_us(UART_RS485_DE_SETUP_US);

    for (size_t i = 0U; i < size; ++i) {
        if (!uart_rs485_wait_status(desc->base,
                                    uart_stat_tx_slot_avail,
                                    slot_timeout_us)) {
            service->line_error_count++;
            if (service->port_index == (uint8_t)UART_RS485_HW_PORT_1) {
                uart_rs485_1_set_receive_direction();
            } else {
                uart_rs485_set_receive_direction(desc);
            }
            return false;
        }
        uart_write_byte(desc->base, data[i]);
    }

    if (!uart_rs485_wait_status(desc->base,
                                uart_stat_transmitter_empty,
                                empty_timeout_us)) {
        service->line_error_count++;
        if (service->port_index == (uint8_t)UART_RS485_HW_PORT_1) {
            uart_rs485_1_set_receive_direction();
        } else {
            uart_rs485_set_receive_direction(desc);
        }
        return false;
    }

    board_delay_us(UART_RS485_DE_HOLD_US);
    if (service->port_index == (uint8_t)UART_RS485_HW_PORT_1) {
        uart_rs485_1_set_receive_direction();
    } else {
        uart_rs485_set_receive_direction(desc);
    }
    service->tx_count += (uint32_t)size;
    return true;
}

size_t uart_rs485_hw_read(uart_rs485_hw_t *service,
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

void uart_rs485_hw_clear_rx(uart_rs485_hw_t *service)
{
    if (service == 0) {
        return;
    }

    const uart_rs485_port_desc_t *desc = uart_rs485_get_desc(service->port_index);
    taskENTER_CRITICAL();
    service->rx_size = 0U;
    service->rx_head = 0U;
    service->rx_tail = 0U;
    if (desc != 0) {
        uart_reset_rx_fifo(desc->base);
    }
    taskEXIT_CRITICAL();
}

bool uart_rs485_1_hw_send(uart_rs485_hw_t *service,
                          const uint8_t *data,
                          size_t size)
{
    return uart_rs485_hw_send(service, data, size);
}

size_t uart_rs485_1_hw_read(uart_rs485_hw_t *service,
                            uint8_t *out,
                            size_t max_size)
{
    return uart_rs485_hw_read(service, out, max_size);
}

void uart_rs485_1_hw_clear_rx(uart_rs485_hw_t *service)
{
    uart_rs485_hw_clear_rx(service);
}

static void uart_rs485_hw_handle_isr(uint8_t port_index)
{
    const uart_rs485_port_desc_t *desc = uart_rs485_get_desc(port_index);
    if (desc == 0) {
        return;
    }

    uint8_t irq_id = uart_get_irq_id(desc->base);

    uart_rs485_note_line_status_errors(port_index);

    if ((irq_id == uart_intr_id_rx_data_avail) ||
        (irq_id == uart_intr_id_rx_timeout) ||
        uart_check_status(desc->base, uart_stat_data_ready)) {
        uart_rs485_drain_rx_fifo(port_index);
    }

#if defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
    if (uart_is_rxline_idle(desc->base)) {
        uart_rs485_drain_rx_fifo(port_index);
        uart_clear_rxline_idle_flag(desc->base);
    }
#endif
}

SDK_DECLARE_EXT_ISR_M(BOARD_RS485_1_UART_IRQ, uart_rs485_1_hw_isr)
void uart_rs485_1_hw_isr(void)
{
    uart_rs485_hw_handle_isr((uint8_t)UART_RS485_HW_PORT_1);
}

SDK_DECLARE_EXT_ISR_M(BOARD_RS485_2_UART_IRQ, uart_rs485_2_hw_isr)
void uart_rs485_2_hw_isr(void)
{
    uart_rs485_hw_handle_isr((uint8_t)UART_RS485_HW_PORT_2);
}
