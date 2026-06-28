#include "can_bus_hw.h"

#include <string.h>

#include "board.h"
#include "ecu_config.h"
#include "hpm_can_drv.h"
#include "hpm_common.h"
#include "hpm_interrupt.h"

#ifndef CAN_BUS_HW_MAX_RX_FRAMES_PER_POLL
#define CAN_BUS_HW_MAX_RX_FRAMES_PER_POLL (16U)
#endif

static can_bus_service_t *s_can2_service;

static uint8_t can_bus_hw_size_from_dlc(uint8_t dlc)
{
    return dlc <= CANOPEN_FRAME_MAX_DATA_BYTES ? dlc : CANOPEN_FRAME_MAX_DATA_BYTES;
}

static void can_bus_hw_record_rx_message(can_bus_service_t *service,
                                         const can_receive_buf_t *rx_message)
{
    uint8_t size = can_bus_hw_size_from_dlc((uint8_t)rx_message->dlc);
    can_bus_service_note_rx_from_isr(service,
                                     rx_message->id,
                                     rx_message->extend_id != 0U,
                                     rx_message->remote_frame != 0U,
                                     rx_message->data,
                                     size);
}

static void can_bus_hw_update_can2_debug_status(can_bus_service_t *service)
{
    if (service == 0) {
        return;
    }

    can_bus_service_note_controller_status(service,
                                           can_get_receive_buffer_status(BOARD_CAN2_BASE),
                                           can_get_tx_rx_flags(BOARD_CAN2_BASE),
                                           can_get_error_interrupt_flags(BOARD_CAN2_BASE),
                                           can_get_receive_error_count(BOARD_CAN2_BASE),
                                           can_get_transmit_error_count(BOARD_CAN2_BASE),
                                           can_get_last_error_kind(BOARD_CAN2_BASE));
}

bool can_bus_hw_init_can2_rx_only(can_bus_service_t *service, uint32_t bitrate)
{
    if (service == 0 || bitrate == 0U) {
        return false;
    }

    can_config_t can_config;
    if (can_get_default_config(&can_config) != status_success) {
        return false;
    }

    board_init_can(BOARD_CAN2_BASE);
    board_set_can_termination(2U, ECU_CAN2_TERMINATION_ENABLE != 0);

    can_config.baudrate = bitrate;
    can_config.mode = can_mode_normal;
    can_config.enable_canfd = false;
    can_config.irq_txrx_enable_mask = CAN_EVENT_RECEIVE |
                                      CAN_EVENT_RX_BUF_OVERRUN |
                                      CAN_EVENT_ERROR;
    can_config.irq_error_enable_mask = CAN_ERROR_PASSIVE_INT_ENABLE |
                                       CAN_ERROR_ARBITRATION_LOST_INT_ENABLE |
                                       CAN_ERROR_BUS_ERROR_INT_ENABLE;

    uint32_t src_clk_hz = board_init_can_clock(BOARD_CAN2_BASE);
    hpm_stat_t status = can_init(BOARD_CAN2_BASE, &can_config, src_clk_hz);
    if (status != status_success) {
        s_can2_service = 0;
        return false;
    }

    s_can2_service = service;
    can_bus_hw_update_can2_debug_status(service);
    intc_m_enable_irq_with_priority(BOARD_CAN2_IRQn, 2);
    return true;
}

void can_bus_hw_poll_can2_rx(can_bus_service_t *service)
{
    if (service == 0) {
        return;
    }

    uint32_t frames_drained = 0U;
    while ((frames_drained < CAN_BUS_HW_MAX_RX_FRAMES_PER_POLL) &&
           can_is_data_available_in_receive_buffer(BOARD_CAN2_BASE)) {
        can_receive_buf_t rx_message;
        memset(&rx_message, 0, sizeof(rx_message));

        hpm_stat_t status = can_read_received_message(BOARD_CAN2_BASE, &rx_message);
        if (status == status_success) {
            can_bus_hw_record_rx_message(service, &rx_message);
            frames_drained++;
        } else {
            can_bus_service_note_error_from_isr(service);
            break;
        }
    }
    can_bus_hw_update_can2_debug_status(service);
}

#if !ECU_ENABLE_CANOPENNODE
SDK_DECLARE_EXT_ISR_M(BOARD_CAN2_IRQn, can_bus_hw_can2_isr)
#endif
void can_bus_hw_can2_isr(void)
{
    uint8_t flags = can_get_tx_rx_flags(BOARD_CAN2_BASE);

    if ((flags & CAN_EVENT_RECEIVE) != 0U) {
        can_receive_buf_t rx_message;
        memset(&rx_message, 0, sizeof(rx_message));

        hpm_stat_t status = can_read_received_message(BOARD_CAN2_BASE, &rx_message);
        if (status == status_success) {
            can_bus_hw_record_rx_message(s_can2_service, &rx_message);
        } else {
            can_bus_service_note_error_from_isr(s_can2_service);
        }
    }

    if ((flags & (CAN_EVENT_RX_BUF_OVERRUN | CAN_EVENT_ERROR)) != 0U) {
        can_bus_service_note_error_from_isr(s_can2_service);
    }
    can_clear_tx_rx_flags(BOARD_CAN2_BASE, flags);

    uint8_t error_flags = can_get_error_interrupt_flags(BOARD_CAN2_BASE);
    if (error_flags != 0U) {
        can_bus_service_note_error_from_isr(s_can2_service);
        can_clear_error_interrupt_flags(BOARD_CAN2_BASE, error_flags);
    }
    can_bus_hw_update_can2_debug_status(s_can2_service);
}
