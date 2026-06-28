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

static can_bus_service_t *s_can1_service;
static can_bus_service_t *s_can2_service;

static uint8_t can_bus_hw_size_from_dlc(uint8_t dlc)
{
    return dlc <= CAN_BUS_FRAME_MAX_DATA_BYTES ? dlc : CAN_BUS_FRAME_MAX_DATA_BYTES;
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

static void can_bus_hw_update_status(can_bus_service_t *service, CAN_Type *base)
{
    if (service == 0) {
        return;
    }

    can_bus_service_note_controller_status(service,
                                           can_get_receive_buffer_status(base),
                                           can_get_tx_rx_flags(base),
                                           can_get_error_interrupt_flags(base),
                                           can_get_receive_error_count(base),
                                           can_get_transmit_error_count(base),
                                           can_get_last_error_kind(base));
}

static bool can_bus_hw_init_classic_bus(CAN_Type *base,
                                        uint32_t bus_index,
                                        bool enable_tx,
                                        can_bus_service_t *service,
                                        uint32_t bitrate)
{
    if (service == 0 || bitrate == 0U) {
        return false;
    }

    can_config_t can_config;
    if (can_get_default_config(&can_config) != status_success) {
        return false;
    }

    board_init_can(base);
    if (bus_index == 1U) {
        board_set_can_termination(1U, ECU_CAN1_TERMINATION_ENABLE != 0);
    } else if (bus_index == 2U) {
        board_set_can_termination(2U, ECU_CAN2_TERMINATION_ENABLE != 0);
    }

    can_config.baudrate = bitrate;
    can_config.mode = can_mode_normal;
    can_config.enable_canfd = false;
    can_config.irq_txrx_enable_mask = CAN_EVENT_RECEIVE |
                                      CAN_EVENT_RX_BUF_OVERRUN |
                                      CAN_EVENT_ERROR;
    can_config.irq_error_enable_mask = CAN_ERROR_PASSIVE_INT_ENABLE |
                                       CAN_ERROR_ARBITRATION_LOST_INT_ENABLE |
                                       CAN_ERROR_BUS_ERROR_INT_ENABLE;

    uint32_t src_clk_hz = board_init_can_clock(base);
    hpm_stat_t status = can_init(base, &can_config, src_clk_hz);
    if (status != status_success) {
        return false;
    }

    if (!enable_tx) {
        can_bus_service_set_tx_backend(service, 0);
    }
    can_bus_hw_update_status(service, base);
    return true;
}

bool can_bus_hw_init_can1_power(can_bus_service_t *service, uint32_t bitrate)
{
    if (!can_bus_hw_init_classic_bus(BOARD_CAN1_BASE, 1U, true, service, bitrate)) {
        s_can1_service = 0;
        return false;
    }

    s_can1_service = service;
    can_bus_service_set_tx_backend(service, can_bus_hw_send_can1_frame);
    intc_m_enable_irq_with_priority(BOARD_CAN1_IRQn, 2);
    return true;
}

bool can_bus_hw_init_can2_rx_only(can_bus_service_t *service, uint32_t bitrate)
{
    if (!can_bus_hw_init_classic_bus(BOARD_CAN2_BASE, 2U, false, service, bitrate)) {
        s_can2_service = 0;
        return false;
    }

    s_can2_service = service;
    intc_m_enable_irq_with_priority(BOARD_CAN2_IRQn, 2);
    return true;
}

static void can_bus_hw_poll_rx(CAN_Type *base, can_bus_service_t *service)
{
    if (service == 0) {
        return;
    }

    uint32_t frames_drained = 0U;
    while ((frames_drained < CAN_BUS_HW_MAX_RX_FRAMES_PER_POLL) &&
           can_is_data_available_in_receive_buffer(base)) {
        can_receive_buf_t rx_message;
        memset(&rx_message, 0, sizeof(rx_message));

        hpm_stat_t status = can_read_received_message(base, &rx_message);
        if (status == status_success) {
            can_bus_hw_record_rx_message(service, &rx_message);
            frames_drained++;
        } else {
            can_bus_service_note_error_from_isr(service);
            break;
        }
    }
    can_bus_hw_update_status(service, base);
}

void can_bus_hw_poll_can1_rx(can_bus_service_t *service)
{
    can_bus_hw_poll_rx(BOARD_CAN1_BASE, service);
}

void can_bus_hw_poll_can2_rx(can_bus_service_t *service)
{
    can_bus_hw_poll_rx(BOARD_CAN2_BASE, service);
}

bool can_bus_hw_send_can1_frame(const ecu_can_frame_t *frame)
{
    if (frame == 0 || frame->size > CAN_BUS_FRAME_MAX_DATA_BYTES) {
        return false;
    }

    can_transmit_buf_t tx_message;
    memset(&tx_message, 0, sizeof(tx_message));
    tx_message.id = frame->id & 0x1FFFFFFFUL;
    tx_message.dlc = frame->size;
    tx_message.remote_frame = frame->remote;
    tx_message.extend_id = frame->extended;
    tx_message.canfd_frame = 0U;
    tx_message.bitrate_switch = 0U;
    if (!frame->remote && frame->size > 0U) {
        memcpy(tx_message.data, frame->data, frame->size);
    }

    hpm_stat_t status;
    if (!can_is_primary_transmit_buffer_full(BOARD_CAN1_BASE)) {
        status = can_send_high_priority_message_nonblocking(BOARD_CAN1_BASE, &tx_message);
    } else {
        status = can_send_message_nonblocking(BOARD_CAN1_BASE, &tx_message);
    }

    if (status != status_success) {
        can_bus_service_note_error_from_isr(s_can1_service);
    }
    can_bus_hw_update_status(s_can1_service, BOARD_CAN1_BASE);
    return status == status_success;
}

static void can_bus_hw_handle_isr(CAN_Type *base, can_bus_service_t *service)
{
    uint8_t flags = can_get_tx_rx_flags(base);

    if ((flags & CAN_EVENT_RECEIVE) != 0U) {
        can_receive_buf_t rx_message;
        memset(&rx_message, 0, sizeof(rx_message));

        hpm_stat_t status = can_read_received_message(base, &rx_message);
        if (status == status_success) {
            can_bus_hw_record_rx_message(service, &rx_message);
        } else {
            can_bus_service_note_error_from_isr(service);
        }
    }

    if ((flags & (CAN_EVENT_RX_BUF_OVERRUN | CAN_EVENT_ERROR)) != 0U) {
        can_bus_service_note_error_from_isr(service);
    }
    can_clear_tx_rx_flags(base, flags);

    uint8_t error_flags = can_get_error_interrupt_flags(base);
    if (error_flags != 0U) {
        can_bus_service_note_error_from_isr(service);
        can_clear_error_interrupt_flags(base, error_flags);
    }
    can_bus_hw_update_status(service, base);
}

SDK_DECLARE_EXT_ISR_M(BOARD_CAN1_IRQn, can_bus_hw_can1_isr)
void can_bus_hw_can1_isr(void)
{
    can_bus_hw_handle_isr(BOARD_CAN1_BASE, s_can1_service);
}

#if !ECU_ENABLE_CANOPENNODE
SDK_DECLARE_EXT_ISR_M(BOARD_CAN2_IRQn, can_bus_hw_can2_isr)
#endif
void can_bus_hw_can2_isr(void)
{
    can_bus_hw_handle_isr(BOARD_CAN2_BASE, s_can2_service);
}
