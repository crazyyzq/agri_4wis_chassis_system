/* CPU0-owned SBUS receiver boundary. ISR feed is intentionally lightweight. */
#ifndef SBUS_SERVICE_H
#define SBUS_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "sbus_decoder.h"

typedef struct {
    uint16_t channels[SBUS_CHANNEL_COUNT];
    bool valid;
    bool connected;
    bool failsafe;
    bool frame_lost;
    bool channel17;
    bool channel18;
    uint32_t frame_count;
    uint32_t decode_error_count;
    uint32_t uart_error_count;
    uint32_t last_frame_ms;
} sbus_service_snapshot_t;

typedef struct {
    uint8_t frame_buffer[SBUS_FRAME_LENGTH];
    uint8_t frame_position;
    uint32_t timeout_ms;
    sbus_service_snapshot_t snapshot;
} sbus_service_t;

void sbus_service_init(sbus_service_t *service, uint32_t timeout_ms);
void sbus_service_feed_byte_from_isr(sbus_service_t *service, uint8_t byte, uint32_t now_ms);
void sbus_service_note_uart_error_from_isr(sbus_service_t *service);
void sbus_service_poll(sbus_service_t *service, uint32_t now_ms);
void sbus_service_get_snapshot(const sbus_service_t *service, sbus_service_snapshot_t *out);

#endif /* SBUS_SERVICE_H */
