#include <string.h>

#include "ecu_time.h"
#include "sbus_service.h"

void sbus_service_init(sbus_service_t *service, uint32_t timeout_ms)
{
    if (service == 0) {
        return;
    }
    memset(service, 0, sizeof(*service));
    service->timeout_ms = timeout_ms;
}

static void copy_decoded(sbus_service_snapshot_t *snapshot,
                         const sbus_decoded_frame_t *decoded,
                         uint32_t now_ms)
{
    for (uint8_t i = 0U; i < SBUS_CHANNEL_COUNT; ++i) {
        snapshot->channels[i] = decoded->channels[i];
    }
    snapshot->valid = true;
    snapshot->connected = !decoded->failsafe;
    snapshot->failsafe = decoded->failsafe;
    snapshot->frame_lost = decoded->frame_lost;
    snapshot->channel17 = decoded->channel17;
    snapshot->channel18 = decoded->channel18;
    snapshot->last_frame_ms = now_ms;
    snapshot->frame_count++;
}

void sbus_service_feed_byte_from_isr(sbus_service_t *service, uint8_t byte, uint32_t now_ms)
{
    if (service == 0) {
        return;
    }
    if (service->frame_position == 0U && byte != 0x0FU) {
        return;
    }
    service->frame_buffer[service->frame_position++] = byte;
    if (service->frame_position < SBUS_FRAME_LENGTH) {
        return;
    }

    sbus_decoded_frame_t decoded;
    if (sbus_decode_frame(service->frame_buffer, SBUS_FRAME_LENGTH, &decoded) == SBUS_DECODE_OK) {
        copy_decoded(&service->snapshot, &decoded, now_ms);
    } else {
        service->snapshot.decode_error_count++;
    }
    service->frame_position = 0U;
}

void sbus_service_note_uart_error_from_isr(sbus_service_t *service)
{
    if (service != 0) {
        service->snapshot.uart_error_count++;
    }
}

void sbus_service_poll(sbus_service_t *service, uint32_t now_ms)
{
    if (service == 0 || !service->snapshot.valid) {
        return;
    }
    if (ecu_time_elapsed(now_ms, service->snapshot.last_frame_ms, service->timeout_ms)) {
        service->snapshot.connected = false;
    }
}

void sbus_service_get_snapshot(const sbus_service_t *service, sbus_service_snapshot_t *out)
{
    if (service == 0 || out == 0) {
        return;
    }
    *out = service->snapshot;
}
