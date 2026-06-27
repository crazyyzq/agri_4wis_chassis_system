#include <string.h>

#include "can_bus_service.h"

void can_bus_service_init(can_bus_service_t *service, uint32_t bitrate)
{
    if (service == 0) {
        return;
    }
    memset(service, 0, sizeof(*service));
    service->bitrate = bitrate;
    service->online = true;
    service->diagnostic = DIAG_OK;
}

void can_bus_service_set_online(can_bus_service_t *service, bool online)
{
    if (service != 0) {
        service->online = online;
    }
}

bool can_bus_service_send_canopen(can_bus_service_t *service,
                                  const canopen_frame_t *frame)
{
    if (service == 0 || frame == 0 || !service->online) {
        return false;
    }
    service->last_tx = *frame;
    service->tx_count++;
    service->diagnostic = DIAG_OK;
    return true;
}
