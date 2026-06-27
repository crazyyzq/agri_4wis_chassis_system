#include <string.h>

#include "dio_service.h"

static void dio_service_refresh_physical(dio_service_t *service)
{
    uint32_t logical = service->output_mask & service->managed_output_mask;
    service->physical_output_mask = service->active_high ?
                                    logical :
                                    ((~logical) & service->managed_output_mask);
}

void dio_service_init(dio_service_t *service,
                      bool active_high,
                      uint32_t managed_output_mask)
{
    if (service == 0) {
        return;
    }
    memset(service, 0, sizeof(*service));
    service->active_high = active_high;
    service->managed_output_mask = managed_output_mask;
    dio_service_refresh_physical(service);
}

void dio_service_write_masked(dio_service_t *service,
                              uint32_t mask,
                              bool enabled)
{
    if (service == 0) {
        return;
    }
    mask &= service->managed_output_mask;
    if (enabled) {
        service->output_mask |= mask;
    } else {
        service->output_mask &= ~mask;
    }
    dio_service_refresh_physical(service);
}

uint32_t dio_service_get_physical_mask(const dio_service_t *service)
{
    return service != 0 ? service->physical_output_mask : 0U;
}
