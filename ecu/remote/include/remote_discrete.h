#ifndef REMOTE_DISCRETE_H
#define REMOTE_DISCRETE_H

#include <stdint.h>

#include "ecu_config.h"
#include "remote_types.h"

void remote_discrete_init(remote_discrete_channel_t *channel, remote_position_t baseline, uint32_t now_ms);
void remote_discrete_sync_baseline(remote_discrete_channel_t *channel, remote_position_t baseline, uint32_t now_ms);
void remote_discrete_update(remote_discrete_channel_t *channel,
                            remote_position_t candidate,
                            uint32_t now_ms,
                            uint32_t debounce_ms);
remote_position_t remote_discrete_position_from_raw(uint16_t raw, const ecu_sbus_thresholds_t *thresholds);

#endif /* REMOTE_DISCRETE_H */
