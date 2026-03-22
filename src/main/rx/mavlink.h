/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define MAVLINK_COMM_NUM_BUFFERS 1
#define RSSI_DBM_MIN (-130)
#define RSSI_DBM_MAX 0

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "common/mavlink.h"
#include "rx/rx.h"   
#pragma GCC diagnostic pop

typedef enum {
    ATTITUDE = 0,
    MAV_ENUM_END
} mavlink_msg_type;

typedef struct _mavlink_msg_cfg {
    mavlink_msg_type type;
    char* name;
    int freq;
} mavlink_msg_cfg;

extern mavlink_msg_cfg mavlink_msg_configs[];

#define NUM_CONFIGS sizeof(mavlink_msg_configs) / sizeof(mavlink_msg_configs[0])

bool mavlinkSetMsgFreq(mavlink_msg_type type, int freq);
void mavlinkRxHandleMessage(const mavlink_rc_channels_override_t *msg);
bool mavlinkRxInit(const rxConfig_t *initialRxConfig, rxRuntimeState_t *rxRuntimeState);
bool mavlinkRxClose(rxRuntimeState_t *rxRuntimeState);
void mavlinkProcess(void);
void taskProcessMavlink(timeUs_t currentTimeUs);
void taskMavlinkSendHeartbeats(timeUs_t currentTimeUs);
void updateRcCommandsFromMavlink(void);
void mavlinkCustomRxInit(void);
timeMs_t mavlinkGetLastHeartbeatMs(void);
uint32_t mavlinkGetLastSentAckId(void);


#if defined(USE_SERIAL_MAVLINK) || defined (USE_MAVLINK)
bool isValidMavlinkTxBuffer (void);
bool shouldSendMavlinkTelemetry(void);
#else
static inline bool isValidMavlinkTxBuffer(void) { return false; }
static inline bool shouldSendMavlinkTelemetry(void) { return false; }
#endif
