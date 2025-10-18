/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */


// src/main/rx/mavlink.c
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "common/time.h"      // micros(), millis()
#include "common/utils.h"     // UNUSED macro
#include "drivers/serial.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "common/mavlink.h"

#include "drivers/time.h"     // millis

#define MAVLINK_BAUDRATE 115200
#define MAVLINK_COMM_CHANNEL MAVLINK_COMM_0
#define MAVLINK_INACTIVE_TIMEOUT_US 200000  // 200ms
#define MAVLINK_HEARTBEAT_INTERVAL_US 1000000  // 1 Hz
#define MAVLINK_CHANNEL_COUNT 18

static uint16_t mavlinkChannelData[MAVLINK_CHANNEL_COUNT];
static bool frameReceived = false;
static timeUs_t lastFrameTimeUs = 0;


static serialPort_t *mavlinkPort = NULL;
//static mavlink_status_t mavStatus;
//static mavlink_message_t mavMsg;

static timeUs_t lastHeartbeatTimeUs = 0;
bool mavlinkIsActive(void) {
    return mavlinkPort != NULL;
}

void mavlinkRxHandleMessage(const mavlink_rc_channels_override_t *msg) {
    const uint16_t *channelsPtr = (const uint16_t*)&msg->chan1_raw;
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; i++) {
        if (channelsPtr[i] != 0 && channelsPtr[i] != UINT16_MAX) {
            mavlinkChannelData[i] = channelsPtr[i];
        }
    }
    frameReceived = true;
    
    lastFrameTimeUs = micros();
}

static uint8_t mavlinkFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    if (frameReceived) {
        frameReceived = false;
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
}

static float mavlinkReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    return mavlinkChannelData[channel];
}




bool mavlinkRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    if (mavlinkIsActive()) return true;
    // set up rx API
    frameReceived = false;
    rxRuntimeState->channelData = mavlinkChannelData;
    rxRuntimeState->channelCount = MAVLINK_CHANNEL_COUNT;
    rxRuntimeState->rcReadRawFn = mavlinkReadRawRC;
    rxRuntimeState->rcFrameStatusFn = mavlinkFrameStatus;

    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; ++i) {
        mavlinkChannelData[i] = rxConfig->midrc;
    }
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL); 
    if (!portConfig) {
        return false;
    }

    mavlinkPort = openSerialPort(portConfig->identifier,
                                 FUNCTION_RX_SERIAL,
                                 NULL, NULL,
                                 MAVLINK_BAUDRATE,
                                 MODE_RXTX,
                                 SERIAL_NOT_INVERTED);


    return mavlinkPort != NULL;
}


bool isValidMavlinkTxBuffer (void) {
    //TODO
    return false;
}
bool shouldSendMavlinkTelemetry(void) {
    return false;
}



void mavlinkSendHeartbeat(void)
{
    if (!mavlinkPort) {
        return;
    }

    const uint32_t now = micros();
    if (now - lastHeartbeatTimeUs < MAVLINK_HEARTBEAT_INTERVAL_US) {
        return; // not time yet
    }
    lastHeartbeatTimeUs = now;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Construct heartbeat message
    mavlink_msg_heartbeat_pack(
        1,                      // system_id (e.g. your TX)
        MAV_COMP_ID_TELEMETRY_RADIO,  // component_id
        &msg,
        MAV_TYPE_GCS,           // type: GCS or generic controller
        MAV_AUTOPILOT_INVALID,  // autopilot type (not an FC)
        0,                      // base_mode
        0,                      // custom_mode
        MAV_STATE_ACTIVE        // system status
    );

    // Convert to wire format
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send over UART
    for (uint16_t i = 0; i < len; i++) {
        serialWrite(mavlinkPort, buf[i]);
    }
}


void mavlinkRxUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

#ifdef USE_SERIALRX_MAVLINK

    if (!mavlinkPort) {
        return; // Port not ready yet
    }

    // 2. Check for available bytes
    while (serialRxBytesWaiting(mavlinkPort)) {
        uint8_t c = serialRead(mavlinkPort);

        mavlink_message_t msg;
        mavlink_status_t status;

        // 3. Parse the byte
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
                mavlink_rc_channels_override_t rc;
                mavlink_msg_rc_channels_override_decode(&msg, &rc);
                mavlinkRxHandleMessage(&rc);
                break;
            }
            default:
                break;
            }
        }
    }

    // 4. Optionally send periodic heartbeat (non-blocking)
    mavlinkSendHeartbeat();
#endif
}