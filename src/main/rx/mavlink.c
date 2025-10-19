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

#include "build/debug.h"
#include "common/time.h"     
#include "drivers/serial.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "common/mavlink.h"

#include "drivers/time.h"     
#include "telemetry/telemetry.h"


#ifdef USE_SERIALRX_MAVLINK

#define MAVLINK_BAUDRATE 115200
#define MAVLINK_COMM_CHANNEL MAVLINK_COMM_0
#define MAVLINK_INACTIVE_TIMEOUT_US 200000  // 200ms
#define MAVLINK_HEARTBEAT_INTERVAL_US 1000000  // 1 Hz


#define MAVLINK_CHANNEL_COUNT 18
#define MAVLINK_BAUD_RATE_INDEX BAUD_460800
static uint16_t mavlinkChannelData[MAVLINK_CHANNEL_COUNT];
static bool frameReceived;

static uint16_t mavlinkChannelData[MAVLINK_CHANNEL_COUNT];
static bool frameReceived = false;
static timeUs_t lastFrameTimeUs = 0;

static mavlink_message_t mavRecvMsg;
static mavlink_status_t mavRecvStatus;
static volatile uint8_t txbuff_free = 100;  // tx buffer space in %, start with empty buffer
static volatile bool txbuff_valid = false;
static serialPort_t *mavlinkPort = NULL;

bool mavlinkIsActive(void) {
    return mavlinkPort != NULL;
}

void mavlinkSendHeartbeat(timeUs_t now)
{   
    UNUSED(now);
    if (!mavlinkIsActive() || mavlinkPort == NULL) {
        return;
    }

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(
        1,                 // system ID (1 for FC)
        200,               // component ID (usually 200 for autopilot)
        &msg,
        MAV_TYPE_QUADROTOR,       // type of vehicle
        MAV_AUTOPILOT_GENERIC,    // autopilot type
        MAV_MODE_MANUAL_ARMED,    // system mode
        0,                        // custom mode
        MAV_STATE_ACTIVE          // system state
    );


    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    for(int i = 0; i < len; i++)
        serialWrite(mavlinkPort, buf[1]);
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

static bool handleIncoming_RC_CHANNELS_OVERRIDE(void) {
    mavlink_rc_channels_override_t msg;
    mavlink_msg_rc_channels_override_decode(&mavRecvMsg, &msg);
    mavlinkRxHandleMessage(&msg);
    return true;
}

// Get RADIO_STATUS data
static void handleIncoming_RADIO_STATUS(void)
{
    mavlink_radio_status_t msg;
    mavlink_msg_radio_status_decode(&mavRecvMsg, &msg);
    txbuff_valid = true;
    txbuff_free = msg.txbuf;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 1, txbuff_free); // Last known TX buffer free space
}

static float mavlinkReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    return mavlinkChannelData[channel];
}


STATIC_UNIT_TESTED void mavlinkDataReceive(uint16_t c, void *data)
{
    rxRuntimeState_t *const rxRuntimeState = (rxRuntimeState_t *const)data;
    uint8_t result = mavlink_parse_char(0, c, &mavRecvMsg, &mavRecvStatus);
    if (result == MAVLINK_FRAMING_OK) {
        switch (mavRecvMsg.msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            handleIncoming_RC_CHANNELS_OVERRIDE();
            rxRuntimeState->lastRcFrameTimeUs = micros();
            break;
        case MAVLINK_MSG_ID_RADIO_STATUS:
            handleIncoming_RADIO_STATUS();
            break;
        }
    }
}

bool mavlinkRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    frameReceived = false;
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; ++i) {
        mavlinkChannelData[i] = rxConfig->midrc;
    }

    rxRuntimeState->channelData = mavlinkChannelData;
    rxRuntimeState->channelCount = MAVLINK_CHANNEL_COUNT;
    rxRuntimeState->rcReadRawFn = mavlinkReadRawRC;
    rxRuntimeState->rcFrameStatusFn = mavlinkFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO || (portConfig->functionMask & FUNCTION_TELEMETRY_MAVLINK) == 0) {
        // default rate for ELRS TX module
        baudRateIndex = MAVLINK_BAUD_RATE_INDEX;
    }

    mavlinkPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        mavlinkDataReceive,
        rxRuntimeState,
        baudRates[baudRateIndex],
        MODE_RXTX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0)
    );
#ifdef USE_TELEMETRY_MAVLINK
    telemetrySharedPort = mavlinkPort;
#endif

    return mavlinkPort != NULL;
}

#ifdef USE_TELEMETRY_MAVLINK
bool isValidMavlinkTxBuffer (void) {
    return txbuff_valid;
}

bool shouldSendMavlinkTelemetry(void) {
    const uint8_t mavlink_min_txbuff =  telemetryConfig()->mavlink_min_txbuff;
    bool shouldSendTelemetry = false;

    if (txbuff_valid) {
        shouldSendTelemetry = txbuff_free > mavlink_min_txbuff;
    }
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 0, shouldSendTelemetry ? 1 : 0);

    return shouldSendTelemetry;
}
#endif

#endif 