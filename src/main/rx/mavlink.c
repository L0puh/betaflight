#include "telemetry/mavlink.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#include "platform.h"
#include "stm32f7xx_hal_def.h"
#if defined(USE_SERIAL_MAVLINK) || defined(USE_MAVLINK)

#include "common/utils.h"
#include "common/maths.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/mavlink.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "drivers/time.h"
#include "drivers/nvic.h"

#include "build/debug.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop



#define MAVLINK_CHANNEL_COUNT 18
#define MAVLINK_BAUD_RATE_INDEX BAUD_115200  

#define MAV_SYS_ID   1
#define MAV_COMP_ID  MAV_COMP_ID_AUTOPILOT1

typedef struct {
    float roll;
    float pitch;
    float yawRate;
    float thrust;

    timeMs_t lastHeartbeatMs;
    timeMs_t lastSetpointMs;

    bool heartbeatSeen;
    bool setpointValid;
} mavlinkState_t;

static mavlinkState_t mav;



static uint16_t mavlinkChannelData[MAVLINK_CHANNEL_COUNT];
static bool frameReceived;

static serialPort_t *serialPort = NULL;

static mavlink_message_t mavRecvMsg;
static mavlink_status_t mavRecvStatus;


static volatile uint8_t txbuff_free = 100;
static volatile bool txbuff_valid = false;



void mavlinkRxHandleMessage(const mavlink_rc_channels_override_t *msg)
{
    const uint16_t *channelsPtr = (uint16_t*)&msg->chan1_raw;
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; i++) {
        if (channelsPtr[i] != 0 && channelsPtr[i] != UINT16_MAX) {
            mavlinkChannelData[i] = channelsPtr[i];
        }
    }
    frameReceived = true;
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

static void send_mavlink_ack(uint8_t sysid, uint8_t compid)
{
    mavlink_message_t ack;

    mavlink_msg_command_ack_pack(
        MAV_SYS_ID,
        MAV_COMP_ID,
        &ack,
        mavRecvMsg.msgid,
        MAV_RESULT_ACCEPTED,
        0, 0,
        sysid,
        compid
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &ack);
    serialWriteBuf(serialPort, buf, len);
}


static void handleIncoming_HEARTBEAT(void)
{
    mav.lastHeartbeatMs = millis();
    mav.heartbeatSeen = true;

    send_mavlink_ack(mavRecvMsg.sysid, mavRecvMsg.compid);
}


static void handleIncoming_RC_CHANNELS_OVERRIDE(void)
{
    mavlink_rc_channels_override_t msg;
    mavlink_msg_rc_channels_override_decode(&mavRecvMsg, &msg);
    mavlinkRxHandleMessage(&msg);
}

static void handleIncoming_RADIO_STATUS(void)
{
    mavlink_radio_status_t msg;
    mavlink_msg_radio_status_decode(&mavRecvMsg, &msg);
    txbuff_valid = true;
    txbuff_free = msg.txbuf;
}


static void handleIncoming_SET_ATTITUDE_TARGET(void)
{
    mavlink_set_attitude_target_t msg;
    mavlink_msg_set_attitude_target_decode(&mavRecvMsg, &msg);

    mav.thrust = constrainf(msg.thrust, 0.0f, 1.0f);
    mav.yawRate = msg.body_yaw_rate;
    mav.lastSetpointMs = millis();
    mav.setpointValid = true;

    DEBUG_SET(DEBUG_MAVLINK, 0, (int)(mav.thrust * 1000));

    send_mavlink_ack(mavRecvMsg.sysid, mavRecvMsg.compid);
}

STATIC_UNIT_TESTED void mavlinkDataReceive(uint16_t c, void *data)
{
    
    rxRuntimeState_t *const rxRuntimeState = (rxRuntimeState_t *const)data;

    if (mavlink_parse_char(0, c, &mavRecvMsg, &mavRecvStatus) == MAVLINK_FRAMING_OK) {
        switch (mavRecvMsg.msgid) {

        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            handleIncoming_RC_CHANNELS_OVERRIDE();
            if (rxRuntimeState)
                rxRuntimeState->lastRcFrameTimeUs = micros();
            break;
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleIncoming_HEARTBEAT();
            break;
        case MAVLINK_MSG_ID_RADIO_STATUS:
            handleIncoming_RADIO_STATUS();
            break;

        case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:  
            handleIncoming_SET_ATTITUDE_TARGET();
            break;
        default:
            break;
        }
    }
}
void updateRcCommandsFromMavlink(){
    return;
}



#ifdef USE_SERIAL_MAVLINK

bool mavlinkRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    frameReceived = false;
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; ++i) {
        mavlinkChannelData[i] = rxConfig->midrc;
    }

    DEBUG_SET(DEBUG_MAVLINK, 1, 2);
    rxRuntimeState->channelData = mavlinkChannelData;
    rxRuntimeState->channelCount = MAVLINK_CHANNEL_COUNT;
    rxRuntimeState->rcReadRawFn = mavlinkReadRawRC;
    rxRuntimeState->rcFrameStatusFn = mavlinkFrameStatus;

    const serialPortConfig_t *portConfig =
        findSerialPortConfig(FUNCTION_RX_SERIAL);

    if (!portConfig) {
        DEBUG_SET(DEBUG_MAVLINK, 1, -1);
        return false;
    }

    serialPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_RX_SERIAL,
        mavlinkDataReceive,
        rxRuntimeState,
        baudRates[MAVLINK_BAUD_RATE_INDEX],
        MODE_RXTX,
        SERIAL_NOT_INVERTED
    );

    #ifdef USE_TELEMETRY_MAVLINK
        telemetrySharedPort = serialPort;
        DEBUG_SET(DEBUG_MAVLINK, 1, -3);
    #endif

    DEBUG_SET(DEBUG_MAVLINK, 1, 3);
    return serialPort != NULL;
}

#endif 

void taskProcessMavlink(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!serialPort) {
        const serialPortConfig_t *portConfig =
            findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);

        if (!portConfig) {
            return;
        }

        serialPort = openSerialPort(
            portConfig->identifier,
            FUNCTION_TELEMETRY_MAVLINK,
            NULL,
            NULL,
            baudRates[MAVLINK_BAUD_RATE_INDEX],
            MODE_RXTX,
            SERIAL_NOT_INVERTED
        );

        if (!serialPort) {
            return;
        }
    }

    while (serialRxBytesWaiting(serialPort) > 0) {
        uint8_t c = serialRead(serialPort);
        mavlink_parse_char(
            MAVLINK_COMM_0,
            c,
            &mavRecvMsg,
            &mavRecvStatus
        );
    }
}


#ifdef USE_TELEMETRY_MAVLINK
bool isValidMavlinkTxBuffer(void)
{
    return txbuff_valid;
}

bool shouldSendMavlinkTelemetry(void)
{
    return false;   
}
#endif

#endif 
