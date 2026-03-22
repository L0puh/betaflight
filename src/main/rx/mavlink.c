#include "telemetry/mavlink.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#include "fc/custom_mode.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
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

#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1
static mavlink_message_t mavMsg;
static uint32_t last_sent_ack = -1;
static uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];

mavlink_msg_cfg mavlink_msg_configs[] = {
    {ATTITUDE, "ATTITUDE", 10},
};
bool mavlinkSetMsgFreq(mavlink_msg_type type, int freq) {
    if (freq < 0) {
        return 0;
    }
    mavlink_msg_configs[type].freq = freq;
    return 1;
}

void mavlinkRxHandleMessage(const mavlink_rc_channels_override_t *msg)
{
    const uint16_t *channelsPtr = (uint16_t*)&msg->chan1_raw;
    DEBUG_SET(DEBUG_MAVLINK, 0, -10);
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; i++) {
        if (channelsPtr[i] != 0 && channelsPtr[i] != UINT16_MAX) {
            
            DEBUG_SET(DEBUG_MAVLINK, 0, i);
         
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

    last_sent_ack = mavRecvMsg.msgid;
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


    send_mavlink_ack(mavRecvMsg.sysid, mavRecvMsg.compid);
}

STATIC_UNIT_TESTED void mavlinkDataReceive(uint16_t c, void *data)
{
    
    rxRuntimeState_t *const rxRuntimeState = (rxRuntimeState_t *const)data;
    DEBUG_SET(DEBUG_MAVLINK, 1, -1);
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


timeMs_t mavlinkGetLastHeartbeatMs(void){
    return mav.lastHeartbeatMs;
}

uint32_t mavlinkGetLastSentAckId(void){
    return last_sent_ack;
}

bool mavlinkRxClose(rxRuntimeState_t *rxRuntimeState)
{
    if (!serialPort) return 0;

    *rxRuntimeState = old_rxRuntimeState;
    closeSerialPort(serialPort);
    
    return 1;
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

    const serialPortConfig_t *portConfig =
        findSerialPortConfig(FUNCTION_RX_SERIAL);

    if (!portConfig) {
        DEBUG_SET(DEBUG_MAVLINK, 2, -1);
        return false;
    }

    serialPort = openSerialPort(
        SERIAL_PORT_UART4, 
        FUNCTION_RX_SERIAL,
        mavlinkDataReceive,
        rxRuntimeState,
        baudRates[MAVLINK_BAUD_RATE_INDEX],
        MODE_RXTX,
        SERIAL_NOT_INVERTED
    );

    #ifdef USE_TELEMETRY_MAVLINK
        telemetrySharedPort = serialPort;
    #endif
    DEBUG_SET(DEBUG_MAVLINK, 2, 1);
    return serialPort != NULL;
}

static void mavlinkSerialWrite(uint8_t * buf, uint16_t length)
{
    for (int i = 0; i < length; i++)
        serialWrite(serialPort, buf[i]);
}


void mavlinkSendHeartbeat(void){
 
    uint16_t msgLength;
 
    uint8_t mavModes = MAV_MODE_MANUAL_DISARMED;
    if (ARMING_FLAG(ARMED))
        mavModes |= MAV_MODE_MANUAL_ARMED;

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode)
    {
        case MIXER_TRI:
            mavSystemType = MAV_TYPE_TRICOPTER;
            break;
        case MIXER_QUADP:
        case MIXER_QUADX:
        case MIXER_Y4:
        case MIXER_VTAIL4:
            mavSystemType = MAV_TYPE_QUADROTOR;
            break;
        case MIXER_Y6:
        case MIXER_HEX6:
        case MIXER_HEX6X:
            mavSystemType = MAV_TYPE_HEXAROTOR;
            break;
        case MIXER_OCTOX8:
        case MIXER_OCTOX8P:
        case MIXER_OCTOFLATP:
        case MIXER_OCTOFLATX:
            mavSystemType = MAV_TYPE_OCTOROTOR;
            break;
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_CUSTOM_AIRPLANE:
            mavSystemType = MAV_TYPE_FIXED_WING;
            break;
        case MIXER_HELI_120_CCPM:
        case MIXER_HELI_90_DEG:
            mavSystemType = MAV_TYPE_HELICOPTER;
            break;
        default:
            mavSystemType = MAV_TYPE_GENERIC;
            break;
    }

    // Custom mode for compatibility with APM OSDs
    uint8_t mavCustomMode = 1;  // Acro by default

    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | ALT_HOLD_MODE | POS_HOLD_MODE)) {
        mavCustomMode = 0;      //Stabilize
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    uint8_t mavSystemState = 0;
    if (ARMING_FLAG(ARMED)) {
        if (failsafeIsActive()) {
            mavSystemState = MAV_STATE_CRITICAL;
        }
        else {
            mavSystemState = MAV_STATE_ACTIVE;
        }
    }
    else {
        mavSystemState = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        mavSystemType,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        MAV_AUTOPILOT_GENERIC,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Packets transmit counter to debug actual data rate
    static uint32_t transmitCounter = 0;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 6, transmitCounter);
    transmitCounter = (transmitCounter + 1) % 100;
}


void mavlinkCustomRxInit(void){
    serialPort = openSerialPort(
        SERIAL_PORT_UART4,
        FUNCTION_RX_SERIAL,
        NULL,
        &rxRuntimeState,
        baudRates[MAVLINK_BAUD_RATE_INDEX],
        MODE_RXTX,
        SERIAL_NOT_INVERTED
    );
}

void taskMavlinkSendHeartbeats(timeUs_t currentTimeUs){
    if (!serialPort || !is_mavlink()) return;
    mav.lastHeartbeatMs = currentTimeUs;
    mavlinkSendHeartbeat();
}


void taskProcessMavlink(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (!serialPort || !is_mavlink()) return;
    if (serialRxBytesWaiting(serialPort)){
        mavlinkDataReceive(serialRead(serialPort), &rxRuntimeState);
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
