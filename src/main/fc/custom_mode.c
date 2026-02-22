#include "custom_mode.h"
#include "build/debug.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "rx/rx.h"

static bool lastState = false;
static bool RX_STATUS_RUNNING = true;
static bool MAVLINK_STATUS_RUNNING = false;

bool is_mavlink(void) {
    return MAVLINK_STATUS_RUNNING;
}

bool is_rx(void){
    return RX_STATUS_RUNNING;
}

void force_sync(bool active){
    if (active != FLIGHT_MODE(CUSTOM_MODE)){
        if (active)
            ENABLE_FLIGHT_MODE(CUSTOM_MODE);
        else 
            DISABLE_FLIGHT_MODE(CUSTOM_MODE);
    }
}

void processCustomMode(void)
{
    bool active = IS_RC_MODE_ACTIVE(BOXCUSTOM);
    
    if (active != lastState) {
        if (active) {
            ENABLE_FLIGHT_MODE(CUSTOM_MODE);

            RX_STATUS_RUNNING = false;
            MAVLINK_STATUS_RUNNING = true;
            old_rxRuntimeState = rxRuntimeState;
        } else {
            DISABLE_FLIGHT_MODE(CUSTOM_MODE);

            RX_STATUS_RUNNING = true;
            MAVLINK_STATUS_RUNNING = false;
            rxRuntimeState = old_rxRuntimeState;
        }
    }
  
    lastState = active;
    DEBUG_SET(DEBUG_MAVLINK, 5, active ? 1 : 0);
    DEBUG_SET(DEBUG_MAVLINK, 6, RX_STATUS_RUNNING ? 1: 0);
    DEBUG_SET(DEBUG_MAVLINK, 7, FLIGHT_MODE(CUSTOM_MODE) ? 1 : 0);

    force_sync(active);
}