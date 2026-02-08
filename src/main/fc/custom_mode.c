#include "custom_mode.h"
#include "build/debug.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

static bool lastState = false;
static bool RX_STATUS_RUNNING = true;
static bool MAVLINK_STATUS_RUNNING = false;

bool is_mavlink(void) {
    return MAVLINK_STATUS_RUNNING;
}

bool is_rx(void){
    return RX_STATUS_RUNNING;
}

void processCustomMode(void)
{
    bool active = IS_RC_MODE_ACTIVE(BOXCUSTOM);

    if (active && !lastState) {
       
        ENABLE_FLIGHT_MODE(CUSTOM_MODE);

        RX_STATUS_RUNNING = false;
        MAVLINK_STATUS_RUNNING = true;

        DEBUG_SET(DEBUG_MAVLINK, 5, -4);
    }
    else if (!active && lastState) {
        
        DISABLE_FLIGHT_MODE(CUSTOM_MODE);

        RX_STATUS_RUNNING = true;
        MAVLINK_STATUS_RUNNING = false;

        DEBUG_SET(DEBUG_MAVLINK, 5, 4);
    }

    lastState = active;
}