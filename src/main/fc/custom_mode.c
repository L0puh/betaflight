#include "custom_mode.h"
#include "build/debug.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "rc_modes.h"
#include "runtime_config.h"
#include "rx/mavlink.h"
#include "rx/rx.h"

static bool lastState = false;


bool is_mavlink(void) {
    return FLIGHT_MODE(CUSTOM_MODE);
}

bool is_rx(void){
    return !FLIGHT_MODE(CUSTOM_MODE);
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

        } else {
            DISABLE_FLIGHT_MODE(CUSTOM_MODE);

        }
    }
    lastState = active;
    DEBUG_SET(DEBUG_MAVLINK, 6, active ? 1 : 0);
    DEBUG_SET(DEBUG_MAVLINK, 7, FLIGHT_MODE(CUSTOM_MODE) ? 1 : 0);

}