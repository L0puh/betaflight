#include "custom_mode.h"
#include "build/debug.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"


void processCustomMode(void)
{

  
  
    if (IS_RC_MODE_ACTIVE(BOXCUSTOM)) {
        DEBUG_SET(DEBUG_MAVLINK, 5, -4);
    } else {
        DEBUG_SET(DEBUG_MAVLINK, 5, 4);
    }
}