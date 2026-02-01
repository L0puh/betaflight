#pragma once


#include <stdbool.h>
typedef enum control_mode_ {
    CONTROL_MODE_RC = 0,
    CONTROL_MODE_MAVLINK,
} control_mode_e;

control_mode_e getControlMode();
void setControlMode(control_mode_e);
bool isMode(control_mode_e);