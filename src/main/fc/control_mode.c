#include "control_mode.h"

static control_mode_e controlMode = CONTROL_MODE_RC;

control_mode_e getControlMode(){
    return controlMode;
}
void setControlMode(control_mode_e mode){
    controlMode = mode;
    return;

}
bool isMode(control_mode_e mode){
    return controlMode == mode;
}