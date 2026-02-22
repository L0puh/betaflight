
#pragma once

#include <stdbool.h>

void processCustomMode(void);
bool is_rx(void);
bool is_mavlink(void);
void force_sync(bool active);