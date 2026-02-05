#ifndef STATELOGIC_H
#define STATELOGIC_H

#include <stdint.h>

// Update the flight state based on current sensor readings
// Called periodically from main loop
void updateFlightState(uint32_t now_ms);

#endif // STATELOGIC_H
