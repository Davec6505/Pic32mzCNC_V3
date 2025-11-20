#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "data_structures.h"
#include "gcode_parser.h"  // ✅ For GCODE_Event type

// Existing declarations...
void MOTION_Initialize(APP_DATA* appData);
void MOTION_Tasks(APP_DATA* appData);

// ✅ Arc interpolation function
void MOTION_Arc(APP_DATA* appData);

// Add this function declaration after MOTION_Arc()

// Process a single gcode event and convert to motion segments
bool MOTION_ProcessGcodeEvent(APP_DATA* appData, GCODE_Event* event);


#endif /* MOTION_H */