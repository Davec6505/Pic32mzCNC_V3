#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "data_structures.h"

// Existing declarations...
void MOTION_Initialize(void);
void MOTION_Tasks(APP_DATA* appData);

// ✅ Arc interpolation function
void MOTION_Arc(APP_DATA* appData);

#endif /* MOTION_H */