// ============================================================================
// segment_buffer.h - GRBL-style segment buffer system
// ============================================================================

#ifndef SEGMENT_BUFFER_H
#define SEGMENT_BUFFER_H

#include "data_structures.h"

// ============================================================================
// Public API
// ============================================================================

// Initialize segment buffer system
void SEGMENT_Initialize(APP_DATA* appData);

// Prepare next segment from planner blocks
// Call continuously from main loop to keep buffer filled
void SEGMENT_PrepBuffer(APP_DATA* appData);

#endif // SEGMENT_BUFFER_H
