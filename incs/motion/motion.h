#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "data_structures.h"
#include "gcode_parser.h"  // ✅ For GCODE_Event type

// Existing declarations...
void MOTION_Initialize(void);
void MOTION_Tasks(APP_DATA* appData);

// ✅ Arc interpolation function
void MOTION_Arc(APP_DATA* appData);

// Add this function declaration after MOTION_Arc()

// Process a single gcode event and convert to motion segments
bool MOTION_ProcessGcodeEvent(APP_DATA* appData, GCODE_Event* event);

// Submit a constant-speed homing move to the trajectory queue.
// Handles queue-full check, Recalculate, and interpolator kick-start.
// Returns true if the move was queued, false if queue was full.
bool MOTION_HomingMove(APP_DATA* appData, CoordinatePoint start,
                       CoordinatePoint end, float feedrate_mm_min);

// Re-anchor the planned-position tracker to the live step-counter.
// Call after soft-reset, alarm clear, or any event that discards the trajectory
// queue so the next queued move starts from the correct machine position.
void MOTION_SyncPlannedPosition(void);

// Returns true while a G4 dwell is in progress (phase 1 drain OR phase 2 timer).
// Used by flow control to suppress deferred-ok release during the dwell so UGS
// does not think the file is complete while commands are still queued.
bool MOTION_IsDwellActive(void);

// Returns true when both the trajectory queue and the interpolator are idle
// (no motion executing and none queued).  Used by the probe monitor in app.c
// to detect that the probe reached its target without triggering.
bool MOTION_IsIdle(void);

// Stop motion immediately on a probe trigger.
// Stops the interpolator, clears the trajectory queue, and re-anchors the
// planned-position tracker to the live step-counter.
// Does NOT disable steppers so the machine holds position at contact point.
void MOTION_ProbeStop(void);

// Cancel any in-progress jog moves (real-time 0x85 handler).
// Flushes jog-flagged moves from the trajectory queue (preserves G-code moves),
// stops the interpolator if it was executing a jog, and re-anchors planned position.
// Does NOT raise an alarm — distinct from feed hold and soft reset.
void MOTION_JogCancel(void);

// Returns true while a jog move is executing or queued in the trajectory.
// Used by the '?' status handler to report "Jog" state instead of "Run".
bool MOTION_IsJogging(void);

#endif /* MOTION_H */