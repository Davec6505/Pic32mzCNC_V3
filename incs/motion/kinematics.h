#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "motion.h"
#include "data_structures.h"  // For CoordinatePoint definition

typedef struct {
    CoordinatePoint offset;  // G54/G55/etc work coordinates
} WorkCoordinateSystem;

// Initialize work coordinates (single instance pattern)
void KINEMATICS_Initialize(void);  // Fixed: Added void parameter
void KINEMATICS_SetWorkOffset(float x_offset, float y_offset, float z_offset);  // Fixed: Removed wcs parameter

// Coordinate conversion functions (physics calculations)
CoordinatePoint KINEMATICS_WorkToMachine(CoordinatePoint work_pos);  // Fixed: Removed wcs parameter
CoordinatePoint KINEMATICS_MachineToWork(CoordinatePoint machine_pos);  // Fixed: Removed wcs parameter

// Enhanced coordinate conversion with active WCS support
CoordinatePoint KINEMATICS_WorkToMachineWithWCS(CoordinatePoint work_pos, uint8_t activeWCS);
CoordinatePoint KINEMATICS_MachineToWorkWithWCS(CoordinatePoint machine_pos, uint8_t activeWCS);
WorkCoordinateSystem* KINEMATICS_GetWorkCoordinates(void);  // Fixed: Added void parameter
void KINEMATICS_SetWorkCoordinates(float x, float y, float z);  // Fixed: Removed wcs parameter

// Get current active work coordinate system offset (uses appData->activeWCS)
void KINEMATICS_GetActiveWCSOffset(uint8_t activeWCS, float* x_offset, float* y_offset, float* z_offset);

// G92 temporary work offset (not flash-backed; cleared on soft reset)
// machine[NUM_AXIS] = current machine coords, desired_work[NUM_AXIS] = requested work position.
void KINEMATICS_SetG92Offset(const float machine[NUM_AXIS], const float desired_work[NUM_AXIS]);
void KINEMATICS_ClearG92Offset(void);
void KINEMATICS_GetG92Offset(float* x, float* y, float* z);  // live g92_offset[] — not flash-backed

// Active Work Coordinate System management (G54–G59).
// KINEMATICS_SetActiveWCS(n) reloads g_wcs.offset from flash slot n (0=G54…5=G59)
// so that subsequent WorkToMachine / MachineToWork calls use the new offset.
// Replaces the cached G54 that was loaded at KINEMATICS_Initialize().
void KINEMATICS_SetActiveWCS(uint8_t wcs_number);
uint8_t KINEMATICS_GetActiveWCS(void);

// Tool Length Offset (G43 / G43.1 / G49)
// Applied to Z-axis only (GRBL standard). Not flash-backed — cleared on soft reset.
// KINEMATICS_SetTLO activates and stores the offset value (replaces any prior value).
// KINEMATICS_ClearTLO deactivates TLO (G49).
// KINEMATICS_GetTLO returns the current live value and sets *active to its state.
void  KINEMATICS_SetTLO(float offset_mm);
void  KINEMATICS_ClearTLO(void);
float KINEMATICS_GetTLO(bool *active_out);  // returns value; sets *active_out

// Physics & profiling calculations for motion planning
MotionSegment* KINEMATICS_LinearMove(CoordinatePoint start, CoordinatePoint end, float feedrate, 
                                   MotionSegment* segment_buffer,
                                   float entry_velocity, float exit_velocity);  // Added junction velocities

// Backward compatibility wrapper (for single segments without junction planning)
MotionSegment* KINEMATICS_LinearMoveSimple(CoordinatePoint start, CoordinatePoint end, float feedrate, 
                                          MotionSegment* segment_buffer);

// Pure GRBL constant-speed segment for homing (no Taylor profiler, no acceleration ramps)
MotionSegment* KINEMATICS_HomingMove(CoordinatePoint start, CoordinatePoint end, float feedrate_mm_min,
                                     MotionSegment* segment_buffer);
MotionSegment* KINEMATICS_ArcMove(CoordinatePoint start, CoordinatePoint end, CoordinatePoint center, 
                                 bool clockwise, float feedrate, MotionSegment* segment_buffer);

// High-level arc planning function - calculates arc parameters for incremental generation
bool KINEMATICS_PlanArc(CoordinatePoint start, CoordinatePoint end, CoordinatePoint center,
                       bool clockwise, float feedrate, float* out_radius, float* out_total_angle,
                       uint32_t* out_num_segments);

// Reset step accumulators (call when starting new arc or after G92)
void KINEMATICS_ResetAccumulators(void);

// Get current position from stepper counts
CoordinatePoint KINEMATICS_GetCurrentPosition(void);

// Set machine position for a single axis (used during homing)
void KINEMATICS_SetAxisMachinePosition(E_AXIS axis, float position);

// Junction deviation calculation for smooth cornering
float KINEMATICS_CalculateJunctionSpeed(CoordinatePoint prev_dir, CoordinatePoint curr_dir,
                                       float junction_deviation, float acceleration);

// GRBL-exact planner helpers
// Re-computes ISR timing fields (initial_rate, nominal_rate, final_rate,
// accelerate_until, decelerate_after) for a segment given its planned entry/exit
// speeds in mm/s.  Called by MOTION_PlannerRecalculate() after the reverse+forward pass.
void KINEMATICS_RecalculateTrapezoid(MotionSegment *seg, float entry_mms, float exit_mms);

// Reset the planner's previous-segment state (call at init and after soft reset).
void KINEMATICS_ResetPlannerState(void);

#endif /* KINEMATICS_H */