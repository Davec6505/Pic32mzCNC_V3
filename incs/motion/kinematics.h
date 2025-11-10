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

// Physics & profiling calculations for motion planning
MotionSegment* KINEMATICS_LinearMove(CoordinatePoint start, CoordinatePoint end, float feedrate, 
                                   MotionSegment* segment_buffer,
                                   float entry_velocity, float exit_velocity);  // Added junction velocities

// Backward compatibility wrapper (for single segments without junction planning)
MotionSegment* KINEMATICS_LinearMoveSimple(CoordinatePoint start, CoordinatePoint end, float feedrate, 
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

#endif /* KINEMATICS_H */