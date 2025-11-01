#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "motion.h"

typedef struct {
    float x, y, z, a;
} CoordinatePoint;

typedef struct {
    CoordinatePoint offset;  // G54/G55/etc work coordinates
} WorkCoordinateSystem;

// Initialize work coordinates (single instance pattern)
void KINEMATICS_Initialize(void);  // Fixed: Added void parameter
void KINEMATICS_SetWorkOffset(float x_offset, float y_offset, float z_offset);  // Fixed: Removed wcs parameter

// Coordinate conversion functions (physics calculations)
CoordinatePoint KINEMATICS_WorkToMachine(CoordinatePoint work_pos);  // Fixed: Removed wcs parameter
CoordinatePoint KINEMATICS_MachineToWork(CoordinatePoint machine_pos);  // Fixed: Removed wcs parameter
WorkCoordinateSystem* KINEMATICS_GetWorkCoordinates(void);  // Fixed: Added void parameter
void KINEMATICS_SetWorkCoordinates(float x, float y, float z);  // Fixed: Removed wcs parameter

// Physics & profiling calculations for motion planning
MotionSegment* KINEMATICS_LinearMove(CoordinatePoint start, CoordinatePoint end, float feedrate, 
                                   MotionSegment* segment_buffer);  // Fixed: Removed wcs parameter
MotionSegment* KINEMATICS_ArcMove(CoordinatePoint start, CoordinatePoint end, CoordinatePoint center, 
                                 bool clockwise, MotionSegment* segment_buffer);  // Fixed: Removed wcs parameter

#endif /* KINEMATICS_H */