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
void KINEMATICS_Initialize(WorkCoordinateSystem* wcs);
void KINEMATICS_SetWorkOffset(WorkCoordinateSystem* wcs, float x_offset, float y_offset, float z_offset);

// Coordinate conversion functions (physics calculations)
CoordinatePoint KINEMATICS_WorkToMachine(CoordinatePoint work_pos, WorkCoordinateSystem* wcs);
CoordinatePoint KINEMATICS_MachineToWork(CoordinatePoint machine_pos, WorkCoordinateSystem* wcs);
WorkCoordinateSystem* KINEMATICS_GetWorkCoordinates();

// Physics & profiling calculations for motion planning
MotionSegment* KINEMATICS_LinearMove(CoordinatePoint start, CoordinatePoint end, float feedrate, 
                                   WorkCoordinateSystem* wcs, MotionSegment* segment_buffer);
MotionSegment* KINEMATICS_ArcMove(CoordinatePoint start, CoordinatePoint end, CoordinatePoint center, 
                                 bool clockwise, WorkCoordinateSystem* wcs, MotionSegment* segment_buffer);



#endif /* KINEMATICS_H */