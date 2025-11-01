#include "kinematics.h"
#include "motion.h"
#include "stepper.h"
#include "common.h"
#include <stdlib.h>
#include <math.h>

// Physics & profiling calculations - no static data (single instance pattern)

void KINEMATICS_Initialize(WorkCoordinateSystem* wcs) {
    // Initialize work coordinate system to default (G54) 
    wcs->offset.x = 0.0f;
    wcs->offset.y = 0.0f;
    wcs->offset.z = 0.0f;
}

void KINEMATICS_SetWorkOffset(WorkCoordinateSystem* wcs, float x_offset, float y_offset, float z_offset) {
    wcs->offset.x = x_offset;
    wcs->offset.y = y_offset;
    wcs->offset.z = z_offset;
}

// Physics calculations - coordinate conversions
CoordinatePoint KINEMATICS_WorkToMachine(CoordinatePoint work_pos, WorkCoordinateSystem* wcs) {
    CoordinatePoint machine_pos;
    
    // Machine position = Work position + Work coordinate offset
    machine_pos.x = work_pos.x + wcs->offset.x;
    machine_pos.y = work_pos.y + wcs->offset.y;
    machine_pos.z = work_pos.z + wcs->offset.z;
    machine_pos.a = work_pos.a;  // A axis typically not affected by work coordinates
    
    return machine_pos;
}

CoordinatePoint KINEMATICS_MachineToWork(CoordinatePoint machine_pos, WorkCoordinateSystem* wcs) {
    CoordinatePoint work_pos;
    
    // Work position = Machine position - Work coordinate offset
    work_pos.x = machine_pos.x - wcs->offset.x;
    work_pos.y = machine_pos.y - wcs->offset.y;
    work_pos.z = machine_pos.z - wcs->offset.z;
    work_pos.a = machine_pos.a;  // A axis typically not affected by work coordinates
    
    return work_pos;
}

// Physics & profiling calculations for linear moves
MotionSegment* KINEMATICS_LinearMove(CoordinatePoint start, CoordinatePoint end, float feedrate, 
                                   WorkCoordinateSystem* wcs, MotionSegment* segment_buffer) {
    // Convert to machine coordinates
    CoordinatePoint machine_start = KINEMATICS_WorkToMachine(start, wcs);
    CoordinatePoint machine_end = KINEMATICS_WorkToMachine(end, wcs);
    
    // Calculate Bresenham parameters
    segment_buffer->delta_x = fabsf(machine_end.x - machine_start.x);
    segment_buffer->delta_y = fabsf(machine_end.y - machine_start.y);  
    segment_buffer->delta_z = fabsf(machine_end.z - machine_start.z);
    segment_buffer->delta_a = fabsf(machine_end.a - machine_start.a);
    
    // Determine dominant axis (highest step count)
    int32_t max_delta = segment_buffer->delta_x;
    segment_buffer->dominant_axis = AXIS_X;
    
    if(segment_buffer->delta_y > max_delta) {
        max_delta = segment_buffer->delta_y;
        segment_buffer->dominant_axis = AXIS_Y;
    }
    if(segment_buffer->delta_z > max_delta) {
        max_delta = segment_buffer->delta_z;
        segment_buffer->dominant_axis = AXIS_Z;
    }
    if(segment_buffer->delta_a > max_delta) {
        max_delta = segment_buffer->delta_a;
        segment_buffer->dominant_axis = AXIS_A;
    }
    
    // Physics calculations - velocity profiling
    segment_buffer->steps_remaining = max_delta;
    segment_buffer->step_interval = 1000;  // TODO: Calculate from feedrate
    segment_buffer->pulse_width = 2;       // 20µs pulse width
    
    // Initialize Bresenham error terms
    segment_buffer->error_y = max_delta / 2;
    segment_buffer->error_z = max_delta / 2;  
    segment_buffer->error_a = max_delta / 2;
    
    // TODO: Implement physics calculations for acceleration/deceleration profiles
    segment_buffer->start_velocity = 0.0f;     // Start from stop
    segment_buffer->max_velocity = feedrate;   // Target feedrate
    segment_buffer->end_velocity = 0.0f;       // End at stop
    segment_buffer->acceleration = 100.0f;     // TODO: Configure acceleration
    
    return segment_buffer;
}

// Physics calculations for arc interpolation  
MotionSegment* KINEMATICS_ArcMove(CoordinatePoint start, CoordinatePoint end, CoordinatePoint center, 
                                 bool clockwise, WorkCoordinateSystem* wcs, MotionSegment* segment_buffer) {
    // TODO: Implement arc interpolation physics
    // Break arcs into linear segments for Bresenham execution
    // Calculate velocity profiles for smooth arc motion
    return NULL;  // Placeholder
}