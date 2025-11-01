#include "kinematics.h"
#include "motion.h"
#include "stepper.h"
#include "common.h"
#include <stdlib.h>
#include <math.h>

// Single instance of work coordinates managed by kinematics (physics module)
static WorkCoordinateSystem work_coordinates;

void KINEMATICS_Initialize(void) {
    // Initialize work coordinate system to default (G54)
    work_coordinates.offset.x = 0.0f;
    work_coordinates.offset.y = 0.0f;
    work_coordinates.offset.z = 0.0f;
}

WorkCoordinateSystem* KINEMATICS_GetWorkCoordinates(void) {
    return &work_coordinates;  // Return reference to internal single instance
}

void KINEMATICS_SetWorkOffset(float x_offset, float y_offset, float z_offset) {
    work_coordinates.offset.x = x_offset;
    work_coordinates.offset.y = y_offset;
    work_coordinates.offset.z = z_offset;
}

void KINEMATICS_SetWorkCoordinates(float x, float y, float z) {
    work_coordinates.offset.x = x;
    work_coordinates.offset.y = y;
    work_coordinates.offset.z = z;
}

// Physics calculations using internal work coordinates
CoordinatePoint KINEMATICS_WorkToMachine(CoordinatePoint work_pos) {
    CoordinatePoint machine_pos;
    
    // Machine position = Work position + Work coordinate offset
    machine_pos.x = work_pos.x + work_coordinates.offset.x;
    machine_pos.y = work_pos.y + work_coordinates.offset.y;
    machine_pos.z = work_pos.z + work_coordinates.offset.z;
    machine_pos.a = work_pos.a;  // A axis typically not affected by work coordinates
    
    return machine_pos;
}

CoordinatePoint KINEMATICS_MachineToWork(CoordinatePoint machine_pos) {
    CoordinatePoint work_pos;
    
    // Work position = Machine position - Work coordinate offset
    work_pos.x = machine_pos.x - work_coordinates.offset.x;
    work_pos.y = machine_pos.y - work_coordinates.offset.y;
    work_pos.z = machine_pos.z - work_coordinates.offset.z;
    work_pos.a = machine_pos.a;  // A axis typically not affected by work coordinates
    
    return work_pos;
}

// Physics & profiling calculations for linear moves
MotionSegment* KINEMATICS_LinearMove(CoordinatePoint start, CoordinatePoint end, float feedrate, 
                                   MotionSegment* segment_buffer) {
    // Convert to machine coordinates using internal work coordinates
    CoordinatePoint machine_start = KINEMATICS_WorkToMachine(start);
    CoordinatePoint machine_end = KINEMATICS_WorkToMachine(end);
    
    // Calculate Bresenham parameters (physics calculations)
    segment_buffer->delta_x = (int32_t)fabsf(machine_end.x - machine_start.x);
    segment_buffer->delta_y = (int32_t)fabsf(machine_end.y - machine_start.y);  
    segment_buffer->delta_z = (int32_t)fabsf(machine_end.z - machine_start.z);
    segment_buffer->delta_a = (int32_t)fabsf(machine_end.a - machine_start.a);
    
    // Determine dominant axis (highest step count) - dynamic dominant axis tracking
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
    segment_buffer->steps_completed = 0;
    segment_buffer->step_interval = 1000;  // TODO: Calculate from feedrate (10ms default)
    segment_buffer->pulse_width = 2;       // 20µs pulse width (2 * 10µs ticks)
    
    // Initialize Bresenham error terms
    segment_buffer->error_y = max_delta / 2;
    segment_buffer->error_z = max_delta / 2;  
    segment_buffer->error_a = max_delta / 2;
    
    return segment_buffer;
}

// Placeholder for arc interpolation (future implementation)
MotionSegment* KINEMATICS_ArcMove(CoordinatePoint start, CoordinatePoint end, CoordinatePoint center, 
                                 bool clockwise, MotionSegment* segment_buffer) {
    // TODO: Implement arc interpolation using absolute compare mode
    // Will break arcs into linear segments for Bresenham execution
    return NULL;  // Placeholder
}