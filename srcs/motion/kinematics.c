#include "kinematics.h"
#include "motion.h"
#include "stepper.h"
#include "settings.h"  // For GRBL_Settings access
#include "common.h"
#include <stdlib.h>
#include <math.h>

// Timer configuration for step interval calculations
#define TIMER_TICKS_PER_SECOND 12500000UL  // 12.5MHz timer frequency
#define TIMER_TICKS_PER_MICROSECOND 12.5f  // 12.5 ticks per microsecond

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
    // Get settings and stepper position (reuse existing modules - no duplication)
    GRBL_Settings* settings = SETTINGS_GetCurrent();
    StepperPosition* stepper = STEPPER_GetPosition();
    
    // Convert to machine coordinates using internal work coordinates
    CoordinatePoint machine_start = KINEMATICS_WorkToMachine(start);
    CoordinatePoint machine_end = KINEMATICS_WorkToMachine(end);
    
    // Calculate distance in mm using FPU (PRESERVE SIGN for direction!)
    float dx_mm = machine_end.x - machine_start.x;  // ✅ Signed - positive=forward, negative=reverse
    float dy_mm = machine_end.y - machine_start.y;
    float dz_mm = machine_end.z - machine_start.z;
    float da_mm = machine_end.a - machine_start.a;
    
    // Convert mm to steps using existing steps_per_mm from settings (PRESERVE SIGN!)
    segment_buffer->delta_x = (int32_t)(dx_mm * stepper->steps_per_mm_x);
    segment_buffer->delta_y = (int32_t)(dy_mm * stepper->steps_per_mm_y);
    segment_buffer->delta_z = (int32_t)(dz_mm * stepper->steps_per_mm_z);
    segment_buffer->delta_a = (int32_t)(da_mm * stepper->steps_per_deg_a);
    
    // Determine dominant axis (highest ABSOLUTE step count) - dynamic dominant axis tracking
    int32_t max_delta = abs(segment_buffer->delta_x);  // ✅ Use abs() for comparison
    segment_buffer->dominant_axis = AXIS_X;
    E_AXIS limiting_axis = AXIS_X;
    
    if(abs(segment_buffer->delta_y) > max_delta) {
        max_delta = abs(segment_buffer->delta_y);
        segment_buffer->dominant_axis = AXIS_Y;
        limiting_axis = AXIS_Y;
    }
    if(abs(segment_buffer->delta_z) > max_delta) {
        max_delta = abs(segment_buffer->delta_z);
        segment_buffer->dominant_axis = AXIS_Z;
        limiting_axis = AXIS_Z;
    }
    if(abs(segment_buffer->delta_a) > max_delta) {
        max_delta = abs(segment_buffer->delta_a);
        segment_buffer->dominant_axis = AXIS_A;
        limiting_axis = AXIS_A;
    }
    
    // Convert feedrate from mm/min to mm/sec
    float feedrate_mm_sec = feedrate / 60.0f;
    
    // Get max_rate and acceleration for limiting axis (reuse existing settings)
    float max_rate_mm_min = 0.0f;
    float acceleration_mm_sec2 = 0.0f;
    
    switch(limiting_axis) {
        case AXIS_X:
            max_rate_mm_min = settings->max_rate_x;
            acceleration_mm_sec2 = settings->acceleration_x;
            break;
        case AXIS_Y:
            max_rate_mm_min = settings->max_rate_y;
            acceleration_mm_sec2 = settings->acceleration_y;
            break;
        case AXIS_Z:
            max_rate_mm_min = settings->max_rate_z;
            acceleration_mm_sec2 = settings->acceleration_z;
            break;
        case AXIS_A:
            max_rate_mm_min = settings->max_rate_a;
            acceleration_mm_sec2 = settings->acceleration_a;
            break;
        default:
            max_rate_mm_min = settings->max_rate_x;
            acceleration_mm_sec2 = settings->acceleration_x;
            break;
    }
    
    // Clamp feedrate to max rate
    float max_rate_mm_sec = max_rate_mm_min / 60.0f;
    if(feedrate_mm_sec > max_rate_mm_sec) {
        feedrate_mm_sec = max_rate_mm_sec;
    }
    
    // Timer frequency (reuse constant - no duplication)
    const float TIMER_FREQ = 12500000.0f;  // 12.5 MHz from copilot-instructions.md
    
    // Get steps_per_mm for dominant axis (reuse existing variable)
    float steps_per_mm_dominant = stepper->steps_per_mm_x;
    switch(segment_buffer->dominant_axis) {
        case AXIS_X: steps_per_mm_dominant = stepper->steps_per_mm_x; break;
        case AXIS_Y: steps_per_mm_dominant = stepper->steps_per_mm_y; break;
        case AXIS_Z: steps_per_mm_dominant = stepper->steps_per_mm_z; break;
        case AXIS_A: steps_per_mm_dominant = stepper->steps_per_deg_a; break;
        default: break;
    }
    
    // Calculate nominal step interval (cruise speed)
    // steps_per_sec = feedrate_mm_sec * steps_per_mm
    float steps_per_sec = feedrate_mm_sec * steps_per_mm_dominant;
    segment_buffer->nominal_rate = (uint32_t)(TIMER_FREQ / steps_per_sec);
    
    // Calculate acceleration profile (GRBL-style trapezoidal)
    // Distance to accelerate: d = v² / (2*a)
    float accel_distance_mm = (feedrate_mm_sec * feedrate_mm_sec) / (2.0f * acceleration_mm_sec2);
    uint32_t accel_steps = (uint32_t)(accel_distance_mm * steps_per_mm_dominant);
    
    // Check if we have room for full accel + decel
    if(accel_steps * 2 > (uint32_t)max_delta) {
        // Triangle profile - can't reach full speed
        accel_steps = max_delta / 2;
        segment_buffer->accelerate_until = accel_steps;
        segment_buffer->decelerate_after = accel_steps;
    } else {
        // Trapezoid profile - accel, cruise, decel
        segment_buffer->accelerate_until = accel_steps;
        segment_buffer->decelerate_after = max_delta - accel_steps;
    }
    
    // Initial rate (minimum speed to avoid stalling)
    float min_steps_per_sec = 500.0f;  // Configurable minimum
    segment_buffer->initial_rate = (uint32_t)(TIMER_FREQ / min_steps_per_sec);
    segment_buffer->final_rate = segment_buffer->initial_rate;
    
    // Rate delta per step (for integer ISR math)
    if(accel_steps > 0) {
        segment_buffer->rate_delta = (int32_t)((segment_buffer->initial_rate - segment_buffer->nominal_rate) / accel_steps);
    } else {
        segment_buffer->rate_delta = 0;
    }
    
    // Physics parameters (for debugging/reference)
    segment_buffer->start_velocity = min_steps_per_sec / steps_per_mm_dominant;
    segment_buffer->max_velocity = feedrate_mm_sec;
    segment_buffer->end_velocity = min_steps_per_sec / steps_per_mm_dominant;
    segment_buffer->acceleration = acceleration_mm_sec2;
    
    // Motion state initialization
    segment_buffer->steps_remaining = max_delta;
    segment_buffer->steps_completed = 0;
    segment_buffer->step_interval = segment_buffer->initial_rate;
    
    // Pulse width from settings (reuse existing calculation from stepper.c)
    segment_buffer->pulse_width = (uint32_t)(settings->step_pulse_time * 12.5f);  // µs to timer ticks
    
    // Initialize Bresenham error terms (symmetric rounding)
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