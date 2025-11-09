#include "kinematics.h"
#include "motion.h"
#include "stepper.h"
#include "settings.h"  // For CNC_Settings access
#include "common.h"
#include "utils/uart_utils.h"  // For DEBUG_PRINT_MOTION
#include "utils/utils.h"       // For AxisConfig
#include <stdlib.h>
#include <math.h>

// Timer configuration for step interval calculations
// Timer frequency now queried dynamically (TMR4 in 16-bit mode)
#define TIMER_TICKS_PER_SECOND_DYNAMIC() (TMR4_FrequencyGet())

// Single instance of work coordinates managed by kinematics (physics module)
static WorkCoordinateSystem work_coordinates;

// Step accumulators - preserve fractional steps across arc segments (file scope for reset access)
static float step_accumulator_x = 0.0f;
static float step_accumulator_y = 0.0f;
static float step_accumulator_z = 0.0f;
static float step_accumulator_a = 0.0f;

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
    CNC_Settings* settings = SETTINGS_GetCurrent();
    StepperPosition* stepper = STEPPER_GetPosition();
    
    // Convert to machine coordinates using internal work coordinates
    CoordinatePoint machine_start = KINEMATICS_WorkToMachine(start);
    CoordinatePoint machine_end = KINEMATICS_WorkToMachine(end);
    
    // Calculate distance in mm using FPU (PRESERVE SIGN for direction!)
    float dx_mm = machine_end.x - machine_start.x;  // ✅ Signed - positive=forward, negative=reverse
    float dy_mm = machine_end.y - machine_start.y;
    float dz_mm = machine_end.z - machine_start.z;
    float da_mm = machine_end.a - machine_start.a;
    
    // Convert mm to steps WITH ACCUMULATION to preserve fractional steps
    step_accumulator_x += dx_mm * stepper->steps_per_mm_x;
    step_accumulator_y += dy_mm * stepper->steps_per_mm_y;
    step_accumulator_z += dz_mm * stepper->steps_per_mm_z;
    step_accumulator_a += da_mm * stepper->steps_per_deg_a;
    
    // Extract integer steps and keep remainder
    segment_buffer->delta_x = (int32_t)step_accumulator_x;
    segment_buffer->delta_y = (int32_t)step_accumulator_y;
    segment_buffer->delta_z = (int32_t)step_accumulator_z;
    segment_buffer->delta_a = (int32_t)step_accumulator_a;
    
    step_accumulator_x -= (float)segment_buffer->delta_x;
    step_accumulator_y -= (float)segment_buffer->delta_y;
    step_accumulator_z -= (float)segment_buffer->delta_z;
    step_accumulator_a -= (float)segment_buffer->delta_a;
    
    DEBUG_PRINT_MOTION("[KINEMATICS] dx=%.4f dy=%.4f → steps: X=%ld Y=%ld Z=%ld A=%ld (acc: %.3f,%.3f)\r\n",
                      dx_mm, dy_mm, segment_buffer->delta_x, segment_buffer->delta_y,
                      segment_buffer->delta_z, segment_buffer->delta_a,
                      step_accumulator_x, step_accumulator_y);
    
    // Determine dominant axis (highest ABSOLUTE step count) - for Bresenham and timing
    int32_t max_delta = abs(segment_buffer->delta_x);
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
    
    // Store dominant delta (used by Bresenham in ISR)
    segment_buffer->dominant_delta = max_delta;
    
    // Convert feedrate from mm/min to mm/sec
    float feedrate_mm_sec = feedrate / 60.0f;
    // Guard: if no feed specified (0 or negative), fall back to a safe default
    // This prevents divide-by-zero when computing nominal_rate and ensures motion proceeds after reset.
    if (feedrate_mm_sec <= 0.0f) {
        // Use a conservative default of 600 mm/min (10 mm/sec)
        feedrate_mm_sec = 600.0f / 60.0f;
    }
    
    // Get max_rate and acceleration for limiting axis using axis config
    const AxisConfig* axis_cfg = UTILS_GetAxisConfig(limiting_axis);
    if (!axis_cfg) {
        // Fallback to X axis if invalid
        axis_cfg = UTILS_GetAxisConfig(AXIS_X);
    }
    
    float max_rate_mm_min = *(axis_cfg->max_rate);
    float acceleration_mm_sec2 = *(axis_cfg->acceleration);
    
    // Clamp feedrate to max rate
    float max_rate_mm_sec = max_rate_mm_min / 60.0f;
    if(feedrate_mm_sec > max_rate_mm_sec) {
        feedrate_mm_sec = max_rate_mm_sec;
    }
    
    // Timer frequency (TMR4 in 16-bit mode)
    const float TIMER_FREQ = (float)TMR4_FrequencyGet();
    
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
    if (steps_per_sec < 1.0f) {
        // Ensure at least 1 step/sec to avoid INF/NaN conversions
        steps_per_sec = 1.0f;
    }
    segment_buffer->nominal_rate = (uint32_t)(TIMER_FREQ / steps_per_sec);
    
    // ✅ DEBUG: Print timing calculations
    DEBUG_PRINT_MOTION("[KINEMATICS] F=%.1f mm/min, steps/mm=%.1f, steps/sec=%.1f, nominal_rate=%lu ticks (%.2fms)\r\n",
        feedrate, steps_per_mm_dominant, steps_per_sec, 
        segment_buffer->nominal_rate, (float)segment_buffer->nominal_rate / TIMER_FREQ * 1000.0f);
    
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

    // Jerk smoothing: if trapezoidal profiling is effectively disabled (we currently
    // run constant velocity in motion.c), clamp the initial rate to nominal so the
    // first step does not arrive much earlier than subsequent steps.
    if (segment_buffer->initial_rate < segment_buffer->nominal_rate) {
        // initial_rate is a shorter interval (faster). For uniform timing we use nominal.
        segment_buffer->initial_rate = segment_buffer->nominal_rate;
        segment_buffer->final_rate   = segment_buffer->nominal_rate;
    }
    
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
    // TODO: Implement arc interpolation
    // Will break arcs into linear segments for Bresenham execution
    return NULL;  // Placeholder
}

// Reset step accumulators - call when starting new arc or after position reset (G92)
void KINEMATICS_ResetAccumulators(void) {
    step_accumulator_x = 0.0f;
    step_accumulator_y = 0.0f;
    step_accumulator_z = 0.0f;
    step_accumulator_a = 0.0f;
}
// Get current position from stepper counts
CoordinatePoint KINEMATICS_GetCurrentPosition(void) {
    CoordinatePoint current;
    
    // Use axis config abstraction for clean access
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        const AxisConfig* cfg = UTILS_GetAxisConfig(axis);
        if (!cfg) continue;
        
        float position = (float)AXIS_GetSteps(axis) / (*cfg->steps_per_mm);
        
        switch(axis) {
            case AXIS_X: current.x = position; break;
            case AXIS_Y: current.y = position; break;
            case AXIS_Z: current.z = position; break;
            case AXIS_A: current.a = position; break;
            default: break;
        }
    }
    
    return current;
}

// Set machine position for a single axis (used during homing)
void KINEMATICS_SetAxisMachinePosition(E_AXIS axis, float position) {
    const AxisConfig* cfg = UTILS_GetAxisConfig(axis);
    if (!cfg) return;  // Invalid axis
    
    // Convert position to steps and update using abstracted inline helper
    int32_t steps = (int32_t)(position * (*cfg->steps_per_mm));
    AXIS_SetSteps(axis, steps);
}
